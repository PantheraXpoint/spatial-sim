"""Create sensors from the registry. Layer 1 (scene/USD) + Layer 2 consumer.

Reads ``config/sensors.yaml`` through ``core.registry`` and instantiates what it
finds. Scripts never hardcode a sensor (hard rule 5), and this module never
invents a prim path (hard rule 1): a spec whose parent Xform is not on the stage
is *skipped and reported*, never guessed into existence.

Execution model -- EXEC MODE, and it is not optional
-----------------------------------------------------
Anything that reads sensor data runs under the launcher that actually renders::

    ./runheadless.sh --exec /workspace/sim/sensor_factory.py

Measured on this host: every annotator stays empty under ``SimulationApp`` and
fills under ``runheadless.sh``. So there is no ``SimulationApp`` here, no
import-ordering constraint, and no ``app.update()`` loop -- calling update()
from inside an ``--exec`` script re-enters the main loop. Frames are driven from
the update event stream, config comes from environment variables (Kit's
``--exec SCRIPT ARGS...`` makes trailing-argument parsing ambiguous), and
results are written incrementally and fsync'd because this renderer dies
mid-run and a write-at-exit design loses everything.

Environment
-----------
    SF_STAGE     stage to open   (default: /workspace/sim/observatory_avatar.usd)
    SF_MODE      station | camera | lidar   (default: station -- everything)
    SF_FRAMES    frames to sample (default: 120)
    SF_OUT       results directory (default: /isaac-sim/.nvidia-omniverse/logs)

Where the sensors go
--------------------
Stations come from ``config/scene.yaml`` and sensors hang off them at the paths
``config/sensors.yaml`` declares. :func:`create_stations` authors the station
Xform at its declared position -- that is not an invented path, it is the
contract being made real, and it runs first because nothing under a station
resolves until it exists. Sensors whose parent is still absent (BOT_*, pending
S9) are logged and skipped.

The station pose is deliberately NOT authored into the stage: sim/avatar.py
rebuilds observatory_avatar.usd from the base, so anything written there by
hand is lost on the next rebuild. Config is the durable place for it.

INFRA_01 is a WALL mount at 2.60 m, not a ceiling mount, and that is a measured
constraint. Example_Rotary sweeps elevations -15..+10 deg only; from the ceiling
directly above the avatar the lidar returned 418,235 points and none on the
body, silently. :meth:`Run.setup` recomputes and logs that geometry every run
rather than trusting the comment.
"""

from __future__ import annotations

import json
import math
import os
import sys
import traceback
from pathlib import Path

import carb
import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
import yaml
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
from isaacsim.core.experimental.utils.app import enable_extension
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from core.observation import Modality  # noqa: E402
from core.registry import SensorRegistry, SensorSpec  # noqa: E402

STAGE = os.environ.get("SF_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
MODE = os.environ.get("SF_MODE", "camera")
FRAMES = int(os.environ.get("SF_FRAMES", "120"))
OUT_DIR = Path(os.environ.get("SF_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
# Scales every render product. The registry's resolution is the contract for
# what a sensor DELIVERS; it is not a statement about what a smoke test needs.
# Halving it quarters the pixels, and a framing check or a point count reads
# the same at 640x360.
RES_SCALE = float(os.environ.get("SF_RES_SCALE", "1.0"))
# Lidar debug-draw appearance. The defaults are the writer's own, and they are
# ~1 cm dark points, which is why 419,000 of them were invisible against a grey
# floor and orange racks. attach_writer forwards these to writer.initialize().
LIDAR_PT_SIZE = float(os.environ.get("SF_LIDAR_PT_SIZE", "0.06"))
LIDAR_PT_COLOR = [float(v) for v in
                  os.environ.get("SF_LIDAR_PT_COLOR", "0.1,1.0,0.2,1.0").split(",")]

ROOT = "/Root"
PROVISIONAL = f"{ROOT}/_Provisional"
AVATAR = f"{ROOT}/Avatar"

# The no-detection placeholder recorded in CLAUDE.md for the RTX radar:
# azimuth 0, elevation 0, range exactly 100 m, and it carries the VALID bit.
# Whether the lidar emits it too is one of the questions this script answers.
SENTINEL = (0.0, 0.0, 100.0)
VALID = 64  # ElementFlags.VALID

# Example_Rotary's vertical field of view, read from the shipped profile
# Example_Rotary_BEAMS.json in omni.sensors.nv.common: 128 emitters spanning
# elevations -15.0 to +10.0 deg, nearRangeM 1.0. This is the whole reason
# INFRA_01 is a wall mount and not a ceiling mount.
LIDAR_EL_MIN_DEG, LIDAR_EL_MAX_DEG = -15.0, 10.0


# ---------------------------------------------------------------------------
# Results: incremental and fsync'd. This renderer dies mid-run.
# ---------------------------------------------------------------------------
class Results:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(path, "w", encoding="utf-8")
        self.path = path

    def write(self, **record) -> None:
        self._fh.write(json.dumps(record, default=str) + "\n")
        self._fh.flush()
        os.fsync(self._fh.fileno())

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


def log(msg: str) -> None:
    print(f"[sensor_factory] {msg}", flush=True)


# ---------------------------------------------------------------------------
# Registry -> stage
# ---------------------------------------------------------------------------
def load_registry() -> SensorRegistry:
    """The registry is the single source of truth (hard rule 5)."""
    return SensorRegistry.from_yaml(str(REPO / "config" / "sensors.yaml"))


def load_stations() -> list[dict]:
    """Station declarations, straight out of config/scene.yaml.

    The station pose lives in config rather than in the USD on purpose: the
    stage this runs against is rebuilt from sim/avatar.py, so anything authored
    into it by hand would be lost on the next rebuild. The factory recreates
    the stations every run from the contract instead.
    """
    with open(REPO / "config" / "scene.yaml", encoding="utf-8") as fh:
        return yaml.safe_load(fh).get("stations") or []


def resolvable(stage: Usd.Stage, spec: SensorSpec) -> bool:
    """Is this spec's parent Xform actually on the stage?

    A wrong prim path does not render badly -- it crashes, or silently does
    nothing. So an unresolvable spec is reported and skipped, never guessed.
    """
    if not spec.parent:
        return False
    return stage.GetPrimAtPath(spec.parent).IsValid()


def audit_registry(stage: Usd.Stage, registry: SensorRegistry) -> dict:
    """What the registry asks for vs. what the stage can currently provide."""
    ready, missing = [], []
    for spec in registry:
        (ready if resolvable(stage, spec) else missing).append(spec)
    return {
        "ready": [s.sensor_id for s in ready],
        "missing": [
            {"sensor_id": s.sensor_id, "modality": s.modality.value,
             "parent": s.parent, "prim_path": s.prim_path}
            for s in missing
        ],
    }


COLLIDER_MASK = REPO / "sim" / "collider_mask.json"


def disable_unreachable_colliders(stage: Usd.Stage, reach_m: float = 2.2) -> dict:
    """Turn off collision on geometry the avatar can never touch.

    THE frame-rate lever, and it is not marginal. Measured at Play on the full
    warehouse, load average 13.9 throughout:

        baseline                       2.48 fps
        1,486 colliders above 2.2 m off  19.69 fps   (+694%)
        ...and then at 960x540           19.85 fps   (+0.8%)

    So the cost was PhysX carrying 3,469 exact triangle-mesh colliders, and
    resolution is irrelevant next to it. Collision only: these prims keep their
    render geometry, and cameras and lidar trace the render BVH rather than
    colliders, so nothing about the picture changes.

    The selection is CACHED to sim/collider_mask.json by prim path. Finding it
    means a world-bbox computation over 3,469 prims, which took about fifteen
    minutes on a loaded box; applying a known list of paths takes under a
    second. The warehouse is static, so the list only goes stale if the base
    stage changes -- delete the file to regenerate.
    """
    import json as _json

    paths: list[str] | None = None
    if COLLIDER_MASK.exists():
        try:
            cached = _json.loads(COLLIDER_MASK.read_text())
            if abs(float(cached.get("reach_m", -1)) - reach_m) < 1e-9:
                paths = cached.get("paths")
        except Exception as exc:
            log(f"  ! unreadable collider mask, recomputing: {exc!r}")

    if paths is None:
        log(f"  computing collider mask (no cache) -- a bbox pass over the warehouse, minutes")
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        paths = []
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if not prim.GetPath().pathString.startswith("/Root/Warehouse"):
                continue
            if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
                continue
            try:
                rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                if rng.IsEmpty() or float(rng.GetMin()[2]) <= reach_m:
                    continue
            except Exception:
                continue
            paths.append(prim.GetPath().pathString)
        try:
            COLLIDER_MASK.write_text(_json.dumps(
                {"reach_m": reach_m, "note": "colliders entirely above reach_m; "
                 "regenerate by deleting this file", "paths": paths}, indent=1))
            log(f"  cached {len(paths)} paths -> {COLLIDER_MASK}")
        except Exception as exc:
            log(f"  ! could not cache the mask: {exc!r}")

    n = 0
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid() and prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
            n += 1
    log(f"  collision disabled on {n} prims entirely above {reach_m} m")
    return {"disabled": n, "reach_m": reach_m, "cached": COLLIDER_MASK.exists()}


def create_stations(stage: Usd.Stage) -> dict[str, list[float]]:
    """Author each declared station Xform at its declared position.

    Not an invented path (hard rule 1): the path is declared in
    config/scene.yaml, which is the contract a human signs off, and this
    function is what makes it exist. Sensors hang off these, so this runs
    first or nothing under them resolves.
    """
    made: dict[str, list[float]] = {}
    for st in load_stations():
        # `stage_position` is the confirmed pose in the Isaac stage, and its
        # PRESENCE is what marks a station as belonging on this stage at all.
        # `position` alone is a Layer 3 declaration for the mock's synthetic
        # world -- INFRA_02 has one and lives 60 m away in a second building
        # that does not exist here. Falling back to it built
        # /World/Infrastructure/INFRA_02 out of thin air, complete with a
        # /World root this stage does not have and two sensors staring at
        # nothing from 56-96 m. Declared is not the same as sited.
        path, pos = st.get("prim_path"), st.get("stage_position")
        if not path or pos is None:
            log(f"station {st.get('id')} declared but not sited on this stage "
                f"(no stage_position) -- skipped")
            continue
        xf = UsdGeom.Xform.Define(stage, path)
        existing = xf.GetPrim().GetAttribute("xformOp:translate")
        (existing or xf.AddTranslateOp()).Set(Gf.Vec3d(*[float(v) for v in pos]))
        made[path] = [float(v) for v in pos]
    return made


def create_station_marker(stage: Usd.Stage, station_path: str, look_at: Gf.Vec3d):
    """A small emissive beacon so a human can SEE where a station is.

    The camera panel was "a view from nowhere I can point at" because the
    station is an Xform -- zero geometry, invisible in the viewport. This puts a
    glowing sphere and a short cone at it: obviously an instrument, obviously
    not warehouse furniture.

    Built from UsdGeom primitives rather than a referenced prop because Isaac
    ships no marker asset -- searched exts/ and extscache/ for
    marker|beacon|arrow|axis|frustum and found only schema and test files.

    Placed directly ABOVE the station, not behind it. Behind was the first
    attempt and it is wrong twice over: the lidar sweeps a full 360 degrees in
    azimuth, so a beacon 0.28 m away carves a blind wedge out of it, and
    anything inside nearRangeM (1.0 m) is discarded as a return while still
    occluding whatever is behind it. Straight up is outside the sensor's
    elevation band (-15..+10 deg) entirely, so it cannot occlude, and it is
    out of the camera's frame because the camera is pitched down at the floor.
    """
    station = stage.GetPrimAtPath(station_path)
    if not station.IsValid():
        return None
    pos = UsdGeom.XformCache().GetLocalToWorldTransform(station).ExtractTranslation()
    d = Gf.Vec3d(look_at[0] - pos[0], look_at[1] - pos[1], look_at[2] - pos[2])
    n = d.GetLength() or 1.0
    back = Gf.Vec3d(-d[0] / n, -d[1] / n, -d[2] / n)

    marker = UsdGeom.Xform.Define(stage, f"{station_path}/marker")
    marker.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.38))

    head = UsdGeom.Sphere.Define(stage, f"{station_path}/marker/head")
    head.CreateRadiusAttr(0.09)
    head.CreateExtentAttr([(-0.09, -0.09, -0.09), (0.09, 0.09, 0.09)])

    stalk = UsdGeom.Cone.Define(stage, f"{station_path}/marker/aim")
    stalk.CreateRadiusAttr(0.05)
    stalk.CreateHeightAttr(0.22)
    stalk.CreateAxisAttr(UsdGeom.Tokens.z)
    stalk.CreateExtentAttr([(-0.05, -0.05, -0.11), (0.05, 0.05, 0.11)])
    # Point the cone along the view direction: rotate +Z onto it.
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -back[2]))))
    yaw = math.degrees(math.atan2(-back[1], -back[0]))
    stalk.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    stalk.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 90.0 - pitch, yaw))

    looks = UsdGeom.Scope.Define(stage, f"{station_path}/marker/Looks")  # noqa: F841
    mat = PreviewSurfaceMaterial(f"{station_path}/marker/Looks/beacon")
    mat.set_input_values("diffuseColor", [0.0, 0.9, 1.0])
    mat.set_input_values("emissiveColor", [0.0, 1.2, 1.6])
    mat.set_input_values("roughness", [0.4])
    material = UsdShade.Material(stage.GetPrimAtPath(f"{station_path}/marker/Looks/beacon"))
    for path in (f"{station_path}/marker/head", f"{station_path}/marker/aim"):
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(material)
    log(f"station marker at {station_path}/marker (emissive, above the station)")
    return marker


def reference_robots(stage: Usd.Stage) -> dict[str, str]:
    """Reference each robot that has BOTH an asset and a confirmed stage pose.

    Does NOT pin them -- see pin_robots_static(). Splitting the two is not
    tidiness: the Go2's physics lives behind payloads, and pinning in the same
    breath as referencing found 0 rigid bodies on it and let PhysX settle the
    animal 5.9 cm at Play. Counted: 0 bodies at reference, 17 once loaded.
    """
    from isaacsim.storage.native import get_assets_root_path

    root = get_assets_root_path()
    made: dict[str, str] = {}
    for r in load_stations.__globals__["yaml"].safe_load(
            open(REPO / "config" / "scene.yaml", encoding="utf-8")).get("robots") or []:
        asset, pos, path = r.get("asset"), r.get("stage_position"), r.get("prim_path")
        if not (asset and pos and path):
            log(f"robot {r.get('id')} declared but not sited (asset/stage_position) -- skipped")
            continue
        xf = UsdGeom.Xform.Define(stage, path)
        existing = xf.GetPrim().GetAttribute("xformOp:translate")
        (existing or xf.AddTranslateOp()).Set(Gf.Vec3d(*[float(v) for v in pos]))
        xf.GetPrim().GetReferences().AddReference(f"{root}/Isaac/{asset}")
        stage.Load(xf.GetPrim().GetPath())
        made[r["id"]] = path
        log(f"robot {r['id']} -> {path}")
    return made


def pin_robots_static(stage: Usd.Stage, paths: dict[str, str]) -> dict[str, list[int]]:
    """Make every robot rigid body kinematic and every articulation root off.

    Call AFTER payloads have settled, or there may be nothing there to pin.
    Legged robots collapse on spawn without a locomotion policy; this is what
    keeps them standing, and it is measured rather than assumed -- see
    sim/spikes/_place_robots.py, which compares bbox height across Play.
    """
    out: dict[str, list[int]] = {}
    for rid, path in paths.items():
        bodies = arts = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
                bodies += 1
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                arts += 1
                a = prim.GetAttribute("physxArticulation:articulationEnabled")
                if not a:
                    a = prim.CreateAttribute(
                        "physxArticulation:articulationEnabled", Sdf.ValueTypeNames.Bool)
                a.Set(False)
        out[rid] = [bodies, arts]
        log(f"robot {rid} pinned static: {bodies} bodies, {arts} articulation roots")
    return out


def look_at_rotate_xyz(eye: Gf.Vec3d, target: Gf.Vec3d) -> Gf.Vec3f:
    """rotateXYZ that aims a USD camera from `eye` at `target`, +Z up.

    A USD camera looks down its own -Z with +Y up. Composing Rx then Rz gives a
    forward of (-sin(rz), cos(rz)) horizontally and a downward pitch of
    (90 - rx), so rx = 90 - depression and rz = azimuth - 90.
    """
    d = Gf.Vec3d(target[0] - eye[0], target[1] - eye[1], target[2] - eye[2])
    horiz = math.hypot(d[0], d[1])
    azimuth = math.degrees(math.atan2(d[1], d[0]))
    depression = math.degrees(math.atan2(-d[2], horiz)) if horiz > 1e-9 else 90.0
    return Gf.Vec3f(90.0 - depression, 0.0, azimuth - 90.0)


def avatar_target(stage: Usd.Stage) -> Gf.Vec3d:
    """Where the sensors should point: the middle of the avatar's body."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    char = stage.GetPrimAtPath(f"{AVATAR}/character")
    if char.IsValid():
        r = cache.ComputeWorldBound(char).ComputeAlignedRange()
        mid = r.GetMidpoint()
        return Gf.Vec3d(float(mid[0]), float(mid[1]), float(mid[2]))
    return Gf.Vec3d(0.0, 0.0, 0.9)


def create_camera(
    stage: Usd.Stage, path: str, *, resolution, look_at: Gf.Vec3d | None,
    render_product: bool = True,
):
    """A camera at its registry path, aimed at `look_at`, plus its render product.

    No translate op is authored: the camera hangs off its station Xform, which
    already carries the world pose. That is what "three modalities share one
    pose" means in practice -- one transform, not three that can drift apart.
    """
    cam = UsdGeom.Camera.Define(stage, path)
    prim = cam.GetPrim()
    if look_at is not None:
        eye = UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractTranslation()
        rot = look_at_rotate_xyz(Gf.Vec3d(float(eye[0]), float(eye[1]), float(eye[2])), look_at)
        (prim.GetAttribute("xformOp:rotateXYZ") or cam.AddRotateXYZOp()).Set(rot)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1_000_000.0))
    cam.CreateFocalLengthAttr(18.0)
    if not render_product:
        # In a GUI session the VIEWPORT is the render product. Creating one
        # here as well renders every camera twice -- once into a 1280x720
        # Replicator target nothing reads, once into the panel you are looking
        # at. Three cameras plus the lidar at 3-4 FPS was mostly this.
        return cam, None
    w, h = (max(16, int(v * RES_SCALE)) for v in resolution)
    if RES_SCALE != 1.0:
        log(f"  render product {w}x{h} (SF_RES_SCALE={RES_SCALE})")
    return cam, rep.create.render_product(path, resolution=(w, h))


def create_registry_sensors(
    stage: Usd.Stage, registry: SensorRegistry, *, modalities=None,
    attach_annotators: bool = True, render_products: bool = True,
    markers: bool = True,
) -> dict[str, dict]:
    """Instantiate every registry sensor whose parent Xform exists.

    Returns one record per created sensor. Anything unresolvable is logged and
    skipped -- never guessed into existence.
    """
    target = avatar_target(stage)
    if markers:
        for st in load_stations():
            if st.get("prim_path") and st.get("stage_position"):
                create_station_marker(stage, st["prim_path"], target)
    made: dict[str, dict] = {}
    for spec in registry:
        if modalities and spec.modality not in modalities:
            continue
        if not resolvable(stage, spec):
            log(f"skip {spec.sensor_id}: parent {spec.parent} is not on the stage")
            continue
        if spec.modality is Modality.RADAR:
            log(f"skip {spec.sensor_id}: radar needs the three Motion BVH kit flags")
            continue

        if spec.modality is Modality.LIDAR:
            enable_extension("isaacsim.sensors.rtx.nodes")
            from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

            lidar = Lidar.create(
                spec.prim_path, config=spec.config, translations=np.array([[0.0, 0.0, 0.0]])
            )
            sensor = LidarSensor(lidar, annotators=["generic-model-output"])
            draw = "attached"
            try:
                # 6.x debug draw. NOT the RtxLidarDebugDrawPointCloudBuffer
                # replicator writer that 5.x examples reach for. size/color are
                # forwarded to writer.initialize(): the defaults draw small dark
                # points, which is how 419,000 returns managed to be invisible
                # against a grey floor.
                sensor.attach_writer(
                    "draw-point-cloud", size=LIDAR_PT_SIZE, color=LIDAR_PT_COLOR
                )
            except TypeError:
                # Older writer signature: fall back rather than lose the draw.
                sensor.attach_writer("draw-point-cloud")
                draw = "attached (no size/color support)"
            except Exception as exc:
                draw = f"failed: {exc!r}"
            log(f"{spec.sensor_id} -> {spec.prim_path} (lidar {spec.config}, draw {draw})")
            made[spec.sensor_id] = {
                "prim_path": spec.prim_path, "kind": "lidar", "sensor": sensor,
                "draw_writer": draw, "annotators": {},
            }
            continue

        _, rp = create_camera(
            stage, spec.prim_path, resolution=spec.resolution or (1280, 720),
            look_at=target, render_product=render_products,
        )
        anns = {}
        if attach_annotators and rp is not None:
            for name in spec.annotators:
                params = {"colorize": False} if name == "semantic_segmentation" else None
                ann = (
                    rep.AnnotatorRegistry.get_annotator(name, init_params=params)
                    if params else rep.AnnotatorRegistry.get_annotator(name)
                )
                ann.attach([rp])
                anns[name] = ann
        log(f"{spec.sensor_id} -> {spec.prim_path} (camera, annotators {list(anns)})")
        made[spec.sensor_id] = {
            "prim_path": spec.prim_path, "kind": "camera", "sensor": None,
            "render_product": rp, "annotators": anns,
        }
    return made


# ---------------------------------------------------------------------------
# Lidar decode
# ---------------------------------------------------------------------------
def decode_gmo(gmo, sensor_to_world: Gf.Matrix4d | None) -> dict:
    """Turn a generic-model-output buffer into metric world points.

    THE DEFAULTS ARE THE TRAP. Per-element x/y/z are azimuth degrees,
    elevation degrees and range metres -- because elementsCoordsType defaults
    to SPHERICAL -- and they are sensor-local, because frameOfReference
    defaults to SENSOR. Read as Cartesian metres they look entirely plausible
    and are silently wrong. So: convert, then transform, then mask on VALID --
    and then drop the sentinel, because VALID does not mean real.
    """
    n = int(gmo.numElements)
    out: dict = {"numElements": n}
    if n == 0:
        return out

    az = np.asarray(gmo.x[:n], dtype=np.float64)
    el = np.asarray(gmo.y[:n], dtype=np.float64)
    rng = np.asarray(gmo.z[:n], dtype=np.float64)
    flags = np.asarray(gmo.flags[:n]).astype(np.int64) if hasattr(gmo, "flags") else np.full(n, VALID)

    valid = (flags & VALID) != 0
    out["valid"] = int(valid.sum())

    # The exact no-detection triple. Identified by its exact value rather than
    # by any flag or range bound, because it passes both.
    sentinel = (np.abs(az) < 1e-6) & (np.abs(el) < 1e-6) & (np.abs(rng - 100.0) < 1e-6)
    out["sentinel_hits"] = int(sentinel.sum())
    out["sentinel_and_valid"] = int((sentinel & valid).sum())

    keep = valid & ~sentinel
    out["real"] = int(keep.sum())
    if keep.sum() == 0:
        return out

    a = np.radians(az[keep])
    e = np.radians(el[keep])
    r = rng[keep]
    # Proper spherical -> Cartesian. NVIDIA's own radar test drops the cos(el)
    # factor on x/y; with a rotary lidar's elevation spread that is a real
    # error, so it is kept here.
    local = np.stack([r * np.cos(e) * np.cos(a), r * np.cos(e) * np.sin(a), r * np.sin(e)], axis=1)

    if sensor_to_world is not None:
        m = np.array(sensor_to_world, dtype=np.float64).reshape(4, 4)
        world = local @ m[:3, :3] + m[3, :3]
    else:
        world = local

    out["range_min"] = float(r.min())
    out["range_max"] = float(r.max())
    out["world_z_min"] = float(world[:, 2].min())
    out["world_z_max"] = float(world[:, 2].max())
    out["_points"] = world
    return out


def count_in_box(points: np.ndarray, lo: Gf.Vec3d, hi: Gf.Vec3d, pad: float = 0.15) -> int:
    lo = np.array([lo[0] - pad, lo[1] - pad, lo[2] - pad])
    hi = np.array([hi[0] + pad, hi[1] + pad, hi[2] + pad])
    inside = np.all((points >= lo) & (points <= hi), axis=1)
    return int(inside.sum())


# ---------------------------------------------------------------------------
# Exec-mode driver
# ---------------------------------------------------------------------------
class Run:
    """A small state machine driven off the update event stream.

    Staged rather than sequential because the stage's references load
    asynchronously and sensors created against a half-loaded stage see a
    half-loaded world.
    """

    def __init__(self, results: Results) -> None:
        self.results = results
        self.phase = "loading"
        self.frame = 0
        self.sampled = 0
        self.ctx = omni.usd.get_context()
        self.state: dict = {}
        self.sub = None

    # -- setup ------------------------------------------------------------
    def setup(self) -> None:
        stage = self.ctx.get_stage()
        registry = load_registry()

        stations = create_stations(stage)
        for path, pos in stations.items():
            log(f"station {path} at {[round(v, 3) for v in pos]}")
        self.results.write(event="stations", stations=stations)

        audit = audit_registry(stage, registry)
        log(f"registry: {len(registry)} sensors, {len(audit['ready'])} resolvable")
        self.results.write(event="registry_audit", **audit)

        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        char = stage.GetPrimAtPath(f"{AVATAR}/character")
        if char.IsValid():
            r = cache.ComputeWorldBound(char).ComputeAlignedRange()
            self.state["avatar_lo"], self.state["avatar_hi"] = r.GetMin(), r.GetMax()
            self.results.write(
                event="avatar_bbox",
                min=[float(v) for v in r.GetMin()], max=[float(v) for v in r.GetMax()],
            )
        target = avatar_target(stage)
        self.state["target"] = target

        # THE geometry check this pose has to pass, computed rather than hoped:
        # a rotary lidar only sees what falls inside its elevation band.
        for path, pos in stations.items():
            for label, z in (("head", 1.86), ("centre", float(target[2])), ("feet", 0.0)):
                dx, dy = target[0] - pos[0], target[1] - pos[1]
                d = math.hypot(dx, dy)
                dep = math.degrees(math.atan2(pos[2] - z, d)) if d > 1e-9 else 90.0
                # Elevation as the sensor sees it: a point below the sensor has
                # negative elevation, so elevation = -depression.
                elev = -dep
                inband = LIDAR_EL_MIN_DEG <= elev <= LIDAR_EL_MAX_DEG
                log(f"  {path} -> avatar {label}: d={d:.2f} m elevation={elev:+.2f} deg "
                    f"{'IN BAND' if inband else 'OUT OF BAND'}")
                self.results.write(
                    event="band_check", station=path, point=label,
                    distance_m=d, elevation_deg=elev, in_band=bool(inband),
                    band_deg=[LIDAR_EL_MIN_DEG, LIDAR_EL_MAX_DEG],
                )

        mods = None
        if MODE == "camera":
            mods = {Modality.RGB, Modality.RGBD, Modality.DEPTH, Modality.SEMANTIC}
        elif MODE == "lidar":
            mods = {Modality.LIDAR}
        # MODE == "station" -> everything the registry offers

        made = create_registry_sensors(stage, registry, modalities=mods)
        self.state["made"] = made
        self.results.write(
            event="sensors_created",
            sensors={k: {"prim_path": v["prim_path"], "kind": v["kind"]} for k, v in made.items()},
        )
        if not made:
            raise RuntimeError("no sensors were created -- nothing to sample")

        omni.timeline.get_timeline_interface().play()

    # -- sampling ---------------------------------------------------------
    def _sample_camera(self, sensor_id: str, rec: dict) -> None:
        st = self.state
        anns = rec["annotators"]
        best = st.setdefault("cam", {})
        slot = best.setdefault(sensor_id, {"rgb": 0, "person": 0})

        rgb_ann = anns.get("rgb")
        if rgb_ann is not None:
            arr = np.asarray(rgb_ann.get_data())
            nonzero = int((arr != 0).sum()) if arr.size else 0
            if nonzero > slot["rgb"]:
                slot["rgb"] = nonzero
                slot["shape"] = list(arr.shape)
                if arr.size and arr.ndim == 3:
                    st.setdefault("frames", {})[sensor_id] = arr.copy()

        seg_ann = anns.get("semantic_segmentation")
        if seg_ann is not None:
            seg = seg_ann.get_data()
            if isinstance(seg, dict) and seg.get("data") is not None:
                data = np.asarray(seg["data"])
                labels = (seg.get("info") or {}).get("idToLabels")
                if labels:
                    ids = [int(k) for k, v in labels.items() if "person" in json.dumps(v).lower()]
                    if ids:
                        slot["person"] = max(slot["person"], int(np.isin(data, ids).sum()))
                    slot["labels"] = labels

    def _sample_lidar(self, sensor_id: str, rec: dict) -> None:
        from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

        st = self.state
        stage = self.ctx.get_stage()
        try:
            buf, _ = rec["sensor"].get_data("generic-model-output")
        except Exception:
            return
        if buf is None:
            return
        gmo = parse_generic_model_output_data(buf)
        if gmo is None:
            return
        if not st.get("gmo_introspected"):
            st["gmo_introspected"] = True
            header = {}
            for f in ("elementsCoordsType", "frameOfReference", "maxRangeM"):
                try:
                    header[f] = float(getattr(gmo, f))
                except Exception:
                    header[f] = "<unavailable>"
            self.results.write(event="gmo_header", which=sensor_id, header=header)
            log(f"gmo header: {header}")

        m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(rec["prim_path"]))
        dec = decode_gmo(gmo, m)
        pts = dec.pop("_points", None)
        best = st.setdefault("lidar", {}).setdefault(sensor_id, {"real": 0, "avatar_hits": 0})
        if dec.get("real", 0) >= best.get("real", 0):
            best.update(dec)
        best["sentinel_total"] = best.get("sentinel_total", 0) + dec.get("sentinel_hits", 0)
        if pts is not None and "avatar_lo" in st:
            best["avatar_hits"] = max(
                best.get("avatar_hits", 0), count_in_box(pts, st["avatar_lo"], st["avatar_hi"])
            )

    def sample(self) -> None:
        for sensor_id, rec in self.state.get("made", {}).items():
            if rec["kind"] == "camera":
                self._sample_camera(sensor_id, rec)
            else:
                self._sample_lidar(sensor_id, rec)

    # -- finish -----------------------------------------------------------
    def finish(self) -> None:
        st = self.state
        summary = {
            "mode": MODE,
            "frames": self.sampled,
            "cameras": st.get("cam", {}),
            "lidar": st.get("lidar", {}),
            "avatar_bbox": [
                [float(v) for v in st["avatar_lo"]], [float(v) for v in st["avatar_hi"]]
            ] if "avatar_lo" in st else None,
        }
        # The framing PNG: the whole point is that a human can judge the shot
        # from an image viewer instead of opening Isaac Sim.
        for sensor_id, arr in st.get("frames", {}).items():
            path = OUT_DIR / f"framing_{sensor_id}.png"
            try:
                from PIL import Image

                Image.fromarray(np.asarray(arr)[:, :, :3].astype(np.uint8)).save(path)
                log(f"framing PNG -> {path}")
                summary.setdefault("png", []).append(str(path))
            except Exception as exc:
                log(f"! could not write {path}: {exc!r}")
        self.results.write(event="summary", **summary)
        log("SUMMARY " + json.dumps(summary, default=str)[:3000])
        log("DONE")

    # -- the update pump --------------------------------------------------
    def on_update(self, _e) -> None:
        self.frame += 1
        try:
            if self.phase == "loading":
                status = self.ctx.get_stage_loading_status()
                if self.frame > 5 and not any(status[1:]):
                    log(f"stage loaded after {self.frame} frames")
                    self.setup()
                    self.phase = "sampling"
                return
            if self.phase == "sampling":
                self.sampled += 1
                self.sample()
                if self.sampled >= FRAMES:
                    self.finish()
                    self.phase = "done"
                    self.sub = None
                    omni.kit.app.get_app().post_quit()
        except Exception as exc:
            log("FAILED: " + repr(exc))
            self.results.write(event="error", error=repr(exc), tb=traceback.format_exc())
            self.sub = None
            omni.kit.app.get_app().post_quit()


def main() -> None:
    out = OUT_DIR / f"sensor_factory_{MODE}.jsonl"
    results = Results(out)
    log(f"stage={STAGE} mode={MODE} frames={FRAMES}")
    log(f"results -> {out}")
    results.write(event="start", stage=STAGE, mode=MODE, frames=FRAMES)

    # Returns a bool in some Kit builds and (bool, error) in others.
    opened = omni.usd.get_context().open_stage(STAGE)
    ok, err = opened if isinstance(opened, tuple) else (opened, None)
    log(f"open_stage ok={ok} err={err}")
    results.write(event="open_stage", ok=bool(ok), err=str(err))

    run = Run(results)
    run.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        run.on_update, name="sensor_factory"
    )
    log("subscribed to the update stream")


def _is_exec_entrypoint() -> bool:
    """True when Kit --exec'd THIS file; false when another module imports it.

    Deliberately not ``__name__ == "__main__"``. Kit's ``--exec`` does not
    reliably set that, and both ways of getting it wrong are silent and bad:
    too strict and the capture runs zero frames while looking fine, too loose
    and merely *importing* this module opens a stage, samples 120 frames and
    then post_quit()s -- which, from sim/gui_viewports.py, would close the GUI
    out from under whoever is connected. So the importer says so explicitly
    and the exec path keeps the behaviour it already has, unchanged.
    """
    return os.environ.get("SF_NO_AUTORUN") != "1"


if _is_exec_entrypoint():
    main()
