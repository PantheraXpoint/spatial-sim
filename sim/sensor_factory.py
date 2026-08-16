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
from isaacsim.core.experimental.utils.app import enable_extension
from pxr import Gf, Usd, UsdGeom

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from core.observation import Modality  # noqa: E402
from core.registry import SensorRegistry, SensorSpec  # noqa: E402

STAGE = os.environ.get("SF_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
MODE = os.environ.get("SF_MODE", "camera")
FRAMES = int(os.environ.get("SF_FRAMES", "120"))
OUT_DIR = Path(os.environ.get("SF_OUT", "/isaac-sim/.nvidia-omniverse/logs"))

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


def create_camera(stage: Usd.Stage, path: str, *, resolution, look_at: Gf.Vec3d | None):
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
    return cam, rep.create.render_product(path, resolution=tuple(resolution))


def create_registry_sensors(
    stage: Usd.Stage, registry: SensorRegistry, *, modalities=None, attach_annotators: bool = True
) -> dict[str, dict]:
    """Instantiate every registry sensor whose parent Xform exists.

    Returns one record per created sensor. Anything unresolvable is logged and
    skipped -- never guessed into existence.
    """
    target = avatar_target(stage)
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
                # replicator writer that 5.x examples reach for.
                sensor.attach_writer("draw-point-cloud")
            except Exception as exc:
                draw = f"failed: {exc!r}"
            log(f"{spec.sensor_id} -> {spec.prim_path} (lidar {spec.config}, draw {draw})")
            made[spec.sensor_id] = {
                "prim_path": spec.prim_path, "kind": "lidar", "sensor": sensor,
                "draw_writer": draw, "annotators": {},
            }
            continue

        _, rp = create_camera(
            stage, spec.prim_path, resolution=spec.resolution or (1280, 720), look_at=target
        )
        anns = {}
        if attach_annotators:
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
