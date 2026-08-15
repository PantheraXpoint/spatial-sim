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
    SF_MODE      camera | lidar  (default: camera)
    SF_FRAMES    frames to sample (default: 120)
    SF_OUT       results directory (default: /isaac-sim/.nvidia-omniverse/logs)

Provisional placement
---------------------
``config/sensors.yaml`` still carries ``/World/...`` placeholders for INFRA_*
and BOT_*, and those Xforms do not exist on the stage. Until a human confirms
real paths in the GUI, this module can place ONE camera at a derived pose --
see :func:`derive_provisional_pose`, which reads the ceiling height off the
warehouse's bounding box and the aisle position off the Worker's own transform.
Derived from the stage, not invented, and named so nobody mistakes it for a
confirmed station: it lands under ``/Root/_Provisional/``.
"""

from __future__ import annotations

import json
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


def derive_provisional_pose(stage: Usd.Stage, drop: float = 0.6) -> tuple[Gf.Vec3d, dict]:
    """A ceiling-height pose above an aisle, DERIVED, not chosen by taste.

    Ceiling height comes from the warehouse's own world bounding box. The
    horizontal position comes from where the Worker character stands -- a spot
    a human-sized body demonstrably fits, which is the definition of an aisle
    here, and the same reasoning sim/avatar.py uses for the spawn point. The
    avatar spawns there too, so this camera is guaranteed to have the avatar
    under it on frame one.

    Returns the pose and the evidence behind it, so the report can show the
    derivation rather than assert the number.
    """
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    warehouse = stage.GetPrimAtPath(f"{ROOT}/Warehouse")
    rng = cache.ComputeWorldBound(warehouse).ComputeAlignedRange()
    ceiling_z = float(rng.GetMax()[2])

    worker = stage.GetPrimAtPath(f"{ROOT}/Worker")
    t = UsdGeom.XformCache().GetLocalToWorldTransform(worker).ExtractTranslation()

    pose = Gf.Vec3d(float(t[0]), float(t[1]), ceiling_z - drop)
    evidence = {
        "warehouse_bbox_min": [float(v) for v in rng.GetMin()],
        "warehouse_bbox_max": [float(v) for v in rng.GetMax()],
        "ceiling_z": ceiling_z,
        "drop_below_ceiling_m": drop,
        "aisle_xy_from": f"{ROOT}/Worker",
        "aisle_xy": [float(t[0]), float(t[1])],
        "pose": [float(v) for v in pose],
    }
    return pose, evidence


def create_camera(stage: Usd.Stage, path: str, pose: Gf.Vec3d, resolution: tuple[int, int]):
    """A downward-looking camera at `pose`, plus its render product.

    No rotation is authored on purpose: a USD camera looks along its own -Z,
    which in this Z-up stage is already straight down at the floor.
    """
    cam = UsdGeom.Camera.Define(stage, path)
    if not cam.GetPrim().GetAttribute("xformOp:translate"):
        cam.AddTranslateOp().Set(pose)
    else:
        cam.GetPrim().GetAttribute("xformOp:translate").Set(pose)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1_000_000.0))
    cam.CreateFocalLengthAttr(14.0)  # wide, so a ceiling camera sees an aisle
    return cam, rep.create.render_product(path, resolution=tuple(resolution))


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
        audit = audit_registry(stage, registry)
        log(f"registry: {len(registry)} sensors, {len(audit['ready'])} resolvable")
        self.results.write(event="registry_audit", **audit)

        pose, evidence = derive_provisional_pose(stage)
        log(f"provisional pose {evidence['pose']} (ceiling {evidence['ceiling_z']:.2f})")
        self.results.write(event="provisional_pose", **evidence)

        # The avatar's world bounds, for "did anything land on the body".
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        char = stage.GetPrimAtPath(f"{AVATAR}/character")
        if char.IsValid():
            r = cache.ComputeWorldBound(char).ComputeAlignedRange()
            self.state["avatar_lo"], self.state["avatar_hi"] = r.GetMin(), r.GetMax()
            self.results.write(
                event="avatar_bbox",
                min=[float(v) for v in r.GetMin()],
                max=[float(v) for v in r.GetMax()],
            )
        else:
            log("! no avatar on this stage -- nothing to hit")

        UsdGeom.Xform.Define(stage, PROVISIONAL)
        if MODE == "camera":
            self._setup_camera(stage, registry, pose)
        elif MODE == "lidar":
            self._setup_lidar(stage, registry, pose)
        else:
            raise ValueError(f"SF_MODE={MODE!r} is not camera or lidar")

        omni.timeline.get_timeline_interface().play()

    def _setup_camera(self, stage, registry, pose) -> None:
        spec = next(iter(registry.by_modality(Modality.RGBD)), None)
        if spec is None:
            raise RuntimeError("no rgbd sensor in the registry")
        path = f"{PROVISIONAL}/{spec.sensor_id}"
        log(f"PROVISIONAL camera for {spec.sensor_id} at {path} (registry path {spec.prim_path} is unresolvable)")
        _, rp = create_camera(stage, path, pose, spec.resolution or (1280, 720))

        self.state["rgb"] = rep.AnnotatorRegistry.get_annotator("rgb")
        self.state["rgb"].attach([rp])
        seg = rep.AnnotatorRegistry.get_annotator(
            "semantic_segmentation", init_params={"colorize": False}
        )
        seg.attach([rp])
        self.state["seg"] = seg
        self.results.write(
            event="camera_created", sensor_id=spec.sensor_id, prim_path=path,
            registry_prim_path=spec.prim_path, resolution=list(spec.resolution or (1280, 720)),
            annotators=spec.annotators,
        )

    def _setup_lidar(self, stage, registry, pose) -> None:
        enable_extension("isaacsim.sensors.rtx.nodes")
        from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor  # noqa: E402

        spec = next(iter(registry.by_modality(Modality.LIDAR)), None)
        if spec is None:
            raise RuntimeError("no lidar in the registry")

        # A rotary lidar's vertical FOV is a narrow BAND, not a hemisphere.
        # Example_Rotary_BEAMS.json (shipped in omni.sensors.nv.common) sweeps
        # elevations -15.0 deg to +10.0 deg with nearRangeM 1.0. A ceiling mount
        # directly above the avatar looks at it from 90 deg of depression, which
        # is six times outside the band -- the sensor returns hundreds of
        # thousands of floor points and not one on the body, with no error.
        # SF_LIDAR_POSE overrides the ceiling pose so a pose that respects the
        # band can be measured rather than argued about.
        override = os.environ.get("SF_LIDAR_POSE")
        if override:
            pose = Gf.Vec3d(*[float(v) for v in override.split(",")])
            log(f"SF_LIDAR_POSE override -> {[float(v) for v in pose]}")

        path = f"{PROVISIONAL}/{spec.sensor_id}"
        log(f"PROVISIONAL lidar for {spec.sensor_id} at {path}, config={spec.config}")
        lidar = Lidar.create(path, config=spec.config, translations=np.array([[pose[0], pose[1], pose[2]]]))
        sensor = LidarSensor(lidar, annotators=["generic-model-output"])
        # 6.x debug draw. NOT the RtxLidarDebugDrawPointCloudBuffer replicator
        # writer that 5.x examples reach for. Guarded: the writer draws into a
        # viewport, and this run is headless -- if it cannot attach here that
        # says nothing about the GUI, and it must not take the capture down
        # with it.
        try:
            sensor.attach_writer("draw-point-cloud")
            self.state["draw_writer"] = "attached"
        except Exception as exc:
            self.state["draw_writer"] = f"failed: {exc!r}"
        log(f"draw-point-cloud writer: {self.state['draw_writer']}")
        self.state["lidar"] = sensor
        self.state["lidar_path"] = path

        # CONTROL: an identical lidar in empty space, far above everything, so
        # nothing is within range. "No sentinel in the warehouse" would prove
        # nothing on its own -- in a warehouse every ray hits something.
        ctrl_path = f"{PROVISIONAL}/LIDAR_CONTROL_EMPTY"
        ctrl = Lidar.create(ctrl_path, config=spec.config, translations=np.array([[0.0, 0.0, 1000.0]]))
        self.state["control"] = LidarSensor(ctrl, annotators=["generic-model-output"])
        self.state["control_path"] = ctrl_path
        log(f"control lidar at {ctrl_path} (z=1000 m, nothing in range)")
        self.results.write(
            event="lidar_created", sensor_id=spec.sensor_id, prim_path=path,
            registry_prim_path=spec.prim_path, config=spec.config,
            control_prim_path=ctrl_path, pose=[float(v) for v in pose],
        )

    # -- sampling ---------------------------------------------------------
    def sample(self) -> None:
        if MODE == "camera":
            self._sample_camera()
        else:
            self._sample_lidar()

    def _sample_camera(self) -> None:
        st = self.state
        arr = np.asarray(st["rgb"].get_data())
        nonzero = int((arr != 0).sum()) if arr.size else 0
        st["best_rgb"] = max(st.get("best_rgb", 0), nonzero)
        st["rgb_shape"] = list(arr.shape) if arr.size else None

        seg = st["seg"].get_data()
        labels = None
        n_person = 0
        if isinstance(seg, dict) and seg.get("data") is not None:
            data = np.asarray(seg["data"])
            info = seg.get("info") or {}
            labels = info.get("idToLabels")
            st["seg_shape"] = list(data.shape)
            if labels:
                person_ids = [
                    int(k) for k, v in labels.items()
                    if "person" in json.dumps(v).lower()
                ]
                if person_ids:
                    n_person = int(np.isin(data, person_ids).sum())
                st["labels"] = labels
        st["best_person_px"] = max(st.get("best_person_px", 0), n_person)

        if self.sampled % 20 == 0 or n_person:
            self.results.write(
                event="camera_frame", frame=self.frame, rgb_nonzero=nonzero,
                rgb_shape=st.get("rgb_shape"), seg_shape=st.get("seg_shape"),
                person_px=n_person, n_labels=len(labels) if labels else 0,
            )

    def _sample_lidar(self) -> None:
        from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

        st = self.state
        stage = self.ctx.get_stage()
        cache = UsdGeom.XformCache()

        for key, path_key in (("lidar", "lidar_path"), ("control", "control_path")):
            sensor = st.get(key)
            if sensor is None:
                continue
            try:
                buf, _ = sensor.get_data("generic-model-output")
            except Exception:
                continue
            if buf is None:
                continue
            gmo = parse_generic_model_output_data(buf)
            if gmo is None:
                continue
            # Report what the buffer actually offers once, rather than assuming
            # field names -- the conventions here are settable per prim and the
            # defaults are the documented trap.
            if not st.get("gmo_introspected"):
                st["gmo_introspected"] = True
                fields = [a for a in dir(gmo) if not a.startswith("_")]
                # The conventions and the FOV, read off the buffer rather than
                # assumed: both coordinate type and frame of reference are
                # settable per prim, and the elevation band is what decides
                # whether a given mount can see the avatar at all.
                header = {}
                for f in ("elementsCoordsType", "frameOfReference", "minElRad", "maxElRad",
                          "minAzRad", "maxAzRad", "maxRangeM", "numRows", "numCols"):
                    try:
                        header[f] = float(getattr(gmo, f))
                    except Exception:
                        try:
                            header[f] = str(getattr(gmo, f))
                        except Exception:
                            header[f] = "<unavailable>"
                self.results.write(event="gmo_fields", which=key, fields=fields, header=header)
                log(f"gmo header: {header}")
            m = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(st[path_key]))
            dec = decode_gmo(gmo, m)
            pts = dec.pop("_points", None)

            best = st.setdefault(f"{key}_best", {"real": 0})
            if dec.get("real", 0) >= best.get("real", 0):
                st[f"{key}_best"] = dict(dec)
            st[f"{key}_sentinel_total"] = st.get(f"{key}_sentinel_total", 0) + dec.get("sentinel_hits", 0)

            if pts is not None and key == "lidar" and "avatar_lo" in st:
                hits = count_in_box(pts, st["avatar_lo"], st["avatar_hi"])
                st["avatar_hits_best"] = max(st.get("avatar_hits_best", 0), hits)
                if self.sampled % 20 == 0:
                    self.results.write(
                        event="lidar_frame", frame=self.frame, which=key,
                        avatar_hits=hits, **{k: v for k, v in dec.items()},
                    )
            elif self.sampled % 20 == 0:
                self.results.write(event="lidar_frame", frame=self.frame, which=key, **dec)

    # -- finish -----------------------------------------------------------
    def finish(self) -> None:
        st = self.state
        if MODE == "camera":
            summary = {
                "mode": "camera",
                "frames": self.sampled,
                "rgb_max_nonzero_px": st.get("best_rgb", 0),
                "rgb_shape": st.get("rgb_shape"),
                "seg_shape": st.get("seg_shape"),
                "person_px_max": st.get("best_person_px", 0),
                "idToLabels": st.get("labels"),
            }
        else:
            summary = {
                "mode": "lidar",
                "frames": self.sampled,
                "warehouse_lidar_best": st.get("lidar_best"),
                "control_lidar_best": st.get("control_best"),
                "warehouse_sentinel_total": st.get("lidar_sentinel_total", 0),
                "control_sentinel_total": st.get("control_sentinel_total", 0),
                "avatar_hits_max": st.get("avatar_hits_best", 0),
                "avatar_bbox": [
                    [float(v) for v in st["avatar_lo"]],
                    [float(v) for v in st["avatar_hi"]],
                ] if "avatar_lo" in st else None,
            }
        self.results.write(event="summary", **summary)
        log("SUMMARY " + json.dumps(summary, default=str)[:2000])
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


main()
