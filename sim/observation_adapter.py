"""S11: the live simulator as an ``ObservationSource``. Layer 1 -> Layer 3.

This is the **only** place where a simulator type becomes a plain dict. Isaac
objects -- annotators, warp buffers, ``Gf.Matrix4d``, the RTX
``generic-model-output`` struct -- stop here, and what leaves is
``core.observation.Observation``: floats, numpy arrays, and strings a Habitat
adapter could produce just as well.

The gate is not "it runs". The gate is ``tests/contract.py``, which
``core.mock_source.MockObservationSource`` already passes and which this class
must pass **unchanged** -- see ``tasks/MACBOOK.md`` M3, which built that suite
against the protocol rather than against the mock precisely so it could be
pointed here. ``sim/tests/test_observation_adapter.py`` is the few lines that
point it, and :func:`main` below is what runs it inside Kit.

WHAT THIS FILE OWES, AND WHY EACH DEBT IS SILENT
------------------------------------------------
Failure mode 2 in CLAUDE.md is aimed at this file. ``generic-model-output`` is
**spherical and sensor-local by default** -- per-element x/y/z are azimuth
degrees, elevation degrees, range metres -- while ``core/observation.py``
promises ``points`` as ``(N, 3)`` float **world metres**. ``(N, 3) float`` is
satisfied by degrees exactly as well as by metres, so every one of these fails
without an error:

1. **spherical -> Cartesian.** Read raw, the numbers look plausible and are
   wrong everywhere. Done here by ``IsaacExtractRTXSensorPointCloud``, which
   exists for this and is the preferred path (CLAUDE.md); the spherical decode
   below is the fallback for when that annotator is unavailable.
2. **sensor -> world.** Skipping this is the one that breaks *fusion
   specifically*: every station's cloud lands on the origin and each cloud
   still looks plausible on its own. The annotator hands back the capture-time
   sensor-to-world matrix; it does **not** apply it -- NVIDIA's own
   ``test_point_cloud_annotator.py`` asserts its output equals
   ``r*cos(el)*cos(az)``, i.e. sensor-local. Applying it is this file's job.
3. **``flags & VALID``.** And VALID does not mean "real": the radar's
   empty-scene sentinel at exactly ``azimuth 0, elevation 0, range 100.000 m``
   carries the bit, passes range gating (``maxRangeM`` defaults to 200) and
   passes ``(N, 3) float`` typing. It is dropped here by its exact triple,
   because no single flag and no range bound will do it for you. ``flags`` is
   also the reason the raw buffer is still read every tick: the extract
   annotator publishes azimuth, elevation, distance, intensity, normals and
   ids -- and no flags at all.

Two more of the same shape, on the camera side:

4. **``distance_to_camera`` writes 0 where the ray hit nothing.** Replicator's
   own annotator documentation: "0 in the 2d array represents infinity". The
   contract's spelling for that is ``inf``; left at 0 it reads as a surface
   *at the sensor origin* -- nearer than anything real, so it wins every
   min-depth and nearest-obstacle query in the project.
5. **``rgb`` arrives (H, W, 4).** The payload table says three channels. The
   alpha comes off here, not in whatever reads this next.

Derivations for 1-3: ``sim/spikes/FINDINGS.md``.

FRAMES AND UNITS
----------------
``points`` leave here in the **world** frame, in metres, per the promise in
``core/observation.py``. ``core/mock_source.py`` emits its clouds
sensor-local, and says so in a comment -- the two sources disagree, the
contract cannot see it (for a FIXED sensor the two conventions are
indistinguishable), and ``core/observation.py``'s own "STILL NOT PINNED DOWN"
note is about exactly this. Every range reading records which frame it is in,
under ``intrinsics["frame"]``, so a consumer can at least ask.

Poses come from USD and are therefore in **stage units**, which is a different
statement: they are scaled by ``UsdGeom.GetStageMetersPerUnit`` here.
Replicator's depth already accounts for it and is not scaled twice.

EXECUTION MODEL -- and what ``step()`` can possibly mean
--------------------------------------------------------
Anything that reads sensor data runs in exec mode (CLAUDE.md), so this module
lives inside an already-built renderer and may not call ``app.update()``:
frames come from the update event stream. But ``ObservationSource.step()`` is
synchronous -- it advances time and returns readings -- and a synchronous loop
on the main thread can never yield to that stream. The contract calls
``step()`` in a plain loop, so both have to be true at once.

They are, by making ``step()`` thread-aware and changing nothing else:

    on Kit's main thread     read what the renderer last produced. This is the
                             capture loop: you are already inside an update
                             callback and a frame has just happened.
    on any other thread      post a request, let the main thread service it on
                             its next update, block until it has.

Sampling ALWAYS happens on the main thread. The worker only ever touches numpy
arrays that were copied there. That is what makes running the contract suite
in a thread (see :func:`main`) safe rather than merely lucky.

Copies are not optional either: annotator buffers are recycled, and the
contract's ``trace`` fixture holds every tick of a walk at once. Handing out
views would make all of them alike and turn "the sensor never reacted" into a
finding about this adapter.

Run the contract suite against the live simulator::

    docker compose -f docker/docker-compose.yml run --rm sim \\
        ./runheadless.sh --exec /workspace/sim/observation_adapter.py

Environment (argv is ambiguous after ``--exec``, so config is env vars):

    OA_STAGE        stage to open  (default: /workspace/sim/observatory_avatar.usd)
    OA_MODE         contract | smoke        (default: contract)
    OA_OUT          results directory       (default: the logs volume)
    OA_WARMUP       max frames to wait for every sensor to fill (default 300)
    OA_SETTLE       frames between advancing the world and sampling (default 2)
    OA_STEPS        smoke mode only: how many steps to sample (default 20)
    OA_NO_AUTORUN=1 import this module without running anything
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time as _time
import traceback
from pathlib import Path
from typing import Any, Callable

import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.experimental.utils.app import enable_extension
from pxr import Gf, UsdGeom

REPO = Path(__file__).resolve().parent.parent
HERE = Path(__file__).resolve().parent
for _p in (str(REPO), str(HERE)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Importing sensor_factory must not run its capture -- see
# sensor_factory._is_exec_entrypoint. Set before the import, never after.
os.environ.setdefault("SF_NO_AUTORUN", "1")

import sensor_factory as sf  # noqa: E402  -- sibling module, see sys.path above
from core.observation import (  # noqa: E402
    ANNOTATOR_DATA_KEYS,
    MODALITY_DATA_KEYS,
    Modality,
    MountType,
    Observation,
    Pose,
)
from core.registry import RANGE_MODALITIES, SensorRegistry, SensorSpec  # noqa: E402

STAGE = os.environ.get("OA_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
MODE = os.environ.get("OA_MODE", "contract")
OUT_DIR = Path(os.environ.get("OA_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
WARMUP_FRAMES = int(os.environ.get("OA_WARMUP", "300"))
SETTLE_FRAMES = int(os.environ.get("OA_SETTLE", "2"))
SMOKE_STEPS = int(os.environ.get("OA_STEPS", "20"))

# The Replicator annotator that converts the GMO buffer to Cartesian and hands
# back the capture-time sensor-to-world matrix. Registered by
# isaacsim.sensors.rtx.nodes -- the same extension that registers the
# "draw-point-cloud" writer sim/sensor_factory.py attaches.
POINT_CLOUD_ANNOTATOR = "IsaacExtractRTXSensorPointCloud"
RTX_NODES_EXT = "isaacsim.sensors.rtx.nodes"

# Read out of the shipped enums rather than recalled:
#   CoordsType       {CARTESIAN: 0, SPHERICAL: 1, NOT_APPLICABLE: 2}
#   FrameOfReference {SENSOR: 0, WORLD: 1, CUSTOM: 2, PARENT: 3}
#   ElementFlags     {..., VALID: 64}
# (omni.sensors.generic_model_output, Isaac Sim 6.0.1, verified in-image.)
COORDS_CARTESIAN, COORDS_SPHERICAL = 0, 1
FRAME_SENSOR, FRAME_WORLD, FRAME_CUSTOM, FRAME_PARENT = 0, 1, 2, 3
VALID = sf.VALID

_DEFAULT_DT = 1.0 / 60.0

# This module is imported on Kit's main thread -- by --exec, or by the test
# module the worker thread runs. Captured HERE and not in the constructor,
# because a source built by pytest is built on the worker: a constructor-time
# capture would record the worker as "main", step() would sample without ever
# asking for a frame, and every tick of the walk would come back identical.
_MAIN_THREAD = threading.get_ident()

# One point-cloud reader per sensor prim, not per source. The contract builds
# a fresh source for every test, and an annotator attached per source would
# stack twenty-odd of them on one render product.
_RANGE_READERS: dict[str, "_RangeReader"] = {}

#: The rig the exec-mode driver built, published for
#: sim/tests/test_observation_adapter.py. Empty in any other context.
LIVE: dict[str, Any] = {}


def log(msg: str) -> None:
    print(f"[observation_adapter] {msg}", flush=True)


# ---------------------------------------------------------------------------
# Simulator types -> plain values. Everything below this line is a conversion
# that fails silently if it is skipped.
# ---------------------------------------------------------------------------
def _to_numpy(value: Any) -> np.ndarray | None:
    """A host numpy array from whatever an annotator handed back.

    Warp arrays (the RTX annotators return one, on the GPU) carry ``.numpy()``;
    Replicator's camera annotators already return numpy. Either way the result
    is COPIED: annotator buffers are recycled between frames.
    """
    if value is None:
        return None
    if hasattr(value, "numpy"):
        value = value.numpy()
    arr = np.asarray(value)
    return np.array(arr, copy=True)


def _pose_from_matrix(m: Gf.Matrix4d, mpu: float) -> Pose:
    """World ``Pose`` from a USD local-to-world matrix.

    Two conversions, both invisible if skipped. The translation is in STAGE
    UNITS and the contract's unit is the metre (``LENGTH_UNIT``): a stage
    authored at metersPerUnit 0.01 reports 650.0 for a station 6.5 m up and
    nothing complains. And the rotation is taken after RemoveScaleShear, so a
    scaled parent yields a rotation rather than a scaled quaternion that fails
    ``|q| == 1`` two layers downstream.
    """
    t = m.ExtractTranslation()
    position = (float(t[0]) * mpu, float(t[1]) * mpu, float(t[2]) * mpu)
    q = Gf.Matrix4d(m).RemoveScaleShear().ExtractRotationQuat()
    i = q.GetImaginary()
    quat = np.array([q.GetReal(), i[0], i[1], i[2]], dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    quat = quat / norm if norm > 0.0 else np.array([1.0, 0.0, 0.0, 0.0])
    return Pose(position, (float(quat[0]), float(quat[1]),
                           float(quat[2]), float(quat[3])))


def _rgb_payload(raw: Any) -> np.ndarray | None:
    """``(H, W, 3)`` uint8 from Isaac's ``(H, W, 4)`` RGBA.

    The payload table in core/observation.py says THREE channels, and says so
    because nobody reading only the key name would know that both Isaac's rgb
    annotator and Habitat's color sensor hand over four.
    """
    arr = _to_numpy(raw)
    if arr is None or arr.size == 0:
        return None
    if arr.ndim == 3 and arr.shape[2] >= 3:
        arr = arr[:, :, :3]
    return np.ascontiguousarray(arr).astype(np.uint8, copy=False)


def _depth_payload(raw: Any) -> np.ndarray | None:
    """Euclidean metres from ``distance_to_camera``, with 0 restored to inf.

    ``distance_to_camera`` is euclidean range from the camera origin -- which
    is what DEPTH_CONVENTION asks for, and is why this annotator and not
    ``distance_to_image_plane``. Replicator already applies metersPerUnit, so
    it is not scaled again here.

    The trap is the background value. Replicator's annotator documentation:
    "0 in the 2d array represents infinity (which means there is no object in
    that pixel)". Zero is a legal float and passes every type check, and it
    reads as a surface at zero range -- nearer than anything real, so it wins
    every min-depth and nearest-obstacle query. ``inf`` is the contract's
    spelling for a ray that hit nothing. NaN never is, so any that arrive are
    mapped rather than propagated.
    """
    arr = _to_numpy(raw)
    if arr is None or arr.size == 0:
        return None
    if arr.ndim == 3 and arr.shape[2] == 1:
        arr = arr[:, :, 0]
    depth = np.array(arr, dtype=np.float32, copy=True)
    depth[depth == 0.0] = np.inf
    depth[np.isnan(depth)] = np.inf
    return depth


def _semantic_payload(raw: Any) -> tuple[np.ndarray, dict] | None:
    """``(H, W)`` class ids plus the id->label mapping they are useless without.

    Isaac's ``semantic_segmentation`` with ``colorize=False`` returns
    ``{"data": ..., "info": {"idToLabels": ...}}``; the mapping is passed
    through as Isaac spells it (ids as strings, class name nested a level
    down) rather than rewritten, because the contract is that the id is NAMED,
    not that every source spells its mapping the same way.
    """
    if not isinstance(raw, dict):
        return None
    data = _to_numpy(raw.get("data"))
    if data is None or data.size == 0:
        return None
    if data.ndim == 3 and data.shape[2] == 1:
        data = data[:, :, 0]
    labels = (raw.get("info") or {}).get("idToLabels") or {}
    return data, dict(labels)


def _matrix_4x4(value: Any) -> np.ndarray | None:
    """A 4x4 row-major matrix from the annotator's ``transform`` output.

    USD's convention is row-vector: translation sits in row 3 and a point
    transforms as ``p_world = p_local * M``. That is what
    sim/sensor_factory.decode_gmo already applies, and it is applied the same
    way here so the two paths cannot quietly disagree.
    """
    if value is None:
        return None
    arr = _to_numpy(value)
    if arr is None or arr.size != 16:
        return None
    return np.asarray(arr, dtype=np.float64).reshape(4, 4)


# ---------------------------------------------------------------------------
# Range sensors: the GMO buffer, and the three conversions it owes
# ---------------------------------------------------------------------------
class _RangeReader:
    """Reads one RTX lidar/radar and returns a metric WORLD point cloud.

    Prefers ``IsaacExtractRTXSensorPointCloud`` because CLAUDE.md says to and
    because it is NVIDIA's own decode of NVIDIA's own buffer: it does the
    spherical->Cartesian conversion and publishes the capture-time
    sensor-to-world matrix. It does not *apply* that matrix -- their test
    asserts the output is sensor-local -- so step 2 happens here either way.

    ``generic-model-output`` is read every tick as well, and not as a
    fallback: it is the only place ``flags`` lives, so the VALID mask and the
    sentinel drop are only possible from it. The two are element-aligned by
    construction -- the extract node emits exactly ``numElements`` points, in
    input order, which is what NVIDIA's test asserts elementwise.
    """

    @classmethod
    def for_record(cls, sensor_id: str, spec: SensorSpec, rec: dict) -> "_RangeReader":
        """One reader per sensor prim, shared across sources. See _RANGE_READERS."""
        key = rec.get("prim_path") or sensor_id
        reader = _RANGE_READERS.get(key)
        if reader is None:
            reader = cls(sensor_id, spec, rec)
            _RANGE_READERS[key] = reader
        return reader

    def __init__(self, sensor_id: str, spec: SensorSpec, rec: dict) -> None:
        self.sensor_id = sensor_id
        self.spec = spec
        self.rec = rec
        self.sensor = rec.get("sensor")
        self.annotator = None
        self.source = "generic-model-output"
        self._warned: set[str] = set()
        self._attach_point_cloud()

    def _attach_point_cloud(self) -> None:
        """Attach the extract annotator to this sensor's own render product.

        Attached directly rather than through ``LidarSensor(annotators=[...])``
        because the runtime class copies ANNOTATOR_SPEC at CONSTRUCTION time:
        registering the name afterwards -- which is the only moment this module
        gets, since sensor_factory built the sensor -- would be accepted by the
        registry and then rejected by the instance. ``render_product`` is
        public API on the sensor runtime and needs no such ordering.
        """
        if self.sensor is None:
            return
        try:
            enable_extension(RTX_NODES_EXT)
            rp_path = str(self.sensor.render_product.GetPath())
            annotator = rep.AnnotatorRegistry.get_annotator(POINT_CLOUD_ANNOTATOR)
            annotator.attach(rp_path)
            self.annotator = annotator
            self.source = POINT_CLOUD_ANNOTATOR
            log(f"{self.sensor_id}: {POINT_CLOUD_ANNOTATOR} attached to {rp_path}")
        except Exception as exc:
            log(f"! {self.sensor_id}: {POINT_CLOUD_ANNOTATOR} unavailable "
                f"({exc!r}) -- decoding generic-model-output directly instead")

    def _warn_once(self, key: str, msg: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            log(msg)

    # -- the read ----------------------------------------------------------
    def read(self, stage, xform_cache, mpu: float) -> tuple[dict, dict] | None:
        """One tick's payload and intrinsics, or None if nothing is ready."""
        from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

        if self.sensor is None:
            return None
        try:
            buf, _ = self.sensor.get_data("generic-model-output")
        except Exception as exc:
            self._warn_once("gmo", f"! {self.sensor_id}: get_data failed: {exc!r}")
            return None
        if buf is None:
            return None
        gmo = parse_generic_model_output_data(buf)
        if gmo is None or int(gmo.numElements) == 0:
            return None
        n = int(gmo.numElements)

        # ONE read of the extract annotator per tick. Two reads could straddle
        # a frame and pair a cloud with the wrong sensor pose.
        extracted = self._extracted()

        az = self._element_array(gmo, "x", n)
        el = self._element_array(gmo, "y", n)
        rng = self._element_array(gmo, "z", n)
        if az is None or el is None or rng is None:
            self._warn_once(
                "basic",
                f"! {self.sensor_id}: the buffer reports {n} elements but its "
                f"x/y/z arrays are not that long -- nothing decodable here")
            return None
        flags = self._element_array(gmo, "flags", n, dtype=np.int64)
        if flags is None:
            flags = np.full(n, VALID, dtype=np.int64)
        coords = self._enum(gmo, "elementsCoordsType", COORDS_SPHERICAL)
        frame = self._enum(gmo, "outputFrameOfReference",
                           self._enum(gmo, "frameOfReference", FRAME_SENSOR))

        # (3) VALID, and then the sentinel -- because VALID does not mean real.
        valid = (flags & VALID) != 0
        sentinel = ((np.abs(az) < 1e-6) & (np.abs(el) < 1e-6)
                    & (np.abs(rng - 100.0) < 1e-6))
        keep = valid & ~sentinel
        n_valid, n_sentinel = int(valid.sum()), int(sentinel.sum())
        if n_sentinel:
            self._warn_once(
                "sentinel",
                f"{self.sensor_id}: dropping the no-detection sentinel at "
                f"exactly (0, 0, 100.000 m) -- {n_sentinel} element(s) this "
                f"tick, {int((sentinel & valid).sum())} of them flagged VALID")

        # (1) spherical -> Cartesian, sensor-local, metres.
        local = self._cartesian_local(extracted, coords, az, el, rng, n)
        if local is None:
            return None
        local = local[keep]

        # (2) sensor -> world. Skipped, every station's cloud lands on the
        # origin and each one still looks plausible on its own.
        matrix, matrix_from = self._sensor_to_world(extracted, stage, xform_cache, mpu)
        in_world = True
        if frame == FRAME_WORLD:
            points = local
            matrix_from = "not applied (buffer already in the world frame)"
        elif matrix is not None:
            points = local @ matrix[:3, :3] + matrix[3, :3]
        else:
            points = local
            in_world = False
            self._warn_once(
                "frame",
                f"! {self.sensor_id}: frameOfReference={frame} and no "
                f"sensor-to-world matrix -- points stay SENSOR-LOCAL and will "
                f"not fuse with any other station")
            matrix_from = "none available"

        points = np.ascontiguousarray(points, dtype=np.float32)
        data: dict[str, Any] = {
            "points": points,
            "ranges": rng[keep].astype(np.float32),
            "num_returns": int(points.shape[0]),
        }
        scalar = self._element_array(gmo, "scalar", n, dtype=np.float32)
        if scalar is not None:
            # Modality-specific per the GMO spec: lidar normalised intensity,
            # radar RCS in dBsm. Named for what it is in each case, and the
            # mock fills the same two keys.
            key = "rcs" if self.spec.modality is Modality.RADAR else "intensities"
            data[key] = scalar[keep]
        radial = self._element_array(gmo, "rv_ms", n, dtype=np.float32)
        if radial is not None:
            data["radial_velocities"] = radial[keep]

        intrinsics = {
            "config": self.spec.config,
            "frame": "world" if in_world else "sensor",
            "decoded_by": self.source,
            "sensor_to_world": matrix_from,
            "coords_type": "spherical" if coords == COORDS_SPHERICAL else "cartesian",
            "frame_of_reference": int(frame),
            "elements": n,
            "valid": n_valid,
            "sentinel_dropped": n_sentinel,
            "max_range_m": self._maybe_float(gmo, "maxRangeM"),
        }
        return data, intrinsics

    # -- pieces ------------------------------------------------------------
    def _extracted(self) -> dict | None:
        """This tick's ``IsaacExtractRTXSensorPointCloud`` output, or None."""
        if self.annotator is None:
            return None
        try:
            raw = self.annotator.get_data()
        except Exception as exc:
            self._warn_once("annerr", f"! {self.sensor_id}: "
                            f"{POINT_CLOUD_ANNOTATOR} read failed: {exc!r}")
            return None
        if isinstance(raw, dict):
            return {"data": raw.get("data"), "info": raw.get("info") or {}}
        return {"data": raw, "info": {}}

    def _cartesian_local(self, extracted, coords, az, el, rng, n) -> np.ndarray | None:
        if extracted is not None:
            arr = _to_numpy(extracted.get("data"))
            if arr is not None and arr.size >= 3 * n:
                return arr.reshape(-1, 3)[:n].astype(np.float64)
            self._warn_once(
                "short",
                f"! {self.sensor_id}: {POINT_CLOUD_ANNOTATOR} returned "
                f"{0 if arr is None else arr.size} floats for {n} elements -- "
                f"decoding the buffer directly instead")
        if coords != COORDS_SPHERICAL:
            return np.stack([az, el, rng], axis=1)
        a, e = np.radians(az), np.radians(el)
        # cos(el) kept on x and y. NVIDIA's own radar test drops it; across a
        # rotary lidar's elevation spread that is a real error.
        return np.stack([rng * np.cos(e) * np.cos(a),
                         rng * np.cos(e) * np.sin(a),
                         rng * np.sin(e)], axis=1)

    def _sensor_to_world(self, extracted, stage, xform_cache,
                         mpu: float) -> tuple[np.ndarray | None, str]:
        """The capture-time matrix if the annotator published one, else USD's.

        The annotator's is authoritative: it is the pose the rays were cast
        from, not the pose the prim happens to hold now, and for a sensor that
        moves those differ by a frame.
        """
        if extracted is not None:
            matrix = _matrix_4x4(extracted["info"].get("transform"))
            if matrix is not None:
                matrix = matrix.copy()
                matrix[3, :3] *= mpu
                return matrix, f"{POINT_CLOUD_ANNOTATOR}.transform"
        prim = stage.GetPrimAtPath(self.rec["prim_path"])
        if not prim.IsValid():
            return None, "none"
        matrix = np.array(xform_cache.GetLocalToWorldTransform(prim),
                          dtype=np.float64).reshape(4, 4)
        matrix[3, :3] *= mpu
        return matrix, "UsdGeom.XformCache"

    @staticmethod
    def _element_array(gmo, name: str, n: int, dtype=np.float64) -> np.ndarray | None:
        """One per-element array of length exactly `n`, or None.

        The length is CHECKED, not assumed. A lidar's buffer carries the
        radar-only ``rv_ms`` member as an unfilled pointer, and slicing that
        returns an EMPTY array rather than raising -- which then surfaces, a
        hundred lines later, as "boolean index did not match indexed array,
        size of axis is 0". Measured 2026-08-25: the lidar returned 289,930
        elements and an empty rv_ms every frame, and every reading was
        dropped for it.
        """
        try:
            values = getattr(gmo, name, None)
            if values is None:
                return None
            arr = np.asarray(values[:n], dtype=dtype)
        except Exception:
            return None
        return arr.copy() if arr.shape == (n,) else None

    @staticmethod
    def _enum(gmo, name: str, default: int) -> int:
        try:
            return int(getattr(gmo, name))
        except Exception:
            return default

    @staticmethod
    def _maybe_float(gmo, name: str):
        try:
            return float(getattr(gmo, name))
        except Exception:
            return None


# ---------------------------------------------------------------------------
# The source
# ---------------------------------------------------------------------------
class _Request:
    """One off-thread ``step()`` waiting for the main loop to service it."""

    __slots__ = ("t", "settle", "advanced", "result", "error", "done")

    def __init__(self, t: float, settle: int) -> None:
        self.t = t
        self.settle = settle
        self.advanced = False
        self.result: list[Observation] | None = None
        self.error: BaseException | None = None
        self.done = threading.Event()


class IsaacObservationSource:
    """``core.observation.ObservationSource``, backed by a live Isaac stage.

    Built from sensors that already exist -- ``sensor_factory`` creates them,
    from the registry, and this class only reads. The split is deliberate: a
    GUI session (sim/gui_viewports.py) and a headless capture both hand it the
    same ``created`` dict, and neither has to rebuild its sensors to be
    observed through.

    ``sensor_ids`` and ``time`` return real values from the constructor and
    raise nothing. Not a style preference: ``isinstance(src,
    ObservationSource)`` calls ``hasattr`` on every protocol member, ``hasattr``
    evaluates a property, and a property raising anything but AttributeError
    propagates -- so a half-built adapter dies *inside*
    ``test_satisfies_the_protocol`` with that error instead of failing
    readably. Both are answerable with no simulator involved.
    """

    def __init__(
        self,
        stage,
        registry: SensorRegistry,
        created: dict[str, dict],
        *,
        dt: float = _DEFAULT_DT,
        settle_frames: int = SETTLE_FRAMES,
        timeout_s: float = 600.0,
        advance_world: Callable[[float], None] | None = None,
        action_source: Callable[[float], Any] | None = None,
        owns_sensors: bool = False,
    ) -> None:
        if dt <= 0.0:
            raise ValueError(f"dt must be positive, got {dt}")
        self._stage = stage
        self._registry = registry
        self._created = dict(created)
        self._dt = float(dt)
        self._settle = max(0, int(settle_frames))
        self._timeout = float(timeout_s)
        self._advance_world = advance_world
        self._action_source = action_source
        self._owns = owns_sensors
        self._t = 0.0
        self._closed = False
        self._mpu = float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)

        # Only sensors that were actually built AND are in the registry.
        # Declaring one that was never created would promise a reading no tick
        # can produce; the contract allows returning fewer than `sensor_ids`,
        # never an id that was not declared.
        self._specs: dict[str, SensorSpec] = {}
        for sensor_id in self._created:
            if sensor_id in registry:
                self._specs[sensor_id] = registry.get(sensor_id)
            else:
                log(f"! {sensor_id} was created but is not in the registry -- ignored")
        self._ids = tuple(sorted(self._specs))

        self._range = {
            sensor_id: _RangeReader.for_record(sensor_id, spec, self._created[sensor_id])
            for sensor_id, spec in self._specs.items()
            if spec.modality in RANGE_MODALITIES
        }

        self._missing_warned: set[str] = set()
        self._lock = threading.Lock()
        self._request: _Request | None = None
        self._sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_update, name=f"observation_adapter_{id(self)}"
        )

        if getattr(sf, "RES_SCALE", 1.0) != 1.0:
            log(f"! SF_RES_SCALE={sf.RES_SCALE} -- render products are NOT at the "
                f"registry's declared resolution. Fine for a smoke test, wrong "
                f"for a capture: the declared resolution is the contract.")

    # --- construction helper -------------------------------------------------
    @classmethod
    def build(cls, stage, registry: SensorRegistry | None = None, **kwargs: Any):
        """Create the registry's sensors on `stage`, then observe them.

        For a caller holding a stage and nothing else. A GUI session already
        has a ``created`` dict and should pass it to the constructor rather
        than building a second set of sensors on the same prims.
        """
        registry = registry or sf.load_registry()
        created = sf.create_registry_sensors(stage, registry)
        return cls(stage, registry, created, owns_sensors=True, **kwargs)

    # --- ObservationSource ---------------------------------------------------
    @property
    def sensor_ids(self) -> tuple[str, ...]:
        return self._ids

    @property
    def time(self) -> float:
        return self._t

    def step(self, dt: float | None = None) -> list[Observation]:
        """Advance simulated time and return this tick's readings.

        On Kit's main thread this reads what the renderer last produced -- you
        are inside an update callback and a frame has just happened. From any
        other thread it asks the main loop for a fresh frame and blocks until
        it arrives. See the execution-model note in the module docstring.
        """
        if self._closed:
            raise RuntimeError("step() on a closed source")
        self._t += self._dt if dt is None else float(dt)
        if threading.get_ident() == _MAIN_THREAD:
            if self._advance_world is not None:
                self._advance_world(self._t)
            return self._read_all()
        return self._step_off_thread()

    def close(self) -> None:
        """Release what this source holds. Safe to call twice.

        The SENSORS are not released unless this source created them: a GUI
        session's viewports are bound to those prims, and the contract suite
        builds and closes a source per test over one set of sensors.
        """
        if self._closed:
            return
        self._closed = True
        self._sub = None
        with self._lock:
            request, self._request = self._request, None
        if request is not None:                  # never leave a waiter hanging
            request.error = RuntimeError("source closed while a step was pending")
            request.done.set()
        if self._owns:
            for sensor_id, rec in self._created.items():
                sensor = rec.get("sensor")
                if sensor is None:
                    continue
                try:
                    sensor.detach_annotators(sensor.annotators)
                except Exception as exc:
                    log(f"! {sensor_id}: detach failed: {exc!r}")

    # --- beyond the protocol: what a driver needs ----------------------------
    def sample_now(self) -> list[Observation]:
        """Read the current buffers without advancing the clock. Main thread.

        This is what warm-up polls: "has every sensor filled yet" has to be
        answerable without spending simulated time to ask.
        """
        return self._read_all()

    def missing_payloads(self) -> dict[str, list[str]]:
        """Per sensor, the promised payload keys that have not arrived yet.

        Empty means every sensor is live. An RTX sensor returns nothing for
        the first frames after Play and says nothing about it, so a capture
        that samples immediately records a scene that had not started.
        """
        present = {obs.sensor_id: set(obs.data) for obs in self._read_all()}
        out: dict[str, list[str]] = {}
        for sensor_id, spec in self._specs.items():
            promised = set(MODALITY_DATA_KEYS[spec.modality])
            promised |= {ANNOTATOR_DATA_KEYS[a] for a in spec.annotators}
            gap = promised - present.get(sensor_id, set())
            if gap:
                out[sensor_id] = sorted(gap)
        return out

    @property
    def registry(self) -> SensorRegistry:
        return self._registry

    # --- the frame pump ------------------------------------------------------
    def _step_off_thread(self) -> list[Observation]:
        request = _Request(self._t, self._settle)
        with self._lock:
            if self._request is not None:
                raise RuntimeError("a step is already pending -- one caller at a time")
            self._request = request
        if not request.done.wait(self._timeout):
            with self._lock:
                self._request = None
            raise TimeoutError(
                f"no frame in {self._timeout:.0f}s. Either the main loop is "
                f"not running or the renderer died mid-run."
            )
        if request.error is not None:
            raise request.error
        return request.result or []

    def _on_update(self, _event) -> None:
        """Main thread. Services at most one pending step per frame."""
        request = self._request
        if request is None or self._closed:
            return
        try:
            if not request.advanced:
                request.advanced = True
                if self._advance_world is not None:
                    # The world advances ONCE, at the start of servicing, so
                    # that what gets sampled is the world at `t` rather than
                    # somewhere between two ticks. Determinism is a
                    # capture-mode requirement (CLAUDE.md rule 6), not a
                    # nicety.
                    self._advance_world(request.t)
            if request.settle > 0:
                # Let the renderer see the pose that was just written before
                # anything is read off it.
                request.settle -= 1
                return
            request.result = self._read_all()
        except BaseException as exc:                        # noqa: BLE001
            request.error = exc
        with self._lock:
            self._request = None
        request.done.set()

    # --- reading -------------------------------------------------------------
    def _read_all(self) -> list[Observation]:
        # A FRESH cache every tick. A retained XformCache returns the pose it
        # saw first, forever -- the avatar then never moves, every sensor
        # keeps reporting, and nothing raises. sim/avatar.py hit exactly this
        # one layer down, and calls Clear() per frame for the same reason.
        xform_cache = UsdGeom.XformCache()
        out: list[Observation] = []
        for sensor_id in self._ids:
            spec = self._specs[sensor_id]
            rec = self._created[sensor_id]
            prim = self._stage.GetPrimAtPath(rec["prim_path"])
            if not prim.IsValid():
                self._warn_missing(sensor_id, f"{rec['prim_path']} is not on the stage")
                continue
            try:
                if spec.modality in RANGE_MODALITIES:
                    payload = self._range[sensor_id].read(
                        self._stage, xform_cache, self._mpu)
                else:
                    payload = self._camera_payload(spec, rec, prim)
            except Exception as exc:
                self._warn_missing(sensor_id, f"read failed: {exc!r}")
                continue
            if payload is None:
                self._warn_missing(sensor_id, "no data yet (warm-up, or Play is off)")
                continue
            data, intrinsics = payload
            pose = _pose_from_matrix(
                xform_cache.GetLocalToWorldTransform(prim), self._mpu)
            # ONLY the avatar acts. The contract forbids an action on a FIXED
            # sensor, but a static robot platform has no more claim to one: it
            # is the moving mount that has an action before its reading and a
            # consequence after it, and stamping the avatar's walk onto a
            # bystander destroys exactly the state/experience distinction the
            # field exists to preserve.
            action = None
            if spec.mount is MountType.AVATAR and self._action_source is not None:
                action = self._action_source(self._t)
            out.append(Observation(
                sensor_id=sensor_id,
                timestamp=self._t,
                modality=spec.modality,
                mount=spec.mount,
                pose=pose,
                intrinsics=intrinsics,
                data=data,
                action=action,
            ))
        return out

    def _warn_missing(self, sensor_id: str, why: str) -> None:
        key = f"{sensor_id}:{why}"
        if key not in self._missing_warned:
            self._missing_warned.add(key)
            log(f"! {sensor_id} not reporting -- {why}")

    def _camera_payload(self, spec: SensorSpec, rec: dict,
                        prim) -> tuple[dict, dict] | None:
        annotators = rec.get("annotators") or {}
        if not annotators:
            return None
        data: dict[str, Any] = {}

        raw = annotators.get("rgb")
        if raw is not None:
            rgb = _rgb_payload(raw.get_data())
            if rgb is None:
                return None
            data["rgb"] = rgb

        raw = annotators.get("distance_to_camera")
        if raw is not None:
            depth = _depth_payload(raw.get_data())
            if depth is None:
                return None
            data["depth"] = depth

        raw = annotators.get("semantic_segmentation")
        if raw is not None:
            parsed = _semantic_payload(raw.get_data())
            if parsed is None:
                return None
            data["semantic"], data["semantic_labels"] = parsed

        if not data:
            return None
        return data, self._camera_intrinsics(spec, prim)

    def _camera_intrinsics(self, spec: SensorSpec, prim) -> dict:
        """The declared resolution, plus the pinhole model USD actually holds.

        Width and height come from the REGISTRY, not from the array: the
        registry is what a consumer was promised, and an array that disagrees
        with it is exactly what the contract is looking for -- reporting the
        array's own shape here would hide it.
        """
        width, height = spec.resolution or (0, 0)
        intrinsics: dict[str, Any] = {"width": int(width), "height": int(height)}
        try:
            camera = UsdGeom.Camera(prim)
            focal_mm = float(camera.GetFocalLengthAttr().Get() or 0.0)
            aperture_mm = float(camera.GetHorizontalApertureAttr().Get() or 0.0)
            if focal_mm > 0.0 and aperture_mm > 0.0:
                hfov = 2.0 * math.atan(aperture_mm / (2.0 * focal_mm))
                intrinsics["focal_length_mm"] = focal_mm
                intrinsics["horizontal_aperture_mm"] = aperture_mm
                intrinsics["horizontal_fov_deg"] = math.degrees(hfov)
                if width:
                    intrinsics["focal_length_px"] = (width / 2.0) / math.tan(hfov / 2.0)
            clip = camera.GetClippingRangeAttr().Get()
            if clip is not None:
                intrinsics["clipping_range_m"] = (float(clip[0]) * self._mpu,
                                                  float(clip[1]) * self._mpu)
        except Exception as exc:
            log(f"! {spec.sensor_id}: could not read the USD camera model: {exc!r}")
        return intrinsics


def live_source(**overrides: Any) -> IsaacObservationSource:
    """A source over the rig the exec-mode driver built. See :data:`LIVE`.

    This is what sim/tests/test_observation_adapter.py calls: the contract
    builds a fresh source per test, and each one has to observe the SAME
    sensors -- rebuilding them per test would recreate every prim, re-warm
    every annotator, and take the run from minutes to hours.
    """
    if not LIVE:
        raise RuntimeError(
            "no live rig. sim/observation_adapter.py has to be running under "
            "Kit for this -- see the module docstring for the exec-mode line.")
    kwargs = dict(LIVE.get("kwargs") or {})
    kwargs.update(overrides)
    return IsaacObservationSource(
        LIVE["stage"], LIVE["registry"], LIVE["created"], **kwargs)


# ===========================================================================
# Exec-mode driver: run the contract suite against the live simulator.
#
# Same shape as sim/sensor_factory.py -- a state machine on the update stream,
# results written incrementally and fsync'd, post_quit at the end. What is
# different is the last phase: pytest runs on a WORKER thread while this one
# keeps pumping Kit, because the contract calls step() in a plain loop and a
# plain loop here would never yield a frame. Every sample is still taken on
# this thread; the worker only ever sees numpy.
# ===========================================================================

#: Radius of the scripted walk, metres. Big enough that the ground-plane
#: spread over a trace is unambiguous, small enough to keep the avatar inside
#: the wall station's usable band (Example_Rotary sees -15..+10 deg only).
WALK_RADIUS = 2.5


class _CircuitWalk:
    """Moves the avatar's capsule as a function of the source's clock.

    The real avatar is keyboard-driven and has no trajectory at all, so a
    headless run has to supply one -- exactly as ``core/mock_source.py`` does,
    for the same reason. A circle, because constant speed makes distance over
    time something a test can reason about in closed form.

    USD writes, not physics: this driver runs WITHOUT
    ``--enable omni.physx.cct``, so the character-controller node type is
    unregistered, the Controls graph loads and does nothing, and nothing
    contends for the capsule's transform. The visible character follows via
    ``avatar.install_character_follow``, and it is the character -- render
    geometry -- that lidar and cameras actually see.

    Deliberately a function of SIMULATED time and not of frames: two runs at
    different frame rates then produce the same trajectory, which is what
    capture mode means by determinism.
    """

    def __init__(self, stage, avatar_path: str = "/Root/Avatar",
                 speed: float = 1.4, radius: float = WALK_RADIUS) -> None:
        self.speed = float(speed)
        self.radius = float(radius)
        body = stage.GetPrimAtPath(f"{avatar_path}/body_mesh")
        if not body.IsValid():
            raise RuntimeError(f"no capsule at {avatar_path}/body_mesh to walk")
        self.attr = body.GetAttribute("xformOp:translate")
        if not self.attr:
            self.attr = UsdGeom.Xform(body).AddTranslateOp().GetAttr()
        start = self.attr.Get() or Gf.Vec3d(0.0, 0.0, 0.0)
        # Keep the authored z: it is what puts the first-person camera at eye
        # height, and the contract checks that number against scene.yaml.
        self.z = float(start[2])
        # Start ON the circle so t=0 is the spawn pose and the avatar does not
        # teleport on the first tick.
        self.centre = (float(start[0]) - self.radius, float(start[1]))
        self.at(0.0)

    def position(self, t: float) -> tuple[float, float]:
        theta = (self.speed / self.radius) * t
        return (self.centre[0] + self.radius * math.cos(theta),
                self.centre[1] + self.radius * math.sin(theta))

    def at(self, t: float) -> None:
        x, y = self.position(t)
        self.attr.Set(Gf.Vec3d(x, y, self.z))

    def action(self, t: float) -> dict:
        """What the mount did to arrive at this reading.

        Untyped by design (``Observation.action`` is ``Any``): a keyboard event
        here, a discrete ``move_forward`` in Habitat. What matters is that an
        avatar reading carries one and a fixed reading does not.
        """
        theta = (self.speed / self.radius) * t
        return {"kind": "walk", "speed_mps": self.speed,
                "heading_rad": theta + math.pi / 2.0}


class Run:
    """loading -> pinning -> warmup -> (contract | smoke) -> done."""

    def __init__(self, results: sf.Results) -> None:
        self.results = results
        self.phase = "loading"
        self.frame = 0
        self.warm = 0
        self.pin_at = None
        self.ctx = omni.usd.get_context()
        self.source: IsaacObservationSource | None = None
        self.walk: _CircuitWalk | None = None
        self.follow_sub = None
        self.robots: dict = {}
        self.thread: threading.Thread | None = None
        self.exit_code: int | None = None
        self.smoke = 0
        self.sub = None

    # -- setup ------------------------------------------------------------
    def setup(self) -> None:
        import avatar as av

        stage = self.ctx.get_stage()
        default_prim = stage.GetDefaultPrim()
        root = default_prim.GetPath().pathString if default_prim else "<none>"
        log(f"stage root: {root} ({len(list(stage.Traverse()))} prims)")

        registry = sf.load_registry()
        self.robots = sf.reference_robots(stage)
        stations = sf.create_stations(stage)
        for path, pos in stations.items():
            log(f"station {path} at {[round(v, 3) for v in pos]}")

        # Annotators and render products ON: this run reads data, which is the
        # whole point. NO collider mask and no minFrameRate -- both are GUI
        # relaxations, both change the physics under test, and this is capture
        # mode (CLAUDE.md rule 6).
        created = sf.create_registry_sensors(stage, registry)
        self.results.write(
            event="sensors_created",
            sensors={k: {"prim_path": v["prim_path"], "kind": v["kind"]}
                     for k, v in created.items()})
        if not created:
            raise RuntimeError("no sensors were created -- nothing to observe")

        self.follow_sub = av.install_character_follow(stage)
        if self.follow_sub is None:
            log("! character follow NOT installed -- the visible body will not "
                "move and every sensor will report a static scene")

        cfg = av.load_avatar_config()
        self.walk = _CircuitWalk(stage, speed=float(cfg.get("move_speed", 1.4)))
        log(f"scripted walk: r={self.walk.radius} m about "
            f"{[round(v, 3) for v in self.walk.centre]} at {self.walk.speed} m/s")

        LIVE.clear()
        LIVE.update(
            stage=stage, registry=registry, created=created,
            kwargs={"settle_frames": SETTLE_FRAMES,
                    "advance_world": self.walk.at,
                    "action_source": self.walk.action},
        )
        self.source = live_source()
        omni.timeline.get_timeline_interface().play()
        log(f"playing; sensors: {', '.join(self.source.sensor_ids)}")

    # -- warm-up ----------------------------------------------------------
    def warmup(self) -> bool:
        """True once every sensor has filled, or the budget is spent.

        Required, not defensive: RTX sensors and Replicator annotators return
        nothing for the first frames after Play and say nothing about it, so a
        run that samples immediately records a scene that had not started yet.
        """
        self.warm += 1
        missing = self.source.missing_payloads()
        if not missing:
            log(f"warm-up complete after {self.warm} frames -- every sensor is live")
            self.results.write(event="warmup", frames=self.warm, missing={})
            return True
        if self.warm >= WARMUP_FRAMES:
            log(f"! warm-up gave up after {self.warm} frames; still empty: {missing}")
            self.results.write(event="warmup", frames=self.warm, missing=missing)
            return True
        if self.warm % 60 == 0:
            log(f"warm-up {self.warm}/{WARMUP_FRAMES}, still empty: {sorted(missing)}")
        return False

    # -- the two modes ----------------------------------------------------
    def start_contract(self) -> None:
        """Run tests/contract.py against this source, on a worker thread."""
        def _run() -> None:
            # No setuptools-entrypoint plugins. Isaac ships ROS 2's
            # `launch_testing`, which registers itself as a pytest11 plugin and
            # imports `lark`, which is not in this image -- so pytest died at
            # startup with ModuleNotFoundError('lark') before collecting a
            # single test. Measured 2026-08-25. Nothing here wants a plugin.
            os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
            import pytest

            # -s, deliberately: pytest's default capture replaces file
            # descriptor 1 for the whole PROCESS, so it would swallow Kit's
            # own logging from the main thread as well -- and if the renderer
            # dies mid-run (it does), the buffered output dies with it. The
            # failure report still prints; everything else stays streamed and
            # fsync'd, which is the rule for this launcher.
            args = [str(REPO / "sim" / "tests" / "test_observation_adapter.py"),
                    "-v", "-s", "--tb=short", "--color=no",
                    "-p", "no:cacheprovider", "--rootdir", str(REPO)]
            log("pytest " + " ".join(args))
            try:
                self.exit_code = int(pytest.main(args))
            except BaseException as exc:                     # noqa: BLE001
                log("pytest could not run: " + repr(exc))
                log(traceback.format_exc())
                self.exit_code = 99

        self.thread = threading.Thread(target=_run, name="contract", daemon=True)
        self.thread.start()

    def smoke_step(self) -> bool:
        """Sample a few ticks on this thread and print what arrived.

        Shapes alone are not a diagnosis: an all-inf depth buffer and a
        working one have the same shape, and the difference decides whether a
        consumer's mean depth is a number or not one. So the numbers that
        would be wrong silently are the ones printed.
        """
        self.smoke += 1
        observations = self.source.step(0.25)
        report = [self._probe(obs) for obs in observations]
        if self.smoke <= 2 or self.smoke == SMOKE_STEPS:
            for line in report:
                log(f"  {line}")
        self.results.write(event="smoke_step", step=self.smoke,
                           t=self.source.time, readings=report)
        return self.smoke >= SMOKE_STEPS

    @staticmethod
    def _probe(obs: Observation) -> dict:
        """summary() plus the values that fail silently. Diagnosis, not logging."""
        out = obs.summary()
        depth = obs.data.get("depth")
        if depth is not None:
            finite = np.isfinite(depth)
            out["depth_finite_frac"] = round(float(finite.mean()), 4)
            if finite.any():
                out["depth_m"] = [round(float(depth[finite].min()), 3),
                                  round(float(depth[finite].max()), 3)]
                out["depth_mean_m"] = round(float(depth[finite].mean()), 3)
        points = obs.data.get("points")
        if points is not None and len(points):
            out["xyz_min"] = [round(float(v), 3) for v in points.min(axis=0)]
            out["xyz_max"] = [round(float(v), 3) for v in points.max(axis=0)]
            out["intrinsics"] = obs.intrinsics
        labels = obs.data.get("semantic_labels")
        if labels:
            out["classes"] = sorted({str(v) for v in labels.values()})[:8]
        return out

    # -- the update pump --------------------------------------------------
    def on_update(self, _event) -> None:
        self.frame += 1
        try:
            if self.phase == "loading":
                status = self.ctx.get_stage_loading_status()
                if self.frame > 5 and not any(status[1:]):
                    log(f"stage loaded after {self.frame} frames")
                    self.setup()
                    self.pin_at = self.frame + 90
                    self.phase = "pinning"
                return

            if self.phase == "pinning":
                # Robots must be kinematic before Play settles them, or their
                # cameras drift and "only the avatar moves" stops being true.
                # Deferred because the Go2's physics lives behind payloads:
                # pinning at reference time finds 0 rigid bodies on it.
                if self.frame >= self.pin_at:
                    if self.robots:
                        sf.pin_robots_static(self.ctx.get_stage(), self.robots)
                    self.phase = "warmup"
                return

            if self.phase == "warmup":
                if self.warmup():
                    if MODE == "smoke":
                        self.phase = "smoke"
                    else:
                        self.start_contract()
                        self.phase = "running"
                return

            if self.phase == "smoke":
                if self.smoke_step():
                    self.finish()
                return

            if self.phase == "running":
                if self.thread is not None and not self.thread.is_alive():
                    self.finish()
                return
        except Exception as exc:
            log("FAILED: " + repr(exc))
            log(traceback.format_exc())
            self.results.write(event="error", error=repr(exc),
                               tb=traceback.format_exc())
            self.finish()

    def finish(self) -> None:
        if self.phase == "done":
            return
        self.phase = "done"
        summary = {"mode": MODE, "frames": self.frame, "warmup_frames": self.warm,
                   "pytest_exit_code": self.exit_code}
        if self.source is not None:
            summary["sensors"] = list(self.source.sensor_ids)
            summary["missing_payloads"] = self.source.missing_payloads()
            self.source.close()
        self.results.write(event="summary", **summary)
        log("SUMMARY " + json.dumps(summary, default=str)[:3000])
        if MODE == "contract":
            log("CONTRACT PASSED" if self.exit_code == 0
                else f"CONTRACT FAILED (pytest exit {self.exit_code})")
        log("DONE")
        self.sub = None
        omni.kit.app.get_app().post_quit()


def main() -> None:
    out = OUT_DIR / f"observation_adapter_{MODE}.jsonl"
    results = sf.Results(out)
    log(f"stage={STAGE} mode={MODE}")
    log(f"results -> {out}")
    results.write(event="start", stage=STAGE, mode=MODE, started=_time.time())

    opened = omni.usd.get_context().open_stage(STAGE)
    ok, err = opened if isinstance(opened, tuple) else (opened, None)
    log(f"open_stage ok={ok} err={err}")
    results.write(event="open_stage", ok=bool(ok), err=str(err))

    run = Run(results)
    run.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        run.on_update, name="observation_adapter")
    log("subscribed to the update stream")


def _exec_entrypoint() -> None:
    """Kit --exec'd this file. Hand over to the IMPORTED copy and drive that.

    ``--exec`` runs the file without registering it in ``sys.modules``, so the
    test module's ``import observation_adapter`` would execute it a second
    time and get a second set of module globals -- including a second, empty
    :data:`LIVE`. Importing it here first means there is exactly one canonical
    module, one rig, and one set of classes; this copy does nothing else.

    OA_NO_AUTORUN is set first so the import is inert. Same reasoning as
    sensor_factory._is_exec_entrypoint, and the same two bad outcomes: too
    strict and the run does nothing while looking fine, too loose and merely
    importing this module opens a stage and post_quit()s the session out from
    under whoever is connected.
    """
    os.environ["OA_NO_AUTORUN"] = "1"
    import observation_adapter as canonical

    canonical.main()


if os.environ.get("OA_NO_AUTORUN") != "1":
    _exec_entrypoint()
