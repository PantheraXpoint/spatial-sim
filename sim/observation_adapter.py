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
``points`` leave here in the **world** frame, in metres -- ``POINTS_FRAME`` in
``core/observation.py``, which this adapter is what pinned. ``core/mock_source``
emitted its clouds sensor-local until 2026-08-25, and the two sources
contradicted each other for as long as they did because no test could see it:
a fixed translation breaks none of ``(N, 3)``, finite, non-empty, or
reacts-to-the-avatar, and for a sensor at the origin the two conventions are
the same numbers. ``test_range_clouds_are_in_the_world_frame`` is the check
that can, and it needed a target of known world position to exist at all.
Every range reading also records its frame under ``intrinsics["frame"]``.

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
    OA_MODE         contract | smoke | freshness    (default: contract)
    OA_OUT          results directory       (default: the logs volume)
    OA_WARMUP       max frames to wait for every sensor to fill (default 300)
    OA_REFRESHES    sensor buffer refreshes a step waits for    (default 2)
    OA_MAX_WAIT     frames to wait for those before giving up   (default 120)
    OA_SETTLE_FRAMES  reproduce the REPLACED fixed wait, for comparison (0)
    OA_STEPS        smoke mode only: how many steps to sample (default 20)
    OA_NO_AUTORUN=1 import this module without running anything

FRESHNESS: WHAT ``step()`` WAITS FOR, AND WHY IT IS NOT A NUMBER OF FRAMES
--------------------------------------------------------------------------
An off-thread ``step()`` advances the world and then **waits until every
sensor that can be tracked has published a new buffer**, twice. It does not
wait a fixed number of frames, and the constant it used to wait -- ``OA_SETTLE``,
default 2 -- is gone.

The reason is measured, not stylistic. The RTX lidar's
``generic-model-output`` buffer changes **once every six application frames**
on this rig, and ``get_data()`` returns the same buffer in between without
saying so: 10 Hz scan, 60 fps application, both read off the buffer's own
header (``sim/spikes/_diag_buffer_clock.py``, 2026-08-26). A fixed settle is
therefore a bet on which phase of that cycle the write lands in, and the
earlier measurement of "a 5 to 10 frame lag" was that bet losing by different
amounts each time rather than a lag that varied. ``OA_SETTLE=2`` never once
cleared a refresh, so **every lidar reading this module produced before
2026-08-26 was up to six frames older than the pose and timestamp stamped
beside it** -- and nothing in the reading said so.

Waiting on the sensor's own counter fixes both halves of that. It is exact:
``frameId`` and ``timestampNs`` tick on publication and on nothing else,
including while the scene is completely still, which is the case a "has the
content changed" test cannot handle at all. And it is scene-independent --
nothing here needs re-tuning when the scan rate, the frame rate or the number
of render products changes, because what is being waited for is not a
duration.

Two refreshes rather than one, because a sweep takes time: the first buffer
published after a change covers an interval that began before it, and can
hold both worlds at once. See ``IsaacObservationSource._on_update``.

Every reading now carries ``intrinsics["freshness"]`` -- refreshes required,
refreshes waited, frames spent, whether the wait completed -- and every range
reading carries the ``frame_id`` of the buffer it came from. A reading that
could not be made fresh says so in the data, not only in a log.

**``tests/contract.py`` cannot see any of this and is not evidence for it.**
It checks shape, dtype, units, frame and that readings react to the avatar;
temporal alignment between a payload and the pose beside it is not among them,
which is why the S11 contract run passed at ``OA_SETTLE=2`` from well inside
the stale window. The evidence is ``OA_MODE=freshness``, which walks the
avatar, steps the source, and counts returns in the box the avatar is in now
against the box it was in one step ago -- under the replaced behaviour and the
new one, in the same run, on the same rig.
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
#: How many buffer REFRESHES a step waits for before it samples. Not frames:
#: see `_RangeReader.version` and the docstring above. Two, because one is not
#: enough and the reason is measured -- a refresh published while a sweep is
#: part-way across the change carries both worlds at once.
SETTLE_REFRESHES = int(os.environ.get("OA_REFRESHES", "2"))
#: Hard ceiling on that wait. A sensor whose buffer never advances -- Play off,
#: render product detached, a dead renderer -- must not hang the caller
#: forever. On this rig a refresh is six frames, so 120 is twenty of them.
MAX_WAIT_FRAMES = int(os.environ.get("OA_MAX_WAIT", "120"))
#: The old fixed wait. Kept, defaulting to OFF, for exactly one purpose: the
#: freshness verification reproduces the behaviour being replaced with it, so
#: the improvement is measured on this rig rather than argued from the change.
LEGACY_SETTLE_FRAMES = int(os.environ.get("OA_SETTLE_FRAMES", "0"))
SMOKE_STEPS = int(os.environ.get("OA_STEPS", "20"))

if os.environ.get("OA_SETTLE") is not None:
    # Refuse to be quietly misconfigured. OA_SETTLE named a fixed frame count
    # and no longer does anything; a caller who set it was asking for a
    # freshness guarantee and would otherwise get a different one in silence.
    print(f"[observation_adapter] ! OA_SETTLE={os.environ['OA_SETTLE']} is "
          f"OBSOLETE and IGNORED. The fixed settle it named was replaced by "
          f"waiting for the sensor buffer to actually change -- see "
          f"OA_REFRESHES (now {SETTLE_REFRESHES}). To reproduce the old "
          f"behaviour for comparison, set OA_SETTLE_FRAMES.", flush=True)

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

    # -- which frame is this buffer? ---------------------------------------
    def version(self):
        """A token that changes exactly when this sensor publishes a new buffer.

        `(frameId, timestampNs)` off the GMO header, or None if neither field
        is there. This is the whole basis of the freshness wait, so what it is
        and what it is NOT both matter.

        MEASURED 2026-08-26, `sim/spikes/_diag_buffer_clock.py`, with the
        scene deliberately held still for 72 frames:

            gmo.frameId       13 distinct values, in runs of exactly 6
            gmo.timestampNs   13 distinct, same runs, stepping 0.1 s each time
            gmo.numElements   13 distinct, same runs
            48 other header fields   never changed at all

        So `frameId` counts APPLICATION FRAMES -- it went 28, 34, 40, six
        apart -- and names the frame the sweep was published on, while
        `timestampNs` is the sensor's own clock and steps 100,000,000 ns per
        refresh. A 10 Hz scan published every six frames is a 60 fps
        application, which is where the six comes from and it is now measured
        rather than inferred.

        **The point of using the header and not the payload:** the scene in
        that run never moved, and the two fields ticked anyway. A "has the
        content changed" test cannot do that -- it cannot tell a buffer that
        has not refreshed from one that refreshed while nothing moved, and a
        source built on it would block forever the first time the world
        happened to be still. It is also free: two header ints against a
        digest of 290,000 points.

        **Two fields that look like this and are not:** `frameStart` and
        `frameEnd` are `FrameAtTime` OBJECTS, so anything that stringifies
        them sees a new memory address every frame and reads as a per-frame
        tick. `scanComplete` and `scanIdx` were 0 on every one of the 72
        frames. Neither is a refresh counter.
        """
        from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

        if self.sensor is None:
            return None
        try:
            got = self.sensor.get_data("generic-model-output")
            buf = got[0] if isinstance(got, tuple) else got
            if buf is None:
                return None
            gmo = parse_generic_model_output_data(buf)
            if gmo is None:
                return None
        except Exception as exc:                                  # noqa: BLE001
            self._warn_once("version", f"! {self.sensor_id}: could not read a "
                                       f"buffer version: {exc!r}")
            return None
        token = []
        for name in ("frameId", "timestampNs"):
            try:
                token.append(int(getattr(gmo, name)))
            except Exception:                                     # noqa: BLE001
                continue
        if not token:
            self._warn_once(
                "noversion",
                f"! {self.sensor_id}: the buffer carries neither frameId nor "
                f"timestampNs, so its refreshes cannot be counted and step() "
                f"will not wait for it. Readings from it may be older than "
                f"the pose stamped beside them.")
            return None
        return tuple(token)

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
            # WHICH buffer this is, carried out with the payload. The reading
            # could say what its pose was and when it was asked for, and not
            # which sweep it came from -- so two readings a caller believed
            # were different could be one buffer handed out twice, and nothing
            # in either of them said so.
            "frame_id": self._enum(gmo, "frameId", -1),
            "timestamp_ns": self._enum(gmo, "timestampNs", -1),
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
    """One off-thread ``step()`` waiting for the main loop to service it.

    Carries the freshness bookkeeping because the wait is per REQUEST, not per
    source: what a step is waiting for is that every trackable sensor has
    published a new buffer since the world was advanced for THIS step.
    """

    __slots__ = ("advanced", "baseline", "done", "error", "frames",
                 "max_frames", "refreshes", "result", "seen", "t", "timed_out")

    def __init__(self, t: float, refreshes: int, max_frames: int) -> None:
        self.t = t
        self.refreshes = refreshes
        self.max_frames = max_frames
        self.advanced = False
        #: sensor_id -> the version token that sensor last showed
        self.baseline: dict[str, Any] = {}
        #: sensor_id -> how many times it has changed since the advance
        self.seen: dict[str, int] = {}
        self.frames = 0
        self.timed_out = False
        self.result: list[Observation] | None = None
        self.error: BaseException | None = None
        self.done = threading.Event()

    def freshness(self) -> dict:
        """What this step actually waited for, in a form a reading can carry."""
        return {
            "refreshes_required": self.refreshes,
            "refreshes_waited": (min(self.seen.values()) if self.seen else 0),
            "frames_waited": self.frames,
            "wait_complete": not self.timed_out,
            "tracked_sensors": sorted(self.seen),
        }


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
        settle_refreshes: int = SETTLE_REFRESHES,
        settle_frames: int = LEGACY_SETTLE_FRAMES,
        max_wait_frames: int = MAX_WAIT_FRAMES,
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
        self._refreshes = max(0, int(settle_refreshes))
        self._settle = max(0, int(settle_frames))
        self._max_wait = max(1, int(max_wait_frames))
        #: What the last serviced step waited for. Read by drivers that report
        #: on freshness; never used to make a decision here.
        self.last_wait: dict = {}
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
            # No wait is possible here and none is claimed. This call is
            # already inside an update callback: blocking for a refresh would
            # block the loop that produces refreshes. The readings say
            # `waited: false` rather than carrying the freshness of a wait
            # that did not happen.
            return self._read_all(freshness={
                "refreshes_required": self._refreshes,
                "refreshes_waited": 0,
                "frames_waited": 0,
                "wait_complete": False,
                "why": "sampled on Kit's main thread, where step() cannot block",
            })
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
        request = _Request(self._t, self._refreshes, self._max_wait)
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
        """Main thread. Services at most one pending step per frame.

        WAITS FOR THE BUFFER TO CHANGE, not for a number of frames. The
        difference is the point of this method.

        A fixed settle has to be chosen against the slowest sensor in the
        scene and is wrong in both directions at once: too small and the
        payload describes the world before the step's own action -- measured
        2026-08-26, an RTX lidar keeps handing back the same buffer for six
        application frames and `get_data()` says nothing about it -- and too
        large and every tick of every trace pays for the worst case. It is
        also not a property of anything: the right number changes with the
        scan rate, the frame rate and how many render products are up.

        Waiting on the sensor's own refresh counter is none of those. It is
        exact, it costs what it needs to and no more, and it moves with the
        rig instead of having to be re-tuned when the rig changes.

        WHY TWO REFRESHES AND NOT ONE. A rotary lidar publishes a sweep, and a
        sweep takes time. Advance the world at application frame T and the
        next buffer covers an interval that STARTED before T -- so it holds
        the old world and the new one at once, which is exactly the state the
        object-motion spike caught: 7 returns at the new position for one
        refresh, then 849 at the next. The second refresh is the first whose
        whole interval is after T. One is enough only when the advance happens
        to land on a publication boundary, and nothing here controls the
        phase.

        Cameras are not waited on, and that is measured rather than assumed:
        their annotator buffers changed on every one of 72 consecutive frames
        with the scene held still, so they carry no cadence to wait for. A
        sensor whose version token is None is likewise not waited on -- it is
        reported once, loudly, by `_RangeReader.version`.
        """
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
                # The baseline is taken AFTER the advance, in the same
                # callback. Any refresh counted from here was published by a
                # render that ran after the world moved; one taken before the
                # advance could be satisfied by a buffer that predates it.
                request.baseline = self._versions()
                request.seen = {k: 0 for k in request.baseline}
                if not request.baseline:
                    self._warn_missing(
                        "step", "no sensor exposes a refresh counter -- step() "
                        "cannot wait for fresh data and will sample "
                        "immediately")
                return

            if self._settle > 0 and request.frames < self._settle:
                # The replaced behaviour, off by default. Only the freshness
                # verification turns it on, and only to measure what it used
                # to do.
                request.frames += 1
                return

            request.frames += 1
            current = self._versions()
            for sensor_id, token in current.items():
                if sensor_id in request.baseline and token != request.baseline[sensor_id]:
                    request.baseline[sensor_id] = token
                    request.seen[sensor_id] = request.seen.get(sensor_id, 0) + 1

            if request.seen and min(request.seen.values()) < request.refreshes:
                if request.frames < request.max_frames:
                    return
                request.timed_out = True
                self._warn_missing(
                    "wait",
                    f"waited {request.frames} frames and "
                    f"{ {k: v for k, v in request.seen.items()} } refreshes "
                    f"arrived of the {request.refreshes} needed -- sampling "
                    f"anyway. The readings say so in "
                    f"intrinsics['freshness']; they are not necessarily "
                    f"newer than the pose beside them.")

            self.last_wait = request.freshness()
            request.result = self._read_all(freshness=self.last_wait)
        except BaseException as exc:                        # noqa: BLE001
            request.error = exc
        with self._lock:
            self._request = None
        request.done.set()

    def _versions(self) -> dict:
        """Every trackable sensor's current buffer token. Main thread."""
        out = {}
        for sensor_id, reader in self._range.items():
            token = reader.version()
            if token is not None:
                out[sensor_id] = token
        return out

    # --- reading -------------------------------------------------------------
    def _read_all(self, freshness: dict | None = None) -> list[Observation]:
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
            if freshness is not None:
                # The reading carries what its step waited for. Before this,
                # an Observation could say what its pose was and when it was
                # asked for and nothing at all about whether its payload had
                # caught up -- which is precisely how a cloud five frames
                # behind the pose beside it went unnoticed for a week.
                intrinsics = dict(intrinsics or {})
                intrinsics["freshness"] = dict(freshness)
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

    USD writes, not physics -- but NOT because nothing else is writing. This
    paragraph used to say that the driver runs without ``--enable
    omni.physx.cct``, so the node type is unregistered, the Controls graph
    loads and does nothing, and nothing contends for the capsule's transform.
    **The last clause is false under this launcher.** Measured 2026-08-26:
    ``runheadless.sh`` starts ``omni.physx.cct`` on its own -- it is in the
    extension startup log of a run whose command line carried no ``--enable``
    at all -- and the capsule's authored z of 0.925 becomes 0.895 within the
    first frames of Play, which is a controller's resting height and not a
    coincidence. A character controller is live under this walk. It always
    was.

    What that does and does not change (see sim/spikes/FINDINGS.md):

      * It changes **nothing the contract asserts**. No check in
        tests/contract.py depends on which agency moved the capsule; they are
        about payload shape, dtype, units, frame, and that readings react.
      * It retracts the *reason* this walk was called safe. It was contended
        and won anyway, rather than by design.
      * ``__init__`` reads the capsule's z once, before ``play()``, so it
        captures the authored 0.925 and re-asserts it every tick while physics
        pulls the controller to 0.895 -- a ~3 cm disagreement, refreshed every
        frame. Which value the renderer sees was not measured.
      * The caveat below is unchanged and now sharper: a controller was there,
        and a USD transform write went straight through it.

    Still a scripted walk and still not CCT collide-and-slide -- it says
    nothing about what the character controller does when you walk into a
    shelf. ``avatar.set_avatar_pose()`` is the deliberate version of this same
    write, for episode reset, and carries the same caveat.

    The visible character follows via ``avatar.install_character_follow``, and
    it is the character -- render geometry -- that lidar and cameras actually
    see.

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
        self.fresh: list[dict] = []
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
            kwargs={"settle_refreshes": SETTLE_REFRESHES,
                    "settle_frames": LEGACY_SETTLE_FRAMES,
                    "max_wait_frames": MAX_WAIT_FRAMES,
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

    # -- the freshness verification ---------------------------------------
    #: How far the avatar walks between steps in freshness mode. Chosen so the
    #: two boxes are disjoint rather than by feel: 1.2 s at 1.4 m/s on a
    #: 2.5 m circle is a 1.65 m chord, and the boxes are 1.2 m across.
    FRESH_DT = 1.2
    FRESH_STEPS = 8
    #: Half-width of the box a step's returns are counted in, and the band it
    #: spans. Same numbers as sim/verify_avatar_pose.py, so the counts here
    #: and there mean the same thing.
    FRESH_BOX_HALF = 0.60
    FRESH_BOX_Z = (0.30, 1.80)

    def start_freshness(self) -> None:
        """Measure whether a step's payload describes the world that step asked for.

        THE evidence for replacing the settle constant, and it exists because
        `tests/contract.py` cannot be that evidence: it checks shape, dtype,
        units, frame and reactivity, and a payload six frames behind the pose
        beside it satisfies every one of them. So this asks the one question
        the contract does not.

        The avatar walks a circle. After each `step(dt)` the lidar's cloud is
        counted in the box the avatar is in NOW against the box it was in one
        step ago. A source that returns fresh data puts the returns in the
        first box; a source that hands back whatever buffer happened to be
        sitting there puts them in the second, and the reading looks
        completely normal either way -- right density, right extent, genuinely
        on a body, just not this step's body.

        Both behaviours run in the same session on the same rig: the fixed
        wait that was replaced, then the refresh wait that replaced it. One
        arm is not a measurement of the other's absence.
        """
        def _run() -> None:
            try:
                self.fresh = [self._freshness_arm(*arm) for arm in (
                    ("replaced: fixed 2-frame settle",
                     {"settle_frames": 2, "settle_refreshes": 0}),
                    (f"new: wait for {SETTLE_REFRESHES} buffer refreshes",
                     {"settle_frames": 0, "settle_refreshes": SETTLE_REFRESHES}),
                )]
                self.exit_code = 0 if self._freshness_verdict() else 1
            except BaseException as exc:                     # noqa: BLE001
                log("freshness run failed: " + repr(exc))
                log(traceback.format_exc())
                self.exit_code = 98

        self.thread = threading.Thread(target=_run, name="freshness", daemon=True)
        self.thread.start()

    def _box_at(self, xy):
        h = self.FRESH_BOX_HALF
        return (Gf.Vec3d(xy[0] - h, xy[1] - h, self.FRESH_BOX_Z[0]),
                Gf.Vec3d(xy[0] + h, xy[1] + h, self.FRESH_BOX_Z[1]))

    def _freshness_arm(self, name: str, kwargs: dict) -> dict:
        """One arm: N steps, and where each step's returns actually landed."""
        log(f"--- freshness arm: {name} ---")
        source = live_source(**kwargs)
        lidar_id = next((sid for sid in source.sensor_ids
                         if source.registry.get(sid).modality in RANGE_MODALITIES
                         and source.registry.get(sid).mount is MountType.FIXED), None)
        rows: list[dict] = []
        try:
            if lidar_id is None:
                return {"name": name, "kwargs": kwargs, "rows": [],
                        "why": "no FIXED range sensor in the registry"}
            previous = self.walk.position(0.0)
            for _ in range(self.FRESH_STEPS):
                observations = source.step(self.FRESH_DT)
                here = self.walk.position(source.time)
                obs = next((o for o in observations if o.sensor_id == lidar_id), None)
                points = None if obs is None else obs.data.get("points")
                arr = (np.asarray(points)
                       if points is not None and len(points) else None)
                intr = (obs.intrinsics or {}) if obs is not None else {}
                fresh = intr.get("freshness") or {}
                rows.append({
                    "t": round(source.time, 3),
                    "here": [round(v, 3) for v in here],
                    "there": [round(v, 3) for v in previous],
                    "returns_here": (0 if arr is None else
                                     sf.count_in_box(arr, *self._box_at(here), pad=0.0)),
                    "returns_there": (0 if arr is None else
                                      sf.count_in_box(arr, *self._box_at(previous), pad=0.0)),
                    "frame_id": intr.get("frame_id"),
                    "frames_waited": fresh.get("frames_waited"),
                    "refreshes_waited": fresh.get("refreshes_waited"),
                    "wait_complete": fresh.get("wait_complete"),
                })
                log(f"  t={rows[-1]['t']:>5}  here={rows[-1]['returns_here']:>5}  "
                    f"there={rows[-1]['returns_there']:>5}  "
                    f"frame_id={rows[-1]['frame_id']}  "
                    f"waited {rows[-1]['frames_waited']} frames / "
                    f"{rows[-1]['refreshes_waited']} refreshes")
                previous = here
        finally:
            source.close()
        arm = {"name": name, "kwargs": kwargs, "rows": rows}
        self.results.write(event="freshness_arm", **arm)
        return arm

    def _freshness_verdict(self) -> bool:
        """Print both arms side by side and decide. True == the new behaviour holds."""
        print("\n" + "=" * 78, flush=True)
        print("DOES A STEP'S PAYLOAD DESCRIBE THE WORLD THAT STEP ASKED FOR?", flush=True)
        print("=" * 78, flush=True)
        ok = True
        for arm in self.fresh:
            rows = arm["rows"]
            if not rows:
                print(f"  {arm['name']}: {arm.get('why', 'no steps')}", flush=True)
                continue
            correct = sum(1 for r in rows if r["returns_here"] > r["returns_there"])
            frames = [r["frames_waited"] for r in rows if r["frames_waited"] is not None]
            print(f"\n  {arm['name']}", flush=True)
            print(f"    {'t':>6}{'here':>8}{'there':>8}{'frame_id':>10}"
                  f"{'frames':>8}{'refresh':>9}", flush=True)
            for r in rows:
                print(f"    {r['t']:>6}{r['returns_here']:>8}{r['returns_there']:>8}"
                      f"{r['frame_id']!s:>10}{r['frames_waited']!s:>8}"
                      f"{r['refreshes_waited']!s:>9}", flush=True)
            print(f"    steps whose returns are at the CURRENT position: "
                  f"{correct}/{len(rows)}; frames waited "
                  f"{min(frames) if frames else '-'}..{max(frames) if frames else '-'}",
                  flush=True)
            arm["correct"] = correct
            arm["steps"] = len(rows)
            arm["frames_waited_range"] = [min(frames), max(frames)] if frames else None
            # Only the NEW arm is required to hold. The replaced one is
            # measured, not asserted: it is here to show what was being
            # replaced, and a run where it happened to get lucky on phase is
            # not a failure of anything.
            if arm["kwargs"].get("settle_refreshes"):
                ok = ok and correct == len(rows)
        print("\n" + "=" * 78, flush=True)
        new = next((a for a in self.fresh if a["kwargs"].get("settle_refreshes")), None)
        old = next((a for a in self.fresh if not a["kwargs"].get("settle_refreshes")), None)
        if new and old and new.get("steps"):
            print(f"  replaced behaviour: {old.get('correct')}/{old.get('steps')} steps "
                  f"current    new behaviour: {new.get('correct')}/{new.get('steps')} "
                  f"steps current", flush=True)
        print(f"  FRESHNESS {'HOLDS' if ok else 'DOES NOT HOLD'}", flush=True)
        print("=" * 78, flush=True)
        self.results.write(event="freshness_verdict", ok=ok, arms=[
            {k: v for k, v in a.items() if k != "rows"} for a in self.fresh])
        return ok

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
                    elif MODE == "freshness":
                        self.start_freshness()
                        self.phase = "running"
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
        if MODE == "freshness":
            log("FRESHNESS PASSED" if self.exit_code == 0
                else f"FRESHNESS FAILED (exit {self.exit_code})")
        log("DONE")
        self.sub = None
        omni.kit.app.get_app().post_quit(self.exit_code or 0)


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
