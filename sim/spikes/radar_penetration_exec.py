"""S4 radar spike, EXEC MODE — does RTX radar see through what stops lidar?

Same experiment as ``radar_penetration.py``, restructured for the only
execution model in which sensor readback actually works on this host.

Why exec mode
-------------
Offscreen capture is dead under ``SimulationApp`` and works under
``runheadless.sh``. Measured, same scene, same GPU:

    SimulationApp     camera rgb 0 px        lidar GMO 0 points
    runheadless.sh    camera rgb 32,700 px   lidar GMO 460,800 points

So this script must NOT construct a ``SimulationApp`` — Kit is already running
— and must drive frames from the update event stream rather than calling
``app.update()`` in a loop, which would re-enter the main loop.

Run it as:

    ./runheadless.sh \
        --/renderer/raytracingMotion/enabled=true \
        --/renderer/raytracingMotion/enableHydraEngineMasking=true \
        --/renderer/raytracingMotion/enabledForHydraEngines="'0'" \
        --exec /workspace/sim/spikes/radar_penetration_exec.py

Motion BVH must be on the COMMAND LINE, not set from here: the BVH is built at
renderer init, and by the time this script runs that has already happened.
All three settings are required. ``Radar._create_prim`` only raises on the
first, so passing that one alone yields a radar that constructs cleanly and
returns nothing — the silent-failure class this project keeps hitting. This
script therefore asserts all three at runtime and aborts if any is missing,
rather than inferring success from the absence of an exception.

Configuration comes from the environment, not argv, because Kit's
``--exec SCRIPT ARGS...`` makes trailing-argument parsing ambiguous:

    SPIKE_MATERIALS  controls | smoke | all | comma-separated list
    SPIKE_FRAMES     frames accumulated per condition (default 30)
    SPIKE_WARMUP     frames discarded after each scene change (default 15)
    SPIKE_SENSORS    both | lidar | radar   (default both)
    SPIKE_OUT        results path; MUST be container-writable (uid 1234)
    SPIKE_RADAR_ATTRS  JSON dict of OmniRadar prim attributes applied at
                     creation, e.g. {"omni:sensor:WpmDmat:cfarMode": "4D"}

Reading the buffer
------------------
`generic-model-output` is NOT a point cloud. Its per-element x/y/z are azimuth
degrees, elevation degrees and range metres whenever `elementsCoordsType ==
SPHERICAL`, which is the schema DEFAULT for both sensors, and they are
sensor-local because `frameOfReference` defaults to SENSOR. An earlier version
of this script read those three arrays as Cartesian metres. The resulting masks
decoded to "azimuth 6-10.5 deg AND range under 2.5 m", which nothing in the
scene can satisfy, so every target count was a guaranteed zero; and the wall
count was a fixed 2 deg slice of solid angle, which a wall at 1 km satisfies as
happily as one at 4 m. `_to_world()` now reads both conventions out of the
buffer header and converts, and `_report()` refuses to answer unless the wall
mask demonstrably tells 4 m from 1 km.

``SPIKE_SENSORS`` exists to isolate the CUDA-interop crash: with Motion BVH on
and both sensors present, the renderer collapses with
``cudaErrorIllegalAddress``. Running one sensor at a time separates "Motion BVH
alone is fatal" from "radar is fatal" from "two RTX render products at once are
fatal". A sensor that is not selected is never created, so its render product
never exists -- omitting it must not merely skip its readback.

Results are appended per condition and fsync'd, not written once at the end,
so a killed container loses at most the condition in flight.
"""

from __future__ import annotations

import json
import math
import os

import carb
import numpy as np
import omni.kit.app
import omni.replicator.core as rep  # noqa: F401  (ensures Replicator is up)
import omni.timeline

FRAMES = int(os.environ.get("SPIKE_FRAMES", "30"))
WARMUP = int(os.environ.get("SPIKE_WARMUP", "15"))
MATERIALS = os.environ.get("SPIKE_MATERIALS", "controls")
SENSORS_MODE = os.environ.get("SPIKE_SENSORS", "both").strip().lower()
if SENSORS_MODE not in ("both", "lidar", "radar"):
    raise SystemExit(f"SPIKE_SENSORS must be both|lidar|radar, got {SENSORS_MODE!r}")
USE_RADAR = SENSORS_MODE in ("both", "radar")
USE_LIDAR = SENSORS_MODE in ("both", "lidar")
# Radar prim attributes, as JSON, applied at creation. The generic WpmDmat
# radar's own schema defaults are what produced ~1 detection per frame, and
# several of them are the knob for that -- `omni:sensor:WpmDmat:cfarMode`
# above all ("2D" by default, "4D" allowed). Attribute names are the full USD
# attribute paths; NVIDIA's own test passes them exactly this way.
RADAR_ATTRS = json.loads(os.environ.get("SPIKE_RADAR_ATTRS", "{}"))
# Default lives in the logs VOLUME: /workspace is the bind mount, owned by uid
# 1004, and this container runs as 1234 -- writes there fail silently.
OUT = os.environ.get("SPIKE_OUT", "/isaac-sim/.nvidia-omniverse/logs/radar_spike.jsonl")

_app = omni.kit.app.get_app()
_s = carb.settings.get_settings()


def _emit(record: dict) -> None:
    """Append one result and force it to disk immediately."""
    with open(OUT, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(record) + "\n")
        fh.flush()
        os.fsync(fh.fileno())


def _abort(reason: str) -> None:
    print(f"\n!! ABORT: {reason}", flush=True)
    _emit({"kind": "abort", "reason": reason})
    _app.post_quit()


print("\n" + "=" * 78, flush=True)
print("S4 RADAR PENETRATION SPIKE -- EXEC MODE", flush=True)
print("=" * 78, flush=True)
print(f"  materials={MATERIALS}  frames={FRAMES}  warmup={WARMUP}  sensors={SENSORS_MODE}", flush=True)
print(f"  out={OUT}", flush=True)

# --------------------------------------------------------------------------
# MOTION BVH -- assert, do not assume
# --------------------------------------------------------------------------
_bvh = {
    "/renderer/raytracingMotion/enabled": _s.get("/renderer/raytracingMotion/enabled"),
    "/renderer/raytracingMotion/enableHydraEngineMasking": _s.get(
        "/renderer/raytracingMotion/enableHydraEngineMasking"
    ),
    "/renderer/raytracingMotion/enabledForHydraEngines": _s.get(
        "/renderer/raytracingMotion/enabledForHydraEngines"
    ),
}
print("\n=== MOTION BVH ===", flush=True)
for k, v in _bvh.items():
    print(f"  {k} = {v!r}", flush=True)

_missing = [k for k, v in _bvh.items() if v in (None, False, "", [])]
if _missing:
    _abort(
        "Motion BVH is not active. Missing/false: "
        + ", ".join(_missing)
        + ". Pass all three on the kit command line -- the BVH is built at "
        "renderer init and cannot be enabled from here."
    )
    raise SystemExit(0)

_emit(
    {
        "kind": "config",
        "motion_bvh": _bvh,
        "frames": FRAMES,
        "warmup": WARMUP,
        "materials": MATERIALS,
        "sensors": SENSORS_MODE,
        "radar_attrs": RADAR_ATTRS,
    }
)

# --------------------------------------------------------------------------
# SCENE  (identical geometry to radar_penetration.py)
#
#   sensors           wall                 target
#   x=0, z=1          x=4                  x=8 (oscillating +/-0.5 in x)
#      |               |                     |
#      +-------------->|############        [] <- 2 m cube, steel
#                      |############
#
# The wall is 12 m wide and runs from below the floor to 4 m up. The target
# subtends ~+/-7 deg of azimuth; the wall covers ~+/-56 deg. The target is
# fully shadowed with a large margin, so the result does not hinge on getting
# the wall's extent exactly right.
# --------------------------------------------------------------------------
from isaacsim.core.experimental.materials import NonVisualMaterial  # noqa: E402
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane  # noqa: E402
from isaacsim.core.experimental.utils.app import enable_extension  # noqa: E402

enable_extension("isaacsim.sensors.rtx.nodes")

from isaacsim.sensors.experimental.rtx import (  # noqa: E402
    Lidar,
    LidarSensor,
    Radar,
    RadarSensor,
    parse_generic_model_output_data,
)

SENSOR_Z = 1.0
WALL_X, TARGET_X = 4.0, 8.0
FAR_AWAY = 1000.0
WALL_POS = np.array([WALL_X, 0.0, 1.5])
WALL_SCALE = np.array([0.1, 12.0, 4.0])
TARGET_POS = np.array([TARGET_X, 0.0, SENSOR_Z])
TARGET_SCALE = np.array([2.0, 2.0, 2.0])

MOTION_AMPLITUDE, MOTION_PERIOD = 0.5, 20.0

# --------------------------------------------------------------------------
# TRUE WORLD EXTENTS
#
# `UsdGeom.Cube` has size = 2 by default and this script never calls
# `set_sizes`, so `scales` multiplies a 2 m edge: the half-extent is the scale
# itself, not half of it. Getting this wrong by a factor of two is what makes a
# "2 m cube" out of a 4 m one, so it is written out rather than inferred:
#
#   wall    centre (4, 0, 1.5)  half (0.1, 12, 4)  ->  x 3.90..4.10
#   target  centre (8, 0, 1)    half (2,   2,  2)  ->  x 6.00..10.00, z -1..3
#
# The wall's near face at x = 3.90 is the number the radar already reported as
# 3.9002 m under the old (mis-decoded) run. That agreement is the reason to
# trust this arithmetic.
# --------------------------------------------------------------------------
WALL_FACE_X = WALL_X - WALL_SCALE[0]        # 3.90
TARGET_FACE_X = TARGET_X - TARGET_SCALE[0]  # 6.00

# Boxes in WORLD metres. Every one of these is a real volume you could point at
# in the viewport -- which is the whole difference from the version they
# replace, where `wmask` was a 2 deg azimuth wedge of infinite length.
WALL_BOX = {"x": (3.5, 4.5), "y": 12.5, "z": (0.20, 5.6)}
# The z floor is not cosmetic. The ground plane runs through this box, so a box
# reaching down to z = 0 collects a ~559k-point strip of FLOOR whether the wall
# is at 4 m or at 1 km -- measured, lidar, 2026-08-14. That floor made the
# discrimination gate below unpassable by construction. The wall spans
# z -2.5..5.5, so excluding everything under 0.20 m costs a sliver of wall and
# removes the entire confound.
TARGET_BOX = {"x": (5.5, 10.5), "y": 2.5, "z": (0.20, 3.5)}
# The target straddles the ground (z from -1). A box that reaches down to the
# floor counts ground returns as target hits -- the exact artifact the old
# comment warned about and the old mask could not avoid. The z floor at 0.20 m
# excludes the ground plane; `target_points_with_ground` below keeps the
# unfiltered count so the size of that contamination stays visible.
GROUND_ABS_Z = 0.10  # |world z| under this == a hit on the ground plane

# ElementFlags.VALID. Confirmed from the shipped stub
# (generic_model_output/_rtx_sensors_gmo.pyi): VALID = 64, and the shipped
# docs note it is currently the only flag any modality sets.
FLAG_VALID = 64
COORDS_CARTESIAN, COORDS_SPHERICAL = 0, 1
FRAME_SENSOR, FRAME_WORLD = 0, 1

# Both sensors are created below at (0, 0, SENSOR_Z) with identity orientation,
# so sensor->world is a pure translation. This is not an assumption about the
# scene -- it is the two literals passed to the constructors, asserted at the
# point of use.
SENSOR_ORIGIN_W = np.array([0.0, 0.0, SENSOR_Z])

SMOKE = ["steel", "concrete", "cardboard", "plastic", "clear_glass", "fabric"]

DistantLight("/World/light").set_intensities(3000.0)
GroundPlane("/World/ground")

target = Cube("/World/target", positions=TARGET_POS, scales=TARGET_SCALE, colors=[0.9, 0.1, 0.1])
target_mat = NonVisualMaterial("/World/target/material", bases="steel", coatings="none", attributes="none")
target.apply_visual_materials(target_mat)

occluder = Cube("/World/occluder", positions=WALL_POS, scales=WALL_SCALE, colors=[0.6, 0.6, 0.6])
occluder_mat = NonVisualMaterial(
    "/World/occluder/material", bases="concrete", coatings="none", attributes="none"
)
occluder.apply_visual_materials(occluder_mat)

# Identity orientation: RTX range sensors look down local +X.
# A deselected sensor is not created at all -- no prim, no render product. That
# is the point of the isolation test; merely skipping its readback would leave
# the render product live and measure nothing.
radar_sensor = None
lidar_sensor = None
_created = []

if USE_RADAR:
    radar = Radar(
        "/World/radar",
        translations=np.array([0.0, 0.0, SENSOR_Z]),
        orientations=np.array([1.0, 0.0, 0.0, 0.0]),
        aux_output_level="BASIC",  # required for rv_ms; radar allows NONE|BASIC only
        attributes=RADAR_ATTRS or None,
    )
    radar_sensor = RadarSensor(radar, annotators=["generic-model-output"])
    _created.append(f"radar {radar.paths[0]}")

if USE_LIDAR:
    lidar = Lidar.create(
        "/World/lidar",
        config="Example_Rotary",
        translations=np.array([0.0, 0.0, SENSOR_Z]),
        orientations=np.array([1.0, 0.0, 0.0, 0.0]),
        aux_output_level="FULL",
    )
    lidar_sensor = LidarSensor(lidar, annotators=["generic-model-output"])
    _created.append(f"lidar {lidar.paths[0]}")

print("\ncreated " + ", ".join(_created), flush=True)

omni.timeline.get_timeline_interface().play()


# --------------------------------------------------------------------------
# ACCUMULATION
# --------------------------------------------------------------------------
def _new_acc() -> dict:
    return {
        "frames_with_data": 0,
        "raw_points": 0,  # numElements, before the VALID mask
        "total_points": 0,  # after the VALID mask -- what everything else counts
        "target_points": 0,
        "target_points_with_ground": 0,
        "wall_points": 0,
        "ground_points": 0,
        "_wi_sum": 0.0,
        "_wi_n": 0,
        # Extents of the returned cloud. Without these a zero target count is
        # unreadable: "the wall blocked it" and "my box is in the wrong frame
        # or the wrong units" produce the same 0. Kept in BOTH conventions now,
        # because reading one as the other is precisely what went wrong before.
        "_ext": None,  # world Cartesian metres
        "_sph": None,  # azimuth deg / elevation deg / range m
        # Header provenance. Never inferred: the buffer says what convention it
        # is in, and every run now records what it said.
        "coords_type": None,
        "frame_of_ref": None,
        "flag_values": set(),
        "_rv": None,  # radial velocity min/max, valid returns only
        "_rv_moving": 0,  # |rv| > RV_MOVING_MPS
        "samples": [],  # a few raw detections, verbatim
    }


ACC = {"radar": _new_acc(), "lidar": _new_acc()}
COLLECTING = {"on": False}
SENSORS = tuple(
    (kind, s)
    for kind, s in (("radar", radar_sensor), ("lidar", lidar_sensor))
    if s is not None
)


RV_MOVING_MPS = 0.1  # above this a return is Doppler-distinguishable from clutter
MAX_SAMPLES = 12


def _to_world(gmo, n: int):
    """GMO basic elements -> (N, 3) Cartesian metres in the WORLD frame.

    Three steps stand between `generic-model-output` and a metric point cloud,
    and the old code took none of them:

      1. `elementsCoordsType` -- SPHERICAL by DEFAULT, in which case x and y
         are DEGREES of azimuth and elevation and z is a RANGE in metres.
      2. `frameOfReference` -- SENSOR by DEFAULT, so the result is sensor-local
         and needs the sensor pose applied.
      3. `flags & VALID` -- returned separately by this function.

    Both conventions are read from the buffer header rather than assumed, so
    this stays correct if the prim attributes are changed to CARTESIAN/WORLD.
    Anything unexpected raises: a wrong convention must not degrade to a
    plausible-looking cloud.

    Returns:
        (points_world, azimuth_deg, elevation_deg, range_m, valid_mask)
    """
    xa = np.asarray(gmo.x, dtype=np.float64).ravel()[:n]
    ya = np.asarray(gmo.y, dtype=np.float64).ravel()[:n]
    za = np.asarray(gmo.z, dtype=np.float64).ravel()[:n]

    coords = int(gmo.elementsCoordsType)
    if coords == COORDS_SPHERICAL:
        az, el, rng = xa, ya, za
        ce = np.cos(np.radians(el))
        p = np.stack(
            [rng * ce * np.cos(np.radians(az)), rng * ce * np.sin(np.radians(az)), rng * np.sin(np.radians(el))],
            axis=1,
        )
    elif coords == COORDS_CARTESIAN:
        p = np.stack([xa, ya, za], axis=1)
        rng = np.linalg.norm(p, axis=1)
        az = np.degrees(np.arctan2(ya, xa))
        el = np.degrees(np.arcsin(np.divide(za, rng, out=np.zeros_like(za), where=rng > 0)))
    else:
        raise RuntimeError(f"unhandled elementsCoordsType {coords}")

    frame = int(gmo.frameOfReference)
    if frame == FRAME_SENSOR:
        p = p + SENSOR_ORIGIN_W
    elif frame != FRAME_WORLD:
        raise RuntimeError(f"unhandled frameOfReference {frame}; expected SENSOR or WORLD")

    flags = np.asarray(gmo.flags).ravel()[:n]
    valid = (flags.astype(np.int64) & FLAG_VALID) != 0
    return p, az, el, rng, valid, flags


def _sample() -> None:
    for kind, sensor in SENSORS:
        try:
            buf, _ = sensor.get_data("generic-model-output")
        except Exception:
            continue
        if buf is None:
            continue
        gmo = parse_generic_model_output_data(buf)
        if gmo is None:
            continue
        n = int(gmo.numElements)
        if n <= 0:
            continue
        n = min(n, len(np.asarray(gmo.x).ravel()))

        p, az, el, rng, valid, flags = _to_world(gmo, n)

        rec = ACC[kind]
        rec["frames_with_data"] += 1
        rec["raw_points"] += n
        rec["coords_type"] = int(gmo.elementsCoordsType)
        rec["frame_of_ref"] = int(gmo.frameOfReference)
        rec["flag_values"].update(int(v) for v in np.unique(flags))

        nv = int(valid.sum())
        rec["total_points"] += nv
        if nv == 0:
            continue
        p, az, el, rng = p[valid], az[valid], el[valid], rng[valid]
        wx, wy, wz = p[:, 0], p[:, 1], p[:, 2]

        for key, arrs in (("_ext", (wx, wy, wz)), ("_sph", (az, el, rng))):
            ext = [[float(a.min()), float(a.max())] for a in arrs]
            if rec[key] is None:
                rec[key] = ext
            else:
                for a in range(3):
                    rec[key][a][0] = min(rec[key][a][0], ext[a][0])
                    rec[key][a][1] = max(rec[key][a][1], ext[a][1])

        # --- the masks, now real volumes in world metres --------------------
        gmask = np.abs(wz) < GROUND_ABS_Z
        rec["ground_points"] += int(gmask.sum())

        in_xy = (
            (wx > TARGET_BOX["x"][0])
            & (wx < TARGET_BOX["x"][1])
            & (np.abs(wy) < TARGET_BOX["y"])
            & (wz < TARGET_BOX["z"][1])
        )
        rec["target_points_with_ground"] += int((in_xy & (wz > -1.5)).sum())
        rec["target_points"] += int((in_xy & (wz > TARGET_BOX["z"][0])).sum())

        # A BOX, not a wedge. The old `3 < azimuth < 5 deg` had no range bound
        # at all, so it counted a fixed slice of solid angle and could not tell
        # a wall at 4 m from one at 1 km. This cannot see the far one: at
        # x = 1000 nothing lands in x 3.5..4.5.
        wmask = (
            (wx > WALL_BOX["x"][0])
            & (wx < WALL_BOX["x"][1])
            & (np.abs(wy) < WALL_BOX["y"])
            & (wz > WALL_BOX["z"][0])
            & (wz < WALL_BOX["z"][1])
        )
        rec["wall_points"] += int(wmask.sum())

        scalar = getattr(gmo, "scalar", None)
        if scalar is not None and wmask.any():
            s = np.asarray(scalar, dtype=np.float64).ravel()[:n][valid]
            rec["_wi_sum"] += float(s[wmask].sum())
            rec["_wi_n"] += int(wmask.sum())

        # --- radial velocity, kept rather than discarded --------------------
        # `aux_output_level="BASIC"` already populates rv_ms; the old code paid
        # for it and threw it away. It is what separates "the wall blocked the
        # target" from "the radar only reports things that move".
        # rv_ms is a radar-only aux field. Asking the lidar for it does not
        # raise -- the bindings just print "modality is not radar" per frame,
        # which buries the log. Gate on the sensor kind instead of catching it.
        rv = None
        if kind == "radar":
            try:
                rv = np.asarray(gmo.rv_ms, dtype=np.float64).ravel()
            except Exception:
                rv = None
        if rv is not None and rv.size >= n:
            rv = rv[:n][valid]
            lo, hi = float(rv.min()), float(rv.max())
            rec["_rv"] = [lo, hi] if rec["_rv"] is None else [min(rec["_rv"][0], lo), max(rec["_rv"][1], hi)]
            rec["_rv_moving"] += int((np.abs(rv) > RV_MOVING_MPS).sum())

        # Radar returns a handful of detections per frame; printing them costs
        # nothing and turns "0 in the box" into a locatable fact.
        if len(rec["samples"]) < MAX_SAMPLES:
            order = np.argsort(-np.asarray(scalar, dtype=np.float64).ravel()[:n][valid]) if scalar is not None else range(len(rng))
            for i in list(order)[: MAX_SAMPLES - len(rec["samples"])]:
                rec["samples"].append(
                    {
                        "az_deg": round(float(az[i]), 4),
                        "el_deg": round(float(el[i]), 4),
                        "range_m": round(float(rng[i]), 4),
                        "world": [round(float(v), 4) for v in p[i]],
                        "scalar": round(float(np.asarray(scalar).ravel()[:n][valid][i]), 6) if scalar is not None else None,
                        "rv_ms": round(float(rv[i]), 4) if rv is not None and i < len(rv) else None,
                    }
                )


_COORDS_NAME = {0: "CARTESIAN", 1: "SPHERICAL", 2: "NOT_APPLICABLE"}
_FRAME_NAME = {0: "SENSOR", 1: "WORLD", 2: "CUSTOM", 3: "PARENT"}


def _harvest(label: str) -> dict:
    out = {}
    for kind in ("radar", "lidar"):
        rec = dict(ACC[kind])
        n, ssum = rec.pop("_wi_n"), rec.pop("_wi_sum")
        rec["wall_intensity_mean"] = (ssum / n) if n else None
        rec["extents_world_xyz"] = rec.pop("_ext")
        rec["extents_az_el_range"] = rec.pop("_sph")
        rec["rv_ms_range"] = rec.pop("_rv")
        rec["rv_moving_points"] = rec.pop("_rv_moving")
        rec["flag_values"] = sorted(rec["flag_values"])
        rec["coords_type_name"] = _COORDS_NAME.get(rec["coords_type"])
        rec["frame_of_ref_name"] = _FRAME_NAME.get(rec["frame_of_ref"])
        # A silent zero here has cost this project weeks. If the buffer carried
        # elements but none passed the VALID bit, say so out loud.
        rec["valid_flag_never_set"] = bool(rec["raw_points"] > 0 and rec["total_points"] == 0)
        out[kind] = rec

    for kind in ("radar", "lidar"):
        r = out[kind]
        if not r["frames_with_data"]:
            continue
        e, s = r["extents_world_xyz"], r["extents_az_el_range"]
        print(
            f"    {kind}  {r['coords_type_name']}/{r['frame_of_ref_name']}  "
            f"flags={r['flag_values']}  raw={r['raw_points']} valid={r['total_points']}",
            flush=True,
        )
        if s:
            print(
                f"      az[{s[0][0]:8.3f},{s[0][1]:8.3f}]deg  el[{s[1][0]:7.3f},{s[1][1]:7.3f}]deg"
                f"  range[{s[2][0]:7.3f},{s[2][1]:8.3f}]m",
                flush=True,
            )
        if e:
            print(
                f"      world x[{e[0][0]:8.3f},{e[0][1]:8.3f}] y[{e[1][0]:8.3f},{e[1][1]:8.3f}]"
                f" z[{e[2][0]:7.3f},{e[2][1]:7.3f}]  ground={r['ground_points']} wall={r['wall_points']}",
                flush=True,
            )
        if r["rv_ms_range"]:
            print(
                f"      rv_ms[{r['rv_ms_range'][0]:.3f},{r['rv_ms_range'][1]:.3f}]"
                f"  moving(|rv|>{RV_MOVING_MPS})={r['rv_moving_points']}",
                flush=True,
            )
        if r["valid_flag_never_set"]:
            print(f"      !! {kind}: {r['raw_points']} elements, NONE with the VALID bit set", flush=True)
        for smp in r["samples"][:6]:
            print(
                f"      . az{smp['az_deg']:>9.3f} el{smp['el_deg']:>8.3f} r{smp['range_m']:>9.3f}"
                f"  world({smp['world'][0]:>8.3f},{smp['world'][1]:>8.3f},{smp['world'][2]:>7.3f})"
                f"  s={smp['scalar']}  rv={smp['rv_ms']}",
                flush=True,
            )
    print(
        f"[{label:<32}] radar {out['radar']['target_points']:>6} @tgt /"
        f"{out['radar']['total_points']:>8} tot | "
        f"lidar {out['lidar']['target_points']:>6} @tgt /{out['lidar']['total_points']:>8} tot",
        flush=True,
    )
    return out


def _move(prim, pos) -> None:
    prim.set_world_poses(positions=np.asarray(pos, dtype=np.float32).reshape(1, 3))


RESULTS = {"controls": {}, "materials": {}}


# --------------------------------------------------------------------------
# PROTOCOL
#
# A generator keeps the sequential measurement logic readable while frames are
# actually driven by the update event stream. Each `yield` is one rendered
# frame; the callback samples the frame that just completed, then resumes.
# --------------------------------------------------------------------------
# Where the target is parked right now. `_condition(moving=True)` oscillates
# about THIS, not about TARGET_X.
#
# It used to oscillate about the literal TARGET_X, which meant `no_target` --
# whose entire job is to remove the target -- pushed it to 1 km and then
# teleported it straight back to x ~ 8 on every single collected frame. The
# target was therefore present in all four controls, and `empty` inherited it
# at wherever the previous oscillation stopped. Under the old Cartesian
# misreading this was invisible; with the decode fixed it shows up immediately
# as `empty` reporting the target's near face at 6.155 m, which is exactly
# 8.154 - 2 for the last phase of the sine. Two separate bugs, and only the
# first one had to be fixed before the second could be seen.
SCENE = {"target_base": np.asarray(TARGET_POS, dtype=np.float64)}


def _park_target(pos) -> None:
    SCENE["target_base"] = np.asarray(pos, dtype=np.float64)
    _move(target, pos)


def _condition(label, *, bucket, key, moving=True):
    COLLECTING["on"] = False
    for _ in range(WARMUP):
        yield
    ACC["radar"], ACC["lidar"] = _new_acc(), _new_acc()
    COLLECTING["on"] = True
    base = SCENE["target_base"]
    for i in range(FRAMES):
        if moving:
            phase = 2.0 * math.pi * i / MOTION_PERIOD
            _move(target, [base[0] + MOTION_AMPLITUDE * math.sin(phase), base[1], base[2]])
        yield
    COLLECTING["on"] = False
    res = _harvest(label)
    RESULTS[bucket][key] = res
    _emit({"kind": "condition", "bucket": bucket, "key": key, "result": res})


def _material_list():
    if MATERIALS == "controls":
        return []
    if MATERIALS == "smoke":
        return [(m, "none", "none") for m in SMOKE]
    if MATERIALS == "all":
        from isaacsim.core.experimental.materials.impl.non_visual_material import BASE_SPEC

        lst = [(m, "none", "none") for m in BASE_SPEC if m != "none"]
        lst += [
            ("clear_glass", "none", "visually_transparent"),
            ("plexiglass", "none", "visually_transparent"),
            ("plastic", "none", "visually_transparent"),
            ("cardboard", "none", "visually_transparent"),
            ("fabric", "none", "single_sided"),
            ("plastic", "none", "single_sided"),
            ("cardboard", "none", "single_sided"),
            ("steel", "paint", "none"),
            ("concrete", "paint", "none"),
        ]
        return lst
    return [(m.strip(), "none", "none") for m in MATERIALS.split(",")]


def _protocol():
    # --- controls ---------------------------------------------------------
    _move(occluder, [FAR_AWAY, FAR_AWAY, 0.0])
    _park_target(TARGET_POS)
    yield from _condition("CONTROL no_occluder (moving)", bucket="controls", key="no_occluder_moving")
    # Re-centre: the moving condition leaves the target wherever the last sine
    # phase put it, so without this the "static" baseline is measured at
    # x = 8.154 rather than the nominal 8.0.
    _park_target(TARGET_POS)
    yield from _condition(
        "CONTROL no_occluder (static)", bucket="controls", key="no_occluder_static", moving=False
    )

    _move(occluder, WALL_POS)
    _park_target([FAR_AWAY, -FAR_AWAY, 0.0])
    yield from _condition("CONTROL no_target", bucket="controls", key="no_target")

    _move(occluder, [FAR_AWAY, FAR_AWAY, 0.0])
    yield from _condition("CONTROL empty", bucket="controls", key="empty", moving=False)

    # --- the measurement --------------------------------------------------
    _move(occluder, WALL_POS)
    _park_target(TARGET_POS)
    for base, coating, attribute in _material_list():
        occluder_mat.set_bases(base)
        occluder_mat.set_coatings(coating)
        occluder_mat.set_attributes(attribute)
        key = f"{base}|{coating}|{attribute}"
        yield from _condition(key, bucket="materials", key=key)


# --------------------------------------------------------------------------
# REPORT
# --------------------------------------------------------------------------
def _report() -> None:
    print("\n" + "=" * 78, flush=True)
    print("S4 RADAR PENETRATION SPIKE -- RESULT", flush=True)
    print("=" * 78, flush=True)

    c = RESULTS["controls"]
    base_r = c.get("no_occluder_moving", {}).get("radar", {}).get("target_points", 0)
    base_l = c.get("no_occluder_moving", {}).get("lidar", {}).get("target_points", 0)
    stat_r = c.get("no_occluder_static", {}).get("radar", {}).get("target_points", 0)
    art_r = c.get("no_target", {}).get("radar", {}).get("target_points", 0)
    art_l = c.get("no_target", {}).get("lidar", {}).get("target_points", 0)

    print(f"\nsensors present                    : {SENSORS_MODE}", flush=True)
    print(f"Clear line of sight, target MOVING : radar={base_r}  lidar={base_l}", flush=True)
    print(f"Clear line of sight, target STATIC : radar={stat_r}", flush=True)
    print(f"Artifact floor (wall, no target)   : radar={art_r}  lidar={art_l}", flush=True)
    emp = c.get("empty", {})
    print(
        f"Empty scene (ground only) @tgt box : radar={emp.get('radar', {}).get('target_points', 0)}"
        f"  lidar={emp.get('lidar', {}).get('target_points', 0)}",
        flush=True,
    )

    # ----------------------------------------------------------------------
    # WALL-MASK DISCRIMINATION GATE
    #
    # The mask this replaces was a fixed 2 deg azimuth wedge with no range
    # bound: it counted the same ~76,330 lidar points whether the wall stood at
    # 4 m or a kilometre away, so every occlusion number built on it was
    # meaningless. A mask that cannot tell those two scenes apart must not be
    # trusted, so the run proves it can before reporting anything else.
    #
    #   `no_target` puts the wall at x = 4     -> the box must FILL
    #   `empty`     puts the wall at x = 1000  -> the box must EMPTY
    # ----------------------------------------------------------------------
    print(
        f"\nWall-mask discrimination (box x {WALL_BOX['x']}, |y|<{WALL_BOX['y']}, z {WALL_BOX['z']}):",
        flush=True,
    )
    mask_ok = {}
    for kind, used in (("radar", USE_RADAR), ("lidar", USE_LIDAR)):
        if not used:
            continue
        near = c.get("no_target", {}).get(kind, {}).get("wall_points", 0)
        far = c.get("empty", {}).get(kind, {}).get("wall_points", 0)
        ok = near > 0 and far == 0
        mask_ok[kind] = ok
        print(
            f"  {kind:<6} wall @4m -> {near:>8} pts | wall @1km -> {far:>8} pts   "
            f"{'DISCRIMINATES' if ok else '!! DOES NOT DISCRIMINATE'}",
            flush=True,
        )

    # A sensor that was never created reads zero by construction. That is not a
    # failed baseline, so only judge the sensors actually present.
    invalid = []
    if USE_RADAR and base_r == 0:
        invalid.append("radar saw nothing with a clear line of sight")
    if USE_LIDAR and base_l == 0:
        invalid.append("lidar saw nothing with a clear line of sight")
    for kind, ok in mask_ok.items():
        if not ok:
            invalid.append(f"{kind} wall mask does not distinguish a wall at 4 m from one at 1 km")
    if invalid:
        print("\n!! TEST INVALID: " + "; ".join(invalid), flush=True)
    if USE_RADAR and base_r > 0 and stat_r == 0:
        print("\nNote: this radar reports MOVING targets only; a static target at the", flush=True)
        print("same spot returns nothing. Static shelves would be invisible to it", flush=True)
        print("regardless of material.", flush=True)

    penetrating = []
    if RESULTS["materials"]:
        print(f"\n{'material (base|coating|attribute)':<42} {'radar':>8} {'lidar':>8}   verdict", flush=True)
        print("-" * 78, flush=True)
        for key, rec in RESULTS["materials"].items():
            r = rec["radar"]["target_points"]
            ln = rec["lidar"]["target_points"]
            rn, lnn = r - art_r, ln - art_l
            if rn > 0 and lnn <= 0:
                verdict = "RADAR PENETRATES, lidar blocked"
                penetrating.append((key, rn))
            elif rn > 0 and lnn > 0:
                verdict = "both penetrate -- wall not opaque?"
            elif rn <= 0 and lnn > 0:
                verdict = "lidar only (unexpected)"
            else:
                verdict = "neither -- opaque to both"
            print(f"{key:<42} {r:>8} {ln:>8}   {verdict}", flush=True)

    # ----------------------------------------------------------------------
    # MATERIAL-SWAP VALIDITY
    #
    # This used to read lidar intensity ONLY -- so a radar-only run reported
    # "INCONCLUSIVE" and the sweep's headline answer sailed through unchecked,
    # which is how "NO, radar penetrates nothing" got printed over six
    # measurements of an unchanged wall. It now checks every sensor that was
    # actually present, and radar's scalar is the RCS in dBsm, which is the
    # quantity a non-visual material swap is supposed to move.
    # ----------------------------------------------------------------------
    swap_ok = None
    if RESULTS["materials"]:
        print("\nMaterial-swap validity (mean wall-face scalar; radar = RCS dBsm):", flush=True)
        per_sensor = {}
        for kind, used in (("radar", USE_RADAR), ("lidar", USE_LIDAR)):
            if not used:
                continue
            vals = {
                k: v[kind]["wall_intensity_mean"]
                for k, v in RESULTS["materials"].items()
                if v[kind]["wall_intensity_mean"] is not None
            }
            if len(vals) < 2:
                print(f"  {kind}: INCONCLUSIVE -- fewer than two materials produced samples.", flush=True)
                continue
            spread = max(vals.values()) - min(vals.values())
            per_sensor[kind] = spread > 1e-9
            for k, v in sorted(vals.items()):
                print(f"    {kind:<6} {k:<42} {v:.9f}", flush=True)
            print(
                f"  {kind}: {'OK' if per_sensor[kind] else '!! IDENTICAL'} -- spread {spread:.9f}"
                + (
                    ""
                    if per_sensor[kind]
                    else "; the swap never reached this sensor, its material results are vacuous."
                ),
                flush=True,
            )
        if per_sensor:
            swap_ok = any(per_sensor.values())

    print("\nANSWER: ", end="", flush=True)
    if invalid:
        print("INVALID -- see the baseline warning above.", flush=True)
    elif not RESULTS["materials"]:
        print("controls only; no material sweep run.", flush=True)
    elif swap_ok is False:
        print("INVALID -- the material swap never reached the renderer.", flush=True)
    elif penetrating:
        print(f"YES -- radar penetrates {len(penetrating)} material(s) that block lidar:", flush=True)
        for key, rn in penetrating:
            print(f"          {key}  ({rn} net radar points behind the wall)", flush=True)
    else:
        print("NO -- radar penetrated none of the materials tested.", flush=True)
        print("        Every occluder was opaque to radar and to lidar alike.", flush=True)

    _emit(
        {
            "kind": "summary",
            "sensors": SENSORS_MODE,
            "radar_attrs": RADAR_ATTRS,
            "wall_mask_discriminates": mask_ok,
            "baseline_radar_moving": base_r,
            "baseline_radar_static": stat_r,
            "baseline_lidar": base_l,
            "artifact_radar": art_r,
            "artifact_lidar": art_l,
            "material_swap_effective": swap_ok,
            "invalid": invalid,
            "penetrating": [{"material": k, "radar_net": n} for k, n in penetrating],
        }
    )
    print("\nDONE", flush=True)


# --------------------------------------------------------------------------
# DRIVE
# --------------------------------------------------------------------------
_gen = _protocol()
_state = {"sub": None, "frames": 0}


def _on_update(_e) -> None:
    if COLLECTING["on"]:
        _sample()
    _state["frames"] += 1
    try:
        next(_gen)
    except StopIteration:
        _state["sub"] = None
        try:
            _report()
        except Exception as exc:  # never die silently inside a callback
            import traceback

            print(f"\n!! report failed: {type(exc).__name__}: {exc}", flush=True)
            traceback.print_exc()
        _app.post_quit()
    except Exception as exc:
        import traceback

        print(f"\n!! protocol failed: {type(exc).__name__}: {exc}", flush=True)
        traceback.print_exc()
        _state["sub"] = None
        _app.post_quit()


_state["sub"] = _app.get_update_event_stream().create_subscription_to_pop(
    _on_update, name="radar_spike_exec"
)
print("\nsubscribed to update stream; measurement starting\n", flush=True)
