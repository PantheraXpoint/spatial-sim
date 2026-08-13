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
TARGET_BOX_X = (6.0, 10.5)
TARGET_BOX_Y = TARGET_BOX_Z = 2.5
WALL_BOX_X = (3.0, 5.0)

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
        aux_output_level="BASIC",
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
        "total_points": 0,
        "target_points": 0,
        "wall_points": 0,
        "_wi_sum": 0.0,
        "_wi_n": 0,
        # Extents of the returned cloud. Without these a zero target count is
        # unreadable: "the wall blocked it" and "my box is in the wrong frame
        # or the wrong units" produce the same 0.
        "_ext": None,
    }


ACC = {"radar": _new_acc(), "lidar": _new_acc()}
COLLECTING = {"on": False}
SENSORS = tuple(
    (kind, s)
    for kind, s in (("radar", radar_sensor), ("lidar", lidar_sensor))
    if s is not None
)


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
        x = np.asarray(gmo.x, dtype=np.float64).ravel()
        y = np.asarray(gmo.y, dtype=np.float64).ravel()
        z = np.asarray(gmo.z, dtype=np.float64).ravel()
        n = min(n, len(x), len(y), len(z))
        x, y, z = x[:n], y[:n], z[:n]

        rec = ACC[kind]
        rec["frames_with_data"] += 1
        rec["total_points"] += n

        ext = [
            [float(x.min()), float(x.max())],
            [float(y.min()), float(y.max())],
            [float(z.min()), float(z.max())],
        ]
        if rec["_ext"] is None:
            rec["_ext"] = ext
        else:
            for a in range(3):
                rec["_ext"][a][0] = min(rec["_ext"][a][0], ext[a][0])
                rec["_ext"][a][1] = max(rec["_ext"][a][1], ext[a][1])

        # "Behind the wall" means inside a box around the TARGET, not merely
        # past the wall's x -- ground returns reach around the wall's edge.
        tmask = (
            (x > TARGET_BOX_X[0])
            & (x < TARGET_BOX_X[1])
            & (np.abs(y) < TARGET_BOX_Y)
            & (np.abs(z) < TARGET_BOX_Z)
        )
        rec["target_points"] += int(tmask.sum())

        wmask = (x > WALL_BOX_X[0]) & (x < WALL_BOX_X[1])
        rec["wall_points"] += int(wmask.sum())

        scalar = getattr(gmo, "scalar", None)
        if scalar is not None and wmask.any():
            s = np.asarray(scalar, dtype=np.float64).ravel()[:n]
            rec["_wi_sum"] += float(s[wmask].sum())
            rec["_wi_n"] += int(wmask.sum())


def _harvest(label: str) -> dict:
    out = {}
    for kind in ("radar", "lidar"):
        rec = dict(ACC[kind])
        n, ssum = rec.pop("_wi_n"), rec.pop("_wi_sum")
        rec["wall_intensity_mean"] = (ssum / n) if n else None
        rec["extents_xyz"] = rec.pop("_ext")
        out[kind] = rec
    for kind in ("radar", "lidar"):
        e = out[kind]["extents_xyz"]
        if e:
            print(
                f"    {kind} extents  x[{e[0][0]:.2f},{e[0][1]:.2f}] "
                f"y[{e[1][0]:.2f},{e[1][1]:.2f}] z[{e[2][0]:.2f},{e[2][1]:.2f}]",
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
def _condition(label, *, bucket, key, moving=True):
    COLLECTING["on"] = False
    for _ in range(WARMUP):
        yield
    ACC["radar"], ACC["lidar"] = _new_acc(), _new_acc()
    COLLECTING["on"] = True
    for i in range(FRAMES):
        if moving:
            phase = 2.0 * math.pi * i / MOTION_PERIOD
            _move(target, [TARGET_X + MOTION_AMPLITUDE * math.sin(phase), 0.0, SENSOR_Z])
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
    _move(target, TARGET_POS)
    yield from _condition("CONTROL no_occluder (moving)", bucket="controls", key="no_occluder_moving")
    yield from _condition(
        "CONTROL no_occluder (static)", bucket="controls", key="no_occluder_static", moving=False
    )

    _move(occluder, WALL_POS)
    _move(target, [FAR_AWAY, -FAR_AWAY, 0.0])
    yield from _condition("CONTROL no_target", bucket="controls", key="no_target")

    _move(occluder, [FAR_AWAY, FAR_AWAY, 0.0])
    yield from _condition("CONTROL empty", bucket="controls", key="empty", moving=False)

    # --- the measurement --------------------------------------------------
    _move(occluder, WALL_POS)
    _move(target, TARGET_POS)
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

    # A sensor that was never created reads zero by construction. That is not a
    # failed baseline, so only judge the sensors actually present.
    invalid = []
    if USE_RADAR and base_r == 0:
        invalid.append("radar saw nothing with a clear line of sight")
    if USE_LIDAR and base_l == 0:
        invalid.append("lidar saw nothing with a clear line of sight")
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

    intens = {
        k: v["lidar"]["wall_intensity_mean"]
        for k, v in RESULTS["materials"].items()
        if v["lidar"]["wall_intensity_mean"] is not None
    }
    swap_ok = None
    if RESULTS["materials"]:
        print("\nMaterial-swap validity (mean lidar intensity off the wall face):", flush=True)
        if len(intens) < 2:
            print("  INCONCLUSIVE -- fewer than two materials produced intensity samples.", flush=True)
        else:
            spread = max(intens.values()) - min(intens.values())
            swap_ok = spread > 1e-6
            for k, v in sorted(intens.items()):
                print(f"  {k:<42} {v:.6f}", flush=True)
            print(
                f"  {'OK' if swap_ok else '!! IDENTICAL'} -- spread {spread:.6f}"
                + ("" if swap_ok else "; the swap never reached the renderer, results are vacuous."),
                flush=True,
            )

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
