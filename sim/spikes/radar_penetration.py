"""S4 radar spike -- does RTX radar see through an occluder that stops lidar?

THROWAWAY. Not imported by anything. Not part of the main scene. It lives here
so the answer is reproducible, not so the code gets reused.

The question, precisely
-----------------------
Put a flat wall between the sensors and a target. The wall spans the entire
line of sight, so there is no geometric path around it. Then:

  * does the RADAR report detections at the target's location?
  * does the LIDAR not?

and does the answer depend on the wall's ``omni:simready:nonvisual:*``
material -- the USD attributes that in 6.0 replaced the 4.5-era CSV material
mapping as the thing governing how RTX sensors interact with a surface?

(The CSVs still exist, but only as the name->index specification that
``NonVisualMaterial`` encodes into those attributes. Per-prim assignment is
now purely a USD attribute.)

Method
------
One Isaac Sim launch. The occluder's non-visual material is swapped at runtime
-- ``NonVisualMaterial.set_bases`` writes the USD attribute directly -- so
every material is measured against an otherwise byte-identical scene.

Data comes through Replicator ``Writer`` subclasses, not ``sensor.get_data``.
This is not a style preference: ``get_data("generic-model-output")`` hands back
a buffer before the sensor has filled it, and
``parse_generic_model_output_data`` rejects it with "Invalid magic number" on
every frame. The writer callback only fires when a frame is genuinely ready.
All four shipped 6.0 examples use writers; so does this.

Controls
--------
"Radar returned points behind the wall" is only evidence of penetration if the
wall is really opaque and the target is really the source. So:

  no_occluder_moving  wall parked far away, target oscillating. Baseline: what
                      100% visibility looks like for each sensor.
  no_occluder_static  same, target frozen. Isolates whether this radar reports
                      STATIC geometry at all -- an FMCW/Doppler model may
                      suppress zero-velocity returns entirely, which would make
                      every "no penetration" reading meaningless.
  no_target           wall present, target parked far away. Whatever still
                      lands in the target box is an artifact (ground bounce,
                      multipath, sidelobe) and is subtracted from every result.
  empty               neither. Floor noise.

A detection counts as "behind the wall" only inside a box drawn around the
target. Counting everything past the wall's x would score ground returns that
the sensor reaches by going *around* the wall's edge, not through it.

The target moves during measurement because the demo this spike serves has
exactly one moving entity, and because a static target is the worst case for a
Doppler sensor. Moving it gives radar its best shot at a "yes".

Validity check
--------------
A runtime material swap is only meaningful if it reaches the renderer. Mean
lidar intensity (GMO ``scalar``) off the wall face is recorded per material: if
that number does not move between ``steel`` and ``fabric``, the swap never took
effect and every "no penetration" result is vacuous. Checked, not assumed.

Usage
-----
    ./python.sh -u /workspace/sim/spikes/radar_penetration.py \
        --materials smoke --json /workspace/radar_spike.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys

parser = argparse.ArgumentParser(description="RTX radar penetration spike.")
parser.add_argument(
    "--materials",
    default="smoke",
    help="'smoke' (6 representative), 'all' (every base plus attribute combos), or a comma-separated list.",
)
parser.add_argument("--frames", type=int, default=40, help="Frames accumulated per condition.")
parser.add_argument("--warmup", type=int, default=20, help="Frames discarded after each scene change.")
parser.add_argument("--json", default=None, help="Write full results here.")
parser.add_argument("--dump", action="store_true", help="Print raw detection coords during the controls.")
parser.add_argument(
    "--controls-only", action="store_true", help="Run the controls and stop. Use to validate the rig cheaply."
)
parser.add_argument(
    "--multi-gpu", action="store_true", help="Re-enable multi-GPU rendering. See the note below -- do not."
)
args, _ = parser.parse_known_args()

# --------------------------------------------------------------------------
# Hard rule 3: SimulationApp before any omni/isaacsim import.
# enable_motion_bvh is REQUIRED for radar -- Radar._create_prim raises without
# it. In 6.0 this SimulationApp key is the supported way to set the underlying
# /renderer/raytracingMotion/* carb settings; setting them by hand afterwards
# is too late, the BVH is built at renderer init.
# --------------------------------------------------------------------------
# --------------------------------------------------------------------------
# multi_gpu=False is not a tuning knob here, it is the difference between the
# spike running and the spike not running.
#
# SimulationApp defaults multi_gpu to True. This host has IOMMU enabled, and
# Isaac Sim's own startup probe measures the consequence:
#
#     Detected IOMMU is enabled. Running CUDA peer-to-peer bandwidth ...
#     P2P Writes:  GPU0->GPU0 830 GB/s,  GPU0->GPU1 11.2 GB/s
#
# Two rendering GPUs plus a 74x P2P penalty means every RTX sensor frame is
# dominated by cross-GPU copies. Measured on the SHIPPED example
# (standalone_examples/.../inspect_radar_gmo.py, unmodified): ~150 s/frame,
# and Replicator eventually gives up with
#     "Timed out while waiting for pending Replicator writer schedules to
#      drain."
# so the writer never fires and the radar reports nothing at all. That looks
# exactly like "radar sees nothing" and is really "the renderer never
# delivered a frame". Single-GPU sidesteps it entirely.
#
# Note this is narrower than CLAUDE.md's "cap at 2 rendering GPUs" -- that cap
# is about not stealing GPU 2 from inference. It says nothing about IOMMU,
# which is what actually bites here. Worth re-checking before M4 places
# several sensors at once.
# --------------------------------------------------------------------------
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {
        "headless": True,
        "enable_motion_bvh": True,  # REQUIRED for radar
        "multi_gpu": args.multi_gpu,
        "active_gpu": 0,
        "physics_gpu": 0,
        # Main-viewport settings only. RTX sensors render into their own
        # render products, so shrinking the beauty pass costs us nothing and
        # takes the default 1280x720 / 64-spp RealTimePathTracing frame off
        # the critical path.
        "width": 256,
        "height": 256,
        "samples_per_pixel_per_frame": 1,
        "denoiser": False,
        "anti_aliasing": 0,
    }
)

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.experimental.materials import NonVisualMaterial  # noqa: E402
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane  # noqa: E402
from isaacsim.core.experimental.utils.app import enable_extension  # noqa: E402
from isaacsim.sensors.experimental.rtx import (  # noqa: E402
    Lidar,
    LidarSensor,
    Radar,
    RadarSensor,
    parse_generic_model_output_data,
)
from omni.replicator.core import Writer  # noqa: E402

# --------------------------------------------------------------------------
# omni.hydratexture is NOT enabled by isaacsim.exp.base.python.kit, the app
# SimulationApp launches by default. Without it, rep.create.render_product --
# which is what RadarSensor/LidarSensor build internally -- produces a hydra
# texture that never renders. The failure is completely silent:
#
#   * the writer's write() callback is invoked 0 times, ever
#   * sensor.get_data() hands back an unfilled buffer, and
#     parse_generic_model_output_data rejects it: "Invalid magic number"
#   * rep.orchestrator.step() blocks forever waiting for a frame that will
#     never arrive (measured: >10 min, killed)
#
# This is CLAUDE.md failure mode #2 -- "RTX sensor without its own viewport
# silently does not simulate" -- in its headless form. There is no viewport to
# forget; the missing piece is the extension that backs the render product.
# The shipped standalone example inspect_radar_gmo.py hits this too and exits
# 0 having printed nothing, so "the example works" is not evidence.
#
# Must be enabled BEFORE any sensor is constructed.
# --------------------------------------------------------------------------
enable_extension("omni.hydratexture")
enable_extension("isaacsim.sensors.rtx.nodes")

if not carb.settings.get_settings().get("/renderer/raytracingMotion/enabled"):
    print("FATAL: Motion BVH is off; radar cannot be created.", file=sys.stderr)
    simulation_app.close()
    sys.exit(2)

# --------------------------------------------------------------------------
# GEOMETRY
#
#   sensors           wall                 target
#   x=0, z=1          x=4                  x=8 (oscillating +/-0.5 in x)
#      |               |                     |
#      +-------------->|############        [] <- 2 m cube, steel
#                      |############
#
# The wall is 12 m wide and runs from below the floor to 4 m up. The target
# subtends about +/-7 deg of azimuth from the sensor; the wall covers +/-56
# deg. The target is fully shadowed with a large margin, so the result does
# not hinge on getting the wall's extent exactly right.
# --------------------------------------------------------------------------
SENSOR_Z = 1.0
WALL_X = 4.0
TARGET_X = 8.0
FAR_AWAY = 1000.0  # where a prim goes to be "removed" without deleting it

WALL_POS = np.array([WALL_X, 0.0, 1.5])
WALL_SCALE = np.array([0.1, 12.0, 4.0])  # thin in x, wide in y, tall in z
TARGET_POS = np.array([TARGET_X, 0.0, SENSOR_Z])
TARGET_SCALE = np.array([2.0, 2.0, 2.0])

# Radial oscillation -- Doppler responds to range rate, so move along x.
# ~0.5 m amplitude over a 20-frame period is several m/s, far above any
# plausible minimum-velocity gate.
MOTION_AMPLITUDE = 0.5
MOTION_PERIOD = 20.0

# A detection is "at the target" only inside this box: generous enough to
# absorb range noise and the oscillation, tight enough to exclude the ground
# and the wall's edges.
TARGET_BOX_X = (6.0, 10.5)
TARGET_BOX_Y = 2.5
TARGET_BOX_Z = 2.5

# The wall face, for reading back the intensity that proves a material swap
# actually reached the renderer.
WALL_BOX_X = (3.0, 5.0)

SMOKE_MATERIALS = ["steel", "concrete", "cardboard", "plastic", "clear_glass", "fabric"]

# --------------------------------------------------------------------------
# SCENE
# --------------------------------------------------------------------------
DistantLight("/World/light").set_intensities(3000.0)
GroundPlane("/World/ground")

target = Cube("/World/target", positions=TARGET_POS, scales=TARGET_SCALE, colors=[0.9, 0.1, 0.1])
target_material = NonVisualMaterial(
    "/World/target/material", bases="steel", coatings="none", attributes="none"
)
target.apply_visual_materials(target_material)

occluder = Cube("/World/occluder", positions=WALL_POS, scales=WALL_SCALE, colors=[0.6, 0.6, 0.6])
occluder_material = NonVisualMaterial(
    "/World/occluder/material", bases="concrete", coatings="none", attributes="none"
)
occluder.apply_visual_materials(occluder_material)

# Identity orientation: RTX range sensors look down local +X.
radar = Radar(
    "/World/radar",
    translations=np.array([0.0, 0.0, SENSOR_Z]),
    orientations=np.array([1.0, 0.0, 0.0, 0.0]),
    aux_output_level="BASIC",
)
radar_sensor = RadarSensor(radar, annotators=[])

lidar = Lidar.create(
    "/World/lidar",
    config="Example_Rotary",
    translations=np.array([0.0, 0.0, SENSOR_Z]),
    orientations=np.array([1.0, 0.0, 0.0, 0.0]),
    aux_output_level="FULL",
)
lidar_sensor = LidarSensor(lidar, annotators=[])

# --------------------------------------------------------------------------
# COLLECTION
#
# Writers push; we cannot pull. So the writers deposit into ACC and the main
# loop flips COLLECTING on once warm-up frames are behind us.
# --------------------------------------------------------------------------
COLLECTING = {"on": False, "dump": False}


def _new_acc():
    return {
        "frames_with_data": 0,
        "total_points": 0,
        "target_points": 0,
        "wall_points": 0,
        "_wall_int_sum": 0.0,
        "_wall_int_n": 0,
        "samples": [],
    }


ACC = {"radar": _new_acc(), "lidar": _new_acc()}


def _ingest(kind: str, gmo) -> None:
    """Fold one parsed GMO frame into the accumulator for `kind`."""
    rec = ACC[kind]
    n = int(gmo.numElements)
    if n <= 0:
        return
    x = np.asarray(gmo.x, dtype=np.float64).ravel()[:n]
    y = np.asarray(gmo.y, dtype=np.float64).ravel()[:n]
    z = np.asarray(gmo.z, dtype=np.float64).ravel()[:n]
    n = min(n, len(x), len(y), len(z))
    x, y, z = x[:n], y[:n], z[:n]

    rec["frames_with_data"] += 1
    rec["total_points"] += n

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
        rec["_wall_int_sum"] += float(s[wmask].sum())
        rec["_wall_int_n"] += int(wmask.sum())

    if COLLECTING["dump"] and len(rec["samples"]) < 10:
        order = np.argsort(-x)  # farthest first -- the interesting end
        for i in order[: 10 - len(rec["samples"])]:
            rec["samples"].append(
                [round(float(x[i]), 2), round(float(y[i]), 2), round(float(z[i]), 2)]
            )


def _make_collector(kind: str):
    """Build a Writer subclass that funnels GMO frames into ACC[kind]."""

    class _Collector(Writer):
        def __init__(self) -> None:
            self.data_structure = "renderProduct"
            self.annotators = [rep.annotators.get("GenericModelOutput")]

        def write(self, data: dict) -> None:
            if not COLLECTING["on"] or "renderProducts" not in data:
                return
            for _rp, rp_data in data["renderProducts"].items():
                raw = rp_data.get("GenericModelOutput")
                if isinstance(raw, dict):
                    raw = raw.get("data")
                if raw is None:
                    continue
                gmo = parse_generic_model_output_data(raw)
                if gmo is not None and gmo.numElements > 0:
                    _ingest(kind, gmo)

    _Collector.__name__ = f"{kind.capitalize()}Collector"
    return _Collector


RadarCollector = _make_collector("radar")
LidarCollector = _make_collector("lidar")
rep.WriterRegistry.register(RadarCollector)
rep.WriterRegistry.register(LidarCollector)
radar_sensor.attach_writer("RadarCollector")
lidar_sensor.attach_writer("LidarCollector")

timeline = omni.timeline.get_timeline_interface()
timeline.play()


# --------------------------------------------------------------------------
# MEASUREMENT
# --------------------------------------------------------------------------
def _move(prim, position) -> None:
    prim.set_world_poses(positions=np.asarray(position, dtype=np.float32).reshape(1, 3))


_frame_clock = {"n": 0}


def _step(moving_target: bool) -> None:
    """Advance one frame, optionally driving the target's radial oscillation."""
    if moving_target:
        phase = 2.0 * math.pi * _frame_clock["n"] / MOTION_PERIOD
        _move(target, [TARGET_X + MOTION_AMPLITUDE * math.sin(phase), 0.0, SENSOR_Z])
    _frame_clock["n"] += 1
    simulation_app.update()


def measure(label: str, *, moving_target: bool = True, dump: bool = False) -> dict:
    """Warm up, then pool detections from both sensors over --frames frames.

    Counts are summed over frames rather than averaged: radar detections are
    sparse and vary frame to frame, so a sum over a fixed frame budget is the
    honest way to compare conditions.
    """
    COLLECTING["on"] = False
    for _ in range(args.warmup):
        _step(moving_target)

    ACC["radar"] = _new_acc()
    ACC["lidar"] = _new_acc()
    COLLECTING["on"] = True
    COLLECTING["dump"] = dump
    for _ in range(args.frames):
        _step(moving_target)
    COLLECTING["on"] = False
    COLLECTING["dump"] = False

    out = {}
    for kind in ("radar", "lidar"):
        rec = dict(ACC[kind])
        n = rec.pop("_wall_int_n")
        s = rec.pop("_wall_int_sum")
        rec["wall_intensity_mean"] = (s / n) if n else None
        out[kind] = rec

    print(
        f"[{label:<32}] "
        f"radar {out['radar']['target_points']:>6} @tgt /{out['radar']['total_points']:>7} tot | "
        f"lidar {out['lidar']['target_points']:>6} @tgt /{out['lidar']['total_points']:>7} tot",
        flush=True,
    )
    if dump:
        for kind in ("radar", "lidar"):
            print(f"    {kind} farthest samples (x,y,z): {out[kind]['samples']}", flush=True)
    return out


results = {
    "geometry": {
        "sensor_z": SENSOR_Z,
        "wall_x": WALL_X,
        "target_x": TARGET_X,
        "target_box_x": TARGET_BOX_X,
        "frames_per_condition": args.frames,
    },
    "controls": {},
    "materials": {},
}

# --- controls --------------------------------------------------------------
_move(occluder, [FAR_AWAY, FAR_AWAY, 0.0])
_move(target, TARGET_POS)
results["controls"]["no_occluder_moving"] = measure("CONTROL no_occluder (moving)", dump=args.dump)
results["controls"]["no_occluder_static"] = measure(
    "CONTROL no_occluder (static)", moving_target=False, dump=args.dump
)

_move(occluder, WALL_POS)
_move(target, [FAR_AWAY, -FAR_AWAY, 0.0])
results["controls"]["no_target"] = measure("CONTROL no_target", dump=args.dump)

_move(occluder, [FAR_AWAY, FAR_AWAY, 0.0])
results["controls"]["empty"] = measure("CONTROL empty")

# --- the measurement -------------------------------------------------------
_move(occluder, WALL_POS)
_move(target, TARGET_POS)

if args.controls_only:
    print("\n--controls-only: stopping before the material sweep.")
    material_list = []
elif args.materials == "smoke":
    material_list = [(m, "none", "none") for m in SMOKE_MATERIALS]
elif args.materials == "all":
    from isaacsim.core.experimental.materials.impl.non_visual_material import BASE_SPEC

    material_list = [(m, "none", "none") for m in BASE_SPEC if m != "none"]
    material_list += [
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
else:
    material_list = [(m.strip(), "none", "none") for m in args.materials.split(",")]

for base, coating, attribute in material_list:
    occluder_material.set_bases(base)
    occluder_material.set_coatings(coating)
    occluder_material.set_attributes(attribute)
    key = f"{base}|{coating}|{attribute}"
    results["materials"][key] = measure(key)

# --------------------------------------------------------------------------
# REPORT
# --------------------------------------------------------------------------
print("\n" + "=" * 82)
print("S4 RADAR PENETRATION SPIKE -- RESULT")
print("=" * 82)

ctl = results["controls"]
base_radar = ctl["no_occluder_moving"]["radar"]["target_points"]
base_lidar = ctl["no_occluder_moving"]["lidar"]["target_points"]
static_radar = ctl["no_occluder_static"]["radar"]["target_points"]
art_radar = ctl["no_target"]["radar"]["target_points"]
art_lidar = ctl["no_target"]["lidar"]["target_points"]

print(f"\nBaseline, clear line of sight, target MOVING: radar={base_radar}  lidar={base_lidar}")
print(f"Baseline, clear line of sight, target STATIC: radar={static_radar}")
print(f"Artifact floor, wall present and no target  : radar={art_radar}  lidar={art_lidar}")

invalid = []
if base_radar == 0:
    invalid.append("radar saw nothing even with a clear line of sight")
if base_lidar == 0:
    invalid.append("lidar saw nothing even with a clear line of sight")
if invalid:
    print("\n!! TEST INVALID: " + "; ".join(invalid))
    print("!! Fix sensor range/FOV/RCS before reading anything below.")
if base_radar > 0 and static_radar == 0:
    print("\nNote: this radar reports MOVING targets only -- a static target at the")
    print("same spot returns nothing. Relevant to the demo: static shelves are")
    print("invisible to it regardless of material.")

print(f"\n{'material (base|coating|attribute)':<42} {'radar':>8} {'lidar':>8}   verdict")
print("-" * 82)
penetrating = []
for key, rec in results["materials"].items():
    r = rec["radar"]["target_points"]
    ln = rec["lidar"]["target_points"]
    r_net, l_net = r - art_radar, ln - art_lidar
    if r_net > 0 and l_net <= 0:
        verdict = "RADAR PENETRATES, lidar blocked"
        penetrating.append((key, r_net, l_net))
    elif r_net > 0 and l_net > 0:
        verdict = "both penetrate -- wall not opaque?"
    elif r_net <= 0 and l_net > 0:
        verdict = "lidar only (unexpected)"
    else:
        verdict = "neither -- opaque to both"
    print(f"{key:<42} {r:>8} {ln:>8}   {verdict}")

intensities = {
    k: v["lidar"]["wall_intensity_mean"]
    for k, v in results["materials"].items()
    if v["lidar"]["wall_intensity_mean"] is not None
}
print("\nMaterial-swap validity (mean lidar intensity off the wall face):")
swap_ok = None
if len(intensities) < 2:
    print("  INCONCLUSIVE -- fewer than two materials produced intensity samples.")
else:
    for k, v in intensities.items():
        print(f"  {k:<42} {v:.6f}")
    spread = max(intensities.values()) - min(intensities.values())
    swap_ok = spread > 1e-6
    if swap_ok:
        print(f"  OK -- intensity varies by {spread:.6f} across materials; the swap takes effect.")
    else:
        print("  !! Intensity IDENTICAL across materials. The non-visual material swap")
        print("  !! never reached the renderer -- every result above is vacuous.")

print("\nANSWER: ", end="")
if invalid:
    print("INVALID -- see the baseline warning above.")
elif swap_ok is False:
    print("INVALID -- the material swap never reached the renderer.")
elif penetrating:
    print(f"YES -- radar penetrates {len(penetrating)} material(s) that block lidar:")
    for key, r_net, _ in penetrating:
        print(f"          {key}  ({r_net} net radar points behind the wall)")
else:
    print("NO -- radar penetrated none of the materials tested.")
    print("        Every occluder was opaque to radar and to lidar alike.")

results["summary"] = {
    "baseline_radar_moving": base_radar,
    "baseline_radar_static": static_radar,
    "baseline_lidar": base_lidar,
    "artifact_radar": art_radar,
    "artifact_lidar": art_lidar,
    "material_swap_effective": swap_ok,
    "penetrating": [{"material": k, "radar_net": r, "lidar_net": ln} for k, r, ln in penetrating],
}

if args.json:
    with open(args.json, "w", encoding="utf-8") as fh:
        json.dump(results, fh, indent=2)
    print(f"\nWrote {args.json}")

timeline.stop()
simulation_app.close()
