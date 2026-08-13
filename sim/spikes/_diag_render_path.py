"""Diagnostic: does a fuller Kit experience restore Replicator capture?

Throwaway debugging aid for S4.

Evidence so far, under the DEFAULT experience (isaacsim.exp.base.python.kit,
which is what SimulationApp picks for ./python.sh scripts):

  * plain camera ``rgb`` annotator, 60x update()   -> shape (0,), no pixels
  * RTX lidar ``generic-model-output``, 60x update -> "Invalid magic number"
  * ``rep.orchestrator.step()``, camera only       -> hangs; not even SIGALRM
    interrupts it, because it blocks inside C++ and Python never gets to run
    its handler. Container timeout is the only way out.
  * no renderer error anywhere in the log

So nothing errors, nothing renders. Meanwhile S3 proved this host CAN render:
livestreaming produced a real picture. The streaming path runs a different
experience file (isaacsim.exp.full.streaming.kit) that loads the viewport and
rendering stack, which the bare python experience does not.

This tests that hypothesis directly: same script, same scene, richer
experience. ``--experience`` selects which.

Never calls orchestrator.step() -- it cannot be interrupted.
"""

from __future__ import annotations

import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "--experience",
    default="/isaac-sim/apps/isaacsim.exp.full.kit",
    help="Kit experience file. Pass '' for the SimulationApp default.",
)
parser.add_argument("--frames", type=int, default=90)
parser.add_argument(
    "--kit-log",
    default="/workspace/kit_diag.log",
    help="Where Kit writes its own log. The default --portable launch puts it "
    "somewhere the container discards on exit, which is why earlier runs "
    "appeared to log nothing at all.",
)
parser.add_argument("--log-level", default="warning", help="Kit log level: warning | info | verbose.")
parser.add_argument(
    "--force-async",
    action="store_true",
    help="Set /app/asyncRendering=true at launch. This is the single measured "
    "difference between the working runheadless.sh path (True) and the dead "
    "SimulationApp path (False). Set before renderer init, not after.",
)
parser.add_argument(
    "--start-orchestrator",
    action="store_true",
    help="Call rep.orchestrator.run() before stepping. Under runheadless.sh the "
    "orchestrator is STARTED on every frame and capture works; under "
    "SimulationApp it is STOPPED and capture is empty. This tests whether "
    "simply starting it closes that gap. NOT the same as step(), which hangs.",
)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

_cfg = {
    "headless": True,
    "enable_motion_bvh": True,
    "multi_gpu": False,
    "active_gpu": 0,
    "physics_gpu": 0,
    "width": 256,
    "height": 256,
    "samples_per_pixel_per_frame": 1,
    "denoiser": False,
    "anti_aliasing": 0,
    # Force Kit's own log somewhere that survives the container, at a level
    # that includes [Error]. Without this the interesting failures -- Vulkan
    # external-memory export, gpu.foundation -- never reach stdout.
    "extra_args": [
        "--/log/enabled=true",
        f"--/log/level={args.log_level}",
        f"--/log/file={args.kit_log}",
        "--/log/flushStandardStreamOutput=true",
    ]
    + (["--/app/asyncRendering=true", "--/app/asyncRenderingLowLatency=true"] if args.force_async else []),
}
print(f"[diag] experience = {args.experience or '<default>'}", flush=True)
print(f"[diag] kit log    = {args.kit_log}", flush=True)
simulation_app = SimulationApp(_cfg, experience=args.experience)

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.experimental.objects import Cube, DistantLight  # noqa: E402
from isaacsim.core.experimental.utils.app import enable_extension  # noqa: E402
from isaacsim.sensors.experimental.rtx import (  # noqa: E402
    Lidar,
    LidarSensor,
    parse_generic_model_output_data,
)

enable_extension("isaacsim.sensors.rtx.nodes")

DistantLight("/World/light").set_intensities(3000.0)
Cube("/World/cube_front", positions=np.array([0.0, 0.0, 0.0]), scales=np.array([2.0, 2.0, 2.0]))
Cube("/World/cube_side", positions=np.array([5.0, 0.0, 0.0]), scales=np.array([2.0, 2.0, 2.0]))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

# --- camera: does anything render at all? ---------------------------------
cam = rep.create.camera(position=(0, 0, 12), look_at=(0, 0, 0))
cam_rp = rep.create.render_product(cam, resolution=(128, 128))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([cam_rp])

# --- RTX lidar: the thing the spike actually needs -------------------------
lidar = Lidar.create("/World/lidar", config="Example_Rotary", translations=np.array([0.0, 0.0, 1.0]))
sensor = LidarSensor(lidar, annotators=["generic-model-output"])

# --- is the renderer even switched on? ------------------------------------
# Universally empty annotators with ZERO errors is the signature of a renderer
# that was never asked to run, not one that failed. These settings say which.
import carb  # noqa: E402

_s = carb.settings.get_settings()
print("\n=== RENDERER STATE ===", flush=True)
for key in (
    "/app/renderer/enabled",
    "/app/renderer/skipWhileMinimized",
    "/app/renderer/resolution/width",
    "/app/renderer/resolution/height",
    "/app/hydraEngine/waitIdle",
    "/app/asyncRendering",
    "/omni/replicator/captureOnPlay",
    "/rtx/rendermode",
):
    print(f"  {key} = {_s.get(key)}", flush=True)

try:
    print(f"  orchestrator status = {rep.orchestrator.get_status()}", flush=True)
except Exception as exc:
    print(f"  orchestrator status unavailable: {type(exc).__name__}: {exc}", flush=True)

try:
    import omni.usd
    from pxr import UsdRender

    _stage = omni.usd.get_context().get_stage()
    for _p in _stage.Traverse():
        if _p.IsA(UsdRender.Product):
            print(f"  render product prim: {_p.GetPath()} active={_p.IsActive()}", flush=True)
except Exception as exc:
    print(f"  render product scan failed: {type(exc).__name__}: {exc}", flush=True)

best_rgb = 0
best_gmo = 0
first_rgb_frame = None
first_gmo_frame = None

# Sample the orchestrator on EVERY frame. "STOPPED when queried once" is much
# weaker evidence than "never left STOPPED", and only the per-frame histogram
# can tell those apart.
_status_hist: dict[str, int] = {}

if args.start_orchestrator:
    try:
        rep.orchestrator.run()
        print(f"orchestrator.run() -> status now {rep.orchestrator.get_status()}", flush=True)
    except Exception as exc:
        print(f"orchestrator.run() raised {type(exc).__name__}: {exc}", flush=True)

for i in range(args.frames):
    simulation_app.update()

    try:
        _st = str(rep.orchestrator.get_status())
    except Exception as exc:
        _st = f"<unavailable: {type(exc).__name__}>"
    _status_hist[_st] = _status_hist.get(_st, 0) + 1

    arr = np.asarray(rgb.get_data())
    if arr.size:
        nz = int((arr != 0).sum())
        if nz > best_rgb:
            best_rgb = nz
            if first_rgb_frame is None:
                first_rgb_frame = i

    try:
        buf, _ = sensor.get_data("generic-model-output")
    except Exception:
        buf = None
    if buf is not None:
        gmo = parse_generic_model_output_data(buf)
        if gmo is not None and int(gmo.numElements) > best_gmo:
            best_gmo = int(gmo.numElements)
            if first_gmo_frame is None:
                first_gmo_frame = i

print("\n=== RESULT ===", flush=True)
print(f"experience      : {args.experience or '<default>'}", flush=True)
print(f"camera rgb      : max nonzero px = {best_rgb} (first at frame {first_rgb_frame})", flush=True)
print(f"lidar GMO       : max numElements = {best_gmo} (first at frame {first_gmo_frame})", flush=True)
print(f"orchestrator    : per-frame status histogram = {_status_hist}", flush=True)
print(
    "VERDICT         : "
    + (
        "BOTH OK -- use this experience"
        if best_rgb and best_gmo
        else "CAMERA ONLY -- render works, RTX sensor does not"
        if best_rgb
        else "STILL DEAD"
    ),
    flush=True,
)

simulation_app.close()
