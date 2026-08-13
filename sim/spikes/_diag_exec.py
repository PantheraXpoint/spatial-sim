"""Launcher bisect: run the capture test inside the KNOWN-GOOD launcher.

Throwaway debugging aid for S4.

Every capture test so far ran under ``SimulationApp``, which launches Kit with
``--portable`` and its own arg set. Streaming renders on this host under
``runheadless.sh``. Loading the streaming *experience* through SimulationApp is
NOT the same test -- it changes the experience file while leaving the launcher
untouched, and the launcher is the variable nobody has controlled.

This script is designed to be run by that launcher via Kit's ``--exec``:

    ./runheadless.sh --exec /workspace/sim/spikes/_diag_exec.py

so it must NOT construct a SimulationApp -- Kit is already up.

If the annotator fills here, the fault is in the standalone launch path and the
problem collapses to a config diff between the two launchers. If it stays
empty, the stopped-orchestrator lead is the right thread.

Drives frames from the update event stream rather than calling app.update() in
a loop, because calling update() from inside an --exec script re-enters the
main loop.
"""

from __future__ import annotations

import carb
import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline

FRAMES = 60

_s = carb.settings.get_settings()
print("\n" + "=" * 70, flush=True)
print("LAUNCHER BISECT -- running inside runheadless.sh's Kit, not SimulationApp", flush=True)
print("=" * 70, flush=True)
print(f"  /app/asyncRendering           = {_s.get('/app/asyncRendering')}", flush=True)
print(f"  /app/asyncRenderingLowLatency = {_s.get('/app/asyncRenderingLowLatency')}", flush=True)
print(f"  /omni/replicator/asyncRendering = {_s.get('/omni/replicator/asyncRendering')}", flush=True)
print(f"  /omni/replicator/captureOnPlay  = {_s.get('/omni/replicator/captureOnPlay')}", flush=True)
print(f"  /renderer/multiGpu/enabled      = {_s.get('/renderer/multiGpu/enabled')}", flush=True)

from isaacsim.core.experimental.objects import Cube, DistantLight  # noqa: E402
from isaacsim.core.experimental.utils.app import enable_extension  # noqa: E402

enable_extension("isaacsim.sensors.rtx.nodes")

from isaacsim.sensors.experimental.rtx import (  # noqa: E402
    Lidar,
    LidarSensor,
    parse_generic_model_output_data,
)

DistantLight("/World/light").set_intensities(3000.0)
Cube("/World/cube_a", positions=np.array([0.0, 0.0, 0.0]), scales=np.array([2.0, 2.0, 2.0]))
Cube("/World/cube_b", positions=np.array([5.0, 0.0, 0.0]), scales=np.array([2.0, 2.0, 2.0]))

cam = rep.create.camera(position=(0, 0, 12), look_at=(0, 0, 0))
cam_rp = rep.create.render_product(cam, resolution=(128, 128))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([cam_rp])

lidar = Lidar.create("/World/lidar", config="Example_Rotary", translations=np.array([0.0, 0.0, 1.0]))
sensor = LidarSensor(lidar, annotators=["generic-model-output"])

omni.timeline.get_timeline_interface().play()

# --------------------------------------------------------------------------
# Per-frame sampling. The open question is not "what is the orchestrator status
# at the end" but "was it EVER anything other than STOPPED" -- so record the
# status on every single frame, not once after the fact.
# --------------------------------------------------------------------------
state = {"frame": 0, "best_rgb": 0, "best_gmo": 0, "statuses": {}, "sub": None}


def _on_update(_e) -> None:
    st = state
    st["frame"] += 1

    try:
        s = str(rep.orchestrator.get_status())
    except Exception as exc:
        s = f"<unavailable: {type(exc).__name__}>"
    st["statuses"][s] = st["statuses"].get(s, 0) + 1

    arr = np.asarray(rgb.get_data())
    if arr.size:
        st["best_rgb"] = max(st["best_rgb"], int((arr != 0).sum()))

    try:
        buf, _ = sensor.get_data("generic-model-output")
    except Exception:
        buf = None
    if buf is not None:
        gmo = parse_generic_model_output_data(buf)
        if gmo is not None:
            st["best_gmo"] = max(st["best_gmo"], int(gmo.numElements))

    if st["frame"] >= FRAMES:
        print("\n" + "=" * 70, flush=True)
        print("LAUNCHER BISECT RESULT", flush=True)
        print("=" * 70, flush=True)
        print(f"  frames sampled       : {st['frame']}", flush=True)
        print(f"  camera rgb max px    : {st['best_rgb']}", flush=True)
        print(f"  lidar GMO max points : {st['best_gmo']}", flush=True)
        print(f"  orchestrator status  : {st['statuses']}", flush=True)
        verdict = (
            "CAPTURE WORKS under runheadless.sh -> fault is the SimulationApp launch path"
            if (st["best_rgb"] or st["best_gmo"])
            else "STILL DEAD under runheadless.sh -> launcher is not the variable"
        )
        print(f"  VERDICT              : {verdict}", flush=True)
        st["sub"] = None
        omni.kit.app.get_app().post_quit()


state["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    _on_update, name="diag_exec_capture"
)
print(f"\nsubscribed; sampling {FRAMES} frames from the update stream", flush=True)
