"""Open the observatory with every sensor built and every panel bound.

The one job: reduce tomorrow's GUI session to *dock and save*. It opens the
stage, builds the stations and sensors from config, creates one viewport per
camera and binds each viewport to its own sensor -- so nothing is left to do by
hand except dragging panels into place once and saving the layout.

Run it as the GUI session itself::

    make gui            # == runheadless.sh -v --enable omni.physx.cct \\
                        #      --exec /workspace/sim/gui_viewports.py

Unlike ``sim/sensor_factory.py`` this script **does not quit**. It subscribes to
the update stream, does its setup on the first fully-loaded frame,
unsubscribes, and leaves Kit running for you.

Once it is up, in this order
----------------------------
1. Drag the new viewport panels where you want them.
2. ``Window -> Layout -> Save Layout As...``  Save it. Load it next time and
   this step disappears for good.
3. **Then** press Play. Build the layout BEFORE Play.
4. Do not re-dock or rearrange while an RTX lidar sim is running -- that is the
   documented way to crash Isaac Sim. Press Stop first, rearrange, Play again.

Controls once you press Play: W/S forward/back, A/D left/right, E/Q up/down.
Movement is world-axis; the view does not turn (see sim/avatar.py).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import omni.kit.app
import omni.usd

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
# Kit's --exec runs this file under Kit's OWN sys.path, which contains neither
# the repo root nor this directory. sensor_factory.py already inserts the repo
# root so that `core` imports work; the sibling import needs this directory as
# well, or the script dies at line one and the GUI comes up on Kit's default
# World/Environment stage with no sensors and no explanation.
for _p in (str(REPO), str(HERE)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Importing sensor_factory must NOT run its capture: that would sample 120
# frames and post_quit() this session. See sensor_factory._is_exec_entrypoint.
os.environ["SF_NO_AUTORUN"] = "1"

import avatar as av  # noqa: E402  -- sibling module, see sys.path above
import sensor_factory as sf  # noqa: E402
from core.observation import Modality  # noqa: E402

STAGE = os.environ.get("SF_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
VIEW_W, VIEW_H = 640, 360


def log(msg: str) -> None:
    print(f"[gui_viewports] {msg}", flush=True)


def build_viewports(stage, registry, created: dict) -> list[str]:
    """One viewport per camera sensor, each bound to its own camera prim.

    Each RTX sensor must be attached to its own viewport or it silently does
    not simulate. Measured caveat: headless, the lidar produced 419k points
    with no viewport at all, because LidarSensor creates its own render
    product -- so the rule is about the GUI's render pipeline, not the sensor.
    The cameras genuinely need one each, and get one here.
    """
    try:
        from omni.kit.viewport.utility import create_viewport_window
    except Exception as exc:
        log(f"! viewport utility unavailable: {exc!r}")
        return []

    made = []
    for sensor_id, prim_path in created.items():
        if not stage.GetPrimAtPath(prim_path).IsValid():
            continue
        try:
            win = create_viewport_window(name=sensor_id, width=VIEW_W, height=VIEW_H)
            win.viewport_api.set_active_camera(prim_path)
            made.append(sensor_id)
            log(f"viewport '{sensor_id}' -> {prim_path}")
        except Exception as exc:
            log(f"! viewport for {sensor_id} failed: {exc!r}")
    return made


class Boot:
    def __init__(self) -> None:
        self.ctx = omni.usd.get_context()
        self.frame = 0
        self.sub = None
        self.done = False
        self.follow_sub = None

    def on_update(self, _e) -> None:
        if self.done:
            return
        self.frame += 1
        if self.frame <= 5 or any(self.ctx.get_stage_loading_status()[1:]):
            return
        self.done = True
        try:
            self.setup()
        except Exception as exc:
            import traceback

            log("FAILED: " + repr(exc))
            log(traceback.format_exc())
        finally:
            self.sub = None

    def setup(self) -> None:
        stage = self.ctx.get_stage()
        # Report the stage root explicitly. If the script had died on import,
        # Kit would sit on its default World/Environment stage and look
        # superficially fine -- this line is how you tell the two apart.
        default_prim = stage.GetDefaultPrim()
        root = default_prim.GetPath().pathString if default_prim else "<none>"
        log(f"stage root: {root}   ({len(list(stage.Traverse()))} prims)")
        if root != "/Root":
            log(f"! expected /Root -- this is not the observatory stage")

        registry = sf.load_registry()

        # Stations first, or nothing under them resolves.
        stations = sf.create_stations(stage)
        for path, pos in stations.items():
            log(f"station {path} at {[round(v, 3) for v in pos]}")

        # Annotators off: this session is for looking, and the viewports are
        # what you look through. sensor_factory attaches them when capturing.
        created = sf.create_registry_sensors(
            stage, registry, attach_annotators=False, render_products=False
        )
        for sensor_id, rec in created.items():
            log(f"sensor {sensor_id} -> {rec['prim_path']} ({rec['kind']})")

        skipped = [s.sensor_id for s in registry if s.modality is Modality.RADAR]
        if skipped:
            log(f"radar not created (needs the three Motion BVH kit flags): {skipped}")

        # The visible character follows the collision capsule from here, in
        # Python. The OmniGraph version was correctly wired and copied a
        # constant -- OmniGraph reads Fabric, the controller writes USD. The
        # subscription is stashed on the instance because dropping it
        # unsubscribes and the body silently stops following.
        self.follow_sub = av.install_character_follow(stage)
        if self.follow_sub is None:
            log("! character follow NOT installed -- the body will not move with the capsule")

        cams = {sid: rec["prim_path"] for sid, rec in created.items() if rec["kind"] == "camera"}
        made = build_viewports(stage, registry, cams)

        log("=" * 68)
        log(f"READY -- {len(created)} sensors, {len(made)} viewport panels bound, "
            f"follow={'on' if self.follow_sub else 'OFF'}")
        log("1. drag the panels into place")
        log("2. Window -> Layout -> Save Layout As...")
        log("3. THEN press Play. Never re-dock while a lidar sim is running.")
        log("=" * 68)


boot = Boot()
omni.usd.get_context().open_stage(STAGE)
log(f"opening {STAGE}")
boot.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    boot.on_update, name="gui_viewports"
)
