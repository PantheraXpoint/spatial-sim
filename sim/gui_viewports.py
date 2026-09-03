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

Ten floor-level props are dynamic rigid bodies (config/scene.yaml ->
pushable_props): walk into a cone or a carton and it moves; walk into a rack,
a wall or the 60 kg drum and it does not. ``GUI_PUSHABLE=0`` puts them back to
static colliders. See sim/pushable_props.py for the impulse model and
sim/spikes/FINDINGS.md for what it costs.

The Worker gets a static collider and the three robots get DYNAMIC physics
proxies at their real masses -- Burger 1 kg, Go2 15 kg, H1 47 kg
(sim/nav_obstacles.py). Walk into the Burger and it skitters; walk into the H1
and it barely shifts. That is an amendment to CLAUDE.md's opening invariant and
to S9, both of which say so. The character controller is tuned here too --
step offset 0.04 m, slope limit 40 deg, climbing mode constrained -- and the
step offset is re-applied after every Play because OgnCharacterController
overwrites it with 0.01. ``GUI_NAV_OBSTACLES=0`` and ``GUI_CCT_TUNING=0``
restore the old behaviour for an A/B.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import omni.kit.app
import omni.timeline
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
import sensor_inspector as si  # noqa: E402
import nav_obstacles as no  # noqa: E402
import pushable_props as pp  # noqa: E402
from core.observation import Modality  # noqa: E402

STAGE = os.environ.get("SF_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
VIEW_W, VIEW_H = 640, 360
# Two panels by default: the main 3D view plus the station camera. The avatar's
# own cameras are opt-in, because every extra viewport is another full render
# of the scene and the demo reads fine without them.
AVATAR_CAMS = os.environ.get("GUI_AVATAR_CAMS") == "1"
ROBOT_CAMS = os.environ.get("GUI_ROBOT_CAMS") == "1"
# WHICH sensor the single camera panel shows. The height contrast IS the demo --
# the same walk seen from 0.2 m, 0.4 m, 1.7 m and 2.6 m -- so this is the knob
# that reaches it:
#     GUI_PANEL=BOT_01_CAM make gui     0.2 m, looks up at you
#     GUI_PANEL=BOT_02_CAM make gui     0.4 m
#     GUI_PANEL=BOT_03_CAM make gui     1.7 m, meets your face
#     GUI_PANEL=INFRA_01_CAM make gui   2.6 m, the wall station (default)
# You can also retarget it live without relaunching: the viewport panel's own
# camera menu (top-left of the panel) lists every sensor camera on the stage.
PANEL = os.environ.get("GUI_PANEL", "INFRA_01_CAM")
# Collision is the frame-rate lever, not resolution: measured 2.48 -> 19.69 fps
# from disabling unreachable colliders, and +0.8% from 1280x720 -> 960x540.
DISABLE_HIGH_COLLIDERS = os.environ.get("GUI_KEEP_ALL_COLLIDERS") != "1"
# The two gates that make the Play-state frame rate attributable. Measured
# 2026-08-24: 16.99 fps at Play with two panels + physics + the RTX lidar + its
# point-cloud draw, against 50 fps stopped with two panels and no lidar draw.
# Three costs -- physics, the lidar's ray casting, drawing ~419,000 points --
# are folded into one number with nothing separating them. These gates produce
# the two missing measurements:
#     GUI_LIDAR=0        no lidar created at all   (physics + panels only)
#     GUI_LIDAR_DRAW=0   lidar casts and returns points, nothing is drawn
# Take the difference between the three runs, at the same Play state, to
# attribute the cost. Both default ON, so an unset environment is exactly the
# behaviour every earlier session measured.
LIDAR = os.environ.get("GUI_LIDAR") != "0"
LIDAR_DRAW = os.environ.get("GUI_LIDAR_DRAW") != "0"
# The pushable props (config/scene.yaml -> pushable_props). Two gates rather
# than one, because they cost differently and separating them is what made the
# cost attributable. Measured 2026-09-01, headless, collider mask on, no
# annotator read on any arm:
#     10 dynamic bodies, avatar standing    +0.01 ms/frame   (they sleep)
#     10 dynamic bodies, walked through    +11.05 ms
#     ... plus the hit callback             +5.44 ms         (5 sweeps a step)
# The idle arms sit on the 60 fps cap, so those two deltas are lower bounds.
#     GUI_PUSHABLE=0        props stay static colliders (as shipped)
#     GUI_PUSH_CALLBACK=0   props are dynamic, nothing pushes them
# Both default ON. Derivation in sim/spikes/FINDINGS.md; the config's own
# `enabled: false` turns them off for every entry point at once.
PUSHABLE = os.environ.get("GUI_PUSHABLE") != "0"
PUSH_CALLBACK = os.environ.get("GUI_PUSH_CALLBACK") != "0"
# Static collision for the Worker and the three robots, and the character
# controller's own step offset / slope limit / climbing mode. Both default ON;
# GUI_NAV_OBSTACLES=0 and GUI_CCT_TUNING=0 restore the state in which the
# avatar walked through the Worker and climbed onto a barrel, which is the only
# reason the gates exist -- they are for A/B, not for production.
NAV_OBSTACLES = os.environ.get("GUI_NAV_OBSTACLES") != "0"
CCT_TUNING = os.environ.get("GUI_CCT_TUNING") != "0"


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
            # width/height above size the WINDOW. They say nothing about the
            # render target, which starts at the /app/renderer/resolution
            # default: measured 2026-08-24, every panel rendered 1280x720 and
            # downscaled into a 640x360 box, so ~75% of each panel's pixels
            # were computed and thrown away. GUI_ROBOT_CAMS=1 opens five of
            # them.
            #
            # The render resolution is a separate property on the viewport
            # API -- ViewportAPI.resolution, a (w, h) tuple with a real
            # setter (omni.kit.widget.viewport 109.2.0 in this image), which
            # writes through to the backing Hydra texture. It is also exactly
            # what isaacsim.core.rendering_manager's own
            # ViewportManager.create_viewport_window does after constructing
            # the window, for the same reason. Failing here costs frame rate,
            # not the panel, so it must not take the panel down with it.
            try:
                win.viewport_api.resolution = (VIEW_W, VIEW_H)
            except Exception as exc:
                log(f"! {sensor_id} render resolution not set -- panel stays "
                    f"at the renderer default: {exc!r}")
            made.append(sensor_id)
            log(f"viewport '{sensor_id}' -> {prim_path}")
        except Exception as exc:
            log(f"! viewport for {sensor_id} failed: {exc!r}")
    return made


def log_panel_resolutions(names: list[str]) -> None:
    """What each panel is ACTUALLY rendering, printed at READY.

    Read back from the live window rather than echoing the number we asked
    for: a set that silently did not take looks identical from the call site,
    and the viewport HUD was until now the only place the truth appeared.
    """
    try:
        from omni.kit.viewport.utility import get_viewport_from_window_name
    except Exception as exc:
        log(f"! panel resolutions unavailable: {exc!r}")
        return
    for name in names:
        try:
            api = get_viewport_from_window_name(name)
            if api is None:
                log(f"! panel {name}: no window found to read resolution from")
                continue
            res, full = api.resolution, api.full_resolution
            if not res or not full:
                log(f"! panel {name}: viewport reports no resolution yet")
                continue
            # .resolution is post-scale (what the renderer computes);
            # .full_resolution is what was requested. They differ only when
            # /app/renderer/resolution/multiplier is not 1.
            scaled = ""
            if (int(res[0]), int(res[1])) != (int(full[0]), int(full[1])):
                scaled = f" (requested {int(full[0])}x{int(full[1])}, scaled)"
            log(f"panel {name} renders {int(res[0])}x{int(res[1])} px{scaled}"
                f"  [asked for {VIEW_W}x{VIEW_H}]")
        except Exception as exc:
            log(f"! panel {name}: resolution readback failed: {exc!r}")


def log_lidar_config(created: dict) -> None:
    """Which of the lidar's costs are live, printed at READY.

    An fps number is only comparable to another session's if the configuration
    travels with it, and these two gates are what change it. Read back from the
    created records rather than echoing the environment: attach_writer can fail
    on its own, and a run whose draw failed is not the run GUI_LIDAR_DRAW=0 was
    meant to produce.
    """
    lidars = {sid: rec for sid, rec in created.items() if rec.get("kind") == "lidar"}
    gates = f"GUI_LIDAR={'1' if LIDAR else '0'} GUI_LIDAR_DRAW={'1' if LIDAR_DRAW else '0'}"
    if not lidars:
        why = "gated off" if not LIDAR else "none in the registry resolved"
        log(f"lidar config: NONE created ({why}) -- {gates}. "
            f"fps from this session carries no lidar cost.")
        return
    for sid, rec in lidars.items():
        log(f"lidar config: {sid} created, draw {rec.get('draw_writer', 'unknown')}"
            f" -- {gates}")


class Boot:
    def __init__(self) -> None:
        self.ctx = omni.usd.get_context()
        self.frame = 0
        self.sub = None
        self.done = False
        self.follow_sub = None
        self.robots: dict = {}
        self.inspector = None
        self.pushable: dict = {}
        self.push_cb = None
        self.nav: dict = {}
        self.cct_tuning = None
        self.proxy_follow = None
        self.pin_at = None
        self.fps_at = None
        self._t = None

    def _pin_pending(self) -> None:
        """Pin the robots once, after their payloads have had time to compose."""
        if self.pin_at is None or self.frame < self.pin_at:
            return
        self.pin_at = None
        stage = self.ctx.get_stage()
        if self.robots:
            sf.pin_robots_static(stage, self.robots)
            log("robots pinned static -- their own articulations stay disabled; "
                "a physics proxy carries them from here")

        # Everything physics-side that depends on a settled robot, in order.
        if NAV_OBSTACLES:
            self.nav = no.add_nav_obstacles(stage)
            self.proxy_follow = no.install_proxy_follow(stage, self.nav)
        else:
            log("nav obstacles SKIPPED (GUI_NAV_OBSTACLES=0) -- the avatar will "
                "walk through the Worker and the robots are back to being walls")
        if PUSHABLE and PUSH_CALLBACK:  # noqa: SIM102
            # Kept on the instance. Dropping the reference unsubscribes and the
            # boxes silently stop moving -- the same failure shape as
            # follow_sub. The robots' proxies go through the SAME callback, so
            # a 15 kg robot and a 15 kg crate get the same impulse.
            self.push_cb = pp.install_push_callback(
                stage, self.pushable,
                extra_bodies=no.dynamic_bodies(self.nav) if NAV_OBSTACLES else None)
            n_dyn = sum(1 for r in (self.nav.get("made") or {}).values()
                        if r.get("dynamic"))
            log("=" * 68)
            log(f"PHYSICS READY -- {len(self.pushable.get('made') or {})} pushable "
                f"props, {len(self.nav.get('made') or {})} nav obstacles "
                f"({n_dyn} of them dynamic robots), "
                f"follow={'on' if self.proxy_follow else 'OFF'}, "
                f"push={'on' if self.push_cb else 'OFF'}")
            log("=" * 68)
        elif PUSHABLE:
            log("push callback SKIPPED (GUI_PUSH_CALLBACK=0) -- props and robot "
                "proxies are dynamic but the avatar will not push them")

    def _log_fps(self) -> None:
        """A frame-rate line every ~300 frames, so the number in this session is
        observed rather than quoted from a different one."""
        import time

        if self.fps_at is None:
            return
        if self._t is None:
            self._t = (time.perf_counter(), self.frame)
            return
        if self.frame - self._t[1] < 300:
            return
        t0, f0 = self._t
        now = time.perf_counter()
        playing = omni.timeline.get_timeline_interface().is_playing()
        log(f"fps {round((self.frame - f0) / (now - t0), 2)}  "
            f"({'PLAYING' if playing else 'stopped'})")
        self._t = (now, self.frame)

    def on_update(self, _e) -> None:
        self.frame += 1
        if self.done:
            self._pin_pending()
            self._log_fps()
            return
        if self.frame <= 5 or any(self.ctx.get_stage_loading_status()[1:]):
            return
        self.done = True
        try:
            self.setup()
        except Exception as exc:
            import traceback

            log("FAILED: " + repr(exc))
            log(traceback.format_exc())
        # The subscription is deliberately KEPT. It used to be dropped here,
        # which unsubscribed the moment setup finished -- so the deferred robot
        # pin never fired and the frame-rate line never printed. Robots would
        # then collapse the first time you pressed Play, having been referenced
        # but never made kinematic. Cheap to keep: two comparisons a frame.

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

        # Robots first: their cameras are registry entries whose parent prims
        # have to exist before create_registry_sensors can resolve them.
        # Pinning waits -- see _pin_pending below.
        self.robots = sf.reference_robots(stage)

        # Stations, or nothing under them resolves either.
        stations = sf.create_stations(stage)
        for path, pos in stations.items():
            log(f"station {path} at {[round(v, 3) for v in pos]}")

        # Annotators off: this session is for looking, and the viewports are
        # what you look through. sensor_factory attaches them when capturing.
        # GUI_LIDAR=0 drops the lidar by narrowing the modality filter the
        # factory already takes, rather than by deleting the prim afterwards: a
        # sensor never created costs nothing, and nothing else in the registry
        # changes. The filter is built from the Modality enum, not from what
        # this registry happens to contain, so it can never come out empty --
        # which the factory would read as "no filter" and create the lidar
        # anyway. With the gate on, `modalities` stays None and the call is the
        # one every earlier session made.
        modalities = None if LIDAR else {m for m in Modality if m is not Modality.LIDAR}
        created = sf.create_registry_sensors(
            stage, registry, attach_annotators=False, render_products=False,
            modalities=modalities, lidar_draw=LIDAR_DRAW,
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
        # The controller's own settings, and they are re-applied after every
        # Play on purpose: OgnCharacterController writes stepOffset=0.01 when
        # the graph activates. Kept on the instance -- dropping it stops the
        # re-apply and the step offset silently reverts on the next Play.
        if CCT_TUNING:
            self.cct_tuning = av.install_controller_tuning(stage)
        else:
            log("controller tuning SKIPPED (GUI_CCT_TUNING=0) -- expect "
                "step_offset 0.01 m and climbing_mode 'easy' at Play")
        # S12 readout. Kept on the instance: dropping either reference stops
        # the panel updating, silently.
        try:
            self.inspector = si.install_inspector(stage, created)
        except Exception as exc:
            self.inspector = None
            log(f"! sensor inspector failed to install: {exc!r}")
        if self.follow_sub is None:
            log("! character follow NOT installed -- the body will not move with the capsule")

        if DISABLE_HIGH_COLLIDERS:
            sf.disable_unreachable_colliders(stage)
        else:
            log("collider mask SKIPPED (GUI_KEEP_ALL_COLLIDERS=1) -- expect ~2.5 fps at Play")

        # Pushable props LAST among the physics edits, and after the mask: the
        # mask walks every collider on the stage and switches off the ones out
        # of reach, and these props are all within reach, so the order is not
        # load-bearing -- but a prop that the mask had disabled would be a
        # dynamic body with no collision, which is the kind of silent nothing
        # this project keeps a list of. Running second makes that impossible.
        # Props here; the nav obstacles, the proxy follow and the push callback
        # all wait for _pin_pending. ONE authoring pass, after the robots have
        # been pinned and dropped onto the floor, because a proxy sized from a
        # bounding box taken before the drop is a metre underground and the
        # follow's reference pose would be taken from a robot that is about to
        # move. The push callback goes last of all because it needs both the
        # props and the robot proxies.
        if PUSHABLE:
            self.pushable = pp.make_pushable(stage)
        else:
            log("pushable props SKIPPED (GUI_PUSHABLE=0) -- every prop stays a "
                "static collider")

        cams = {sid: rec["prim_path"] for sid, rec in created.items() if rec["kind"] == "camera"}
        if PANEL not in cams:
            log(f"! GUI_PANEL={PANEL!r} is not a camera on this stage; "
                f"available: {', '.join(sorted(cams))}")
        wanted = {PANEL} if PANEL in cams else set()
        if AVATAR_CAMS:
            wanted |= {c for c in cams if c.startswith("AVATAR_")}
        if ROBOT_CAMS:
            wanted |= {c for c in cams if c.startswith("BOT_")}
        hidden = sorted(set(cams) - wanted)
        cams = {k: v for k, v in cams.items() if k in wanted}
        log(f"panel shows {PANEL} (GUI_PANEL=... to change, or use the panel's "
            f"own camera menu live)")
        if hidden:
            log(f"cameras created but not panelled: {', '.join(hidden)} "
                f"(GUI_AVATAR_CAMS=1 / GUI_ROBOT_CAMS=1)")
        made = build_viewports(stage, registry, cams)

        log("=" * 68)
        log(f"READY -- {len(created)} sensors, {len(made)} viewport panels bound, "
            f"follow={'on' if self.follow_sub else 'OFF'}, "
            f"inspector={'on' if self.inspector else 'OFF'}, "
            f"pushable={len(self.pushable.get('made') or {})} props "
            f"({'push on' if self.push_cb else 'push OFF'}), "
            f"cct={'tuned' if self.cct_tuning else 'SHIPPED'} "
            f"(nav obstacles and the push callback arrive at the robot pin, "
            f"~90 frames from now)")
        log_panel_resolutions(made)
        log_lidar_config(created)
        log("Sensor Inspector panel: select a sensor prim to see its live numbers")
        log("1. drag the panels into place")
        log("2. Window -> Layout -> Save Layout As...")
        log("3. THEN press Play. Never re-dock while a lidar sim is running.")
        log("=" * 68)
        self.pin_at = self.frame + 90   # give payloads time before pinning
        self.fps_at = self.frame + 30


boot = Boot()
omni.usd.get_context().open_stage(STAGE)
log(f"opening {STAGE}")
boot.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    boot.on_update, name="gui_viewports"
)
