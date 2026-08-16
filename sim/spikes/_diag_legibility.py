"""Can a human SEE the sensors and the lidar returns? Verified by looking.

Two legibility failures reported from the GUI:
  - the station is an Xform, so there is nothing to point at
  - 419,000 lidar points were invisible in the main viewport

Both are visual claims, so this settles them visually: it builds the scene,
presses Play, and CAPTURES THE VIEWPORT TO PNG. The images are the evidence.
A point count cannot tell you whether something is visible against a grey
floor; a picture can.

It also reports the three candidate causes for the invisible point cloud, so a
negative result is diagnosable rather than just disappointing:
  - is omni.debugdraw actually enabled in this app?
  - did the writer attach, and with what size/colour?
  - how many points, and how many on the avatar (i.e. was the band the issue)?

CAVEAT, stated up front: debug draw is a viewport overlay, and an overlay does
not necessarily survive a capture. If the PNG shows no points but the counts
are healthy, that is inconclusive for the GUI, not proof of absence -- and the
fallback is to draw the cloud as real UsdGeom.Points, which is guaranteed to
render anywhere.

Exec mode. Environment:
    SF_STAGE, SF_OUT, SF_LIDAR_PT_SIZE, SF_LIDAR_PT_COLOR, SF_FRAMES
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for _p in (str(REPO), str(REPO / "sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
os.environ["SF_NO_AUTORUN"] = "1"

import avatar as av  # noqa: E402
import sensor_factory as sf  # noqa: E402

STAGE = os.environ.get("SF_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
OUT = Path(os.environ.get("SF_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
FRAMES = int(os.environ.get("SF_FRAMES", "150"))

S: dict = {"phase": "loading", "frame": 0, "n": 0, "sub": None, "follow": None}


def log(m: str) -> None:
    print(f"[legibility] {m}", flush=True)


def ext_enabled(name: str) -> bool:
    try:
        return omni.kit.app.get_app().get_extension_manager().is_extension_enabled(name)
    except Exception:
        return False


def setup():
    stage = omni.usd.get_context().get_stage()
    log(f"stage root {stage.GetDefaultPrim().GetPath()}")

    for ext in ("omni.debugdraw", "isaacsim.util.debug_draw", "omni.physx.cct"):
        log(f"  extension {ext:28s} enabled={ext_enabled(ext)}")

    registry = sf.load_registry()
    stations = sf.create_stations(stage)
    log(f"stations {list(stations)}")
    made = sf.create_registry_sensors(stage, registry, render_products=False,
                                      attach_annotators=False)
    for sid, rec in made.items():
        extra = f" draw={rec.get('draw_writer')}" if rec["kind"] == "lidar" else ""
        log(f"  {sid} -> {rec['prim_path']} ({rec['kind']}){extra}")
    S["made"] = made
    log(f"point size={sf.LIDAR_PT_SIZE} colour={sf.LIDAR_PT_COLOR}")

    S["follow"] = av.install_character_follow(stage)

    # A main view that frames the avatar and the station together -- that is
    # the shot the legibility question is about.
    target = sf.avatar_target(stage)
    st_pos = stations.get("/Root/Infrastructure/INFRA_01") or [0, 0, 2.6]
    eye = Gf.Vec3d(float(st_pos[0]) - 9.0, float(st_pos[1]) - 7.0, 5.5)
    cam_path = "/Root/_Diag/overview_cam"
    cam = UsdGeom.Camera.Define(stage, cam_path)
    cam.AddTranslateOp().Set(eye)
    cam.AddRotateXYZOp().Set(sf.look_at_rotate_xyz(eye, target))
    cam.CreateFocalLengthAttr(16.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 1_000_000.0))

    try:
        from omni.kit.viewport.utility import get_active_viewport

        vp = get_active_viewport()
        vp.set_active_camera(cam_path)
        S["viewport"] = vp
        log(f"main viewport -> {cam_path}, resolution {vp.resolution}")
    except Exception as exc:
        log(f"! viewport unavailable: {exc!r}")

    omni.timeline.get_timeline_interface().play()
    log("play()")


def report_lidar():
    from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

    stage = omni.usd.get_context().get_stage()
    for sid, rec in S.get("made", {}).items():
        if rec["kind"] != "lidar":
            continue
        try:
            buf, _ = rec["sensor"].get_data("generic-model-output")
            gmo = parse_generic_model_output_data(buf) if buf is not None else None
        except Exception:
            gmo = None
        if gmo is None:
            continue
        m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(rec["prim_path"]))
        dec = sf.decode_gmo(gmo, m)
        pts = dec.pop("_points", None)
        hits = 0
        char = stage.GetPrimAtPath("/Root/Avatar/character")
        if pts is not None and char.IsValid():
            cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
            rng = cache.ComputeWorldBound(char).ComputeAlignedRange()
            hits = sf.count_in_box(pts, rng.GetMin(), rng.GetMax())
        best = S.setdefault("lidar_best", {})
        if dec.get("real", 0) >= best.get("real", 0):
            best.update(dec)
        best["avatar_hits"] = max(best.get("avatar_hits", 0), hits)


def capture():
    vp = S.get("viewport")
    if vp is None:
        log("! no viewport to capture")
        return
    try:
        from omni.kit.viewport.utility import capture_viewport_to_file

        path = str(OUT / "legibility_overview.png")
        capture_viewport_to_file(vp, file_path=path)
        log(f"capture requested -> {path}")
        S["captured"] = path
    except Exception as exc:
        log(f"! capture failed: {exc!r}")


def finish():
    best = S.get("lidar_best", {})
    log("=" * 72)
    log(f"lidar points {best.get('real')}   on the avatar {best.get('avatar_hits')}")
    log(f"point size {sf.LIDAR_PT_SIZE}  colour {sf.LIDAR_PT_COLOR}")
    log(f"captured {S.get('captured')}")
    (OUT / "legibility.json").write_text(json.dumps(
        {"lidar": best, "size": sf.LIDAR_PT_SIZE, "color": sf.LIDAR_PT_COLOR,
         "captured": S.get("captured")}, default=str))
    log("DONE")


def on_update(_e):
    S["frame"] += 1
    try:
        if S["phase"] == "loading":
            if S["frame"] > 5 and not any(omni.usd.get_context().get_stage_loading_status()[1:]):
                setup()
                S["phase"] = "playing"
            return
        if S["phase"] == "playing":
            S["n"] += 1
            if S["n"] % 10 == 0:
                report_lidar()
            if S["n"] == FRAMES - 30:
                capture()
            if S["n"] >= FRAMES:
                finish()
                S["phase"] = "done"
                S["sub"] = None
                omni.kit.app.get_app().post_quit()
    except Exception as exc:
        import traceback

        log(f"FAILED: {exc!r}")
        log(traceback.format_exc())
        S["sub"] = None
        omni.kit.app.get_app().post_quit()


log(f"open_stage {STAGE} -> {omni.usd.get_context().open_stage(STAGE)}")
S["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    on_update, name="legibility"
)
