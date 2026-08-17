"""Demo prep: pick a clean spawn, and LOOK at the three robots.

1. SPAWN. Derive a floor position that is clear of obstacles, at least 2.5 m
   from the static Worker (so the demo does not open with two identical men
   inside each other), 7-12 m from INFRA_01 (inside the lidar's usable zone,
   which starts at 6.34 m and where the avatar was measured at 1,406 returns),
   and clear of the three robots.

2. ROBOTS. The bbox pass said "upright" and the eye says otherwise, so this
   renders a frame and dumps joint geometry:
     a. H1 leg links -- hip/knee/ankle heights say sitting vs standing, which
        a width/height ratio cannot.
     b. Go2 mesh census -- are the leg prims present and visible after full
        composition, or does GEOMETRY need the same payload barrier its
        physics needed?
     c. TurtleBot mesh census and measured height, to tell "correctly small"
        from "half-loaded".

Exec mode.  SF_STAGE, SF_OUT
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for _p in (str(REPO), str(REPO / "sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
os.environ["SF_NO_AUTORUN"] = "1"

import sensor_factory as sf  # noqa: E402

STAGE = os.environ.get("SF_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
OUT = Path(os.environ.get("SF_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
WORKER = "/Root/Worker"

S: dict = {"phase": "loading", "frame": 0, "n": 0, "sub": None, "result": {}}


def log(m: str) -> None:
    print(f"[demo_prep] {m}", flush=True)


def world_xy(stage, path):
    t = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path)).ExtractTranslation()
    return float(t[0]), float(t[1])


def pick_spawn(stage, station, robots):
    """A spot that is clear, away from the Worker, and inside the lidar's zone."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    obstacles = []
    for prim in stage.Traverse():
        p = prim.GetPath().pathString
        if not p.startswith("/Root/Warehouse") or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        try:
            r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            lo_z, hi_z = float(r.GetMin()[2]), float(r.GetMax()[2])
            if lo_z > 1.2 or hi_z < 0.30:      # floors are not obstacles
                continue
            obstacles.append((float(r.GetMin()[0]), float(r.GetMin()[1]),
                              float(r.GetMax()[0]), float(r.GetMax()[1])))
        except Exception:
            continue
    wx, wy = world_xy(stage, WORKER)
    log(f"worker at ({wx:.3f}, {wy:.3f}); {len(obstacles)} obstacles; station {station}")

    best = None
    for radius in (3.0, 3.5, 4.0, 4.5, 5.0):
        for deg in range(0, 360, 5):
            a = math.radians(deg)
            x, y = wx + radius * math.cos(a), wy + radius * math.sin(a)
            pad = 0.5
            if any(x0 - pad <= x <= x1 + pad and y0 - pad <= y <= y1 + pad
                   for x0, y0, x1, y1 in obstacles):
                continue
            d_station = math.hypot(x - station[0], y - station[1])
            if not (7.0 <= d_station <= 12.0):
                continue
            if any(math.hypot(x - rx, y - ry) < 1.8 for rx, ry in robots):
                continue
            score = abs(d_station - 8.5)          # aim for the middle of the zone
            if best is None or score < best[0]:
                best = (score, x, y, radius, d_station)
        if best:
            break
    if best is None:
        log("! no spawn satisfied every constraint")
        return None
    _, x, y, radius, d_station = best
    log(f"SPAWN ({x:.3f}, {y:.3f}) -- {radius:.1f} m from the Worker, "
        f"{d_station:.2f} m from INFRA_01")
    return {"xy": [round(x, 3), round(y, 3)], "from_worker_m": round(radius, 2),
            "from_station_m": round(d_station, 2)}


def robot_report(stage, rid, path):
    """Mesh census plus the joint geometry a bbox cannot show."""
    prim = stage.GetPrimAtPath(path)
    out = {"prim_path": path, "valid": bool(prim.IsValid())}
    if not prim.IsValid():
        return out
    meshes = [p for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Mesh)]
    visible = [p for p in meshes
               if UsdGeom.Imageable(p).ComputeVisibility() != UsdGeom.Tokens.invisible]
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    size = rng.GetSize()
    out.update({
        "meshes": len(meshes), "visible_meshes": len(visible),
        "prims": len(list(Usd.PrimRange(prim))),
        "size_m": [round(float(v), 3) for v in size],
        "z_min": round(float(rng.GetMin()[2]), 3),
        "z_max": round(float(rng.GetMax()[2]), 3),
    })
    legs = []
    for p in Usd.PrimRange(prim):
        n = p.GetName().lower()
        if any(k in n for k in ("hip", "knee", "ankle", "foot", "thigh", "calf", "leg")):
            t = UsdGeom.XformCache().GetLocalToWorldTransform(p).ExtractTranslation()
            legs.append((p.GetName(), round(float(t[2]), 3)))
    out["leg_link_heights"] = legs[:16]
    log(f"{rid}: {out['meshes']} meshes ({out['visible_meshes']} visible), "
        f"{out['prims']} prims, size {out['size_m']}, z {out['z_min']}..{out['z_max']}")
    for n, z in legs[:10]:
        log(f"    leg link {n:34s} z={z}")
    return out


def render(stage, robots):
    """Frame the robots and capture, because looking is the only settling test."""
    if not robots:
        return None
    cx = sum(r[0] for r in robots) / len(robots)
    cy = sum(r[1] for r in robots) / len(robots)
    eye = Gf.Vec3d(cx - 4.5, cy - 4.5, 2.4)
    cam = UsdGeom.Camera.Define(stage, "/Root/_Diag/robot_cam")
    cam.AddTranslateOp().Set(eye)
    cam.AddRotateXYZOp().Set(sf.look_at_rotate_xyz(eye, Gf.Vec3d(cx, cy, 0.5)))
    cam.CreateFocalLengthAttr(22.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1_000_000.0))
    try:
        from omni.kit.viewport.utility import get_active_viewport

        vp = get_active_viewport()
        vp.set_active_camera("/Root/_Diag/robot_cam")
        return vp
    except Exception as exc:
        log(f"! viewport unavailable: {exc!r}")
        return None


def setup():
    stage = omni.usd.get_context().get_stage()
    sf.create_stations(stage)
    S["made"] = sf.create_registry_sensors(stage, sf.load_registry(),
                                           render_products=False, attach_annotators=False)
    import avatar as av

    S["follow"] = av.install_character_follow(stage)
    sf.disable_unreachable_colliders(stage)
    paths = sf.reference_robots(stage)
    S["robot_paths"] = paths
    S["station"] = [float(v) for v in sf.load_stations()[0]["stage_position"]][:2]
    S["phase"] = "settling"
    S["settle"] = 0


def after_settle():
    stage = omni.usd.get_context().get_stage()
    sf.pin_robots_static(stage, S["robot_paths"])
    reports = {rid: robot_report(stage, rid, path) for rid, path in S["robot_paths"].items()}
    S["result"]["robots"] = reports

    # VERIFY the two fixes rather than assume them.
    for rid, rep in reports.items():
        ok = abs(rep["z_min"]) < 0.05
        log(f"  {rid} rests on the floor: {ok}  (z_min {rep['z_min']})")
        rep["on_floor"] = ok

    ax, ay = world_xy(stage, "/Root/Avatar/character")
    wx, wy = world_xy(stage, WORKER)
    gap = math.hypot(ax - wx, ay - wy)
    d_station = math.hypot(ax - S["station"][0], ay - S["station"][1])
    S["result"]["spawn_check"] = {
        "avatar_xy": [round(ax, 3), round(ay, 3)],
        "worker_xy": [round(wx, 3), round(wy, 3)],
        "gap_from_worker_m": round(gap, 3),
        "distance_to_station_m": round(d_station, 3),
        "clear_of_worker": gap > 1.5,
        "inside_lidar_zone": 6.5 <= d_station <= 15.0,
    }
    log(f"  avatar spawns at ({ax:.3f}, {ay:.3f}); {gap:.2f} m from the Worker, "
        f"{d_station:.2f} m from INFRA_01")

    robots_xy = [world_xy(stage, p) for p in S["robot_paths"].values()]
    S["viewport"] = render(stage, robots_xy)
    omni.timeline.get_timeline_interface().play()
    log("play()")
    S["phase"] = "rendering"
    S["n"] = 0


def finish():
    # Does the lidar read the avatar AT SPAWN? That is the whole point of
    # putting the spawn inside the usable zone.
    import sensor_inspector as si

    stage = omni.usd.get_context().get_stage()
    rec = S.get("made", {}).get("INFRA_01_LIDAR")
    if rec is not None:
        stats = si.read_stats(rec, stage)
        S["result"]["at_spawn"] = {
            "points_on_avatar": stats.get("points_on_avatar"),
            "avatar_range_m": stats.get("avatar_range_m"),
            "points": stats.get("points"),
        }
        log(f"  AT SPAWN: points_on_avatar={stats.get('points_on_avatar')} "
            f"avatar_range={stats.get('avatar_range_m')} m")

    vp = S.get("viewport")
    if vp is not None:
        try:
            from omni.kit.viewport.utility import capture_viewport_to_file

            path = str(OUT / "demo_prep_robots.png")
            capture_viewport_to_file(vp, file_path=path)
            S["result"]["png"] = path
            log(f"capture -> {path}")
        except Exception as exc:
            log(f"! capture failed: {exc!r}")
    (OUT / "demo_prep.json").write_text(json.dumps(S["result"], indent=1, default=str))
    log("DONE")


def on_update(_e):
    S["frame"] += 1
    try:
        ph = S["phase"]
        if ph == "loading":
            if S["frame"] > 5 and not any(omni.usd.get_context().get_stage_loading_status()[1:]):
                setup()
            return
        if ph == "settling":
            S["settle"] += 1
            quiet = not any(omni.usd.get_context().get_stage_loading_status()[1:])
            if S["settle"] > 120 or (quiet and S["settle"] > 45):
                log(f"payloads quiet after {S['settle']} frames")
                after_settle()
            return
        if ph == "rendering":
            S["n"] += 1
            if S["n"] == 90:
                finish()
            if S["n"] >= 120:
                S["phase"] = "done"
                S["sub"] = None
                omni.kit.app.get_app().post_quit()
            return
    except Exception as exc:
        import traceback

        log(f"FAILED: {exc!r}")
        log(traceback.format_exc())
        S["sub"] = None
        omni.kit.app.get_app().post_quit()


log(f"open_stage {STAGE} -> {omni.usd.get_context().open_stage(STAGE)}")
S["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    on_update, name="demo_prep"
)
