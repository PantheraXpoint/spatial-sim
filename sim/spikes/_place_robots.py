"""S9 groundwork: can the three robot platforms be placed, and do they see the avatar?

Everything here is discovery plus verification, and it refuses to guess:

  1. RESOLVE the asset URLs. config/scene.yaml carries paths "from the plan"
     which its own header says must be confirmed. BOT_03 (H1) has no asset at
     all, so the Robots directory is listed to find one. Anything that does not
     resolve is REPORTED, not substituted.
  2. DERIVE floor positions near the avatar, by testing candidate points for
     clearance against the warehouse's own colliders. A spot with a rack in it
     is not a spot.
  3. PLACE each robot static -- every rigid body kinematic -- because legged
     robots collapse on spawn without a locomotion policy.
  4. VERIFY at Play that they stayed upright, by comparing each robot's world
     bbox height before and after. Collapse is a height drop, and it is
     measurable rather than a thing to assume.
  5. FRAME CHECK each robot camera against the avatar: elevation from the
     camera to the avatar's head, torso and feet, then a segmentation capture
     counting 'person' pixels. The point of S9 is the contrast between heights,
     so a camera that cannot see the avatar at all is a finding, not a panel.

Writes its chosen positions to sim/robot_placement.json for transcription into
config/scene.yaml -- deliberately NOT written into the config automatically,
because a derived position is a proposal and the config is the contract.

Exec mode.  SF_STAGE, SF_OUT, SF_FRAMES
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
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

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
FRAMES = int(os.environ.get("SF_FRAMES", "120"))
ROBOT_ROOT = "/Root/Robots"

S: dict = {"phase": "loading", "frame": 0, "n": 0, "sub": None, "robots": {}, "follow": None}


def log(m: str) -> None:
    print(f"[robots] {m}", flush=True)


def resolve_assets():
    """Which robot assets actually exist. Reports; never substitutes."""
    from isaacsim.storage.native import get_assets_root_path
    import omni.client

    root = get_assets_root_path()
    log(f"asset root {root}")
    out = {"root": root, "resolved": {}, "missing": [], "listing": []}

    with open(REPO / "config" / "scene.yaml", encoding="utf-8") as fh:
        import yaml

        robots = yaml.safe_load(fh).get("robots") or []

    for r in robots:
        rid, rel = r.get("id"), r.get("asset")
        if not rel:
            out["missing"].append({"id": rid, "reason": "asset is null in scene.yaml"})
            continue
        for candidate in (f"{root}/Isaac/{rel}", f"{root}/{rel}"):
            try:
                res, _ = omni.client.stat(candidate)
                if res == omni.client.Result.OK:
                    out["resolved"][rid] = candidate
                    break
            except Exception:
                pass
        else:
            out["missing"].append({"id": rid, "reason": f"does not resolve: {rel}"})

    # For anything unresolved, list what IS there rather than guess a filename.
    try:
        for sub in ("Isaac/Robots/Unitree/H1", "Isaac/Robots/Unitree/H1/Props"):
            res, entries = omni.client.list(f"{root}/{sub}")
            if res == omni.client.Result.OK:
                names = sorted(e.relative_path for e in entries)
                out["listing"].append({sub: names[:40]})
                usds = [n for n in names if n.endswith(".usd") or n.endswith(".usda")]
                if usds and "BOT_03" not in out["resolved"]:
                    out["h1_candidates"] = [f"{root}/{sub}/{n}" for n in usds]
    except Exception as exc:
        out["listing"].append({"error": repr(exc)})

    log(f"resolved {out['resolved']}")
    log(f"missing  {out['missing']}")
    for entry in out["listing"]:
        log(f"listing  {json.dumps(entry)[:400]}")
    return out


def clear_positions(stage, centre, count, radii=(2.6, 3.2, 4.0, 5.0, 6.5, 8.0)):
    """Floor points near the avatar with nothing already occupying them.

    Derived, not chosen: each candidate is tested against the warehouse's own
    collision geometry below 1.2 m. A point inside a rack is rejected.
    """
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    obstacles = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not path.startswith("/Root/Warehouse") or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        try:
            rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            lo_z, hi_z = float(rng.GetMin()[2]), float(rng.GetMax()[2])
            # An obstacle has to STAND somewhere: reach into the robot's height
            # band AND rise off the floor. Without the second test the floor
            # slabs qualify -- they start at z=0 and their bounding boxes cover
            # the whole warehouse, so every candidate point on earth is "inside
            # an obstacle" and nothing is ever placeable. That is exactly what
            # happened: 931 obstacles, zero clear positions at any radius.
            if lo_z > 1.2 or hi_z < 0.30:
                continue
            obstacles.append((float(rng.GetMin()[0]), float(rng.GetMin()[1]),
                              float(rng.GetMax()[0]), float(rng.GetMax()[1])))
        except Exception:
            continue
    log(f"  {len(obstacles)} low obstacles considered")

    chosen = []
    pad = 0.45
    for radius in radii:
        for deg in range(0, 360, 10):
            if len(chosen) >= count:
                break
            a = math.radians(deg)
            x = centre[0] + radius * math.cos(a)
            y = centre[1] + radius * math.sin(a)
            if any(x0 - pad <= x <= x1 + pad and y0 - pad <= y <= y1 + pad
                   for x0, y0, x1, y1 in obstacles):
                continue
            if any(math.hypot(x - cx, y - cy) < 1.6 for cx, cy in chosen):
                continue
            chosen.append((x, y))
            log(f"  clear at r={radius} {deg:3d} deg -> ({x:.3f}, {y:.3f})  "
                f"{math.hypot(x - centre[0], y - centre[1]):.2f} m from the avatar")
        if len(chosen) >= count:
            break
    if not chosen:
        log("  ! NO clear position found at any radius -- the avatar's spot is boxed in")
    return chosen


def reference_robot(stage, rid, url, pos):
    """Reference a robot. Does NOT pin it -- see pin_static() for why."""
    path = f"{ROBOT_ROOT}/{rid}"
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(pos[0], pos[1], 0.0))
    xf.GetPrim().GetReferences().AddReference(url)
    stage.Load(xf.GetPrim().GetPath())
    log(f"  referenced {rid} -> {path}")
    return path


def count_physics(stage, path):
    prim = stage.GetPrimAtPath(path)
    bodies = arts = 0
    if prim.IsValid():
        for p in Usd.PrimRange(prim):
            bodies += bool(p.HasAPI(UsdPhysics.RigidBodyAPI))
            arts += bool(p.HasAPI(UsdPhysics.ArticulationRootAPI))
    return bodies, arts


def pin_static(stage, path):
    """Make every rigid body kinematic and disable every articulation root.

    SEPARATED from referencing on purpose. The first attempt pinned immediately
    after AddReference and found 0 rigid bodies and 0 articulation roots on the
    Go2 -- then watched it sag 5.9 cm at Play. The asset's physics lives behind
    PAYLOADS that had not finished composing when the traversal ran, so there
    was literally nothing there to pin, and PhysX picked the articulation up
    later once it had loaded. Pinning now waits for the stage to stop loading.
    """
    prim = stage.GetPrimAtPath(path)
    bodies = arts = 0
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).CreateKinematicEnabledAttr().Set(True)
            bodies += 1
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            arts += 1
            a = p.GetAttribute("physxArticulation:articulationEnabled")
            if not a:
                a = p.CreateAttribute(
                    "physxArticulation:articulationEnabled", Sdf.ValueTypeNames.Bool)
            a.Set(False)
    log(f"  {path}: pinned {bodies} rigid bodies kinematic, {arts} articulation roots off")
    return bodies, arts


def robot_cameras(stage, placed):
    """One RGB-D camera per robot at its declared sensor height, aimed at the avatar.

    Attaches semantic segmentation so "can this platform see the avatar" is a
    PIXEL COUNT, not a guess. A panel of floor is a failure worth naming.
    """
    import omni.replicator.core as rep

    target = sf.avatar_target(stage)
    cams = {}
    # The station camera too: its 'person' pixel count is the measure of
    # whether stripping the asset's per-part labels made the whole silhouette
    # read as one person.
    station_cam = "/Root/Infrastructure/INFRA_01/cam_01"
    if stage.GetPrimAtPath(station_cam).IsValid():
        rp = rep.create.render_product(station_cam, resolution=(1280, 720))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([rp])
        seg = rep.AnnotatorRegistry.get_annotator(
            "semantic_segmentation", init_params={"colorize": False})
        seg.attach([rp])
        cams["INFRA_01_CAM"] = {"path": station_cam, "rgb": rgb, "seg": seg,
                                "height": 2.6, "best_rgb": 0, "best_person": 0}
        log("  camera INFRA_01_CAM (station, for the label-strip measurement)")
    for rid, rec in placed.items():
        h = float(next((r.get("sensor_height", 0.3) for r in _robot_cfg() if r["id"] == rid), 0.3))
        path = f"{rec['prim_path']}/cam"
        cam = UsdGeom.Camera.Define(stage, path)
        eye = Gf.Vec3d(rec["position"][0], rec["position"][1], h)
        cam.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, h))
        cam.AddRotateXYZOp().Set(sf.look_at_rotate_xyz(eye, target))
        cam.CreateFocalLengthAttr(18.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 1_000_000.0))
        rp = rep.create.render_product(path, resolution=(640, 480))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([rp])
        seg = rep.AnnotatorRegistry.get_annotator(
            "semantic_segmentation", init_params={"colorize": False})
        seg.attach([rp])
        cams[rid] = {"path": path, "rgb": rgb, "seg": seg, "height": h,
                     "best_rgb": 0, "best_person": 0}
        log(f"  camera {rid} at {h} m -> {path}")
    return cams


def sample_cameras():
    import numpy as np

    for rid, c in S.get("cams", {}).items():
        arr = np.asarray(c["rgb"].get_data())
        if arr.size:
            c["best_rgb"] = max(c["best_rgb"], int((arr != 0).sum()))
        seg = c["seg"].get_data()
        if isinstance(seg, dict) and seg.get("data") is not None:
            data = np.asarray(seg["data"])
            labels = (seg.get("info") or {}).get("idToLabels") or {}
            ids = [int(k) for k, v in labels.items() if "person" in json.dumps(v).lower()]
            if ids:
                c["best_person"] = max(c["best_person"], int(np.isin(data, ids).sum()))


def bbox_height(stage, path):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    if rng.IsEmpty():
        return None
    return round(float(rng.GetSize()[2]), 4)


def setup():
    stage = omni.usd.get_context().get_stage()
    sf.create_stations(stage)
    S["made"] = sf.create_registry_sensors(stage, sf.load_registry(),
                                           render_products=False, attach_annotators=False)
    S["follow"] = av.install_character_follow(stage)
    sf.disable_unreachable_colliders(stage)

    assets = resolve_assets()
    S["assets"] = assets

    target = sf.avatar_target(stage)
    S["target"] = [float(v) for v in target]
    spots = clear_positions(stage, (target[0], target[1]), count=len(assets["resolved"]) or 3)
    S["spots"] = spots

    placed = {}
    for (rid, url), pos in zip(sorted(assets["resolved"].items()), spots):
        path = reference_robot(stage, rid, url, pos)
        placed[rid] = {"prim_path": path, "asset": url,
                       "position": [round(pos[0], 3), round(pos[1], 3), 0.0]}
        b, a = count_physics(stage, path)
        placed[rid]["physics_at_reference"] = [b, a]
        log(f"  {rid} physics immediately after reference: {b} bodies, {a} articulations")
    S["placed"] = placed
    log(f"referenced {list(placed)} -- waiting for payloads before pinning")


def finish():
    stage = omni.usd.get_context().get_stage()
    log("=" * 72)
    for rid, rec in S.get("placed", {}).items():
        b, a = count_physics(stage, rec["prim_path"])
        rec["physics_after_play"] = [b, a]
        after = bbox_height(stage, rec["prim_path"])
        rec["height_after"] = after
        before = rec.get("height_before") or 0
        upright = after is not None and before and after > before * 0.8
        rec["upright"] = bool(upright)
        log(f"  {rid:8s} height {before} -> {after} m   "
            f"{'UPRIGHT' if upright else 'COLLAPSED / MOVED'}   "
            f"physics ref{rec.get('physics_at_reference')} "
            f"load{rec.get('physics_after_load')} pinned{rec.get('pinned')} "
            f"play{rec.get('physics_after_play')}")
        # Elevation from a camera at this robot's sensor height to the avatar.
        t = S["target"]
        h = float(next((r.get("sensor_height", 0.3) for r in _robot_cfg() if r["id"] == rid), 0.3))
        d = math.hypot(t[0] - rec["position"][0], t[1] - rec["position"][1])
        for label, z in (("head", 1.75), ("torso", 0.95), ("feet", 0.05)):
            el = math.degrees(math.atan2(z - h, d))
            log(f"      -> avatar {label:5s} elevation {el:+6.2f} deg at {d:.2f} m")
    for rid, c in S.get("cams", {}).items():
        log(f"  {rid:8s} camera at {c['height']} m: rgb {c['best_rgb']:,} px, "
            f"person {c['best_person']:,} px"
            + ("" if c["best_person"] else "   <== CANNOT SEE THE AVATAR"))
        if rid in S.get("placed", {}):
            S["placed"][rid]["camera"] = {"prim_path": c["path"], "height_m": c["height"],
                                          "rgb_px": c["best_rgb"], "person_px": c["best_person"]}
    payload = {"assets": S.get("assets"), "placed": S.get("placed"),
               "avatar_target": S.get("target")}
    (OUT / "robot_placement.json").write_text(json.dumps(payload, indent=1, default=str))
    try:
        (REPO / "sim" / "robot_placement.json").write_text(json.dumps(payload, indent=1, default=str))
    except Exception as exc:
        log(f"  ! could not write into the repo: {exc!r}")
    log("DONE")


def _robot_cfg():
    import yaml

    with open(REPO / "config" / "scene.yaml", encoding="utf-8") as fh:
        return yaml.safe_load(fh).get("robots") or []


def settle_and_pin():
    """Count physics again once loading is quiet, pin it, then start Play."""
    stage = omni.usd.get_context().get_stage()
    for rid, rec in S["placed"].items():
        b, a = count_physics(stage, rec["prim_path"])
        rec["physics_after_load"] = [b, a]
        log(f"  {rid} physics after payloads loaded: {b} bodies, {a} articulations "
            f"(was {rec['physics_at_reference']})")
        pb, pa = pin_static(stage, rec["prim_path"])
        rec["pinned"] = [pb, pa]
        rec["height_before"] = bbox_height(stage, rec["prim_path"])
        log(f"  {rid} height before Play: {rec['height_before']} m")
    S["cams"] = robot_cameras(stage, S["placed"])
    omni.timeline.get_timeline_interface().play()
    log("play()")


def on_update(_e):
    S["frame"] += 1
    try:
        if S["phase"] == "loading":
            if S["frame"] > 5 and not any(omni.usd.get_context().get_stage_loading_status()[1:]):
                setup()
                S["phase"], S["settle"] = "settling", 0
            return

        if S["phase"] == "settling":
            S["settle"] += 1
            quiet = not any(omni.usd.get_context().get_stage_loading_status()[1:])
            if S["settle"] > 90 or (quiet and S["settle"] > 30):
                log(f"payloads quiet after {S['settle']} frames")
                settle_and_pin()
                S["phase"] = "playing"
            return
        if S["phase"] == "playing":
            S["n"] += 1
            sample_cameras()
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
    on_update, name="place_robots"
)
