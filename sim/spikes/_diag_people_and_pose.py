"""Two questions, one run: is there a walkable character, and how does H1 stand?

1. ISAAC/PEOPLE. Does the asset library ship a character on the SAME Reallusion
   CC skeleton the avatar already uses (RL_BoneRoot/Hip/..., 101 joints) and
   carrying a walk clip? If yes, walk-when-moving is a reference swap plus a
   blend. If the skeletons differ, it is a retargeting job and gets dropped.
   The test is concrete: list the directory, open each character's layer, and
   compare joint COUNT and joint NAMES against the Worker's.

2. H1 POSE. Bounding-box width against height tells a T-pose from a standing
   one -- a standing humanoid is ~0.5 m across, a T-pose is ~1.6 m. Then dump
   the shoulder/elbow/hand link transforms to say precisely what the arms are
   doing, and list any variant sets, because selecting a shipped standing
   variant would be the cheap fix and authoring joint drives would not.

Exec mode.  SF_STAGE, SF_OUT
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import omni.kit.app
import omni.usd
from pxr import Usd, UsdGeom, UsdSkel

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for _p in (str(REPO), str(REPO / "sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
os.environ["SF_NO_AUTORUN"] = "1"

import sensor_factory as sf  # noqa: E402

STAGE = os.environ.get("SF_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
OUT = Path(os.environ.get("SF_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
WORKER_SKEL = "/Root/Worker/ManRoot/Worker/Worker"

S: dict = {"phase": "loading", "frame": 0, "sub": None, "result": {}}


def log(m: str) -> None:
    print(f"[people_pose] {m}", flush=True)


def worker_skeleton(stage):
    """The skeleton the avatar already rides, for comparison."""
    prim = stage.GetPrimAtPath(WORKER_SKEL)
    if not prim.IsValid():
        return None
    joints = UsdSkel.Skeleton(prim).GetJointsAttr().Get()
    return [str(j) for j in joints] if joints else None


def scan_people(root: str, reference: list[str] | None):
    """List Isaac/People and compare every character's skeleton to the avatar's."""
    import omni.client

    out: dict = {"listing": [], "characters": [], "match": None}
    for sub in ("Isaac/People", "Isaac/People/Characters",
                "Isaac/People/Animations", "Isaac/People/MotionLibrary"):
        try:
            res, entries = omni.client.list(f"{root}/{sub}")
        except Exception as exc:
            out["listing"].append({sub: f"<error {exc!r}>"})
            continue
        if res != omni.client.Result.OK:
            out["listing"].append({sub: f"<{res}>"})
            continue
        names = sorted(e.relative_path for e in entries)
        out["listing"].append({sub: names[:60]})
        log(f"{sub}: {names[:30]}")

        for name in names:
            if name.startswith("."):
                continue
            url = f"{root}/{sub}/{name}"
            candidates = [url] if name.endswith((".usd", ".usda")) else [
                f"{url}/{name}.usd", f"{url}/{name}.usda"]
            for cand in candidates:
                try:
                    sub_stage = Usd.Stage.Open(cand)
                except Exception:
                    continue
                if sub_stage is None:
                    continue
                skels, anims, joints = [], [], None
                for p in sub_stage.Traverse():
                    t = str(p.GetTypeName())
                    if t == "Skeleton":
                        skels.append(p.GetPath().pathString)
                        j = UsdSkel.Skeleton(p).GetJointsAttr().Get()
                        if j and joints is None:
                            joints = [str(x) for x in j]
                    elif t == "SkelAnimation":
                        anims.append(p.GetPath().pathString)
                # An animation asset has clips and no skin; a character has
                # skin and (here) no clips. Both are interesting, so do not
                # require a skeleton before recording.
                if not skels and not anims:
                    continue
                same = (reference is not None and joints is not None
                        and len(joints) == len(reference) and joints == reference)
                # A clip is only usable without retargeting if its joint list
                # IS the avatar's -- same count and same names, in order.
                rec = {"asset": cand, "skeletons": len(skels), "animations": len(anims),
                       "joint_count": len(joints) if joints else 0,
                       "same_skeleton_as_avatar": bool(same),
                       "first_joints": (joints or [])[:4]}
                out["characters"].append(rec)
                log(f"  {name}: joints={rec['joint_count']} anims={len(anims)} "
                    f"same_skeleton={same}")
                if anims and (same or joints is None) and out["match"] is None:
                    out["match"] = rec
                if anims:
                    out.setdefault("clip_assets", []).append(rec)
                break
    return out


def h1_pose(stage):
    """Is H1 standing or in a T-pose, and is there a shipped variant to pick?"""
    path = "/Root/Robots/BOT_03"
    prim = stage.GetPrimAtPath(path)
    out: dict = {"prim_path": path, "valid": bool(prim.IsValid())}
    if not prim.IsValid():
        log("! BOT_03 not on the stage")
        return out

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    size = rng.GetSize()
    width = max(float(size[0]), float(size[1]))
    depth = min(float(size[0]), float(size[1]))
    height = float(size[2])
    out.update({"width_m": round(width, 3), "depth_m": round(depth, 3),
                "height_m": round(height, 3),
                "width_over_height": round(width / height, 3) if height else None})
    # A standing adult humanoid is roughly 0.25-0.35 of its height across the
    # shoulders. A T-pose is about 0.9-1.0, because arm span tracks height.
    out["verdict"] = ("T-POSE / ARMS OUT" if width > 0.6 * height
                      else "STANDING (arms in)" if width < 0.45 * height
                      else "AMBIGUOUS")
    log(f"H1 bbox  w={width:.3f}  d={depth:.3f}  h={height:.3f}  "
        f"w/h={out['width_over_height']}  -> {out['verdict']}")

    arms = []
    for p in Usd.PrimRange(prim):
        name = p.GetName().lower()
        if any(k in name for k in ("shoulder", "elbow", "wrist", "hand")):
            t = UsdGeom.XformCache().GetLocalToWorldTransform(p).ExtractTranslation()
            arms.append((p.GetName(), [round(float(v), 3) for v in t]))
    out["arm_links"] = arms[:12]
    for n, t in arms[:12]:
        log(f"   link {n:28s} world {t}")

    vsets = []
    for p in Usd.PrimRange(prim):
        names = p.GetVariantSets().GetNames()
        if names:
            vsets.append({"prim": p.GetPath().pathString,
                          "sets": {n: p.GetVariantSets().GetVariantSet(n).GetVariantNames()
                                   for n in names}})
    out["variant_sets"] = vsets[:6]
    log(f"variant sets found: {len(vsets)}")
    for v in vsets[:6]:
        log(f"   {v['prim']}: {v['sets']}")
    return out


def run():
    stage = omni.usd.get_context().get_stage()
    from isaacsim.storage.native import get_assets_root_path

    root = get_assets_root_path()
    log(f"asset root {root}")

    ref = worker_skeleton(stage)
    log(f"avatar skeleton: {len(ref) if ref else 0} joints, first {ref[:3] if ref else None}")

    people = scan_people(root, ref)
    sf.reference_robots(stage)
    stage.Load()
    pose = h1_pose(stage)

    S["result"] = {"avatar_joint_count": len(ref) if ref else 0,
                   "people": people, "h1": pose}
    (OUT / "people_and_pose.json").write_text(json.dumps(S["result"], indent=1, default=str))

    log("=" * 70)
    m = people.get("match")
    log(f"WALK CLIP: {'MATCH -> ' + m['asset'] if m else 'no character matches the avatar skeleton'}")
    log(f"H1 POSE  : {pose.get('verdict')}  (w/h {pose.get('width_over_height')})")
    log("DONE")


def on_update(_e):
    S["frame"] += 1
    try:
        if S["phase"] == "loading":
            if S["frame"] > 5 and not any(omni.usd.get_context().get_stage_loading_status()[1:]):
                S["phase"] = "running"
                run()
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
    on_update, name="people_pose"
)
