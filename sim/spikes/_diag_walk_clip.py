"""Throwaway: can Isaac/People's walk clip drive the avatar's own skeleton?

Closes the caveat left open by the 2026-08-17 scoping spike
(`_diag_people_and_pose.py`), which concluded that the walk cycle is a
reference swap rather than a retargeting job -- and then said so from an
inference it could not test:

    "the clips are SkelAnimation-only assets with no Skeleton prim, so the
     joint-list comparison could not run against them directly -- joints=0 in
     the log means 'no Skeleton here', not 'different skeleton'. Strong
     inference, not proof; the first ten minutes of that task should confirm
     it."

This is those ten minutes. The comparison it wanted is possible after all: a
`SkelAnimation` carries its OWN `joints` token array -- that is how UsdSkel
maps a clip onto a skeleton in the first place, by NAME rather than by index.
The earlier spike looked for a `Skeleton` prim and correctly found none; the
answer was on the animation prim the whole time.

Three questions, all of which decide code that has not been written yet:

  1. **Do the clip's joints exist on the Worker's skeleton, by name?** If a
     joint is missing the clip does not fail -- UsdSkel simply ignores it, and
     the limb it drives stays in its bind pose. A partial match is therefore
     the dangerous answer, not an error, so the count of matched joints is
     reported rather than a boolean.
  2. **What is the STRIDE?** The in-place clip is the one to play -- the
     character controller already supplies world translation -- but in-place
     means the root joint no longer says how far one cycle carries you, and
     without that number the leg cycle cannot be locked to ground speed and
     the feet skate. The non-in-place variant of the same motion still has its
     root translation, so the stride is measurable there and transferable.
  3. **How long is a cycle, and at what frame rate?** Needed to turn "metres
     travelled" into "where in the clip we are".

Reads only. Creates no prims, binds nothing, saves nothing.

Exec mode. Run::

    docker compose -f docker/docker-compose.yml run --rm -T sim \\
        ./runheadless.sh --exec /workspace/sim/spikes/_diag_walk_clip.py
"""

from __future__ import annotations

import json
import os
import sys
import traceback
from pathlib import Path

import omni.kit.app
import omni.usd
from pxr import Usd, UsdGeom, UsdSkel

REPO = Path(__file__).resolve().parent.parent.parent
SIM = REPO / "sim"
for _p in (str(REPO), str(SIM)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

STAGE = os.environ.get("WC_STAGE", str(SIM / "observatory_avatar.usd"))
OUT_DIR = Path(os.environ.get("WC_OUT", "/isaac-sim/.nvidia-omniverse/logs"))

#: Confirmed 2026-08-17 by listing the live asset root with omni.client, not
#: recalled: this is where Isaac/People's clips actually are, and they are
#: under 6.0 while the Worker character the avatar rides is under 5.0.
PEOPLE = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/6.0/Isaac/People/Animations")
CLIPS = {
    "walk_in_place": f"{PEOPLE}/stand_walk_loop_in_place.skelanim.usd",
    "walk_loop": f"{PEOPLE}/stand_walk_loop.skelanim.usd",
    "idle": f"{PEOPLE}/stand_idle_loop.skelanim.usd",
}
AVATAR = "/Root/Avatar"


def log(msg: str) -> None:
    print(f"[walk_clip] {msg}", flush=True)


def find_skeleton(stage, root_path: str):
    """The avatar's Skeleton prim, its joint list, and what drives it now."""
    root = stage.GetPrimAtPath(root_path)
    out = {"root": root_path, "valid": bool(root.IsValid())}
    if not root.IsValid():
        return out, None
    skels, skelroots, anims = [], [], []
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdSkel.Skeleton):
            skels.append(prim)
        if prim.IsA(UsdSkel.Root):
            skelroots.append(prim)
        if prim.IsA(UsdSkel.Animation):
            anims.append(prim)
    out["skel_roots"] = [str(p.GetPath()) for p in skelroots]
    out["skeletons"] = [str(p.GetPath()) for p in skels]
    out["animations_already_present"] = [str(p.GetPath()) for p in anims]
    if not skels:
        return out, None
    skel = UsdSkel.Skeleton(skels[0])
    joints = [str(j) for j in (skel.GetJointsAttr().Get() or [])]
    out["skeleton"] = str(skels[0].GetPath())
    out["joint_count"] = len(joints)
    out["joints_head"] = joints[:6]
    # What is bound RIGHT NOW. `skel:animationSource` is a relationship and
    # lives on the Skeleton or on any ancestor; report where it resolves from,
    # because rebinding has to happen at the winning site or it does nothing.
    binding = UsdSkel.BindingAPI(skels[0])
    target = binding.GetAnimationSource()
    out["bound_animation"] = str(target.GetPath()) if target else None
    prim = skels[0]
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        rel = prim.GetRelationship("skel:animationSource")
        if rel and rel.GetTargets():
            out.setdefault("animation_source_rels", []).append(
                {"on": str(prim.GetPath()),
                 "targets": [str(t) for t in rel.GetTargets()]})
        prim = prim.GetParent()
    return out, joints


def read_clip(name: str, url: str) -> dict:
    """Everything about one clip that the driver will need as a constant."""
    row: dict = {"name": name, "url": url}
    try:
        clip = Usd.Stage.Open(url)
    except Exception as exc:                                      # noqa: BLE001
        row["error"] = repr(exc)
        return row
    if clip is None:
        row["error"] = "Usd.Stage.Open returned None"
        return row

    row["fps"] = float(clip.GetTimeCodesPerSecond() or 0.0)
    row["start"] = float(clip.GetStartTimeCode())
    row["end"] = float(clip.GetEndTimeCode())
    anims = [p for p in clip.Traverse() if p.IsA(UsdSkel.Animation)]
    row["animation_prims"] = [str(p.GetPath()) for p in anims]
    if not anims:
        row["error"] = "no UsdSkelAnimation in this layer"
        return row

    anim = UsdSkel.Animation(anims[0])
    joints = [str(j) for j in (anim.GetJointsAttr().Get() or [])]
    row["joint_count"] = len(joints)
    row["joints"] = joints
    rot = anim.GetRotationsAttr()
    row["rotation_samples"] = len(rot.GetTimeSamples()) if rot else 0
    tr = anim.GetTranslationsAttr()
    row["translation_samples"] = len(tr.GetTimeSamples()) if tr else 0
    sc = anim.GetScalesAttr()
    row["scale_samples"] = len(sc.GetTimeSamples()) if sc else 0
    first = (rot.Get(row["start"]) if rot else None)
    row["rotations_per_frame"] = len(first) if first is not None else 0

    # THE STRIDE. The root joint's translation across the loop is how far one
    # cycle of this motion carries a body; on the in-place variant it should
    # be ~0 by construction, which is the check that the two are the same
    # motion with the root removed rather than two different clips.
    if tr and tr.GetTimeSamples():
        samples = tr.GetTimeSamples()
        t0, t1 = samples[0], samples[-1]
        a, b = tr.Get(t0), tr.Get(t1)
        if a is not None and b is not None and len(a) and len(b):
            d = [float(b[0][i] - a[0][i]) for i in range(3)]
            row["root_joint"] = joints[0] if joints else None
            row["root_translation_over_clip"] = [round(v, 4) for v in d]
            row["root_distance"] = round(max(abs(d[0]), abs(d[1])), 4)
            row["clip_seconds"] = round((t1 - t0) / (row["fps"] or 1.0), 4)
    return row


# ---------------------------------------------------------------------------
# Round two. The People clips do NOT fit this skeleton -- measured above -- so
# the question becomes: does a clip on the RIGHT skeleton ship anywhere?
# ---------------------------------------------------------------------------
ASSET_5 = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
           "/Assets/Isaac/5.0")
ASSET_6 = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
           "/Assets/Isaac/6.0")
LIST_DIRS = [
    f"{ASSET_5}/NVIDIA/Assets/Characters/Reallusion/Worker",
    f"{ASSET_5}/NVIDIA/Assets/Characters/Reallusion",
    f"{ASSET_6}/Isaac/People/Characters",
    f"{ASSET_6}/Isaac/People",
]
SAMPLE_STAGES = [
    f"{ASSET_6}/Isaac/Samples/Replicator/Stage/full_warehouse_worker_and_anim_cameras.usd",
    f"{ASSET_5}/Isaac/Samples/Replicator/Stage/full_warehouse_worker_and_anim_cameras.usd",
]
ANIM_EXTS = ["omni.anim.people", "omni.anim.graph.core", "omni.anim.retarget.core",
             "omni.anim.retarget.bundle", "omni.anim.timeline", "omni.anim.skeljoint"]


def list_dir(url: str) -> list:
    import omni.client
    try:
        res, entries = omni.client.list(url)
    except Exception as exc:                                      # noqa: BLE001
        return [f"<error {exc!r}>"]
    if res != omni.client.Result.OK:
        return [f"<{res}>"]
    return sorted(e.relative_path for e in entries)


def describe_skel_stage(url: str) -> dict:
    """Skeletons and animations in a shipped stage, with root motion."""
    row: dict = {"url": url}
    try:
        st = Usd.Stage.Open(url)
    except Exception as exc:                                      # noqa: BLE001
        row["error"] = repr(exc)
        return row
    if st is None:
        row["error"] = "open returned None"
        return row
    skels, anims = [], []
    for prim in st.Traverse():
        if prim.IsA(UsdSkel.Skeleton):
            sk = UsdSkel.Skeleton(prim)
            js = [str(j) for j in (sk.GetJointsAttr().Get() or [])]
            skels.append({"path": str(prim.GetPath()), "joints": len(js),
                          "head": js[:4]})
        if prim.IsA(UsdSkel.Animation):
            an = UsdSkel.Animation(prim)
            js = [str(j) for j in (an.GetJointsAttr().Get() or [])]
            rot = an.GetRotationsAttr()
            tr = an.GetTranslationsAttr()
            entry = {"path": str(prim.GetPath()), "joints": len(js),
                     "head": js[:4],
                     "rot_samples": len(rot.GetTimeSamples()) if rot else 0}
            if tr and tr.GetTimeSamples():
                ts = tr.GetTimeSamples()
                a, b = tr.Get(ts[0]), tr.Get(ts[-1])
                if a is not None and b is not None and len(a) and len(b):
                    d = [round(float(b[0][i] - a[0][i]), 4) for i in range(3)]
                    entry["root_translation"] = d
                    entry["root_distance"] = round(max(abs(v) for v in d), 4)
            anims.append(entry)
    row["skeletons"] = skels
    row["animations"] = anims
    return row


def round_two(stage, skel_joints) -> dict:
    out: dict = {}
    for d in LIST_DIRS:
        out.setdefault("listings", {})[d] = list_dir(d)
        log(f"{d.rsplit('/', 2)[-2]}/{d.rsplit('/', 1)[-1]}: "
            f"{out['listings'][d][:14]}")

    # The clip the avatar ALREADY carries -- on the right skeleton by
    # definition, since it came with the rig. Is it a walk or a sway?
    own = stage.GetPrimAtPath(
        "/Root/Avatar/character/rig/ManRoot/Worker/Worker/Animation")
    if own.IsValid():
        an = UsdSkel.Animation(own)
        js = [str(j) for j in (an.GetJointsAttr().Get() or [])]
        rot, tr = an.GetRotationsAttr(), an.GetTranslationsAttr()
        row = {"path": str(own.GetPath()), "joints": len(js), "head": js[:4],
               "rot_samples": len(rot.GetTimeSamples()) if rot else 0}
        if skel_joints:
            row["matched"] = sum(1 for j in js if j in set(skel_joints))
        if tr and tr.GetTimeSamples():
            ts = tr.GetTimeSamples()
            a, b = tr.Get(ts[0]), tr.Get(ts[-1])
            if a is not None and b is not None and len(a) and len(b):
                d = [round(float(b[0][i] - a[0][i]), 4) for i in range(3)]
                row["root_translation"] = d
                row["root_distance"] = round(max(abs(v) for v in d), 4)
        out["avatar_own_clip"] = row
        log(f"avatar's own clip: {json.dumps(row)[:300]}")

    for url in SAMPLE_STAGES:
        row = describe_skel_stage(url)
        out.setdefault("sample_stages", []).append(row)
        if "error" in row:
            log(f"! sample stage {url.rsplit('/', 1)[-1]}: {row['error']}")
            continue
        log(f"sample stage {url.rsplit('/', 1)[-1]}: "
            f"skeletons={[(s['joints'], s['head'][:1]) for s in row['skeletons']]} "
            f"animations={[(a['joints'], a.get('root_distance')) for a in row['animations']]}")

    try:
        import omni.kit.app
        mgr = omni.kit.app.get_app().get_extension_manager()
        out["extensions"] = {e: bool(mgr.is_extension_enabled(e)) for e in ANIM_EXTS}
        avail = {e: any(e in str(x.get("name", "")) for x in mgr.get_extensions())
                 for e in ANIM_EXTS}
        out["extensions_present"] = avail
        log(f"anim extensions enabled: {out['extensions']}")
        log(f"anim extensions present in image: {avail}")
    except Exception as exc:                                      # noqa: BLE001
        out["extensions_error"] = repr(exc)
    return out


def run() -> None:
    stage = omni.usd.get_context().get_stage()
    report: dict = {"stage": STAGE}

    try:
        skel_info, skel_joints = find_skeleton(stage, f"{AVATAR}/character")
    except Exception as exc:                                      # noqa: BLE001
        log(f"! skeleton scan failed: {exc!r}")
        skel_info, skel_joints = {"error": repr(exc)}, None
    report["avatar_skeleton"] = skel_info
    log(f"avatar skeleton: {json.dumps(skel_info, default=str)[:600]}")

    report["clips"] = {}
    for name, url in CLIPS.items():
        row = read_clip(name, url)
        report["clips"][name] = row
        if "error" in row:
            log(f"! {name}: {row['error']}")
            continue
        log(f"{name}: {row['joint_count']} joints, "
            f"{row['rotation_samples']} rotation samples, "
            f"{row['start']}..{row['end']} @ {row['fps']} fps "
            f"({row.get('clip_seconds')} s), root moves "
            f"{row.get('root_translation_over_clip')}")

    # -- THE COMPARISON the earlier spike could not run --------------------
    if skel_joints:
        skel_set = set(skel_joints)
        for name, row in report["clips"].items():
            if "joints" not in row:
                continue
            clip_joints = row["joints"]
            matched = [j for j in clip_joints if j in skel_set]
            missing = [j for j in clip_joints if j not in skel_set]
            row["matched_joints"] = len(matched)
            row["missing_joints"] = missing[:10]
            row["missing_count"] = len(missing)
            row["identical_order"] = clip_joints == skel_joints
            log(f"{name}: {len(matched)}/{len(clip_joints)} clip joints exist on "
                f"the avatar's skeleton; {len(missing)} do not"
                + (f" (first missing: {missing[:4]})" if missing else "")
                + f"; identical order: {row['identical_order']}")
            # A joint the clip drives that the skeleton lacks is silently
            # ignored by UsdSkel and its limb stays in the bind pose. That is
            # why the number matters and a boolean would not.
            skel_only = [j for j in skel_joints if j not in set(clip_joints)]
            row["skeleton_joints_not_in_clip"] = len(skel_only)
            row["skeleton_only_head"] = skel_only[:8]

    print("\n" + "=" * 78, flush=True)
    print("CAN THE PEOPLE WALK CLIP DRIVE THIS AVATAR?", flush=True)
    print("=" * 78, flush=True)
    n_skel = skel_info.get("joint_count", 0)
    print(f"  avatar skeleton      {skel_info.get('skeleton')}", flush=True)
    print(f"  joints               {n_skel}", flush=True)
    print(f"  bound animation now  {skel_info.get('bound_animation')}", flush=True)
    for name, row in report["clips"].items():
        if "error" in row:
            print(f"  {name:<16s} UNAVAILABLE: {row['error']}", flush=True)
            continue
        print(f"  {name:<16s} {row.get('matched_joints', '?')}/"
              f"{row.get('joint_count', '?')} joints match, "
              f"{row.get('clip_seconds', '?')} s, stride "
              f"{row.get('root_distance', '?')}", flush=True)
    print("=" * 78, flush=True)

    try:
        report["round_two"] = round_two(stage, skel_joints)
    except Exception as exc:                                      # noqa: BLE001
        log(f"! round two failed: {exc!r}")
        report["round_two"] = {"error": repr(exc)}

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "walk_clip.json").write_text(json.dumps(report, indent=1, default=str))
    log(f"wrote {OUT_DIR / 'walk_clip.json'}")


S = {"frame": 0, "sub": None}


def on_update(_e) -> None:
    S["frame"] += 1
    ctx = omni.usd.get_context()
    if S["frame"] <= 5 or any(ctx.get_stage_loading_status()[1:]):
        return
    try:
        run()
    except Exception as exc:                                      # noqa: BLE001
        log("FAILED: " + repr(exc))
        log(traceback.format_exc())
    log("DONE")
    S["sub"] = None
    omni.kit.app.get_app().post_quit(0)


log(f"open_stage {STAGE} -> {omni.usd.get_context().open_stage(STAGE)}")
S["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    on_update, name="walk_clip")
