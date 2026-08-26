"""Does the walk cycle reach the RENDERER, and what does it cost?

Three questions, none of which the code answering them can answer about
itself, and one that no machine can answer at all.

1. **Does a per-frame write to a bound SkelAnimation re-skin the mesh?**
   `sim/avatar.py`'s WalkCycle poses the character by setting `rotations` on a
   SkelAnimation it authored, at the default time code, once per frame. USD
   will happily accept those writes whether or not anything downstream is
   listening -- so "the animation is correct" and "the picture changed" are
   different claims and only the second one matters. Measured the way this
   project measures everything else that could fail silently: hold the world
   still, advance ONLY the phase, and count pixels that changed against a
   control where the phase is frozen too.

2. **What does it cost?** The character was already being re-skinned every
   frame before any of this -- the Worker ships a 582-sample idle clip and Play
   advances the timeline -- so the honest question is not "what does skinning
   cost" but "what does the GAIT cost on top of skinning that was already
   happening". Both arms run in this process, back to back.

3. **Does it look like a person walking?** Not answerable here. This writes
   third-person PNGs at four points around the cycle and stops; the gate is a
   human looking at them.

The avatar never actually moves. The capsule stays where it is and the phase
is advanced by handing WalkCycle.update() a position that is not where the
avatar is -- so the third-person camera, which is mounted on the capsule, does
not move either, and any pixel that changes is a limb rather than a
background. That is the whole reason the test is arranged this way.

Exec mode. Play is pressed before anything is sampled (failure mode 10).

Run::

    docker compose -f docker/docker-compose.yml run --rm -T sim \\
        ./runheadless.sh --exec /workspace/sim/spikes/_diag_walk_render.py
"""

from __future__ import annotations

import json
import os
import statistics
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from pxr import Sdf, Usd, UsdSkel

REPO = Path(__file__).resolve().parent.parent.parent
SIM = REPO / "sim"
for _p in (str(REPO), str(SIM)):
    if _p not in sys.path:
        sys.path.insert(0, _p)
os.environ.setdefault("SF_NO_AUTORUN", "1")
os.environ.setdefault("OA_NO_AUTORUN", "1")

import avatar as av  # noqa: E402

STAGE = os.environ.get("WR_STAGE", str(SIM / "observatory_avatar.usd"))
OUT_DIR = Path(os.environ.get("WR_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
FRAMES = int(os.environ.get("WR_FRAMES", "40"))
RES = (640, 480)
AVATAR = "/Root/Avatar"
TP_CAM = f"{AVATAR}/body_mesh/cam_third_person"
#: Metres of pretend travel per frame. 0.03 m at 60 fps is 1.8 m/s, comfortably
#: past _GAIT_FULL_SPEED_MPS, so the swing runs at full amplitude.
STEP_M = 0.03
#: A pixel counts as changed when any channel moves by more than this. The
#: renderer's own frame-to-frame variation is measured, not assumed -- see the
#: control phase -- and this only has to sit above quantisation.
PIXEL_TOL = 12


def log(m: str) -> None:
    print(f"[walk_render] {m}", flush=True)


S: dict = {"phase": "loading", "frame": 0, "warm": 0, "n": 0, "sub": None,
           "rgb": None, "rp": None, "cycle": None, "prev": None,
           "changed": {}, "times": {}, "last_t": None, "report": {},
           "orig_anim": None, "skel": None, "shots": []}


def changed_pixels(now) -> int | None:
    prev = S["prev"]
    S["prev"] = now
    if prev is None or prev.shape != now.shape:
        return None
    d = np.abs(now.astype(np.int16) - prev.astype(np.int16)).max(axis=2)
    return int((d > PIXEL_TOL).sum())


def sample_rgb():
    arr = np.asarray(S["rgb"].get_data())
    if arr.size == 0:
        return None
    return arr[:, :, :3].copy()


def tick_time(phase: str) -> None:
    now = time.perf_counter()
    if S["last_t"] is not None:
        S["times"].setdefault(phase, []).append(now - S["last_t"])
    S["last_t"] = now


def fps_of(phase: str):
    xs = S["times"].get(phase) or []
    xs = sorted(xs)[2:-2] if len(xs) > 8 else xs      # drop the tails
    return round(1.0 / statistics.median(xs), 2) if xs else None


def setup() -> None:
    stage = omni.usd.get_context().get_stage()
    cam = stage.GetPrimAtPath(TP_CAM)
    if not cam.IsValid():
        raise RuntimeError(f"no third-person camera at {TP_CAM}")
    S["rp"] = rep.create.render_product(TP_CAM, resolution=RES)
    S["rgb"] = rep.AnnotatorRegistry.get_annotator("rgb")
    S["rgb"].attach([S["rp"]])
    log(f"render product on {TP_CAM} at {RES}")

    # Remember what the asset had bound, so arm A can put it back and be a
    # measurement of the world BEFORE this change rather than of a half-applied
    # version of it.
    skels = [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"{AVATAR}/character"))
             if p.IsA(UsdSkel.Skeleton)]
    if skels:
        S["skel"] = skels[0]
        src = UsdSkel.BindingAPI(skels[0]).GetAnimationSource()
        S["orig_anim"] = str(src.GetPath()) if src else None
    log(f"skeleton {S['skel'].GetPath() if S['skel'] else None}, "
        f"shipped animation {S['orig_anim']}")

    omni.timeline.get_timeline_interface().play()
    log("play() called -- capture requires it (failure mode 10)")


def build_cycle() -> None:
    stage = omni.usd.get_context().get_stage()
    cycle = av.WalkCycle(stage, AVATAR)
    S["cycle"] = cycle
    S["report"]["cycle_ok"] = cycle.ok
    S["report"]["cycle_why"] = cycle.why
    if not cycle.ok:
        log(f"! WalkCycle did not build: {cycle.why}")
        return
    S["report"]["driven_joints"] = sorted(cycle.driven)
    S["report"]["axes"] = {k: [round(float(v[i]), 4) for i in range(3)]
                           for k, v in cycle.axis.items()}
    S["report"]["signs"] = {k: v for k, v in cycle.sign.items()}
    S["report"]["stride_m"] = cycle.stride_m
    log(f"WalkCycle: {len(cycle.driven)} joints {sorted(cycle.driven)}")
    log(f"  derived swing signs: {cycle.sign}")

    # USD-side proof, independent of any picture: ask UsdSkel what the joints
    # resolve to at two different phases and check they are not the same
    # numbers. If this fails the animation is not bound and no amount of
    # looking at pixels will help.
    try:
        cache = UsdSkel.Cache()
        skel_q = cache.GetSkelQuery(UsdSkel.Skeleton(S["skel"]))
        cycle.phase, cycle.speed = 0.0, 2.0
        cycle.update((0.0, 0.0))
        a = skel_q.ComputeJointLocalTransforms(Usd.TimeCode.Default())
        cycle.phase = 0.5
        cycle.update((STEP_M, 0.0))
        b = skel_q.ComputeJointLocalTransforms(Usd.TimeCode.Default())
        if a and b:
            deltas = [max(abs(a[i][r][c] - b[i][r][c]) for r in range(4)
                          for c in range(4)) for i in range(min(len(a), len(b)))]
            S["report"]["usd_joint_max_delta"] = round(float(max(deltas)), 6)
            S["report"]["usd_joints_moved"] = int(sum(1 for d in deltas if d > 1e-4))
            log(f"  UsdSkel resolves {S['report']['usd_joints_moved']} joints "
                f"differently between phase 0.0 and 0.5 "
                f"(max element delta {S['report']['usd_joint_max_delta']})")
    except Exception as exc:                                      # noqa: BLE001
        log(f"  ! UsdSkel cross-check unavailable: {exc!r}")
        S["report"]["usd_check_error"] = repr(exc)

    # Park it where the run starts: still, facing the built heading, so the
    # control phase has nothing at all to change.
    cycle.phase = 0.0
    cycle.speed = 0.0
    cycle.heading_deg = av._DEFAULT_HEADING_DEG
    cycle.last_xy = (0.0, 0.0)
    cycle.update((0.0, 0.0))


def shoot(tag: str) -> None:
    arr = sample_rgb()
    if arr is None:
        return
    path = OUT_DIR / f"walk_{tag}.png"
    try:
        from PIL import Image

        Image.fromarray(arr).save(str(path))
        S["shots"].append(str(path))
        log(f"  wrote {path}")
    except Exception as exc:                                      # noqa: BLE001
        log(f"  ! could not write {path}: {exc!r}")


def restore_shipped_animation() -> None:
    if S["skel"] is None or not S["orig_anim"]:
        return
    UsdSkel.BindingAPI.Apply(S["skel"]).CreateAnimationSourceRel().SetTargets(
        [Sdf.Path(S["orig_anim"])])
    log(f"rebound the shipped animation {S['orig_anim']} for the baseline arm")


def report() -> None:
    r = S["report"]
    ctrl = S["changed"].get("control") or []
    test = S["changed"].get("test") or []
    r["control_changed_px"] = {"max": max(ctrl, default=0),
                               "median": int(statistics.median(ctrl)) if ctrl else 0}
    r["test_changed_px"] = {"max": max(test, default=0),
                            "median": int(statistics.median(test)) if test else 0}
    r["fps_bound_not_written"] = fps_of("C_bound_static")
    r["frame_ms_bound_not_written"] = (
        round(1000 * statistics.median(S["times"]["C_bound_static"]), 2)
        if S["times"].get("C_bound_static") else None)
    r["fps_shipped_idle_only"] = fps_of("A_shipped")
    r["fps_with_walk_cycle"] = fps_of("B_walk")
    r["frame_ms_shipped_idle_only"] = (
        round(1000 * statistics.median(S["times"]["A_shipped"]), 2)
        if S["times"].get("A_shipped") else None)
    r["frame_ms_with_walk_cycle"] = (
        round(1000 * statistics.median(S["times"]["B_walk"]), 2)
        if S["times"].get("B_walk") else None)
    r["pixels"] = RES[0] * RES[1]
    r["shots"] = S["shots"]

    print("\n" + "=" * 78, flush=True)
    print("DOES THE WALK CYCLE REACH THE RENDERER, AND WHAT DOES IT COST?", flush=True)
    print("=" * 78, flush=True)
    print(f"  WalkCycle built            {r.get('cycle_ok')} "
          f"{'' if r.get('cycle_ok') else r.get('cycle_why')}", flush=True)
    print(f"  joints driven              {len(r.get('driven_joints') or [])}", flush=True)
    print(f"  UsdSkel joints that moved  {r.get('usd_joints_moved')} "
          f"(max element delta {r.get('usd_joint_max_delta')})", flush=True)
    print(f"  changed px, phase FROZEN   median {r['control_changed_px']['median']}"
          f"  max {r['control_changed_px']['max']}", flush=True)
    print(f"  changed px, phase ADVANCING median {r['test_changed_px']['median']}"
          f"  max {r['test_changed_px']['max']}", flush=True)
    ratio = (r['test_changed_px']['median'] / max(1, r['control_changed_px']['median']))
    print(f"  ratio                      {ratio:.1f}x", flush=True)
    print(f"  frame time, shipped idle   {r['frame_ms_shipped_idle_only']} ms "
          f"({r['fps_shipped_idle_only']} fps)", flush=True)
    print(f"  frame time, ours bound,    {r['frame_ms_bound_not_written']} ms "
          f"({r['fps_bound_not_written']} fps)", flush=True)
    print(f"    but never written", flush=True)
    print(f"  frame time, + walk cycle   {r['frame_ms_with_walk_cycle']} ms "
          f"({r['fps_with_walk_cycle']} fps)", flush=True)
    if r['frame_ms_shipped_idle_only'] and r['frame_ms_with_walk_cycle']:
        d = r['frame_ms_with_walk_cycle'] - r['frame_ms_shipped_idle_only']
        print(f"  cost of the gait           {d:+.2f} ms/frame", flush=True)
        r["gait_cost_ms"] = round(d, 3)
    print(f"  third-person stills        {len(S['shots'])} written -- "
          f"A HUMAN HAS TO LOOK AT THESE", flush=True)
    print("=" * 78, flush=True)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "walk_render.json").write_text(json.dumps(r, indent=1, default=str))
    log(f"wrote {OUT_DIR / 'walk_render.json'}")


def on_update(_e) -> None:
    S["frame"] += 1
    try:
        ph = S["phase"]
        if ph == "loading":
            ctx = omni.usd.get_context()
            if S["frame"] > 5 and not any(ctx.get_stage_loading_status()[1:]):
                setup()
                S["phase"] = "warmup"
            return

        if ph == "warmup":
            S["warm"] += 1
            if sample_rgb() is not None or S["warm"] > 300:
                log(f"first pixels after {S['warm']} frames")
                S["phase"] = "A_shipped"
                S["n"] = 0
                S["last_t"] = None
            return

        # --- A: the world as it is today, shipped idle clip only ----------
        if ph == "A_shipped":
            tick_time("A_shipped")
            S["n"] += 1
            if S["n"] == 1:
                shoot("A_shipped_idle")
            if S["n"] >= FRAMES:
                build_cycle()
                S["phase"] = "control" if S["cycle"] and S["cycle"].ok else "done"
                S["n"] = 0
                S["prev"] = None
                S["last_t"] = None
            return

        # --- control: everything running, phase deliberately frozen -------
        if ph == "control":
            tick_time("C_bound_static")
            S["n"] += 1
            c = changed_pixels(sample_rgb())
            if c is not None and S["n"] > 3:
                S["changed"].setdefault("control", []).append(c)
            if S["n"] >= FRAMES:
                S["phase"] = "test"
                S["n"] = 0
                S["prev"] = None
                S["last_t"] = None
            return

        # --- test: only the phase advances --------------------------------
        if ph == "test":
            tick_time("B_walk")
            S["n"] += 1
            cyc = S["cycle"]
            # Pretend travel along the heading the avatar already faces, so
            # the facing never changes and the limbs are the only variable.
            cyc.speed = 2.0
            x = -STEP_M * S["n"]
            cyc.update((x, 0.0))
            c = changed_pixels(sample_rgb())
            if c is not None and S["n"] > 3:
                S["changed"].setdefault("test", []).append(c)
            if S["n"] in (8, 16, 24, 32):
                shoot(f"B_walk_phase{S['n']:02d}")
            if S["n"] >= FRAMES:
                S["phase"] = "done"
            return

        if ph == "done":
            report()
            log("DONE")
            S["sub"] = None
            omni.kit.app.get_app().post_quit(0)
            return
    except Exception as exc:                                      # noqa: BLE001
        log("FAILED: " + repr(exc))
        log(traceback.format_exc())
        try:
            report()
        except Exception:                                         # noqa: BLE001
            pass
        log("DONE")
        S["sub"] = None
        omni.kit.app.get_app().post_quit(1)


log(f"open_stage {STAGE} -> {omni.usd.get_context().open_stage(STAGE)}")
S["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    on_update, name="walk_render")
