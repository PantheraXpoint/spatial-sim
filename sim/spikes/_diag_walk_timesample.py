"""Does authoring the gait as TIME SAMPLES recover the 15.9 ms it costs to bind?

Short answer, measured below: there was no 15.9 ms to recover. It was the
annotator readback, and it was in two of the three rows of the table that
raised the question.

THE CLAIM UNDER TEST
--------------------
The 2026-08-26 walk-cycle run (`_diag_walk_render.py`) priced the gait three
ways:

    A  shipped idle clip bound            17.37 ms/frame
    B  ours bound, NEVER written          33.30 ms/frame
    C  ours bound and written every frame 33.34 ms/frame

and concluded that binding a different animation costs +15.97 ms while the
gait's arithmetic costs +0.04. It named one structural difference between the
two animations -- the shipped clip is TIME SAMPLED, ours held values at the
default time code -- and stopped there, because naming where a fix would look
is not testing it.

Row A, though, is not the same measurement as rows B and C. It never called
`annotator.get_data()`; rows B and C called it on every frame in order to count
changed pixels. So the table has two variables in it, not one, and the second
one is a synchronous GPU readback of a 640x480 buffer.

WHAT IS MEASURED
----------------
A 3x2 matrix, back to back IN ONE PROCESS, because absolute frame times on this
shared host are not comparable between runs -- the shipped baseline alone has
measured 17.61, 26.11, 17.37 and 17.63 ms on four different days. Only a delta
inside one process means anything.

    variant                            x   readback OFF / readback ON
    ---------------------------------------------------------------
    shipped idle clip bound                measured at BOTH ENDS of the run
    ours, default time code, written
    ours, TIME SAMPLED, written

plus `default_static_read`, which reproduces row B of the original table
exactly -- our animation bound and never written, with the readback on.

The shipped rows are repeated after every other arm has run, so the run is only
readable if the two shipped pairs agree. `anim_walk` still exists on the stage
while unbound during those closing arms, so they also say whether the mere
PRESENCE of the prim is a cost rather than the binding.

Within the readback arms the two halves are timed separately -- the
`get_data()` call and the numpy difference -- so "the readback" is attributed
rather than inferred.

The avatar never moves. The capsule stays where it is and the phase is advanced
by handing update() a position that is not where the avatar is, so the
third-person camera -- which is mounted on the capsule -- does not move either
and any pixel that changes is a limb. That is what makes "did the sampled
animation still reach the renderer?" answerable: a variant that got cheaper by
quietly not being applied would show its changed pixels collapse to the static
arm's.

Exec mode. Play is pressed before anything is sampled (failure mode 10).
Results are appended per arm and fsync'd, never written once at the end.

Run::

    docker compose -f docker/docker-compose.yml run --rm -T sim \\
        ./runheadless.sh --exec /workspace/sim/spikes/_diag_walk_timesample.py

    docker stop <that container>     # it does not exit on DONE
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

STAGE = os.environ.get("WT_STAGE", str(SIM / "observatory_avatar.usd"))
OUT_DIR = Path(os.environ.get("WT_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
FRAMES = int(os.environ.get("WT_FRAMES", "40"))
RES = (640, 480)
AVATAR = "/Root/Avatar"
TP_CAM = f"{AVATAR}/body_mesh/cam_third_person"
ANIM_PATH = f"{AVATAR}/character/anim_walk"
#: Metres of pretend travel per frame, as in _diag_walk_render.py: 0.03 m at
#: 60 fps is 1.8 m/s, past _GAIT_FULL_SPEED_MPS, so the swing is at full
#: amplitude and the two runs' pixel counts are comparable.
STEP_M = 0.03
PIXEL_TOL = 12

#: (name, variant, write every frame, read the annotator back every frame).
#: Order matters: the variant only changes between arms, never inside one, and
#: the shipped pair is measured first and last.
ARMS = [
    ("shipped_noread_1",    "shipped", False, False),
    ("shipped_read_1",      "shipped", False, True),
    ("default_static_read", "default", False, True),   # row B of the original
    ("default_write_noread", "default", True, False),
    ("default_write_read",  "default", True, True),    # row C of the original
    ("sampled_write_noread", "sampled", True, False),
    ("sampled_write_read",  "sampled", True, True),
    ("shipped_noread_2",    "shipped", False, False),  # drift control
    ("shipped_read_2",      "shipped", False, True),   # drift control
]
BY_NAME = {a[0]: a for a in ARMS}


def log(m: str) -> None:
    print(f"[walk_ts] {m}", flush=True)


S: dict = {"phase": "loading", "frame": 0, "warm": 0, "n": 0, "sub": None,
           "rgb": None, "rp": None, "cycle": None, "prev": None,
           "changed": {}, "times": {}, "readback": {}, "diff": {},
           "last_t": None, "report": {}, "orig_anim": None, "skel": None,
           "shots": [], "arm": 0, "variant": "shipped", "tcps": 1.0}


# ---------------------------------------------------------------------------
# The variant under test
# ---------------------------------------------------------------------------
class SampledWalkCycle(av.WalkCycle):
    """The gait, authored as time samples at the timeline's own time code.

    The ONLY difference from ``WalkCycle`` is where the rotations land: at
    ``Usd.TimeCode(now)`` instead of at the default time code. Everything else
    -- the joints, the derived axes, the distance-driven phase, the blend onto
    the shipped idle -- is inherited unchanged, so the two arms differ in one
    variable and not in two.

    Lives here and not in ``sim/avatar.py`` on purpose: it is a hypothesis
    being priced, and a speculative flag in the shipped module would have
    outlived the answer. If the answer had been yes, it would have moved.
    """

    def __init__(self, *a, **kw) -> None:
        self.sampled_writes = 0
        self.write_tcs: list[float] = []
        super().__init__(*a, **kw)

    def _now_tc(self) -> float:
        return (omni.timeline.get_timeline_interface().get_current_time()
                * S["tcps"])

    def _write_rotations(self, rots) -> None:
        tc = self._now_tc()
        self.rot_attr.Set(rots, Usd.TimeCode(tc))
        self.sampled_writes += 1
        self.write_tcs.append(tc)


# ---------------------------------------------------------------------------
def changed_pixels(now) -> int | None:
    prev = S["prev"]
    S["prev"] = now
    if prev is None or now is None or prev.shape != now.shape:
        return None
    d = np.abs(now.astype(np.int16) - prev.astype(np.int16)).max(axis=2)
    return int((d > PIXEL_TOL).sum())


def sample_rgb():
    arr = np.asarray(S["rgb"].get_data())
    if arr.size == 0:
        return None
    return arr[:, :, :3].copy()


def med_ms(bucket: str, arm: str):
    xs = S[bucket].get(arm) or []
    xs = sorted(xs)[2:-2] if len(xs) > 8 else xs      # drop the tails
    return round(1000 * statistics.median(xs), 2) if xs else None


def append_jsonl(record: dict) -> None:
    """One line per arm, flushed and fsync'd. This renderer dies mid-run."""
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    with open(OUT_DIR / "walk_timesample.jsonl", "a") as fh:
        fh.write(json.dumps(record, default=str) + "\n")
        fh.flush()
        os.fsync(fh.fileno())


def anim_structure(prim_path: str) -> dict:
    """Per-attribute time-sample counts for a SkelAnimation.

    The whole hypothesis is a claim about structure, so the structure is read
    off the stage rather than assumed from what the code meant to author.
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return {"path": prim_path, "valid": False}
    out: dict = {"path": prim_path, "valid": True}
    for name in ("joints", "rotations", "translations", "scales"):
        attr = prim.GetAttribute(name)
        if not attr:
            out[name] = None
            continue
        ts = attr.GetNumTimeSamples()
        out[name] = {"time_samples": int(ts),
                     "has_default": bool(attr.HasAuthoredValue() and ts == 0),
                     "might_be_time_varying": bool(attr.ValueMightBeTimeVarying())}
        if name == "joints":
            v = attr.Get()
            out[name]["count"] = 0 if v is None else len(v)
    return out


def setup() -> None:
    stage = omni.usd.get_context().get_stage()
    cam = stage.GetPrimAtPath(TP_CAM)
    if not cam.IsValid():
        raise RuntimeError(f"no third-person camera at {TP_CAM}")
    S["rp"] = rep.create.render_product(TP_CAM, resolution=RES)
    S["rgb"] = rep.AnnotatorRegistry.get_annotator("rgb")
    S["rgb"].attach([S["rp"]])

    S["tcps"] = float(stage.GetTimeCodesPerSecond() or 60.0)
    S["report"]["stage"] = STAGE
    S["report"]["resolution"] = list(RES)
    S["report"]["frames_per_arm"] = FRAMES
    S["report"]["time_codes_per_second"] = S["tcps"]

    skels = [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"{AVATAR}/character"))
             if p.IsA(UsdSkel.Skeleton)]
    if skels:
        S["skel"] = skels[0]
        src = UsdSkel.BindingAPI(skels[0]).GetAnimationSource()
        S["orig_anim"] = str(src.GetPath()) if src else None
    S["report"]["skeleton"] = str(S["skel"].GetPath()) if S["skel"] else None
    S["report"]["shipped_animation"] = S["orig_anim"]
    if S["orig_anim"]:
        S["report"]["shipped_structure"] = anim_structure(S["orig_anim"])
        log(f"shipped clip: {S['report']['shipped_structure']}")

    omni.timeline.get_timeline_interface().play()
    log("play() called -- capture requires it (failure mode 10)")


def park(cycle) -> None:
    """Still, facing the built heading: a static arm has nothing to change."""
    cycle.phase = 0.0
    cycle.speed = 0.0
    cycle.heading_deg = av._DEFAULT_HEADING_DEG
    cycle.last_xy = (0.0, 0.0)
    cycle.update((0.0, 0.0))


def build(kind: str) -> bool:
    """Author and bind one variant of the gait, replacing whatever is bound.

    `anim_walk` is REMOVED first rather than reused: leaving it would keep the
    default-time value authored alongside the new time samples, and USD
    resolves samples over a default -- so the sampled arm would be measuring an
    attribute carrying both, which is neither variant.
    """
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(Sdf.Path(ANIM_PATH))
    cls = SampledWalkCycle if kind == "sampled" else av.WalkCycle
    cycle = cls(stage, AVATAR, verbose=False)
    S["cycle"] = cycle
    S["report"][f"{kind}_cycle_ok"] = cycle.ok
    if not cycle.ok:
        log(f"! {kind} WalkCycle did not build: {cycle.why}")
        S["report"][f"{kind}_cycle_why"] = cycle.why
        return False
    park(cycle)
    S["report"][f"{kind}_structure"] = anim_structure(ANIM_PATH)
    log(f"{kind} cycle built: {len(cycle.driven)} joints, rotations "
        f"{S['report'][f'{kind}_structure']['rotations']}")
    return True


def restore_shipped_animation() -> None:
    if S["skel"] is None or not S["orig_anim"]:
        return
    UsdSkel.BindingAPI.Apply(S["skel"]).CreateAnimationSourceRel().SetTargets(
        [Sdf.Path(S["orig_anim"])])
    S["cycle"] = None
    log(f"rebound the shipped animation {S['orig_anim']}")


def sampled_readback_check() -> None:
    """Read two of our OWN time samples back at different time codes.

    Proof the sampled arm authored DIFFERENT values at different time codes,
    rather than one pose re-authored under a moving clock -- which would look
    identical in the timing table and be a different experiment.
    """
    cyc = S["cycle"]
    tcs = getattr(cyc, "write_tcs", None) or []
    S["report"]["sampled_writes"] = getattr(cyc, "sampled_writes", None)
    if len(tcs) < 2:
        return
    a = cyc.rot_attr.Get(Usd.TimeCode(tcs[1]))
    b = cyc.rot_attr.Get(Usd.TimeCode(tcs[-1]))
    if a is None or b is None or len(a) != len(b):
        return
    d = max(max(abs(a[i].GetReal() - b[i].GetReal()),
                max(abs(a[i].GetImaginary()[k] - b[i].GetImaginary()[k])
                    for k in range(3)))
            for i in range(len(a)))
    S["report"]["sampled_readback_max_delta"] = round(float(d), 6)
    S["report"]["sampled_timecodes_first_last"] = [round(tcs[0], 3),
                                                   round(tcs[-1], 3)]
    S["report"]["sampled_timecodes_monotonic"] = all(
        y >= x for x, y in zip(tcs, tcs[1:]))


def shoot(tag: str) -> None:
    arr = sample_rgb()
    if arr is None:
        return
    path = OUT_DIR / f"walkts_{tag}.png"
    try:
        from PIL import Image

        Image.fromarray(arr).save(str(path))
        S["shots"].append(str(path))
    except Exception as exc:                                      # noqa: BLE001
        log(f"  ! could not write {path}: {exc!r}")


# ---------------------------------------------------------------------------
def start_arm(name: str) -> None:
    S["phase"] = name
    S["n"] = 0
    S["prev"] = None
    S["last_t"] = None
    _, variant, write, read = BY_NAME[name]
    log(f"--- arm {name}  (variant={variant} write={write} readback={read}) ---")


def end_arm(name: str) -> None:
    ch = S["changed"].get(name) or []
    _, variant, write, read = BY_NAME[name]
    rec = {"arm": name, "variant": variant, "write": write, "readback": read,
           "frames": len(S["times"].get(name) or []),
           "median_ms": med_ms("times", name),
           "readback_ms": med_ms("readback", name),
           "pixel_diff_ms": med_ms("diff", name),
           "changed_px_median": int(statistics.median(ch)) if ch else None,
           "changed_px_max": max(ch) if ch else None}
    S["report"].setdefault("arms", {})[name] = rec
    append_jsonl(rec)
    log(f"    {name}: {rec['median_ms']} ms/frame"
        + (f"  (get_data {rec['readback_ms']} ms, diff {rec['pixel_diff_ms']} ms)"
           if read else "")
        + f"  changed px median {rec['changed_px_median']}")

    S["arm"] += 1
    if S["arm"] >= len(ARMS):
        S["phase"] = "done"
        return
    nxt = ARMS[S["arm"]]
    if nxt[1] != S["variant"]:
        if nxt[1] == "shipped":
            if S["variant"] == "sampled":
                S["report"]["sampled_structure_after_writing"] = anim_structure(
                    ANIM_PATH)
                sampled_readback_check()
            restore_shipped_animation()
        elif not build(nxt[1]):
            S["phase"] = "done"
            return
        S["variant"] = nxt[1]
    start_arm(nxt[0])


def report() -> None:
    r = S["report"]
    arms = r.get("arms", {})

    def ms(name):
        return (arms.get(name) or {}).get("median_ms")

    pairs = {
        "shipped_noread": ("shipped_noread_1", "shipped_noread_2"),
        "shipped_read": ("shipped_read_1", "shipped_read_2"),
    }
    for tag, (a, b) in pairs.items():
        va, vb = ms(a), ms(b)
        if va and vb:
            r[f"drift_{tag}_ms"] = round(vb - va, 2)
            r[f"baseline_{tag}_ms"] = round(statistics.mean([va, vb]), 2)

    base_nr = r.get("baseline_shipped_noread_ms")
    base_rd = r.get("baseline_shipped_read_ms")
    if base_nr and base_rd:
        r["cost_of_readback_ms"] = round(base_rd - base_nr, 2)
    for tag, arm, base in (("default_write", "default_write_noread", base_nr),
                           ("sampled_write", "sampled_write_noread", base_nr),
                           ("default_write_read", "default_write_read", base_rd),
                           ("sampled_write_read", "sampled_write_read", base_rd),
                           ("default_static_read", "default_static_read", base_rd)):
        v = ms(arm)
        if v and base:
            r[f"cost_{tag}_ms"] = round(v - base, 2)
    d, t = ms("default_write_noread"), ms("sampled_write_noread")
    if d and t:
        r["sampled_minus_default_noread_ms"] = round(t - d, 2)
    d, t = ms("default_write_read"), ms("sampled_write_read")
    if d and t:
        r["sampled_minus_default_read_ms"] = round(t - d, 2)

    print("\n" + "=" * 82, flush=True)
    print("DOES TIME-SAMPLING THE GAIT RECOVER THE COST OF BINDING IT?", flush=True)
    print("=" * 82, flush=True)
    print(f"  {'arm':<22}{'ms/frame':>9}{'get_data':>10}{'diff':>7}"
          f"{'changed px':>12}", flush=True)
    for name, _v, _w, _rd in ARMS:
        a = arms.get(name) or {}
        def f(k, w):
            v = a.get(k)
            return f"{v:>{w}}" if v is not None else f"{'--':>{w}}"
        print(f"  {name:<22}{f('median_ms', 9)}{f('readback_ms', 10)}"
              f"{f('pixel_diff_ms', 7)}{f('changed_px_median', 12)}", flush=True)
    print("-" * 82, flush=True)
    print(f"  shipped baseline, readback OFF   {base_nr} ms "
          f"(drift between its two arms {r.get('drift_shipped_noread_ms')})", flush=True)
    print(f"  shipped baseline, readback ON    {base_rd} ms "
          f"(drift {r.get('drift_shipped_read_ms')})", flush=True)
    print(f"  PRICE OF THE READBACK            {r.get('cost_of_readback_ms')} ms/frame",
          flush=True)
    print(f"  cost of the gait, readback OFF   "
          f"default {r.get('cost_default_write_ms')}   "
          f"sampled {r.get('cost_sampled_write_ms')}", flush=True)
    print(f"  cost of the gait, readback ON    "
          f"default {r.get('cost_default_write_read_ms')}   "
          f"sampled {r.get('cost_sampled_write_read_ms')}", flush=True)
    print(f"  bound-but-never-written (row B)  "
          f"{r.get('cost_default_static_read_ms')} ms vs its OWN baseline", flush=True)
    print(f"  sampled - default                "
          f"{r.get('sampled_minus_default_noread_ms')} ms (no readback), "
          f"{r.get('sampled_minus_default_read_ms')} ms (readback)", flush=True)
    print(f"  sampled writes {r.get('sampled_writes')}, readback max delta "
          f"{r.get('sampled_readback_max_delta')} "
          f"(0 would mean the gait never varied)", flush=True)
    print(f"  rotations, ours sampled  "
          f"{(r.get('sampled_structure_after_writing') or {}).get('rotations')}",
          flush=True)
    print(f"  rotations, shipped clip  "
          f"{(r.get('shipped_structure') or {}).get('rotations')}", flush=True)
    print("=" * 82, flush=True)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    p = OUT_DIR / "walk_timesample.json"
    with open(p, "w") as fh:
        json.dump(r, fh, indent=1, default=str)
        fh.flush()
        os.fsync(fh.fileno())
    log(f"wrote {p}")


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
                start_arm(ARMS[0][0])
            return

        if ph == "done":
            report()
            log("DONE")
            S["sub"] = None
            omni.kit.app.get_app().post_quit(0)
            return

        # --- a measurement arm -------------------------------------------
        now = time.perf_counter()
        if S["last_t"] is not None:
            S["times"].setdefault(ph, []).append(now - S["last_t"])
        S["last_t"] = now

        _, _variant, write, read = BY_NAME[ph]
        S["n"] += 1
        cyc = S["cycle"]
        if write and cyc is not None and cyc.ok:
            cyc.speed = 2.0
            cyc.update((-STEP_M * S["n"], 0.0))
        if read:
            t0 = time.perf_counter()
            arr = sample_rgb()
            t1 = time.perf_counter()
            c = changed_pixels(arr)
            t2 = time.perf_counter()
            S["readback"].setdefault(ph, []).append(t1 - t0)
            S["diff"].setdefault(ph, []).append(t2 - t1)
            if c is not None and S["n"] > 3:
                S["changed"].setdefault(ph, []).append(c)
            if write and S["n"] in (8, 24):
                shoot(f"{ph}_{S['n']:02d}")
        if S["n"] >= FRAMES:
            end_arm(ph)
        return
    except Exception as exc:                                      # noqa: BLE001
        log("FAILED: " + repr(exc))
        log(traceback.format_exc())
        S["report"]["error"] = repr(exc)
        try:
            report()
        except Exception:                                         # noqa: BLE001
            pass
        log("DONE")
        S["sub"] = None
        omni.kit.app.get_app().post_quit(1)


log(f"open_stage {STAGE} -> {omni.usd.get_context().open_stage(STAGE)}")
S["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    on_update, name="walk_timesample")
