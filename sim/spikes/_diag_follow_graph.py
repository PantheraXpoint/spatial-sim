"""Why does the character not follow the capsule at Play -- and why does Play cost 5 FPS?

The follow graph is correctly authored: execIn and value both connected,
usePath 1 on both nodes, evaluator "execution", pipeline stage simulation. It
still does nothing. Reading it again cannot settle that, so this measures it.

Three questions, one run, because the container is exclusive and each launch
costs four minutes of startup.

A. WHY DOESN'T THE CHARACTER FOLLOW?
   Four causes look identical from outside:
     1. the graph never ticks         -> follow_tick compute count stays 0
     2. it ticks but reads a constant -> the CCT writes the capsule pose to
                                         FABRIC, not USD, so a USD-level
                                         ReadPrimAttribute copies a stale value
     3. it ticks and writes, and the write lands nowhere visible
     4. it works here and fails only in the GUI
   So the capsule and character poses are read through BOTH USD and Fabric
   every frame, alongside get_compute_count() on every node.

B. IS THE FPS DROP PHYSICS OR RENDERING?
   Measured, not argued: frame intervals over three phases -- STOPPED (render
   only), PLAYING (render + physics), then PAUSED (render only again). If
   PLAYING is slow and PAUSED recovers, it is physics. Rendering is identical
   in all three.

C. DO ALL 3,469 WAREHOUSE COLLIDERS NEED TO BE ACTIVE?
   A census by height: the avatar is 1.75 m and touches the floor and the near
   faces of shelving. Anything whose collision geometry sits entirely above
   head height cannot ever be touched by it.

Exec mode. Environment:
    SF_STAGE   stage (default /workspace/sim/observatory_avatar.usd)
    SF_OUT     results dir (default /isaac-sim/.nvidia-omniverse/logs)

    ./runheadless.sh --exec /workspace/sim/spikes/_diag_follow_graph.py
"""

from __future__ import annotations

import json
import os
import time
from collections import defaultdict
from pathlib import Path

import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.experimental.utils.app import enable_extension
from pxr import Usd, UsdGeom, UsdPhysics

STAGE = os.environ.get("SF_STAGE", "/workspace/sim/observatory_avatar.usd")
OUT = Path(os.environ.get("SF_OUT", "/isaac-sim/.nvidia-omniverse/logs"))

BODY = "/Root/Avatar/body_mesh"
CHAR = "/Root/Avatar/character"
GRAPH = "/Root/Avatar/Controls"
NODES = ("follow_tick", "read_body", "write_character", "cct")
STEP = 0.02          # m/frame, the magnitude the auto-controls use at 1 m/s
REACH_M = 2.2        # a 1.75 m avatar's plausible touch height, generously

WARM, MEASURE, PLAY_FRAMES = 10, 30, 120


def log(m: str) -> None:
    print(f"[diag_follow] {m}", flush=True)


class Results:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(path, "w", encoding="utf-8")

    def write(self, **rec) -> None:
        self._fh.write(json.dumps(rec, default=str) + "\n")
        self._fh.flush()
        os.fsync(self._fh.fileno())


results = Results(OUT / "diag_follow_graph.jsonl")
ctx = omni.usd.get_context()
S: dict = {"phase": "loading", "frame": 0, "n": 0, "sub": None, "rt": None,
           "iface": None, "times": defaultdict(list), "last_t": None}


def usd_translate(stage, path):
    attr = stage.GetPrimAtPath(path).GetAttribute("xformOp:translate")
    v = attr.Get() if attr else None
    return [round(float(x), 4) for x in v] if v is not None else None


def fabric_translate(path):
    rt = S.get("rt")
    if rt is None:
        return None
    try:
        prim = rt.GetPrimAtPath(path)
        if not prim:
            return None
        for name in ("_worldPosition", "xformOp:translate"):
            attr = prim.GetAttribute(name)
            if attr:
                v = attr.Get()
                if v is not None:
                    return [round(float(x), 4) for x in v]
        return None
    except Exception:
        return None


def counts():
    out = {}
    for name in NODES:
        try:
            out[name] = int(og.Controller.node(f"{GRAPH}/{name}").get_compute_count())
        except Exception as exc:
            out[name] = f"<{type(exc).__name__}>"
    return out


def collider_census(stage):
    """How much of the warehouse's collision can the avatar ever touch?"""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    total = reachable = above = disabled = 0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not prim.GetPath().pathString.startswith("/Root/Warehouse"):
            continue
        total += 1
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            disabled += 1
            continue
        try:
            rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            if float(rng.GetMin()[2]) > REACH_M:
                above += 1
            else:
                reachable += 1
        except Exception:
            pass
    out = {"warehouse_colliders": total, "disabled": disabled,
           "entirely_above_%.1fm" % REACH_M: above, "reachable": reachable}
    log(f"collider census: {out}")
    results.write(event="collider_census", reach_m=REACH_M, **out)
    return out


def setup():
    stage = ctx.get_stage()
    log(f"stage root {stage.GetDefaultPrim().GetPath()}")
    try:
        from usdrt import Usd as RtUsd

        S["rt"] = RtUsd.Stage.Attach(ctx.get_stage_id())
        log("usdrt attached")
    except Exception as exc:
        log(f"! usdrt unavailable: {exc!r}")

    try:
        for g in og.get_all_graphs():
            log(f"graph {g.get_path_to_graph()} pipeline={g.get_pipeline_stage()} "
                f"disabled={g.is_disabled()}")
    except Exception as exc:
        log(f"! graph query failed: {exc!r}")

    log(f"counts before Play: {counts()}")
    collider_census(stage)

    enable_extension("omni.physx.cct")
    try:
        from omni.physxcct.scripts.ifaces import get_physx_cct_interface

        S["iface"] = get_physx_cct_interface()
    except Exception as exc:
        log(f"! cct interface unavailable: {exc!r}")


def tick_time(phase):
    now = time.perf_counter()
    if S["last_t"] is not None:
        S["times"][phase].append(now - S["last_t"])
    S["last_t"] = now


def fps(phase):
    v = S["times"].get(phase) or []
    if not v:
        return None
    v = sorted(v)[len(v) // 10: max(1, len(v) - len(v) // 10)]  # trim outliers
    mean = sum(v) / len(v)
    return round(1.0 / mean, 2) if mean > 0 else None


def read_body_output():
    """What the reader node actually hands downstream.

    This is the whole disambiguation. If the capsule moves and this stays
    constant, the READ is stale. If this tracks the capsule and the character
    does not move, the WRITE is not landing. Nothing else separates them.
    """
    try:
        node = og.Controller.node(f"{GRAPH}/read_body")
        val = og.Controller.get(og.Controller.attribute("outputs:value", node))
        return [round(float(x), 4) for x in val] if val is not None else None
    except Exception as exc:
        return f"<{type(exc).__name__}>"


def node_state(stage, name, attr):
    a = stage.GetPrimAtPath(f"{GRAPH}/{name}").GetAttribute(attr)
    return a.Get() if a else None


def sample_play():
    stage = ctx.get_stage()
    # set_position, not set_move: set_move is consumed by a pre-physics stage
    # update node, so calling it from the app update stream lands in the wrong
    # phase and does nothing -- 150 frames of it moved the capsule 0 mm in X.
    if S["iface"] is not None and S.get("home"):
        try:
            x, y, z = S["home"]
            S["iface"].set_position(BODY, (x + S["n"] * STEP, y, z))
        except Exception as exc:
            if S["n"] == 0:
                log(f"! set_position failed: {exc!r}")
    rec = {"frame": S["n"], "body_usd": usd_translate(stage, BODY),
           "body_fabric": fabric_translate(BODY), "char_usd": usd_translate(stage, CHAR),
           "char_fabric": fabric_translate(CHAR), "read_out": read_body_output(),
           "counts": counts()}
    if S["n"] % 30 == 0 or S["n"] == PLAY_FRAMES - 1:
        log(f"  f{rec['frame']:>4} body_usd={rec['body_usd']} read_out={rec['read_out']} "
            f"char_usd={rec['char_usd']} char_fab={rec['char_fabric']}")
        results.write(event="sample", **rec)
    S.setdefault("first", rec)
    S["last"] = rec


def finish():
    first, last = S.get("first"), S.get("last")
    log("=" * 74)
    log("A. FOLLOW GRAPH")
    log(f"   body   USD    {first['body_usd']} -> {last['body_usd']}")
    log(f"   body   FABRIC {first['body_fabric']} -> {last['body_fabric']}")
    log(f"   char   USD    {first['char_usd']} -> {last['char_usd']}")
    log(f"   char   FABRIC {first['char_fabric']} -> {last['char_fabric']}")
    log(f"   counts {first['counts']} -> {last['counts']}")

    ticked = isinstance(last["counts"].get("follow_tick"), int) and last["counts"]["follow_tick"] > 0
    body_usd_moved = first["body_usd"] != last["body_usd"]
    body_fab_moved = last["body_fabric"] is not None and first["body_fabric"] != last["body_fabric"]
    char_moved = first["char_usd"] != last["char_usd"] or (
        last["char_fabric"] is not None and first["char_fabric"] != last["char_fabric"])

    read_moved = (isinstance(first.get("read_out"), list)
                  and first["read_out"] != last.get("read_out"))
    log(f"   read_body outputs:value {first.get('read_out')} -> {last.get('read_out')}")
    stage = ctx.get_stage()
    for n in ("read_body", "write_character"):
        log(f"   {n}.state:correctlySetup = {node_state(stage, n, 'state:correctlySetup')}")
    log(f"   write_character.inputs:usdWriteBack = "
        f"{node_state(stage, 'write_character', 'inputs:usdWriteBack')}")

    if not ticked:
        verdict = "GRAPH NEVER TICKS -- follow_tick compute count stayed 0"
    elif body_usd_moved and not read_moved:
        verdict = ("READ IS STALE -- the capsule moves but read_body's own output does "
                   "not, so the graph faithfully writes a constant")
    elif read_moved and not char_moved:
        verdict = ("WRITE DOES NOT LAND -- read_body tracks the capsule, write_character "
                   "computes, and the character's translate never changes")
    elif char_moved:
        verdict = "FOLLOW WORKS HEADLESS -- reproduces only in the GUI"
    elif not body_usd_moved and body_fab_moved:
        verdict = ("TICKS BUT READS A STALE VALUE -- the capsule moves in FABRIC and not "
                   "in USD, so the USD-level read copies a constant")
    elif not body_usd_moved and not body_fab_moved:
        verdict = "THE CAPSULE DID NOT MOVE AT ALL HERE -- set_move never reached the controller"
    else:
        verdict = "TICKS AND READS A MOVING VALUE, BUT THE WRITE DOES NOT LAND"
    log(f"   => {verdict}")

    log("")
    log("B. WHERE THE FRAME TIME GOES")
    for phase in ("stopped", "playing", "paused"):
        log(f"   {phase:8s} {fps(phase)} fps   ({len(S['times'].get(phase) or [])} frames)")
    a, b, c = fps("stopped"), fps("playing"), fps("paused")
    if a and b and c:
        if b < a * 0.7 and c > b * 1.4:
            cause = ("PHYSICS. Rendering is identical in all three phases; only PLAYING "
                     "steps PhysX, and PAUSED recovers.")
        elif b < a * 0.7:
            cause = "SLOWER ON PLAY AND DOES NOT RECOVER WHEN PAUSED -- not a clean physics story"
        else:
            cause = "NO SIGNIFICANT DROP ON PLAY in this headless run"
        log(f"   => {cause}")
    else:
        cause = "timing unavailable"

    results.write(event="verdict", follow=verdict, fps_cause=cause,
                  fps={"stopped": a, "playing": b, "paused": c}, first=first, last=last)
    log("DONE")


def on_update(_e):
    S["frame"] += 1
    try:
        ph = S["phase"]
        if ph == "loading":
            if S["frame"] > 5 and not any(ctx.get_stage_loading_status()[1:]):
                setup()
                S["phase"], S["n"], S["last_t"] = "stopped", 0, None
                log(f"phase stopped: {WARM} warm + {MEASURE} measured frames")
            return

        if ph == "stopped":
            S["n"] += 1
            if S["n"] > WARM:
                tick_time("stopped")
            if S["n"] >= WARM + MEASURE:
                S["home"] = usd_translate(ctx.get_stage(), BODY)
                log(f"capsule home {S['home']}")
                omni.timeline.get_timeline_interface().play()
                S["phase"], S["n"], S["last_t"] = "playing", 0, None
                log(f"play() -- phase playing: {WARM} warm + {PLAY_FRAMES} frames")
            return

        if ph == "playing":
            S["n"] += 1
            if S["n"] > WARM:
                tick_time("playing")
            sample_play()
            if S["n"] >= WARM + PLAY_FRAMES:
                omni.timeline.get_timeline_interface().pause()
                S["phase"], S["n"], S["last_t"] = "paused", 0, None
                log(f"pause() -- phase paused: {WARM} warm + {MEASURE} frames")
            return

        if ph == "paused":
            S["n"] += 1
            if S["n"] > WARM:
                tick_time("paused")
            if S["n"] >= WARM + MEASURE:
                finish()
                S["phase"] = "done"
                S["sub"] = None
                omni.kit.app.get_app().post_quit()
            return
    except Exception as exc:
        import traceback

        log(f"FAILED: {exc!r}")
        results.write(event="error", error=repr(exc), tb=traceback.format_exc())
        S["sub"] = None
        omni.kit.app.get_app().post_quit()


log(f"open_stage {STAGE} -> {ctx.open_stage(STAGE)}")
S["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    on_update, name="diag_follow_graph"
)
log("subscribed")
