"""What actually buys frames: colliders, or resolution?

Both levers measured in ONE process so the machine's load is the same for
every phase. Absolute FPS here is not reproducible -- this is a shared box and
another user's jobs are loading the CPU, which matters because PhysX is
CPU-bound -- so the DELTAS are the result and the load average is reported
beside them.

Phases, all at Play with the avatar in the scene:

  A  everything as shipped                       baseline
  B  colliders above REACH_M disabled            physics lever
  C  B, at reduced viewport resolution           rendering lever

Physics was already shown to be the dominant cost (stopped 14.6 / playing 7.8 /
paused 14.7 fps), so B is expected to matter and C is expected not to. Expected
is not measured.

Also captures the overview PNG at the end, so the station marker's new
placement is checked by looking rather than asserted.

Exec mode. Environment: SF_STAGE, SF_OUT, SF_REACH_M, SF_LOWRES
"""

from __future__ import annotations

import json
import os
import sys
import time
from collections import defaultdict
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

import avatar as av  # noqa: E402
import sensor_factory as sf  # noqa: E402

STAGE = os.environ.get("SF_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
OUT = Path(os.environ.get("SF_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
REACH_M = float(os.environ.get("SF_REACH_M", "2.2"))
LOWRES = tuple(int(v) for v in os.environ.get("SF_LOWRES", "960,540").split(","))
WARM, MEASURE = 20, 90

S: dict = {"phase": "loading", "frame": 0, "n": 0, "sub": None,
           "times": defaultdict(list), "last_t": None, "follow": None}


def log(m: str) -> None:
    print(f"[fps] {m}", flush=True)


def loadavg() -> str:
    try:
        return open("/proc/loadavg").read().split(" ")[0:3].__str__()
    except Exception:
        return "?"


def tick(phase):
    now = time.perf_counter()
    if S["last_t"] is not None:
        S["times"][phase].append(now - S["last_t"])
    S["last_t"] = now


def fps(phase):
    v = S["times"].get(phase) or []
    if len(v) < 5:
        return None
    v = sorted(v)[len(v) // 10: max(1, len(v) - len(v) // 10)]
    m = sum(v) / len(v)
    return round(1.0 / m, 2) if m > 0 else None


def disable_high_colliders(stage, reach_m: float) -> int:
    """Turn off collision on anything the avatar can never reach.

    Collision only -- these prims keep their render geometry, so the cameras
    and the lidar (which trace the render BVH, not colliders) see exactly what
    they saw before. Nothing about the picture changes.
    """
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n = 0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not prim.GetPath().pathString.startswith("/Root/Warehouse"):
            continue
        api = UsdPhysics.CollisionAPI(prim)
        if api.GetCollisionEnabledAttr().Get() is False:
            continue
        try:
            rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if rng.IsEmpty() or float(rng.GetMin()[2]) <= reach_m:
                continue
        except Exception:
            continue
        api.CreateCollisionEnabledAttr().Set(False)
        n += 1
    return n


def setup():
    stage = omni.usd.get_context().get_stage()
    registry = sf.load_registry()
    sf.create_stations(stage)
    S["made"] = sf.create_registry_sensors(stage, registry, render_products=False,
                                           attach_annotators=False)
    S["follow"] = av.install_character_follow(stage)

    target = sf.avatar_target(stage)
    eye = Gf.Vec3d(-6.0, -1.0, 5.5)
    cam = UsdGeom.Camera.Define(stage, "/Root/_Diag/overview_cam")
    cam.AddTranslateOp().Set(eye)
    cam.AddRotateXYZOp().Set(sf.look_at_rotate_xyz(eye, target))
    cam.CreateFocalLengthAttr(16.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 1_000_000.0))
    try:
        from omni.kit.viewport.utility import get_active_viewport

        vp = get_active_viewport()
        vp.set_active_camera("/Root/_Diag/overview_cam")
        S["viewport"] = vp
        S["res0"] = tuple(vp.resolution)
        log(f"viewport {S['res0']}")
    except Exception as exc:
        log(f"! viewport unavailable: {exc!r}")

    log(f"load average at start: {loadavg()}")
    omni.timeline.get_timeline_interface().play()


def capture(tag):
    vp = S.get("viewport")
    if vp is None:
        return
    try:
        from omni.kit.viewport.utility import capture_viewport_to_file

        path = str(OUT / f"fps_{tag}.png")
        capture_viewport_to_file(vp, file_path=path)
        log(f"capture -> {path}")
    except Exception as exc:
        log(f"! capture failed: {exc!r}")


def finish():
    a, b, c = fps("A_baseline"), fps("B_colliders_off"), fps("C_lowres")
    log("=" * 72)
    log(f"  A baseline                     {a} fps")
    log(f"  B colliders>{REACH_M}m disabled ({S.get('disabled')})   {b} fps")
    log(f"  C B + viewport {LOWRES}       {c} fps")
    if a and b:
        log(f"  => collider lever: {round((b - a) / a * 100, 1):+}%")
    if b and c:
        log(f"  => resolution lever: {round((c - b) / b * 100, 1):+}%")
    log(f"  load average at end: {loadavg()}")
    (OUT / "fps.json").write_text(json.dumps(
        {"A_baseline": a, "B_colliders_off": b, "C_lowres": c,
         "disabled": S.get("disabled"), "reach_m": REACH_M, "lowres": LOWRES,
         "loadavg": loadavg(), "res0": S.get("res0")}, default=str))
    log("DONE")


def on_update(_e):
    S["frame"] += 1
    try:
        ph = S["phase"]
        if ph == "loading":
            if S["frame"] > 5 and not any(omni.usd.get_context().get_stage_loading_status()[1:]):
                setup()
                S["phase"], S["n"], S["last_t"] = "A_baseline", 0, None
                log("phase A: baseline")
            return

        if ph in ("A_baseline", "B_colliders_off", "C_lowres"):
            S["n"] += 1
            if S["n"] > WARM:
                tick(ph)
            if S["n"] < WARM + MEASURE:
                return

            stage = omni.usd.get_context().get_stage()
            if ph == "A_baseline":
                capture("A_baseline")
                omni.timeline.get_timeline_interface().stop()
                S["disabled"] = disable_high_colliders(stage, REACH_M)
                log(f"disabled {S['disabled']} colliders entirely above {REACH_M} m")
                omni.timeline.get_timeline_interface().play()
                S["phase"], S["n"], S["last_t"] = "B_colliders_off", 0, None
                log("phase B: colliders off")
            elif ph == "B_colliders_off":
                vp = S.get("viewport")
                if vp is not None:
                    try:
                        vp.resolution = LOWRES
                        log(f"viewport resolution -> {LOWRES}")
                    except Exception as exc:
                        log(f"! could not set resolution: {exc!r}")
                S["phase"], S["n"], S["last_t"] = "C_lowres", 0, None
                log("phase C: low resolution")
            else:
                capture("C_final")
                finish()
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
    on_update, name="fps"
)
