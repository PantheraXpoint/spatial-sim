"""Does the inspector read numbers that CHANGE when the avatar moves?

The requirement is not "a panel exists". It is: select lidar_01, walk toward
it, and watch the point count and depth move. So this drives exactly that and
asserts it, because a readout that renders a constant is this project's
recurring failure and has already cost three rounds.

  1. build the scene, install the inspector, press Play
  2. select the lidar through the real selection API, the same one the panel
     polls -- not by reaching past it
  3. walk the avatar toward the lidar in steps, calling read_stats() at each
  4. assert the point count and/or depth actually differ across the walk

read_stats() is the function the panel displays, so verifying it verifies the
panel's content. What is NOT verified here is omni.ui drawing pixels; that is
the trivial half.

Exec mode.  SF_STAGE, SF_OUT
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import omni.kit.app
import omni.timeline
import omni.usd

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for _p in (str(REPO), str(REPO / "sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
os.environ["SF_NO_AUTORUN"] = "1"

import avatar as av  # noqa: E402
import sensor_factory as sf  # noqa: E402
import sensor_inspector as si  # noqa: E402

STAGE = os.environ.get("SF_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
OUT = Path(os.environ.get("SF_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
BODY = "/Root/Avatar/body_mesh"
LIDAR = "INFRA_01_LIDAR"
STEPS, FRAMES_PER_STEP = 6, 25

S: dict = {"phase": "loading", "frame": 0, "n": 0, "step": 0, "sub": None,
           "samples": [], "follow": None}


def log(m: str) -> None:
    print(f"[inspector_test] {m}", flush=True)


def setup():
    stage = omni.usd.get_context().get_stage()
    sf.create_stations(stage)
    S["made"] = sf.create_registry_sensors(stage, sf.load_registry(),
                                           render_products=False, attach_annotators=False)
    S["follow"] = av.install_character_follow(stage)
    sf.disable_unreachable_colliders(stage)

    # Select through the real API -- the same call the panel polls.
    omni.usd.get_context().get_selection().set_selected_prim_paths(
        [S["made"][LIDAR]["prim_path"]], True)
    sel = omni.usd.get_context().get_selection().get_selected_prim_paths()
    sid, rec = si.find_selected(S["made"], sel)
    log(f"selection {sel} -> resolves to sensor {sid}")
    S["sid"], S["rec"] = sid, rec

    from omni.physxcct.scripts.ifaces import get_physx_cct_interface

    S["iface"] = get_physx_cct_interface()
    S["home"] = list(sf.avatar_target(stage))
    st = sf.load_stations()[0]["stage_position"]
    S["station"] = [float(v) for v in st]
    omni.timeline.get_timeline_interface().play()
    log(f"play(); walking the avatar from {[round(v,2) for v in S['home']]} "
        f"toward the station at {S['station']}")


def walk_step():
    """Move the avatar a fraction of the way toward the station."""
    home, st = S["home"], S["station"]
    f = S["step"] / float(STEPS)
    x = home[0] + (st[0] - home[0]) * 0.75 * f
    y = home[1] + (st[1] - home[1]) * 0.75 * f
    try:
        S["iface"].set_position(BODY, (x, y, 0.925))
    except Exception as exc:
        log(f"! set_position failed: {exc!r}")
    return x, y


def sample():
    stage = omni.usd.get_context().get_stage()
    stats = si.read_stats(S["rec"], stage)
    lines = si.format_lines(S["sid"], stats)
    S["samples"].append(stats)
    log(f"  step {S['step']}: points={stats.get('points')} "
        f"ON_AVATAR={stats.get('points_on_avatar')} "
        f"avatar_range={stats.get('avatar_range_m')} "
        f"depth_min={stats.get('depth_min_m')} frame={stats.get('frame')}")
    if S["step"] == 0:
        log("  panel would read:")
        for line in lines:
            log(f"    | {line}")


def finish():
    pts = [s.get("points") for s in S["samples"] if s.get("points") is not None]
    dmin = [s.get("depth_min_m") for s in S["samples"] if s.get("depth_min_m") is not None]
    dmax = [s.get("depth_max_m") for s in S["samples"] if s.get("depth_max_m") is not None]
    frames = [s.get("frame") for s in S["samples"] if s.get("frame") is not None]

    log("=" * 70)
    log(f"  points     {pts}")
    log(f"  depth_min  {dmin}")
    log(f"  depth_max  {dmax}")
    log(f"  frame      {frames[:3]}...{frames[-3:] if len(frames) > 3 else ''}")

    on_av = [s.get("points_on_avatar") for s in S["samples"]
             if s.get("points_on_avatar") is not None]
    rng = [s.get("avatar_range_m") for s in S["samples"] if s.get("avatar_range_m") is not None]
    log(f"  on_avatar  {on_av}")
    log(f"  avatar_rng {rng}")

    # The total count and the global depth extremes are asserted only as
    # liveness. They wander with scan phase and cannot show a person moving --
    # that is why points_on_avatar exists, and it is what has to respond.
    spread = (max(on_av) - min(on_av)) / max(1, max(on_av)) if on_av else 0
    checks = {
        "frame counter advances": len(set(frames)) > 1,
        "readout is not empty": bool(pts and pts[0]),
        "points land on the avatar at all": bool(on_av) and max(on_av) > 0,
        "points_on_avatar RESPONDS to the walk (>10% spread)": spread > 0.10,
        "avatar range tracks the walk": len(set(rng)) > 1 and (max(rng) - min(rng)) > 0.3,
    }
    for name, ok in checks.items():
        log(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    verdict = all(checks.values())
    log(f"  => {'INSPECTOR READS LIVE DATA' if verdict else 'SOMETHING IS CONSTANT -- look above'}")
    (OUT / "inspector_test.json").write_text(json.dumps(
        {"checks": checks, "samples": S["samples"]}, indent=1, default=str))
    log("DONE")


def on_update(_e):
    S["frame"] += 1
    try:
        if S["phase"] == "loading":
            if S["frame"] > 5 and not any(omni.usd.get_context().get_stage_loading_status()[1:]):
                setup()
                S["phase"], S["n"] = "walking", 0
            return
        if S["phase"] == "walking":
            S["n"] += 1
            if S["n"] % FRAMES_PER_STEP == 0:
                sample()
                S["step"] += 1
                if S["step"] > STEPS:
                    finish()
                    S["phase"] = "done"
                    S["sub"] = None
                    omni.kit.app.get_app().post_quit()
                    return
            walk_step()
    except Exception as exc:
        import traceback

        log(f"FAILED: {exc!r}")
        log(traceback.format_exc())
        S["sub"] = None
        omni.kit.app.get_app().post_quit()


log(f"open_stage {STAGE} -> {omni.usd.get_context().open_stage(STAGE)}")
S["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    on_update, name="inspector_test"
)
