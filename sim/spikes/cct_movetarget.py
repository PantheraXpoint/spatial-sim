"""Does the PhysX character controller read ``physxCharacterController:moveTarget``
when it is authored from USD, or is ``set_move()`` the only way in?

Why it matters (S6 follow-up): the avatar's movement is world-axis because the
shipped control loop, ``omni.physxcct.scripts.utils.update_movement``, builds
``Gf.Vec3f(forward, right, 0)`` in WORLD space and never applies yaw. Turning
therefore needs either

  * the built-in first-person mode -- which recentres the OS cursor every frame
    and hijacks the active viewport, both wrong over a livestream; or
  * our own movement vector, which an OmniGraph could compute (yaw-rotated) and
    write -- but only if the controller reads a USD attribute.

``PhysxCharacterControllerAPI`` declares ``vector3f moveTarget`` with the
docstring "Desired target position that CCT should try to reach". If that is
live, turning costs a handful of graph nodes and no script, no custom node and
no cursor capture. If it is inert, turning stays off.

Method -- four phases on an empty stage, each measuring HORIZONTAL displacement
of the capsule over 60 physics frames:

  0. settle      no input. Establishes the noise floor (gravity, sway).
  A. absolute    moveTarget = current position + 1 m in +X, per the schema's
                 "target position" wording.
  B. delta       moveTarget = a small per-frame displacement, in case the
                 wording is loose and it behaves like set_move's argument.
  C. set_move    the shipped path, as a CONTROL. If this does not move the
                 capsule, the rig is broken and a negative result in A/B means
                 nothing.

The control phase is the point. "It didn't move" is only evidence if something
in the same script did move.

Execution model: physics and USD only -- no sensors, no annotators, no
rendering -- so exec mode does not apply and SimulationApp is correct. Empty
stage on purpose: the warehouse's 3,469 exact triangle-mesh colliders cost a
multi-minute PhysX cook and nothing here needs them.

Run::

    docker compose -f docker/docker-compose.yml run --rm sim \\
        ./python.sh /workspace/sim/spikes/cct_movetarget.py
"""

from __future__ import annotations

import os
import sys

from isaacsim import SimulationApp  # noqa: I001  -- must be first (hard rule 3)

_APP = SimulationApp({"headless": True})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.experimental.utils.app import enable_extension  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: E402

CAPSULE = "/World/cct_capsule"
RADIUS, CYL = 0.30, 1.15
HALF = CYL / 2.0 + RADIUS
FRAMES = 60
STEP = 0.02          # m per frame ~= 1.2 m/s at 60 Hz
MOVED = 0.10         # m of horizontal travel that counts as "it moved"

P = lambda *a: print(*a, flush=True)  # noqa: E731


def horizontal(a: Gf.Vec3d, b: Gf.Vec3d) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def main() -> int:
    # Step markers, because the first attempt segfaulted with no Python
    # traceback and nothing printed: a native crash localises to whatever
    # marker was last on stdout, and costs nothing.
    P("step: new stage")
    ctx = omni.usd.get_context()
    ctx.new_stage()
    for _ in range(10):
        _APP.update()
    stage = ctx.get_stage()

    P("step: stage metadata")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    P("step: physics scene")
    scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.8)

    P("step: ground plane")
    # Shipped helper rather than a hand-rolled collider (hard rule 4).
    PhysicsSchemaTools.addGroundPlane(stage, "/groundPlane", "Z", 100.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    P("step: enable omni.physx.cct")
    # Enabled AFTER the stage exists, mirroring the order that works in
    # sim/avatar.py. Enabling it first was the shape of the crashing run.
    enable_extension("omni.physx.cct")
    for _ in range(20):
        _APP.update()

    P("step: import cct utils")
    from omni.physxcct.scripts import utils as cct_utils
    from omni.physxcct.scripts.ifaces import get_physx_cct_interface

    P("step: capsule")
    capsule = UsdGeom.Capsule.Define(stage, CAPSULE)
    capsule.CreateAxisAttr(UsdGeom.Tokens.z)
    capsule.CreateRadiusAttr(RADIUS)
    capsule.CreateHeightAttr(CYL)
    capsule.CreateExtentAttr([(-RADIUS, -RADIUS, -HALF), (RADIUS, RADIUS, HALF)])
    capsule.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, HALF + 0.05))
    capsule.AddOrientOp().Set(Gf.Quatf(1.0))
    capsule.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))

    P("step: construct CharacterController")
    cct = cct_utils.CharacterController(CAPSULE, None, True, 0.01)
    P("step: activate")
    cct.activate(stage)
    for _ in range(5):
        _APP.update()
    P("step: get interface")
    iface = get_physx_cct_interface()

    prim = stage.GetPrimAtPath(CAPSULE)
    api = PhysxSchema.PhysxCharacterControllerAPI(prim)
    attr = api.GetMoveTargetAttr() or api.CreateMoveTargetAttr()
    P(f"\nmoveTarget attribute: {attr.GetPath()}  type={attr.GetTypeName()}")
    P(f"CCT interface methods: {[m for m in dir(iface) if not m.startswith('_')]}")

    omni.timeline.get_timeline_interface().play()
    for _ in range(10):
        _APP.update()

    cache = UsdGeom.XformCache()

    def pos() -> Gf.Vec3d:
        cache.Clear()
        return cache.GetLocalToWorldTransform(prim).ExtractTranslation()

    def run(label: str, per_frame) -> tuple[float, float, float]:
        """Signed per-axis displacement. Signs matter as much as magnitudes:
        they decide which way W actually drives, and therefore which way the
        avatar's character must face and its cameras must look."""
        start = pos()
        for _ in range(FRAMES):
            per_frame()
            _APP.update()
        end = pos()
        d = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
        P(f"\n[{label}]")
        P(f"  start  ({start[0]:8.4f}, {start[1]:8.4f}, {start[2]:8.4f})")
        P(f"  end    ({end[0]:8.4f}, {end[1]:8.4f}, {end[2]:8.4f})")
        P(f"  delta  ({d[0]:+8.4f}, {d[1]:+8.4f}, {d[2]:+8.4f})  over {FRAMES} frames")
        P(f"  horizontal travel: {horizontal(end, start):.4f} m")
        if abs(d[2]) > 1.0:
            P("  !! the capsule left the ground -- this phase is not trustworthy")
        return d

    # NOTHING below ever writes a POSITION into moveTarget. The first run of
    # this spike did, on the strength of the schema's "target position" wording,
    # and the capsule's z doubled every frame (z <- z + z) until it reached
    # 1e18 -- which is itself the proof that the value is consumed as a
    # DISPLACEMENT, and which contaminated every phase after it. Small deltas
    # only, so the capsule stays on the ground and each phase is independent.
    expect = STEP * FRAMES

    noise = run("0 settle: no input", lambda: None)
    zero = Gf.Vec3f(0.0, 0.0, 0.0)

    def quiet():
        attr.Set(zero)
        for _ in range(10):
            _APP.update()

    a = run("A moveTarget delta, +X", lambda: attr.Set(Gf.Vec3f(STEP, 0.0, 0.0)))
    quiet()
    b = run("B moveTarget delta, +Y", lambda: attr.Set(Gf.Vec3f(0.0, STEP, 0.0)))
    quiet()
    c = run("C set_move(+X) -- the shipped path, as a control",
            lambda: iface.set_move(CAPSULE, (STEP, 0.0, 0.0)))

    P("\n" + "=" * 78)
    P("VERDICT")
    P("=" * 78)
    P(f"  commanded per phase        : {expect:+.4f} m on one axis")
    P(f"  0 noise floor              : {noise}")
    P(f"  A moveTarget +X            : {a}")
    P(f"  B moveTarget +Y            : {b}")
    P(f"  C set_move   +X (control)  : {c}")
    P("")
    moved_c = horizontal((0, 0, 0), (c[0], c[1], 0)) if False else (c[0] ** 2 + c[1] ** 2) ** 0.5
    moved_a = (a[0] ** 2 + a[1] ** 2) ** 0.5
    moved_b = (b[0] ** 2 + b[1] ** 2) ** 0.5
    if moved_c < MOVED:
        P("  INCONCLUSIVE: the control phase did not move the capsule either, so")
        P("  this rig proves nothing about moveTarget.")
        code = 2
    elif moved_a >= MOVED and moved_b >= MOVED:
        P("  YES -- authoring physxCharacterController:moveTarget from USD drives")
        P("  the controller, as a per-frame DISPLACEMENT in the same units as")
        P("  set_move(). Turning can be wired through an OmniGraph: no script, no")
        P("  custom node, no cursor capture.")
        P(f"  moveTarget +X vs set_move +X agree: {abs(a[0] - c[0]) < 1e-3}")
        P(f"  SIGN: +X command produced {'+X' if a[0] > 0 else '-X'} motion; "
          f"+Y command produced {'+Y' if b[1] > 0 else '-Y'} motion")
        code = 0
    else:
        P("  NO -- authoring moveTarget from USD does nothing. set_move() is the")
        P("  only way in, so yaw-rotated movement needs Python running in the GUI")
        P("  session (script or custom OG node). Turning stays off.")
        code = 1
    P("=" * 78)
    return code


if __name__ == "__main__":
    rc = main()
    # Kit's teardown can abort with "Destroying busy TaskGroup!" and overwrite a
    # perfectly good result with a nonzero exit -- see sim/avatar.py.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)
