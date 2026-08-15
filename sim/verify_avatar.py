"""Assert, headless, that the avatar is actually what it claims to be.

Part one of the S6 gate. Part two is visual and only a human in the GUI can do
it: in third person you can see your own body, and walking into a shelf stops
you. **Do not treat a green run here as S6 passing.** The entire reason this
task is the highest-risk one in the project is that these two can disagree --
every failure mode in play produces no error message, so USD can look perfect
while the demo is broken, and vice versa.

What it checks, and why each one is here (each fails silently in the sim):

    structure   the Xform, the body, both cameras, in the right places
    visible     purpose != guide, visibility != invisible. THIS is what RTX
                lidar/radar/cameras need: they trace render geometry, not
                colliders. NVIDIA's own character-controller demo ships a
                purpose="guide" capsule, which looks fine in the stage tree and
                is invisible to every sensor in the project.
    physics     a collision representation exists, and it is kinematic rather
                than dynamic -- accepting either the character controller or a
                literal kinematic rigid body, and rejecting a dynamic one
    semantics   semantics:labels:class contains 'person', or segmentation and
                bbox annotators come back empty
    cameras     both are descendants of the Avatar Xform, so switching
                first/third person is only a viewport rebind, and neither has
                the USD default 1 m near plane that would hide the warehouse
    controls    the character-controller node points at the REAL capsule path.
                A typo there is a graph that loads clean and does nothing.
    material    the non-visual base is written ('skin')

Execution model: this reads USD properties only -- no sensors, no annotators,
no rendering -- so exec mode does not apply and SimulationApp is correct here.
Kit is used for its asset resolver, so the warehouse's referenced props load
and the stage is the one the GUI will see.

Run::

    docker compose -f docker/docker-compose.yml run --rm sim \\
        ./python.sh /workspace/sim/verify_avatar.py

Exits non-zero on any failure.
"""

from __future__ import annotations

import sys

from isaacsim import SimulationApp  # noqa: I001  -- must be first (hard rule 3)

_APP = SimulationApp({"headless": True})

import carb  # noqa: E402
import omni.usd  # noqa: E402
import yaml  # noqa: E402
from pathlib import Path  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
DEFAULT_STAGE = REPO / "sim" / "observatory_avatar.usd"
SCENE_YAML = REPO / "config" / "scene.yaml"


class Checks:
    """Collect results instead of dying on the first one: a report of five
    failures is worth five runs of a script that stops at the first."""

    def __init__(self) -> None:
        self.rows: list[tuple[bool, str, str]] = []

    def check(self, ok: bool, name: str, detail: str = "") -> bool:
        self.rows.append((bool(ok), name, detail))
        return bool(ok)

    @property
    def failed(self) -> int:
        return sum(1 for ok, _, _ in self.rows if not ok)

    def report(self) -> None:
        print("\n" + "=" * 78, flush=True)
        print("S6 AVATAR VERIFICATION", flush=True)
        print("=" * 78, flush=True)
        for ok, name, detail in self.rows:
            mark = "PASS" if ok else "FAIL"
            print(f"  [{mark}] {name}", flush=True)
            if detail:
                print(f"         {detail}", flush=True)
        print("-" * 78, flush=True)
        n = len(self.rows)
        if self.failed:
            print(f"  {self.failed} of {n} checks FAILED", flush=True)
        else:
            print(f"  all {n} checks passed", flush=True)
            print("  Gate part 1 of 2. Part 2 is the GUI: third person shows", flush=True)
            print("  your body, and a shelf stops you. Do not advance on this alone.", flush=True)
        print("=" * 78, flush=True)


def verify(stage: Usd.Stage, cfg: dict) -> Checks:
    c = Checks()
    avatar_path = cfg["prim_path"]
    body_path = f"{avatar_path}/body_mesh"
    cam_paths = [f"{body_path}/cam_first_person", f"{body_path}/cam_third_person"]

    up_axis = UsdGeom.GetStageUpAxis(stage)

    # --- structure ---------------------------------------------------------
    avatar = stage.GetPrimAtPath(avatar_path)
    if not c.check(avatar.IsValid(), f"{avatar_path} exists", str(avatar.GetTypeName())):
        return c  # nothing downstream can mean anything
    c.check(avatar.IsA(UsdGeom.Xform), f"{avatar_path} is an Xform", str(avatar.GetTypeName()))

    # The controller writes a world pose back onto the body. A parent that is
    # not identity turns that into a teleport on the first simulated frame.
    xf = UsdGeom.Xformable(avatar)
    world = UsdGeom.XformCache().GetLocalToWorldTransform(avatar)
    c.check(
        world == Gf.Matrix4d(1.0),
        f"{avatar_path} is at identity",
        f"xformOps={[o.GetOpName() for o in xf.GetOrderedXformOps()]}",
    )

    body = stage.GetPrimAtPath(body_path)
    if not c.check(body.IsValid(), f"{body_path} exists", str(body.GetTypeName())):
        return c
    c.check(body.IsA(UsdGeom.Capsule), f"{body_path} is renderable geometry", str(body.GetTypeName()))

    # --- visible to sensors ------------------------------------------------
    img = UsdGeom.Imageable(body)
    purpose = img.ComputePurpose()
    c.check(
        purpose == UsdGeom.Tokens.default_,
        "body purpose is 'default', not 'guide'",
        f"purpose={purpose} (guide/proxy geometry is invisible to RTX sensors)",
    )
    vis = img.ComputeVisibility()
    c.check(vis != UsdGeom.Tokens.invisible, "body is visible", f"visibility={vis}")

    capsule = UsdGeom.Capsule(body)
    c.check(
        capsule.GetAxisAttr().Get() == up_axis,
        "capsule axis matches stage up axis",
        f"axis={capsule.GetAxisAttr().Get()} up={up_axis}",
    )

    # --- physics -----------------------------------------------------------
    has_cct = body.HasAPI(PhysxSchema.PhysxCharacterControllerAPI)
    has_rb = body.HasAPI(UsdPhysics.RigidBodyAPI)
    has_col = body.HasAPI(UsdPhysics.CollisionAPI)
    kinematic_rb = bool(UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr().Get()) if has_rb else False

    c.check(
        has_cct or has_col,
        "body has a physics collision representation",
        f"characterController={has_cct} collisionAPI={has_col}",
    )
    c.check(
        (has_cct and not (has_rb and not kinematic_rb)) or (has_rb and kinematic_rb),
        "body is kinematic, not dynamic",
        (
            "PhysxCharacterControllerAPI (a CCT is a kinematic actor that also "
            "collide-and-slides)" if has_cct
            else f"RigidBodyAPI kinematicEnabled={kinematic_rb}" if has_rb
            else "NEITHER a character controller nor a rigid body -- nothing will stop it"
        ),
    )
    if has_cct:
        cct = PhysxSchema.PhysxCharacterControllerAPI(body)
        c.check(
            cct.GetUpAxisAttr().Get() == up_axis,
            "character controller up axis matches stage",
            f"upAxis={cct.GetUpAxisAttr().Get()} up={up_axis}",
        )

    # --- semantics ---------------------------------------------------------
    tax = [s for s in body.GetAppliedSchemas() if s.startswith("SemanticsLabelsAPI:")]
    labels: list[str] = []
    for s in tax:
        attr = body.GetAttribute(f"semantics:labels:{s.split(':', 1)[-1]}")
        if attr and attr.Get():
            labels.extend(list(attr.Get()))
    want = str(cfg["semantic_class"])
    c.check(
        want in labels,
        f"semantic label reads {want!r}",
        f"applied={tax} labels={labels}",
    )

    # --- cameras -----------------------------------------------------------
    for cp in cam_paths:
        cam = stage.GetPrimAtPath(cp)
        if not c.check(cam.IsValid(), f"{Sdf.Path(cp).name} exists", cp):
            continue
        c.check(cam.IsA(UsdGeom.Camera), f"{Sdf.Path(cp).name} is a Camera", str(cam.GetTypeName()))
        c.check(
            Sdf.Path(cp).HasPrefix(Sdf.Path(avatar_path)) and cp != avatar_path,
            f"{Sdf.Path(cp).name} is a descendant of {avatar_path}",
            cp,
        )
        near = UsdGeom.Camera(cam).GetClippingRangeAttr().Get()
        c.check(
            near is not None and float(near[0]) < 0.1,
            f"{Sdf.Path(cp).name} near plane is close enough to see the room",
            f"clippingRange={tuple(near) if near is not None else None}",
        )

    # --- controls ----------------------------------------------------------
    # An OmniGraph node whose capsulePath is wrong loads without complaint and
    # then does nothing at all, which is indistinguishable from "physics is
    # broken" until you go looking.
    cct_nodes = [
        p
        for p in Usd.PrimRange(stage.GetPrimAtPath(avatar_path))
        if p.GetAttribute("node:type") and p.GetAttribute("node:type").Get() == "omni.physx.cct.OgnCharacterController"
    ]
    if c.check(len(cct_nodes) == 1, "exactly one character-controller node", f"found {len(cct_nodes)}"):
        node = cct_nodes[0]
        target = node.GetAttribute("inputs:capsulePath")
        target_v = str(target.Get()) if target else None
        c.check(target_v == body_path, "controller node points at the real capsule", f"capsulePath={target_v!r}")
        setup = node.GetAttribute("inputs:setupControls")
        setup_v = str(setup.Get()) if setup else None
        c.check(
            setup_v == "Auto",
            "controller binds its own WASD controls",
            f"setupControls={setup_v!r}",
        )

    # --- non-visual material ----------------------------------------------
    prefix = carb.settings.get_settings().get("/rtx/materialDb/nonVisualMaterialSemantics/prefix")
    result = UsdShade.MaterialBindingAPI(body).ComputeBoundMaterial()
    bound = result[0] if isinstance(result, tuple) else result
    base_attr = bound.GetPrim().GetAttribute(f"{prefix}:base") if bound else None
    c.check(
        base_attr is not None and base_attr.HasAuthoredValue(),
        "non-visual material base is written",
        f"{prefix}:base on {bound.GetPath() if bound else None} = "
        f"{base_attr.Get() if base_attr else None}",
    )

    return c


def main() -> int:
    stage_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_STAGE
    if not stage_path.exists():
        print(f"FAIL: stage not found: {stage_path}", flush=True)
        print("      Build it first: ./python.sh /workspace/sim/avatar.py", flush=True)
        return 2

    with open(SCENE_YAML, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)["avatar"]

    ctx = omni.usd.get_context()
    ctx.open_stage(str(stage_path))
    for _ in range(200):
        _APP.update()
        if not any(ctx.get_stage_loading_status()[1:]):
            break
    stage = ctx.get_stage()
    if stage is None:
        print(f"FAIL: could not open {stage_path}", flush=True)
        return 2

    print(f"stage      : {stage_path}", flush=True)
    print(f"prims      : {len(list(stage.Traverse()))}", flush=True)
    print(f"up axis    : {UsdGeom.GetStageUpAxis(stage)}   m/unit: {UsdGeom.GetStageMetersPerUnit(stage)}", flush=True)

    checks = verify(stage, cfg)
    checks.report()
    _advise_launch_flag()
    return 1 if checks.failed else 0


def _advise_launch_flag() -> None:
    """Explain the Kit warning this script provokes, and how not to ship it.

    Opening this stage without omni.physx.cct enabled logs

        [Warning] Could not find node type interface for
                  'omni.physx.cct.OgnCharacterController'

    and then carries on. The graph is present, the stage looks correct, Play
    does nothing, and nothing says why. This script does NOT enable the
    extension on purpose: doing so against the composed warehouse puts PhysX
    into a multi-minute cook of 3,469 exact triangle meshes, and verification
    does not need physics running. So the warning is expected here -- and
    fatal in the GUI, where the flag below is what prevents it.
    """
    import omni.kit.app

    mgr = omni.kit.app.get_app().get_extension_manager()
    on = mgr.is_extension_enabled("omni.physx.cct")
    print(f"\nnote: omni.physx.cct enabled in THIS process: {on} (expected False)", flush=True)
    print("      The GUI session must enable it or the controls silently do nothing:", flush=True)
    print("        make stream        # already passes --enable omni.physx.cct", flush=True)
    print("      Any hand-rolled launch needs that flag too.", flush=True)


if __name__ == "__main__":
    code = main()
    _APP.close()
    sys.exit(code)
