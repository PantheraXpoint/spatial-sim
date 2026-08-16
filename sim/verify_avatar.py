"""Assert, headless, that the avatar is actually what it claims to be.

Part one of the S6 gate. Part two is visual and only a human in the GUI can do
it: in third person you can see your own body, and walking into a shelf stops
you. **Do not treat a green run here as S6 passing.** The entire reason this
task is the highest-risk one in the project is that these two can disagree --
every failure mode in play produces no error message, so USD can look perfect
while the demo is broken, and vice versa.

What it checks, and why each one is here (each fails silently in the sim):

    structure   the Xform, the collision capsule, both cameras, the visible
                body, in the right places
    visible     the CHARACTER renders (purpose default, not invisible, and is a
                rigged mesh rather than a capsule) while the CAPSULE does not.
                THIS is what RTX lidar/radar/cameras need: they trace render
                geometry, not colliders. NVIDIA's own character-controller demo
                ships a purpose="guide" capsule, which looks fine in the stage
                tree and is invisible to every sensor in the project.
    follow      AT PLAY, not on paper: the stage is played, the capsule is
                moved, and the character's transform must move with it. The
                structural version of this check passed while the follow copied
                a constant for 120 frames -- OmniGraph reads Fabric, the
                character controller writes USD. Structure is not behaviour.
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

import os
import sys

from isaacsim import SimulationApp  # noqa: I001  -- must be first (hard rule 3)

_APP = SimulationApp({"headless": True})

import carb  # noqa: E402
import omni.timeline  # noqa: E402
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


def _labels_on(prim: Usd.Prim) -> list[str]:
    """Semantic labels directly on a prim, in the 6.x UsdSemantics schema.

    Direct only -- not inherited. Whether the annotators resolve inherited
    labels is exactly the thing this is meant to pin down, so assuming it here
    would defeat the check.
    """
    labels: list[str] = []
    for schema in prim.GetAppliedSchemas():
        if schema.startswith("SemanticsLabelsAPI:"):
            attr = prim.GetAttribute(f"semantics:labels:{schema.split(':', 1)[-1]}")
            if attr and attr.Get():
                labels.extend(list(attr.Get()))
    return labels


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
    # The capsule is collision only and must NOT render, or the demo shows a
    # plastic pill wrapped around the character. Checked as a positive
    # assertion rather than left to the eye.
    c.check(
        img.ComputeVisibility() == UsdGeom.Tokens.invisible,
        "collision capsule does not render",
        f"visibility={img.ComputeVisibility()}",
    )

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

    # --- the visible body --------------------------------------------------
    # Everything above is about the thing that COLLIDES. This is the thing
    # sensors actually see: RTX lidar, radar and cameras trace render geometry,
    # so an avatar whose only renderable prim is hidden is invisible to the
    # whole observatory while looking perfect in the stage tree.
    char_path = f"{avatar_path}/character"
    char = stage.GetPrimAtPath(char_path)
    if c.check(char.IsValid(), f"{char_path} exists", str(char.GetTypeName())):
        cimg = UsdGeom.Imageable(char)
        c.check(
            cimg.ComputeVisibility() != UsdGeom.Tokens.invisible,
            "visible body is visible",
            f"visibility={cimg.ComputeVisibility()}",
        )
        c.check(
            cimg.ComputePurpose() == UsdGeom.Tokens.default_,
            "visible body purpose is 'default', not 'guide'",
            f"purpose={cimg.ComputePurpose()}",
        )
        subtree = list(Usd.PrimRange(char))
        meshes = [x for x in subtree if x.IsA(UsdGeom.Mesh)]
        skels = [x for x in subtree if x.GetTypeName() == "Skeleton"]
        # "Is it a character rather than the capsule?" -- asserted on the two
        # things a rigged human has and a capsule cannot: skinned Meshes and a
        # Skeleton. A capsule is an analytic Gprim with neither.
        c.check(
            len(meshes) > 0 and len(skels) > 0,
            "visible body is a rigged character, not a capsule",
            f"{len(meshes)} Mesh prims, {len(skels)} Skeleton prims, "
            f"{len(subtree)} prims total",
        )
        c.check(
            not any(x.IsA(UsdGeom.Capsule) for x in subtree),
            "no capsule geometry inside the visible body",
            f"{char_path} subtree",
        )
        # It must stand roughly where the capsule does, or it is decoration
        # floating somewhere else in the warehouse.
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        rng = bbox.ComputeWorldBound(char).ComputeAlignedRange()
        h = rng.GetSize()[2]
        c.check(1.2 <= h <= 2.6, "visible body is human-sized", f"height {h:.3f} m")
        body_t = UsdGeom.XformCache().GetLocalToWorldTransform(body).ExtractTranslation()
        centre = rng.GetMidpoint()
        offset = ((centre[0] - body_t[0]) ** 2 + (centre[1] - body_t[1]) ** 2) ** 0.5
        c.check(
            offset < 0.75,
            "visible body is co-located with the collision capsule",
            f"horizontal offset {offset:.3f} m",
        )
        labels = _labels_on(char)
        c.check(
            str(cfg["semantic_class"]) in labels,
            f"visible body semantic label reads {str(cfg['semantic_class'])!r}",
            f"labels={labels}",
        )
        unlabelled = [m.GetPath().name for m in meshes if str(cfg["semantic_class"]) not in _labels_on(m)]
        c.check(
            not unlabelled,
            "every visible mesh carries the label",
            f"{len(meshes) - len(unlabelled)}/{len(meshes)} meshes labelled"
            + (f"; missing: {unlabelled[:4]}" if unlabelled else ""),
        )

    # --- semantics ---------------------------------------------------------
    want = str(cfg["semantic_class"])
    c.check(
        want in _labels_on(body),
        f"semantic label reads {want!r}",
        f"applied={[s for s in body.GetAppliedSchemas() if 'Semantic' in s]} "
        f"labels={_labels_on(body)}",
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

    # The follow used to be checked here, structurally: reader present, writer
    # present, connected, right paths. All four passed for a graph that copied
    # a constant. Behaviour is asserted at Play instead -- see verify_at_play.

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


def verify_at_play(stage: Usd.Stage, cfg: dict, c: Checks) -> None:
    """Press Play, move the capsule, and assert the visible body went with it.

    THE CHECK THAT WAS MISSING. Every graph check in this file is structural,
    and structure is exactly what passed while nothing worked: the follow was
    correctly wired, ticked 176 times, read 130 times, wrote 130 times, and
    copied a byte-identical constant for 120 frames while the capsule moved
    2.58 m. Three scripts were declared ready on structure alone. Structure is
    not behaviour, so this one runs the thing.

    Set VA_SKIP_PLAY=1 to skip -- only for a machine with no GPU to spare.
    """
    import sys as _sys

    _sys.path.insert(0, str(REPO / "sim"))
    import avatar as av  # noqa: E402
    from isaacsim.core.experimental.utils.app import enable_extension  # noqa: E402

    avatar_path = cfg["prim_path"]
    body = stage.GetPrimAtPath(f"{avatar_path}/body_mesh")
    char = stage.GetPrimAtPath(f"{avatar_path}/character")
    if not body.IsValid() or not char.IsValid():
        c.check(False, "play-time follow", "avatar or character missing")
        return

    sub = av.install_character_follow(stage, avatar_path)
    if not c.check(sub is not None, "character follow installs", "sim.avatar.install_character_follow"):
        return

    enable_extension("omni.physx.cct")
    from omni.physxcct.scripts import utils as cct_utils  # noqa: E402
    from omni.physxcct.scripts.ifaces import get_physx_cct_interface  # noqa: E402

    cct = cct_utils.CharacterController(f"{avatar_path}/body_mesh", None, True, 0.01)
    cct.activate(stage)
    iface = get_physx_cct_interface()

    cache = UsdGeom.XformCache()

    def world(prim):
        cache.Clear()
        t = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
        return (float(t[0]), float(t[1]), float(t[2]))

    omni.timeline.get_timeline_interface().play()
    for _ in range(20):
        _APP.update()

    b0, ch0 = world(body), world(char)
    home = b0
    # set_position, not set_move: set_move is consumed by a pre-physics stage
    # update node and does nothing when called from here -- measured.
    for i in range(1, 61):
        iface.set_position(f"{avatar_path}/body_mesh", (home[0] + i * 0.02, home[1], home[2]))
        _APP.update()
    b1, ch1 = world(body), world(char)

    moved_body = ((b1[0] - b0[0]) ** 2 + (b1[1] - b0[1]) ** 2) ** 0.5
    moved_char = ((ch1[0] - ch0[0]) ** 2 + (ch1[1] - ch0[1]) ** 2) ** 0.5
    gap = ((b1[0] - ch1[0]) ** 2 + (b1[1] - ch1[1]) ** 2) ** 0.5

    c.check(moved_body > 0.5, "the capsule actually moved at Play",
            f"{moved_body:.3f} m (if this fails the test proves nothing)")
    c.check(moved_char > 0.5, "THE VISIBLE BODY FOLLOWED IT",
            f"capsule {moved_body:.3f} m, character {moved_char:.3f} m")
    c.check(gap < 0.25, "character stayed co-located with the capsule",
            f"horizontal gap after moving: {gap:.3f} m")

    omni.timeline.get_timeline_interface().stop()
    for _ in range(5):
        _APP.update()


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
    if os.environ.get("VA_SKIP_PLAY") != "1":
        verify_at_play(stage, cfg, checks)
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
    # Same reason as sim/avatar.py: SimulationApp.close() can abort with
    # "Destroying busy TaskGroup!" during teardown, which would report a
    # PASSING verification as a non-zero exit. This script's exit code is the
    # gate, so it must mean what the checks said and nothing else.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
