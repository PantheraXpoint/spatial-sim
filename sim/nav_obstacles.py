"""Static collision for the things that are obstacles but were never colliders.

Layer 1 (scene/USD), server only. Companion to ``sim/pushable_props.py``: that
module makes a short list of props *move*; this one makes a short list of
prims *stop you*.

Why this exists, measured rather than assumed
---------------------------------------------
``sim/spikes/_diag_pushable.py PP_PHASE=obstacles`` asked PhysX directly, with
an overlap box over each prim's own footprint, at Play::

    /Root/Worker         0 own colliders in the physics scene
    /Root/Robots/BOT_01  5   base_link, base_scan, caster, two wheels
    /Root/Robots/BOT_02  25  calves, thighs, feet
    /Root/Robots/BOT_03  3   left_ankle_link, right_ankle_link, torso_link

So the four prims the avatar walks through fail in **two different ways**, and
only one of them is "no collision":

* **The Worker has none at all.** 520 prims, 11 render meshes, zero colliders.
  A skinned character asset ships geometry, not physics.
* **The robots have collision, and it is sparse and shaped like a robot.**
  BOT_03 is a humanoid with 25 rigid bodies and colliders on three of them, so
  an avatar walking into it passes through the legs, the arms and the head.
  BOT_01 has full coverage and is 0.19 m tall. BOT_02's colliders are its legs.
  Between the legs of a quadruped is a gap a person walks through.

**The first version of that audit reported 0 colliders for all three robots and
was wrong**, because ``Usd.PrimRange`` does not descend into instance
prototypes and a referenced robot is mostly instance proxies. It is corrected
here so nobody re-derives the wrong number: use
``Usd.PrimRange(prim, Usd.TraverseInstanceProxies())``.

And the sensing half, because the brief called it the bigger problem
--------------------------------------------------------------------
"If they have no colliders the RTX lidar is not seeing them either" is
**false on this stage, and measured false.** Ray-based sensors trace the RENDER
BVH, not colliders -- which is the whole reason ``sim/avatar.py`` can give the
avatar an invisible collision capsule and a visible body with no collider and
still have every sensor see it. Counting INFRA_01_LIDAR returns inside each
prim's world bounding box, one frame, 290,155 points in the cloud::

    /Root/Worker         1046 points     <- zero colliders, most visible of the four
    /Root/Robots/BOT_03   377 points
    /Root/Robots/BOT_02    66 points
    /Root/Robots/BOT_01     0 points

BOT_01's zero is geometry, not collision: it is 0.19 m tall and the INFRA_01
lidar sits at 2.60 m with a -15..+10 degree elevation band, so a Burger at that
range falls under the band. Nothing here is CLAUDE.md failure mode 1 -- that
one is about a prim with no RENDER geometry, and all four of these render.

So the walk-through is a navigation defect and not a sensing defect, and this
module fixes exactly that.

Static for the Worker, DYNAMIC for the robots
----------------------------------------------
The Worker is a stationary character and gets a static collider --
``CollisionAPI`` with no ``RigidBodyAPI``, which is what a fixture is.

**The three robots are dynamic bodies with their real hardware masses**, and
that is a deliberate amendment to CLAUDE.md's opening invariant rather than an
oversight; see that file and ``tasks/SERVER.md``. Nothing is tuned to be
immovable: the masses are what the machines weigh, and the impulse model in
``sim/pushable_props.py`` does the discriminating from there. Against a 51 kg
stall mass and 0.082 m/s of friction per frame::

    TurtleBot3 Burger   1 kg    dv 1.38 m/s per contact frame   skitters
    Unitree Go2        15 kg    dv 0.278                        slides
    Unitree H1         47 kg    dv 0.0887 against 0.0818 lost   barely shifts

The H1 is the heaviest real object on this floor and it is still **below** the
stall mass, so it creeps rather than refuses. That is the honest outcome and it
is why the prop list no longer carries a stall control.

The dynamic-proxy design, and why not the shipped articulation
--------------------------------------------------------------
The robot's own physics is left switched off. ``pin_robots_static`` already
makes every shipped body kinematic and disables the articulation -- which is
what stops a legged robot collapsing without a locomotion policy -- and this
module additionally clears ``rigidBodyEnabled`` and ``collisionEnabled`` on
them. **One convex proxy carries all the physics and the robot art is written
from it every frame.** That is the avatar's own design: the capsule is the
physics, the character is the picture, and a subscription copies one to the
other.

Doing it the other way -- re-enabling the articulation and letting the shipped
links be dynamic -- fails three ways: the legged robots collapse, the Burger's
wheel joints spin freely, and the per-link masses total about a kilogram, which
a 70 kg avatar would launch.

Two properties the proxy needs and a free body does not:

* **Rotation about X and Y is locked** (``physxRigidBody:lockedRotAxis``,
  bits 1|2). A 1.8 m capsule standing on a plane topples the moment anything
  touches it, and then rolls. These are machines that stay upright; yaw is left
  free so a shove can spin them.
* **Its mass is the whole robot's**, not the shell's, because it is the only
  body in the scene standing for that robot.

The collider is a single volume per target, sized from the prim's own measured
world bounding box (hard rule 1: read it, do not invent it).
"""

from __future__ import annotations

from pathlib import Path

import yaml
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

REPO = Path(__file__).resolve().parent.parent
SCENE_YAML = REPO / "config" / "scene.yaml"


def log(msg: str) -> None:
    print(f"[nav_obstacles] {msg}", flush=True)


def load_config(path: Path = SCENE_YAML) -> dict:
    with open(path, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh) or {}
    block = cfg.get("nav_obstacles") or {}
    block.setdefault("enabled", False)
    block.setdefault("targets", [])
    block.setdefault("scope", "/Root/NavObstacles")
    block.setdefault("footprint_scale", 0.85)
    block.setdefault("linear_damping", 0.15)
    block.setdefault("angular_damping", 0.60)
    block.setdefault("max_linear_velocity", 4.0)
    block.setdefault("sleep_threshold", 0.005)
    return block


def _world_bbox(stage: Usd.Stage, prim: Usd.Prim):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        return None
    return Gf.Vec3d(*rng.GetMin()), Gf.Vec3d(*rng.GetMax())


def has_live_colliders(prim: Usd.Prim) -> int:
    """Enabled colliders at or under ``prim``, INCLUDING instance proxies.

    The traversal flag is the whole point of this helper -- see the module
    docstring. Reported by :func:`add_nav_obstacles` so the log says whether a
    target was collisionless or merely sparse.
    """
    n = 0
    for q in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if q.HasAPI(UsdPhysics.CollisionAPI) and \
                UsdPhysics.CollisionAPI(q).GetCollisionEnabledAttr().Get() is not False:
            n += 1
    return n


def add_nav_obstacles(stage: Usd.Stage, cfg: dict | None = None) -> dict:
    """Give every declared target one static collider sized to its own bounds.

    Idempotent: re-running replaces the shape's size rather than stacking a
    second collider, so a GUI session that calls this after a robot has been
    re-referenced gets the right footprint and not two.
    """
    cfg = cfg or load_config()
    if not cfg.get("enabled"):
        log("nav_obstacles.enabled is false -- nothing given collision")
        return {"enabled": False, "made": {}, "missing": []}

    made: dict[str, dict] = {}
    missing: list[str] = []
    scale = float(cfg["footprint_scale"])
    UsdGeom.Scope.Define(stage, cfg["scope"])

    for spec in cfg["targets"]:
        path = spec["prim_path"]
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            missing.append(path)
            log(f"! declared target not on this stage: {path}")
            continue
        box = _world_bbox(stage, prim)
        if box is None:
            missing.append(path)
            log(f"! {path} has no bounding box -- nothing to size a collider from")
            continue
        lo, hi = box
        existing = has_live_colliders(prim)

        shape = str(spec.get("shape", "box")).lower()
        # WORLD SPACE, under a scope of its own -- NOT a child of the target.
        # Parenting it under the target looked tidier and silently produced a
        # collider 9 mm across: `/Root/Worker`'s own Xform carries a scale
        # (measured below), so a capsule authored with radius 0.30 in that
        # frame comes out at 0.30 * scale in the world. It existed, its
        # CollisionAPI was enabled, and PhysX reported it in an overlap -- every
        # check said yes and the thing was the size of a grape. The robots did
        # not show it because `reference_robots` gives them a translate and
        # nothing else.
        #
        # These prims are declared static ("Robots do not move", CLAUDE.md), so
        # nothing is lost by not following the target: if one is ever
        # repositioned, re-running this function re-reads the bbox and re-sizes.
        child = f"{cfg['scope']}/{path.strip('/').replace('/', '_')}"
        xform_scale = _scale_of(prim)
        c_world = Gf.Vec3d(*[(lo[i] + hi[i]) / 2.0 for i in range(3)])
        c_local = c_world
        size = [float(hi[i] - lo[i]) for i in range(3)]

        if shape == "capsule":
            # A person and a humanoid: a capsule cannot catch a corner, and its
            # round cross-section is what the avatar's own collision is.
            #
            # THE RADIUS IS NOT THE BOUNDING BOX, and that is measured. A
            # world bbox of a standing figure includes its arms: the Worker's
            # narrower horizontal span is 1.45 m and the H1's is 2.1 m, so a
            # bbox-derived radius gave a 0.62 m barrel for the man and, for the
            # robot, a capsule 1.33 m across and 0.01 m tall -- degenerate,
            # because twice the radius had swallowed the whole height. Declare
            # the radius (a person is about 0.30 m, which is what the avatar's
            # own capsule uses) and clamp whatever is derived to a quarter of
            # the measured height so it can never invert.
            derived = min(size[0], size[1]) * 0.5 * scale
            radius = float(spec.get("radius") or min(derived, size[2] * 0.25))
            radius = max(0.05, radius)
            height = max(0.10, size[2] - 2.0 * radius)
            geom = UsdGeom.Capsule.Define(stage, child)
            geom.CreateAxisAttr(UsdGeom.Tokens.z)
            geom.CreateRadiusAttr(radius)
            geom.CreateHeightAttr(height)
            geom.CreateExtentAttr([(-radius, -radius, -(height / 2 + radius)),
                                   (radius, radius, height / 2 + radius)])
            dims = {"radius": round(radius, 4), "height": round(height, 4),
                    "total_height": round(height + 2 * radius, 4),
                    "radius_source": "declared" if spec.get("radius") else "derived"}
        else:
            half = [max(0.01, size[i] * 0.5 * (scale if i < 2 else 1.0))
                    for i in range(3)]
            geom = UsdGeom.Cube.Define(stage, child)
            geom.CreateSizeAttr(2.0)
            geom.CreateExtentAttr([(-1, -1, -1), (1, 1, 1)])
            dims = {"half_extent": [round(v, 4) for v in half]}

        prim_child = geom.GetPrim()
        xf = UsdGeom.Xformable(prim_child)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(c_local))
        if shape != "capsule":
            xf.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in half]))

        # Invisible, purpose DEFAULT. Same reasoning as the avatar's capsule in
        # sim/avatar.py: physics ignores visibility, but `purpose = "guide"` is
        # inherited and would take the robot's render geometry down with it --
        # and these prims are exactly the ones the lidar can already see.
        UsdGeom.Imageable(prim_child).CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdGeom.Imageable(prim_child).CreatePurposeAttr(UsdGeom.Tokens.default_)

        # CollisionAPI and NO RigidBodyAPI == static collider. If a rigid body
        # ancestor ever appears above this prim it silently becomes part of that
        # body instead, so say so rather than leave it to be discovered.
        UsdPhysics.CollisionAPI.Apply(prim_child).CreateCollisionEnabledAttr().Set(True)
        ancestor = _rigid_body_ancestor(prim_child)
        if ancestor:
            log(f"! {child} sits under the rigid body {ancestor} -- it is part "
                f"of that body, not a static collider")

        dynamic = bool(spec.get("dynamic"))
        mass = float(spec.get("mass_kg") or 0.0)
        silenced = None
        if dynamic and mass > 0.0:
            _make_dynamic(prim_child, mass, cfg, spec)
            # The robot's own physics is switched OFF. The proxy is now the
            # only body standing for it, and leaving the shipped kinematic
            # bodies live would mean two sets of colliders in the same place
            # and PhysX writing the old world pose back over the one the follow
            # subscription just wrote.
            silenced = _silence_shipped_physics(prim)
        elif dynamic:
            log(f"! {path} declares dynamic but no mass_kg -- left static")
            dynamic = False

        made[path] = {
            "prim_path": path, "collider": child, "shape": shape, "dims": dims,
            "dynamic": dynamic, "mass_kg": mass if dynamic else None,
            "target_xform_scale": [round(v, 5) for v in xform_scale],
            "existing_live_colliders": existing,
            "silenced_shipped_physics": silenced,
            "world_bbox_size": [round(v, 3) for v in size],
            "rigid_body_ancestor": ancestor,
            "note": spec.get("note", ""),
        }
        log(f"{'DYNAMIC' if dynamic else 'static '} collider {child}  "
            f"{shape} {dims}"
            + (f"  mass {mass:g} kg" if dynamic else "")
            + f"  (target had {existing} enabled collider"
              f"{'s' if existing != 1 else ''}"
            + (f", silenced {silenced['bodies']} bodies / "
               f"{silenced['colliders']} colliders" if silenced else "")
            + f", xform scale {[round(v, 4) for v in xform_scale]})")

    n_dyn = sum(1 for r in made.values() if r["dynamic"])
    log(f"{len(made)} nav obstacles: {n_dyn} dynamic, {len(made) - n_dyn} static, "
        f"{len(missing)} missing")
    return {"enabled": True, "made": made, "missing": missing,
            "footprint_scale": scale}


def _make_dynamic(prim: Usd.Prim, mass: float, cfg: dict, spec: dict) -> None:
    """Turn the proxy into a rigid body that stays upright.

    ``lockedRotAxis`` bits 1|2 lock rotation about X and Y and leave Z free.
    Without it a standing capsule falls over the first time anything touches
    it and then rolls away, which is not what a robot does. Yaw is deliberately
    left free so a shove can spin the machine on the spot.
    """
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateRigidBodyEnabledAttr().Set(True)
    rb.CreateKinematicEnabledAttr().Set(False)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(float(mass))

    prb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    prb.CreateDisableGravityAttr().Set(False)
    prb.CreateLinearDampingAttr().Set(float(spec.get(
        "linear_damping", cfg["linear_damping"])))
    prb.CreateAngularDampingAttr().Set(float(spec.get(
        "angular_damping", cfg["angular_damping"])))
    prb.CreateMaxLinearVelocityAttr().Set(float(spec.get(
        "max_linear_velocity", cfg["max_linear_velocity"])))
    prb.CreateSleepThresholdAttr().Set(float(spec.get(
        "sleep_threshold", cfg["sleep_threshold"])))
    prb.CreateLockedRotAxisAttr().Set(int(spec.get("locked_rot_axis", 0b011)))
    # Same reason as the props: the push callback wakes a body before pushing
    # it, and a contact report costs nothing while nothing subscribes.
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)


def _silence_shipped_physics(prim: Usd.Prim) -> dict:
    """Switch off the target's own rigid bodies and colliders.

    Not a performance measure -- a correctness one. The proxy writes the
    robot's Xform every frame; a live kinematic body under that Xform has its
    own world pose and PhysX writes it straight back, so the two fight. It also
    removes a duplicate collider set sitting exactly where the proxy is, and a
    kinematic collider set would win every contact against the proxy it is
    supposed to be represented by.

    DE-INSTANCE FIRST. The robots' colliders live inside instance prototypes --
    BOT_02 is 213 instance proxies of 283 prims -- and USD refuses the write:
    ``Cannot create property spec at path <...>; authoring to an instance proxy
    is not allowed``. Clearing ``instanceable`` on the instance roots turns
    those proxies into ordinary prims that can be edited. It costs the memory
    the instancing was saving, which for three robots of a few hundred prims is
    nothing, and it is done only for targets declared dynamic.
    """
    # COLLECT, THEN MUTATE. `SetInstanceable` recomposes the stage, and calling
    # it from inside the PrimRange that is walking that stage invalidates the
    # iterator. Doing it inline produced robots whose bounding-box centres had
    # moved TWO KILOMETRES while their Xform origins had not moved at all --
    # scattered geometry, no error.
    instances = [q.GetPath() for q in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
                 if q.IsInstance()]
    de_instanced = 0
    for path in instances:
        q = prim.GetStage().GetPrimAtPath(path)
        if not q.IsValid():
            continue
        try:
            q.SetInstanceable(False)
            de_instanced += 1
        except Exception as exc:                                  # noqa: BLE001
            log(f"! could not de-instance {path}: {exc!r}")

    bodies = colliders = skipped = 0
    for q in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if q.IsInstanceProxy():
            # Still inside a prototype: report it rather than let a live
            # collider sit invisibly under a robot that is supposed to be inert.
            if q.HasAPI(UsdPhysics.CollisionAPI) or q.HasAPI(UsdPhysics.RigidBodyAPI):
                skipped += 1
            continue
        if q.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(q).CreateRigidBodyEnabledAttr().Set(False)
            bodies += 1
        if q.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr().Set(False)
            colliders += 1
    if skipped:
        log(f"! {skipped} physics prims under {prim.GetPath()} are still "
            f"instance proxies and were NOT silenced")
    return {"bodies": bodies, "colliders": colliders,
            "de_instanced": de_instanced, "still_proxied": skipped}


def _scale_of(prim: Usd.Prim) -> list:
    """The target Xform's own world scale, per axis. Reported, not applied.

    It is here so the log carries the reason a collider would have been the
    wrong size if it were parented under the target -- see add_nav_obstacles.
    """
    m = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    return [float(Gf.Vec3d(m[i][0], m[i][1], m[i][2]).GetLength())
            for i in range(3)]


def _rigid_body_ancestor(prim: Usd.Prim) -> str | None:
    p = prim.GetParent()
    while p and p.IsValid() and p.GetPath().pathString != "/":
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            return p.GetPath().pathString
        p = p.GetParent()
    return None


# ---------------------------------------------------------------------------
# The proxy -> robot follow
# ---------------------------------------------------------------------------
class ProxyFollow:
    """Write each dynamic target's transform from its physics proxy, per frame.

    KEEP A REFERENCE. Dropping it unsubscribes and the robots stop following
    their own physics -- the proxies go on being shoved around and the visible
    robots stand still, which is the same failure shape as
    ``avatar.install_character_follow`` and just as quiet.

    The maths, in USD's row-vector convention (``v' = v * M``, and
    local-to-world composes as ``local * parent``). At install time record the
    proxy's world matrix ``P0`` and the target's ``R0``. Each frame the proxy
    has moved to ``P``, so the world-space rigid motion it underwent is
    ``D = P0^-1 * P`` and the target belongs at ``R0 * D``. That is written
    back through the target's parent, ``local = R0 * D * parent^-1``.

    Deltas rather than absolute poses because the proxy is a bounding-box
    volume: its origin is the target's bbox centre, not the target's own
    origin, and for a robot whose asset origin is at the pelvis those are a
    metre apart. Copying the proxy's pose straight onto the robot would teleport
    it on the first frame.
    """

    def __init__(self, stage: Usd.Stage, converted: dict) -> None:
        import omni.kit.app

        self.stage = stage
        self.pairs: list[tuple[str, str]] = []
        self.base: dict[str, tuple] = {}
        cache = UsdGeom.XformCache()
        for path, rec in (converted.get("made") or {}).items():
            if not rec.get("dynamic"):
                continue
            target = stage.GetPrimAtPath(path)
            proxy = stage.GetPrimAtPath(rec["collider"])
            if not (target.IsValid() and proxy.IsValid()):
                log(f"! cannot follow {path}: target or proxy missing")
                continue
            r0 = cache.GetLocalToWorldTransform(target)
            p0 = cache.GetLocalToWorldTransform(proxy)
            parent = cache.GetLocalToWorldTransform(target.GetParent())
            # One matrix op replaces translate/orient/scale. Done here, once,
            # AFTER pin_robots_static has finished adjusting the translate --
            # it is the op that pinning writes, and clearing it earlier would
            # throw away the drop-to-floor correction.
            op = UsdGeom.Xformable(target).MakeMatrixXform()
            self.pairs.append((path, rec["collider"]))
            # The op is created ONCE and its attribute cached. Calling
            # MakeMatrixXform every frame rewrites the prim's xformOpOrder
            # sixty times a second, which is metadata churn on a prim the
            # renderer is reading.
            self.base[path] = (r0, p0.GetInverse(), parent.GetInverse(), op)
        self.sub = None
        if self.pairs:
            self.sub = omni.kit.app.get_app().get_update_event_stream(
            ).create_subscription_to_pop(self._follow, name="nav_proxy_follow")
        log(f"proxy follow installed for {len(self.pairs)} dynamic target"
            f"{'s' if len(self.pairs) != 1 else ''}"
            + (": " + ", ".join(p for p, _ in self.pairs) if self.pairs else ""))

    def _follow(self, _e) -> None:
        cache = UsdGeom.XformCache()
        for path, proxy_path in self.pairs:
            target = self.stage.GetPrimAtPath(path)
            proxy = self.stage.GetPrimAtPath(proxy_path)
            if not (target.IsValid() and proxy.IsValid()):
                continue
            r0, p0_inv, parent_inv, op = self.base[path]
            world = r0 * (p0_inv * cache.GetLocalToWorldTransform(proxy))
            op.Set(world * parent_inv)

    def close(self) -> None:
        self.sub = None


def install_proxy_follow(stage: Usd.Stage, converted: dict) -> ProxyFollow | None:
    """Wire the follow for whatever :func:`add_nav_obstacles` made dynamic."""
    if not (converted.get("made") or {}):
        return None
    try:
        follow = ProxyFollow(stage, converted)
    except Exception as exc:                                      # noqa: BLE001
        log(f"! proxy follow failed to install: {exc!r}")
        return None
    return follow if follow.pairs else None


def dynamic_bodies(converted: dict) -> dict:
    """``{proxy prim path: mass}`` for every dynamic target.

    This is what ``pushable_props.install_push_callback`` needs in order to
    push the robots with the same impulse model as the props -- one model, one
    set of numbers, rather than a second push path that could disagree with the
    first.
    """
    return {rec["collider"]: float(rec["mass_kg"])
            for rec in (converted.get("made") or {}).values()
            if rec.get("dynamic") and rec.get("mass_kg")}
