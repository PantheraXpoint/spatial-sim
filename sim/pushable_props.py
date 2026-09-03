"""Floor-level props the avatar can shove. Layer 1 (scene/USD), server only.

Two halves, and they are independent -- the first is USD authoring, the second
is a runtime callback:

  :func:`make_pushable`          static collider  ->  dynamic rigid body
  :func:`install_push_callback`  the momentum transfer the CCT will not do

Which props, and why it is a config list and not a rule
------------------------------------------------------
Hard rule 1: never invent a prim path. The prop list lives in
``config/scene.yaml`` under ``pushable_props.props`` and every path in it was
read off the stage by ``sim/spikes/_diag_pushable.py`` (``PP_PHASE=enumerate``)
and then confirmed by a human. A predicate over the stage -- "everything short
and near the floor" -- was tried on paper and rejected: the same filter that
finds the traffic cones also finds five ``SM_FloorDecal_RecRed1X*`` prims,
which are **zero-thickness paint on the concrete**, and five
``SM_Rackshield_*``, which are the steel guards bolted to the rack feet. Both
would become rigid bodies that fall over. The filter is a way of *proposing*
candidates; the list is what ships.

Why the CCT does not already do this
------------------------------------
A PhysX character controller resolves its own motion by sweeping and sliding;
its actor is kinematic and its target pose is set after the sweep, so a dynamic
body in the way stops the sweep *before* the solver ever sees a deep contact to
resolve. PhysX ships ``PxUserControllerHitReport::onShapeHit`` and the
``defaultCCTInteraction`` helper for exactly this, and Omniverse does not
surface either: ``omni.physx.cct``'s event stream carries
``COLLISION_UP/DOWN/SIDES`` with a **bool** and the CCT's own path, and names
neither the shape that was hit nor a normal (measured -- see
``omni/physxcct/scripts/tests/collisionEvents.py`` in the shipped extension).
So the hit report is rebuilt here, from a source that does name the shape.

The impulse, and why it is not a tuning constant
------------------------------------------------
Every frame the avatar is in contact with a pushable body::

    mu = M * m / (M + m)                    reduced mass
    v_n = dot(v_avatar - v_body, n)         approach speed along the contact normal
    J   = min(mu * v_n,  F_max * dt) * n

Two terms, both named:

* ``mu * v_n`` is the impulse of a **perfectly inelastic** collision between
  the avatar, treated as a body of mass ``M``, and the prop at its current
  velocity. It is what makes the transfer depend on the struck body's mass, and
  it is self-limiting: once the prop is moving at the avatar's speed ``v_n`` is
  zero and the push stops. Without the ``v_body`` term the same impulse lands
  every frame and the box accelerates away like a hockey puck.
* ``F_max * dt`` is a person's **force budget**. It is the term that makes a
  heavy body refuse to move, and without it nothing does: on its own the
  inelastic term gives ``dv = v * M / (M + m)``, which for M=70 kg and a 60 kg
  drum is still 0.54 of walking speed -- and Coulomb friction decelerates at
  ``mu_f * g`` regardless of mass, so the light box and the heavy drum would
  slide almost the same distance. The cap is what separates them.

  Two thresholds fall out of it, and they are 8 kg apart:

      break away    F_max < m * g * mu_static   ->  m > 250/(9.81*0.6) = 42.5 kg
      keep sliding  F_max < m * g * mu_dynamic  ->  m > 250/(9.81*0.5) = 51.0 kg

  :func:`stall_mass_kg` reports the second, because a prop that breaks away and
  then stops after a centimetre reads as "it did not move". That is the design:
  pick masses either side of it. The shipped list runs **1.0-8.0 kg** on one
  side and a single **65 kg large carton** on the other.

  Masses are real-world reference masses for the object each prop depicts, not
  values tuned to produce a result -- see the table in config/scene.yaml. The
  60 kg plastic drum this control replaces was the clearest failure of that
  rule: an empty drum is 7-10 kg, so the demo's one immovable object was the
  object least entitled to be immovable. A 1.02 x 1.00 x 0.50 m packed carton
  at 127 kg/m3 both weighs what it is and looks like it weighs it.

``M`` and ``F_max`` are declared in ``config/scene.yaml`` beside the props, so
the threshold is inspectable rather than emergent.

The velocity is measured, not asked for
---------------------------------------
There is no "controller velocity" to read: the CCT takes a per-frame
*displacement* through ``set_move()`` and exposes no velocity. So both
velocities are finite-differenced from world transforms between physics steps.
That also means the push works no matter who is driving the capsule -- the
keyboard graph, :func:`sim.avatar.set_avatar_pose`, or a scripted policy.

What this changes about the scene, stated plainly
-------------------------------------------------
**A dynamic body cannot keep an exact triangle-mesh collider.** PhysX has no
dynamic trimesh; NVIDIA's own ``omni.physx.scripts.utils.setCollider`` silently
rewrites ``none`` to ``convexHull`` when the prim is part of a rigid body. Every
prop on this list is currently ``approximation = "none"``, so converting it
*necessarily* convexifies it. ``sim/spikes/FINDINGS.md`` refuses convex-hull
conversion for the static warehouse on the grounds that "anything that can move
during an episode must stay exact" -- that refusal was about buying frame rate,
and this is not that trade, but the collision geometry does change and the
change is recorded per prop in the return value of :func:`make_pushable`.
Concave props (the A-frame wet-floor signs, the open crates) take
``convexDecomposition`` instead so the concavity survives; the cartons, cones
and drum are convex already and their hull is exact.

What it costs
-------------
Measured 2026-09-01, five arms in one process, collider mask on, **no arm
reading an annotator** (CLAUDE.md failure mode 11), 240 frames each::

    props static, avatar standing        16.68 ms/frame
    props static, avatar walking         16.72 ms
    10 dynamic bodies, avatar standing   16.69 ms      +0.01  -- free
    10 dynamic bodies, walked through    27.77 ms     +11.05
    ... plus this hit callback           33.21 ms      +5.44

The first three sit on the 60 fps app-loop cap, so those deltas are **lower
bounds**. Two things carry over into how this module is written:

* **Sleeping is what makes idle props free.** Ten dynamic bodies standing
  untouched cost nothing measurable, because a sleeping body is skipped by the
  solver. That is why ``sleep_threshold`` is authored rather than zeroed, and
  why :meth:`PushCallback._apply` wakes a body explicitly instead.
* **The callback's cost is the sweeps**, and it only pays them while the avatar
  is moving (``min_push_speed_ms``). ``n_probes`` is the knob; 5.44 ms is at
  five probes per physics step.

Full derivation, including the A/B that shows the 4 kg carton going 0.000 m to
0.560 m and the 60 kg drum staying at 0.0001 m under seven impulses, is in
``sim/spikes/FINDINGS.md``.

Execution model: this module only touches USD and physics. It reads no sensor
data, so it runs in exec mode or under SimulationApp equally.
"""

from __future__ import annotations

from pathlib import Path

import yaml
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

REPO = Path(__file__).resolve().parent.parent
SCENE_YAML = REPO / "config" / "scene.yaml"

GRAVITY = 9.81


def log(msg: str) -> None:
    print(f"[pushable_props] {msg}", flush=True)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
def load_pushable_config(path: Path = SCENE_YAML) -> dict:
    """The ``pushable_props`` block of config/scene.yaml. The config is the contract.

    Returns a block with ``enabled: False`` and no props if the key is absent,
    so a stage built from an older config simply gets no dynamic bodies rather
    than a KeyError.
    """
    with open(path, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh) or {}
    block = cfg.get("pushable_props") or {}
    block.setdefault("enabled", False)
    block.setdefault("props", [])
    block.setdefault("avatar_mass_kg", 70.0)
    block.setdefault("max_push_force_n", 250.0)
    block.setdefault("min_push_speed_ms", 0.05)
    block.setdefault("detector", "sweep")
    block.setdefault("n_probes", 5)
    block.setdefault("material", {})
    block["material"].setdefault("prim_path", "/Root/PhysicsMaterials/pushable_prop")
    block["material"].setdefault("static_friction", 0.6)
    block["material"].setdefault("dynamic_friction", 0.5)
    block["material"].setdefault("restitution", 0.0)
    return block


def push_impulse(avatar_mass_kg: float, body_mass_kg: float, v_n: float,
                 max_push_force_n: float, dt: float) -> tuple[float, bool]:
    """The scalar impulse along the contact normal, and whether it was capped.

    Pure arithmetic, no simulator, so it can be read and checked without one::

        J = min( (M*m/(M+m)) * v_n ,  F_max * dt )

    ``v_n`` is the APPROACH speed along the normal -- the avatar's velocity
    minus the body's, projected. Zero or negative means separating and returns
    zero, which is what stops a contact that persists for forty frames from
    accelerating the box forty times.
    """
    if v_n <= 0.0 or body_mass_kg <= 0.0 or dt <= 0.0:
        return 0.0, False
    mu = avatar_mass_kg * body_mass_kg / (avatar_mass_kg + body_mass_kg)
    j_free = mu * v_n
    j_cap = max_push_force_n * dt
    return (j_cap, True) if j_free > j_cap else (j_free, False)


def predict_delta_v(cfg: dict, mass_kg: float, walk_speed_ms: float = 1.4,
                    dt: float = 1.0 / 60.0) -> float:
    """Speed one frame of contact would give this body, at walking pace.

    Printed by :func:`make_pushable` for every prop it converts, so the run log
    carries the prediction beside the mass and a leg that does not move can be
    checked against what the model said it would do -- rather than against a
    memory of what "should" happen.
    """
    j, _ = push_impulse(float(cfg["avatar_mass_kg"]), mass_kg, walk_speed_ms,
                        float(cfg["max_push_force_n"]), dt)
    return j / mass_kg if mass_kg > 0 else 0.0


def friction_delta_v(cfg: dict, dt: float = 1.0 / 60.0) -> float:
    """Speed Coulomb friction takes back in one frame. MASS-INDEPENDENT.

    ``a = mu_d * g``, so the deceleration of a 1 kg carton and a 60 kg drum are
    identical. That is exactly why the force cap is needed and why the
    momentum term alone cannot separate them.
    """
    return float(cfg["material"]["dynamic_friction"]) * GRAVITY * dt


def stall_mass_kg(cfg: dict) -> float:
    """The mass above which a prop stops responding at all, from the config.

    Reported rather than assumed: it is the single number that says whether the
    declared masses actually straddle the light/heavy boundary.
    """
    mu_f = float(cfg["material"]["dynamic_friction"])
    if mu_f <= 0.0:
        return float("inf")
    return float(cfg["max_push_force_n"]) / (GRAVITY * mu_f)


# ---------------------------------------------------------------------------
# Authoring: static collider -> dynamic rigid body
# ---------------------------------------------------------------------------
def collider_leaves(prim: Usd.Prim) -> list[Usd.Prim]:
    """Every prim at or under ``prim`` carrying UsdPhysics.CollisionAPI."""
    return [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)]


def _physics_material(stage: Usd.Stage, spec: dict) -> UsdShade.Material:
    """One declared physics material for every pushable prop.

    Authored rather than inherited. Whatever PhysX falls back to when no
    material is bound is a global default that nothing in this repo declares,
    and the whole point of the push model is that its friction term is a number
    a reader can check against the mass table.
    """
    path = spec["prim_path"]
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr().Set(float(spec["static_friction"]))
    api.CreateDynamicFrictionAttr().Set(float(spec["dynamic_friction"]))
    api.CreateRestitutionAttr().Set(float(spec["restitution"]))
    return mat


def make_pushable(stage: Usd.Stage, cfg: dict | None = None) -> dict:
    """Convert every declared prop from static collider to dynamic rigid body.

    Reports rather than raises on a path that is not on the stage: a stale
    config entry should show up as one named line in the log, not as a dead
    session (hard rule 1 -- a wrong prim path is the thing that costs days).

    Returns a record per prop, including what its collision approximation was
    BEFORE conversion, because that is the property this function changes and
    it is otherwise invisible.
    """
    cfg = cfg or load_pushable_config()
    if not cfg.get("enabled"):
        log("pushable_props.enabled is false -- nothing converted")
        return {"enabled": False, "made": {}, "missing": [], "stall_mass_kg": None}

    mat = _physics_material(stage, cfg["material"])
    fdv = friction_delta_v(cfg)
    made: dict[str, dict] = {}
    missing: list[str] = []

    for spec in cfg["props"]:
        path = spec["prim_path"]
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            missing.append(path)
            log(f"! declared prop not on this stage: {path}")
            continue

        leaves = collider_leaves(prim)
        if not leaves:
            missing.append(path)
            log(f"! {path} has no collider under it -- not converted")
            continue

        mass = float(spec["mass_kg"])
        approx = spec.get("approximation", "convexHull")
        dv = predict_delta_v(cfg, mass)

        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        rb.CreateRigidBodyEnabledAttr().Set(True)
        # Failure mode 7 in CLAUDE.md is about the AVATAR: a dynamic avatar
        # ragdolls. These are props -- dynamic is the whole request.
        rb.CreateKinematicEnabledAttr().Set(False)

        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr().Set(mass)

        prb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        prb.CreateDisableGravityAttr().Set(False)
        prb.CreateLinearDampingAttr().Set(float(spec.get("linear_damping", 0.05)))
        prb.CreateAngularDampingAttr().Set(float(spec.get("angular_damping", 0.20)))
        # A shoved box must never become a projectile: the solver can produce a
        # large velocity out of a deep penetration on the frame a collider is
        # first cooked, and an unbounded one leaves the warehouse.
        prb.CreateMaxLinearVelocityAttr().Set(float(spec.get("max_linear_velocity", 5.0)))
        prb.CreateMaxAngularVelocityAttr().Set(float(spec.get("max_angular_velocity", 20.0)))
        # Sleeping is what keeps the frame-rate cost near zero when nobody is
        # touching anything: a sleeping body is skipped by the solver entirely.
        # It is also why the push callback has to WAKE a body before pushing
        # it -- see PushCallback._apply. Do not "fix" a prop that will not move
        # by setting this to 0; that pays the solver cost on every frame of
        # every run to work around a missing one-line wake_up.
        prb.CreateSleepThresholdAttr().Set(float(spec.get("sleep_threshold", 0.005)))
        prb.CreateStabilizationThresholdAttr().Set(float(spec.get("stabilization_threshold", 0.001)))

        # The hit callback's contact-report detector needs this; the sweep
        # detector does not. Applied either way -- it is free when unsubscribed,
        # and it is what makes the two detectors comparable in one run.
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)

        was: list[str] = []
        for leaf in leaves:
            UsdPhysics.CollisionAPI(leaf).CreateCollisionEnabledAttr().Set(True)
            if leaf.IsA(UsdGeom.Mesh) or leaf.IsInstanceable():
                mca = UsdPhysics.MeshCollisionAPI.Apply(leaf)
                prev = mca.GetApproximationAttr().Get()
                was.append(str(prev) if prev is not None else "<unset>")
                mca.CreateApproximationAttr().Set(approx)
            UsdShade.MaterialBindingAPI.Apply(leaf).Bind(
                mat, UsdShade.Tokens.weakerThanDescendants, "physics"
            )

        made[path] = {
            "prim_path": path,
            "mass_kg": mass,
            "approximation": approx,
            "approximation_was": was,
            "n_colliders": len(leaves),
            "note": spec.get("note", ""),
            # Reported so a run can be read without the config beside it.
            "moves_under_push": mass < stall_mass_kg(cfg),
            "predicted_dv_ms": round(dv, 4),
            "friction_dv_ms": round(fdv, 4),
        }
        log(f"dynamic  {path}  {mass:g} kg  {'/'.join(was) or '-'} -> {approx}"
            f"  ({len(leaves)} collider{'s' if len(leaves) != 1 else ''})"
            f"  predicted dv {dv:.3f} m/s per contact frame vs {fdv:.3f} lost to friction"
            f" -> {'MOVES' if dv > fdv else 'STAYS'}")

    log(f"{len(made)} props dynamic, {len(missing)} declared but not converted; "
        f"stall mass {stall_mass_kg(cfg):.1f} kg "
        f"(F_max {cfg['max_push_force_n']:g} N / g / mu_d {cfg['material']['dynamic_friction']:g})")
    return {
        "enabled": True,
        "made": made,
        "missing": missing,
        "stall_mass_kg": round(stall_mass_kg(cfg), 2),
        "material": cfg["material"],
    }


# ---------------------------------------------------------------------------
# Runtime: the hit callback
# ---------------------------------------------------------------------------
class PushCallback:
    """Momentum transfer from the character controller into dynamic props.

    Hold the returned instance. Dropping it drops the subscriptions and the
    avatar silently goes back to walking through boxes -- the same shape of
    failure as ``install_character_follow``.
    """

    def __init__(
        self,
        stage: Usd.Stage,
        body_path: str,
        bodies: dict[str, float],
        *,
        avatar_mass_kg: float = 70.0,
        max_push_force_n: float = 250.0,
        min_push_speed_ms: float = 0.05,
        detector: str = "sweep",
        n_probes: int = 5,
        skin_m: float = 0.06,
    ) -> None:
        from omni.physx import (
            get_physx_interface,
            get_physx_scene_query_interface,
            get_physx_simulation_interface,
        )
        from pxr import UsdUtils

        self.stage = stage
        self.body_path = body_path
        self.bodies = dict(bodies)          # prim path -> mass kg
        self.M = float(avatar_mass_kg)
        self.f_max = float(max_push_force_n)
        self.v_min = float(min_push_speed_ms)
        self.detector = detector
        self.n_probes = max(1, int(n_probes))
        self.skin = float(skin_m)

        self._sim = get_physx_simulation_interface()
        self._query = get_physx_scene_query_interface()
        self._stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self._cache = UsdGeom.XformCache()

        self._prev_avatar: Gf.Vec3d | None = None
        self._prev_body: dict[str, Gf.Vec3d] = {}
        self._v_avatar = Gf.Vec3d(0, 0, 0)
        self._v_body: dict[str, Gf.Vec3d] = {}

        self.radius, self.half_height = self._capsule_dims()

        # Counters, so a run that pushed nothing can say WHICH stage was silent:
        # no steps, no contacts found, or contacts found and no impulse applied.
        self.stats = {
            "steps": 0, "moving_steps": 0, "hits": 0, "impulses": 0,
            "contact_events": 0, "contact_pairs_with_cct": 0,
            "impulse_ns_total": 0.0, "capped": 0, "woken": 0, "errors": 0,
        }
        #: Hits per prop, so a caller can tell "this prop was never touched"
        #: from "this prop was touched and did not move". Those are different
        #: results and the aggregate counter cannot separate them.
        self.hits_by_root: dict[str, int] = {}
        self.log_lines: list[str] = []

        self._step_sub = get_physx_interface().subscribe_physics_step_events(self._on_step)
        self._contact_sub = None
        if detector in ("contact", "both"):
            self._contact_sub = self._sim.subscribe_contact_report_events(self._on_contact)

    # -- geometry ----------------------------------------------------------
    def _capsule_dims(self) -> tuple[float, float]:
        """Read the capsule off the stage rather than off config/scene.yaml.

        The capsule is what physics actually sweeps; scene.yaml's height/radius
        are what avatar.py was asked to build. They agree today. Reading the
        prim is what keeps them agreeing.
        """
        prim = self.stage.GetPrimAtPath(self.body_path)
        cap = UsdGeom.Capsule(prim)
        if not cap:
            return 0.30, 0.575
        r = float(cap.GetRadiusAttr().Get() or 0.30)
        h = float(cap.GetHeightAttr().Get() or 1.15)
        return r, h / 2.0

    def _world_pos(self, path: str) -> Gf.Vec3d | None:
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return None
        return self._cache.GetLocalToWorldTransform(prim).ExtractTranslation()

    # -- per-step ----------------------------------------------------------
    def _on_step(self, dt: float) -> None:
        """Wrapped whole, because this runs inside a PhysX step callback.

        An exception raised in a carb subscription is caught and logged by Kit,
        not propagated: the caller never sees it, the callback keeps being
        invoked, and the only symptom is one traceback per physics step
        forever. Measured 2026-09-01 on this same wiring, elsewhere. So the
        first failure is recorded and the callback disarms itself rather than
        filling the log at 60 Hz.
        """
        try:
            self._step(dt)
        except Exception as exc:                                  # noqa: BLE001
            import traceback

            self.stats["errors"] += 1
            self._note(f"push callback step failed, DISARMING: {exc!r}")
            self._note(traceback.format_exc().replace("\n", " | "))
            self._step_sub = None

    def _step(self, dt: float) -> None:
        self.stats["steps"] += 1
        self._cache.Clear()

        pos = self._world_pos(self.body_path)
        if pos is None or dt <= 0.0:
            return
        if self._prev_avatar is not None:
            self._v_avatar = (pos - self._prev_avatar) / dt
        self._prev_avatar = pos

        for path in self.bodies:
            bp = self._world_pos(path)
            if bp is None:
                continue
            prev = self._prev_body.get(path)
            if prev is not None:
                self._v_body[path] = (bp - prev) / dt
            self._prev_body[path] = bp

        v = Gf.Vec3d(self._v_avatar[0], self._v_avatar[1], 0.0)
        speed = v.GetLength()
        if speed < self.v_min:
            return
        self.stats["moving_steps"] += 1

        if self.detector in ("sweep", "both"):
            self._sweep_and_push(pos, v, speed, dt)

    def _sweep_and_push(self, pos: Gf.Vec3d, v: Gf.Vec3d, speed: float, dt: float) -> None:
        """The rebuilt onShapeHit: sweep the capsule forward, push what it finds.

        Spheres along the capsule axis rather than one sphere at its centre. A
        0.15 m carton on the floor sits entirely below a sphere centred at the
        capsule's middle, so a single-probe sweep walks over the small cartons
        and reports nothing -- with no error, because "no hit" is a perfectly
        ordinary sweep result.

        Five probes rather than three. A sphere at the capsule's mid-height
        bulges 0.3 m past the cylinder's ends, so a sparse ladder both misses
        low props and reaches for things the capsule could not touch; five
        tightens both. It does not remove the real limit at the bottom, which
        the capsule shares: at a 0.14 m crate's height the bottom hemisphere's
        horizontal reach is 0.16 m, not the full 0.30 m radius, so a very low
        prop gets a glancing push and no more. That is the collision shape
        being honest, not the sweep being wrong.
        """
        direction = v / speed
        distance = max(speed * dt, 0.0) + self.skin
        hits: dict[str, tuple] = {}

        def report(hit) -> bool:
            body = hit.rigid_body or hit.collision
            root = self._pushable_root(body)
            if root is not None and root not in hits:
                hits[root] = (
                    Gf.Vec3d(*hit.position),
                    Gf.Vec3d(*hit.normal),
                )
            return True  # keep going: one sweep can touch two props

        for i in range(self.n_probes):
            frac = 0.0 if self.n_probes == 1 else (i / (self.n_probes - 1)) * 2.0 - 1.0
            origin = Gf.Vec3d(pos[0], pos[1], pos[2] + frac * self.half_height)
            try:
                self._query.sweep_sphere_all(
                    self.radius,
                    carb_float3(origin),
                    carb_float3(direction),
                    distance,
                    report,
                    False,
                )
            except Exception as exc:  # a query before the scene is attached
                self._note(f"sweep failed: {exc!r}")
                return

        for root, (point, normal) in hits.items():
            self.stats["hits"] += 1
            self.hits_by_root[root] = self.hits_by_root.get(root, 0) + 1
            self._apply(root, point, normal, direction, dt)

    def _pushable_root(self, path: str) -> str | None:
        """Map a hit collider back to the prop that owns it, or None."""
        if not path:
            return None
        if path in self.bodies:
            return path
        for known in self.bodies:
            if path.startswith(known + "/"):
                return known
        return None

    # -- the impulse -------------------------------------------------------
    def _apply(self, root: str, point: Gf.Vec3d, normal: Gf.Vec3d, direction: Gf.Vec3d, dt: float) -> None:
        from pxr import PhysicsSchemaTools

        m = self.bodies.get(root)
        if not m:
            return

        # The sweep's normal points OUT of the surface that was hit, i.e. back
        # at the avatar. The push goes the other way. Fall back to the travel
        # direction when the normal is degenerate or points along the vertical
        # (a hit on the top face of a low carton, where "push" is meaningless).
        n = Gf.Vec3d(-normal[0], -normal[1], 0.0)
        if n.GetLength() < 1e-6:
            n = Gf.Vec3d(direction[0], direction[1], 0.0)
        n = n.GetNormalized()

        vb = self._v_body.get(root, Gf.Vec3d(0, 0, 0))
        v_rel = Gf.Vec3d(self._v_avatar[0] - vb[0], self._v_avatar[1] - vb[1], 0.0)
        v_n = v_rel[0] * n[0] + v_rel[1] * n[1]
        if v_n <= 0.0:
            return  # separating, or the prop is already outrunning us

        j, capped = push_impulse(self.M, m, v_n, self.f_max, dt)
        if j <= 0.0:
            return
        if capped:
            self.stats["capped"] += 1

        # An SdfPath, not the string: the binding is typed on SdfPath and a
        # str reaches it as the wrong overload.
        pid = PhysicsSchemaTools.sdfPathToInt(Sdf.Path(root))
        try:
            # WAKE IT FIRST. An impulse applied to a SLEEPING body is discarded
            # in silence -- `apply_force_at_pos` returns normally, the stats
            # count an impulse, and the box does not move. A prop that has been
            # standing still since Play is asleep by definition, which is
            # exactly the state every prop is in the moment you walk into it.
            # Measured 2026-09-01: 4.33 N.s delivered to a 1.2 kg crate (enough
            # for 3.6 m/s) displaced it 0.014 m and turned it 0.17 degrees.
            if self._sim.is_sleeping(self._stage_id, pid):
                self.stats["woken"] += 1
                self._sim.wake_up(self._stage_id, pid)
        except Exception as exc:                                  # noqa: BLE001
            self._note(f"wake_up failed on {root}: {exc!r}")
        try:
            self._sim.apply_force_at_pos(
                self._stage_id, pid,
                carb_float3(Gf.Vec3d(n[0] * j, n[1] * j, 0.0)),
                carb_float3(point),
                "Impulse",
            )
        except Exception as exc:
            self._note(f"apply_force_at_pos failed on {root}: {exc!r}")
            return

        self.stats["impulses"] += 1
        self.stats["impulse_ns_total"] += float(j)

    # -- the other detector, kept for comparison ---------------------------
    def _on_contact(self, headers, data) -> None:
        try:
            self._contact(headers, data)
        except Exception as exc:                                  # noqa: BLE001
            self.stats["errors"] += 1
            self._note(f"contact detector failed, DISARMING: {exc!r}")
            self._contact_sub = None

    def _contact(self, headers, data) -> None:
        """PhysX contact reports, for the pairs that include the CCT capsule.

        Whether the character controller's internal kinematic actor shows up
        here at all is the open question this detector exists to answer --
        ``PhysxContactReportAPI`` is applied to the props, not to the capsule,
        and the capsule carries no ``CollisionAPI`` of its own (see
        sim/avatar.py for why it must not).
        """
        from pxr import PhysicsSchemaTools

        for h in headers:
            self.stats["contact_events"] += 1
            a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            if self.body_path not in (a0, a1):
                continue
            root = self._pushable_root(a1 if a0 == self.body_path else a0)
            if root is None:
                continue
            self.stats["contact_pairs_with_cct"] += 1
            if self.detector != "contact":
                continue  # measuring only; the sweep already pushed
            if h.num_contact_data <= 0:
                continue
            # Index the buffer rather than materialising it: this fires per
            # contact pair per step, and list(data) copies the whole frame's
            # contacts every time.
            cd = data[h.contact_data_offset]
            self._apply(root, Gf.Vec3d(*cd.position), Gf.Vec3d(*cd.normal),
                        Gf.Vec3d(0, 0, 0), 1.0 / 60.0)

    def _note(self, msg: str) -> None:
        if len(self.log_lines) < 20:
            self.log_lines.append(msg)
            log("! " + msg)

    def close(self) -> None:
        self._step_sub = None
        self._contact_sub = None


def carb_float3(v):
    import carb

    return carb.Float3(float(v[0]), float(v[1]), float(v[2]))


def install_push_callback(
    stage: Usd.Stage,
    converted: dict,
    cfg: dict | None = None,
    *,
    body_path: str = "/Root/Avatar/body_mesh",
    extra_bodies: dict | None = None,
) -> PushCallback | None:
    """Wire the hit callback for the props :func:`make_pushable` converted.

    ``extra_bodies`` is ``{prim path: mass}`` for dynamic bodies this module did
    not author -- the robots' physics proxies from ``sim/nav_obstacles.py``.
    They go through the SAME impulse model rather than a second push path: one
    set of numbers that can be checked once, and a robot that responds to a
    shove exactly as a crate of the same mass would.

    Returns None -- loudly -- rather than raising, so a GUI session whose props
    failed to convert still comes up with sensors and an avatar.
    """
    cfg = cfg or load_pushable_config()
    bodies = {p: rec["mass_kg"] for p, rec in (converted.get("made") or {}).items()}
    bodies.update({k: float(v) for k, v in (extra_bodies or {}).items()})
    if not bodies:
        log("no dynamic props -- push callback NOT installed")
        return None
    if not stage.GetPrimAtPath(body_path).IsValid():
        log(f"! avatar body {body_path} not on this stage -- push callback NOT installed")
        return None
    try:
        cb = PushCallback(
            stage, body_path, bodies,
            avatar_mass_kg=float(cfg["avatar_mass_kg"]),
            max_push_force_n=float(cfg["max_push_force_n"]),
            min_push_speed_ms=float(cfg["min_push_speed_ms"]),
            detector=str(cfg["detector"]),
            n_probes=int(cfg["n_probes"]),
        )
    except Exception as exc:
        log(f"! push callback failed to install: {exc!r}")
        return None
    log(f"push callback on ({cb.detector}): {len(bodies)} bodies "
        f"({len(extra_bodies or {})} of them robot proxies), "
        f"M={cb.M:g} kg, F_max={cb.f_max:g} N, capsule r={cb.radius:.3f} "
        f"half-h={cb.half_height:.3f}, {cb.n_probes} probes")
    return cb
