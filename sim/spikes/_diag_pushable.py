"""Which floor-level props can the avatar push, and what does it cost?

Three phases, one script, selected with ``PP_PHASE``. They are separate runs
because each container launch costs a minute of shader warm-up, and -- more
importantly -- because of CLAUDE.md failure mode 11: **the frame-rate phase
reads no annotator at all, and the push phase reads one every leg.** Mixing
them would put a 16 ms readback on whichever arm happened to take a picture.

    PP_PHASE=enumerate   read candidate props OFF THE STAGE (hard rule 1) and
                         write them to logs/pushable_candidates.json. Renders
                         nothing, plays nothing, changes nothing.
    PP_PHASE=push        convert the declared props to dynamic rigid bodies,
                         install the hit callback, drive the character
                         controller into each of four targets in turn and
                         report what moved and how far. Reports no frame
                         times, and reads no annotator unless PP_SHOOT=1.
    PP_PHASE=fps         the frame-rate cost, five arms in ONE process so the
                         machine's load is the same for all of them. Creates no
                         render product and calls no ``get_data()`` -- so the
                         arms agree, which is the whole requirement.

Exec mode (CLAUDE.md): no SimulationApp, frames come from the update event
stream, results are written incrementally and fsync'd to a container-writable
path (``/workspace`` is owned by another uid and fails silently).

Environment
    PP_PHASE     enumerate | audit | obstacles | drive | push | fps
                 (default enumerate)
    PP_STAGE     stage to open
    PP_OUT       output directory
    PP_PUSH      0 to convert props but NOT install the callback (the arm that
                 answers "does the CCT already push them by itself?")
    PP_DETECTOR  sweep | contact | both -- overrides config/scene.yaml
    PP_MASK      0 to skip disable_unreachable_colliders
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
# BOTH guards, set before any sibling import and never after. Missing the
# second one is silent in the expensive direction: `import observation_adapter`
# ends in `if os.environ.get("OA_NO_AUTORUN") != "1": _exec_entrypoint()`,
# which OPENS ITS OWN STAGE and runs the contract suite inside this session.
# Measured 2026-09-02: the symptom was not a hijacked run but
# `Stage.GetPrimAtPath(Stage, Path) did not match C++ signature ... SdfPath` --
# an argument-type error on a correctly typed argument, because the Stage
# object still in hand belonged to a context the import had torn down. Nothing
# named the adapter anywhere in that traceback.
os.environ["SF_NO_AUTORUN"] = "1"
os.environ["OA_NO_AUTORUN"] = "1"

PHASE = os.environ.get("PP_PHASE", "enumerate")
STAGE = os.environ.get("PP_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
OUT = Path(os.environ.get("PP_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
REACH_M = float(os.environ.get("PP_REACH_M", "2.2"))
RADIUS_M = float(os.environ.get("PP_RADIUS_M", "14.0"))
WITH_PUSH = os.environ.get("PP_PUSH") != "0"
DETECTOR = os.environ.get("PP_DETECTOR")
WITH_MASK = os.environ.get("PP_MASK") != "0"
# PNGs are OFF by default, and that is a measured decision, not caution:
# creating a 640x360 render product in this phase put the renderer into
# `vkCreateFence -> ERROR_OUT_OF_HOST_MEMORY` and then an unbounded
# `Failed to allocate 640x360 LdrColor resource` loop with 91 GB of host RAM
# free and 22 GB of VRAM free, and the run never reached its first leg.
# Displacement is the measurement here; the picture is the GUI's job.
WITH_SHOTS = os.environ.get("PP_SHOOT") == "1"
# Force every prop to one approximation, overriding config/scene.yaml. The
# point is the A/B: three of the ten ship as convexDecomposition, and one of
# those three is the 1.2 kg crate whose response to an impulse has never been
# explained. Running the same legs with every prop on convexHull is how you
# find out whether the collider shape is involved.
APPROX = os.environ.get("PP_APPROX")
# obstacles phase: 1 to apply the nav colliders and the controller tuning after
# the first probe and probe again, so before/after live in one artifact.
WITH_FIX = os.environ.get("PP_FIX") == "1"
# push phase: "all" runs a leg for EVERY declared prop instead of the
# min/mid/max sample. The sample was chosen by mass and silently skipped the
# 2.0 kg traffic cones entirely -- three of the ten props, and the mass band
# either side of the one prop that has never behaved.
ALL_LEGS = os.environ.get("PP_LEGS") == "all"

BODY = "/Root/Avatar/body_mesh"
CONTROLS = "/Root/Avatar/Controls"
TP_CAM = "/Root/Avatar/body_mesh/cam_third_person"
RES = (640, 360)


_T0 = time.perf_counter()


def log(m: str) -> None:
    """Timestamped, because half of what this run measures is how long a step
    took and the Kit log's own clock stops where the script's work begins."""
    print(f"[pushable +{time.perf_counter() - _T0:7.1f}s] {m}", flush=True)


def write_json(name: str, payload) -> None:
    """Incremental and fsync'd -- this renderer dies mid-run (CLAUDE.md)."""
    path = OUT / name
    try:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=1, default=str)
            fh.flush()
            os.fsync(fh.fileno())
        log(f"wrote {path}")
    except Exception as exc:
        log(f"! could not write {path}: {exc!r}")


def loadavg() -> str:
    try:
        return open("/proc/loadavg").read().split()[0]
    except Exception:
        return "?"


# ---------------------------------------------------------------------------
# Phase: enumerate
# ---------------------------------------------------------------------------
def avatar_cfg() -> dict:
    import yaml

    with open(REPO / "config" / "scene.yaml", encoding="utf-8") as fh:
        return yaml.safe_load(fh)["avatar"]


def collider_leaves(prim: Usd.Prim) -> list[Usd.Prim]:
    return [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)]


def enumerate_candidates(stage: Usd.Stage) -> dict:
    """Every direct child of /Root/Warehouse that could plausibly be pushed.

    Deliberately NOT narrowed to the shipped shortlist. This phase reports the
    whole floor-level population with its measurements so that a human picks
    the shortlist and writes it into config/scene.yaml -- which is what hard
    rule 1 asks for, and what rule 5's registry pattern does for sensors.
    """
    t0 = time.perf_counter()
    sx, sy = (float(v) for v in avatar_cfg()["spawn_xy"])
    wh = stage.GetPrimAtPath("/Root/Warehouse")
    if not wh.IsValid():
        return {"error": "/Root/Warehouse is not on this stage"}

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rows = []
    for child in wh.GetChildren():
        path = child.GetPath().pathString
        try:
            rng = cache.ComputeWorldBound(child).ComputeAlignedRange()
        except Exception as exc:
            rows.append({"path": path, "error": repr(exc)})
            continue
        if rng.IsEmpty():
            continue
        lo, hi = rng.GetMin(), rng.GetMax()
        size = [float(hi[i] - lo[i]) for i in range(3)]
        cx, cy = float((lo[0] + hi[0]) / 2), float((lo[1] + hi[1]) / 2)
        leaves = collider_leaves(child)
        # Whether the geometry is instanced decides whether the collision
        # approximation can be authored at all -- an instance proxy is not
        # editable, and a dynamic body needs a convex approximation.
        instanced = bool(child.IsInstance()) or any(p.IsInstanceProxy() for p in leaves)
        rows.append({
            "path": path,
            "type": str(child.GetTypeName()),
            "z_min": round(float(lo[2]), 4),
            "z_max": round(float(hi[2]), 4),
            "size": [round(v, 4) for v in size],
            "center_xy": [round(cx, 4), round(cy, 4)],
            "dist_from_spawn_m": round(((cx - sx) ** 2 + (cy - sy) ** 2) ** 0.5, 3),
            "n_colliders": len(leaves),
            "instanced": instanced,
            "already_rigid": any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in Usd.PrimRange(child)),
            "approximations": sorted({
                str(UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get())
                for p in leaves if p.HasAPI(UsdPhysics.MeshCollisionAPI)}),
        })

    floor = [
        r for r in rows
        if "error" not in r
        and r["n_colliders"] > 0
        and r["z_min"] <= 0.40                       # actually sits on the floor
        and r["z_max"] <= REACH_M                    # entirely within reach
        and max(r["size"][0], r["size"][1]) <= 1.60  # not a rack, not a wall
        and r["dist_from_spawn_m"] <= RADIUS_M
    ]
    floor.sort(key=lambda r: r["dist_from_spawn_m"])
    fam: dict[str, int] = defaultdict(int)
    for r in floor:
        fam[r["path"].split("/")[-1].rstrip("0123456789_")] += 1
    return {
        "stage": STAGE, "spawn_xy": [sx, sy], "reach_m": REACH_M, "radius_m": RADIUS_M,
        "n_warehouse_children": len(rows), "n_floor_candidates": len(floor),
        "seconds": round(time.perf_counter() - t0, 2),
        "families": dict(sorted(fam.items())), "candidates": floor,
    }


# ---------------------------------------------------------------------------
# Shared: driving the character controller from code
# ---------------------------------------------------------------------------
class Driver:
    """Walk the capsule in a straight line, headlessly.

    The keyboard graph cannot be used here -- there is no keyboard -- and it
    would fight us if it were live: ``setup_controls`` registers a stage-update
    node that writes the (zero) key state every frame, so the last writer wins
    and it is not us. The graph prim is deactivated and the controller is armed
    directly through ``CharacterController.activate``.

    The walk itself is ``set_position`` per frame, not ``set_move``; see the
    bisect in ``__init__`` and in :class:`DriveBisect` for why, and for what
    that costs.
    """

    def __init__(self, stage: Usd.Stage, speed_ms: float) -> None:
        from omni.physxcct.scripts.ifaces import get_physx_cct_interface
        from omni.physxcct.scripts.utils import (
            CharacterController,
            register_stage_update_node,
        )

        self.stage = stage
        self.speed = float(speed_ms)
        self.cct = get_physx_cct_interface()
        self.cache = UsdGeom.XformCache()
        graph = stage.GetPrimAtPath(CONTROLS)
        self.graph_deactivated = False
        if graph.IsValid():
            graph.SetActive(False)
            self.graph_deactivated = True
            log(f"deactivated {CONTROLS} -- this run drives the capsule itself")
        # ACTIVATE THROUGH THE SHIPPED CLASS, not by calling enable_gravity and
        # hoping. Measured 2026-09-01: with only `enable_gravity(path)`,
        # `set_position` works -- the capsule lands exactly where it is put and
        # physics settles its z from 0.900 to 0.895 -- and `set_move` is a
        # silent no-op: 434 physics steps, 2.2 m commanded per leg, capsule
        # displacement 0.000 m on all four legs and not one error. Two calls
        # against the same interface, one live and one dead, is the worst shape
        # a failure can have. `CharacterController.activate()` is what the CCT
        # demo and the OmniGraph node both call; step offset is passed
        # explicitly so activate() does not overwrite avatar.py's 0.2 with its
        # own 0.5 default.
        self.handle = CharacterController(BODY, None, True, 0.2)
        self.handle.activate(stage)
        # THE WALK IS A set_position SCRIPT, NOT set_move, AND THAT IS
        # MEASURED. `PP_PHASE=drive` bisected it (logs/pushable_drive.json):
        # five arms, 70 frames each, 1.4 m/s commanded --
        #
        #     A  set_move, nothing else                    0.0000 m
        #     B  + CharacterController.activate() at Play  0.0000 m
        #     C  + activate_cct(path)                      0.0000 m
        #     D  + enable_worldspace_move(path, True)      0.0000 m
        #     E  set_position walked by hand               1.9806 m
        #
        # Every call returned cleanly. `set_move` is a silent no-op in exec
        # mode on this host whichever way the controller is armed, and it is
        # not an ordering problem: arms A-D all drove it from a **pre-physics
        # stage update node**, which is where NVIDIA's own `update_movement`
        # lives. Arm E is the control and proves the ruler works.
        #
        # THE COST OF THAT, STATED PLAINLY: a set_position walk is a placement
        # per frame, not a swept move, so it does NOT collide and slide. This
        # spike therefore cannot test "walking into a shelf stops you" -- that
        # half of the gate belongs to the GUI, where the keyboard drives the
        # controller through its own OmniGraph node. It is the same caveat
        # FINDINGS already records for the S11 contract circuit: "a scripted
        # walk, not CCT collide-and-slide -- it says nothing about the
        # character controller".
        self.target_xy: tuple[float, float] | None = None
        self.commanded_m = 0.0
        cap = UsdGeom.Capsule(stage.GetPrimAtPath(BODY))
        self.radius = float(cap.GetRadiusAttr().Get() or 0.30) if cap else 0.30
        self._node = register_stage_update_node(
            "pushable_spike_cct", on_update_fn=self._on_stage_update)
        log(f"CCT activated on {BODY} via omni.physxcct CharacterController, "
            f"driven from a pre-physics stage update node; "
            f"interface: {sorted(n for n in dir(self.cct) if not n.startswith('_'))}")

    def _on_stage_update(self, _current_time, dt) -> None:
        """Pre-physics: advance the capsule one frame toward the target.

        Pre-physics still matters even though this is a placement rather than
        a move: the pose physics simulates this step must be the pose the push
        callback differenced, or the avatar's velocity is one step stale
        against the contact it produced.
        """
        try:
            if self.target_xy is None or dt <= 0.0:
                return
            p = self.pos()
            dx = float(self.target_xy[0]) - p[0]
            dy = float(self.target_xy[1]) - p[1]
            n = (dx * dx + dy * dy) ** 0.5
            if n < 1e-6:
                return
            d = min(self.speed * float(dt), n)
            self.cct.set_position(BODY, (p[0] + dx / n * d, p[1] + dy / n * d, p[2]))
            self.commanded_m += d
        except Exception as exc:                                  # noqa: BLE001
            log(f"! stage update driver failed: {exc!r}")
            self.target_xy = None

    def pos(self) -> Gf.Vec3d:
        self.cache.Clear()
        return self.cache.GetLocalToWorldTransform(
            self.stage.GetPrimAtPath(BODY)).ExtractTranslation()

    def teleport(self, x: float, y: float, z: float) -> None:
        # Re-assert gravity here rather than only in __init__: the CCT manager
        # is created when the physics scene attaches, which happens at Play,
        # and __init__ runs before it. A call made before the manager exists is
        # not an error and is not remembered either.
        self.cct.enable_gravity(BODY)
        self.cct.set_position(BODY, (float(x), float(y), float(z)))

    def walk_to(self, target_xy) -> None:
        """Start walking toward a point. The stage node does the rest.

        Distance, not frame count, is what a leg is measured in: the driver
        advances a per-frame DISPLACEMENT, so the ground covered by "110
        frames" depends on the frame rate -- 10 m at 15 fps, 2.5 m at 60. Read
        ``commanded_m`` to end a leg.
        """
        self.target_xy = (float(target_xy[0]), float(target_xy[1]))

    def halt(self) -> None:
        self.target_xy = None


def _dist(a, b) -> float:
    if not a or not b:
        return 0.0
    return sum((float(b[i]) - float(a[i])) ** 2 for i in range(3)) ** 0.5


def _quat_angle_deg(a, b) -> float:
    """Angle between two orientations, in degrees. Tipping is not sliding."""
    import math

    d = abs(sum(float(a[i]) * float(b[i]) for i in range(4)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, d))))


def physics_facts(stage: Usd.Stage) -> dict:
    """Everything that decides whether a dynamic body is reproducible.

    Recorded on every run because it is the evidence for the capture-mode
    question and none of it is inferable from the result. PhysX is
    deterministic for the same binary, the same scene, the same call sequence
    AND the same thread count -- the last of which this repo exposes as a
    Makefile knob whose comment currently says it cannot change results.
    """
    import carb
    from pxr import PhysxSchema

    st = carb.settings.get_settings()
    out = {
        "num_threads": st.get("/persistent/physics/numThreads"),
        "carb_task_threads": st.get("/plugins/carb.tasking.plugin/threadCount"),
        "min_frame_rate": st.get("/persistent/simulation/minFrameRate"),
        "update_to_usd": st.get("/physics/updateToUsd"),
        "fabric_enabled": st.get("/physics/fabricEnabled"),
        "scenes": [],
    }
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Scene):
            continue
        rec = {"path": prim.GetPath().pathString}
        api = PhysxSchema.PhysxSceneAPI(prim)
        if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            for name, attr in (
                ("time_steps_per_second", api.GetTimeStepsPerSecondAttr()),
                ("solver_type", api.GetSolverTypeAttr()),
                ("enable_ccd", api.GetEnableCCDAttr()),
                ("enable_gpu_dynamics", api.GetEnableGPUDynamicsAttr()),
                ("broadphase_type", api.GetBroadphaseTypeAttr()),
                ("enable_stabilization", api.GetEnableStabilizationAttr()),
            ):
                try:
                    rec[name] = attr.Get() if attr else None
                except Exception:
                    rec[name] = "<unreadable>"
        out["scenes"].append(rec)
    return out


def world_xy(stage: Usd.Stage, path: str, cache: UsdGeom.XformCache):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    t = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    return Gf.Vec3d(t[0], t[1], t[2])


def bbox_center(stage: Usd.Stage, path: str):
    c = bbox_center_half(stage, path)[0]
    return c


def bbox_center_half(stage: Usd.Stage, path: str):
    """World centre and horizontal half-extent of a prim's bound."""
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None, 0.0
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        return None, 0.0
    lo, hi = rng.GetMin(), rng.GetMax()
    c = Gf.Vec3d(*[(lo[i] + hi[i]) / 2.0 for i in range(3)])
    half = 0.5 * max(float(hi[0] - lo[0]), float(hi[1] - lo[1]))
    return c, half


# ---------------------------------------------------------------------------
# Phase: audit -- what collision does anything actually have?
# ---------------------------------------------------------------------------
class Audit:
    """Read-only for questions 1 and 2. Three questions, reported before changes.

    1. What collision do ``/Root/Worker`` and the three robots carry? A skinned
       character routinely ships render geometry and no collider, and the
       robots are referenced at runtime by ``reference_robots()``, so what they
       have is a property of the ASSET.

       **The traversal uses ``Usd.TraverseInstanceProxies()``.** A plain
       ``Usd.PrimRange`` does not descend into instance prototypes, so a
       referenced robot reports 0 meshes and 0 colliders whether or not it has
       any -- the first pass of this audit did exactly that and would have had
       me "fix" something I had not measured.

    2. Step offset and slope limit, at load and again after Play -- not the
       same question. ``OgnCharacterController`` constructs
       ``CharacterController(path, cam, gravity, 0.01)`` and its ``activate()``
       writes ``stepOffset`` and ``upAxis``, so the runtime value is whatever
       the graph wrote. This phase leaves the Controls graph ACTIVE to observe
       it. Beside that, the floor-obstacle height table: choosing a step offset
       is choosing a height, and the scene decides which heights are available.

    3. How much solid does convexification invent? Every prop is cooked BOTH
       ways and the collider volumes compared, because "does convexDecomposition
       fix it" is a question about the difference between two colliders and
       neither is inspectable from the mesh.
    """

    WAIT_PAYLOAD = 90
    WAIT_PLAY = 60
    HULL_TIMEOUT = 900

    def __init__(self, stage: Usd.Stage) -> None:
        import sensor_factory as sf

        self.stage = stage
        self.sf = sf
        self.report: dict = {"stage": STAGE, "physics": S.get("physics")}
        self.state = "referencing"
        self.counter = 0
        self.hulls: dict[str, dict] = {}
        self.pending = 0
        self.leaves: list = []
        self.robots = sf.reference_robots(stage)
        log(f"referenced {len(self.robots)} robots; waiting {self.WAIT_PAYLOAD} "
            f"frames for payloads before reading their collision")

    # -- 1. collision audit -------------------------------------------------
    def audit_tree(self, path: str) -> dict:
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return {"path": path, "error": "not on this stage"}
        n = meshes = colliders = enabled = bodies = kinematic = arts = skel = 0
        instanced = 0
        approximations: dict[str, int] = {}
        collider_paths: list[str] = []
        for q in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
            n += 1
            if q.IsInstanceProxy():
                instanced += 1
            if q.IsA(UsdGeom.Mesh):
                meshes += 1
            if q.GetTypeName() in ("SkelRoot", "Skeleton"):
                skel += 1
            if q.HasAPI(UsdPhysics.CollisionAPI):
                colliders += 1
                if len(collider_paths) < 6:
                    collider_paths.append(q.GetPath().pathString)
                if UsdPhysics.CollisionAPI(q).GetCollisionEnabledAttr().Get() is not False:
                    enabled += 1
                if q.HasAPI(UsdPhysics.MeshCollisionAPI):
                    a = str(UsdPhysics.MeshCollisionAPI(q).GetApproximationAttr().Get())
                    approximations[a] = approximations.get(a, 0) + 1
            if q.HasAPI(UsdPhysics.RigidBodyAPI):
                bodies += 1
                if UsdPhysics.RigidBodyAPI(q).GetKinematicEnabledAttr().Get():
                    kinematic += 1
            if q.HasAPI(UsdPhysics.ArticulationRootAPI):
                arts += 1
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        box = None
        if not rng.IsEmpty():
            lo, hi = rng.GetMin(), rng.GetMax()
            box = {"min": [round(float(v), 3) for v in lo],
                   "max": [round(float(v), 3) for v in hi],
                   "size": [round(float(hi[i] - lo[i]), 3) for i in range(3)]}
        return {
            "path": path, "prims": n, "instance_proxies": instanced,
            "meshes": meshes, "skel_prims": skel,
            "colliders": colliders, "colliders_enabled": enabled,
            "rigid_bodies": bodies, "kinematic_bodies": kinematic,
            "articulation_roots": arts, "approximations": approximations,
            "example_colliders": collider_paths, "world_bbox": box,
            "verdict": ("NO ENABLED COLLIDER -- the avatar walks through it, and "
                        "so does anything else that consults colliders"
                        if enabled == 0 else f"{enabled} enabled collider(s)"),
        }

    # -- 2a. the character controller ---------------------------------------
    def audit_cct(self) -> dict:
        import math

        from pxr import PhysxSchema

        prim = self.stage.GetPrimAtPath(BODY)
        if not prim.IsValid():
            return {"error": f"{BODY} not on this stage"}
        api = PhysxSchema.PhysxCharacterControllerAPI(prim)
        out = {"path": BODY, "has_api": bool(prim.HasAPI(
            PhysxSchema.PhysxCharacterControllerAPI))}
        for name, attr in (
            ("step_offset", api.GetStepOffsetAttr()),
            ("slope_limit", api.GetSlopeLimitAttr()),
            ("contact_offset", api.GetContactOffsetAttr()),
            ("up_axis", api.GetUpAxisAttr()),
            ("scale_coeff", api.GetScaleCoeffAttr()),
            ("volume_growth", api.GetVolumeGrowthAttr()),
            ("non_walkable_mode", api.GetNonWalkableModeAttr()),
            ("climbing_mode", api.GetClimbingModeAttr()),
        ):
            try:
                v = attr.Get() if attr else None
            except Exception as exc:                              # noqa: BLE001
                v = f"<{type(exc).__name__}>"
            out[name] = v
        # slopeLimit is the COSINE of the limit angle (schema.usda: "The limit
        # is expressed as the cosine of the desired limit angle. A value of 0
        # disables this feature"). Reported in degrees as well, because 0.5
        # reads like a small number and means 60 degrees.
        try:
            c = float(out.get("slope_limit") or 0.0)
            out["slope_limit_deg"] = (round(math.degrees(math.acos(max(-1.0, min(1.0, c)))), 1)
                                      if c > 0 else "disabled (0)")
        except Exception:                                         # noqa: BLE001
            pass
        cap = UsdGeom.Capsule(prim)
        if cap:
            out["capsule_radius"] = float(cap.GetRadiusAttr().Get() or 0)
            out["capsule_height"] = float(cap.GetHeightAttr().Get() or 0)
        graph = self.stage.GetPrimAtPath(CONTROLS)
        out["controls_graph_active"] = bool(graph.IsValid() and graph.IsActive())
        return out

    # -- 2b. what a step offset would have to clear, or not -----------------
    def floor_obstacles(self) -> dict:
        """Every floor-level collider near the walk, bucketed by height.

        This is the trade-off, made of the scene rather than of intuition:
        a step offset must sit ABOVE everything the avatar should walk over and
        BELOW everything that should stop it. The table says which heights are
        actually occupied, and therefore where the gap is.
        """
        sx, sy = (float(v) for v in avatar_cfg()["spawn_xy"])
        wh = self.stage.GetPrimAtPath("/Root/Warehouse")
        if not wh.IsValid():
            return {"error": "no /Root/Warehouse"}
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        rows = []
        for child in wh.GetChildren():
            if not any(q.HasAPI(UsdPhysics.CollisionAPI)
                       for q in Usd.PrimRange(child, Usd.TraverseInstanceProxies())):
                continue
            rng = cache.ComputeWorldBound(child).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            lo, hi = rng.GetMin(), rng.GetMax()
            if float(lo[2]) > 0.05:          # not resting on the floor
                continue
            top = float(hi[2])
            if top > 0.80:                   # taller than any plausible step
                continue
            cx, cy = float((lo[0] + hi[0]) / 2), float((lo[1] + hi[1]) / 2)
            d = ((cx - sx) ** 2 + (cy - sy) ** 2) ** 0.5
            if d > 20.0:
                continue
            rows.append({"path": child.GetPath().pathString,
                         "top_m": round(top, 4), "dist_m": round(d, 2)})
        rows.sort(key=lambda r: r["top_m"])
        buckets: dict[str, int] = {}
        for r in rows:
            t = r["top_m"]
            k = ("0.000 (flat)" if t < 0.001 else "0.001-0.02" if t < 0.02
                 else "0.02-0.05" if t < 0.05 else "0.05-0.10" if t < 0.10
                 else "0.10-0.20" if t < 0.20 else "0.20-0.40" if t < 0.40
                 else "0.40-0.80")
            buckets[k] = buckets.get(k, 0) + 1
        return {"n": len(rows), "buckets": buckets,
                "shortest_20": rows[:20],
                "note": "colliders resting on the floor (z_min <= 0.05) and "
                        "under 0.80 m tall, within 20 m of spawn"}

    # -- 3. hull vs hull ----------------------------------------------------
    def convert_and_list(self) -> None:
        import pushable_props as pp

        cfg = pp.load_pushable_config()
        # The one change this phase makes, and it is the configuration under
        # test: props ship as `approximation = "none"`, so cooking them
        # unconverted returns the triangle mesh and answers nothing. Questions
        # 1 and 2 were both read before this line.
        self.report["converted"] = pp.make_pushable(self.stage, cfg)
        self.report["configured_approximation"] = {
            s["prim_path"]: s.get("approximation") for s in cfg["props"]}
        for spec in cfg["props"]:
            prim = self.stage.GetPrimAtPath(spec["prim_path"])
            if not prim.IsValid():
                continue
            for leaf in pp.collider_leaves(prim):
                self.leaves.append((spec["prim_path"], leaf))

    def request_all(self, approximation: str) -> None:
        from pxr import PhysicsSchemaTools, Sdf

        try:
            from omni.physx import get_physx_cooking_interface
        except Exception as exc:                                  # noqa: BLE001
            self.report["hulls_error"] = repr(exc)
            return
        cook = get_physx_cooking_interface()
        self.pending = 0
        for root, leaf in self.leaves:
            UsdPhysics.MeshCollisionAPI.Apply(leaf).CreateApproximationAttr().Set(
                approximation)
        for root, leaf in self.leaves:
            key = f"{approximation}|{leaf.GetPath().pathString}"
            self.pending += 1
            self._request_one(cook, key, root, leaf, approximation,
                              PhysicsSchemaTools, Sdf)
        log(f"  requested {self.pending} cooks at approximation={approximation}")

    def _request_one(self, cook, key, root, leaf, approximation, PST, Sdf) -> None:
        def on_result(result, convexes) -> None:
            try:
                self.hulls[key] = self._measure(leaf, convexes, root, approximation)
            except Exception as exc:                              # noqa: BLE001
                import traceback
                self.hulls[key] = {"root": root, "approximation": approximation,
                                   "error": repr(exc),
                                   "tb": traceback.format_exc()[-400:]}
            log(f"  {approximation:20s} {root.split('/')[-1]:32s} "
                f"{self.hulls[key].get('summary', self.hulls[key].get('error'))}")

        try:
            cook.request_convex_collision_representation(
                stage_id=self._stage_id(),
                collision_prim_id=PST.sdfPathToInt(Sdf.Path(leaf.GetPath().pathString)),
                run_asynchronously=True,
                on_result=on_result,
            )
        except Exception as exc:                                  # noqa: BLE001
            self.hulls[key] = {"root": root, "approximation": approximation,
                               "error": f"request failed: {exc!r}"}

    def _stage_id(self) -> int:
        from pxr import UsdUtils

        return UsdUtils.StageCache.Get().GetId(self.stage).ToLongInt()

    def _measure(self, leaf, convexes, root: str, approximation: str) -> dict:
        """Total volume of the cooked collider, in world cubic metres.

        Summed PER PART. Taking one hull over the union of every part's
        vertices is the convexHull answer again, which is how the first pass of
        this audit reported a 16-part decomposition and a single hull as having
        the same volume.
        """
        m = UsdGeom.XformCache().GetLocalToWorldTransform(leaf)
        det = abs(Gf.Matrix3d(m.ExtractRotationMatrix()).GetDeterminant())
        # ExtractRotationMatrix drops scale, so recover the scale factor from
        # the full 3x3 instead: local volumes are in the collider's own frame.
        m3 = Gf.Matrix3d(*[m[i][j] for i in range(3) for j in range(3)])
        scale_vol = abs(m3.GetDeterminant()) or 1.0
        parts = [_convex_volume(c) for c in (convexes or [])]
        parts = [v for v in parts if v is not None]
        if not parts:
            return {"root": root, "approximation": approximation,
                    "error": "no convex data returned"}
        total = sum(parts) * scale_vol
        verts = sum(len(c.vertices) for c in convexes)
        out = {"root": root, "approximation": approximation,
               "parts": len(parts), "hull_vertices": int(verts),
               "collider_volume_m3": round(total, 6),
               "largest_part_m3": round(max(parts) * scale_vol, 6)}
        out["summary"] = (f"{out['parts']:2d} part(s), {out['hull_vertices']:4d} verts, "
                          f"collider volume {out['collider_volume_m3']:.5f} m3")
        del det
        return out

    # -- driver -------------------------------------------------------------
    def tick(self, dt: float) -> bool:
        self.counter += 1

        if self.state == "referencing":
            if self.counter < self.WAIT_PAYLOAD:
                return False
            log("payloads settled; reading collision BEFORE anything is changed")
            targets = ["/Root/Worker"] + sorted(self.robots.values())
            self.report["trees"] = [self.audit_tree(t) for t in targets]
            for t in self.report["trees"]:
                log(f"  {t['path']}: {t.get('verdict', t.get('error'))}")
                log(f"      {t.get('prims')} prims ({t.get('instance_proxies')} "
                    f"instance proxies), {t.get('meshes')} meshes, "
                    f"{t.get('colliders')} colliders, {t.get('rigid_bodies')} rigid "
                    f"bodies, {t.get('articulation_roots')} articulation roots")
            self.report["cct_before_play"] = self.audit_cct()
            log(f"  CCT before Play: {self.report['cct_before_play']}")
            self.report["floor_obstacles"] = self.floor_obstacles()
            log(f"  floor obstacles by height: "
                f"{self.report['floor_obstacles'].get('buckets')}")
            self.convert_and_list()
            self.request_all("convexHull")
            self.state, self.counter = "hull_a", 0
            return False

        if self.state == "hull_a":
            done = sum(1 for k in self.hulls if k.startswith("convexHull|"))
            if done < self.pending and self.counter < self.HULL_TIMEOUT:
                return False
            self.request_all("convexDecomposition")
            self.state, self.counter = "hull_b", 0
            return False

        if self.state == "hull_b":
            done = sum(1 for k in self.hulls if k.startswith("convexDecomposition|"))
            if done < self.pending and self.counter < self.HULL_TIMEOUT:
                return False
            self.report["hulls"] = self.hulls
            self.report["hull_comparison"] = self._compare_table()
            omni.timeline.get_timeline_interface().play()
            log("Play pressed WITH the Controls graph active -- reading the CCT "
                "again to see what OgnCharacterController writes")
            self.state, self.counter = "playing", 0
            return False

        if self.state == "playing":
            if self.counter < self.WAIT_PLAY:
                return False
            self.report["cct_after_play"] = self.audit_cct()
            log(f"  CCT after Play:  {self.report['cct_after_play']}")
            b, a = self.report["cct_before_play"], self.report["cct_after_play"]
            changed = {k: [b.get(k), a.get(k)] for k in b
                       if k != "path" and b.get(k) != a.get(k)}
            self.report["cct_changed_by_play"] = changed
            log(f"  changed by Play: {changed or 'nothing'}")
            return True

        return False

    def _compare_table(self) -> list:
        out = []
        for root, leaf in self.leaves:
            lp = leaf.GetPath().pathString
            h = self.hulls.get(f"convexHull|{lp}") or {}
            d = self.hulls.get(f"convexDecomposition|{lp}") or {}
            vh, vd = h.get("collider_volume_m3"), d.get("collider_volume_m3")
            row = {"root": root, "leaf": lp,
                   "configured": self.report["configured_approximation"].get(root),
                   "convexHull_m3": vh, "convexDecomposition_m3": vd,
                   "parts_decomposed": d.get("parts")}
            if vh and vd:
                row["hull_over_decomposition"] = round(vh / vd, 3)
                row["extra_solid_m3"] = round(vh - vd, 5)
            out.append(row)
        out.sort(key=lambda r: -(r.get("hull_over_decomposition") or 0))
        for r in out:
            log(f"  {r['root'].split('/')[-1]:32s} hull {r['convexHull_m3']} "
                f"vs decomp {r['convexDecomposition_m3']} "
                f"= x{r.get('hull_over_decomposition')} "
                f"({r.get('parts_decomposed')} parts, configured "
                f"{r['configured']})")
        return out

    def result(self) -> dict:
        self.report["loadavg"] = loadavg()
        self.report["note"] = (
            "questions 1 and 2 are read BEFORE anything is changed. Question 3 "
            "converts the ten props and then cooks each collider BOTH ways, "
            "because comparing two colliders is the only way to answer whether "
            "convexDecomposition helps.")
        return self.report


def _convex_volume(cm) -> float | None:
    """Exact volume of one cooked convex mesh, from its own polygons.

    No scipy in this image (checked). PhysX hands back vertices, an index
    buffer and per-polygon (index_base, num_vertices, plane) with consistent
    outward winding, so the divergence theorem over a triangle fan of each
    polygon is exact and needs nothing else.
    """
    import numpy as np

    try:
        v = np.asarray([[p.x, p.y, p.z] for p in cm.vertices], dtype=float)
        idx = list(cm.indices)
        total = 0.0
        for poly in cm.polygons:
            base, n = int(poly.index_base), int(poly.num_vertices)
            if n < 3:
                continue
            ring = [v[idx[base + k]] for k in range(n)]
            a = ring[0]
            for k in range(1, n - 1):
                total += float(np.dot(a, np.cross(ring[k], ring[k + 1])))
        return abs(total) / 6.0
    except Exception:                                             # noqa: BLE001
        return None


# ---------------------------------------------------------------------------
# Phase: obstacles -- does PHYSICS have the Worker and the robots, and do the
# SENSORS?
# ---------------------------------------------------------------------------
class Obstacles:
    """Two different questions about the same four prims, both measured.

    The USD audit says ``/Root/Worker`` has no collider and the three robots
    have 5, 25 and 3. **Neither of those is the question.** What matters is
    what PhysX has in its scene at Play, which is not the same list: the
    robots' colliders belong to articulation links, and
    ``sensor_factory.pin_robots_static`` sets
    ``physxArticulation:articulationEnabled = False`` on every one of them to
    stop the legged robots collapsing. So this phase asks PhysX directly, with
    an overlap box over each prim's own footprint.

    The second question is the one the brief called bigger: if they have no
    colliders, is the lidar blind to them too? **This project's own design says
    no** -- ``sim/avatar.py``: "what ray-based sensors bounce off is RENDER
    geometry, not colliders -- RTX lidar and radar trace the same BVH the
    renderer does", which is why the avatar's visible body carries no collider
    at all and every sensor still sees it. That is a strong argument and not a
    measurement, so this phase counts lidar returns inside each prim's world
    bounding box and settles it.
    """

    WAIT_PAYLOAD = 90
    WAIT_PLAY = 90
    SAMPLES = 12

    def __init__(self, stage: Usd.Stage) -> None:
        import sensor_factory as sf

        self.stage = stage
        self.sf = sf
        self.report: dict = {"stage": STAGE, "physics": S.get("physics")}
        self.state = "referencing"
        self.counter = 0
        self.source = None
        self.lidar_id = None
        self.best: dict[str, int] = {}
        self.robots = sf.reference_robots(stage)
        self.targets = ["/Root/Worker"] + sorted(self.robots.values())
        log(f"referenced {len(self.robots)} robots")

    def _boxes(self) -> dict:
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        out = {}
        for t in self.targets:
            prim = self.stage.GetPrimAtPath(t)
            if not prim.IsValid():
                continue
            rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            lo, hi = rng.GetMin(), rng.GetMax()
            out[t] = (Gf.Vec3d(*lo), Gf.Vec3d(*hi))
        return out

    def probe_physics(self) -> dict:
        """What PhysX has inside each prim's own footprint, at Play."""
        import carb
        from omni.physx import get_physx_scene_query_interface

        query = get_physx_scene_query_interface()
        out = {}
        for t, (lo, hi) in self._boxes().items():
            centre = [(lo[i] + hi[i]) / 2.0 for i in range(3)]
            half = [max((hi[i] - lo[i]) / 2.0, 0.02) for i in range(3)]
            seen: list[str] = []
            own = {"n": 0}

            def report(hit, _t=t, _seen=seen, _own=own) -> bool:
                try:
                    path = str(hit.collision)
                except Exception:                                 # noqa: BLE001
                    path = "<unnamed>"
                if path.startswith(_t):
                    _own["n"] += 1
                if len(_seen) < 12:
                    _seen.append(path)
                return len(_seen) < 64

            try:
                n = query.overlap_box(
                    carb.Float3(*[float(v) for v in half]),
                    carb.Float3(*[float(v) for v in centre]),
                    carb.Float4(0.0, 0.0, 0.0, 1.0), report, False)
            except Exception as exc:                              # noqa: BLE001
                out[t] = {"error": repr(exc)}
                continue
            # SENTINEL. The ground plane is under every one of these boxes,
            # so a probe that does not see it is not reporting an empty scene,
            # it is reporting that PhysX has no scene attached -- which is what
            # a probe taken too soon after Stop/Play looks like, and it reads
            # identically to "the fix did not work".
            grounded = any("GroundPlane" in q for q in seen)
            # The nav collider lives in its own scope now, so "own" can no
            # longer be a path-prefix test on the target alone.
            own["n"] += sum(1 for q in seen if q.endswith(
                t.strip("/").replace("/", "_")))
            out[t] = {
                "overlap_hits": int(n), "own_colliders_in_scene": own["n"],
                "example_paths": seen, "saw_ground_plane": grounded,
                "verdict": ("PROBE INVALID -- no ground plane in the box, so "
                            "the physics scene is not attached yet"
                            if not grounded else
                            "PhysX HAS its collider -- the avatar should be "
                            "stopped by it" if own["n"] else
                            "PhysX has NOTHING of it -- the avatar walks "
                            "through, whatever USD says"),
            }
            log(f"  physics {t}: {out[t]['verdict']} "
                f"({own['n']} own of {int(n)} hits in the box)")
        return out

    def probe_lidar(self, arr) -> dict:
        out = {}
        for t, (lo, hi) in self._boxes().items():
            n = 0 if arr is None else self.sf.count_in_box(arr, lo, hi, pad=0.0)
            self.best[t] = max(self.best.get(t, 0), int(n))
            out[t] = {"lidar_points_in_bbox": self.best[t]}
        return out

    def tick(self, dt: float) -> bool:
        self.counter += 1

        if self.state == "referencing":
            if self.counter < self.WAIT_PAYLOAD:
                return False
            self.sf.pin_robots_static(self.stage, self.robots)
            # The fix is applied BEFORE Play, not around a Stop/Play in the
            # middle of the run. Measured 2026-09-02: stopping and replaying to
            # pick up new static colliders left `overlap_box` returning zero
            # hits for 720 frames -- not even /Root/GroundPlane/CollisionPlane,
            # which is underneath every probe box -- while the timeline played
            # normally. Scene queries do not survive that Stop/Play here, and
            # an empty overlap reads exactly like "the collider is not there".
            # The ground plane is the sentinel that tells the two apart.
            if WITH_FIX:
                self._apply_fix()
            self.report["bboxes_at_pin"] = {
                t: [round(float(v), 3) for v in (hi - lo)]
                for t, (lo, hi) in self._boxes().items()}
            log(f"  bounding boxes after pinning: {self.report['bboxes_at_pin']}")
            self._make_sensors()
            omni.timeline.get_timeline_interface().play()
            log("Play pressed; warming up before any probe (CLAUDE.md 10)")
            self.state, self.counter = "warming", 0
            return False

        if self.state == "warming":
            if self.counter < self.WAIT_PLAY:
                return False
            self.report["physics_probe"] = self.probe_physics()
            self.state, self.counter = "sampling", 0
            return False

        if self.state == "sampling":
            arr = self._sample()
            self.report["lidar_probe"] = self.probe_lidar(arr)
            self.report["lidar_points_total"] = int(0 if arr is None else len(arr))
            if self.counter < self.SAMPLES:
                return False
            for t, rec in self.report["lidar_probe"].items():
                log(f"  lidar {t}: {rec['lidar_points_in_bbox']} points in its bbox")
            self.report["neighbours"] = self.probe_neighbours()
            self.report["nav_check"] = self.check_nav()
            self.report["cct"] = self._cct()
            log(f"  CCT: {self.report['cct']}")
            return True

        return False

    def probe_neighbours(self) -> dict:
        """What else has a collider inside each pushable prop's own footprint.

        Aimed at one open question: a 1.2 kg crate takes 4.33 N.s and moves
        0.000-0.014 m, identically whether its collider is a convex hull or a
        16-part decomposition. A body that cannot move because something is
        holding it would look exactly like that, and nothing measured so far
        would have noticed.
        """
        import carb
        import pushable_props as pp
        from omni.physx import get_physx_scene_query_interface

        query = get_physx_scene_query_interface()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        out = {}
        for spec in pp.load_pushable_config()["props"]:
            path = spec["prim_path"]
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            lo, hi = rng.GetMin(), rng.GetMax()
            centre = [(lo[i] + hi[i]) / 2.0 for i in range(3)]
            # Grown by 3 cm so a neighbour resting against a face is found.
            half = [max((hi[i] - lo[i]) / 2.0 + 0.03, 0.03) for i in range(3)]
            others: list[str] = []

            def report(hit, _p=path, _o=others) -> bool:
                try:
                    q = str(hit.collision)
                except Exception:                                 # noqa: BLE001
                    q = "<unnamed>"
                if not q.startswith(_p) and len(_o) < 10:
                    _o.append(q)
                return len(_o) < 10

            try:
                query.overlap_box(
                    carb.Float3(*[float(v) for v in half]),
                    carb.Float3(*[float(v) for v in centre]),
                    carb.Float4(0.0, 0.0, 0.0, 1.0), report, False)
            except Exception as exc:                              # noqa: BLE001
                out[path] = {"error": repr(exc)}
                continue
            out[path] = {"mass_kg": spec.get("mass_kg"), "touching": others}
            log(f"  neighbours of {path.split('/')[-1]} ({spec.get('mass_kg')} kg): "
                f"{others or 'nothing but itself'}")
        return out

    def check_nav(self) -> dict:
        """Each authored nav collider, as USD sees it and as PhysX sees it.

        Because "the log says I created it" and "PhysX has it" are different
        claims, and on the first verification run they disagreed for exactly
        one of the four.
        """
        import nav_obstacles as no

        out = {}
        probe = self.report.get("physics_probe") or {}
        scope = no.load_config()["scope"]
        for t in self.targets:
            child = f"{scope}/{t.strip('/').replace('/', '_')}"
            prim = self.stage.GetPrimAtPath(child)
            rec = {"exists": bool(prim.IsValid())}
            if prim.IsValid():
                rec["type"] = str(prim.GetTypeName())
                rec["has_collision_api"] = bool(prim.HasAPI(UsdPhysics.CollisionAPI))
                rec["collision_enabled"] = UsdPhysics.CollisionAPI(
                    prim).GetCollisionEnabledAttr().Get()
                cache = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(),
                    [UsdGeom.Tokens.default_, UsdGeom.Tokens.guide,
                     UsdGeom.Tokens.proxy])
                rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                if not rng.IsEmpty():
                    lo, hi = rng.GetMin(), rng.GetMax()
                    rec["world_bbox"] = {
                        "min": [round(float(v), 3) for v in lo],
                        "max": [round(float(v), 3) for v in hi]}
            paths = (probe.get(t) or {}).get("example_paths") or []
            rec["physx_reported_it"] = child in paths
            out[child] = rec
            log(f"  nav check {child}: exists={rec['exists']} "
                f"enabled={rec.get('collision_enabled')} "
                f"physx_saw_it={rec['physx_reported_it']} "
                f"bbox={rec.get('world_bbox')}")
        return out

    def _cct(self) -> dict:
        import math

        from pxr import PhysxSchema

        prim = self.stage.GetPrimAtPath(BODY)
        api = PhysxSchema.PhysxCharacterControllerAPI(prim)
        out = {}
        for name, attr in (("step_offset", api.GetStepOffsetAttr()),
                           ("slope_limit", api.GetSlopeLimitAttr()),
                           ("climbing_mode", api.GetClimbingModeAttr()),
                           ("non_walkable_mode", api.GetNonWalkableModeAttr())):
            out[name] = attr.Get() if attr else None
        try:
            c = float(out["slope_limit"] or 0)
            out["slope_limit_deg"] = round(math.degrees(math.acos(c)), 1) if c > 0 else 0
        except Exception:                                         # noqa: BLE001
            pass
        return out

    def _apply_fix(self) -> None:
        """Author the nav colliders and the controller tuning, before Play.

        Same order the GUI uses, and after ``pin_robots_static`` for a measured
        reason: pinning LIFTS each robot by its own -z_min so its lowest point
        rests on the floor (the H1 by +1.044 m), and a collider sized from a
        bounding box taken before that lift is a metre underground.
        """
        import avatar as av
        import nav_obstacles as no

        self.report["nav"] = no.add_nav_obstacles(self.stage)
        self._tuning = av.install_controller_tuning(self.stage)
        log("fix applied before Play (nav colliders + controller tuning)")

    def _make_sensors(self) -> None:
        import observation_adapter as oa

        registry = self.sf.load_registry()
        # Stations first: the registry's sensors hang off them and an
        # unresolvable spec is skipped, so without this there is no lidar to
        # ask. Same order as sim/gui_viewports.py.
        self.sf.create_stations(self.stage)
        created = self.sf.create_registry_sensors(self.stage, registry)
        self.report["sensors"] = {k: v["kind"] for k, v in created.items()}
        try:
            self.source = oa.IsaacObservationSource(self.stage, registry, created)
            self.lidar_id = next((k for k, v in created.items()
                                  if v["kind"] == "lidar"), None)
            log(f"observation source up; lidar sensor is {self.lidar_id}")
        except Exception as exc:                                  # noqa: BLE001
            self.report["source_error"] = repr(exc)
            log(f"! could not build the observation source: {exc!r}")

    def _sample(self):
        import numpy as np

        if self.source is None or self.lidar_id is None:
            return None
        try:
            obs = {o.sensor_id: o for o in self.source.sample_now()}
        except Exception as exc:                                  # noqa: BLE001
            self.report.setdefault("sample_errors", []).append(repr(exc))
            return None
        lidar = obs.get(self.lidar_id)
        pts = None if lidar is None else lidar.data.get("points")
        if pts is None or not len(pts):
            return None
        return np.asarray(pts)

    def result(self) -> dict:
        self.report["loadavg"] = loadavg()
        self.report["reads_annotators"] = True
        self.report["note"] = (
            "the lidar count is what settles whether a prim with no collider is "
            "also invisible to sensors. Points are world metres straight out of "
            "sim/observation_adapter.py, so the spherical/sensor-local decode is "
            "the shipped one and not re-derived here.")
        return self.report


# ---------------------------------------------------------------------------
# Phase: drive -- which call makes the character controller actually walk?
# ---------------------------------------------------------------------------
class DriveBisect:
    """`set_move` is a silent no-op. Find the call that arms it.

    Three runs have now commanded 2.2 m per leg through
    ``get_physx_cct_interface().set_move()`` and measured **0.000 m** of
    capsule displacement, with no error, while ``set_position()`` on the same
    interface in the same frame moved the capsule exactly where it was told and
    let physics settle its z from 0.900 to 0.895. So the controller exists and
    is simulated; something else arms the move path.

    Candidates, tried in order, each for ``FRAMES`` frames with the
    displacement measured after:

      A  set_move with nothing else done            (the baseline no-op)
      B  after CharacterController.activate(), post-Play rather than pre-Play
      C  after activate_cct(path)                   (in the interface, and the
                                                     shipped class never calls it)
      D  after enable_worldspace_move(path)         (movement here is world-axis)
      E  set_position() walked by hand              (control: proves the ruler)

    Runs with PP_MASK=0 so it costs ~30 s rather than ~2 min: the collider mask
    changes nothing about whether a controller accepts a move command.
    """

    FRAMES = 70
    WARM = 40

    def __init__(self, stage: Usd.Stage) -> None:
        from omni.physxcct.scripts.ifaces import get_physx_cct_interface
        from omni.physxcct.scripts.utils import register_stage_update_node

        self.stage = stage
        self.cct = get_physx_cct_interface()
        self.cache = UsdGeom.XformCache()
        self.speed = float(avatar_cfg()["move_speed"])
        graph = stage.GetPrimAtPath(CONTROLS)
        if graph.IsValid():
            graph.SetActive(False)
            log(f"deactivated {CONTROLS}")
        self.arms = ["A_bare", "B_activate", "C_activate_cct",
                     "D_worldspace", "E_set_position"]
        self.i = 0
        self.counter = 0
        self.warming = True
        self.start = None
        self.results: dict = {}
        self.calls: dict = {}
        self.node = register_stage_update_node(
            "pushable_drive_bisect", on_update_fn=self._on_stage_update)

    def pos(self) -> Gf.Vec3d:
        self.cache.Clear()
        return self.cache.GetLocalToWorldTransform(
            self.stage.GetPrimAtPath(BODY)).ExtractTranslation()

    def _try(self, name: str, *arg_sets) -> str:
        fn = getattr(self.cct, name, None)
        if fn is None:
            return "absent"
        for args in arg_sets:
            try:
                fn(*args)
                return f"ok{list(args)!r}"
            except Exception as exc:                              # noqa: BLE001
                last = f"{type(exc).__name__}: {exc}"
        return f"failed ({last})"

    def _on_stage_update(self, _t, dt) -> None:
        """Pre-physics. +X in world, which is what W drives (sim/avatar.py)."""
        try:
            arm = self.arms[self.i]
            if self.warming or arm == "E_set_position" or dt <= 0.0:
                return
            d = self.speed * float(dt)
            self.cct.set_move(BODY, (d, 0.0, 0.0))
        except Exception as exc:                                  # noqa: BLE001
            log(f"! drive node failed: {exc!r}")

    def tick(self, dt: float) -> bool:
        arm = self.arms[self.i]
        self.counter += 1

        if self.warming:
            if self.counter == 1:
                sx, sy = (float(v) for v in avatar_cfg()["spawn_xy"])
                self.cct.set_position(BODY, (sx, sy, 0.90))
                self._arm(arm)
            if self.counter >= self.WARM:
                self.warming, self.counter = False, 0
                self.start = self.pos()
            return False

        if arm == "E_set_position":
            p = self.pos()
            self.cct.set_position(BODY, (p[0] + self.speed * dt, p[1], p[2]))

        if self.counter < self.FRAMES:
            return False

        end = self.pos()
        moved = ((end[0] - self.start[0]) ** 2 + (end[1] - self.start[1]) ** 2) ** 0.5
        self.results[arm] = round(moved, 4)
        log(f"  {arm}: capsule moved {moved:.4f} m in {self.FRAMES} frames "
            f"(calls: {self.calls.get(arm)})")
        self.i += 1
        self.counter, self.warming = 0, True
        return self.i >= len(self.arms)

    def _arm(self, arm: str) -> None:
        """Apply this arm's extra call, cumulatively: each arm keeps the last."""
        if arm == "B_activate":
            from omni.physxcct.scripts.utils import CharacterController

            handle = CharacterController(BODY, None, True, 0.2)
            handle.activate(self.stage)
            self._handle = handle
            self.calls[arm] = "CharacterController.activate() after Play"
        elif arm == "C_activate_cct":
            self.calls[arm] = "activate_cct -> " + self._try(
                "activate_cct", (BODY,), (BODY, True))
        elif arm == "D_worldspace":
            self.calls[arm] = "enable_worldspace_move -> " + self._try(
                "enable_worldspace_move", (BODY,), (BODY, True))
        else:
            self.calls[arm] = "none"
        log(f"arm {arm}: {self.calls[arm]}")

    def report(self) -> dict:
        works = [a for a, v in self.results.items() if v > 0.2]
        return {
            "stage": STAGE, "question": "which call arms omni.physx.cct set_move?",
            "physics": S.get("physics"), "loadavg": loadavg(),
            "speed_ms": self.speed, "frames_per_arm": self.FRAMES,
            "moved_m": self.results, "calls": self.calls,
            "arms_that_moved": works,
            "verdict": (
                "set_move never moved the capsule under any arming; the only "
                f"arm that moved it is {works[0]}, which does not use set_move "
                "at all -- so the headless driver must be a scripted "
                "set_position walk, which does NOT collide and slide"
                if works == ["E_set_position"] else
                f"set_move is armed by: {works[0]}" if works else
                "NO arm moved the capsule, not even the set_position control -- "
                "the measurement itself is suspect"),
        }


# ---------------------------------------------------------------------------
# Phase: push
# ---------------------------------------------------------------------------
S: dict = {"frame": 0, "sub": None, "state": "loading", "t": None,
           "times": defaultdict(list), "report": {}, "legs": [], "shots": []}


def finish(name: str, payload) -> None:
    write_json(name, payload)
    log("DONE")
    S["sub"] = None
    omni.kit.app.get_app().post_quit()


class PushRun:
    """Four legs: light prop, medium prop, the heavy drum, and a rack shelf.

    The shelf is the negative control and it is not decoration -- it is the
    only leg that can distinguish "the push works" from "the floor is
    frictionless and everything drifts". It also measures the half of the gate
    that is about the AVATAR: walking into a shelf must stop you, so the leg
    records how far the capsule itself got.
    """

    WARM = 60
    SETTLE = 25
    REST = 120           # frames to let a shoved prop finish sliding
    WALK_M = 2.20        # metres of commanded walking per leg
    WALK_MAX_FRAMES = 600
    STAND_OFF = 1.45     # where the leg starts, metres from the target centre
    # STOP THE WALK JUST SHORT OF TOUCHING, and this is the whole design of the
    # leg rather than a detail. MEASURED 2026-09-01, walking all the way in:
    #
    #     prop            callback ON    callback OFF
    #     crate  1.2 kg      0.014 m        0.175 m
    #     carton 4.0 kg      0.676 m        0.496 m
    #     drum  60.0 kg      0.396 m        0.447 m
    #
    # -- the props move about as much either way, and the 60 kg drum, which the
    # model says must not move at all, moves 0.4 m in BOTH arms. That is not
    # the impulse. A `set_position` walk is a placement per frame, so the
    # capsule's kinematic actor teleports INTO the prop and PhysX resolves the
    # overlap by shoving it; kinematic wins every such contact regardless of
    # mass. The depenetration swamps the term being measured.
    #
    # Stopping at STOP_GAP removes it: the capsule never overlaps, so nothing
    # but the callback can move anything, and the control arm should read
    # exactly zero. The sweep still reaches -- it looks ahead
    # radius + speed*dt + skin, about 0.08 m past the capsule surface.
    STOP_GAP = 0.02
    #: Sweep hits on the target before the leg stops walking. The primary stop
    #: condition; the gap below is only a backstop for a prop the callback
    #: never sees, which is itself a result worth recording.
    TOUCHES_TO_STOP = 6
    #: Backstop. Deliberately BELOW zero clearance, because the bbox gap
    #: overestimates the clearance of a tapered prop and stopping on it is what
    #: produced a run of untouched props reported as immovable ones.
    STOP_GAP_HARD = -0.05

    def __init__(self, stage: Usd.Stage) -> None:
        import pushable_props as pp

        self.stage = stage
        self.cfg = pp.load_pushable_config()
        if DETECTOR:
            self.cfg["detector"] = DETECTOR
        if APPROX:
            for spec in self.cfg["props"]:
                spec["approximation"] = APPROX
            log(f"PP_APPROX={APPROX} -- every prop forced to this approximation")
        self.converted = pp.make_pushable(stage, self.cfg)
        self.bodies = list((self.converted.get("made") or {}).keys())
        self.driver = Driver(stage, float(avatar_cfg()["move_speed"]))
        self.cb = None
        self.nav: dict = {}
        self.follow = None
        self.robots: dict = {}
        if WITH_FIX:
            import sensor_factory as sf

            # Referenced here, PINNED LATER. `pin_robots_static`'s own docstring
            # says to call it after the payloads have settled -- "0 bodies at
            # reference, 17 once loaded" -- and pinning in the same breath cost
            # this run BOT_02 entirely: it reported 0 enabled colliders, was
            # silenced of nothing, got a proxy sized from an unloaded bbox and
            # recorded 0 touches. The other two happened to have composed.
            self.robots = sf.reference_robots(stage)
        if WITH_PUSH and not WITH_FIX:
            self.cb = pp.install_push_callback(stage, self.converted, self.cfg)
        else:
            log("PP_PUSH=0 -- props are dynamic, the hit callback is NOT installed")

        # Legs, named by what they are supposed to show.
        made = self.converted.get("made") or {}
        light = min(made, key=lambda p: made[p]["mass_kg"]) if made else None
        heavy = max(made, key=lambda p: made[p]["mass_kg"]) if made else None
        mid = None
        for p, rec in sorted(made.items(), key=lambda kv: kv[1]["mass_kg"]):
            if 3.0 <= rec["mass_kg"] <= 10.0:
                mid = p
                break
        # The rack leg is SELECTIVITY, not blocking. A set_position walk cannot
        # test "the shelf stops you" (see Driver). What it can test, and what
        # matters just as much for the gate, is that the callback pushes only
        # what is on the list: a rack frame is not a declared prop, so
        # `_pushable_root` returns None for it and no impulse is applied to
        # anything on this leg.
        shelf = self._a_shelf()
        # (name, prim whose motion is measured, prim the callback counts hits
        # on). For a prop those are the same prim. For a robot they are not:
        # the physics is the proxy and the thing you watch move is the robot,
        # which the follow writes from it.
        if ALL_LEGS:
            # Config order, so the log reads in the same order as the table a
            # human is checking it against.
            order = [s["prim_path"] for s in self.cfg["props"]
                     if s["prim_path"] in made]
            self.plan = [(f"{made[p]['mass_kg']:g}kg_{p.split('/')[-1]}", p, p)
                         for p in order]
            # Robot legs are appended by _extend_plan once their proxies
            # exist -- see _author_robot_physics.
            if shelf:
                self.plan.append(("rack_selectivity", shelf, shelf))
        else:
            self.plan = [(n, p, p) for n, p in
                         [("light", light), ("mid", mid), ("heavy", heavy),
                          ("rack_selectivity", shelf)]
                         if p]
        self.leg_i = 0
        self.stage_name = "payload" if WITH_FIX else "warm"
        self.counter = 0
        self.leg: dict = {}
        self.rgb = None
        if WITH_SHOTS:
            self._make_render_product()
        else:
            log("PP_SHOOT unset -- no render product, no annotator read "
                "(CLAUDE.md 11: this phase reports no frame times either)")

    def _a_shelf(self) -> str | None:
        """A rack the avatar can walk into, read off the stage, not invented."""
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        sx, sy = (float(v) for v in avatar_cfg()["spawn_xy"])
        best, best_d = None, 1e9
        wh = self.stage.GetPrimAtPath("/Root/Warehouse")
        for child in wh.GetChildren():
            name = child.GetName()
            if not name.startswith(("SM_RackFrame", "SM_RackShelf", "SM_RackPile")):
                continue
            rng = cache.ComputeWorldBound(child).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            lo, hi = rng.GetMin(), rng.GetMax()
            if float(lo[2]) > 0.30:            # must reach the floor to walk into
                continue
            cx, cy = (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0
            d = ((cx - sx) ** 2 + (cy - sy) ** 2) ** 0.5
            if d < best_d:
                best, best_d = child.GetPath().pathString, d
        if best:
            log(f"negative control: nearest floor-reaching rack is {best} ({best_d:.2f} m)")
        return best

    def _make_render_product(self) -> None:
        """One render product on the avatar's own third-person camera.

        THIS PHASE READS ANNOTATORS and says so (CLAUDE.md 11). It reports no
        frame times; PP_PHASE=fps does, and creates no render product at all.
        """
        try:
            import omni.replicator.core as rep

            rp = rep.create.render_product(TP_CAM, resolution=RES)
            self.rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            self.rgb.attach([rp])
            log(f"render product on {TP_CAM} at {RES[0]}x{RES[1]} (annotator READ every leg)")
        except Exception as exc:
            log(f"! no render product ({exc!r}) -- the run still measures displacement")

    def shoot(self, tag: str) -> None:
        if self.rgb is None:
            return
        try:
            import numpy as np
            from PIL import Image

            arr = np.asarray(self.rgb.get_data())
            if arr.size == 0:
                log(f"  no pixels yet for {tag}")
                return
            path = OUT / f"pushable_{tag}.png"
            Image.fromarray(arr[:, :, :3].copy()).save(str(path))
            S["shots"].append(str(path))
            log(f"  wrote {path}")
        except Exception as exc:
            log(f"  ! could not shoot {tag}: {exc!r}")

    def snapshot(self) -> dict:
        """Where every pushable is, three ways, plus this leg's target.

        The target is included even when it is not pushable, because the
        negative control's whole claim is "the rack did not move" and a `None`
        is not that claim -- it is the absence of one.

        THREE numbers per prop, not one, and the reason is measured: the 1.2 kg
        crate's Xform ORIGIN moved 0.014 m on a leg where its bounding box
        moved 0.13 m. A body that tips rather than slides rotates about a point
        near its own origin, so origin displacement reads as "nothing
        happened". Recording the bbox centre and the rotation angle beside it
        makes tipping and sliding distinguishable instead of both being small
        numbers.
        """
        cache = UsdGeom.XformCache()
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        watch = list(self.bodies)
        tgt = (self.leg or {}).get("target")
        if tgt and tgt not in watch:
            watch.append(tgt)
        out = {}
        for p in watch:
            prim = self.stage.GetPrimAtPath(p)
            if not prim.IsValid():
                continue
            m = cache.GetLocalToWorldTransform(prim)
            o = m.ExtractTranslation()
            rec = {"origin": [float(o[0]), float(o[1]), float(o[2])]}
            try:
                rng = bb.ComputeWorldBound(prim).ComputeAlignedRange()
                if not rng.IsEmpty():
                    lo, hi = rng.GetMin(), rng.GetMax()
                    rec["center"] = [float((lo[i] + hi[i]) / 2.0) for i in range(3)]
            except Exception:
                pass
            try:
                q = m.ExtractRotationQuat()
                rec["quat"] = [float(q.GetReal()), *[float(v) for v in q.GetImaginary()]]
            except Exception:
                pass
            out[p] = rec
        return out

    # -- the state machine -------------------------------------------------
    WAIT_PAYLOAD = 90

    def _author_robot_physics(self) -> None:
        """Pin, proxy, follow and (re)install the push callback, in that order.

        Every step depends on the one before it: the bounding box a proxy is
        sized from is only meaningful once the robot has been dropped onto the
        floor, the follow's reference pose is only meaningful once the proxy
        exists, and the push callback needs the proxies in its body list.
        """
        import nav_obstacles as no
        import pushable_props as pp
        import sensor_factory as sf

        sf.pin_robots_static(self.stage, self.robots)
        self.nav = no.add_nav_obstacles(self.stage)
        self.follow = no.install_proxy_follow(self.stage, self.nav)
        extra = no.dynamic_bodies(self.nav)
        self.bodies += list(extra)
        if WITH_PUSH:
            self.cb = pp.install_push_callback(
                self.stage, self.converted, self.cfg, extra_bodies=extra)
        self._extend_plan()
        # Play only NOW. Pressing it earlier gives a legged robot with its
        # articulation still enabled a second to collapse, and every bounding
        # box taken afterwards -- including the one a proxy is sized from -- is
        # taken from a heap. It cost this run an H1 proxy 0.70 m tall for a
        # 1.81 m robot, twice.
        omni.timeline.get_timeline_interface().play()
        log("robot physics authored; playing")

    def _extend_plan(self) -> None:
        if not ALL_LEGS:
            return
        shelf = [t for t in self.plan if t[0] == "rack_selectivity"]
        self.plan = [t for t in self.plan if t[0] != "rack_selectivity"]
        for path, rec in sorted((self.nav.get("made") or {}).items()):
            if rec.get("dynamic"):
                self.plan.append((f"{rec['mass_kg']:g}kg_{path.split('/')[-1]}",
                                  path, rec["collider"]))
        self.plan += shelf

    def tick(self, dt: float) -> bool:
        """Returns True when the whole run is finished."""
        self.counter += 1

        if self.stage_name == "payload":
            if self.counter < self.WAIT_PAYLOAD:
                return False
            self._author_robot_physics()
            self.stage_name, self.counter = "warm", 0
            return False

        if self.stage_name == "warm":
            if self.counter >= self.WARM:
                self._begin_leg()
            return False

        if self.stage_name == "settle":
            self.driver.halt()
            if self.counter >= self.SETTLE:
                self.leg["start"] = self.snapshot()
                self.leg["capsule_start"] = list(self.driver.pos())
                self.driver.commanded_m = 0.0
                self.leg["touches_at_start"] = int(
                    self.cb.hits_by_root.get(self.leg["hit_key"], 0)) if self.cb else 0
                self.shoot(f"{self.leg['name']}_before")
                if self.cb:
                    self.leg["stats_at_start"] = dict(self.cb.stats)
                self.stage_name, self.counter = "walk", 0
            return False

        if self.stage_name == "walk":
            self.driver.walk_to(self.leg["target_xy"])
            self.leg["commanded_m"] = self.driver.commanded_m
            self.leg["walk_frames"] = self.counter
            gap = self._gap()
            self.leg["gap_m"] = round(gap, 4)
            touches = self._touches()
            self.leg["touches"] = touches
            # STOP ON CONTACT THE CALLBACK CAN SEE, not on a bounding-box gap.
            # MEASURED 2026-09-03, and it invalidated the previous run: the gap
            # metric is circle-to-circle on the target's bbox half-extent, and
            # for a TAPERED prop that half-extent is the widest part. A traffic
            # cone's bbox radius is 0.17 m at the base and its radius at the
            # capsule's contact height is nearer 0.05, so the walk halted about
            # 0.12 m too early and the sweep never reached. Result: one of
            # three identical 2 kg cones and both 1.5 kg wet-floor signs
            # recorded ZERO impulses, and their zero displacement was read as
            # the props refusing to move. They were never touched.
            #
            # Stopping when the push callback has actually hit the target N
            # times uses the same envelope that does the pushing, so "no
            # movement" can only mean "pushed and did not move".
            enough = touches >= self.TOUCHES_TO_STOP
            if (enough
                    or gap <= self.STOP_GAP_HARD
                    or self.driver.commanded_m >= self.WALK_M
                    or self.counter >= self.WALK_MAX_FRAMES):
                self.driver.halt()
                self.leg["stopped_on"] = (
                    "touches" if enough
                    else "gap" if gap <= self.STOP_GAP_HARD
                    else "distance" if self.driver.commanded_m >= self.WALK_M
                    else "frames")
                self.stage_name, self.counter = "rest", 0
            return False

        if self.stage_name == "rest":
            self.driver.halt()
            if self.counter >= self.REST:
                self._end_leg()
                self.leg_i += 1
                if self.leg_i >= len(self.plan):
                    return True
                self._begin_leg()
            return False

        return False

    def _touches(self) -> int:
        """Sweep hits the push callback has recorded on this leg's target."""
        if not self.cb:
            return 0
        return int(self.cb.hits_by_root.get(self.leg["hit_key"], 0)
                   - self.leg.get("touches_at_start", 0))

    def _gap(self) -> float:
        """Clearance between the capsule's surface and the target's footprint.

        Approximated as circle-to-circle: the target's half-extent is measured
        once at leg start (the body is rigid; only its centre moves) and its
        centre is read live, because a prop that has just been pushed is not
        where it was.

        The centre is the BOUNDING-BOX centre, recomputed, NOT the prim's
        ``xformOp:translate``. They are not the same point and using the origin
        was wrong by 0.6 m on `SM_CratePlasticNote_B_03_18`, whose mesh sits
        well off its Xform: the leg then stopped somewhere else in each arm and
        the two arms stopped comparing like with like. `target_xy` was already
        the bbox centre, so the walk and the stop condition disagreed about
        where the prop was.
        """
        c, _ = bbox_center_half(self.stage, self.leg["target"])
        if c is None:
            return 1e9
        p = self.driver.pos()
        d = ((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2) ** 0.5
        return d - self.leg["target_half_extent_m"] - self.driver.radius

    def _begin_leg(self) -> None:
        name, path, hit_key = self.plan[self.leg_i]
        c, half = bbox_center_half(self.stage, path)
        sx, sy = (float(v) for v in avatar_cfg()["spawn_xy"])
        # Stand clear of the target's own footprint. A fixed 1.45 m is fine for
        # a 0.34 m cone and puts the capsule INSIDE a 2.4 m rack frame, where
        # set_position does not depenetrate and the leg then reports "blocked"
        # for the wrong reason.
        stand_off = max(self.STAND_OFF, half + 0.95)
        # Approach along the line from the spawn point, so the avatar walks in
        # from open floor rather than out of a rack.
        dx, dy = float(c[0]) - sx, float(c[1]) - sy
        n = (dx * dx + dy * dy) ** 0.5 or 1.0
        start = (float(c[0]) - dx / n * stand_off, float(c[1]) - dy / n * stand_off)
        made = self.converted.get("made") or {}
        self.leg = {
            "name": name,
            "target": path,
            "hit_key": hit_key,
            "mass_kg": made.get(path, {}).get("mass_kg"),
            "target_xy": [float(c[0]), float(c[1])],
            "stand_off_m": round(stand_off, 3),
            "target_half_extent_m": round(half, 3),
            "commanded_m": 0.0,
            "walk_frames": 0,
        }
        self.driver.teleport(start[0], start[1], 0.90)
        log(f"leg {self.leg_i + 1}/{len(self.plan)}  {name}  -> {path} "
            f"({self.leg['mass_kg']} kg)  from ({start[0]:.2f}, {start[1]:.2f})")
        self.stage_name, self.counter = "settle", 0

    def _end_leg(self) -> None:
        end = self.snapshot()
        start = self.leg["start"]
        moved, moved_center, turned = {}, {}, {}
        for p, a in start.items():
            b = end.get(p)
            if not b:
                continue
            moved[p] = round(_dist(a.get("origin"), b.get("origin")), 4)
            if a.get("center") and b.get("center"):
                moved_center[p] = round(_dist(a["center"], b["center"]), 4)
            if a.get("quat") and b.get("quat"):
                turned[p] = round(_quat_angle_deg(a["quat"], b["quat"]), 2)
        cap_end = list(self.driver.pos())
        cap_start = self.leg["capsule_start"]
        cap_d = ((cap_end[0] - cap_start[0]) ** 2 + (cap_end[1] - cap_start[1]) ** 2) ** 0.5
        self.leg["moved_m"] = moved
        self.leg["moved_center_m"] = moved_center
        self.leg["turned_deg"] = turned
        self.leg["target_moved_m"] = moved.get(self.leg["target"])
        self.leg["target_moved_center_m"] = moved_center.get(self.leg["target"])
        self.leg["target_turned_deg"] = turned.get(self.leg["target"])
        self.leg["capsule_travelled_m"] = round(cap_d, 3)
        self.leg["commanded_m"] = round(self.leg["commanded_m"], 3)
        self.leg["capsule_blocked"] = bool(
            self.leg.get("stopped_on") not in ("gap", "touches")
            and cap_d < 0.6 * self.leg["commanded_m"])
        if self.cb:
            a = self.leg.pop("stats_at_start", {})
            self.leg["push"] = {k: round(v - a.get(k, 0), 4) if isinstance(v, float)
                                else v - a.get(k, 0) for k, v in self.cb.stats.items()}
        self.shoot(f"{self.leg['name']}_after")
        self.leg.pop("start", None)
        S["legs"].append(self.leg)
        log(f"  {self.leg['name']}: target moved {self.leg['target_moved_m']} m "
            f"(bbox centre {self.leg['target_moved_center_m']} m, turned "
            f"{self.leg['target_turned_deg']} deg), "
            f"capsule travelled {self.leg['capsule_travelled_m']} m of "
            f"{self.leg['commanded_m']} commanded, stopped on "
            f"{self.leg.get('stopped_on')} after {self.leg.get('touches')} "
            f"touches, gap {self.leg.get('gap_m')} m"
            f"{' (BLOCKED)' if self.leg['capsule_blocked'] else ''}")
        if cap_d < 1e-3 and self.leg["commanded_m"] > 0.1:  # pragma: no cover
            # EXACTLY zero is not "blocked", it is "not driven". A capsule that
            # walks into a shelf still slides a centimetre; one that never got
            # the command does not move at all. Said loudly because every other
            # number in this leg reads as a valid result.
            self.leg["driver_suspect"] = True
            log("  ! capsule displacement is EXACTLY zero -- suspect set_move "
                "is not reaching the controller, not that the avatar was stopped")
        if self.cb:
            log(f"  push stats this leg: {self.leg.get('push')}")
        write_json("pushable_push.json", self._report())

    def _report(self) -> dict:
        return {
            "stage": STAGE,
            "with_push_callback": bool(self.cb),
            "detector": self.cfg.get("detector"),
            "approximation_override": APPROX,
            "collider_mask": WITH_MASK,
            "loadavg": loadavg(),
            "physics": S.get("physics"),
            "converted": self.converted,
            "nav": self.nav,
            "proxy_follow": bool(self.follow),
            "avatar_mass_kg": self.cfg["avatar_mass_kg"],
            "max_push_force_n": self.cfg["max_push_force_n"],
            "stall_mass_kg": self.converted.get("stall_mass_kg"),
            "legs": S["legs"],
            "push_stats_total": dict(self.cb.stats) if self.cb else None,
            "push_notes": list(self.cb.log_lines) if self.cb else None,
            "shots": S["shots"],
            "reads_annotators": self.rgb is not None,
        }


# ---------------------------------------------------------------------------
# Phase: fps
# ---------------------------------------------------------------------------
class FpsRun:
    """Five arms, one process, and NOT ONE OF THEM READS AN ANNOTATOR.

    That sentence is the point of the class (CLAUDE.md failure mode 11). No
    render product is created here at all, so no arm can accidentally acquire a
    16 ms readback the others do not have. The absolute numbers are not
    portable -- this is a shared box and PhysX is CPU-bound -- so the deltas
    are the result and the load average is reported beside them.

        A_static_idle   props as shipped, avatar standing
        A_static_walk   props as shipped, avatar walking the cluster
        B_dyn_idle      props dynamic, no callback, avatar standing
        C_dyn_walk      props dynamic, no callback, avatar walking
        D_push_walk     props dynamic + hit callback, avatar walking

    The conversion happens at Stop between B and A so that both halves run
    under the same process and the same neighbours' CPU load. A_* and C_*/D_*
    walk the same path from the same start, because a walk that ends somewhere
    else is a different measurement.
    """

    WARM, MEASURE = 40, 240

    def __init__(self, stage: Usd.Stage) -> None:
        import avatar as av
        import pushable_props as pp

        self.stage = stage
        self.pp = pp
        self.cfg = pp.load_pushable_config()
        if DETECTOR:
            self.cfg["detector"] = DETECTOR
        self.driver = Driver(stage, float(avatar_cfg()["move_speed"]))
        self.converted: dict = {}
        self.cb = None
        self.nav = None if WITH_FIX else {}
        self.follow = None
        self.robots: dict = {}
        self.prep = 0
        if WITH_FIX:
            import sensor_factory as sf

            # Referenced now, pinned after the payloads compose -- see
            # PushRun._author_robot_physics for what pinning too early costs.
            self.robots = sf.reference_robots(stage)
            self._tuning = av.install_controller_tuning(stage)
        self.arms = ["A_static_idle", "A_static_walk", "B_dyn_idle", "C_dyn_walk", "D_push_walk"]
        self.i = 0
        self.counter = 0
        self.warming = True
        self.last_t = None
        # Walk toward the centroid of the declared props: that is where the new
        # solver work is, so it is the walk that can cost anything.
        cs = [bbox_center(stage, s["prim_path"]) for s in self.cfg["props"]]
        cs = [c for c in cs if c is not None]
        self.cluster = (sum(c[0] for c in cs) / len(cs), sum(c[1] for c in cs) / len(cs)) if cs else None
        sx, sy = (float(v) for v in avatar_cfg()["spawn_xy"])
        self.start_xy = (sx, sy)
        log(f"walk arms head for the prop centroid {self.cluster} from {self.start_xy}")

    def _convert(self) -> None:
        omni.timeline.get_timeline_interface().stop()
        self.converted = self.pp.make_pushable(self.stage, self.cfg)
        omni.timeline.get_timeline_interface().play()
        log("props converted at Stop; timeline restarted")

    def tick(self, dt: float) -> bool:
        if WITH_FIX and self.nav is None:
            self.prep += 1
            if self.prep < 90:
                return False
            import nav_obstacles as no
            import sensor_factory as sf

            sf.pin_robots_static(self.stage, self.robots)
            self.nav = no.add_nav_obstacles(self.stage)
            self.follow = no.install_proxy_follow(self.stage, self.nav)
            omni.timeline.get_timeline_interface().play()
            log("robot physics authored; playing, and the fps arms start here")
            return False

        arm = self.arms[self.i]
        self.counter += 1

        if self.warming:
            if self.counter == 1:
                self.driver.teleport(self.start_xy[0], self.start_xy[1], 0.90)
            if arm.endswith("walk") and self.cluster:
                self.driver.walk_to(self.cluster)
            else:
                self.driver.halt()
            if self.counter >= self.WARM:
                self.warming, self.counter, self.last_t = False, 0, time.perf_counter()
            return False

        if arm.endswith("walk") and self.cluster:
            self.driver.walk_to(self.cluster)
        else:
            self.driver.halt()

        now = time.perf_counter()
        if self.last_t is not None:
            S["times"][arm].append(now - self.last_t)
        self.last_t = now

        if self.counter < self.MEASURE:
            return False

        log(f"  {arm}: {self.frame_ms(arm)} ms/frame ({self.fps(arm)} fps), load {loadavg()}")
        S.setdefault("loads", []).append(loadavg())
        self.i += 1
        self.counter, self.warming, self.last_t = 0, True, None
        if self.i >= len(self.arms):
            return True
        nxt = self.arms[self.i]
        if nxt == "B_dyn_idle":
            self._convert()
        if nxt == "D_push_walk" and self.cb is None:
            # Stop/Play FIRST. Isaac restores rigid bodies to their authored
            # poses on Stop, and without that D would start from wherever the
            # C arm left the props -- a different scene, measured as if it were
            # the same one.
            tl = omni.timeline.get_timeline_interface()
            tl.stop()
            tl.play()
            log("scene reset (Stop/Play) before the push arm")
            self.cb = self.pp.install_push_callback(self.stage, self.converted, self.cfg)
        return False

    @staticmethod
    def _trimmed(v):
        if len(v) < 20:
            return None
        v = sorted(v)[len(v) // 10: len(v) - len(v) // 10]
        return sum(v) / len(v)

    def frame_ms(self, arm):
        m = self._trimmed(S["times"].get(arm) or [])
        return round(m * 1000.0, 2) if m else None

    def fps(self, arm):
        m = self._trimmed(S["times"].get(arm) or [])
        return round(1.0 / m, 2) if m else None

    def report(self) -> dict:
        arms = {a: {"frame_ms": self.frame_ms(a), "fps": self.fps(a),
                    "n": len(S["times"].get(a) or [])} for a in self.arms}
        def d(x, y):
            a, b = arms[x]["frame_ms"], arms[y]["frame_ms"]
            return round(b - a, 2) if (a and b) else None
        # INTERNAL CONSISTENCY, because absolute numbers on a shared box are
        # not portable and a contaminated run reads exactly like a result.
        # Each pair below is (more work, less work); the first cannot be
        # faster than the second unless something outside the run moved.
        checks, bad = {}, []
        for more, less in (("B_dyn_idle", "A_static_idle"),
                           ("C_dyn_walk", "A_static_walk"),
                           ("D_push_walk", "C_dyn_walk")):
            a, b = arms[more]["frame_ms"], arms[less]["frame_ms"]
            ok = None if not (a and b) else a >= b - 0.5
            checks[f"{more} >= {less}"] = ok
            if ok is False:
                bad.append(f"{more} ({a} ms) faster than {less} ({b} ms)")
        return {
            "stage": STAGE,
            "reads_annotators": False,
            "internally_consistent": not bad,
            "consistency_checks": checks,
            "consistency_failures": bad,
            "note": "no render product is created in this phase; every arm "
                    "samples the same way (CLAUDE.md failure mode 11)",
            "collider_mask": WITH_MASK,
            "loadavg": loadavg(),
            "physics": S.get("physics"),
            "n_props": len(self.converted.get("made") or {}),
            "nav_obstacles": len((self.nav or {}).get("made") or {}),
            "with_fix": WITH_FIX,
            "detector": self.cfg.get("detector"),
            "arms": arms,
            "deltas_ms": {
                "dynamic_bodies_idle": d("A_static_idle", "B_dyn_idle"),
                "dynamic_bodies_walking": d("A_static_walk", "C_dyn_walk"),
                "hit_callback_walking": d("C_dyn_walk", "D_push_walk"),
                "walking_vs_standing_static": d("A_static_idle", "A_static_walk"),
            },
            "push_stats_total": dict(self.cb.stats) if self.cb else None,
        }


# ---------------------------------------------------------------------------
# Frame driver
# ---------------------------------------------------------------------------
def _frame_dt(e) -> float:
    """Seconds since the previous update event.

    NOT ``e.payload.get("dt")``. The update event's payload is a carb binding,
    not a dict: ``.get`` does not exist on it, and an AttributeError raised
    inside a subscription callback is caught and LOGGED by Kit rather than
    propagating -- so the run printed one traceback per frame for three
    thousand frames, never advanced past its warm-up, and never wrote a result.
    Nothing in the script's own error handling fired, because the exception
    happened before the try block it was supposed to land in. Measured
    2026-09-01. ``omni.physxcct`` reads ``e.payload["dt"]`` by subscript; the
    wall clock is used here instead so this works under any event shape.

    Clamped, because a stalled frame must not turn into a 2 m teleport.
    """
    now = time.perf_counter()
    prev = S.get("t_prev")
    S["t_prev"] = now
    if prev is None:
        return 1.0 / 60.0
    return min(max(now - prev, 1.0 / 240.0), 1.0 / 10.0)


def on_update(e) -> None:
    S["frame"] += 1
    ctx = omni.usd.get_context()
    if S["state"] == "loading":
        if S["frame"] <= 5 or any(ctx.get_stage_loading_status()[1:]):
            return
        stage = ctx.get_stage()
        root = stage.GetDefaultPrim()
        log(f"stage root {root.GetPath() if root else '<none>'} "
            f"({len(list(stage.Traverse()))} prims), load {loadavg()}")
        try:
            setup(stage)
        except Exception as exc:
            import traceback
            log("FAILED: " + repr(exc))
            log(traceback.format_exc())
            finish("pushable_error.json", {"phase": PHASE, "error": repr(exc),
                                           "tb": traceback.format_exc()})
        return

    if S["state"] != "running":
        return
    dt = _frame_dt(e)
    try:
        if S["run"].tick(dt):
            S["state"] = "done"
            if PHASE == "push":
                finish("pushable_push.json", S["run"]._report())
            elif PHASE == "drive":
                finish("pushable_drive.json", S["run"].report())
            elif PHASE == "audit":
                finish("pushable_audit.json", S["run"].result())
            elif PHASE == "obstacles":
                finish("pushable_obstacles.json", S["run"].result())
            else:
                finish("pushable_fps.json", S["run"].report())
    except Exception as exc:
        import traceback
        log("FAILED mid-run: " + repr(exc))
        log(traceback.format_exc())
        finish("pushable_error.json", {"phase": PHASE, "error": repr(exc),
                                       "tb": traceback.format_exc(),
                                       "legs": S["legs"]})


def setup(stage: Usd.Stage) -> None:
    if PHASE == "enumerate":
        finish("pushable_candidates.json", enumerate_candidates(stage))
        return

    import avatar as av
    import sensor_factory as sf

    S["physics"] = physics_facts(stage)
    log(f"physics: {S['physics']}")

    if WITH_MASK:
        sf.disable_unreachable_colliders(stage)
    else:
        log("collider mask SKIPPED (PP_MASK=0)")
    # The visible character follows the capsule from Python; without it the PNGs
    # show a man standing still while the capsule walks off (sim/avatar.py).
    S["follow"] = av.install_character_follow(stage)

    if PHASE == "push":
        S["run"] = PushRun(stage)
    elif PHASE == "fps":
        S["run"] = FpsRun(stage)
    elif PHASE == "drive":
        S["run"] = DriveBisect(stage)
    elif PHASE == "audit":
        S["run"] = Audit(stage)
    elif PHASE == "obstacles":
        S["run"] = Obstacles(stage)
    else:
        finish("pushable_error.json", {"error": f"unknown PP_PHASE={PHASE!r}"})
        return

    # CLAUDE.md failure mode 10: with the timeline stopped, nothing simulates
    # and nothing captures. Press Play, warm up, THEN sample.
    #
    # EXCEPT for the phases that own their own timeline, and that exception is
    # measured, not tidiness. `audit` and `obstacles` reference the robots and
    # must pin them BEFORE anything simulates: a legged robot with its
    # articulation still enabled and no locomotion policy collapses within a
    # second of Play, and every number taken from it afterwards is taken from a
    # heap. It cost this session a bounding box -- the H1 measured
    # 1.569 x 1.572 x 0.532 m spanning z -0.240..0.291, i.e. lying flat and
    # half under the floor, and a nav collider sized from it was 0.70 m tall
    # for a 1.8 m robot. sim/gui_viewports.py never had the bug because a human
    # presses Play, long after pin_robots_static has run.
    if PHASE in ("audit", "obstacles") or WITH_FIX:
        log(f"phase {PHASE} presses Play itself, after the robots are pinned")
    else:
        omni.timeline.get_timeline_interface().play()
        log(f"playing; phase {PHASE}")
    S["state"] = "running"


log(f"phase={PHASE} stage={STAGE} push={WITH_PUSH} mask={WITH_MASK}")
omni.usd.get_context().open_stage(STAGE)
S["sub"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    on_update, name="diag_pushable"
)
