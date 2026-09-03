"""Move ONE warehouse object mid-simulation and measure what each sensor saw, when.

Dynamic environment change during navigation is this benchmark's contribution
and, before this file, nothing in the repo had moved anything except the
avatar. This spike establishes the primitive: an object is displaced at
runtime, and a fixed camera and a fixed RTX lidar at the SAME station are read
back frame by frame to find out whether -- and when -- each of them noticed.

The question it exists to answer
--------------------------------
`sim/spikes/FINDINGS.md`, 2026-08-26: after an avatar pose write the RTX lidar
keeps describing the PREVIOUS pose for 5 to 10 frames while the camera at the
same station tracks the write immediately, and for six frames in one
transition the cloud held returns at both poses at once. **Does that apply to
a moved OBJECT?** If it does, every dynamic-change event in the benchmark has
a window in which the modalities disagree about where the world is -- and a
stale cloud is full and plausible, so nothing downstream can tell.

So the crossover is MEASURED on every transition rather than assumed constant,
exactly as `sim/verify_avatar_pose.py` measures it, and the three-state
(old / both / new) transition is tested for explicitly on both modalities.

What it measures
----------------
    landed      the object's world bounding-box centre equals what was asked
                for, read back off the stage after the write
    lidar       returns in the box around each waypoint, as a matrix -- count
                at waypoint j while the object stands at waypoint i. The
                diagonal alone proves nothing (racking satisfies it); it is
                the diagonal STANDING OUT that says the sensor tracks the
                object rather than the furniture
    camera      per pixel, whether the DEPTH changed and now reads the
                object's new distance from the camera (it is HERE) or used to
                read the old one (it was THERE), against a reference frame
                captured immediately before the write. Instance-specific by
                construction -- with the avatar held still the only thing in
                the scene that can change a pixel is the object -- and metric,
                with no camera intrinsics involved anywhere. The naive version
                of this, "which pixels got nearer", has a blind spot that cost
                a run: an object pushed straight away along the camera's own
                sight line makes no pixel anywhere nearer. See `depth_change`
    crossover   for BOTH modalities, on every frame of the settle, how much of
                the reading is at the new pose against how much is still at
                the old one, and the frame they cross over
    three-state whether any frame holds the object at BOTH poses at once: for
                the lidar, net returns in both boxes; for the camera, pixels
                reading the new position with nothing yet reading as vacated
                at the old one
    collision   a PhysX overlap over the object's own footprint at each
                waypoint -- does the COLLIDER move with the render geometry,
                or stay where the object was? This is the half no sensor can
                see and the half a navigating agent walks into. Not a downward
                raycast: in a warehouse that finds the carton stacked on top
    kinematic   the same run twice: once on an object exactly as the asset
                authored it (a static collider), once on an object given
                `UsdPhysics.RigidBodyAPI` with `kinematicEnabled` BEFORE Play.
                "Must it be kinematic?" is then a measured difference between
                two arms rather than a recalled PhysX rule

Failure mode 10 is why the run has the shape it has
----------------------------------------------------
In exec mode on this host Replicator captures NOTHING until `play()` is
called, and an empty buffer is indistinguishable from a working sensor looking
at nothing. A move-verification script that skipped Play would report "no
sensor reacted" and look exactly like an object that never moved. So: Play,
warm up until every promised payload has actually arrived, and only then
sample. `IsaacObservationSource.missing_payloads()` is the gate, and the
verdict records whether it ever cleared so an empty run reports itself vacuous
rather than green.

Which sensors this file may assume are still — AMENDED 2026-09-02
-------------------------------------------------------------------
Every comparison here is frame-to-frame: the object moved, the sensor did not,
therefore the change in the reading is the object. **That argument now holds
for the station sensors only.**

`sim/nav_obstacles.py` makes the three robots dynamic rigid bodies at their
real masses, so the avatar can shove them, and a shoved robot has taken its
camera with it. The reading from `BOT_01_CAM`, `BOT_02_CAM` or `BOT_03_CAM` can
therefore change because the sensor moved rather than because the world did,
and nothing in this file would notice the difference.

* **Safe:** `INFRA_01_CAM`, `INFRA_01_LIDAR` and anything else hanging off a
  station Xform. Those are fixed to the building and nothing in the scene can
  displace them.
* **Not safe without a check:** the three robot cameras.

This run does not currently touch the robots -- it moves warehouse props with
the avatar parked -- so its published numbers stand. The constraint is on
whoever extends it: if a future arm walks the avatar, or the run is repeated
after a GUI session in which somebody shoved a robot, then either restrict the
comparison to the station sensors or record each sensor's pose per frame and
reject any arm in which a sensor moved. `sim/observation_adapter.py` publishes
that pose live (a fresh `UsdGeom.XformCache` every tick), so the check is a
comparison, not new plumbing.

`GUI_NAV_OBSTACLES=0` restores static robots for a session that needs the old
guarantee.

Nothing here is a coordinate somebody typed (hard rule 1)
----------------------------------------------------------
The object is not named in this file. Candidates are found by reading the
stage's own semantic labels, filtered by size, by standing on the floor and by
the lidar's -15..+10 deg elevation band, and then the survivors are ranked by
what the LIDAR ACTUALLY RETURNED off each of them on a live frame -- because
the gate that really decides is OCCLUSION, and no amount of geometry predicts
it. Measured: of 40 props that passed every geometric filter, 11 were visible.
Destinations are searched the same way: an arc about the station (which
preserves the elevation band by construction, the same argument
`sim/verify_avatar_pose.py` uses for its waypoints), screened for free space
with a PhysX overlap query, and rejected if the space is occupied. A guessed
free coordinate fails the way a guessed prim path does, except that it buries
the crate inside a shelf and the sensors then report a plausible nothing.

What this run found is in `sim/spikes/FINDINGS.md` under 2026-08-26: the
answer to the question above is yes, and the "lag" turns out to be a six-frame
refresh cadence in the lidar's buffer.

Capture mode (hard rule 6). No collider mask -- it DELETES colliders, which
would silently answer the collision question with "no" -- no raised
minFrameRate, render products at the registry's declared resolution, and the
same sensor configuration `sim/verify_avatar_pose.py` used, so the frame
counts here are comparable with the avatar numbers rather than merely similar.

Execution model: EXEC MODE, not optional. This reads sensor data, and every
annotator stays empty under `SimulationApp` on this host. No `SimulationApp`,
no `app.update()` loop; frames come from the update event stream, config comes
from environment variables, results are written incrementally and fsync'd.

Run::

    docker compose -f docker/docker-compose.yml run --rm sim \\
        ./runheadless.sh --exec /workspace/sim/spikes/move_object_exec.py

    # ...then `docker stop` it: an exec-mode container does not exit when the
    # script prints DONE, and it holds :8011 and ~1.6 GB of VRAM.
    #
    # Do NOT pass `--name`. A stopped `--rm` container can keep its name, and
    # the next launch then dies on `Conflict. The container name ... is
    # already in use` while its log file simply stays empty -- which looks
    # exactly like a run that started and produced nothing. Measured
    # 2026-08-26; `docker ps -a` is what tells them apart.

Environment (argv is ambiguous after ``--exec``, so config is env vars):

    MO_STAGE     stage to open      (default: sim/observatory_avatar.usd)
    MO_OUT       results directory  (default: the logs volume)
    MO_WARMUP    max frames to wait for every sensor to fill (default 300)
    MO_SETTLE    frames profiled after each move               (default 30)
    MO_MIN_MOVE  metres a waypoint must differ by              (default 2.0)
    MO_ARMS      static | kinematic | both                     (default both)
"""

from __future__ import annotations

import json
import math
import os
import sys
import time as _time
import traceback
from pathlib import Path

import carb
import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

REPO = Path(__file__).resolve().parent.parent.parent
SIM = REPO / "sim"
for _p in (str(REPO), str(SIM)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Importing either sibling must not run its own capture. Set BEFORE the
# imports, never after. Getting this wrong is silent in the expensive
# direction: merely importing observation_adapter would open a stage, run the
# contract suite and post_quit() this session.
os.environ.setdefault("SF_NO_AUTORUN", "1")
os.environ.setdefault("OA_NO_AUTORUN", "1")

import observation_adapter as oa  # noqa: E402  -- sim/ modules, see sys.path
import sensor_factory as sf  # noqa: E402
from core.observation import Modality, MountType  # noqa: E402

STAGE = os.environ.get("MO_STAGE", str(SIM / "observatory_avatar.usd"))
OUT_DIR = Path(os.environ.get("MO_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
WARMUP_FRAMES = int(os.environ.get("MO_WARMUP", "300"))
SETTLE_FRAMES = int(os.environ.get("MO_SETTLE", "30"))
MIN_MOVE_M = float(os.environ.get("MO_MIN_MOVE", "2.0"))
ARMS = os.environ.get("MO_ARMS", "both").lower()

AVATAR = "/Root/Avatar"
WAREHOUSE = "/Root/Warehouse"

#: Labels worth moving: discrete props that stand on the floor. Read from the
#: stage's own semantics rather than chosen by prim name -- `SM_CratePlastic_B`
#: is an asset filename and says nothing about what the thing is.
MOVABLE_LABELS = {"crate", "barel", "cone", "fire_extinguisher", "pallet", "box"}
#: Which of those to spend a bounding box on first. A barrel is a metre tall
#: and a cardboard box is a third of that, and there are 1,841 of the boxes --
#: so this decides the ORDER of the expensive pass, nothing else. Every gate
#: still runs on the real geometry.
LABEL_PRIORITY = {"barel": 0, "crate": 1, "fire_extinguisher": 2, "cone": 3,
                  "box": 4, "pallet": 5}
#: Hard cap on how many bounding boxes the candidate scan may compute. The
#: pass stops early once there are enough candidates; this is the backstop for
#: the case where there are not, so a bad stage costs a minute and not a
#: quarter of an hour.
BBOX_BUDGET = 1600

#: Size gate. Too short and the lidar box cannot clear the floor; too tall or
#: too wide and it is furniture, not a prop.
MIN_HEIGHT_M, MAX_HEIGHT_M = 0.20, 2.00
MAX_FOOTPRINT_M = 2.50
#: Floor-standing only. An object lifted off a shelf would hang in mid-air --
#: neither arm falls, both are kinematic or static -- and its destination
#: could not be screened as floor space.
MAX_STAND_Z_M = 0.60
#: Usable ring around the station. The near end is the elevation band; the far
#: end is where a 0.5 m prop stops being worth counting pixels of.
MIN_RANGE_M, MAX_RANGE_M = 6.0, 25.0

#: Padding on the lidar counting box, and how far above the object's own
#: underside the box starts so that floor returns beneath it are not counted
#: as the object.
BOX_PAD_M = 0.15
BOX_FLOOR_CLEAR_M = 0.12

#: How much the diagonal has to beat the off-diagonal by. Not tuned: it is the
#: difference between "there is something here" and "the something here is the
#: object we moved", and the object is the only thing that moved.
CONTRAST = 5.0
#: A pixel counts as changed when its depth moved by more than this. Depth
#: comes off the G-buffer rather than the denoiser, so this is a margin over
#: quantisation, not over noise -- and the noise floor is MEASURED anyway, on
#: the no-op waypoint that opens every arm.
DEPTH_DELTA_M = 0.25
#: Landing tolerance. A USD transform write is exact; anything above float
#: noise means something else wrote the transform after we did, which is the
#: failure this file exists to catch (it is what PhysX does to the avatar
#: capsule's orientation -- FINDINGS 2026-08-26).
POS_TOL_M = 1e-3

#: Example_Rotary's elevation band, read from the shipped beam profile by
#: sensor_factory. An object outside it is invisible to the lidar and a run
#: built on an invisible object must report itself vacuous, not green.
EL_MIN, EL_MAX = sf.LIDAR_EL_MIN_DEG, sf.LIDAR_EL_MAX_DEG
#: Keep a margin: an object whose centre is 0.5 deg inside the band has half
#: its body outside it.
EL_MARGIN_DEG = 1.5

#: Fraction of the camera's half-FOV a candidate must sit inside. Shortlisting
#: only -- what settles whether the camera sees the object is that moving it
#: changed pixels, which is measured later and cannot be argued with.
FOV_FRACTION = 0.75

#: Net returns each box must hold before a frame counts as holding the object
#: in TWO places at once. Absolute rather than a fraction of the signal, and
#: that is the whole point: measured 2026-08-26, the residue left at the old
#: position during a transition was 36 net returns against 1,723 at the new
#: one -- 4.6%, which a 10%-of-signal threshold discards as noise, and it is
#: not noise. It is a cluster on a body, held for six consecutive frames, in a
#: box that reads 0 the frame before and the frame after. Ten is "more than a
#: stray ray", which is the only thing the number has to mean.
BOTH_MIN_RETURNS = 10

#: The downward collision probe: how far above the object's top the ray starts.
PROBE_UP_M = 1.5

#: Frames to let the renderer catch up after the station camera is re-aimed
#: at an arm's object, before that arm's first reference frame is captured.
#: Without it the no-op waypoint's "noise floor" would be the whole image
#: changing at once, and every later detection threshold derives from it.
REAIM_FRAMES = 30

#: How many candidates each arm gets. The kinematic arm's objects have to be
#: chosen BEFORE Play (see `_apply_kinematic`), so it needs a pool rather than
#: a pick, and the pools must be disjoint or "as authored" would not be.
#: Large, because the gate that actually decides is OCCLUSION and no amount of
#: geometry predicts it: the second run of this file pooled two props, both of
#: which turned out to sit in the same shadow, and had nothing to fall back
#: on. Pool members that are never chosen cost one box count each.
POOL = 40
#: How far apart pooled candidates must be, so that a pool is a spread over
#: the room rather than twenty views of one corner.
POOL_SPACING_M = 1.0
#: Returns a candidate's box must hold before it can carry the run. High, and
#: for a measured reason: a box count is a count of returns in a VOLUME, not on
#: an object, and a prop in a stack shares its volume with its neighbours. On
#: 2026-08-26 a carton whose box held 61 returns turned out to hold 61 returns
#: of the two cartons beside it and none of its own -- moving it away left the
#: count unchanged at 63. Below a couple of hundred there is no way to tell
#: those apart before the move, and the object's own contribution is what the
#: whole measurement is made of.
MIN_RETURNS = 150
#: Minimum separation between the two arms' objects, so neither arm's boxes
#: can ever contain the other arm's object.
ARM_SEPARATION_M = 3.0


def log(msg: str) -> None:
    print(f"[move_object] {msg}", flush=True)


# ---------------------------------------------------------------------------
# Checks: collect, never stop at the first. Five failures in one run beats
# five runs -- and this run costs a stage load and a warm-up.
# ---------------------------------------------------------------------------
class Checks:
    def __init__(self, results: sf.Results) -> None:
        self.rows: list[tuple[bool, str, str]] = []
        self.results = results
        self.vacuous: list[str] = []

    def check(self, ok: bool, name: str, detail: str = "") -> bool:
        ok = bool(ok)
        self.rows.append((ok, name, detail))
        self.results.write(event="check", ok=ok, name=name, detail=detail)
        log(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}")
        return ok

    def cannot_run(self, name: str, why: str) -> None:
        """A check that could not be asked, which is not the same as passing.

        Recorded separately and made fatal in the verdict. A green run whose
        discriminating check never executed is the exact shape of the bug this
        file is about.
        """
        self.vacuous.append(f"{name}: {why}")
        self.results.write(event="vacuous", name=name, why=why)
        log(f"  [VACUOUS] {name}  {why}")

    @property
    def failed(self) -> int:
        return sum(1 for ok, _, _ in self.rows if not ok)

    def report(self) -> None:
        print("\n" + "=" * 78, flush=True)
        print("MOVED-OBJECT VERIFICATION", flush=True)
        print("=" * 78, flush=True)
        for ok, name, detail in self.rows:
            print(f"  {'PASS' if ok else 'FAIL'}  {name:<52s} {detail}", flush=True)
        for line in self.vacuous:
            print(f"  ????  {line}", flush=True)
        print("=" * 78, flush=True)


# ---------------------------------------------------------------------------
# Stage reading
# ---------------------------------------------------------------------------
_LABELS_PREFIX = "semantics:labels:"


def labels_of(prim: Usd.Prim) -> list[str]:
    """Every semantic label on this prim, from BOTH schemas that ship in 6.0.1.

    Reading only the current `UsdSemantics.LabelsAPI` is how the 2026-08-12
    recon concluded this stage was 0.3% labelled when it is 98.5%: 3,467 of
    its entries are on the deprecated `Semantics.SemanticsAPI`, which spells a
    label `semantic:<inst>:params:semanticData`. Both are read here.

    Read through the APPLIED SCHEMA LIST rather than by scanning every
    authored property, which is what the first version did and what made the
    candidate scan take four minutes over 3,137 props -- `GetProperties()` on
    a mesh returns its points, normals, uvs and every primvar as well.

    That is a deliberate trade and it is safe HERE for a reason that was
    measured, not assumed: the 2026-08-25 semantics audit found **zero** prims
    on this stage carrying semantics attributes without the matching API
    schema applied. A coverage AUDIT must still scan properties, because the
    schema-less case is exactly what it exists to find. This is a selection
    heuristic, and a prop it misses costs nothing but that prop.
    """
    out: list[str] = []
    for schema in prim.GetAppliedSchemas():
        if schema.startswith("SemanticsLabelsAPI:"):
            attr = prim.GetAttribute(f"{_LABELS_PREFIX}{schema.split(':', 1)[1]}")
            out += [str(v) for v in ((attr.Get() if attr else None) or [])]
        elif schema.startswith("SemanticsAPI:"):
            attr = prim.GetAttribute(
                f"semantic:{schema.split(':', 1)[1]}:params:semanticData")
            raw = attr.Get() if attr else None
            if raw:
                out += [t.strip() for t in str(raw).split(",") if t.strip()]
    return out


def subtree_labels(prim: Usd.Prim, limit: int = 40) -> list[str]:
    """Labels anywhere in this prop's subtree. The label sits on the MESH."""
    out: list[str] = []
    for i, child in enumerate(Usd.PrimRange(prim)):
        if i > limit:
            break
        out += labels_of(child)
    return sorted(set(out))


def has_collider(prim: Usd.Prim) -> tuple[bool, list[str]]:
    found = []
    for child in Usd.PrimRange(prim):
        if child.HasAPI(UsdPhysics.CollisionAPI):
            found.append(child.GetPath().pathString)
    return bool(found), found[:4]


def rigid_bodies_in(prim: Usd.Prim) -> list[str]:
    return [c.GetPath().pathString for c in Usd.PrimRange(prim)
            if c.HasAPI(UsdPhysics.RigidBodyAPI)]


def rigid_body_ancestor(prim: Usd.Prim) -> str | None:
    """A rigid body inside another rigid body is an error PhysX will not fix."""
    p = prim.GetParent()
    while p and p.IsValid() and not p.IsPseudoRoot():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            return p.GetPath().pathString
        p = p.GetParent()
    return None


def world_range(prim: Usd.Prim, cache: UsdGeom.BBoxCache) -> Gf.Range3d | None:
    try:
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    except Exception:
        return None
    return None if rng.IsEmpty() else rng


def world_translation(prim: Usd.Prim, cache: UsdGeom.XformCache) -> tuple[float, float, float]:
    cache.Clear()
    t = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    return (float(t[0]), float(t[1]), float(t[2]))


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
def elevation_deg(sensor_xyz, target_xyz) -> tuple[float, float]:
    """(horizontal distance, elevation as the SENSOR sees the target)."""
    dx = target_xyz[0] - sensor_xyz[0]
    dy = target_xyz[1] - sensor_xyz[1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return d, -90.0
    return d, math.degrees(math.atan2(target_xyz[2] - sensor_xyz[2], d))


def rotate_about(point_xy, centre_xy, degrees: float) -> tuple[float, float]:
    """Swing `point_xy` around `centre_xy`, preserving the distance between them.

    Preserving the distance is the point. A rotary lidar sees only an
    elevation band -- Example_Rotary sweeps -15..+10 deg -- so a destination on
    an arc centred on the station is in band if the origin was, by
    construction, and the run cannot fail because the object was pushed out of
    the sensor's reach. The same argument places the waypoints in
    sim/verify_avatar_pose.py.
    """
    t = math.radians(degrees)
    dx, dy = point_xy[0] - centre_xy[0], point_xy[1] - centre_xy[1]
    return (centre_xy[0] + dx * math.cos(t) - dy * math.sin(t),
            centre_xy[1] + dx * math.sin(t) + dy * math.cos(t))


class CameraFrustum:
    """Is a world point in front of this camera, and how far off axis?

    Built from the camera prim's own transform and USD's definition of a
    camera -- it looks down its own -Z with +Y up -- rather than from any
    arithmetic in this project, so it is an independent test. The vertical
    half-angle assumes square pixels and a horizontal aperture fit, which is
    an ASSUMPTION about how Kit maps a render product onto the aperture, so it
    is used only to shortlist and never to conclude anything.
    """

    def __init__(self, prim: Usd.Prim, width: int, height: int) -> None:
        m = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
        self.eye = m.ExtractTranslation()
        self.fwd = m.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
        self.right = m.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()
        self.up = m.TransformDir(Gf.Vec3d(0, 1, 0)).GetNormalized()
        cam = UsdGeom.Camera(prim)
        focal = float(cam.GetFocalLengthAttr().Get() or 0.0)
        aperture = float(cam.GetHorizontalApertureAttr().Get() or 0.0)
        self.h_half = (math.atan(aperture / (2.0 * focal))
                       if focal > 0 and aperture > 0 else math.radians(30.0))
        aspect = (height / width) if width else 0.5625
        self.v_half = math.atan(math.tan(self.h_half) * aspect)

    def offsets(self, p) -> tuple[float, float, float]:
        """(forward metres, horizontal degrees off axis, vertical degrees)."""
        v = Gf.Vec3d(p[0], p[1], p[2]) - self.eye
        f = float(v * self.fwd)
        if f <= 1e-6:
            return f, 180.0, 180.0
        return (f,
                math.degrees(math.atan2(float(v * self.right), f)),
                math.degrees(math.atan2(float(v * self.up), f)))

    def contains(self, p, fraction: float = FOV_FRACTION) -> bool:
        f, h, v = self.offsets(p)
        return (f > 0.0
                and abs(h) <= math.degrees(self.h_half) * fraction
                and abs(v) <= math.degrees(self.v_half) * fraction)


# ---------------------------------------------------------------------------
# PhysX probes. What a sensor cannot see: where the COLLIDER is.
# ---------------------------------------------------------------------------
class Physics:
    """Scene queries, or an honest report that there are none.

    Everything here is wrapped, and a failure disables the probe rather than
    the run: the collision question then reports itself unanswerable, which is
    the truth, instead of taking the whole measurement down with it.
    """

    def __init__(self) -> None:
        self.query = None
        self.why: str | None = None
        try:
            from omni.physx import get_physx_scene_query_interface

            self.query = get_physx_scene_query_interface()
        except Exception as exc:                                  # noqa: BLE001
            self.why = f"{type(exc).__name__}: {exc}"

    @property
    def ok(self) -> bool:
        return self.query is not None

    def overlap(self, centre, half, *, want: str | None = None,
                ignore: str | None = None, limit: int = 48) -> dict:
        """What has a collider inside this box.

        THE primitive this file's collision claims rest on, and it replaced a
        downward raycast for a measured reason. The raycast asked "what is the
        topmost collider above this spot", and in a warehouse the answer is
        the box stacked on top: on 2026-08-26 it hit a NEIGHBOUR for all seven
        visible candidates -- `Box_21821`'s probe came back holding
        `Box_21803`, 0.195 m down -- so every one of them was rejected as
        unprobeable while its collider was sitting exactly where it belonged.
        An overlap over the object's own footprint asks the question that was
        meant: is THIS object's collider here.
        """
        out = {"n": 0, "paths": [], "has_want": False, "blocking": [],
               "ok": self.ok}
        if not self.ok:
            out["why"] = "no scene query -- nothing was screened"
            return out
        seen: list[str] = []
        state = {"want": False}

        def report(hit) -> bool:
            try:
                path = str(hit.collision)
            except Exception:                                     # noqa: BLE001
                path = "<unnamed>"
            if want and path.startswith(want):
                state["want"] = True
            elif not (ignore and path.startswith(ignore)) \
                    and len(out["blocking"]) < 8:
                out["blocking"].append(path)
            if len(seen) < 8:
                seen.append(path)
            return len(seen) + len(out["blocking"]) < limit

        try:
            n = self.query.overlap_box(
                carb.Float3(float(half[0]), float(half[1]), float(half[2])),
                carb.Float3(float(centre[0]), float(centre[1]), float(centre[2])),
                carb.Float4(0.0, 0.0, 0.0, 1.0), report, False)
        except Exception as exc:                                  # noqa: BLE001
            out["ok"] = False
            out["why"] = repr(exc)
            return out
        out["n"] = int(n) if n is not None else len(seen)
        out["paths"] = seen
        out["has_want"] = bool(state["want"])
        return out

    def collider_at(self, target, xy) -> dict:
        """Is `target`'s own collider standing at `xy`?"""
        return self.overlap(target.overlap_centre(xy), target.half_extent(),
                            want=target.path)

    def free_at(self, target, xy) -> dict:
        """Is `xy` clear of everything except `target` itself?"""
        return self.overlap(target.overlap_centre(xy), target.half_extent(),
                            ignore=target.path)

    def ray_down(self, x: float, y: float, z_top: float, z_bot: float) -> dict:
        """Straight down onto (x, y). Corroboration only -- see `overlap`.

        Kept because it names the topmost collider over a spot, which is worth
        recording next to an overlap that says only whether something is
        there. It is NOT what any check depends on.
        """
        if not self.ok:
            return {"hit": False, "path": None, "z": None, "why": "no scene query"}
        origin = carb.Float3(float(x), float(y), float(z_top + PROBE_UP_M))
        length = float(PROBE_UP_M + (z_top - z_bot) + 0.6)
        try:
            r = self.query.raycast_closest(origin, carb.Float3(0.0, 0.0, -1.0), length)
        except Exception as exc:                                  # noqa: BLE001
            return {"hit": False, "path": None, "z": None, "why": repr(exc)}
        if not r or not r.get("hit"):
            return {"hit": False, "path": None, "z": None}
        pos = r.get("position")
        return {
            "hit": True,
            "path": str(r.get("collision") or r.get("rigidBody") or ""),
            "z": round(float(pos[2]), 4) if pos is not None else None,
            "distance": round(float(r.get("distance", 0.0)), 4),
        }


# ---------------------------------------------------------------------------
# One movable object
# ---------------------------------------------------------------------------
class Target:
    """A prop that can be moved, and everything measured about it."""

    def __init__(self, prim: Usd.Prim, rng: Gf.Range3d, labels: list[str]) -> None:
        self.prim = prim
        self.path = prim.GetPath().pathString
        self.labels = labels
        lo, hi = rng.GetMin(), rng.GetMax()
        mid = rng.GetMidpoint()
        self.origin_xy = (float(mid[0]), float(mid[1]))
        self.origin_z = float(mid[2])
        self.size = (float(hi[0] - lo[0]), float(hi[1] - lo[1]), float(hi[2] - lo[2]))
        self.z_lo, self.z_hi = float(lo[2]), float(hi[2])
        self.kinematic = False
        self.op = None
        self.op_start = None
        self.lidar_rest = 0
        self.score = 0.0

    # -- the box a waypoint's returns are counted in -----------------------
    def box_at(self, xy) -> tuple[Gf.Vec3d, Gf.Vec3d]:
        """The object's own footprint, padded, lifted clear of the floor.

        Lifted because the object stands ON the floor: a box starting at the
        object's underside also contains the floor directly beneath it, and
        floor returns would then count as object returns at every waypoint --
        including the ones the object had already left.
        """
        hx = self.size[0] / 2.0 + BOX_PAD_M
        hy = self.size[1] / 2.0 + BOX_PAD_M
        # Clearance scales with the object: a fixed 12 cm lifted most of a
        # 25 cm crate out of its own box, which would have counted the object
        # as absent everywhere including where it was standing.
        clear = min(BOX_FLOOR_CLEAR_M, 0.35 * (self.z_hi - self.z_lo))
        return (Gf.Vec3d(xy[0] - hx, xy[1] - hy, self.z_lo + clear),
                Gf.Vec3d(xy[0] + hx, xy[1] + hy, self.z_hi + BOX_PAD_M))

    def half_extent(self) -> tuple[float, float, float]:
        return (self.size[0] / 2.0 + 0.10, self.size[1] / 2.0 + 0.10,
                max(0.05, (self.z_hi - self.z_lo) / 2.0 - 0.05))

    def overlap_centre(self, xy) -> tuple[float, float, float]:
        return (xy[0], xy[1], (self.z_lo + self.z_hi) / 2.0 + 0.10)

    # -- the move itself ---------------------------------------------------
    def bind(self, xform_cache: UsdGeom.XformCache) -> str:
        """Find the op the move will be written through, and remember its value.

        Every one of /Root/Warehouse's 3,137 prop groups carries the op order
        [translate, orient, scale] -- measured off the base layer, not assumed
        -- and USD composes those as M = S * R * T with a row-vector point, so
        the translate is the OUTERMOST op and its value is a displacement in
        the parent's frame. /Root/Warehouse is a Scope and /Root is identity,
        so that frame is world here; the run converts through the parent
        transform anyway and then CHECKS the world bbox landed where asked, so
        a stage that stops being true fails loudly instead of quietly moving
        the object somewhere else.
        """
        xf = UsdGeom.Xformable(self.prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                self.op = op
                self.op_start = op.Get()
                return "existing xformOp:translate"
        # No translate op: add one and make it the FIRST op in the order,
        # because the first op is the outermost. Appending it instead would
        # apply it underneath the prop's own rotation and scale, and the
        # object would move somewhere plausible and wrong.
        op = xf.AddTranslateOp(opSuffix="spikeMove")
        op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
        xf.SetXformOpOrder([op] + list(xf.GetOrderedXformOps())[:-1])
        self.op = op
        self.op_start = op.Get()
        return "added xformOp:translate:spikeMove"

    def write_world_xy(self, xy, xform_cache: UsdGeom.XformCache) -> dict:
        """Put the object's bbox centre at `xy`, keeping its height."""
        xform_cache.Clear()
        parent = xform_cache.GetParentToWorldTransform(self.prim)
        delta_world = Gf.Vec3d(xy[0] - self.origin_xy[0], xy[1] - self.origin_xy[1], 0.0)
        delta_local = parent.GetInverse().TransformDir(delta_world)
        start = self.op_start
        base = Gf.Vec3d(float(start[0]), float(start[1]), float(start[2]))
        want = base + delta_local
        # Match the op's authored precision: a float3 op refuses a Vec3d, and
        # the resulting exception would land in the middle of a run.
        value = type(start)(want[0], want[1], want[2]) if start is not None else want
        self.op.Set(value)
        return {"path": self.path, "requested_xy": [round(v, 4) for v in xy],
                "delta_world": [round(float(v), 4) for v in delta_world],
                "authored": [round(float(v), 5) for v in value]}

    def readback(self, bbox_cache: UsdGeom.BBoxCache) -> tuple[float, float, float] | None:
        """Where the object IS, off the stage. The cache must be cleared: it
        caches bounds, and a cached bound is exactly the wrong answer here."""
        bbox_cache.Clear()
        rng = world_range(self.prim, bbox_cache)
        if rng is None:
            return None
        mid = rng.GetMidpoint()
        return (float(mid[0]), float(mid[1]), float(mid[2]))

    def authored_translate(self):
        v = self.op.Get() if self.op is not None else None
        return None if v is None else [round(float(v[i]), 5) for i in range(3)]

    def summary(self) -> dict:
        return {
            "path": self.path, "labels": self.labels,
            "origin_xy": [round(v, 3) for v in self.origin_xy],
            "size": [round(v, 3) for v in self.size],
            "z": [round(self.z_lo, 3), round(self.z_hi, 3)],
            "kinematic": self.kinematic, "lidar_rest": self.lidar_rest,
            "score": round(self.score, 4),
        }


# ---------------------------------------------------------------------------
# Camera evidence: what the DEPTH image says arrived and departed
# ---------------------------------------------------------------------------
def depth_change(ref: np.ndarray | None, now: np.ndarray | None,
                 r_here: float | None = None, r_there: float | None = None,
                 band: float = 0.8) -> dict:
    """What the camera's DEPTH image says about where the object is.

    Built on `distance_to_camera` rather than rgb. Depth comes straight off
    the G-buffer, so a static scene reproduces it exactly frame to frame,
    while rgb carries whatever temporal accumulation the renderer is doing.
    `inf` is a real value here, not a gap -- `sim/observation_adapter.py` maps
    the annotator's 0 (no hit) to inf, failure mode 4 -- so a pixel that went
    from a surface to open sky is a departure and is counted as one.

    Two families of number, and the second exists because the first has a
    blind spot that a measurement walked straight into.

    `arrived` / `departed` are pixels that got NEARER / FARTHER. They are the
    obvious signal and they are enough for a sideways move. **They are blind
    to a move straight along the camera's own sight line.** Measured
    2026-08-26: a carton pushed from 11.04 m to 13.18 m on the same bearing
    produced 8,489 pixels farther and 149 nearer -- because it re-occupied the
    pixels it had just left, only farther away, so nothing anywhere in the
    frame got nearer. The lidar had 400 returns on it at the new position at
    the same instant. Read on its own, `arrived` says the camera never saw it,
    which is false: the camera saw it perfectly and this metric could not
    express it.

    `here_px` / `there_px` fix that by gating on RANGE instead of on the sign
    of the change: a pixel is HERE if it changed at all and now reads the
    object's new distance, and THERE if it changed at all and used to read the
    old one. That covers the sideways case (the new pixels used to be
    background) and the radial case (the new pixels used to be the object,
    nearer) with the same definition, and it is what the crossover is measured
    on. `r_here`/`r_there` are geometric distances from the camera, computed
    from the stage; the band is the object's own size plus a margin.
    """
    out = {"arrived": 0, "departed": 0, "arrived_depth_m": None,
           "departed_depth_m": None, "here_px": 0, "there_px": 0,
           "here_depth_m": None}
    if ref is None or now is None or ref.shape != now.shape:
        return out
    f_ref, f_now = np.isfinite(ref), np.isfinite(now)
    both = f_ref & f_now
    arrived = (both & ((ref - now) > DEPTH_DELTA_M)) | (~f_ref & f_now)
    departed = (both & ((now - ref) > DEPTH_DELTA_M)) | (f_ref & ~f_now)
    out["arrived"] = int(arrived.sum())
    out["departed"] = int(departed.sum())
    if out["arrived"]:
        vals = now[arrived]
        vals = vals[np.isfinite(vals)]
        if vals.size:
            out["arrived_depth_m"] = round(float(np.median(vals)), 3)
    if out["departed"]:
        vals = ref[departed]
        vals = vals[np.isfinite(vals)]
        if vals.size:
            out["departed_depth_m"] = round(float(np.median(vals)), 3)

    changed = arrived | departed
    if r_here is not None:
        here = changed & f_now & (np.abs(now - r_here) <= band)
        out["here_px"] = int(here.sum())
        if out["here_px"]:
            out["here_depth_m"] = round(float(np.median(now[here])), 3)
    if r_there is not None:
        out["there_px"] = int((changed & f_ref
                               & (np.abs(ref - r_there) <= band)).sum())
    return out


def class_pixels(semantic, labels: dict, wanted: list[str]) -> dict | None:
    """Count and centroid of the pixels carrying any of `wanted`.

    Secondary evidence, and reported rather than asserted: the target's class
    is shared with every other prop of the same kind, so this moves when the
    object moves but says nothing on its own. Ids come out of the reading's
    own map -- the contract promises that an id appearing in the map is named,
    not that the numbers are stable between runs, and they are not.
    """
    if semantic is None or not labels:
        return None
    want = {w.lower() for w in wanted}
    ids = []
    for key, value in labels.items():
        name = value.get("class") if isinstance(value, dict) else value
        if isinstance(name, str) and name.strip().lower() in want:
            try:
                ids.append(int(key))
            except (TypeError, ValueError):
                continue
    if not ids:
        return None
    mask = np.isin(np.asarray(semantic), ids)
    count = int(mask.sum())
    if count == 0:
        return {"count": 0, "centroid": None}
    rows, cols = np.nonzero(mask)
    return {"count": count,
            "centroid": [round(float(cols.mean()), 1), round(float(rows.mean()), 1)]}


# ---------------------------------------------------------------------------
# The run
# ---------------------------------------------------------------------------
class Run:
    """loading -> warmup -> select -> arm(static) -> arm(kinematic) -> verdict."""

    def __init__(self, results: sf.Results) -> None:
        self.results = results
        self.checks = Checks(results)
        self.phase = "loading"
        self.frame = 0
        self.warm = 0
        self.settle = 0
        self.ctx = omni.usd.get_context()
        self.source = None
        self.physics = Physics()
        self.bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        self.xform = UsdGeom.XformCache()
        self.lidar_id: str | None = None
        self.camera_id: str | None = None
        self.camera_prim_path: str | None = None
        self.camera_res: tuple[int, int] = (1280, 720)
        self.reaim = 0
        self.station = None
        self.frustum: CameraFrustum | None = None
        self.pool_static: list[Target] = []
        self.pool_kinematic: list[Target] = []
        self.arms: list[dict] = []
        self.arm_index = -1
        self.index = -1
        self.warm_missing: dict = {}
        self.ref_depth: np.ndarray | None = None
        self.avatar_xyz0 = None
        self.noise: dict = {}
        self.sub = None

    # -- setup -------------------------------------------------------------
    def setup(self) -> None:
        stage = self.ctx.get_stage()
        log(f"stage: {len(list(stage.Traverse()))} prims")

        registry = sf.load_registry()
        # The robot platforms are deliberately NOT referenced in, exactly as
        # sim/verify_avatar_pose.py leaves them out: the claim under test is
        # that sensors which did not move see an object that did, and a static
        # bystander adds three render products to warm up and nothing else.
        for path, pos in sf.create_stations(stage).items():
            log(f"station {path} at {[round(v, 3) for v in pos]}")

        # Capture mode, hard rule 6. No disable_unreachable_colliders(): it
        # switches collision OFF on 1,486 prims, and this run is partly ABOUT
        # colliders -- the mask would answer the collision question with a
        # silent no. No raised minFrameRate. Registry resolution, and the same
        # sensor set (debug draw attached, station markers on) that produced
        # the avatar numbers, so the frame counts compare.
        created = sf.create_registry_sensors(stage, registry)
        if not created:
            raise RuntimeError("no sensors were created -- nothing to observe")
        self.results.write(event="sensors_created", sensors={
            k: {"prim_path": v["prim_path"], "kind": v["kind"]} for k, v in created.items()})

        self.source = oa.IsaacObservationSource(stage, registry, created)
        log(f"sensors: {', '.join(self.source.sensor_ids)}")

        # FIXED mounts only. An avatar-mounted camera would report a changed
        # image for the trivial reason that it went along.
        for sensor_id in self.source.sensor_ids:
            spec = registry.get(sensor_id)
            if spec.mount is not MountType.FIXED:
                continue
            if self.lidar_id is None and spec.modality is Modality.LIDAR:
                self.lidar_id = sensor_id
            if self.camera_id is None and spec.modality is Modality.RGBD:
                self.camera_id = sensor_id
                self.camera_prim_path = created[sensor_id]["prim_path"]
                prim = stage.GetPrimAtPath(self.camera_prim_path)
                w, h = spec.resolution or (1280, 720)
                self.camera_res = (int(w), int(h))
                self.frustum = CameraFrustum(prim, int(w), int(h))
                self.station = world_translation(prim, self.xform)
        if self.station is None and self.lidar_id is not None:
            # Same station Xform, same pose -- config/scene.yaml puts all
            # three modalities on one transform, which is what "three
            # modalities at one pose" means in practice. Without it the
            # elevation-band arithmetic below has no origin.
            self.station = world_translation(
                stage.GetPrimAtPath(created[self.lidar_id]["prim_path"]), self.xform)
            log("! no fixed RGBD sensor -- taking the station pose off the lidar")
        if self.station is None:
            raise RuntimeError("no fixed sensor to take a station pose from")
        log(f"fixed lidar: {self.lidar_id}   fixed camera: {self.camera_id}   "
            f"station {[round(v, 3) for v in self.station]}")
        if not self.physics.ok:
            log(f"! no PhysX scene query ({self.physics.why}) -- the collision "
                f"half of this run cannot be measured")

        self.shortlist(stage)

        char = stage.GetPrimAtPath(f"{AVATAR}/character")
        self.avatar_xyz0 = world_translation(char, self.xform) if char.IsValid() else None
        log(f"avatar character at {self.avatar_xyz0} -- it must not move; every "
            f"'only the object changed' claim below rests on it")

        # THE PLAY, and it is not a formality -- failure mode 10. Before this
        # line every annotator returns an empty buffer that reads exactly like
        # a sensor seeing nothing. Whether it took is checked on the next
        # frame: play() is asynchronous and is_playing() is still False on the
        # line after the call.
        omni.timeline.get_timeline_interface().play()
        log("play() called; is_playing() is checked on the next frame")

    # -- candidate objects -------------------------------------------------
    def shortlist(self, stage) -> None:
        """Find movable props by reading the stage, and split them into pools.

        Runs BEFORE Play, because the kinematic arm's object has to have
        `RigidBodyAPI` applied before PhysX parses the scene -- applying it
        afterwards may or may not register, and a kinematic arm that silently
        never became kinematic would answer "must it be kinematic" with a
        confident, wrong no. The two pools are disjoint so that the static
        arm's object is untouched in the literal sense.

        No field-of-view gate here, and that is a correction rather than a
        relaxation. The first run of this file rejected all 3,137 props, 1,778
        of them for being off camera -- because `sensor_factory` aims every
        station camera at the AVATAR, and this run is not about the avatar. A
        camera pointed somewhere else is a fact about how the rig was aimed,
        not about which objects are movable, so the aim is corrected instead
        (`_aim_camera`) and the shortlist keeps only the gates that are really
        about the object: its size, that it stands on the floor, and that it
        is inside the lidar's elevation band.
        """
        warehouse = stage.GetPrimAtPath(WAREHOUSE)
        if not warehouse.IsValid():
            raise RuntimeError(f"{WAREHOUSE} is not on the stage")

        groups = list(warehouse.GetChildren())
        log(f"scanning {len(groups)} props under {WAREHOUSE} for movable candidates")
        rejected = {"far": 0, "labels": 0, "size": 0, "standing": 0, "range": 0,
                    "band": 0, "collider": 0, "bbox": 0, "rigid": 0}
        near_miss: list[dict] = []

        # PASS 1 -- cheap. The prop's own origin, no geometry touched. A
        # bounding box means walking the prop's meshes; sensor_factory
        # measured a bbox pass over this warehouse at about fifteen minutes,
        # and 1,841 of these props are cardboard boxes. One cache for the
        # whole pass, never cleared: nothing moves during it.
        scan = UsdGeom.XformCache()
        rough: list[tuple[float, Usd.Prim, list[str], float]] = []
        for grp in groups:
            o = scan.GetLocalToWorldTransform(grp).ExtractTranslation()
            gap = math.hypot(float(o[0]) - self.station[0], float(o[1]) - self.station[1])
            if not (MIN_RANGE_M - 2.0 <= gap <= MAX_RANGE_M + 2.0):
                rejected["far"] += 1
                continue
            labels = subtree_labels(grp)
            hit = MOVABLE_LABELS & set(labels)
            if not hit:
                rejected["labels"] += 1
                continue
            # Order the expensive pass by what the label implies about size --
            # a barrel is worth a bounding box before a cardboard box is --
            # and then by proximity. A heuristic, and it only decides the
            # ORDER: every gate below runs on the real geometry.
            rough.append((min(LABEL_PRIORITY.get(h, 9) for h in hit) + gap / 100.0,
                          grp, sorted(hit), gap))
        rough.sort(key=lambda r: r[0])
        log(f"{len(rough)} props are in range and carry a movable label; "
            f"computing bounding boxes for at most {BBOX_BUDGET} of them")

        # PASS 2 -- expensive, and BOUNDED. Stops as soon as there are enough
        # candidates, so the usual case costs a few dozen bounding boxes.
        t0 = _time.time()
        cands: list[Target] = []
        examined = 0
        for _, grp, labels, gap in rough:
            if len(cands) >= 6 * POOL or examined >= BBOX_BUDGET:
                break
            examined += 1
            rng = world_range(grp, self.bbox)
            if rng is None:
                rejected["bbox"] += 1
                continue
            t = Target(grp, rng, labels)
            d, el = elevation_deg(self.station,
                                  (t.origin_xy[0], t.origin_xy[1], t.origin_z))
            miss = None
            if not (MIN_HEIGHT_M <= t.size[2] <= MAX_HEIGHT_M) \
                    or max(t.size[0], t.size[1]) > MAX_FOOTPRINT_M:
                rejected["size"] += 1
                miss = "size"
            elif t.z_lo > MAX_STAND_Z_M:
                rejected["standing"] += 1
                miss = "standing"
            elif not (MIN_RANGE_M <= d <= MAX_RANGE_M):
                rejected["range"] += 1
                miss = "range"
            elif not (EL_MIN + EL_MARGIN_DEG <= el <= EL_MAX - EL_MARGIN_DEG):
                rejected["band"] += 1
                miss = "band"
            if miss is not None:
                # Recorded so that a run which finds nothing says WHY it found
                # nothing, in metres, instead of printing a count of zero.
                if len(near_miss) < 20:
                    near_miss.append({"path": t.path, "labels": labels,
                                      "size": [round(v, 3) for v in t.size],
                                      "z_lo": round(t.z_lo, 3), "d": round(d, 2),
                                      "elevation_deg": round(el, 2), "failed": miss})
                continue
            ok, _ = has_collider(grp)
            if not ok:
                rejected["collider"] += 1
                continue
            if rigid_bodies_in(grp) or rigid_body_ancestor(grp):
                rejected["rigid"] += 1
                continue
            # Apparent size at the station, which is what decides how many
            # returns and how many pixels the thing gets. Not a preference: a
            # prop too small to produce returns makes the whole run vacuous.
            t.score = max(t.size[0], t.size[1]) * t.size[2] / max(d, 1.0)
            t.distance_m = d
            t.elevation_deg = el
            cands.append(t)

        cands.sort(key=lambda c: -c.score)
        log(f"{len(cands)} movable candidates from {examined} bounding boxes in "
            f"{_time.time() - t0:.1f}s; rejected {rejected}")
        for nm in near_miss[:8]:
            log(f"  near miss ({nm['failed']}): {nm['path']} size {nm['size']} "
                f"z_lo {nm['z_lo']} d {nm['d']} el {nm['elevation_deg']}")
        self.results.write(event="shortlist", accepted=len(cands), examined=examined,
                           in_range_with_label=len(rough), rejected=rejected,
                           near_miss=near_miss, seconds=round(_time.time() - t0, 2),
                           top=[c.summary() for c in cands[:12]])
        if not cands:
            raise RuntimeError("no movable prop passed the geometric filters -- "
                               "nothing to move, and inventing one is hard rule 1")

        # Alternate into the pools so both arms get comparable objects, and
        # keep pooled objects apart so a pool is a spread over the room rather
        # than twenty views of one corner -- which is what the second run of
        # this file pooled, and both of its two props were in the same shadow.
        # The 3 m separation the two ARMS need is enforced in `choose`, on the
        # objects that turn out to be visible, not here on ones that may not be.
        for c in cands:
            if any(math.dist(c.origin_xy, o.origin_xy) < POOL_SPACING_M
                   for o in self.pool_static + self.pool_kinematic):
                continue
            pool = (self.pool_static if len(self.pool_static) <= len(self.pool_kinematic)
                    else self.pool_kinematic)
            if len(pool) >= POOL:
                if len(self.pool_static) >= POOL and len(self.pool_kinematic) >= POOL:
                    break
                continue
            pool.append(c)
        log(f"pools: {len(self.pool_static)} static, "
            f"{len(self.pool_kinematic)} kinematic")

        for t in self.pool_static + self.pool_kinematic:
            t.bind_note = t.bind(self.xform)
        if ARMS in ("both", "kinematic"):
            for t in self.pool_kinematic:
                self._apply_kinematic(t)

    def _apply_kinematic(self, t: Target) -> None:
        """Make this prop a KINEMATIC rigid body, before Play.

        `UsdPhysics.RigidBodyAPI` with `physics:kinematicEnabled` -- not a
        dynamic body. A dynamic body would be simulated: it would settle,
        topple or fall through the moment Play is pressed, and the run would
        be measuring gravity rather than a move. A kinematic body is moved to
        whatever pose it is given and is never depenetrated, which is exactly
        the semantics a scripted scene change wants. The prop's collider is an
        exact triangle mesh, which PhysX allows on a kinematic body and
        forbids on a dynamic one -- another reason no dynamic arm is offered.
        """
        try:
            api = UsdPhysics.RigidBodyAPI.Apply(t.prim)
            api.CreateRigidBodyEnabledAttr(True)
            api.CreateKinematicEnabledAttr(True)
            t.kinematic = bool(t.prim.HasAPI(UsdPhysics.RigidBodyAPI))
            log(f"kinematic rigid body applied to {t.path} (HasAPI={t.kinematic})")
        except Exception as exc:                                  # noqa: BLE001
            log(f"! could not make {t.path} kinematic: {exc!r}")
            t.kinematic = False
        self.results.write(event="kinematic_applied", path=t.path, ok=t.kinematic)

    # -- warm-up -----------------------------------------------------------
    def warmup(self) -> bool:
        self.warm += 1
        if self.warm == 1:
            self.checks.check(
                omni.timeline.get_timeline_interface().is_playing(),
                "the timeline is playing before anything is sampled",
                "failure mode 10: capture returns empty buffers while stopped")
        missing = self.source.missing_payloads()
        self.warm_missing = missing
        if not missing:
            log(f"warm-up complete after {self.warm} frames -- every sensor is live")
            self.results.write(event="warmup", frames=self.warm, missing={})
            return True
        if self.warm >= WARMUP_FRAMES:
            log(f"! warm-up gave up after {self.warm} frames; still empty: {missing}")
            self.results.write(event="warmup", frames=self.warm, missing=missing)
            return True
        if self.warm % 60 == 0:
            log(f"warm-up {self.warm}/{WARMUP_FRAMES}, still empty: {sorted(missing)}")
        return False

    # -- choosing, now that the sensors are live ---------------------------
    def choose(self) -> None:
        """Rank the shortlist by what the LIDAR ACTUALLY RETURNED off it.

        The geometric filters cannot see occlusion, and occlusion is how this
        run goes vacuous -- measured, not hypothesised: the second run of this
        file pooled a barrel and a traffic cone, both fully inside the band at
        about 10 m, and both produced exactly zero returns out of a 290,057
        point cloud because a rack stands between them and the station. So the
        choice is settled on a live frame.

        When a candidate produces nothing, the DISTANCE TO THE NEAREST RETURN
        is recorded with it. Those two failures are not the same and the count
        alone cannot tell them apart: a nearest return metres away means the
        object is in a shadow, while one a few centimetres away means the
        cloud is fine and this file's counting box is in the wrong place. The
        first is the room's fault and the second would be a bug.

        The probe that will later carry the collision claim is validated on
        the same frame: an overlap over the object's own footprint must find
        the object's own collider, or that object's collision evidence would
        be about its neighbours instead.
        """
        obs = {o.sensor_id: o for o in self.source.sample_now()}
        lidar = obs.get(self.lidar_id)
        points = None if lidar is None else lidar.data.get("points")
        arr = np.asarray(points) if points is not None and len(points) else None
        frame = None if lidar is None else (lidar.intrinsics or {}).get("frame")
        log(f"lidar reports frame={frame}, {0 if arr is None else len(arr)} points")

        pool = self.pool_static + self.pool_kinematic
        for t in pool:
            lo, hi = t.box_at(t.origin_xy)
            t.lidar_rest = 0 if arr is None else sf.count_in_box(arr, lo, hi, pad=0.0)
            t.nearest_m = None
            if arr is not None and not t.lidar_rest:
                c = np.array([t.origin_xy[0], t.origin_xy[1],
                              (t.z_lo + t.z_hi) / 2.0])
                t.nearest_m = round(float(np.linalg.norm(arr - c, axis=1).min()), 3)
            t.probe_rest = self.physics.collider_at(t, t.origin_xy)
            t.probe_ok = bool(t.probe_rest.get("has_want"))
            t.ray_rest = self.physics.ray_down(t.origin_xy[0], t.origin_xy[1],
                                               t.z_hi, t.z_lo)
        seen = sorted((t for t in pool if t.lidar_rest), key=lambda t: -t.lidar_rest)
        log(f"{len(seen)} of {len(pool)} pooled props are visible to the lidar")
        for t in seen[:10]:
            log(f"  SEEN {t.path}: {t.lidar_rest} returns, d={t.distance_m:.1f} m, "
                f"el={t.elevation_deg:+.1f} deg, own collider "
                f"{'FOUND' if t.probe_ok else 'NOT FOUND'} in its own footprint "
                f"({t.probe_rest.get('n')} colliders there); topmost above it "
                f"is {t.ray_rest.get('path')}")
        for t in [t for t in pool if not t.lidar_rest][:6]:
            why = ("occluded -- the cloud is somewhere else entirely"
                   if t.nearest_m is None or t.nearest_m > 1.0
                   else "THE CLOUD IS ON IT AND THE BOX IS NOT -- a bug here, "
                        "not a shadow in the room")
            log(f"  BLIND {t.path}: d={t.distance_m:.1f} m, nearest return "
                f"{t.nearest_m} m away: {why}")
        self.results.write(event="candidate_measurements", lidar_frame=frame,
                           visible=len(seen), pooled=len(pool),
                           candidates=[{**t.summary(), "overlap": t.probe_rest,
                                        "probe_ok": t.probe_ok,
                                        "ray_down": t.ray_rest,
                                        "nearest_return_m": t.nearest_m,
                                        "distance_m": round(t.distance_m, 2),
                                        "elevation_deg": round(t.elevation_deg, 2)}
                                       for t in pool])

        wanted = []
        if ARMS in ("both", "static"):
            wanted.append(("as-authored static collider", self.pool_static))
        if ARMS in ("both", "kinematic"):
            wanted.append(("kinematic rigid body", self.pool_kinematic))

        taken: list[Target] = []
        for name, candidates in wanted:
            live = [t for t in candidates
                    if t.lidar_rest >= MIN_RETURNS
                    and all(math.dist(t.origin_xy, x.origin_xy) >= ARM_SEPARATION_M
                            for x in taken)]
            if not live:
                best = max((t.lidar_rest for t in candidates), default=0)
                self.checks.cannot_run(
                    f"arm '{name}'",
                    f"no candidate in this pool is visible enough: the best "
                    f"produced {best} lidar returns against the {MIN_RETURNS} "
                    f"needed, so an empty box after a move would say nothing "
                    f"about the move")
                continue
            # Visibility first, because it is what the sensor question needs;
            # a working collider probe only breaks ties. An object the sensors
            # can see but the probe cannot resolve still answers the question
            # this file is mainly about, with the collision half reported
            # unanswerable rather than the whole run abandoned.
            live.sort(key=lambda t: (not t.probe_ok, -t.lidar_rest))
            chosen = live[0]
            taken.append(chosen)
            self.arms.append({"name": name, "target": chosen, "waypoints": None,
                              "samples": []})
            log(f"arm '{name}': {chosen.path} ({', '.join(chosen.labels)}), "
                f"{chosen.lidar_rest} returns at rest, {chosen.distance_m:.1f} m out")

        if not self.arms:
            raise RuntimeError("no arm could be planned -- see the vacuous checks")
        self.results.write(event="arms", arms=[
            {"name": a["name"], "target": a["target"].summary()} for a in self.arms])

    # -- aiming, planning ---------------------------------------------------
    def _aim_camera(self, t: Target) -> dict:
        """Point the station camera at the object this arm is about to move.

        `sensor_factory.create_registry_sensors` aims every station camera at
        the avatar, which is right for every other script in this repo and
        wrong here: the first run of this file found 1,778 of 3,137 props
        outside that frame. Re-aimed ONCE per arm, before the arm's first
        write, and never touched again while a transition is in flight -- so
        the camera is still a FIXED sensor in the sense the claim needs, which
        is that it did not move while the object did.

        Reuses `sensor_factory.look_at_rotate_xyz` rather than deriving the
        rotation again: a camera looks down its own -Z with +Y up, and this
        project has already paid for getting that convention right once.
        """
        stage = self.ctx.get_stage()
        prim = stage.GetPrimAtPath(self.camera_prim_path) if self.camera_prim_path else None
        if prim is None or not prim.IsValid():
            return {"aimed": False, "why": "no camera prim"}
        eye = Gf.Vec3d(*self.station)
        look = Gf.Vec3d(t.origin_xy[0], t.origin_xy[1], (t.z_lo + t.z_hi) / 2.0)
        rot = sf.look_at_rotate_xyz(eye, look)
        attr = prim.GetAttribute("xformOp:rotateXYZ")
        if not attr:
            attr = UsdGeom.Camera(prim).AddRotateXYZOp().GetAttr()
        attr.Set(rot)
        self.frustum = CameraFrustum(prim, *self.camera_res)
        report = {"aimed": True, "at": [round(float(v), 3) for v in look],
                  "rotateXYZ": [round(float(v), 3) for v in rot]}
        log(f"camera aimed at {report['at']} (rotateXYZ {report['rotateXYZ']})")
        self.results.write(event="aim_camera", target=t.path, **report)
        return report

    def plan_arm(self, t: Target) -> list[dict] | None:
        """Four waypoints: origin, two destinations, origin again.

        The first is a NO-OP write, on purpose. It runs the whole machine --
        write, settle, profile, sample -- with nothing moving, so its profile
        is this run's measured NOISE FLOOR for both modalities. Every later
        crossover is read against that instead of against a threshold someone
        picked. The last returns the object to where the asset put it, which
        is both the repeatability check and simple hygiene: this stage is
        never saved, but a run that leaves the warehouse rearranged in memory
        makes every later reading in the same session a lie.
        """
        dests = self.destinations(t)
        if not dests:
            return None
        first = dests[0]
        second = next((d for d in dests[1:]
                       if math.dist(d["xy"], first["xy"]) >= MIN_MOVE_M), None)
        plan = [{"name": "P0 origin", "xy": t.origin_xy, "moved": False},
                {"name": "P1 away", "xy": first["xy"], "moved": True, **first["why"]}]
        if second is not None:
            plan.append({"name": "P2 away", "xy": second["xy"], "moved": True,
                         **second["why"]})
        else:
            # One free spot is enough to answer the question. Three waypoints
            # still give two transitions to measure a crossover on, and the
            # alternative -- a shorter second move -- would put two counting
            # boxes inside each other and make every comparison meaningless.
            log(f"only one free destination for {t.path}: this arm runs three "
                f"waypoints, so it measures two transitions instead of three")
        plan.append({"name": "P3 back", "xy": t.origin_xy, "moved": True})
        return plan

    def destinations(self, t: Target) -> list[dict]:
        """Free positions on an arc about the station, screened by PhysX.

        The arc is what keeps the elevation band -- see `rotate_about` -- and
        because the camera is aimed at the object and the arc is centred on
        the camera, a swing is also, to within the radial term, the bearing
        the destination will have in the image. The radial scales are there
        because a destination at a DIFFERENT range makes the camera's depth
        evidence discriminating rather than merely consistent: the arrived
        pixels' median depth can then be checked against a range the object
        did not come from.
        """
        centre = (self.station[0], self.station[1])
        out: list[dict] = []
        why = {"too_short": 0, "other_arm": 0, "range": 0, "band": 0, "fov": 0,
               "occupied": 0}
        # ONLY the other arm's object, not every pooled candidate. Pooling is
        # deliberately generous -- 40 props, so that at least one in each pool
        # turns out to be visible -- and an earlier version kept destinations
        # 3 m clear of all forty. That is a keep-out zone covering most of the
        # aisle, and it rejected every one of the kinematic arm's candidate
        # positions before any of them was even screened for free space. What
        # the separation is FOR is that one arm's counting boxes must never
        # contain the other arm's object; everything else in the room is
        # handled by the overlap check below, which is a measurement.
        others = [a["target"] for a in self.arms if a["target"] is not t]
        for swing in (0.0, 4.0, -4.0, 8.0, -8.0, 12.0, -12.0, 16.0, -16.0,
                      20.0, -20.0):
            for scale in (1.0, 1.1, 1.2, 1.3, 1.45, 0.9, 0.8):
                xy = rotate_about(t.origin_xy, centre, swing)
                if scale != 1.0:
                    xy = (centre[0] + (xy[0] - centre[0]) * scale,
                          centre[1] + (xy[1] - centre[1]) * scale)
                if math.dist(xy, t.origin_xy) < MIN_MOVE_M:
                    why["too_short"] += 1
                    continue
                if any(math.dist(xy, o.origin_xy) < ARM_SEPARATION_M for o in others):
                    why["other_arm"] += 1
                    continue
                d, el = elevation_deg(self.station, (xy[0], xy[1], t.origin_z))
                if not (MIN_RANGE_M <= d <= MAX_RANGE_M):
                    why["range"] += 1
                    continue
                if not (EL_MIN + EL_MARGIN_DEG <= el <= EL_MAX - EL_MARGIN_DEG):
                    why["band"] += 1
                    continue
                if self.frustum is not None and not self.frustum.contains(
                        (xy[0], xy[1], t.origin_z)):
                    why["fov"] += 1
                    continue
                free = self.physics.free_at(t, xy)
                if free["blocking"]:
                    why["occupied"] += 1
                    continue
                out.append({"xy": xy, "why": {"swing_deg": swing, "radial_scale": scale,
                                              "range_m": round(d, 2),
                                              "elevation_deg": round(el, 2),
                                              "screened": free["ok"],
                                              "colliders_there": free["n"]}})
        # SHORTEST qualifying move first. The instinct is the opposite -- a
        # bigger displacement separates the boxes more -- and it is wrong here:
        # anything past MIN_MOVE_M already separates boxes that are half a
        # metre wide, while a long move is a move into a part of the room
        # nothing has established is visible. Measured 2026-08-26: the
        # farthest-first ordering put a carton 24 m out behind a rack, where
        # neither modality could see it and both transitions went vacuous. The
        # object's CURRENT position is the one place known to be observable,
        # so stay near it.
        out.sort(key=lambda d: math.dist(d["xy"], t.origin_xy))
        log(f"{len(out)} free destinations for {t.path}; rejected {why}")
        self.results.write(event="destinations", target=t.path, free=len(out),
                           rejected=why)
        return out


    # -- the move ----------------------------------------------------------
    def start_arm(self) -> None:
        """Advance to the next arm: aim, plan, wait for the aim, then move.

        Planning happens AFTER aiming because a destination has to be in the
        frame the camera will actually be looking through, and the aim is what
        decides that frame. An arm with no plannable destination is recorded
        as unanswerable and skipped rather than fudged with a shorter move:
        two waypoints closer together than their own counting boxes would make
        every box comparison meaningless.
        """
        while self.arm_index + 1 < len(self.arms):
            self.arm_index += 1
            arm = self.arm()
            log(f"--- arm '{arm['name']}' on {arm['target'].path} ---")
            self._aim_camera(arm["target"])
            plan = self.plan_arm(arm["target"])
            if plan is None:
                self.checks.cannot_run(
                    f"arm '{arm['name']}'",
                    f"no free destination for {arm['target'].path}: every "
                    f"candidate position was occupied, out of the lidar's band, "
                    f"or out of the camera's frame")
                continue
            arm["waypoints"] = plan
            # Distance from the CAMERA to each waypoint, off the stage. This
            # is what the depth image's range gate is read against, and it is
            # geometry rather than anything the camera reported -- so "the
            # camera puts the object where the stage says it is" stays a
            # comparison between two independent things.
            t = arm["target"]
            z_mid = (t.z_lo + t.z_hi) / 2.0
            arm["ranges"] = [math.dist(self.station, (w["xy"][0], w["xy"][1], z_mid))
                             for w in plan]
            arm["band_m"] = max(t.size) / 2.0 + 0.4
            for w in plan:
                log(f"  {w['name']}: {[round(v, 3) for v in w['xy']]}")
            self.results.write(event="waypoints", arm=arm["name"], waypoints=[
                {k: v for k, v in w.items()} for w in plan])
            self.index = -1
            self.reaim = REAIM_FRAMES
            return
        self.finish()

    def arm(self) -> dict:
        return self.arms[self.arm_index]

    def write_move(self) -> None:
        arm = self.arm()
        self.index += 1
        wp = arm["waypoints"][self.index]
        t = arm["target"]

        # The reference the camera's arrived/departed pixels are measured
        # against, taken from the buffers as they stand BEFORE the write.
        ref = {o.sensor_id: o for o in self.source.sample_now()}
        cam = ref.get(self.camera_id)
        self.ref_depth = None if cam is None else cam.data.get("depth")
        if self.ref_depth is not None:
            self.ref_depth = np.array(self.ref_depth, copy=True)

        report = t.write_world_xy(wp["xy"], self.xform)
        wp["write"] = report
        self.settle = SETTLE_FRAMES
        log(f"{arm['name']} / {wp['name']}: -> {report['requested_xy']} "
            f"(delta {report['delta_world']})")
        self.results.write(event="write_move", arm=arm["name"], waypoint=wp["name"],
                           **report)

    def profile_frame(self) -> None:
        """One row of BOTH modalities' catch-up profile, every settle frame.

        MEASURED, because the alternative is a magic constant. The avatar run
        that used a 4-frame settle read every lidar cloud a full waypoint
        stale while the camera beside it was correct, and a cloud of the wrong
        moment is a perfectly good cloud. So this records, for each frame
        after the write, how much of each modality's reading is at the new
        position against how much is still at the old one. The frame those
        cross over is this transition's latency, on this host, for that
        modality -- and the verdict checks the settle budget against it
        instead of the other way round.
        """
        arm = self.arm()
        wp = arm["waypoints"][self.index]
        t = arm["target"]
        f = SETTLE_FRAMES - self.settle
        here = t.box_at(wp["xy"])
        prev = (t.box_at(arm["waypoints"][self.index - 1]["xy"])
                if self.index > 0 else None)

        obs = {o.sensor_id: o for o in self.source.sample_now()}
        lidar = obs.get(self.lidar_id)
        points = None if lidar is None else lidar.data.get("points")
        arr = np.asarray(points) if points is not None and len(points) else None
        row = {
            "f": f,
            "here": 0 if arr is None else sf.count_in_box(arr, *here, pad=0.0),
            "there": (0 if arr is None or prev is None
                      else sf.count_in_box(arr, *prev, pad=0.0)),
            "n": 0 if arr is None else len(arr),
        }
        cam = obs.get(self.camera_id)
        ranges = arm.get("ranges") or []
        change = depth_change(
            self.ref_depth, None if cam is None else cam.data.get("depth"),
            r_here=ranges[self.index] if self.index < len(ranges) else None,
            r_there=(ranges[self.index - 1]
                     if 0 < self.index <= len(ranges) else None),
            band=arm.get("band_m", 0.8))
        row["arrived"] = change["arrived"]
        row["departed"] = change["departed"]
        row["here_px"] = change["here_px"]
        row["there_px"] = change["there_px"]
        wp.setdefault("profile", []).append(row)

    def sample(self) -> None:
        """Read the stage back, then read both sensors. Main thread, post-frame."""
        arm = self.arm()
        wp = arm["waypoints"][self.index]
        t = arm["target"]
        stage = self.ctx.get_stage()
        char = stage.GetPrimAtPath(f"{AVATAR}/character")

        centre = t.readback(self.bbox)
        record: dict = {
            "arm": arm["name"], "waypoint": wp["name"],
            "requested_xy": [round(v, 4) for v in wp["xy"]],
            "centre": None if centre is None else [round(v, 4) for v in centre],
            "authored_translate": t.authored_translate(),
            "playing": bool(omni.timeline.get_timeline_interface().is_playing()),
            "avatar_xyz": (world_translation(char, self.xform)
                           if char.IsValid() else None),
        }

        # The collider, at every one of this arm's positions. No sensor can
        # answer this: render geometry and collision geometry are different
        # scene graphs, and a navigating agent walks into the second one.
        record["colliders"] = {w["name"]: self.physics.collider_at(t, w["xy"])
                               for w in arm["waypoints"]}

        obs = {o.sensor_id: o for o in self.source.sample_now()}
        lidar = obs.get(self.lidar_id)
        if lidar is not None:
            points = lidar.data.get("points")
            arr = np.asarray(points) if points is not None and len(points) else None
            record["lidar_points"] = 0 if arr is None else len(arr)
            record["lidar_frame"] = (lidar.intrinsics or {}).get("frame")
            record["box_counts"] = [
                0 if arr is None else sf.count_in_box(arr, *t.box_at(w["xy"]), pad=0.0)
                for w in arm["waypoints"]]
        cam = obs.get(self.camera_id)
        if cam is not None:
            record["camera_pos"] = [round(float(v), 3) for v in cam.pose.position]
            ranges = arm.get("ranges") or []
            record["depth_change"] = depth_change(
                self.ref_depth, cam.data.get("depth"),
                r_here=ranges[self.index] if self.index < len(ranges) else None,
                r_there=(ranges[self.index - 1]
                         if 0 < self.index <= len(ranges) else None),
                band=arm.get("band_m", 0.8))
            record["geometric_range_m"] = (round(ranges[self.index], 3)
                                           if self.index < len(ranges) else None)
            record["class_pixels"] = class_pixels(cam.data.get("semantic"),
                                                 cam.data.get("semantic_labels"),
                                                 t.labels)
            if centre is not None and record.get("camera_pos"):
                record["range_to_object_m"] = round(
                    math.dist(record["camera_pos"], centre), 3)

        arm["samples"].append(record)
        wp["sample"] = record
        self.results.write(event="sample", **record)
        log(f"  {wp['name']}: centre={record['centre']} "
            f"boxes={record.get('box_counts')} "
            f"depth+/-={record.get('depth_change', {}).get('arrived')}/"
            f"{record.get('depth_change', {}).get('departed')}")

    # -- verdict -----------------------------------------------------------
    def verdict(self) -> None:
        c = self.checks
        c.check(not self.warm_missing, "every sensor filled during warm-up",
                f"{self.warm} frames; still empty: {self.warm_missing or 'nothing'}")
        if self.lidar_id is None:
            c.cannot_run("a fixed lidar observed the move", "none in the registry")
        if self.camera_id is None:
            c.cannot_run("a fixed camera observed the move", "no fixed RGBD sensor")
        if not self.physics.ok:
            c.cannot_run("the collider moved with the object",
                         f"no PhysX scene query: {self.physics.why}")
        # Order matters: the camera's detection threshold is DERIVED from
        # the no-op waypoint's profile, and the per-arm checks read it. An
        # earlier version measured the floor afterwards and silently fell back
        # to a hardcoded 50 px -- which is the shape of bug this file is about.
        self._measure_noise_floor()
        for arm in self.arms:
            self._verdict_arm(arm)
        self._verdict_crossovers()

    def _measure_noise_floor(self) -> None:
        """What each modality reports over a settle in which NOTHING moved.

        Every arm opens with a no-op write to its object's own position, which
        runs the whole machine -- write, settle, profile, sample -- with the
        world held still. Whatever that profile shows is this run's floor on
        this host, and the detection thresholds the later transitions are read
        against come from it rather than from a number someone picked.
        """
        for arm in self.arms:
            tag = arm["name"].split()[0]
            prof = (arm["waypoints"][0].get("profile") or []) if arm["waypoints"] else []
            if not prof:
                self.checks.cannot_run(f"[{tag}] the noise floor was measured",
                                       "the no-op waypoint produced no profile")
                self.noise[arm["name"]] = {"max_px": None, "min_px": 50}
                continue
            max_px = max((max(r.get("here_px", 0), r.get("there_px", 0))
                          for r in prof), default=0)
            max_here = max((r["here"] for r in prof), default=0)
            min_here = min((r["here"] for r in prof), default=0)
            pixels = self.camera_res[0] * self.camera_res[1]
            self.noise[arm["name"]] = {
                "max_px": max_px, "min_px": max(150, 3 * max_px),
                "frame_px": pixels, "lidar_box_range": [min_here, max_here]}
            log(f"  [{tag}] noise floor over {len(prof)} frames with nothing "
                f"moving: camera {max_px} px, lidar box {min_here}..{max_here} returns")
            self.results.write(event="noise_floor", arm=arm["name"],
                               **self.noise[arm["name"]])
            # The floor is not zero and there is no reason it should be. The
            # depth buffer is rendered under the renderer's sub-pixel jitter,
            # so silhouette pixels flip between a near surface and a far one
            # and move by metres while nothing in the world moves at all --
            # 534 px of 921,600 on 2026-08-26, all of it edges. What matters
            # is not that the floor is small in absolute terms but that it is
            # small next to the frame, because the detection threshold is
            # derived from it (3x) rather than chosen.
            self.checks.check(
                max_px < 0.01 * pixels,
                f"[{tag}] the camera's depth is quiet when nothing moves",
                f"{max_px} px of {pixels} ({100.0 * max_px / pixels:.3f}%) both "
                f"moved by >{DEPTH_DELTA_M} m and read the object's own range, "
                f"over {len(prof)} frames -- this arm's detection threshold is "
                f"{self.noise[arm['name']]['min_px']} px")

    @staticmethod
    def _backgrounds(arm: dict) -> list[int]:
        """Per waypoint box, how many returns it holds when the object is NOT in it.

        THE correction this measurement cannot do without, and it is not a
        detail. A box count counts returns in a VOLUME. For an avatar standing
        in an aisle the volume empties when it walks away, so the raw count is
        the avatar. A warehouse prop stands in a stack against racking: when it
        leaves, the shelf and the cartons beside it are revealed, and the box
        settles at a few hundred returns that were never the object. Measured
        2026-08-26: P0's box read 1,423 with the carton in it and 604 without,
        so a raw "here vs there" comparison had the object's new position
        (396) losing to the furniture it had left behind (604), and reported
        that the lidar never caught up on a transition where the trace plainly
        shows it catching up at frame 5.

        Taken as the MINIMUM over the settled samples in which the object
        stood somewhere else -- a measurement of that volume with the object
        out of it, not an estimate.
        """
        waypoints, samples = arm["waypoints"] or [], arm["samples"]
        out = []
        for k, wk in enumerate(waypoints):
            vals = [s["box_counts"][k] for j, s in enumerate(samples)
                    if j < len(waypoints) and s.get("box_counts")
                    and math.dist(waypoints[j]["xy"], wk["xy"]) >= MIN_MOVE_M]
            out.append(min(vals) if vals else 0)
        return out

    def _verdict_arm(self, arm: dict) -> None:
        c = self.checks
        t = arm["target"]
        tag = arm["name"].split()[0]
        samples = arm["samples"]
        if not samples:
            c.cannot_run(f"[{tag}] the arm ran", "no waypoint was sampled")
            return

        for rec in samples:
            name = f"[{tag}] {rec['waypoint']}"
            if rec["centre"] is None:
                c.cannot_run(f"{name}: the object landed", "no bounding box")
            else:
                err = math.dist(rec["centre"][:2], rec["requested_xy"])
                c.check(err <= POS_TOL_M, f"{name}: the object is where it was put",
                        f"error {err * 1000:.3f} mm; authored translate "
                        f"{rec['authored_translate']}")
            c.check(rec["playing"], f"{name}: sampled while playing", "failure mode 10")
            if self.avatar_xyz0 and rec.get("avatar_xyz"):
                drift = math.dist(rec["avatar_xyz"], self.avatar_xyz0)
                c.check(drift <= 0.05,
                        f"{name}: the avatar did not move",
                        f"{drift * 1000:.1f} mm -- every 'only the object changed' "
                        f"claim rests on this")

        floor = self.noise.get(arm["name"], {})
        min_px = int(floor.get("min_px", 150))
        # Whether ANY modality resolved the object at each waypoint. Used to
        # separate "the sensor missed it" from "nothing could have seen it".
        observable = [True] * len(samples)

        # -- lidar: the matrix ------------------------------------------
        matrix = [rec.get("box_counts") for rec in samples]
        frames = {rec.get("lidar_frame") for rec in samples}
        if any(m is None for m in matrix):
            c.cannot_run(f"[{tag}] the lidar tracks the object",
                         "no cloud at some waypoint")
        elif frames != {"world"}:
            # The boxes are in WORLD metres. A sensor-local cloud would put
            # every return near the origin, every box would read zero, and the
            # run would report a move that never reached the sensor --
            # failure mode 2, one layer up.
            c.cannot_run(f"[{tag}] the lidar tracks the object",
                         f"the reading reported frame {frames}, not 'world'")
        else:
            bg = self._backgrounds(arm)
            net = [[max(0, v - b) for v, b in zip(row, bg)] for row in matrix]
            observable = [
                net[i][i] > 0
                or (rec.get("depth_change") or {}).get("here_px", 0) >= min_px
                for i, rec in enumerate(samples)]
            print(f"\n  [{tag}] lidar returns per box, RAW and (net of the "
                  f"background that box holds with the object elsewhere)",
                  flush=True)
            head = " ".join(f"{w['name'].split()[0]:>13s}" for w in arm["waypoints"])
            print(f"  {'':<14s}{head}", flush=True)
            for rec, row, nrow in zip(samples, matrix, net):
                print(f"  {rec['waypoint']:<14s}"
                      + " ".join(f"{v:>7d}({n:>4d})" for v, n in zip(row, nrow)),
                      flush=True)
            print(f"  {'background':<14s}" + " ".join(f"{b:>13d}" for b in bg),
                  flush=True)
            self.results.write(event="lidar_matrix", arm=arm["name"],
                               matrix=matrix, net=net, background=bg)
            for i, rec in enumerate(samples):
                here = net[i][i]
                if here <= 0:
                    # Zero net returns is TWO different findings and the count
                    # cannot tell them apart. If the camera saw nothing arrive
                    # either, the destination is simply not observable from
                    # this station and the transition says nothing about the
                    # lidar. If the camera DID see it arrive, the lidar failed
                    # to, and that is a result.
                    ch = rec.get("depth_change") or {}
                    seen_by_camera = ch.get("here_px", 0) >= min_px
                    if i > 0 and not seen_by_camera:
                        c.cannot_run(
                            f"[{tag}] {rec['waypoint']}: lidar has returns on "
                            f"the object",
                            f"neither modality sees anything at this position "
                            f"({ch.get('here_px', 0)} px at its range, 0 net "
                            f"returns) "
                            f"-- the destination is occluded from this station, "
                            f"which is a fact about the room and not about the "
                            f"sensor")
                    else:
                        c.check(False, f"[{tag}] {rec['waypoint']}: lidar has "
                                       f"returns on the object",
                                f"0 net returns in its box (raw {matrix[i][i]}, "
                                f"background {bg[i]}) while the camera counted "
                                f"{ch.get('here_px', 0)} px at its range")
                    continue
                mine = arm["waypoints"][i]["xy"]
                for j, other in enumerate(arm["waypoints"]):
                    # P3 is P0's position, so their boxes are the SAME box and
                    # comparing them would be comparing a waypoint with itself.
                    if j == i or math.dist(other["xy"], mine) < MIN_MOVE_M:
                        continue
                    there = net[i][j]
                    c.check(here >= CONTRAST * max(there, 1),
                            f"[{tag}] {rec['waypoint']}: returns are HERE, not at "
                            f"{other['name'].split()[0]}",
                            f"{here} vs {there} net "
                            f"({here / max(there, 1):.1f}x, need {CONTRAST:.0f}x)")

        # -- camera: arrived and departed, in metres ---------------------
        for i, rec in enumerate(samples):
            if i == 0 or "depth_change" not in rec:
                continue
            ch = rec["depth_change"]
            name = f"[{tag}] {rec['waypoint']}"
            detail = (f"{ch.get('there_px', 0)} px at the old range, "
                      f"{ch.get('here_px', 0)} px at the new one "
                      f"({ch['departed']} farther / {ch['arrived']} nearer; "
                      f"noise floor {floor.get('max_px', '?')} px, need {min_px})")
            if ch.get("here_px", 0) >= min_px and ch.get("there_px", 0) >= min_px:
                c.check(True, f"{name}: the camera saw the object leave and "
                              f"arrive", detail)
            elif not observable[i] or not observable[i - 1]:
                # Nothing reached either modality at one end of this move, so
                # the camera being silent is a fact about what is in the way,
                # not about whether the camera tracks. Recorded as
                # unanswerable, which is fatal to the run's exit code, rather
                # than as a failure of the thing under test.
                c.cannot_run(f"{name}: the camera saw the object leave and arrive",
                             f"{detail}; the lidar cannot see this position "
                             f"either, so the position is occluded from the "
                             f"station")
            else:
                c.check(False, f"{name}: the camera saw the object leave and "
                               f"arrive", detail)
            # Metric, and it needs no camera intrinsics: the object's range
            # comes out of the depth image itself, and out of the geometry
            # independently, and they have to agree. Read off the pixels that
            # got NEARER rather than off the range-gated set, because the
            # range-gated set was selected by this very range and comparing it
            # against that range would prove nothing. Skipped, and said so,
            # when a move has no nearer pixels at all -- a move straight away
            # from the camera has none by construction.
            want = rec.get("range_to_object_m")
            got = ch.get("arrived_depth_m")
            tol = max(t.size) / 2.0 + 0.6
            if want and got and ch["arrived"] >= min_px:
                c.check(abs(got - want) <= tol,
                        f"{name}: the camera puts the object at the right range",
                        f"depth of the pixels that got nearer {got:.2f} m vs "
                        f"geometric {want:.2f} m (tol {tol:.2f} m)")
            else:
                log(f"  {name}: range check skipped -- only {ch['arrived']} px "
                    f"got nearer, which is what a move directly away from the "
                    f"camera looks like")

        # -- collision ---------------------------------------------------
        if not self.physics.ok:
            pass                       # already reported once, in verdict()
        elif not getattr(t, "probe_ok", False):
            # The overlap could not find this object's own collider even where
            # it was standing untouched. Whatever that is, it is not evidence
            # about whether a collider follows a move.
            c.cannot_run(f"[{tag}] the collider moved with the object",
                         f"{t.path}'s own collider was not in its own footprint "
                         f"at rest: {t.probe_rest}")
        else:
            self._verdict_collision(arm, tag)

        # -- repeatability ------------------------------------------------
        if len(samples) >= 4:
            a, b = samples[0], samples[-1]
            if a["centre"] and b["centre"]:
                c.check(math.dist(a["centre"], b["centre"]) <= POS_TOL_M,
                        f"[{tag}] the object came back to where the asset put it",
                        f"{math.dist(a['centre'], b['centre']) * 1000:.3f} mm "
                        f"after three intervening writes")
            ba, bb = a.get("box_counts"), b.get("box_counts")
            if ba and bb:
                ratio = bb[0] / max(ba[0], 1)
                # 40%: returns per box vary tick to tick because the sweep is
                # not phase-locked to the sample. The claim is that the object
                # came back, not that the buffer is byte-identical.
                c.check(0.6 <= ratio <= 1.4,
                        f"[{tag}] the same position reproduces the same cloud",
                        f"{ba[0]} -> {bb[0]} returns in P0's box ({ratio:.2f}x)")

    def _verdict_collision(self, arm: dict, tag: str) -> None:
        """Did the COLLIDER go with the render geometry?

        Read off the overlap recorded at every waypoint: at the sample taken
        while the object stands at P_i, an overlap over the object's own
        footprint at P_i must report the object's own collider, and the same
        overlap at P_(i-1) must not. Reported per waypoint rather than as one
        verdict, because "the collider follows sometimes" is a distinct and
        much worse answer than either "always" or "never".
        """
        c = self.checks
        rows = []
        for i, rec in enumerate(arm["samples"]):
            wp = arm["waypoints"][i]
            probes = rec.get("colliders") or {}
            here = probes.get(wp["name"]) or {}
            hit_here = bool(here.get("has_want"))
            left = None
            if i > 0:
                prev = arm["waypoints"][i - 1]
                if math.dist(prev["xy"], wp["xy"]) >= MIN_MOVE_M:
                    left = not bool((probes.get(prev["name"]) or {}).get("has_want"))
            rows.append({"waypoint": wp["name"], "collider_here": hit_here,
                         "collider_left_previous": left, "overlap": here})
            c.check(hit_here, f"[{tag}] {wp['name']}: the collider is where the "
                              f"object is",
                    f"overlap over the object's own footprint found "
                    f"{here.get('n')} colliders, "
                    f"{'including' if hit_here else 'NOT including'} its own "
                    f"({here.get('paths')})")
            if left is not None:
                c.check(left, f"[{tag}] {wp['name']}: the collider left the "
                              f"previous position",
                        "a collider left behind is an invisible wall: nothing "
                        "renders there and an agent still cannot walk through it")
        self.results.write(event="collision", arm=arm["name"], rows=rows)

    def _verdict_crossovers(self) -> None:
        """The headline: when did each modality catch up, per transition?

        Every count here is NET of the background that box holds with the
        object out of it -- see `_backgrounds`. Without that correction a prop
        that stood against racking loses to the racking it uncovered, and the
        crossover reads as "never happened" on a transition whose raw trace
        shows it happening at frame 5.
        """
        c = self.checks
        table: list[dict] = []
        for arm in self.arms:
            tag = arm["name"].split()[0]
            samples = arm["samples"]
            bg = self._backgrounds(arm) if samples else []
            floor = self.noise.get(arm["name"], {})
            min_px = int(floor.get("min_px", 150))
            for i, wp in enumerate(arm["waypoints"] or []):
                prof = wp.get("profile") or []
                if not prof or i == 0:
                    continue    # the no-op waypoint IS the noise floor, and it
                                # is measured in _measure_noise_floor above
                here_bg = bg[i] if i < len(bg) else 0
                there_bg = bg[i - 1] if i - 1 < len(bg) else 0
                rows = [{"f": r["f"],
                         "here": max(0, r["here"] - here_bg),
                         "there": max(0, r["there"] - there_bg),
                         "arrived": r["arrived"], "departed": r["departed"],
                         "here_px": r.get("here_px", 0),
                         "there_px": r.get("there_px", 0)}
                        for r in prof]
                here_final = rows[-1]["here"]
                there_start = rows[0]["there"]
                lidar_min = max(5, int(0.2 * here_final))
                lidar_cross = next((r["f"] for r in rows
                                    if r["here"] > r["there"] and r["here"] >= lidar_min),
                                   None)
                cam_cross = next((r["f"] for r in rows if r["here_px"] >= min_px), None)
                # The three-state transition: the reading holds the object at
                # BOTH positions at once. For the lidar that is net returns in
                # both boxes; for the camera it is pixels that got nearer with
                # no matching pixels that got farther, which is the same thing
                # in the camera's units -- the object drawn in its new place
                # while its old place has not been vacated.
                both = [r["f"] for r in rows
                        if r["here"] >= BOTH_MIN_RETURNS
                        and r["there"] >= BOTH_MIN_RETURNS]
                # The camera's three-state test, and `there_px` is the WRONG
                # half of it: `there_px` asks what a pixel USED to show, which
                # never stops being true once the object has moved, so it fired
                # on all thirty frames of every transition. What "in two places
                # at once" means for a depth image is that the object is
                # visible at its new range while its old place has NOT yet been
                # vacated -- and a place that has not been vacated is exactly a
                # place with no pixels that got farther.
                cam_both = [r["f"] for r in rows
                            if r["here_px"] >= min_px and r["departed"] < min_px]
                trace = " ".join(f"{r['f']}:{r['here']}/{r['there']}" for r in rows)
                cam_trace = " ".join(f"{r['f']}:{r['here_px']}/{r['there_px']}"
                                     for r in rows)
                row = {"arm": arm["name"], "transition": wp["name"],
                       "lidar_crossover": lidar_cross, "camera_crossover": cam_cross,
                       "lidar_both_frames": both, "camera_both_frames": cam_both,
                       "here_final": here_final, "there_start": there_start,
                       "background_here": here_bg, "background_there": there_bg,
                       "profile_raw": prof, "profile_net": rows}
                table.append(row)
                self.results.write(event="crossover", **row)
                if here_final <= 0 and cam_cross is None:
                    c.cannot_run(
                        f"[{tag}] {wp['name']}: the lidar caught up within "
                        f"{SETTLE_FRAMES} frames",
                        f"nothing arrived at this position on either modality "
                        f"-- it is occluded from the station, so there is no "
                        f"catching up to measure. net here/there: {trace}")
                else:
                    c.check(lidar_cross is not None,
                            f"[{tag}] {wp['name']}: the lidar caught up within "
                            f"{SETTLE_FRAMES} frames",
                            f"crossover at frame {lidar_cross}; net here/there "
                            f"by frame (backgrounds {here_bg}/{there_bg}): {trace}")
                    c.check(cam_cross is not None,
                            f"[{tag}] {wp['name']}: the camera caught up within "
                            f"{SETTLE_FRAMES} frames",
                            f"crossover at frame {cam_cross}; px at the new "
                            f"range/at the old, by frame: {cam_trace}")

        if not table:
            c.cannot_run("the modalities are compared", "no transition was profiled")
            return

        print("\n  CROSSOVER PER TRANSITION -- the frame each modality first "
              "described the NEW position", flush=True)
        print(f"  {'arm':<28s}{'transition':<12s}{'lidar':>7s}{'camera':>8s}"
              f"{'lag':>6s}   both-at-once frames", flush=True)
        for row in table:
            lc, cc = row["lidar_crossover"], row["camera_crossover"]
            lag = "n/a" if lc is None or cc is None else f"{lc - cc:+d}"
            both = (f"lidar {row['lidar_both_frames']}"
                    if row["lidar_both_frames"] else "lidar none")
            both += (f", camera {row['camera_both_frames']}"
                     if row["camera_both_frames"] else ", camera none")
            print(f"  {row['arm']:<28.28s}{row['transition']:<12s}"
                  f"{'-' if lc is None else lc:>7}{'-' if cc is None else cc:>8}"
                  f"{lag:>6s}   {both}", flush=True)

        lags = [row["lidar_crossover"] - row["camera_crossover"] for row in table
                if row["lidar_crossover"] is not None and row["camera_crossover"] is not None]
        three = [row for row in table if row["lidar_both_frames"]]
        if lags:
            log(f"  lidar minus camera, per transition: {lags} "
                f"(min {min(lags)}, max {max(lags)})")
        log(f"  transitions holding the object in two places at once: "
            f"{len(three)} of {len(table)}"
            + ("".join(f"; {r['transition']} for {len(r['lidar_both_frames'])} "
                       f"frames {r['lidar_both_frames'][0]}..{r['lidar_both_frames'][-1]}"
                       for r in three) if three else ""))
        self.results.write(event="crossover_summary", lags=lags,
                           three_state_transitions=len(three), transitions=len(table),
                           lidar_crossovers=[r["lidar_crossover"] for r in table],
                           camera_crossovers=[r["camera_crossover"] for r in table])

    # -- the update pump ---------------------------------------------------
    def on_update(self, _event) -> None:
        self.frame += 1
        try:
            if self.phase == "loading":
                status = self.ctx.get_stage_loading_status()
                if self.frame > 5 and not any(status[1:]):
                    log(f"stage loaded after {self.frame} frames")
                    self.setup()
                    self.phase = "warmup"
                return

            if self.phase == "warmup":
                if self.warmup():
                    self.choose()
                    self.arm_index = -1
                    self.phase = "moving"
                    self.start_arm()
                return

            if self.phase == "moving":
                # The camera was just re-aimed; give the renderer time to
                # trace the new view BEFORE the first reference frame is
                # captured, or the no-op waypoint would measure the re-aim
                # instead of the noise floor.
                if self.reaim > 0:
                    self.reaim -= 1
                    if self.reaim == 0:
                        self.write_move()
                    return
                # Settle before sampling: the write happened on an earlier
                # frame and the renderer has to have traced it. Every frame of
                # the wait is profiled rather than skipped, because the length
                # of the wait is the measurement.
                if self.settle > 0:
                    self.settle -= 1
                    self.profile_frame()
                    return
                self.sample()
                if self.index + 1 < len(self.arm()["waypoints"]):
                    self.write_move()
                else:
                    self.start_arm()
                return
        except Exception as exc:
            log("FAILED: " + repr(exc))
            log(traceback.format_exc())
            self.results.write(event="error", error=repr(exc), tb=traceback.format_exc())
            self.checks.check(False, "the run completed", repr(exc))
            self.finish()

    def finish(self) -> None:
        if self.phase == "done":
            return
        self.phase = "done"
        # Put every object back. The stage is never saved, but a run that
        # leaves the warehouse rearranged makes every later reading in this
        # session wrong, and the next thing to run in this session is usually
        # a check of something else.
        for t in self.pool_static + self.pool_kinematic:
            try:
                if t.op is not None and t.op_start is not None:
                    t.op.Set(t.op_start)
            except Exception as exc:                              # noqa: BLE001
                log(f"! could not restore {t.path}: {exc!r}")

        # The verdict is code, and code called from an error path must not be
        # the reason the run never reports. A crash in here is a FAILED
        # verification, not a session that hangs holding the port.
        try:
            self.verdict()
        except Exception as exc:                                  # noqa: BLE001
            log("verdict failed: " + repr(exc))
            log(traceback.format_exc())
            self.checks.check(False, "the verdict ran", repr(exc))
        self.checks.report()

        failed = self.checks.failed
        code = 1 if (failed or self.checks.vacuous) else 0
        summary = {
            "frames": self.frame, "warmup_frames": self.warm,
            "settle_frames": SETTLE_FRAMES,
            "arms": [{"name": a["name"], "target": a["target"].summary(),
                      "waypoints": len(a["samples"])} for a in self.arms],
            "checks": len(self.checks.rows), "failed": failed,
            "vacuous": self.checks.vacuous,
            "physx_scene_query": self.physics.ok,
            "noise_floor": self.noise,
            "exit_code": code,
        }
        if self.source is not None:
            summary["sensors"] = list(self.source.sensor_ids)
            self.source.close()
        self.results.write(event="summary", **summary)
        log("SUMMARY " + json.dumps(summary, default=str)[:4000])
        log(f"MOVED-OBJECT VERIFICATION {'PASSED' if code == 0 else 'FAILED'} "
            f"({failed} failed, {len(self.checks.vacuous)} could not run, "
            f"{len(self.checks.rows)} checks)")
        log("A move here is a PLACEMENT, not a swept motion: the object is "
            "teleported between poses and nothing sweeps the space in between, "
            "so nothing in this run says anything about an object pushed "
            "through, or into, something else.")
        log("DONE")
        self.sub = None
        omni.kit.app.get_app().post_quit(code)


def main() -> None:
    out = OUT_DIR / "move_object.jsonl"
    results = sf.Results(out)
    log(f"stage={STAGE} settle={SETTLE_FRAMES} arms={ARMS} min_move={MIN_MOVE_M}")
    log(f"results -> {out}")
    results.write(event="start", stage=STAGE, started=_time.time(),
                  settle=SETTLE_FRAMES, arms=ARMS, min_move_m=MIN_MOVE_M,
                  warmup=WARMUP_FRAMES)

    opened = omni.usd.get_context().open_stage(STAGE)
    ok, err = opened if isinstance(opened, tuple) else (opened, None)
    log(f"open_stage ok={ok} err={err}")
    results.write(event="open_stage", ok=bool(ok), err=str(err))

    run = Run(results)
    run.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        run.on_update, name="move_object_exec")
    log("subscribed to the update stream")


def _is_exec_entrypoint() -> bool:
    """True when Kit --exec'd this file, false when something imports it.

    Same reasoning as sensor_factory._is_exec_entrypoint, and the same two bad
    outcomes: too strict and the run moves nothing while looking fine, too
    loose and importing this module opens a stage, presses Play and
    post_quit()s the session.
    """
    return os.environ.get("MO_NO_AUTORUN") != "1"


if _is_exec_entrypoint():
    main()
