"""Assert, headless, that ``avatar.set_avatar_pose()`` lands AND that a fixed
sensor sees it land.

Two halves, and the second is the one worth having. USD says what you wrote;
it does not say what the renderer traced. Every failure mode in this project's
list is of that shape -- the pose is authored perfectly and the sensors keep
reporting the scene they were reporting before, with nothing raised anywhere.
So this script writes a pose, reads it back off the stage, and then reads the
INFRA_01 station's lidar and camera and asserts the readings moved with it.

What it checks
--------------
    pose        the capsule's world translation equals what was asked for
    yaw         the FIRST-PERSON CAMERA's world heading equals what was asked
                for. Deliberately measured off the camera and not off the
                character: the camera's forward is its own -Z, which is USD's
                definition and not this project's, so it is an independent
                check of the yaw convention rather than avatar.py's rig
                arithmetic marking its own homework.
    body        the visible character moved with the capsule, and turned by
                the same angle the camera turned. A relative check, so it
                needs no convention at all.
    lidar       the FIXED station lidar's world-frame cloud has returns at
                each commanded position and not at the other ones. Reported as
                a full matrix -- count of points in the box around waypoint j,
                measured while the avatar stands at waypoint i -- because the
                diagonal alone proves nothing: racking would satisfy it. It is
                the diagonal STANDING OUT that says the sensor is tracking the
                avatar and not the furniture.
    camera      the FIXED station camera's `person` pixels moved, and its rgb
                changed.
    repeatable  the last waypoint returns to the first. Same pose in, same
                readings out -- which is what "callable repeatedly without
                restarting the simulator" has to mean if it means anything.

Failure mode 10 is the reason for the shape of the run
-------------------------------------------------------
In exec mode on this host, Replicator captures **nothing** until ``play()`` is
called -- 40 stopped frames produced no data on any render product, and all of
them filled at the first frame after Play. Nothing raises: ``get_data()``
returns an empty buffer, which is indistinguishable from a working sensor
looking at nothing. **A pose-verification script that skipped Play would
therefore report "the sensor did not react" and look exactly like a broken
pose write.** So: Play first, warm up until every promised payload has
actually arrived, and only then sample. The warm-up gate is
``IsaacObservationSource.missing_payloads()``, and the verdict records whether
it ever cleared, so an empty run is reported as vacuous rather than as a
failure of the thing under test.

Execution model: EXEC MODE, and not optional -- this reads sensor data, and
every annotator stays empty under ``SimulationApp`` on this host (CLAUDE.md).
No ``SimulationApp``, no ``app.update()`` loop; frames come from the update
event stream, config comes from environment variables, results are written
incrementally and fsync'd.

Which route through the character controller does this exercise?
----------------------------------------------------------------
**Both, and not by choice -- measured 2026-08-26.** The intent was to run this
the way ``sim/observation_adapter.py`` believes it runs: no
``--enable omni.physx.cct``, so the node type stays unregistered and nothing
contends for the capsule. **That is not what happens.** ``runheadless.sh``
starts ``omni.physx.cct`` on its own -- it is in the extension startup log of
a run that passed no such flag -- so ``set_avatar_pose()`` takes the
``set_position()`` route as well as the USD write, and PhysX simulates the
capsule as a character controller from the moment Play is pressed. Two
consequences this run records rather than assumes: the capsule settles from
its authored z to its resting z, and the yaw written to its ``orient`` is put
back to identity every frame, which is why ``set_avatar_pose()`` aims the
cameras themselves.

``VP_CCT=1`` additionally constructs and activates a ``CharacterController``
on the capsule, the way the stage's own Controls graph does on Play, so the
controller is registered for certain rather than incidentally.

**Neither route is collide-and-slide.** A pose write can place the avatar
where no walk could have reached, including inside a shelf. Nothing here
asserts otherwise, and nothing here should be read as evidence about the
character controller's collision behaviour -- see ``set_avatar_pose``'s own
docstring, and the same caveat about the S11 scripted walk in
``sim/spikes/FINDINGS.md``.

Run::

    docker compose -f docker/docker-compose.yml run --rm sim \\
        ./runheadless.sh --exec /workspace/sim/verify_avatar_pose.py

    # ...and `docker stop` it afterwards: an exec-mode container does not exit
    # when the script prints DONE, and it holds :8011 and ~1.6 GB of VRAM.

Environment (argv is ambiguous after ``--exec``, so config is env vars):

    VP_STAGE     stage to open   (default: /workspace/sim/observatory_avatar.usd)
    VP_OUT       results directory (default: the logs volume)
    VP_WARMUP    max frames to wait for every sensor to fill (default 300)
    VP_SETTLE    frames between writing a pose and sampling  (default 30)
    VP_ARC_DEG   how far to swing the avatar around the station (default 12)
    VP_CCT       1 to also drive the character-controller route (default 0)
"""

from __future__ import annotations

import json
import math
import os
import sys
import time as _time
import traceback
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

REPO = Path(__file__).resolve().parent.parent
HERE = Path(__file__).resolve().parent
for _p in (str(REPO), str(HERE)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Importing either sibling must not run its own capture. Set BEFORE the
# imports, never after -- see sensor_factory._is_exec_entrypoint and
# observation_adapter._exec_entrypoint. Getting this wrong is silent in the
# expensive direction: merely importing observation_adapter would open a
# stage, run the contract suite and post_quit() this session.
os.environ.setdefault("SF_NO_AUTORUN", "1")
os.environ.setdefault("OA_NO_AUTORUN", "1")

import avatar as av  # noqa: E402  -- sibling module, see sys.path above
import observation_adapter as oa  # noqa: E402
import sensor_factory as sf  # noqa: E402
from core.observation import Modality, MountType  # noqa: E402

STAGE = os.environ.get("VP_STAGE", str(REPO / "sim" / "observatory_avatar.usd"))
OUT_DIR = Path(os.environ.get("VP_OUT", "/isaac-sim/.nvidia-omniverse/logs"))
WARMUP_FRAMES = int(os.environ.get("VP_WARMUP", "300"))
SETTLE_FRAMES = int(os.environ.get("VP_SETTLE", "30"))
ARC_DEG = float(os.environ.get("VP_ARC_DEG", "12"))
USE_CCT = os.environ.get("VP_CCT") == "1"

AVATAR = "/Root/Avatar"

# Tolerances. The pose ones are tight because a USD write is exact: anything
# above float noise means something else wrote the transform after we did,
# which is the failure this script exists to catch.
POS_TOL_M = 1e-3
YAW_TOL_DEG = 0.5
#: How far the character may sit from the capsule. Not float noise -- the
#: character is a sibling that FOLLOWS, so a frame of lag is legal.
BODY_GAP_TOL_M = 0.05
#: Half-width of the box a waypoint's lidar returns are counted in. The avatar
#: is ~0.6 m wide, and the waypoints are metres apart, so the boxes are
#: disjoint.
BOX_HALF_M = 0.60
#: Vertical band the box spans, metres above the floor. Starts above the floor
#: so floor returns under the avatar are not counted as the avatar, and stops
#: below the racking.
BOX_Z_M = (0.30, 1.80)
#: How much the diagonal has to beat the off-diagonal by. Not a tuned number:
#: it is the difference between "there is something here" and "the something
#: here is the avatar", and the avatar is the only thing that moved.
CONTRAST = 5.0
#: Pixels the `person` centroid must move between two waypoints to count.
CENTROID_TOL_PX = 5.0

#: Example_Rotary's elevation band, read from the shipped beam profile by
#: sensor_factory. A waypoint outside it is invisible to the lidar, and a run
#: built on invisible waypoints must report itself vacuous, not green.
EL_MIN, EL_MAX = sf.LIDAR_EL_MIN_DEG, sf.LIDAR_EL_MAX_DEG


def log(msg: str) -> None:
    print(f"[verify_avatar_pose] {msg}", flush=True)


# ---------------------------------------------------------------------------
# Checks: collect, never stop at the first. Five failures in one run beats
# five runs -- and this run costs a stage load and a 300-frame warm-up.
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
        print("AVATAR POSE VERIFICATION", flush=True)
        print("=" * 78, flush=True)
        for ok, name, detail in self.rows:
            print(f"  {'PASS' if ok else 'FAIL'}  {name:<46s} {detail}", flush=True)
        for line in self.vacuous:
            print(f"  ????  {line}", flush=True)
        print("=" * 78, flush=True)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def world_translation(prim, cache) -> tuple[float, float, float]:
    cache.Clear()
    t = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    return (float(t[0]), float(t[1]), float(t[2]))


def camera_heading_deg(prim, cache) -> float:
    """Where a USD camera looks, as a heading about +Z with 0 = world +X.

    A USD camera looks down its own -Z. Taking the heading from the transformed
    -Z rather than from the authored rotateXYZ is what makes this an
    INDEPENDENT check of set_avatar_pose()'s yaw: it goes through USD's
    definition of a camera, not through avatar.py's arithmetic.
    """
    cache.Clear()
    m = cache.GetLocalToWorldTransform(prim)
    fwd = m.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
    return math.degrees(math.atan2(float(fwd[1]), float(fwd[0])))


def body_heading_deg(prim, cache) -> float:
    """The visible character's heading, from its own local -Y.

    -Y because that is the direction the Worker asset's toes lead, measured
    off the rig in sim/avatar.py. Used ONLY in differences (this heading now
    minus this heading before), so the choice of reference axis cancels and
    the check stays convention-free.
    """
    cache.Clear()
    m = cache.GetLocalToWorldTransform(prim)
    fwd = m.TransformDir(Gf.Vec3d(0.0, -1.0, 0.0))
    return math.degrees(math.atan2(float(fwd[1]), float(fwd[0])))


def angle_delta(a: float, b: float) -> float:
    """Signed a - b, wrapped to (-180, 180]. Degrees do not subtract."""
    return (a - b + 180.0) % 360.0 - 180.0


def rotate_about(point_xy, centre_xy, degrees: float) -> tuple[float, float]:
    """Swing `point_xy` around `centre_xy`, preserving the distance between them.

    Preserving the distance is the whole reason the waypoints are built this
    way rather than picked. A rotary lidar only sees an elevation band --
    Example_Rotary sweeps -15..+10 deg -- so a mount at height h only sees a
    target at horizontal distance d >= (h - z) / tan(15 deg). Waypoints on an
    arc centred on the station are all at the same d, so if the avatar starts
    in band, every waypoint is in band by construction, and the run cannot
    fail because the avatar walked out of the sensor's reach.
    """
    t = math.radians(degrees)
    dx, dy = point_xy[0] - centre_xy[0], point_xy[1] - centre_xy[1]
    return (centre_xy[0] + dx * math.cos(t) - dy * math.sin(t),
            centre_xy[1] + dx * math.sin(t) + dy * math.cos(t))


def elevation_deg(sensor_xyz, target_xyz) -> tuple[float, float]:
    """(horizontal distance, elevation as the SENSOR sees the target)."""
    dx = target_xyz[0] - sensor_xyz[0]
    dy = target_xyz[1] - sensor_xyz[1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return d, -90.0
    return d, math.degrees(math.atan2(target_xyz[2] - sensor_xyz[2], d))


def box_at(xy) -> tuple[Gf.Vec3d, Gf.Vec3d]:
    return (Gf.Vec3d(xy[0] - BOX_HALF_M, xy[1] - BOX_HALF_M, BOX_Z_M[0]),
            Gf.Vec3d(xy[0] + BOX_HALF_M, xy[1] + BOX_HALF_M, BOX_Z_M[1]))


def person_ids(labels: dict) -> list[int]:
    """The class ids that mean `person`, from the reading's own label map.

    Read from the map rather than assumed: `semantic` holds class ids and the
    contract only promises that every id appearing in the map has a name --
    not that the numbers are stable between runs, which they are not.
    """
    out = []
    for key, value in (labels or {}).items():
        name = value.get("class") if isinstance(value, dict) else value
        if isinstance(name, str) and "person" in name.lower():
            try:
                out.append(int(key))
            except (TypeError, ValueError):
                continue
    return out


def person_centroid(semantic, labels):
    """(column, row, count) of the `person` pixels, or None if there are none."""
    ids = person_ids(labels)
    if semantic is None or not ids:
        return None
    mask = np.isin(np.asarray(semantic), ids)
    count = int(mask.sum())
    if count == 0:
        return None
    rows, cols = np.nonzero(mask)
    return (float(cols.mean()), float(rows.mean()), count)


# ---------------------------------------------------------------------------
# The run
# ---------------------------------------------------------------------------
class Run:
    """loading -> warmup -> waypoints -> verdict."""

    def __init__(self, results: sf.Results) -> None:
        self.results = results
        self.checks = Checks(results)
        self.phase = "loading"
        self.frame = 0
        self.warm = 0
        self.settle = 0
        self.index = -1
        self.ctx = omni.usd.get_context()
        self.source = None
        self.follow_sub = None
        self.cct = None
        self.waypoints: list[dict] = []
        self.samples: list[dict] = []
        self.warm_missing: dict = {}
        self.lidar_id = None
        self.camera_id = None
        self.cache = UsdGeom.XformCache()
        self.sub = None

    # -- setup -------------------------------------------------------------
    def setup(self) -> None:
        stage = self.ctx.get_stage()
        log(f"stage: {len(list(stage.Traverse()))} prims")

        registry = sf.load_registry()
        # The robot platforms are deliberately NOT referenced in. They are
        # authored at runtime by sf.reference_robots(), so leaving that call
        # out leaves their prim paths absent and sensor_factory skips the
        # BOT_* specs exactly as it does before S9 -- three fewer render
        # products to warm up, and nothing lost: the claim under test is that
        # a sensor which did not move sees an avatar that did, and a static
        # bystander is not part of it.
        for path, pos in sf.create_stations(stage).items():
            log(f"station {path} at {[round(v, 3) for v in pos]}")

        # Capture mode (CLAUDE.md rule 6): no collider mask, no raised
        # minFrameRate, render products at the registry's declared resolution.
        created = sf.create_registry_sensors(stage, registry)
        if not created:
            raise RuntimeError("no sensors were created -- nothing to observe")
        self.results.write(
            event="sensors_created",
            sensors={k: {"prim_path": v["prim_path"], "kind": v["kind"]}
                     for k, v in created.items()})

        self.follow_sub = av.install_character_follow(stage, AVATAR)
        if self.follow_sub is None:
            log("! character follow NOT installed -- set_avatar_pose writes the "
                "character's transform itself, so the pose checks still hold, "
                "but nothing will correct it between writes")

        # No advance_world and no action_source: this script owns the pose, and
        # a trajectory callback would move the avatar out from under it.
        self.source = oa.IsaacObservationSource(stage, registry, created)
        log(f"sensors: {', '.join(self.source.sensor_ids)}")

        # FIXED mounts only. The claim under test is that a sensor which did
        # not move sees an avatar that did; an avatar-mounted camera would
        # report a changed image for the trivial reason that it went along.
        for sensor_id in self.source.sensor_ids:
            spec = registry.get(sensor_id)
            if spec.mount is not MountType.FIXED:
                continue
            if self.lidar_id is None and spec.modality is Modality.LIDAR:
                self.lidar_id = sensor_id
            if self.camera_id is None and "semantic_segmentation" in spec.annotators:
                self.camera_id = sensor_id
        log(f"fixed lidar: {self.lidar_id}   fixed camera: {self.camera_id}")

        if USE_CCT:
            self._activate_cct(stage)

        # THE PLAY, and it is not a formality -- see failure mode 10 and the
        # module docstring. Before this line every annotator returns an empty
        # buffer that reads exactly like a sensor seeing nothing.
        #
        # Whether it TOOK is checked on the next frame, not here. play() is
        # asynchronous: measured 2026-08-26, is_playing() is still False on
        # the line after the call and True by the first update. Asserting it
        # synchronously fails a run that is working -- which is worse than not
        # asserting it, because it teaches whoever sees it to ignore the
        # check that guards failure mode 10.
        omni.timeline.get_timeline_interface().play()
        log("play() called; is_playing() is checked on the next frame")

    def _activate_cct(self, stage) -> None:
        """VP_CCT=1: put a real character controller on the capsule."""
        from isaacsim.core.experimental.utils.app import enable_extension

        log("VP_CCT=1: enabling omni.physx.cct. This composes the warehouse's "
            "colliders into PhysX and can cost minutes.")
        enable_extension("omni.physx.cct")
        from omni.physxcct.scripts import utils as cct_utils

        # Same construction OgnCharacterController.activate() performs when the
        # stage's own Controls graph fires on Play, made explicit so the run
        # does not depend on the graph having evaluated yet.
        self.cct = cct_utils.CharacterController(f"{AVATAR}/body_mesh", None, True, 0.01)
        self.cct.activate(stage)
        log("character controller activated on the capsule")

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

    # -- waypoints ---------------------------------------------------------
    def plan(self) -> None:
        """Build the waypoints from what is on the stage, not from constants.

        The station's position comes out of the reading it just produced, the
        avatar's out of its capsule, and the waypoints are arcs of the one
        about the other. Nothing here is a coordinate somebody typed -- a
        guessed free-space coordinate fails the same way a guessed prim path
        does, except that it lands the avatar inside a shelf instead of
        nowhere, and the sensors then report a plausible nothing.
        """
        stage = self.ctx.get_stage()
        body = stage.GetPrimAtPath(f"{AVATAR}/body_mesh")
        start = world_translation(body, self.cache)

        poses = {obs.sensor_id: obs.pose.position for obs in self.source.sample_now()}
        station = poses.get(self.lidar_id) or poses.get(self.camera_id)
        if station is None:
            raise RuntimeError(
                "no reading from a fixed sensor to take the station pose from; "
                f"lidar={self.lidar_id} camera={self.camera_id}")

        centre = (station[0], station[1])
        d, el = elevation_deg(station, (start[0], start[1], start[2]))
        log(f"station at {[round(v, 3) for v in station]}; avatar {d:.2f} m "
            f"away at elevation {el:+.2f} deg (band {EL_MIN}..{EL_MAX})")

        # The heading the avatar was BUILT facing, measured off its own camera
        # rather than read out of avatar.py's constants. Waypoint yaws are
        # relative to it, so W0 asks set_avatar_pose() to reproduce a heading
        # the stage already holds -- if the two conventions disagree by so
        # much as half a degree, W0 fails and says by how much.
        cam_fp = stage.GetPrimAtPath(f"{AVATAR}/body_mesh/cam_first_person")
        base = camera_heading_deg(cam_fp, self.cache) if cam_fp.IsValid() else 0.0
        log(f"built heading (from {AVATAR}/body_mesh/cam_first_person): {base:.2f} deg")

        plan = [
            ("W0 start", 0.0, base),
            ("W1 +arc", +ARC_DEG, base + 90.0),
            ("W2 -arc", -ARC_DEG, base - 90.0),
            # Back to W0's pose, exactly. The check that "repeatable" is a
            # measurement and not a hope: same pose in, same readings out.
            ("W3 back to start", 0.0, base),
        ]
        for name, swing, yaw in plan:
            xy = rotate_about((start[0], start[1]), centre, swing)
            dist, elev = elevation_deg(station, (xy[0], xy[1], start[2]))
            in_band = EL_MIN <= elev <= EL_MAX
            self.waypoints.append({
                "name": name, "xy": xy, "yaw_deg": yaw, "swing_deg": swing,
                "distance_m": dist, "elevation_deg": elev, "in_band": in_band,
            })
            log(f"  {name}: ({xy[0]:.3f}, {xy[1]:.3f}) yaw {yaw:.1f} deg  "
                f"d={dist:.2f} m elev={elev:+.2f} deg "
                f"{'IN BAND' if in_band else 'OUT OF BAND'}")
        self.results.write(event="waypoints", waypoints=[
            {k: v for k, v in w.items()} for w in self.waypoints])

        if not all(w["in_band"] for w in self.waypoints):
            self.checks.cannot_run(
                "lidar sees every waypoint",
                f"a waypoint is outside {self.lidar_id}'s {EL_MIN}..{EL_MAX} deg "
                f"elevation band, so an empty cloud there would say nothing "
                f"about the pose write")

    def write_pose(self) -> None:
        """Write the next waypoint's pose and start its settle countdown."""
        self.index += 1
        wp = self.waypoints[self.index]
        stage = self.ctx.get_stage()
        report = av.set_avatar_pose(stage, wp["xy"], wp["yaw_deg"],
                                    avatar_path=AVATAR, verbose=True)
        self.results.write(event="set_pose", waypoint=wp["name"], **report)
        wp["report"] = report
        self.settle = SETTLE_FRAMES

    def profile_frame(self) -> None:
        """One row of the lidar's catch-up profile for the current waypoint.

        MEASURED, because the alternative is a magic constant. The first run
        of this script used a 4-frame settle and every lidar reading after the
        first came back describing the PREVIOUS waypoint -- a full waypoint
        stale, while the camera at the same station tracked perfectly. That is
        the RTX lidar's own latency, and it is invisible in a single reading:
        a cloud of the wrong moment is a perfectly good cloud.

        So rather than pick a settle count and hope, this records, for every
        frame of the settle, how many returns are in the box the avatar is
        standing in now against the box it just left. The frame at which those
        cross over is the latency, this run, on this host -- and the verdict
        checks the settle budget against it instead of the other way round.
        """
        if self.lidar_id is None or self.index <= 0:
            return
        here = box_at(self.waypoints[self.index]["xy"])
        prev = box_at(self.waypoints[self.index - 1]["xy"])
        for obs in self.source.sample_now():
            if obs.sensor_id != self.lidar_id:
                continue
            points = obs.data.get("points")
            arr = np.asarray(points) if points is not None and len(points) else None
            row = [SETTLE_FRAMES - self.settle,
                   0 if arr is None else sf.count_in_box(arr, *here, pad=0.0),
                   0 if arr is None else sf.count_in_box(arr, *prev, pad=0.0)]
            self.waypoints[self.index].setdefault("settle_profile", []).append(row)

    @staticmethod
    def _capsule_orient(body) -> list | None:
        """The capsule's authored xformOp:orient as [w, x, y, z], or None."""
        attr = body.GetAttribute("xformOp:orient")
        q = attr.Get() if attr else None
        if q is None:
            return None
        i = q.GetImaginary()
        return [round(float(q.GetReal()), 6), round(float(i[0]), 6),
                round(float(i[1]), 6), round(float(i[2]), 6)]

    def sample(self) -> None:
        """Read the stage back, then read the sensors. Main thread, post-frame."""
        wp = self.waypoints[self.index]
        stage = self.ctx.get_stage()
        body = stage.GetPrimAtPath(f"{AVATAR}/body_mesh")
        char = stage.GetPrimAtPath(f"{AVATAR}/character")
        cam_fp = stage.GetPrimAtPath(f"{AVATAR}/body_mesh/cam_first_person")

        record: dict = {
            "waypoint": wp["name"],
            "requested_xy": wp["xy"],
            "requested_yaw_deg": wp["yaw_deg"],
            "body_xyz": world_translation(body, self.cache),
            "char_xyz": world_translation(char, self.cache) if char.IsValid() else None,
            "cam_heading_deg": camera_heading_deg(cam_fp, self.cache) if cam_fp.IsValid() else None,
            "body_heading_deg": body_heading_deg(char, self.cache) if char.IsValid() else None,
            "playing": bool(omni.timeline.get_timeline_interface().is_playing()),
            # The capsule's AUTHORED orientation, read straight back off the
            # attribute set_avatar_pose() wrote. Recorded because "the yaw did
            # not reach the renderer" has two very different causes -- the
            # write was lost, or the write survived and the world transform
            # ignores it -- and only the attribute can tell them apart. What
            # this measured: physics puts identity back here every frame while
            # the timeline is playing, because it simulates this prim as a
            # character controller and a controller has no orientation.
            "capsule_orient": self._capsule_orient(body),
        }

        for obs in self.source.sample_now():
            if obs.sensor_id == self.lidar_id:
                points = obs.data.get("points")
                record["lidar_points"] = 0 if points is None else int(len(points))
                record["lidar_frame"] = (obs.intrinsics or {}).get("frame")
                counts = []
                for other in self.waypoints:
                    lo, hi = box_at(other["xy"])
                    counts.append(0 if points is None or not len(points)
                                  else sf.count_in_box(np.asarray(points), lo, hi, pad=0.0))
                record["box_counts"] = counts
            elif obs.sensor_id == self.camera_id:
                centroid = person_centroid(obs.data.get("semantic"),
                                           obs.data.get("semantic_labels"))
                record["person"] = centroid
                rgb = obs.data.get("rgb")
                # A cheap, order-independent fingerprint of the image. Not a
                # hash: two poses producing the same mean would still be
                # separated by the person centroid above, and a hash could not
                # report HOW different two frames are.
                record["rgb_mean"] = (None if rgb is None
                                      else [round(float(v), 4) for v in
                                            np.asarray(rgb).reshape(-1, 3).mean(axis=0)])

        self.samples.append(record)
        self.results.write(event="sample", **record)
        heading = record["cam_heading_deg"]
        log(f"  {wp['name']}: body={[round(v, 3) for v in record['body_xyz']]} "
            f"cam_heading={'n/a' if heading is None else f'{heading:.2f}'} deg "
            f"boxes={record.get('box_counts')} person={record.get('person')}")

    # -- verdict -----------------------------------------------------------
    def verdict(self) -> None:
        c = self.checks
        c.check(not self.warm_missing,
                "every sensor filled during warm-up",
                f"{self.warm} frames; still empty: {self.warm_missing or 'nothing'}")

        for i, rec in enumerate(self.samples):
            name = rec["waypoint"]
            want_xy = rec["requested_xy"]
            got = rec["body_xyz"]
            err = math.hypot(got[0] - want_xy[0], got[1] - want_xy[1])
            c.check(err <= POS_TOL_M, f"{name}: the capsule is where it was put",
                    f"error {err * 1000:.3f} mm")
            c.check(rec["playing"], f"{name}: sampled while playing",
                    "failure mode 10")

            if rec["cam_heading_deg"] is None:
                c.cannot_run(f"{name}: yaw", "no first-person camera on the stage")
            else:
                dy = angle_delta(rec["cam_heading_deg"], rec["requested_yaw_deg"])
                c.check(abs(dy) <= YAW_TOL_DEG,
                        f"{name}: the view faces the requested yaw",
                        f"asked {rec['requested_yaw_deg']:.1f}, camera "
                        f"{rec['cam_heading_deg']:.2f}, off by {dy:+.2f} deg")

            if rec["char_xyz"] is None:
                c.cannot_run(f"{name}: visible body", "no character prim")
            else:
                gap = math.hypot(rec["char_xyz"][0] - got[0],
                                 rec["char_xyz"][1] - got[1])
                c.check(gap <= BODY_GAP_TOL_M,
                        f"{name}: the visible body went with the capsule",
                        f"horizontal gap {gap:.4f} m "
                        f"(this, not the capsule, is what the sensors trace)")

            # Turning: relative, so it needs no convention. Skipped at i == 0,
            # which has nothing to be relative to.
            if i > 0 and rec["body_heading_deg"] is not None:
                prev = self.samples[i - 1]
                if prev["body_heading_deg"] is not None and prev["cam_heading_deg"] is not None:
                    turned_body = angle_delta(rec["body_heading_deg"], prev["body_heading_deg"])
                    turned_cam = angle_delta(rec["cam_heading_deg"], prev["cam_heading_deg"])
                    c.check(abs(angle_delta(turned_body, turned_cam)) <= YAW_TOL_DEG,
                            f"{name}: body and view turned by the same angle",
                            f"body {turned_body:+.2f} deg, view {turned_cam:+.2f} deg")

        self._report_capsule_orient()
        self._verdict_settle()
        self._verdict_lidar()
        self._verdict_camera()
        self._verdict_repeatable()

    def _report_capsule_orient(self) -> None:
        """Say out loud what happened to the yaw written to the capsule.

        Reported, not asserted, because it is a fact about PhysX rather than
        about set_avatar_pose(): while the timeline is playing, the capsule's
        orient reads back as identity at every waypoint no matter what was
        written, because PhysX simulates the prim as a character controller
        and a controller has a position and an up direction and no rotation.
        MEASURED 2026-08-26; before that it was a plausible explanation for a
        first-person camera that never turned.

        There IS a check behind this, and it is the yaw check above: if the
        orient ever starts surviving, the capsule's rotation and the cameras'
        own aim would BOTH apply, the view would turn twice as far as asked,
        and "the view faces the requested yaw" fails and says by how much.
        """
        seen = [rec.get("capsule_orient") for rec in self.samples]
        asked = [rec["requested_yaw_deg"] for rec in self.samples]
        identity = all(q is not None and abs(q[0] - 1.0) < 1e-6 and
                       abs(q[3]) < 1e-6 for q in seen)
        note = ("reverted to identity at every waypoint -- PhysX owns this "
                "prim's rotation while playing" if identity else
                "survived; the cameras' own aim is what turned the view")
        log(f"  capsule xformOp:orient readback: {seen} for yaws {asked} -- {note}")
        self.results.write(event="capsule_orient", readback=seen,
                           requested_yaw_deg=asked, all_identity=identity)

    def _verdict_settle(self) -> None:
        """Did the lidar catch up within the settle budget it was given?

        Reported before the lidar checks themselves, because it is the
        difference between "the pose write did not reach the sensor" and "the
        sensor was still describing the previous frame when it was asked".
        Those two look identical in a box count and are not the same bug.
        """
        c = self.checks
        if self.lidar_id is None:
            return
        for wp in self.waypoints[1:]:
            profile = wp.get("settle_profile") or []
            if not profile:
                continue
            crossed = next((f for f, here, prev in profile if here > prev and here > 0), None)
            trace = " ".join(f"{f}:{here}/{prev}" for f, here, prev in profile)
            self.results.write(event="settle_profile", waypoint=wp["name"],
                               crossed_at=crossed, profile=profile)
            c.check(crossed is not None,
                    f"{wp['name']}: the lidar caught up within {SETTLE_FRAMES} frames",
                    f"crossover at frame {crossed}; here/there by frame: {trace}")

    def _verdict_lidar(self) -> None:
        c = self.checks
        if self.lidar_id is None:
            c.cannot_run("fixed lidar tracks the avatar",
                         "no FIXED lidar was created from the registry")
            return
        matrix = [rec.get("box_counts") for rec in self.samples]
        if not matrix or any(m is None for m in matrix):
            c.cannot_run("fixed lidar tracks the avatar",
                         f"{self.lidar_id} produced no cloud at some waypoint")
            return

        # The boxes below are in WORLD metres. A cloud that arrived
        # sensor-local would put every return near the origin, the boxes would
        # all read zero, and the run would report a pose write that never
        # happened -- failure mode 2, one layer up. Ask the reading which
        # frame it is in rather than trusting it.
        frames = {rec.get("lidar_frame") for rec in self.samples}
        if frames != {"world"}:
            c.cannot_run(
                "fixed lidar tracks the avatar",
                f"{self.lidar_id} reported frame {frames}, not 'world'; box "
                f"counts in world coordinates would be meaningless")
            return

        print("\n  lidar returns per box (rows: where the avatar stood; "
              "columns: which waypoint's box)", flush=True)
        header = "  " + " ".join(f"{w['name'].split()[0]:>8s}" for w in self.waypoints)
        print(f"  {'':<18s}{header}", flush=True)
        for rec, row in zip(self.samples, matrix):
            cells = " ".join(f"{v:>8d}" for v in row)
            print(f"  {rec['waypoint']:<18s}  {cells}", flush=True)
        self.results.write(event="lidar_matrix", matrix=matrix,
                           waypoints=[w["name"] for w in self.waypoints])

        # W3 is W0's pose, so their boxes are the same box: comparing them
        # would be comparing a waypoint against itself and would fail by
        # construction. Only genuinely distinct places take part.
        for i, rec in enumerate(self.samples):
            here = matrix[i][i]
            if here <= 0:
                c.check(False, f"{rec['waypoint']}: lidar has returns on the avatar",
                        "0 points in the box around the commanded position")
                continue
            mine = self.waypoints[i]["xy"]
            for j, other in enumerate(self.waypoints):
                same_place = math.hypot(other["xy"][0] - mine[0],
                                        other["xy"][1] - mine[1]) < 2 * BOX_HALF_M
                if j == i or same_place:
                    continue
                there = matrix[i][j]
                c.check(here >= CONTRAST * max(there, 1),
                        f"{rec['waypoint']}: returns are HERE, not at "
                        f"{other['name'].split()[0]}",
                        f"{here} vs {there} points "
                        f"({here / max(there, 1):.1f}x, need {CONTRAST:.0f}x)")

    def _verdict_camera(self) -> None:
        c = self.checks
        if self.camera_id is None:
            c.cannot_run("fixed camera sees the avatar move",
                         "no FIXED camera with semantic_segmentation in the registry")
            return
        if len(self.samples) < 3:
            c.cannot_run("fixed camera sees the avatar move",
                         f"only {len(self.samples)} waypoints were reached")
            return
        people = [rec.get("person") for rec in self.samples]
        if any(p is None for p in people):
            missing = [rec["waypoint"] for rec, p in zip(self.samples, people) if p is None]
            c.cannot_run(
                "fixed camera sees the avatar move",
                f"{self.camera_id} found no `person` pixels at {missing} -- an "
                f"absent label is failure mode 6, not evidence about the pose")
            return

        cols = [p[0] for p in people]
        for i in (1, 2):
            shift = abs(cols[i] - cols[0])
            c.check(shift >= CENTROID_TOL_PX,
                    f"{self.samples[i]['waypoint']}: the person moved in the image",
                    f"centroid column {cols[0]:.1f} -> {cols[i]:.1f} px "
                    f"({shift:.1f} px)")

        means = [rec.get("rgb_mean") for rec in self.samples]
        if means[0] is None or means[1] is None:
            c.cannot_run("fixed camera's rgb changed", "no rgb payload")
        else:
            delta = max(abs(a - b) for a, b in zip(means[0], means[1]))
            c.check(delta > 0.0, "fixed camera's rgb changed between poses",
                    f"max channel-mean delta {delta:.4f}")

        # Monotonic ordering is REPORTED, not asserted. Swinging the avatar
        # around the station's vertical axis is pure horizontal parallax, so
        # the three centroids should be ordered -- but the warehouse's own
        # Worker carries a `person` label too, and a second blob in frame
        # shifts a centroid without saying anything about this avatar.
        order = "increasing" if cols[1] > cols[0] > cols[2] else (
            "decreasing" if cols[1] < cols[0] < cols[2] else "NOT MONOTONIC")
        log(f"  person centroid columns W1/W0/W2: {cols[1]:.1f}/{cols[0]:.1f}/"
            f"{cols[2]:.1f} -- {order} (reported, not asserted)")
        self.results.write(event="centroid_order", columns=cols, order=order)

    def _verdict_repeatable(self) -> None:
        """W3 asked for W0's pose again. Did the world come back to W0?"""
        c = self.checks
        if len(self.samples) < 4:
            c.cannot_run("the same pose reproduces the same readings",
                         "the run did not reach the fourth waypoint")
            return
        first, last = self.samples[0], self.samples[-1]

        gap = math.hypot(last["body_xyz"][0] - first["body_xyz"][0],
                         last["body_xyz"][1] - first["body_xyz"][1])
        c.check(gap <= POS_TOL_M,
                "the fourth call lands exactly where the first did",
                f"{gap * 1000:.3f} mm apart, after three intervening writes")

        a, b = first.get("box_counts"), last.get("box_counts")
        if not a or not b:
            c.cannot_run("the same pose reproduces the same lidar reading",
                         "no cloud at one of the two waypoints")
        else:
            # 40%: the lidar's returns per box vary tick to tick because the
            # sweep is not phase-locked to the sample. The claim is that the
            # avatar came back, not that the buffer is byte-identical.
            ratio = b[0] / max(a[0], 1)
            c.check(0.6 <= ratio <= 1.4,
                    "the same pose reproduces the same lidar reading",
                    f"{a[0]} -> {b[0]} points in W0's box ({ratio:.2f}x)")

        pa, pb = first.get("person"), last.get("person")
        if pa and pb:
            shift = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
            c.check(shift <= 4 * CENTROID_TOL_PX,
                    "the same pose reproduces the same camera reading",
                    f"person centroid moved {shift:.1f} px between the two")

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
                    self.plan()
                    self.write_pose()
                    self.phase = "waypoints"
                return

            if self.phase == "waypoints":
                # Settle before sampling: the pose was written on a previous
                # frame and the renderer has to have traced it. Cheap
                # insurance against reading the last waypoint's frame and
                # calling it this one's.
                if self.settle > 0:
                    self.settle -= 1
                    self.profile_frame()
                    return
                self.sample()
                if self.index + 1 < len(self.waypoints):
                    self.write_pose()
                else:
                    self.finish()
                return
        except Exception as exc:
            log("FAILED: " + repr(exc))
            log(traceback.format_exc())
            self.results.write(event="error", error=repr(exc),
                               tb=traceback.format_exc())
            self.checks.check(False, "the run completed", repr(exc))
            self.finish()

    def finish(self) -> None:
        if self.phase == "done":
            return
        self.phase = "done"
        # The verdict is itself code, and code called from an error path must
        # not be the reason the run never reports. A crash in here is a FAILED
        # verification, not a session that hangs holding the port.
        try:
            self.verdict()
        except Exception as exc:                             # noqa: BLE001
            log("verdict failed: " + repr(exc))
            log(traceback.format_exc())
            self.checks.check(False, "the verdict ran", repr(exc))
        self.checks.report()

        # A vacuous check is fatal. A run whose discriminating check never
        # executed is not a passing run -- it is a run that did not ask.
        failed = self.checks.failed
        code = 1 if (failed or self.checks.vacuous) else 0
        # The route is read back off what set_avatar_pose() REPORTED doing,
        # never off VP_CCT. The first version of this line reported
        # "usd_transform" because the flag was off, while every call had in
        # fact also gone through set_position() -- runheadless.sh starts
        # omni.physx.cct on its own. A run that misreports its own
        # configuration is worse than one that does not report it.
        reports = [w.get("report") or {} for w in self.waypoints]
        via_cct = [bool(r.get("via_character_controller")) for r in reports]
        route = ("usd_transform+character_controller" if all(via_cct) and via_cct
                 else "usd_transform" if not any(via_cct)
                 else "mixed")
        summary = {
            "frames": self.frame,
            "warmup_frames": self.warm,
            "waypoints": len(self.samples),
            "checks": len(self.checks.rows),
            "failed": failed,
            "vacuous": self.checks.vacuous,
            "route": route,
            "cct_activated_by_this_script": USE_CCT,
            "exit_code": code,
        }
        if self.source is not None:
            summary["sensors"] = list(self.source.sensor_ids)
            self.source.close()
        self.results.write(event="summary", **summary)
        log("SUMMARY " + json.dumps(summary, default=str)[:3000])
        if code == 0:
            log(f"POSE VERIFICATION PASSED ({len(self.checks.rows)} checks, "
                f"route={summary['route']})")
        else:
            log(f"POSE VERIFICATION FAILED ({failed} failed, "
                f"{len(self.checks.vacuous)} could not run)")
        log(f"route: {route}. "
            + ("omni.physx.cct is up, so every pose also went through the "
               "controller's set_position(). " if any(via_cct) else
               "omni.physx.cct is not up, so the USD write was authoritative. ")
            + "NEITHER is collide-and-slide: a pose write is a placement, not "
              "a swept move, and says nothing about walking into a shelf.")
        log("DONE")
        self.sub = None
        omni.kit.app.get_app().post_quit(code)


def main() -> None:
    out = OUT_DIR / "verify_avatar_pose.jsonl"
    results = sf.Results(out)
    log(f"stage={STAGE} settle={SETTLE_FRAMES} arc={ARC_DEG} cct={USE_CCT}")
    log(f"results -> {out}")
    results.write(event="start", stage=STAGE, started=_time.time(),
                  settle=SETTLE_FRAMES, arc_deg=ARC_DEG, use_cct=USE_CCT)

    opened = omni.usd.get_context().open_stage(STAGE)
    ok, err = opened if isinstance(opened, tuple) else (opened, None)
    log(f"open_stage ok={ok} err={err}")
    results.write(event="open_stage", ok=bool(ok), err=str(err))

    run = Run(results)
    run.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        run.on_update, name="verify_avatar_pose")
    log("subscribed to the update stream")


def _is_exec_entrypoint() -> bool:
    """True when Kit --exec'd this file, false when something imports it.

    Same reasoning as sensor_factory._is_exec_entrypoint, and the same two bad
    outcomes: too strict and the verification runs zero waypoints while
    looking fine, too loose and importing this module opens a stage, presses
    Play and post_quit()s the session.
    """
    return os.environ.get("VP_NO_AUTORUN") != "1"


if _is_exec_entrypoint():
    main()
