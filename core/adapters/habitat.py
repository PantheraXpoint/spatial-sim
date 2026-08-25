"""
What a Habitat adapter would have to do. A SKETCH -- nothing here runs.

WHY THIS FILE EXISTS. The project's claim is that `core/` is simulator-
agnostic: the same memory module runs against Isaac Sim and against Habitat,
which is what makes a benchmark comparison possible at all. That claim is free
to check today and expensive to check after Layer 4 is built on top of it. So
this file writes down, in detail, the mapping a Habitat adapter would perform
-- and the four places where it could not.

WHAT IS DELIBERATELY NOT HERE. Habitat is not installed, not in
docker/requirements-dev.txt, and must not be. There is no implementation, no
`import habitat_sim`, and no test that runs one. The findings at the bottom of
this docstring are the deliverable; the code below is scaffolding for them.

THE IMPORT RULE, which is not optional. `scripts/check_layer_boundary.sh`
imports every module under `core/` on a machine with no simulator and fails on
ANY unresolvable import, not only omni/pxr/isaacsim. A module-scope
`import habitat_sim` here would break `make check` on both machines and in CI.
An implementation imports inside `__init__`, after checking, and says
something useful when it is absent.

HABITAT SPECIFICS ARE MARKED [VERIFY]. Everything below about habitat-sim was
read from the docs rather than from a running install, and the same rule that
governs the Isaac API-era trap applies here: check it against the version you
actually have before relying on it. Everything about OUR contract is checked
by tests/test_habitat_sketch.py and is not marked.

---------------------------------------------------------------------------
WHAT HABITAT HANDS YOU                                              [VERIFY]
---------------------------------------------------------------------------

`sim.get_sensor_observations()` returns `{uuid: array}` for the sensors of one
agent, all from a single render pass:

    color      (H, W, 4) uint8   RGBA -- four channels, not three
    depth      (H, W)    float32 axial z, and normalised to [0, 1] by default
                                 under habitat-lab (min_depth 0.0, max_depth
                                 10.0, normalize_depth True)
    semantic   (H, W)    uint32  INSTANCE ids -- two chairs get two ids

`sim.get_agent(i).get_state()` gives `.position` (3,) float32 and `.rotation`
(a quaternion object), plus `.sensor_states[uuid]` with the same two fields in
world frame. Habitat is y-up; our contract is z-up (`UP_AXIS`).

There is no lidar and no radar. There are no world-anchored sensors either:
every Habitat sensor hangs off an agent. Both are workable -- see
`HABITAT_PAYLOAD_SOURCES` and `MOUNT_STRATEGY` -- and neither is a hole in the
contract.

---------------------------------------------------------------------------
FINDINGS -- what the contract could not absorb
---------------------------------------------------------------------------

Writing this file was the point of task M4, and three of its four findings
have since been acted on. They are kept here rather than deleted: the record
of what a second simulator broke is the evidence that the contract is worth
anything, and the next adapter author should be able to see which parts were
designed and which were repaired.

1. THE CONTRACT HAD NOWHERE TO PUT AN ACTION -- an Isaac-shaped assumption.
   CLOSED: `Observation.action`.

   `ObservationSource.step(dt)` says "advance time and tell me what you saw".
   Isaac works that way: the world runs on a clock and you watch it. Habitat
   does not -- `sim.step(action)` is the API, and an agent does not move
   unless something tells it how.

   A Habitat adapter hides this by owning an action policy internally
   (`HabitatObservationSource.__init__` takes one below), so the protocol
   survives. What did not survive was the state/experience split:
   `core/memory/interfaces.py` calls an avatar reading EXPERIENCE because it
   has an action before it and a consequence after it, and `Observation`
   carried neither. A documented claim with no representation.

   `Observation.action` closes the representation half. STILL OPEN: Layer 4
   cannot reach through Layer 3 to DRIVE an agent, which is what running the
   memory module on Habitat would require. That is a `step()` signature
   question and nothing needs it yet.

2. `semantic` DID NOT SAY WHETHER IT HELD CLASS IDS OR INSTANCE IDS.
   CLOSED: `SEMANTIC_ID_CONVENTION`, plus
   `test_every_semantic_id_in_the_map_has_a_name` in tests/contract.py.

   Isaac's annotator returns class ids with an id->label mapping; Habitat's
   sensor returns instance ids needing `sim.semantic_scene` per dataset. The
   same shape as the `depth` ambiguity DEPTH_CONVENTION closed: a portable key
   name that stopped carrying its definition. Both maps passed every contract
   test, including the one looking for "person", which found it in the label
   mapping either way. The completeness assertion is what separates them
   without the contract having to see the scene.

3. `rgb` CHANNEL COUNT AND DTYPE WERE ENFORCED BY A TEST AND WRITTEN DOWN
   NOWHERE. CLOSED: the shape-and-dtype table under `ANNOTATOR_DATA_KEYS`,
   and a dtype assertion alongside the existing shape one.

4. The registry demands an Isaac profile name. OPEN, deliberately -- to be
   decided alongside the radar question.

   `_validate_spec` in core/registry.py rejects a lidar or radar with no
   `config`, quoting "Example_Rotary". That is Layer 2, not the observation
   contract, and a Habitat source overrides the `registry` fixture, so it is a
   wart rather than a wall -- but a Habitat pseudo-lidar would have to invent
   an Isaac-flavoured profile name to satisfy it.

NOT A FINDING, worth recording so nobody re-derives it: `Modality.RADAR` is
unservable and `Modality.LIDAR` only servable by unprojecting depth. That is
Habitat lacking a sensor, not the contract over-reaching. The contract asks a
range sensor for `points` and nothing else; `radial_velocities` and `rcs` are
extras the mock adds, which is exactly why extras belong in `data` and not in
`MODALITY_DATA_KEYS`.

ALREADY CAUGHT, no action needed: habitat-lab's normalised [0, 1] depth is
rejected by `test_depth_is_a_metric_range_and_not_a_normalised_buffer`, and
its y-up poses by the two pose tests. Those three guards were written for a
hypothetical adapter and this is the adapter they were hypothesising about.

NOTHING IN THIS MODULE MAY IMPORT omni, pxr, isaacsim, OR habitat_sim.
"""

from __future__ import annotations

from typing import Any

from core.observation import Modality, MountType, Observation, Pose

_SKETCH = (
    "core/adapters/habitat.py is a sketch (task M4). It establishes what a "
    "Habitat adapter would have to map, not how. Nothing here is implemented "
    "and Habitat is deliberately not installed -- see the module docstring."
)

#: Index of the up-axis in Habitat's world frame. Ours is `UP_AXIS` == 2.
#: An adapter swizzles the components AND rotates the quaternion; relabelling
#: one without the other mirrors the scene. See `pose_from_agent_state`.
HABITAT_UP_AXIS = 1

#: Payload key -> what a Habitat adapter has to do to produce it.
#: Read as the adapter's job description. Every key here is a key the contract
#: already defines; tests/test_habitat_sketch.py checks that it stays true.
HABITAT_PAYLOAD_SOURCES: dict[str, str] = {
    "rgb": (
        "color sensor, (H, W, 4) uint8 RGBA -> drop the alpha channel"
    ),
    "depth": (
        "depth sensor, (H, W) float32. Two conversions, both silent if "
        "skipped: de-normalise from [0, 1] back to metres using min_depth "
        "and max_depth, then axial z -> euclidean range per DEPTH_CONVENTION"
    ),
    "semantic": (
        "semantic sensor, (H, W) instance ids -> class ids via "
        "sim.semantic_scene.objects[id].category, per SEMANTIC_ID_CONVENTION. "
        "Forwarding the instance ids raw now fails a contract test rather "
        "than passing quietly -- see finding 2"
    ),
    "points": (
        "no native range sensor. Unproject the depth buffer through the "
        "camera intrinsics; the result is a frustum, not a 360-degree sweep, "
        "so it is a pseudo-lidar and should be labelled one"
    ),
}

#: Modalities a Habitat adapter could serve, and what it would serve them with.
SERVABLE_MODALITIES: frozenset[Modality] = frozenset({
    Modality.RGB,
    Modality.RGBD,
    Modality.DEPTH,
    Modality.SEMANTIC,
    Modality.LIDAR,      # pseudo-lidar, unprojected from depth
})

#: Modalities it could not, and why. Every modality in the contract must be in
#: exactly one of these two collections -- a test enforces it, so adding a
#: modality forces someone to decide what Habitat would do with it rather than
#: discovering the answer years later.
UNSERVABLE_MODALITIES: dict[Modality, str] = {
    Modality.RADAR: (
        "Habitat has no radar and no doppler. Unlike lidar this cannot be "
        "faked from depth: range comes out of the depth buffer, but radial "
        "velocity and RCS are properties of a sensor model that does not "
        "exist there. A Habitat run is a three-station scene with two "
        "modalities, and any comparison has to say so."
    ),
}

#: How our three mount types would be realised. Habitat attaches every sensor
#: to an agent, so a "fixed" station is an agent that is never given an action
#: -- which is the same trick the scene already plays with the robots, one
#: layer down. [VERIFY] multi-agent configuration and per-agent observation
#: retrieval against the installed habitat-sim.
MOUNT_STRATEGY: dict[MountType, str] = {
    MountType.FIXED: "an agent parked at the station pose, never acted on",
    MountType.ROBOT: "likewise -- our robots do not move either",
    MountType.AVATAR: "the one agent the action policy actually drives",
}


class HabitatObservationSource:
    """
    Satisfies `core.observation.ObservationSource`, against Habitat. SKETCH.

    Every method below raises. The signatures and the docstrings are the
    deliverable: they are what an implementation would have to satisfy, and
    writing them is what surfaced the findings in the module docstring.

    Construction takes the two things the protocol has no room for:

      `bindings`  our sensor_id -> (habitat agent index, sensor uuid). The
                  registry cannot supply this; Habitat uuids are its own.
      `act`       what to do each step. See finding 1 -- this parameter is the
                  shape of the leak. Isaac needs no equivalent because its
                  world advances whether or not anyone acts on it.
    """

    def __init__(
        self,
        sim: Any,
        bindings: dict[str, tuple[int, str]],
        act: Any | None = None,
        *,
        seconds_per_step: float = 1.0 / 30.0,
    ) -> None:
        # Deliberately inert: no habitat_sim import, no connection, no
        # validation of `sim`. Constructing this proves only that the shape
        # fits the protocol, which is exactly what it is for.
        self._sim = sim
        self._bindings = dict(bindings)
        self._act = act
        self._seconds_per_step = seconds_per_step
        self._t = 0.0

    # --- ObservationSource ---------------------------------------------------

    # The only two members answerable without a simulator, so the only two
    # that are real here. They also HAVE to be: `isinstance(src,
    # ObservationSource)` against a runtime_checkable Protocol calls hasattr
    # on every member, hasattr evaluates a property, and a property that
    # raises anything other than AttributeError propagates. A source whose
    # properties raise therefore fails the contract's very first test with a
    # NotImplementedError rather than a readable failure -- worth knowing
    # before writing sim/observation_adapter.py (S11) the same way.

    @property
    def sensor_ids(self) -> tuple[str, ...]:
        """Our ids, not Habitat's uuids. Stable order, from `bindings`."""
        return tuple(self._bindings)

    @property
    def time(self) -> float:
        """
        Seconds since the source was created.

        Habitat has no clock unless physics is being stepped, so the adapter
        keeps its own and advances it by `seconds_per_step`. That is a
        bookkeeping fiction and should be labelled one: two sources agreeing
        on `timestamp` does not mean they agree on what a second is.
        """
        return self._t

    def step(self, dt: float | None = None) -> list[Observation]:
        """
        Act, render, convert.

        An implementation would: ask `self._act` what the avatar agent does
        this tick and apply it; call `get_sensor_observations` per agent;
        convert each array per `HABITAT_PAYLOAD_SOURCES`; read every agent's
        sensor states and convert each with `pose_from_agent_state`; stamp
        them all with one timestamp.

        `dt` is the awkward parameter. Habitat's discrete actions have no
        duration, so honouring `dt` means either stepping physics for `dt`
        seconds or quietly ignoring it. Ignoring it is defensible and must be
        documented, because a caller passing dt=0.25 will otherwise believe
        the avatar moved a quarter second's worth.
        """
        raise NotImplementedError(_SKETCH)

    def close(self) -> None:
        """`sim.close()`, idempotently."""
        raise NotImplementedError(_SKETCH)


# --- The conversions, one per finding-adjacent hazard -------------------------
# Signatures only. Each of these is a place where a plausible implementation
# silently produces wrong numbers rather than raising.


def pose_from_agent_state(position: Any, rotation: Any) -> Pose:
    """
    Habitat's y-up world pose -> ours, z-up metres, quaternion (w, x, y, z).

    Two things to get wrong. The axis swizzle is a rotation of the frame, not
    a permutation of a tuple: permuting alone flips handedness and mirrors the
    world, which renders perfectly and puts every left turn on the right. And
    the quaternion has to be carried through the same change of basis, not
    just reordered into (w, x, y, z).

    Habitat is already in metres, so `LENGTH_UNIT` needs no work here -- the
    one conversion on that list that Habitat gives away free.
    """
    raise NotImplementedError(_SKETCH)


def depth_to_euclidean_metres(
    axial: Any, intrinsics: dict[str, Any], *, normalised: bool = True
) -> Any:
    """
    Habitat depth -> `DEPTH_CONVENTION`: euclidean metres from the origin.

    Two conversions, in order, and both are invisible where you would first
    look. De-normalise: `axial * (max_depth - min_depth) + min_depth`, if the
    sensor was configured with normalize_depth (habitat-lab's default). Then
    axial -> euclidean with the formula in `core/observation.py`, which is a
    no-op at the principal point and grows toward the corners.

    Skip the first and every distance is a fraction under 1.0; the contract
    catches that today. Skip the second and the middle of the frame is right
    while the edges are short by a few percent; nothing catches that, which is
    why the constant exists.
    """
    raise NotImplementedError(_SKETCH)


def semantic_from_instance_ids(
    instances: Any, semantic_scene: Any
) -> tuple[Any, dict[int, str]]:
    """
    Habitat instance ids -> (class-id map, id->label mapping). See finding 2.

    `sim.semantic_scene.objects[instance_id].category` is the lookup, per
    dataset [VERIFY]. Returning the instance map unchanged with a label
    mapping bolted on would satisfy every test in tests/contract.py and be
    wrong in a way no consumer could detect.
    """
    raise NotImplementedError(_SKETCH)


def points_from_depth(euclidean: Any, intrinsics: dict[str, Any]) -> Any:
    """
    Unproject a depth buffer into an (N, 3) cloud -- a pseudo-lidar.

    Frustum-shaped, not a sweep: a Habitat "lidar" sees what the camera sees.
    Comparing it against a rotary lidar is comparing fields of view, not
    modalities, and any benchmark that does so should say which.

    Returns WORLD metres, per POINTS_FRAME in `core/observation.py`. That was
    an open question when this sketch was written and is now settled: the
    unprojection is naturally sensor-local, so this owes the sensor-to-world
    step exactly as `sim/observation_adapter.py` does -- and skipping it is
    invisible until two sensors are fused.
    """
    raise NotImplementedError(_SKETCH)
