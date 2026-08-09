"""
The ObservationSource contract, written against the protocol and nothing else.

THIS FILE IS THE HANDOFF FOR SERVER TASK S11. It never imports the mock, never
imports the simulator, and never touches an attribute that is not in
`core.observation.ObservationSource`. Subclass it, say how to build a source,
and the whole suite runs against it::

    # tests/test_mock_source.py                (MacBook, no GPU)
    class TestMockSource(ObservationSourceContract):
        def make_source(self):
            return MockObservationSource.from_config()

    # sim/tests/test_observation_adapter.py    (server, needs a stage)
    class TestIsaacSource(ObservationSourceContract):
        def make_source(self):
            return IsaacObservationSource(world, registry)

If the second one fails, the simulator does not satisfy the contract the
research code was written against -- which is exactly the thing worth finding
out by running a file rather than by arguing about it.

Deliberately NOT here: anything about values. This suite asserts shape,
typing, key presence, and the two invariants the whole design rests on (one
moving mount; sensors that react to it). Whether the pixels are pretty is not
a contract.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from core.observation import (
    ANNOTATOR_DATA_KEYS,
    LENGTH_UNIT,
    MODALITY_DATA_KEYS,
    UP_AXIS,
    Modality,
    MountType,
    Observation,
    ObservationSource,
)
from core.registry import CAMERA_MODALITIES, RANGE_MODALITIES, SensorRegistry

REGISTRY_PATH = "config/sensors.yaml"
SCENE_PATH = "config/scene.yaml"


def payload_scalar(obs: Observation) -> float:
    """
    One number summarising a reading, whatever its modality.

    Used only to ask "did this change?". Return count for range sensors, mean
    depth or mean intensity for cameras -- all of which move when something
    enters the field of view and none of which depend on how a particular
    source formats its arrays.
    """
    if "num_returns" in obs.data:
        return float(obs.data["num_returns"])
    if "points" in obs.data:
        return float(len(obs.data["points"]))
    for key in ("depth", "semantic", "rgb"):
        value = obs.data.get(key)
        if value is not None:
            return float(np.asarray(value, dtype=np.float64).mean())
    return 0.0


class ObservationSourceContract:
    """
    Every source must pass this. Override `make_source`; override `registry`
    too if the source is not driven by config/sensors.yaml.

    STEPS/STEP_DT are how much simulated time the motion tests get. The
    defaults walk the avatar about 14 m, which is far enough for a station's
    view of it to change substantially and short enough to stay quick.
    """

    STEPS = 40
    STEP_DT = 0.25

    def make_source(self) -> ObservationSource:
        raise NotImplementedError("subclasses say how to build their source")

    @pytest.fixture
    def source(self):
        source = self.make_source()
        try:
            yield source
        finally:
            source.close()

    @pytest.fixture
    def registry(self) -> SensorRegistry:
        return SensorRegistry.from_yaml(REGISTRY_PATH)

    @pytest.fixture
    def avatar_eye_height(self) -> float:
        """
        How high off the floor the avatar's cameras sit, in metres, as
        declared in config/scene.yaml. Both sources build the avatar from that
        file, so it is a fair thing for the contract to hold either of them to.
        """
        raw = yaml.safe_load(Path(SCENE_PATH).read_text())
        return float(raw["avatar"]["eye_height"])

    @pytest.fixture
    def trace(self, source) -> list[list[Observation]]:
        """One list of readings per tick. The motion tests share this."""
        return [source.step(self.STEP_DT) for _ in range(self.STEPS)]

    # --- The protocol itself -------------------------------------------------

    def test_satisfies_the_protocol(self, source):
        assert isinstance(source, ObservationSource)

    def test_declares_its_sensors(self, source):
        ids = source.sensor_ids
        assert ids, "a source with no sensors cannot be observed with"
        assert len(set(ids)) == len(ids), "duplicate sensor ids"
        assert all(isinstance(i, str) for i in ids)

    def test_time_starts_at_zero_and_advances(self, source):
        assert source.time == 0.0
        source.step(self.STEP_DT)
        assert source.time == pytest.approx(self.STEP_DT)

    def test_close_is_safe_to_call_twice(self, source):
        source.close()
        source.close()

    # --- Shape of a tick -----------------------------------------------------

    def test_a_tick_returns_only_declared_sensors(self, source):
        observations = source.step(self.STEP_DT)
        assert observations, "a tick produced nothing at all"
        ids = [o.sensor_id for o in observations]
        assert len(set(ids)) == len(ids), f"a sensor reported twice: {ids}"
        assert set(ids) <= set(source.sensor_ids), (
            f"undeclared sensor(s): {sorted(set(ids) - set(source.sensor_ids))}"
        )

    def test_a_tick_shares_one_timestamp(self, source):
        observations = source.step(self.STEP_DT)
        stamps = {o.timestamp for o in observations}
        assert len(stamps) == 1, f"readings from one tick disagree on time: {stamps}"
        assert stamps.pop() == pytest.approx(source.time)

    def test_timestamps_increase(self, source):
        first = source.step(self.STEP_DT)[0].timestamp
        second = source.step(self.STEP_DT)[0].timestamp
        assert second > first

    # --- Against the registry ------------------------------------------------

    def test_modality_and_mount_come_from_the_registry(self, source, registry):
        for obs in source.step(self.STEP_DT):
            spec = registry.get(obs.sensor_id)
            assert obs.modality is spec.modality
            assert obs.mount is spec.mount

    def test_every_promised_payload_key_is_present(self, source, registry):
        """
        A sensor declares annotators; each annotator owes a payload key. A key
        that never arrives is the failure this whole registry exists to catch:
        the annotator attached, returned nothing, and said nothing about it.
        """
        for obs in source.step(self.STEP_DT):
            spec = registry.get(obs.sensor_id)
            promised = set(MODALITY_DATA_KEYS[spec.modality])
            promised |= {ANNOTATOR_DATA_KEYS[a] for a in spec.annotators}
            missing = promised - set(obs.data)
            assert not missing, (
                f"{obs.sensor_id}: promised {sorted(promised)}, "
                f"missing {sorted(missing)}. Present: {sorted(obs.data)}"
            )

    def test_camera_arrays_match_the_declared_resolution(self, source, registry):
        for obs in source.step(self.STEP_DT):
            spec = registry.get(obs.sensor_id)
            if spec.modality not in CAMERA_MODALITIES:
                continue
            width, height = spec.resolution
            assert obs.intrinsics is not None, f"{obs.sensor_id}: no intrinsics"
            assert obs.intrinsics["width"] == width
            assert obs.intrinsics["height"] == height
            for key, shape in (("rgb", (height, width, 3)),
                               ("depth", (height, width)),
                               ("semantic", (height, width))):
                if key in obs.data:
                    got = np.asarray(obs.data[key]).shape
                    assert got == shape, f"{obs.sensor_id}.{key}: {got} != {shape}"

    def test_depth_is_a_metric_range_and_not_a_normalised_buffer(self, source):
        """
        `depth` is euclidean metres from the sensor origin -- see
        DEPTH_CONVENTION in core/observation.py.

        Euclidean-versus-axial is NOT decidable from the observation stream
        without ground truth about the scene, so this test cannot check it and
        does not pretend to: that half is pinned by the constant and enforced
        when an adapter is reviewed. What is decidable is everything else that
        arrives behind this key in practice -- a normalised [0, 1] buffer, an
        inverse-depth buffer, centimetres, or a NaN where the ray missed.
        """
        for obs in source.step(self.STEP_DT):
            if "depth" not in obs.data:
                continue
            depth = np.asarray(obs.data["depth"], dtype=np.float64)
            assert not np.isnan(depth).any(), (
                f"{obs.sensor_id}: NaN in the depth buffer. 'ray hit nothing' "
                f"is inf; NaN propagates through every consumer silently."
            )
            finite = depth[np.isfinite(depth)]
            assert finite.size, f"{obs.sensor_id}: every depth pixel was inf"
            assert (finite >= 0.0).all(), (
                f"{obs.sensor_id}: negative depth. A euclidean range cannot "
                f"be negative -- this is a signed axial buffer."
            )
            assert finite.max() > 1.0, (
                f"{obs.sensor_id}: no depth value exceeds 1.0 in a warehouse "
                f"tens of {LENGTH_UNIT}s across. This is a normalised or "
                f"inverse-depth buffer, not a range."
            )
            assert finite.max() < 1000.0, (
                f"{obs.sensor_id}: depth reaches {finite.max():.0f}. The unit "
                f"is the {LENGTH_UNIT} -- a stage authored in centimetres has "
                f"to be scaled in the adapter."
            )

    def test_range_payloads_are_point_clouds(self, source, registry):
        for obs in source.step(self.STEP_DT):
            spec = registry.get(obs.sensor_id)
            if spec.modality not in RANGE_MODALITIES:
                continue
            points = np.asarray(obs.data["points"])
            assert points.ndim == 2 and points.shape[1] == 3, (
                f"{obs.sensor_id}: point cloud is {points.shape}, want (N, 3)"
            )
            assert points.shape[0] > 0, f"{obs.sensor_id}: empty point cloud"
            assert np.isfinite(points).all(), f"{obs.sensor_id}: non-finite points"
            assert obs.intrinsics is not None
            assert obs.intrinsics.get("config") == spec.config

    # --- Poses ---------------------------------------------------------------

    def test_orientations_are_unit_quaternions(self, source):
        for obs in source.step(self.STEP_DT):
            norm = math.sqrt(sum(v * v for v in obs.pose.orientation))
            assert norm == pytest.approx(1.0, abs=1e-6), (
                f"{obs.sensor_id}: |q| = {norm}, not a rotation"
            )

    def test_the_avatars_height_arrives_in_the_up_axis(
        self, source, avatar_eye_height
    ):
        """
        Three separate mistakes, one assertion, because they are indis-
        tinguishable downstream. With eye height 1.65 m, position[UP_AXIS] is:

            ~1.65   correct
            ~0.0    the source is y-up and the height went into position[1]
            ~165    the stage is in centimetres and nobody scaled it

        None of the three raises anywhere. All three render.
        """
        low = 0.5 * avatar_eye_height
        # Generous at the top: a third-person camera legitimately floats above
        # the head. Still nowhere near a factor of a hundred.
        high = avatar_eye_height + 1.0

        seen = 0
        for obs in source.step(self.STEP_DT):
            assert all(math.isfinite(v) for v in obs.pose.position), (
                f"{obs.sensor_id}: non-finite position {obs.pose.position}"
            )
            if obs.mount is not MountType.AVATAR:
                continue
            seen += 1
            up = obs.pose.position[UP_AXIS]
            assert low <= up <= high, (
                f"{obs.sensor_id} is carried by an avatar {avatar_eye_height} "
                f"{LENGTH_UNIT}s tall, but position[{UP_AXIS}] is {up}. "
                f"Expected {low}..{high}. See UP_AXIS and LENGTH_UNIT in "
                f"core/observation.py -- an adapter converts, it does not "
                f"relabel."
            )
        assert seen, (
            "no avatar-mounted sensor reported. The avatar is the only moving "
            "entity in this design; without one there is nothing to observe."
        )

    def test_the_moving_mount_travels_in_the_plane_perpendicular_to_up(
        self, trace
    ):
        """
        The other half of z-up, and the half a single frame cannot show: the
        avatar walks on a floor. Its height barely varies while it covers
        metres of ground.

        A source that swapped two axes passes the height check above on the
        first frame and fails here on the fortieth, because the walking plane
        would then contain the up-axis.
        """
        tracks: dict[str, list[tuple[float, float, float]]] = {}
        for tick in trace:
            for obs in tick:
                if obs.mount is MountType.AVATAR:
                    tracks.setdefault(obs.sensor_id, []).append(obs.pose.position)

        assert tracks, "no avatar-mounted sensor reported"
        for sensor_id, positions in tracks.items():
            spread = [max(axis) - min(axis) for axis in zip(*positions)]
            up = spread[UP_AXIS]
            ground = max(s for i, s in enumerate(spread) if i != UP_AXIS)
            assert ground > 1.0, (
                f"{sensor_id} covered {ground:.2f} {LENGTH_UNIT}s of ground "
                f"over {len(positions)} ticks -- too little to say which axis "
                f"is up. Give the motion tests more time."
            )
            assert up < 0.25 * ground, (
                f"{sensor_id} moved {up:.2f} along axis {UP_AXIS} while "
                f"covering {ground:.2f} across the floor. Either the avatar "
                f"is climbing, or the up-axis is not {UP_AXIS} in this source."
            )

    def test_the_avatar_is_the_only_thing_that_moves(self, trace):
        """
        The design rests on exactly one moving entity. A fixed station whose
        pose drifts means something is parented wrong; a robot that moves means
        a locomotion policy got switched on and the scene is no longer static.
        """
        poses: dict[str, set] = {}
        mounts: dict[str, MountType] = {}
        for tick in trace:
            for obs in tick:
                poses.setdefault(obs.sensor_id, set()).add(obs.pose.position)
                mounts[obs.sensor_id] = obs.mount

        for sensor_id, seen in poses.items():
            if mounts[sensor_id] is MountType.AVATAR:
                assert len(seen) > 1, (
                    f"{sensor_id} is avatar-mounted but never moved -- either "
                    f"the avatar is not being driven or its pose is not "
                    f"reaching the sensor."
                )
            else:
                assert len(seen) == 1, (
                    f"{sensor_id} is mounted '{mounts[sensor_id].value}' but "
                    f"moved through {len(seen)} poses."
                )

    # --- The one that catches a silent scene ---------------------------------

    def test_a_fixed_sensor_reacts_to_the_moving_avatar(self, trace):
        """
        THE most valuable test in this file, and the reason it is written
        against the contract rather than against the mock.

        Against the live simulator this is the collision-mesh check. The
        viewport camera is not a physical object: no mesh, no collider, no
        material. Lidar rays pass through it, cameras render nothing, radar
        returns nothing -- and every sensor keeps reporting, at full rate, a
        completely constant scene. There is no error anywhere. If this test
        fails on the server, the avatar has no collision mesh; do not go
        looking at the sensors.

        Only *some* fixed sensor need react: INFRA_02 is in the second building
        and correctly sees nothing, which is what makes it a useful control.
        """
        extremes: dict[str, list[float]] = {}
        for tick in trace:
            for obs in tick:
                if obs.mount is MountType.AVATAR:
                    continue
                extremes.setdefault(obs.sensor_id, []).append(payload_scalar(obs))

        assert extremes, "no non-avatar sensors reported at all"
        reacted = {
            sensor_id: (min(values), max(values))
            for sensor_id, values in extremes.items()
            if max(values) - min(values) > 0.01 * max(abs(max(values)), 1.0)
        }
        assert reacted, (
            "not one fixed or robot-mounted sensor changed its reading while "
            "the avatar walked past. Every reading was constant and nothing "
            "raised. Check the avatar has a collision mesh before anything "
            f"else. Sensors examined: {sorted(extremes)}"
        )

    def test_readings_summarise_without_dragging_arrays_along(self, source):
        """Logging and the inspector panel call this on every reading."""
        for obs in source.step(self.STEP_DT):
            summary = obs.summary()
            assert summary["sensor_id"] == obs.sensor_id
            assert summary["modality"] == obs.modality.value
            for key, value in summary.items():
                assert not hasattr(value, "shape"), f"{key} is still an array"

    def test_semantic_labels_are_present_where_segmentation_was_asked_for(
        self, source, registry
    ):
        """A semantic map with no id->label mapping cannot be read by anyone."""
        for obs in source.step(self.STEP_DT):
            spec = registry.get(obs.sensor_id)
            if "semantic_segmentation" not in spec.annotators:
                continue
            labels = obs.data.get("semantic_labels")
            assert labels, f"{obs.sensor_id}: semantic map with no label mapping"
            # Stringified rather than indexed: Isaac's id_to_labels nests the
            # class name a level down, and the contract is that the class is
            # *there*, not that every source spells the mapping the same way.
            assert "person" in str(labels).lower(), (
                f"{obs.sensor_id}: nothing is labelled 'person' -- {labels}. "
                f"Segmentation returns empty when the avatar carries no "
                f"semantic class."
            )

    def test_no_modality_is_a_surprise(self, source):
        for obs in source.step(self.STEP_DT):
            assert isinstance(obs.modality, Modality)
            assert obs.modality in MODALITY_DATA_KEYS
