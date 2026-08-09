"""
The mock observation source.

Two halves, and the split is the point:

  TestMockSource      -- the shared contract from tests/contract.py, which the
                         live Isaac source must pass unchanged (task S11).
  the rest of the file -- properties only the mock can be asked about, because
                         they are about *driving* the world rather than
                         observing it.

Nothing here needs a GPU, a simulator, or a stage.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from core.mock_source import (
    CircuitPath,
    MockObservationSource,
    _beams_on_target,
    _quaternion_from_matrix,
)
from core.observation import Modality, MountType, ObservationSource
from tests.contract import ObservationSourceContract, payload_scalar

SENSORS = "config/sensors.yaml"
SCENE = "config/scene.yaml"


def make() -> MockObservationSource:
    return MockObservationSource.from_config(SENSORS, SCENE)


def frozen_world(**kwargs) -> MockObservationSource:
    """
    The same world with nothing moving in it: time still advances, but the
    avatar is parked half a kilometre away, outside every sensor's range. The
    control case for everything that asks "did this reading change, and why?".
    """
    return MockObservationSource.from_config(
        SENSORS, SCENE,
        path=CircuitPath(centre=(500.0, 500.0), radius=1.0, speed=0.0),
        **kwargs,
    )


class TestMockSource(ObservationSourceContract):
    """The contract, run against the mock. See tests/contract.py."""

    def make_source(self) -> ObservationSource:
        return make()


@pytest.fixture
def source() -> MockObservationSource:
    return make()


# --- Driving the world -------------------------------------------------------


def test_every_declared_sensor_reports_every_tick(source):
    """
    The contract allows a source to skip a sensor on a given tick. The mock
    never does -- consumers get a complete frame, which makes a missing reading
    downstream unambiguously the consumer's bug.
    """
    observations = source.step()
    assert {o.sensor_id for o in observations} == set(source.sensor_ids)
    assert len(source.sensor_ids) == len(source.registry)


def test_avatar_walks_at_the_declared_speed(source):
    """
    `move_speed` in scene.yaml is honoured exactly, so a test can convert
    elapsed time into distance travelled without measuring anything.
    """
    start = np.array(source.avatar_pose.position)
    source.step(1.0)
    step_one = np.array(source.avatar_pose.position)
    source.step(1.0)
    step_two = np.array(source.avatar_pose.position)
    # Chord length over one second of arc, not arc length -- a hair under
    # move_speed by exactly the circle's sagitta.
    first = float(np.linalg.norm(step_one - start))
    second = float(np.linalg.norm(step_two - step_one))
    assert first == pytest.approx(second, rel=1e-6)
    assert 0.9 * 1.4 < first <= 1.4


def test_lidar_returns_grow_as_the_avatar_approaches(source):
    """
    The whole reason this file exists. A source where the numbers do not move
    with the avatar teaches a consumer nothing and hides every bug in it.
    """
    counts, distances = [], []
    for _ in range(60):
        for obs in source.step(0.25):
            if obs.sensor_id == "INFRA_01_LIDAR":
                counts.append(obs.data["num_returns"])
                distances.append(
                    float(np.linalg.norm(
                        np.array(source.avatar_pose.position)
                        - np.array(obs.pose.position)
                    ))
                )
    nearest = counts[int(np.argmin(distances))]
    farthest = counts[int(np.argmax(distances))]
    assert nearest > farthest, (
        f"{nearest} returns at {min(distances):.1f} m but {farthest} at "
        f"{max(distances):.1f} m -- returns are not tracking distance"
    )
    assert min(counts) > 0, "the static scene disappeared entirely"


def test_a_station_in_the_other_building_never_sees_the_avatar(source):
    """
    INFRA_02 is 60 m away, past every sensor's range. It is the control: a
    change detector that fires here is firing on noise.
    """
    readings = set()
    for _ in range(40):
        for obs in source.step(0.25):
            if obs.sensor_id == "INFRA_02_LIDAR":
                readings.add(obs.data["num_returns"])
    assert readings == {1024}


def test_zero_noise_means_an_unchanged_world_reads_identically(source):
    """
    The property the residual work in M5 rests on. Two ticks with the world in
    the same place must be bit-identical, or "nothing changed" and "everything
    changed slightly" are indistinguishable downstream.
    """
    # A path with no speed: time advances, the avatar does not.
    frozen = frozen_world()
    first = {o.sensor_id: o for o in frozen.step()}
    second = {o.sensor_id: o for o in frozen.step()}
    for sensor_id, before in first.items():
        after = second[sensor_id]
        assert after.timestamp > before.timestamp
        for key, value in before.data.items():
            if isinstance(value, np.ndarray):
                assert np.array_equal(value, after.data[key]), f"{sensor_id}.{key}"
            else:
                assert value == after.data[key], f"{sensor_id}.{key}"


def test_noise_makes_an_unchanged_world_read_differently(source):
    """The other half: a consumer that assumes exactness should be caught."""
    noisy = frozen_world(noise=0.05)
    first = {o.sensor_id: o for o in noisy.step()}
    second = {o.sensor_id: o for o in noisy.step()}
    assert not np.array_equal(
        first["INFRA_01_LIDAR"].data["points"],
        second["INFRA_01_LIDAR"].data["points"],
    )


def test_a_placed_object_shows_up_and_a_moved_one_shows_up_elsewhere(source):
    """
    The hook M5 needs: change the world without the observer causing it, and
    confirm the readings move. This is a residual spike in its rawest form.
    """
    frozen = frozen_world()
    baseline = _lidar_count(frozen, "INFRA_01_LIDAR")

    frozen.place_object("crate", (5.5, 0.5, 0.0), radius=0.5, half_height=0.5)
    with_crate = _lidar_count(frozen, "INFRA_01_LIDAR")
    assert with_crate > baseline, "a crate under the ceiling lidar returned nothing"

    frozen.move_object("crate", (60.0, 40.0, 0.0))  # 68 m away: past lidar range
    moved_away = _lidar_count(frozen, "INFRA_01_LIDAR")
    assert moved_away == baseline

    frozen.remove_object("crate")
    assert _lidar_count(frozen, "INFRA_01_LIDAR") == baseline


def test_a_stationary_object_does_not_shimmer(source):
    """
    Points on a motionless object must be identical tick to tick. If they are
    redrawn each frame, every change detector sees change everywhere and the
    residual signal is buried in the fake's own noise.
    """
    frozen = frozen_world()
    frozen.place_object("crate", (5.5, 0.5, 0.0))
    first = _lidar_points(frozen, "INFRA_01_LIDAR")
    second = _lidar_points(frozen, "INFRA_01_LIDAR")
    assert np.array_equal(first, second)


def test_radar_only_returns_what_moves(source):
    """
    Radar's contribution to the demo is that it sees a moving body and largely
    ignores the static room. A placed crate is invisible to it; the avatar is
    not.
    """
    frozen = frozen_world()
    by_id = {o.sensor_id: o for o in frozen.step()}
    static_only = by_id["INFRA_01_RADAR"].data["num_returns"]

    frozen.place_object("crate", (5.5, 0.5, 0.0), radius=0.6)
    by_id = {o.sensor_id: o for o in frozen.step()}
    assert by_id["INFRA_01_RADAR"].data["num_returns"] == static_only
    assert by_id["INFRA_01_LIDAR"].data["num_returns"] > static_only

    # A walking avatar, on the other hand, is exactly what radar is for.
    walking = {o.sensor_id: o for o in source.step()}["INFRA_01_RADAR"]
    assert walking.data["num_returns"] > static_only
    assert (walking.data["radial_velocities"] > 0.0).any(), (
        "the avatar is walking but radar reported no doppler shift at all"
    )


def test_avatar_camera_sees_a_scene_that_changes(source):
    """
    First-person imagery must change as you walk past the racking, or it
    teaches a consumer nothing and hides every bug in it.
    """
    frames = []
    for _ in range(8):
        for obs in source.step(0.5):
            if obs.sensor_id == "AVATAR_CAM_FP":
                frames.append(obs.data["rgb"].copy())
    assert not all(np.array_equal(frames[0], f) for f in frames[1:]), (
        "every first-person frame was identical -- the avatar camera is not "
        "seeing the world it is moving through"
    )


def test_the_first_person_camera_is_not_inside_the_avatar(source):
    """
    Your own body must not fill your own first-person frame. Third person is
    the view that should contain it -- that is what makes it third person.
    """
    by_id = {o.sensor_id: o for o in source.step(4.0)}
    assert _person_pixels(by_id["AVATAR_CAM_FP"]) == 0
    assert _person_pixels(by_id["AVATAR_CAM_TP"]) > 0


def test_sensors_at_one_station_share_a_pose(source):
    """
    Co-location is the experimental control the registry enforces. If the mock
    broke it, every comparison drawn against the mock would be a comparison of
    viewpoints instead of modalities.
    """
    positions = {
        obs.sensor_id: obs.pose.position
        for obs in source.step()
        if obs.sensor_id.startswith("INFRA_01_")
    }
    assert len(set(positions.values())) == 1, positions


def test_reset_replays_the_same_world(source):
    """A fixture you cannot rewind is a fixture you cannot debug with."""
    first = [o.data["num_returns"] for o in source.step(0.5)
             if o.modality is Modality.LIDAR]
    source.reset()
    assert source.time == 0.0
    again = [o.data["num_returns"] for o in source.step(0.5)
             if o.modality is Modality.LIDAR]
    assert first == again


def test_mounts_come_from_the_scene_file(source):
    """
    Poses are read from config/scene.yaml, never invented. INFRA_01 sits at
    the ceiling position declared there.
    """
    by_id = {o.sensor_id: o for o in source.step()}
    assert by_id["INFRA_01_LIDAR"].pose.position == (5.0, 0.0, 6.5)
    assert by_id["BOT_01_CAM"].pose.position == (4.0, 2.0, 0.0)


def test_state_and_experience_stay_distinguishable(source):
    """
    A fixed camera cannot experience anything -- it never acts, never collides,
    never pays a traversal cost. Anything fusing the two has to be able to tell
    them apart, so the mount survives into every reading.
    """
    mounts = {o.mount for o in source.step()}
    assert MountType.FIXED in mounts
    assert MountType.AVATAR in mounts
    assert MountType.ROBOT in mounts


# --- Failing loudly ----------------------------------------------------------


def test_a_sensor_the_scene_cannot_place_is_named(tmp_path):
    """
    Rule 1, one layer up: rather than invent a pose for an unknown mount, say
    which sensor could not be placed. A mock that defaults to the origin puts
    every station on top of every other and still looks plausible.
    """
    sensors = tmp_path / "sensors.yaml"
    sensors.write_text(
        "sensors:\n"
        "  - id: GHOST_CAM\n"
        "    modality: rgb\n"
        "    mount: fixed\n"
        "    parent: /World/Nowhere\n"
        "    prim_path: /World/Nowhere/cam\n"
        "    resolution: [64, 48]\n"
        "    annotators: [rgb]\n"
    )
    scene = tmp_path / "scene.yaml"
    scene.write_text("avatar:\n  prim_path: /World/Avatar\nstations: []\n")
    with pytest.raises(ValueError, match="GHOST_CAM.*parent /World/Nowhere"):
        MockObservationSource.from_config(sensors, scene)


def test_a_scene_with_no_avatar_is_rejected(tmp_path):
    scene = tmp_path / "scene.yaml"
    scene.write_text("stations: []\n")
    with pytest.raises(ValueError, match="avatar has no prim_path"):
        MockObservationSource.from_config(SENSORS, scene)


def test_the_avatar_cannot_be_overwritten(source):
    with pytest.raises(ValueError, match="driven by the trajectory"):
        source.place_object("avatar", (0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="cannot be removed"):
        source.remove_object("avatar")


def test_a_nonpositive_timestep_is_rejected(source):
    with pytest.raises(ValueError, match="dt must be positive"):
        MockObservationSource.from_config(SENSORS, SCENE, dt=0.0)


# --- The geometry underneath -------------------------------------------------


def test_beams_on_target_falls_off_with_the_square_of_distance():
    near = _beams_on_target(4.0, 0.35, 0.875, 0.4, 1.0)
    far = _beams_on_target(8.0, 0.35, 0.875, 0.4, 1.0)
    assert near / far == pytest.approx(4.0, rel=0.05)


def test_beams_on_target_is_zero_at_zero_range():
    assert _beams_on_target(0.0, 0.35, 0.875, 0.4, 1.0) == 0


def test_quaternion_from_identity_is_identity():
    assert _quaternion_from_matrix(np.eye(3)) == pytest.approx((1.0, 0.0, 0.0, 0.0))


@pytest.mark.parametrize("angle", [0.5, math.pi / 2, math.pi - 0.01, 2.5])
def test_quaternion_round_trips_through_a_rotation_matrix(angle):
    c, s = math.cos(angle), math.sin(angle)
    for matrix in (
        np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float),
        np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float),
        np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float),
    ):
        w, x, y, z = _quaternion_from_matrix(matrix)
        assert math.sqrt(w * w + x * x + y * y + z * z) == pytest.approx(1.0)
        # Rebuild the matrix from the quaternion and compare.
        rebuilt = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        assert rebuilt == pytest.approx(matrix, abs=1e-9)


def test_payload_scalar_reads_every_modality(source):
    """The contract's change detector must not silently return 0 for anything."""
    for obs in source.step():
        assert payload_scalar(obs) != 0.0, obs.sensor_id


# --- Helpers -----------------------------------------------------------------


def _lidar_count(source: MockObservationSource, sensor_id: str) -> int:
    return next(o.data["num_returns"] for o in source.step()
                if o.sensor_id == sensor_id)


def _lidar_points(source: MockObservationSource, sensor_id: str) -> np.ndarray:
    return next(o.data["points"] for o in source.step()
                if o.sensor_id == sensor_id)


def _person_pixels(obs) -> int:
    """Pixels painted with the 'person' label colour."""
    rgb = obs.data["rgb"]
    return int(np.all(rgb == np.array([220, 170, 120], dtype=np.uint8),
                      axis=-1).sum())
