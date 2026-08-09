"""
The closed loop, run against the mock.

Two tests here are the M5 gate and everything else supports them:

    test_a_consistent_world_predicts_itself     residual == 0 in a static world
    test_a_new_object_contradicts_memory        residual spikes on a change

That pair is the research claim in miniature -- the same number meaning model
noise in one world and change detection in the other -- demonstrated on a
laptop with no simulator anywhere near it.

The loop under test touches nothing but `core.observation.ObservationSource`,
so every one of these could be pointed at the live Isaac source (task S11) by
changing `walking()` and `parked()`. What could NOT move over is the world
driving: `place_object` and a parked avatar are the mock's, and they are how a
test changes the world without the observer having caused the change.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from core.memory import (
    Evidence,
    LastObservationMemory,
    MemoryLoop,
    RevisionKind,
    Turn,
    Viewpoint,
    changes,
    evidence_of,
    magnitude,
    payload_residual,
)
from core.memory.interfaces import SpatialMemory
from core.mock_source import CircuitPath, MockObservationSource
from core.observation import MountType, ObservationSource

SENSORS = "config/sensors.yaml"
SCENE = "config/scene.yaml"

CEILING_LIDAR = "INFRA_01_LIDAR"    # 6.5 m above the circuit; sees the avatar
CEILING_CAM = "INFRA_01_CAM"
CEILING_RADAR = "INFRA_01_RADAR"
CONTROL = "INFRA_02_LIDAR"          # 60 m away, second building; sees nothing
AVATAR_SENSORS = ("AVATAR_CAM_FP", "AVATAR_CAM_TP")

# The middle of the floor both infrastructure stations are pointed at, well
# inside INFRA_01's lidar range and squarely in the middle of its frame.
CRATE_AT = (9.0, 0.0, 0.0)
CRATE = {"radius": 0.6, "half_height": 0.9, "label": "prop"}


def walking(**kwargs) -> MockObservationSource:
    """The scene as designed: one avatar, walking its circuit."""
    return MockObservationSource.from_config(SENSORS, SCENE, **kwargs)


def parked(**kwargs) -> MockObservationSource:
    """
    The same warehouse with nothing moving in it. Time still advances; the
    avatar sits 700 m away, outside every sensor's range.

    This is the static-scene control. Without it, "the residual is zero" is
    untestable, because in the real scene something is always moving.
    """
    return MockObservationSource.from_config(
        SENSORS, SCENE,
        path=CircuitPath(centre=(500.0, 500.0), radius=1.0, speed=0.0),
        **kwargs,
    )


def warmed(source, ticks: int = 1, dt: float = 0.25, **memory_kwargs):
    """A loop that has already seen `ticks` frames, so nothing is novel."""
    loop = MemoryLoop(LastObservationMemory(**memory_kwargs))
    loop.run(source, ticks, dt)
    return loop


def by_sensor(turns: list[Turn]) -> dict[str, Turn]:
    return {turn.sensor_id: turn for turn in turns}


# --- The loop is made of the contract and nothing else -----------------------


def test_the_loop_runs_against_anything_that_satisfies_the_contract():
    """
    The reason M5 could be written before the simulator existed. If this test
    needed a mock-only attribute, the memory module would be married to the
    fake and S11 would inherit a rewrite.
    """
    source = walking()
    assert isinstance(source, ObservationSource)
    memory = LastObservationMemory()
    assert isinstance(memory, SpatialMemory)

    trace = MemoryLoop(memory).run(source, ticks=3, dt=0.25)
    assert len(trace) == 3
    assert {t.sensor_id for t in trace[0]} == set(source.sensor_ids)
    source.close()


def test_the_first_reading_from_any_viewpoint_is_novel():
    """
    Memory starts empty, so nothing is predicted and nothing is contradicted.
    Novel must not be reported as change -- a detector that fires the first
    time it looks anywhere fires constantly.
    """
    loop = MemoryLoop(LastObservationMemory())
    turns = loop.tick(parked(), 0.25)

    assert all(t.residual.novel for t in turns)
    assert all(t.residual.magnitude == 1.0 for t in turns)
    assert all(t.revision.kind is RevisionKind.WROTE for t in turns)
    assert changes(turns) == [], "a first look reported the world as changed"


# --- Half one of the claim: a static world ------------------------------------


def test_a_consistent_world_predicts_itself():
    """
    M5 GATE, first half. Feed the loop a world that is not changing and the
    residual must go to zero -- exactly zero here, because the mock is
    deterministic at noise=0.
    """
    source = parked()
    loop = warmed(source)

    for _ in range(3):
        for turn in loop.tick(source, 0.25):
            assert not turn.residual.novel, f"{turn.sensor_id} lost its memory"
            assert turn.residual.magnitude == pytest.approx(0.0, abs=1e-12), (
                f"{turn.sensor_id} disagreed with memory in an unchanged "
                f"world: {turn.residual.by_key}"
            )
            assert turn.revision.kind is RevisionKind.REINFORCED


def test_an_unchanging_world_accumulates_confidence_instead_of_rewrites():
    """The other half of "reinforced": memory gets more sure, not rewritten."""
    source = parked()
    loop = warmed(source)
    loop.run(source, ticks=4, dt=0.25)

    record = loop.memory.records(Evidence.STATE)[CEILING_LIDAR]
    assert record.observations == 5   # the warm-up frame plus four
    assert record.revisions == 0


# --- Half two: a world that changed underneath the observer -------------------


def test_a_new_object_contradicts_memory():
    """
    M5 GATE, second half. The avatar is parked and the observer does nothing;
    a crate appears under the ceiling station. The residual has to spike, and
    it has to spike only where the crate is visible.
    """
    source = parked()
    loop = warmed(source)
    threshold = loop.memory.change_threshold

    source.place_object("crate", CRATE_AT, **CRATE)
    turns = by_sensor(loop.tick(source, 0.25))

    # About 370 new returns in a cloud of 2150: the crate is a sixth of what
    # the ceiling lidar can see, and memory predicted none of it.
    lidar = turns[CEILING_LIDAR].residual
    assert lidar.magnitude > 0.1, f"a crate arrived unnoticed: {lidar.by_key}"
    assert turns[CEILING_LIDAR].changed
    assert turns[CEILING_CAM].residual.magnitude > threshold

    # The control. INFRA_02 is 54 m away in the second building; a change
    # detector that fires there is firing on its own noise.
    assert turns[CONTROL].residual.magnitude == 0.0
    assert turns[CONTROL].revision.kind is RevisionKind.REINFORCED
    # Its *camera* does register the crate faintly -- about 500 pixels at 54 m
    # -- because the mock has no occlusion and does not know there is a
    # building in between. It stays under the threshold, and in the real scene
    # the wall removes even that. A known artefact of the fake, pinned here so
    # that it fails loudly if it ever grows into a false positive.
    far_camera = turns["INFRA_02_CAM"].residual.magnitude
    assert 0.0 < far_camera < threshold, (
        f"the second building's camera scored {far_camera:.3f} against a "
        f"threshold of {threshold}: the mock's lack of occlusion has stopped "
        f"being negligible and the control station is about to fire"
    )
    assert not [t for t in changes(list(turns.values()))
                if t.sensor_id.startswith("INFRA_02")]

    # Radar sees only what moves, so a placed crate is correctly invisible to
    # it. Not a bug -- it is why change detection cannot be one number per
    # station, and why the stations are multi-modal at all.
    assert turns[CEILING_RADAR].residual.magnitude == 0.0


def test_the_loop_closes_and_the_change_is_absorbed():
    """
    Revision is the arrow the write-only systems are missing. Having been
    contradicted once, memory must hold the new world: the same crate on the
    next tick is no longer news.
    """
    source = parked()
    loop = warmed(source)

    source.place_object("crate", CRATE_AT, **CRATE)
    first = by_sensor(loop.tick(source, 0.25))[CEILING_LIDAR]
    second = by_sensor(loop.tick(source, 0.25))[CEILING_LIDAR]

    assert first.changed
    assert second.residual.magnitude == pytest.approx(0.0, abs=1e-12)
    assert second.revision.kind is RevisionKind.REINFORCED
    assert loop.memory.records(Evidence.STATE)[CEILING_LIDAR].revisions == 1


def test_something_going_away_is_a_change_too():
    """
    Change detection that only notices arrivals is half a detector, and the
    absent half is the one that matters for a memory: the shelf you remember
    is no longer there.
    """
    source = parked()
    source.place_object("crate", CRATE_AT, **CRATE)
    loop = warmed(source)

    source.remove_object("crate")
    turn = by_sensor(loop.tick(source, 0.25))[CEILING_LIDAR]
    assert turn.changed, f"a crate vanished unnoticed: {turn.residual.by_key}"


def test_the_residual_tracks_how_far_the_world_has_moved_from_memory():
    """
    Open loop: memory learns the room once and is then frozen, which is the
    demo. The residual at the ceiling station must rise as the avatar walks
    towards it and peak when it is nearest -- and the control station must
    stay flat throughout, or the residual is measuring the mock's noise.
    """
    source = walking()
    loop = warmed(source, ticks=1, dt=0.25)

    ceiling: list[float] = []
    control: list[float] = []
    distances: list[float] = []
    for _ in range(40):
        for obs in source.step(0.25):
            if obs.sensor_id == CEILING_LIDAR:
                ceiling.append(loop.observe(obs).magnitude)
                distances.append(
                    float(np.linalg.norm(
                        np.array(source.avatar_pose.position)
                        - np.array(obs.pose.position)
                    ))
                )
            elif obs.sensor_id == CONTROL:
                control.append(loop.observe(obs).magnitude)

    assert max(ceiling) > 0.1, (
        f"the avatar walked under the ceiling lidar and the residual never "
        f"rose above {max(ceiling):.3f}"
    )
    assert ceiling[int(np.argmin(distances))] == pytest.approx(max(ceiling))
    assert max(control) == 0.0, "the station in the other building saw something"


# --- The sentence the whole project is about ----------------------------------


def test_noise_stays_under_the_threshold_that_a_real_change_clears():
    """
    "Prediction residual equals model noise in a static scene and change
    detection in a dynamic one." Both halves, one test, with the sensor noise
    turned on so the static half is not trivially zero.

    The gap between the two numbers is the entire signal-to-noise margin of
    the method, so it is worth having a test fail when it narrows.
    """
    source = parked(noise=0.002)
    loop = warmed(source)
    threshold = loop.memory.change_threshold

    noise_floor = 0.0
    for _ in range(4):
        for turn in loop.tick(source, 0.25):
            assert turn.revision.kind is RevisionKind.REINFORCED, (
                f"{turn.sensor_id}: sensor noise was mistaken for a change "
                f"({turn.residual.magnitude:.4f})"
            )
            noise_floor = max(noise_floor, turn.residual.magnitude)

    assert noise_floor > 0.0, "noise=0.002 produced a bit-identical world"
    assert noise_floor < threshold

    source.place_object("crate", CRATE_AT, **CRATE)
    signal = by_sensor(loop.tick(source, 0.25))[CEILING_LIDAR].residual.magnitude
    assert signal > 10 * noise_floor, (
        f"change {signal:.4f} is not clear of the noise floor "
        f"{noise_floor:.4f} -- the residual cannot separate them"
    )


# --- State and experience -----------------------------------------------------


@pytest.mark.parametrize("mount", list(MountType))
def test_every_mount_declares_what_kind_of_evidence_it_produces(mount):
    """
    Adding a mount type is a claim about whether it can act. Defaulting is how
    a moving platform's readings quietly end up filed as allocentric state.
    """
    assert isinstance(evidence_of(mount), Evidence)


def test_an_undeclared_mount_is_refused_rather_than_guessed():
    with pytest.raises(ValueError, match="no evidence kind declared"):
        evidence_of("hovering")


def test_state_and_experience_are_stored_apart():
    """
    Fixed and robot cameras report where things are. The avatar's cameras
    report what it was like to be there. Nothing may fuse the two without
    saying which it is holding, so they do not even share a dict.
    """
    source = parked()
    loop = warmed(source)
    memory = loop.memory

    state = memory.records(Evidence.STATE)
    experience = memory.records(Evidence.EXPERIENCE)

    assert set(experience) == set(AVATAR_SENSORS)
    assert not set(state) & set(experience)
    assert set(state) | set(experience) == set(source.sensor_ids)
    assert all(r.evidence is Evidence.STATE for r in state.values())
    assert AVATAR_SENSORS[0] not in state


def test_a_residual_says_which_kind_of_evidence_it_came_from():
    """It travels on the residual, so a consumer cannot lose it downstream."""
    turns = by_sensor(MemoryLoop(LastObservationMemory()).tick(parked(), 0.25))
    assert turns[CEILING_CAM].residual.evidence is Evidence.STATE
    assert turns["BOT_01_CAM"].residual.evidence is Evidence.STATE
    assert turns[AVATAR_SENSORS[0]].residual.evidence is Evidence.EXPERIENCE


def test_a_sensor_cannot_change_which_kind_it_produces():
    """
    A fixed camera that starts producing experience is a parenting bug in the
    scene. Memory refuses it by name rather than filing the same sensor in
    both stores and letting a consumer average them.
    """
    source = parked()
    loop = warmed(source)
    ceiling = next(o for o in source.step(0.25) if o.sensor_id == CEILING_CAM)

    unparented = dataclasses.replace(ceiling, mount=MountType.AVATAR)
    with pytest.raises(ValueError, match="cannot change what kind of evidence"):
        loop.ingest(unparented)


# --- The gap the research has to close ----------------------------------------


def test_a_moving_viewpoint_is_never_predicted_from_a_stale_pose():
    """
    Every tick of the walking avatar is a viewpoint memory has never occupied,
    so a last-observation memory can say nothing about any of them and must
    say so rather than offering the last frame from two metres back.

    This is the honest limitation of the baseline and the reason this package
    is a skeleton: predicting an unvisited viewpoint needs a map, not a cache.
    """
    source = walking()
    loop = MemoryLoop(LastObservationMemory())
    predicted_something = False
    for tick, turns in enumerate(loop.run(source, ticks=6, dt=0.25)):
        for turn in turns:
            if turn.residual.evidence is Evidence.EXPERIENCE:
                assert turn.residual.novel, (
                    f"{turn.sensor_id} was predicted from a pose it has "
                    f"already left"
                )
                assert turn.revision.kind is RevisionKind.WROTE
            elif tick > 0:
                # The other half: a viewpoint that stayed put IS predictable,
                # so "novel" is about the pose and not about the mount.
                assert not turn.residual.novel
                predicted_something = True
    assert predicted_something


def test_a_viewpoint_that_stops_moving_becomes_predictable_again():
    """The pose gate, not the mount, is what makes experience unpredictable."""
    source = parked()
    loop = warmed(source)
    turns = by_sensor(loop.tick(source, 0.25))
    for sensor_id in AVATAR_SENSORS:
        assert not turns[sensor_id].residual.novel
        assert turns[sensor_id].residual.magnitude == pytest.approx(0.0)


def test_memory_refuses_to_predict_from_somewhere_it_has_not_been():
    source = parked()
    loop = warmed(source)
    ceiling = next(o for o in source.step(0.25) if o.sensor_id == CEILING_CAM)

    here = Viewpoint.of(ceiling)
    assert loop.memory.predict(here) is not None

    x, y, z = here.pose.position
    moved = dataclasses.replace(
        here, pose=dataclasses.replace(here.pose, position=(x + 1.0, y, z))
    )
    assert loop.memory.predict(moved) is None


# --- The residual metric ------------------------------------------------------


def test_identical_payloads_have_no_residual():
    payload = {"points": np.zeros((10, 3)), "num_returns": 10}
    assert magnitude(payload_residual(payload, payload)) == 0.0


def test_a_payload_key_that_vanished_is_maximally_wrong():
    """
    The registry's whole reason for existing, one layer up: an annotator that
    attaches and returns nothing. If it reaches Layer 4 it must be loud.
    """
    by_key = payload_residual({"rgb": np.zeros((2, 2, 3)), "depth": np.ones((2, 2))},
                              {"rgb": np.zeros((2, 2, 3))})
    assert by_key["depth"] == 1.0
    assert by_key["rgb"] == 0.0


def test_an_eight_bit_frame_does_not_wrap_around():
    """
    uint8 250 minus uint8 10 is 16, not -240. A metric that did the
    subtraction in the payload's own dtype would score a black frame as a near
    perfect match for a white one.
    """
    dark = np.full((4, 4, 3), 10, dtype=np.uint8)
    bright = np.full((4, 4, 3), 250, dtype=np.uint8)
    # 240/250. Wrapping would have scored these two frames 16/250 = 0.06.
    assert payload_residual({"rgb": dark}, {"rgb": bright})["rgb"] > 0.9


def test_a_point_cloud_that_grew_is_scored_by_how_much_of_it_is_new():
    small = np.zeros((100, 3))
    large = np.zeros((125, 3))
    assert payload_residual({"points": small}, {"points": large})["points"] == (
        pytest.approx(0.2)
    )


def test_label_maps_are_compared_as_labels_not_as_numbers():
    """
    Class 7 is not "further" from class 1 than class 2 is. Segmentation is
    scored as the fraction of pixels that disagree.
    """
    before = np.zeros((10, 10), dtype=np.uint8)
    after = before.copy()
    after[:2, :] = 7
    assert payload_residual({"semantic": before},
                            {"semantic": after})["semantic"] == pytest.approx(0.2)


def test_an_infinite_depth_reading_does_not_poison_the_residual():
    """
    Isaac's distance_to_camera returns inf where the ray hit nothing. inf
    minus inf is nan, and a nan residual looks like an ordinary float all the
    way downstream.
    """
    sky = np.array([np.inf, np.inf, 2.0])
    same = np.array([np.inf, np.inf, 2.0])
    hit = np.array([np.inf, 5.0, 2.0])

    assert payload_residual({"depth": sky}, {"depth": same})["depth"] == 0.0
    scored = payload_residual({"depth": sky}, {"depth": hit})["depth"]
    assert np.isfinite(scored) and scored > 0.0


def test_the_magnitude_is_the_loudest_channel_not_the_average():
    """Averaging buries a changed channel under the ones that stayed still."""
    assert magnitude({"points": 0.9, "ranges": 0.0, "intensities": 0.0}) == 0.9
    assert magnitude({}) == 0.0
