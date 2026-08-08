"""Observation contract tests. No GPU, no simulator."""

import pytest

from core.observation import Modality, MountType, Observation, Pose


def test_pose_rejects_wrong_length_position():
    with pytest.raises(ValueError, match="position must be 3"):
        Pose(position=(0.0, 0.0), orientation=(1.0, 0.0, 0.0, 0.0))


def test_pose_rejects_wrong_length_quaternion():
    with pytest.raises(ValueError, match="orientation must be 4"):
        Pose(position=(0.0, 0.0, 0.0), orientation=(1.0, 0.0, 0.0))


def test_summary_is_array_free():
    """
    The inspector panel and the logs want a printable summary, never the
    payload. Point clouds must degrade to a count, not get stringified.
    """
    obs = Observation(
        sensor_id="INFRA_01_LIDAR",
        timestamp=1.23456,
        modality=Modality.LIDAR,
        mount=MountType.FIXED,
        pose=Pose((1.0, 2.0, 6.5), (1.0, 0.0, 0.0, 0.0)),
        data={"points": [(0.0, 0.0, 0.0)] * 4096, "frame": 17},
    )
    summary = obs.summary()
    assert summary["points_len"] == 4096
    assert summary["frame"] == 17
    assert summary["modality"] == "lidar"
    assert summary["mount"] == "fixed"
    assert "points" not in summary


def test_fixed_and_avatar_observations_are_distinguishable():
    """
    Fusion code has to know whether a reading is allocentric state or embodied
    experience. That distinction must survive into the observation itself.
    """
    pose = Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    fixed = Observation("F", 0.0, Modality.RGB, MountType.FIXED, pose)
    avatar = Observation("A", 0.0, Modality.RGB, MountType.AVATAR, pose)
    assert fixed.summary()["mount"] != avatar.summary()["mount"]
