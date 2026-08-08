"""
Registry tests. These MUST pass on the MacBook with no GPU and no simulator.
If they ever can't run there, the layer boundary has leaked.
"""

import pytest

from core.observation import Modality, MountType
from core.registry import SensorRegistry, SensorSpec

REGISTRY_PATH = "config/sensors.yaml"


@pytest.fixture(scope="module")
def registry() -> SensorRegistry:
    return SensorRegistry.from_yaml(REGISTRY_PATH)


def test_registry_loads(registry):
    assert len(registry) > 0


def test_every_sensor_id_unique(registry):
    ids = [s.sensor_id for s in registry]
    assert len(ids) == len(set(ids))


def test_infra_stations_are_colocated(registry):
    """
    Co-location is the experimental control: three modalities at the SAME pose
    observing the SAME event isolates modality as the only variable. Different
    poses would confound modality with viewpoint, which would quietly
    invalidate the comparison the demo is built to make.
    """
    for station in ("/World/Infrastructure/INFRA_01",
                    "/World/Infrastructure/INFRA_02"):
        sensors = registry.by_station(station)
        modalities = {s.modality for s in sensors}
        assert Modality.LIDAR in modalities, f"{station} missing lidar"
        assert Modality.RADAR in modalities, f"{station} missing radar"
        assert modalities & {Modality.RGB, Modality.RGBD}, f"{station} missing camera"


def test_all_fixed_sensors_have_a_parent_station(registry):
    for spec in registry.by_mount(MountType.FIXED):
        assert spec.parent, f"{spec.sensor_id} has no parent Xform"


def test_sensor_lives_under_its_parent(registry):
    """A sensor whose prim_path isn't under its parent won't move with it."""
    for spec in registry:
        if spec.parent:
            assert spec.prim_path.startswith(spec.parent + "/"), (
                f"{spec.sensor_id}: prim_path {spec.prim_path} "
                f"is not under parent {spec.parent}"
            )


def test_rtx_sensors_flagged_as_needing_viewports(registry):
    """
    Each RTX sensor must be attached to its own viewport or it silently does
    not simulate. This test only asserts the registry knows that; the viewport
    creation itself is sim/'s job.
    """
    for spec in registry:
        assert spec.needs_viewport


def test_avatar_is_the_only_moving_mount(registry):
    """
    The whole design rests on exactly one moving entity. If a second appears,
    the 'fixed infrastructure vs. mobile agent' contrast stops being clean.
    """
    avatar_parents = {s.parent for s in registry.by_mount(MountType.AVATAR)}
    assert avatar_parents == {"/World/Avatar"}


def test_duplicate_ids_rejected():
    spec = SensorSpec(
        sensor_id="DUP",
        modality=Modality.RGB,
        mount=MountType.FIXED,
        prim_path="/World/a",
    )
    other = SensorSpec(
        sensor_id="DUP",
        modality=Modality.RGB,
        mount=MountType.FIXED,
        prim_path="/World/b",
    )
    with pytest.raises(ValueError, match="duplicate sensor_id"):
        SensorRegistry([spec, other])


def test_duplicate_prim_paths_rejected():
    a = SensorSpec("A", Modality.RGB, MountType.FIXED, "/World/same")
    b = SensorSpec("B", Modality.RGB, MountType.FIXED, "/World/same")
    with pytest.raises(ValueError, match="duplicate prim_path"):
        SensorRegistry([a, b])


def test_unknown_sensor_lookup_is_loud(registry):
    with pytest.raises(KeyError, match="unknown sensor"):
        registry.get("NO_SUCH_SENSOR")
