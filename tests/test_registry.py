"""
Registry tests. These MUST pass on the MacBook with no GPU and no simulator.
If they ever can't run there, the layer boundary has leaked.
"""

import textwrap
from pathlib import Path

import pytest
import yaml

from core.observation import Modality, MountType
from core.registry import (
    KNOWN_ANNOTATORS,
    SensorRegistry,
    SensorSpec,
    StationSpec,
)

REGISTRY_PATH = "config/sensors.yaml"
SCENE_PATH = "config/scene.yaml"

STATION = "/World/Infrastructure/TEST"


@pytest.fixture(scope="module")
def registry() -> SensorRegistry:
    return SensorRegistry.from_yaml(REGISTRY_PATH)


# --- Builders for the negative tests -----------------------------------------
# Each returns a spec that is valid on its own; a test breaks exactly one field
# so that the assertion is about that field and nothing else.

def _cam(sensor_id: str = "TEST_CAM", **overrides) -> SensorSpec:
    fields = {
        "sensor_id": sensor_id,
        "modality": Modality.RGBD,
        "mount": MountType.FIXED,
        "prim_path": f"{STATION}/cam",
        "parent": STATION,
        "resolution": (640, 480),
        "annotators": ["rgb"],
    }
    fields.update(overrides)
    return SensorSpec(**fields)


def _lidar(sensor_id: str = "TEST_LIDAR", **overrides) -> SensorSpec:
    fields = {
        "sensor_id": sensor_id,
        "modality": Modality.LIDAR,
        "mount": MountType.FIXED,
        "prim_path": f"{STATION}/lidar",
        "parent": STATION,
        "config": "Example_Rotary",
        "annotators": ["generic-model-output"],
    }
    fields.update(overrides)
    return SensorSpec(**fields)


def _radar(sensor_id: str = "TEST_RADAR", **overrides) -> SensorSpec:
    fields = {
        "sensor_id": sensor_id,
        "modality": Modality.RADAR,
        "mount": MountType.FIXED,
        "prim_path": f"{STATION}/radar",
        "parent": STATION,
        "config": "Example_Radar",
        "annotators": ["generic-model-output"],
    }
    fields.update(overrides)
    return SensorSpec(**fields)


def _full_station() -> list[SensorSpec]:
    return [_cam(), _lidar(), _radar()]


def _declared(station_type: str | None = "multimodal") -> list[StationSpec]:
    return [StationSpec("TEST", STATION, station_type)]


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
    # Paths come from the declarations, not from literals here. They used to be
    # spelled out, and went stale the moment S7 confirmed INFRA_01's real path
    # -- a test asserting co-location at a prim path that does not exist is
    # worse than no test.
    for station in [s.prim_path for s in registry.stations]:
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

    The path is read from scene.yaml rather than written out here. It used to
    be the literal '/World/Avatar', which made this test fail when S6 corrected
    the stage root to '/Root' -- a green test asserting a prim path that does
    not exist is worse than no test, and duplicating the path in two files is
    what let them disagree.
    """
    avatar_parents = {s.parent for s in registry.by_mount(MountType.AVATAR)}
    declared = yaml.safe_load(Path(SCENE_PATH).read_text())["avatar"]["prim_path"]
    assert avatar_parents == {declared}


def test_duplicate_ids_rejected():
    with pytest.raises(ValueError, match="duplicate sensor_id"):
        SensorRegistry([_cam("DUP"), _cam("DUP", prim_path=f"{STATION}/cam_b")])


def test_duplicate_prim_paths_rejected():
    """
    Predates the other validators and must survive them: two sensors at one
    path means the second silently overwrites the first. Both specs here are
    otherwise valid, so this can only fail for the reason it is named for.
    """
    with pytest.raises(ValueError, match="duplicate prim_path"):
        SensorRegistry([_cam("A"), _cam("B")])


def test_unknown_sensor_lookup_is_loud(registry):
    with pytest.raises(KeyError, match="unknown sensor"):
        registry.get("NO_SUCH_SENSOR")


# =============================================================================
# Validation at load. Every check below exists because the failure it catches
# produces no error at runtime -- just a sensor that quietly returns nothing.
# =============================================================================


def test_a_valid_station_loads():
    """Guard against the validators rejecting a legitimate registry."""
    assert len(SensorRegistry(_full_station(), _declared())) == 3


def test_fixed_sensor_without_parent_rejected():
    with pytest.raises(ValueError, match="TEST_CAM.*no parent station"):
        SensorRegistry([_cam(parent=None, prim_path="/World/loose_cam")])


def test_sensor_outside_its_parent_rejected():
    """A sensor not under its station Xform silently keeps its own pose."""
    with pytest.raises(ValueError, match="not under its parent"):
        SensorRegistry(
            [_cam(prim_path="/World/Elsewhere/cam"), _lidar(), _radar()]
        )


def test_station_declared_multimodal_must_have_all_three():
    """
    Camera + lidar and no radar is a confound, not a control: any difference
    between stations could then be modality *or* instrumentation. The station
    said it was complete, so the registry holds it to that.
    """
    with pytest.raises(ValueError, match="declared 'multimodal'.*missing.*radar"):
        SensorRegistry([_cam(), _lidar()], _declared())


def test_station_declared_multimodal_with_no_sensors_rejected():
    """A declared station nobody registered sensors under is a wiring bug."""
    with pytest.raises(ValueError, match="TEST.*declared 'multimodal'.*missing"):
        SensorRegistry([], _declared())


def test_undeclared_station_composition_is_unconstrained():
    """
    Composition is never inferred from the sensor census. A camera+lidar station
    is a legitimate ablation, and adding a lidar to a robot platform must not
    silently demand a radar be bolted to a TurtleBot.
    """
    assert len(SensorRegistry([_cam(), _lidar()], _declared(None))) == 2
    assert len(SensorRegistry([_cam(), _lidar()])) == 2

    bot = "/World/Robots/BOT_01"
    lidar_on_a_turtlebot = [
        _cam("BOT_01_CAM", mount=MountType.ROBOT, parent=bot,
             prim_path=f"{bot}/cam"),
        _lidar("BOT_01_LIDAR", mount=MountType.ROBOT, parent=bot,
               prim_path=f"{bot}/lidar"),
    ]
    assert len(
        SensorRegistry(lidar_on_a_turtlebot, [StationSpec("BOT_01", bot)])
    ) == 2


def test_unknown_station_type_rejected():
    """A typo'd station_type would otherwise silently enforce nothing."""
    with pytest.raises(ValueError, match="TEST.*unknown station_type 'multimodel'"):
        SensorRegistry(_full_station(), _declared("multimodel"))


def test_real_scene_declarations_are_loaded_and_enforced(registry):
    """
    from_yaml picks up scene.yaml next to sensors.yaml. If it silently didn't,
    every station rule above would be dead code against the real config.
    """
    declared = {s.station_id: s for s in registry.stations}
    assert declared, "no stations loaded from config/scene.yaml"
    assert declared["INFRA_01"].station_type == "multimodal"
    assert declared["INFRA_02"].station_type == "multimodal"


def test_scene_file_can_be_pointed_at_explicitly(tmp_path):
    scene = tmp_path / "elsewhere.yaml"
    scene.write_text("stations: []\n")
    loaded = SensorRegistry.from_yaml(REGISTRY_PATH, scene_path=str(scene))
    assert loaded.stations == []


def test_camera_without_resolution_rejected():
    with pytest.raises(ValueError, match="TEST_CAM.*is a camera and needs"):
        SensorRegistry([_cam(resolution=None), _lidar(), _radar()])


@pytest.mark.parametrize("builder", [_lidar, _radar])
def test_range_sensor_with_resolution_rejected(builder):
    """Lidar and radar have no image plane; a resolution there is a mistake."""
    with pytest.raises(ValueError, match="no image plane"):
        SensorRegistry([s for s in _full_station() if s.modality
                        is not builder().modality] + [builder(resolution=(640, 480))])


@pytest.mark.parametrize("builder", [_lidar, _radar])
def test_range_sensor_without_config_profile_rejected(builder):
    with pytest.raises(ValueError, match="needs a 'config' profile"):
        SensorRegistry([s for s in _full_station() if s.modality
                        is not builder().modality] + [builder(config=None)])


def test_unknown_annotator_rejected():
    """
    A typo'd annotator attaches without complaint and returns an empty buffer.
    This is the whole reason the annotator set is closed.
    """
    with pytest.raises(ValueError, match="unknown annotator"):
        SensorRegistry(
            [_cam(annotators=["rgb", "semantic_segmentaton"]), _lidar(), _radar()]
        )


def test_every_annotator_in_the_real_registry_is_known(registry):
    for spec in registry:
        for annotator in spec.annotators:
            assert annotator in KNOWN_ANNOTATORS, (
                f"{spec.sensor_id}: '{annotator}' is not in KNOWN_ANNOTATORS"
            )


def test_all_problems_reported_at_once():
    """One load, one list. Fixing them one round-trip at a time is miserable."""
    with pytest.raises(ValueError) as excinfo:
        SensorRegistry([_cam(resolution=None, annotators=["rbg"])])
    message = str(excinfo.value)
    assert "resolution" in message
    assert "unknown annotator" in message
    assert "multi-modal" not in message  # single-kind station: not a problem


# --- Parse-time checks, before a spec even exists ----------------------------


def _write(tmp_path, body: str) -> str:
    path = tmp_path / "sensors.yaml"
    path.write_text(textwrap.dedent(body))
    return str(path)


def test_malformed_yaml_names_the_bad_entry(tmp_path):
    """
    The gate for this task: a bad registry fails at load, and the message says
    which entry is bad. 'validation failed' with no name is not usable.
    """
    path = _write(tmp_path, """
        sensors:
          - id: GOOD_CAM
            modality: rgbd
            mount: fixed
            parent: /World/Infrastructure/S
            prim_path: /World/Infrastructure/S/cam
            resolution: [640, 480]
            annotators: [rgb]
          - id: GOOD_RADAR
            modality: radar
            mount: fixed
            parent: /World/Infrastructure/S
            prim_path: /World/Infrastructure/S/radar
            config: Example_Radar
            annotators: [generic-model-output]
          - id: BROKEN_LIDAR
            modality: lidar
            mount: fixed
            parent: /World/Infrastructure/S
            prim_path: /World/Infrastructure/S/lidar
            annotators: [generic-model-output]
    """)
    with pytest.raises(ValueError) as excinfo:
        SensorRegistry.from_yaml(path)
    message = str(excinfo.value)
    assert "BROKEN_LIDAR" in message      # names the offender
    assert "GOOD_CAM" not in message      # and only the offender
    assert "config" in message            # and says what is wrong with it


def test_unknown_modality_names_the_entry(tmp_path):
    path = _write(tmp_path, """
        sensors:
          - id: TYPO_CAM
            modality: rgdb
            mount: fixed
            prim_path: /World/S/cam
    """)
    with pytest.raises(ValueError, match="TYPO_CAM.*unknown modality 'rgdb'"):
        SensorRegistry.from_yaml(path)


def test_unknown_mount_names_the_entry(tmp_path):
    path = _write(tmp_path, """
        sensors:
          - id: TYPO_MOUNT
            modality: rgb
            mount: drone
            prim_path: /World/S/cam
    """)
    with pytest.raises(ValueError, match="TYPO_MOUNT.*unknown mount 'drone'"):
        SensorRegistry.from_yaml(path)


@pytest.mark.parametrize("bad", ["1280", "[1280]", "[1280, 720, 3]", "[0, 720]"])
def test_bad_resolution_shape_rejected(tmp_path, bad):
    path = _write(tmp_path, f"""
        sensors:
          - id: BAD_RES
            modality: rgb
            mount: avatar
            parent: /World/Avatar
            prim_path: /World/Avatar/cam
            resolution: {bad}
    """)
    with pytest.raises(ValueError, match="BAD_RES.*resolution must be"):
        SensorRegistry.from_yaml(path)
