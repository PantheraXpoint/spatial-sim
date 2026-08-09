"""
The sensor registry. Layer 2.

One YAML file is the single source of truth for every sensor in the scene.
Viewport creation, annotator attachment, the inspector panel, and all future
logging read from here. Adding a sensor means adding a YAML entry, not editing
five scripts.

NOTHING IN THIS MODULE MAY IMPORT omni, pxr, OR isaacsim.
The registry is *parsed* here and *acted on* in sim/. That split is what lets
the whole registry be unit-tested on a laptop with no GPU.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from core.observation import ANNOTATOR_DATA_KEYS, Modality, MountType

# --- What counts as what -----------------------------------------------------
# A "camera" here is anything that renders an image plane and therefore needs a
# resolution. Lidar and radar are range sensors: they have no resolution, they
# have a profile.
CAMERA_MODALITIES: frozenset[Modality] = frozenset(
    {Modality.RGB, Modality.RGBD, Modality.DEPTH, Modality.SEMANTIC}
)
RANGE_MODALITIES: frozenset[Modality] = frozenset({Modality.LIDAR, Modality.RADAR})

# Sensor kinds, as named in error messages. A station's composition is judged
# on these, not on individual modalities.
_KIND_CAMERA = "camera"
_KIND_LIDAR = "lidar"
_KIND_RADAR = "radar"

# --- Station types -----------------------------------------------------------
# What a station declares itself to be, in config/scene.yaml. Composition is
# NOT inferred from the sensor census: a camera+lidar station is a legitimate
# ablation, and a lidar added to a robot platform must not silently create an
# obligation to bolt a radar onto a TurtleBot. If a station must be complete,
# it says so.
STATION_TYPE_REQUIREMENTS: dict[str, frozenset[str]] = {
    "multimodal": frozenset({_KIND_CAMERA, _KIND_LIDAR, _KIND_RADAR}),
}

# --- Annotator allow-list ----------------------------------------------------
# A misspelled annotator name does not raise at runtime: the annotator attaches,
# returns nothing, and the demo looks like a broken scene. So the set of legal
# names is closed, and a typo fails here, at load, naming the sensor.
#
# Exactly the four names in use, no more. Adding one is a deliberate edit and
# the new name must be verified against the Isaac Sim 6.0 docs first -- do not
# add a name recalled from a 4.x tutorial. These four are additionally verified
# empirically when the camera and the semantics/radar paths come up at S7/S10;
# a name that has not been through that has not been confirmed to work.
#
# The set is the key set of core.observation.ANNOTATOR_DATA_KEYS rather than a
# second list, so adding an annotator here is impossible without saying which
# payload key it fills. An annotator no consumer can read is the same silent
# failure as a misspelled one, one layer up.
#   rgb, distance_to_camera, semantic_segmentation -- Replicator camera annotators
#   generic-model-output -- RTX range sensors; read via sensor.get_data() in 6.x
KNOWN_ANNOTATORS: frozenset[str] = frozenset(ANNOTATOR_DATA_KEYS)


@dataclass(frozen=True)
class StationSpec:
    """
    One mounting Xform from config/scene.yaml -- the pose several co-located
    sensors share. Only the fields the registry validates against are parsed
    here; sim/ reads the file itself for position and mount hardware.
    """

    station_id: str
    prim_path: str
    station_type: str | None = None


@dataclass(frozen=True)
class SensorSpec:
    """Declarative description of one sensor. No simulator objects."""

    sensor_id: str
    modality: Modality
    mount: MountType
    prim_path: str
    parent: str | None = None
    resolution: tuple[int, int] | None = None
    config: str | None = None          # e.g. Isaac lidar/radar profile name
    annotators: list[str] = field(default_factory=list)
    notes: str = ""

    @property
    def needs_viewport(self) -> bool:
        """
        RTX sensors do not simulate unless attached to their own viewport.
        There is no warning when you forget -- the sensor simply returns
        nothing, which reads exactly like a broken scene.
        """
        return self.modality in (
            Modality.RGB,
            Modality.RGBD,
            Modality.DEPTH,
            Modality.LIDAR,
            Modality.RADAR,
            Modality.SEMANTIC,
        )

    @property
    def kind(self) -> str:
        """Coarse sensor kind -- what a station is judged multi-modal on."""
        if self.modality in RANGE_MODALITIES:
            return self.modality.value
        return _KIND_CAMERA


def _validate_spec(spec: SensorSpec) -> list[str]:
    """Everything checkable about one sensor in isolation."""
    problems: list[str] = []
    sid = spec.sensor_id

    if spec.mount is MountType.FIXED and not spec.parent:
        problems.append(
            f"{sid}: mount is 'fixed' but it has no parent station Xform. "
            f"A fixed sensor with no station has no pose to inherit."
        )

    if spec.parent and not spec.prim_path.startswith(spec.parent + "/"):
        problems.append(
            f"{sid}: prim_path '{spec.prim_path}' is not under its parent "
            f"'{spec.parent}', so it will not inherit the station's transform."
        )

    if spec.modality in CAMERA_MODALITIES and spec.resolution is None:
        problems.append(
            f"{sid}: modality '{spec.modality.value}' is a camera and needs a "
            f"'resolution: [w, h]'."
        )
    if spec.modality in RANGE_MODALITIES and spec.resolution is not None:
        problems.append(
            f"{sid}: modality '{spec.modality.value}' has no image plane, so "
            f"'resolution' is meaningless here -- remove it "
            f"(the beam pattern comes from 'config')."
        )

    if spec.modality in RANGE_MODALITIES and not spec.config:
        problems.append(
            f"{sid}: modality '{spec.modality.value}' needs a 'config' profile "
            f"name (e.g. Example_Rotary) -- it cannot be created without one."
        )

    unknown = [a for a in spec.annotators if a not in KNOWN_ANNOTATORS]
    if unknown:
        problems.append(
            f"{sid}: unknown annotator(s) {sorted(unknown)}. "
            f"Known: {sorted(KNOWN_ANNOTATORS)}. A misspelled annotator attaches "
            f"fine and returns nothing, so it is rejected here instead."
        )

    return problems


def _validate_stations(
    specs: list[SensorSpec], stations: list[StationSpec]
) -> list[str]:
    """
    Check each station against what it *declares* itself to be in scene.yaml.

    A station declared `multimodal` must carry all three kinds: camera, lidar,
    and radar at one pose observing one event isolates modality as the only
    variable, and two of three is a confound rather than a control. Everything
    else is unconstrained -- a camera+lidar station is a valid ablation, and a
    lidar on a robot platform is a valid experiment.
    """
    problems: list[str] = []
    members: dict[str, list[SensorSpec]] = {}
    for spec in specs:
        if spec.parent:
            members.setdefault(spec.parent, []).append(spec)

    for station in sorted(stations, key=lambda s: s.station_id):
        if station.station_type is None:
            continue
        required = STATION_TYPE_REQUIREMENTS.get(station.station_type)
        if required is None:
            problems.append(
                f"station '{station.station_id}': unknown station_type "
                f"'{station.station_type}'. Known: "
                f"{sorted(STATION_TYPE_REQUIREMENTS)}"
            )
            continue

        mounted = members.get(station.prim_path, [])
        missing = required - {s.kind for s in mounted}
        if missing:
            problems.append(
                f"station '{station.station_id}' is declared "
                f"'{station.station_type}' but is missing {sorted(missing)}. "
                f"Sensors registered at {station.prim_path}: "
                f"{sorted(s.sensor_id for s in mounted)}."
            )
    return problems


class SensorRegistry:
    """
    Loads and validates config/sensors.yaml.

    `stations` comes from config/scene.yaml and is what station composition is
    checked against. Omit it and the per-sensor checks still run; only the
    "this station declares itself multi-modal" rule goes unenforced.
    """

    def __init__(
        self,
        specs: list[SensorSpec],
        stations: list[StationSpec] | None = None,
    ) -> None:
        stations = list(stations or [])
        problems: list[str] = []

        ids = [s.sensor_id for s in specs]
        duplicates = {i for i in ids if ids.count(i) > 1}
        if duplicates:
            problems.append(f"duplicate sensor_id(s) in registry: {sorted(duplicates)}")

        paths = [s.prim_path for s in specs]
        dup_paths = {p for p in paths if paths.count(p) > 1}
        if dup_paths:
            problems.append(f"duplicate prim_path(s) in registry: {sorted(dup_paths)}")

        station_ids = [s.station_id for s in stations]
        dup_stations = {i for i in station_ids if station_ids.count(i) > 1}
        if dup_stations:
            problems.append(f"duplicate station id(s) in scene: {sorted(dup_stations)}")

        for spec in specs:
            problems.extend(_validate_spec(spec))
        problems.extend(_validate_stations(specs, stations))

        if problems:
            raise ValueError(
                "invalid sensor registry:\n  - " + "\n  - ".join(problems)
            )

        self._specs = {s.sensor_id: s for s in specs}
        self._stations = {s.station_id: s for s in stations}

    @classmethod
    def from_yaml(
        cls, path: str | Path, scene_path: str | Path | None = None
    ) -> SensorRegistry:
        """
        Load the sensor registry, and the station declarations it is validated
        against. `scene_path` defaults to scene.yaml sitting next to the sensor
        file; if there isn't one, station composition simply goes unchecked.
        """
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"sensor registry not found: {path}")
        raw = yaml.safe_load(path.read_text()) or {}
        entries = raw.get("sensors")
        if not entries:
            raise ValueError(f"no 'sensors:' key (or empty) in {path}")

        if scene_path is None:
            sibling = path.parent / "scene.yaml"
            scene_path = sibling if sibling.exists() else None
        stations = cls._parse_scene(scene_path) if scene_path is not None else []

        return cls([cls._parse_entry(e) for e in entries], stations)

    @staticmethod
    def _parse_scene(scene_path: str | Path) -> list[StationSpec]:
        scene_path = Path(scene_path)
        if not scene_path.exists():
            raise FileNotFoundError(f"scene file not found: {scene_path}")
        raw = yaml.safe_load(scene_path.read_text()) or {}
        stations = []
        for entry in raw.get("stations") or []:
            for required in ("id", "prim_path"):
                if required not in entry:
                    raise ValueError(
                        f"station entry missing required key '{required}': {entry}"
                    )
            stations.append(
                StationSpec(
                    station_id=entry["id"],
                    prim_path=entry["prim_path"],
                    station_type=entry.get("station_type"),
                )
            )
        return stations

    @staticmethod
    def _parse_entry(entry: dict[str, Any]) -> SensorSpec:
        if not isinstance(entry, dict):
            # ValueError, not TypeError: every registry problem should surface
            # to the caller as one kind of "your YAML is wrong".
            raise ValueError(f"sensor entry is not a mapping: {entry!r}")  # noqa: TRY004
        for required in ("id", "modality", "mount", "prim_path"):
            if required not in entry:
                raise ValueError(
                    f"sensor entry missing required key '{required}': {entry}"
                )
        sid = entry["id"]

        # Enum() alone raises "'foo' is not a valid Modality", which does not
        # say *which* entry is wrong -- useless in a 12-sensor file.
        try:
            modality = Modality(entry["modality"])
        except ValueError:
            raise ValueError(
                f"sensor '{sid}': unknown modality '{entry['modality']}'. "
                f"Known: {sorted(m.value for m in Modality)}"
            ) from None
        try:
            mount = MountType(entry["mount"])
        except ValueError:
            raise ValueError(
                f"sensor '{sid}': unknown mount '{entry['mount']}'. "
                f"Known: {sorted(m.value for m in MountType)}"
            ) from None

        resolution = entry.get("resolution")
        if resolution is not None:
            if (
                not isinstance(resolution, (list, tuple))
                or len(resolution) != 2
                or not all(isinstance(v, int) and v > 0 for v in resolution)
            ):
                raise ValueError(
                    f"sensor '{sid}': resolution must be [width, height] with two "
                    f"positive integers, got {resolution!r}"
                )
            resolution = (resolution[0], resolution[1])

        annotators = entry.get("annotators", [])
        if not isinstance(annotators, (list, tuple)) or not all(
            isinstance(a, str) for a in annotators
        ):
            raise ValueError(
                f"sensor '{sid}': annotators must be a list of strings, "
                f"got {annotators!r}"
            )

        return SensorSpec(
            sensor_id=sid,
            modality=modality,
            mount=mount,
            prim_path=entry["prim_path"],
            parent=entry.get("parent"),
            resolution=resolution,
            config=entry.get("config"),
            annotators=list(annotators),
            notes=entry.get("notes", ""),
        )

    def __len__(self) -> int:
        return len(self._specs)

    def __iter__(self):
        return iter(self._specs.values())

    def __contains__(self, sensor_id: object) -> bool:
        return sensor_id in self._specs

    def get(self, sensor_id: str) -> SensorSpec:
        if sensor_id not in self._specs:
            raise KeyError(
                f"unknown sensor '{sensor_id}'. Known: {sorted(self._specs)}"
            )
        return self._specs[sensor_id]

    def by_modality(self, modality: Modality) -> list[SensorSpec]:
        return [s for s in self if s.modality == modality]

    def by_mount(self, mount: MountType) -> list[SensorSpec]:
        return [s for s in self if s.mount == mount]

    @property
    def stations(self) -> list[StationSpec]:
        """Station declarations from scene.yaml. Empty if none were loaded."""
        return list(self._stations.values())

    def by_station(self, parent: str) -> list[SensorSpec]:
        """
        All sensors sharing a parent Xform -- i.e. one co-located station.
        Co-location is the experimental control: three sensors at the same pose
        observing the same event isolates modality as the only variable.
        """
        return [s for s in self if s.parent == parent]
