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

from core.observation import Modality, MountType


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


class SensorRegistry:
    """Loads and validates config/sensors.yaml."""

    def __init__(self, specs: list[SensorSpec]) -> None:
        ids = [s.sensor_id for s in specs]
        duplicates = {i for i in ids if ids.count(i) > 1}
        if duplicates:
            raise ValueError(f"duplicate sensor_id(s) in registry: {sorted(duplicates)}")

        paths = [s.prim_path for s in specs]
        dup_paths = {p for p in paths if paths.count(p) > 1}
        if dup_paths:
            raise ValueError(f"duplicate prim_path(s) in registry: {sorted(dup_paths)}")

        self._specs = {s.sensor_id: s for s in specs}

    @classmethod
    def from_yaml(cls, path: str | Path) -> SensorRegistry:
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"sensor registry not found: {path}")
        raw = yaml.safe_load(path.read_text()) or {}
        entries = raw.get("sensors")
        if not entries:
            raise ValueError(f"no 'sensors:' key (or empty) in {path}")
        return cls([cls._parse_entry(e) for e in entries])

    @staticmethod
    def _parse_entry(entry: dict[str, Any]) -> SensorSpec:
        for required in ("id", "modality", "mount", "prim_path"):
            if required not in entry:
                raise ValueError(
                    f"sensor entry missing required key '{required}': {entry}"
                )
        resolution = entry.get("resolution")
        return SensorSpec(
            sensor_id=entry["id"],
            modality=Modality(entry["modality"]),
            mount=MountType(entry["mount"]),
            prim_path=entry["prim_path"],
            parent=entry.get("parent"),
            resolution=tuple(resolution) if resolution else None,
            config=entry.get("config"),
            annotators=list(entry.get("annotators", [])),
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

    def by_station(self, parent: str) -> list[SensorSpec]:
        """
        All sensors sharing a parent Xform -- i.e. one co-located station.
        Co-location is the experimental control: three sensors at the same pose
        observing the same event isolates modality as the only variable.
        """
        return [s for s in self if s.parent == parent]
