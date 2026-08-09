"""
The observation contract. Layer 3.

Design this once and keep it stable: every downstream consumer takes an
Observation and nothing else. When the same memory module has to run against
Habitat for benchmark comparison, this file is the entire adapter surface.

NOTHING IN THIS MODULE MAY IMPORT omni, pxr, OR isaacsim.
Enforced by scripts/check_layer_boundary.sh in CI.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Protocol, runtime_checkable


class Modality(str, Enum):
    """Sensor modalities. str-valued so YAML round-trips without a converter."""

    RGB = "rgb"
    RGBD = "rgbd"
    DEPTH = "depth"
    LIDAR = "lidar"
    RADAR = "radar"
    SEMANTIC = "semantic"


class MountType(str, Enum):
    """
    Fixed infrastructure vs. robot-mounted vs. carried by the moving avatar.

    This distinction is the research claim, not a bookkeeping detail: FIXED
    sensors can only ever produce *state*, while a moving mount is the only
    possible source of *experience*. Anything downstream that fuses these two
    needs to know which it is holding, so it is a first-class field.
    """

    FIXED = "fixed"
    ROBOT = "robot"
    AVATAR = "avatar"


# --- What is inside `data` ---------------------------------------------------
# The payload types stay loose -- an array library is an implementation detail
# of whoever produced them -- but the *keys* cannot, or no consumer can be
# written before a source exists. These two tables are the whole agreement.
#
# The keys are deliberately not Isaac's annotator names. 'distance_to_camera'
# is Replicator vocabulary; 'depth' is something Habitat, a ROS bag, or a
# recorded dataset can equally well produce. Naming the payload after the
# simulator that happens to fill it is how a contract stops being portable.

#: Registry annotator -> the payload key it is responsible for filling.
#: Every annotator a sensor may legally declare appears here, which is what
#: makes `core.registry.KNOWN_ANNOTATORS` a closed set: an annotator with no
#: defined payload key is an annotator no consumer could ever read.
ANNOTATOR_DATA_KEYS: dict[str, str] = {
    "rgb": "rgb",
    "distance_to_camera": "depth",
    "semantic_segmentation": "semantic",
    # RTX range sensors return one opaque buffer that sim/ parses; what reaches
    # Layer 3 is the point cloud, not the buffer.
    "generic-model-output": "points",
}

#: Modality -> payload keys any source MUST provide, whatever its annotators.
#: A source may always add more; it may never provide less.
MODALITY_DATA_KEYS: dict[Modality, frozenset[str]] = {
    Modality.RGB: frozenset({"rgb"}),
    Modality.RGBD: frozenset({"rgb", "depth"}),
    Modality.DEPTH: frozenset({"depth"}),
    Modality.SEMANTIC: frozenset({"semantic"}),
    Modality.LIDAR: frozenset({"points"}),
    Modality.RADAR: frozenset({"points"}),
}


@dataclass(frozen=True)
class Pose:
    """World-frame pose. Quaternion is (w, x, y, z) -- Isaac Sim's convention."""

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        if len(self.position) != 3:
            raise ValueError(f"position must be 3 floats, got {len(self.position)}")
        if len(self.orientation) != 4:
            raise ValueError(
                f"orientation must be 4 floats (w,x,y,z), got {len(self.orientation)}"
            )


@dataclass
class Observation:
    """
    One reading from one sensor at one instant.

    `data` stays deliberately loose (arrays, point clouds, label maps) because
    its shape is modality-specific. Everything *around* it is strict, because
    that is what fusion and logging actually index on.
    """

    sensor_id: str
    timestamp: float
    modality: Modality
    mount: MountType
    pose: Pose
    intrinsics: dict[str, Any] | None = None
    data: dict[str, Any] = field(default_factory=dict)

    def summary(self) -> dict[str, Any]:
        """
        Small, printable, array-free description. This is what the inspector
        panel and the logs want -- never the payload itself.
        """
        out: dict[str, Any] = {
            "sensor_id": self.sensor_id,
            "timestamp": round(self.timestamp, 4),
            "modality": self.modality.value,
            "mount": self.mount.value,
            "position": tuple(round(v, 3) for v in self.pose.position),
        }
        for key, value in self.data.items():
            if hasattr(value, "shape"):
                out[f"{key}_shape"] = tuple(value.shape)
            elif isinstance(value, (list, tuple)):
                out[f"{key}_len"] = len(value)
            elif isinstance(value, (int, float, str, bool)):
                out[key] = value
        return out

    def required_data_keys(self) -> frozenset[str]:
        """Payload keys this observation's modality obliges its source to fill."""
        return MODALITY_DATA_KEYS[self.modality]


@runtime_checkable
class ObservationSource(Protocol):
    """
    Anything that produces observations over time. Layer 3's only input.

    Two implementations are planned and must be interchangeable:
    `core.mock_source.MockObservationSource` (no simulator, runs on a laptop)
    and `sim/observation_adapter.py` (Isaac Sim, server-only, task S11). A
    third -- Habitat -- is what the whole exercise is insurance against.

    The suite in tests/contract.py is written against this protocol and
    nothing below it. Point it at either source; it must pass unchanged. That
    is the actual deliverable: a contract, not a fake.

    Deliberately minimal. There is no per-sensor `read(sensor_id)` because a
    live simulator cannot honour one -- you cannot read one RTX sensor without
    stepping the renderer, and pretending otherwise would bake an assumption
    into the contract that only the mock could satisfy.
    """

    @property
    def sensor_ids(self) -> tuple[str, ...]:
        """Every sensor this source can produce readings for. Stable order."""
        ...

    @property
    def time(self) -> float:
        """Seconds of simulated time elapsed. Starts at 0.0."""
        ...

    def step(self, dt: float | None = None) -> list[Observation]:
        """
        Advance simulated time and return this tick's readings.

        At most one Observation per sensor_id, every one sharing the tick's
        timestamp. A source may return fewer than `sensor_ids` (a sensor can
        be slower than the tick rate) but never an id it did not declare.
        """
        ...

    def close(self) -> None:
        """Release whatever the source holds. Must be safe to call twice."""
        ...
