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
from typing import Any


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
