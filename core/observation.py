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
from typing import Any, Final, Protocol, runtime_checkable


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


# --- Frames, units, conventions ----------------------------------------------
# THE CHECKLIST FOR ANYONE WRITING AN ADAPTER, and the reason it is three
# constants and not three sentences buried in a docstring.
#
# Each of these is a conversion the adapter performs on the way IN, never a
# preference a consumer accommodates on the way out. A source that puts a
# differently-defined quantity behind a correctly-spelled key does not fail --
# it produces numbers that are wrong by an axis or a factor of a hundred,
# silently, for as long as the project lasts.

#: Index of the world up-axis in `Pose.position`. Isaac/USD is z-up, so 2.
#:
#: Habitat is y-up. An adapter must SWIZZLE the components and rotate the
#: quaternion to match. Relabelling which slot is called "up" is not a
#: conversion -- done alone it mirrors the world, and a mirrored warehouse
#: renders and simulates perfectly.
UP_AXIS: Final[int] = 2

#: The unit of every distance that crosses this boundary: positions, `depth`,
#: `points`, `ranges`. Metres.
#:
#: A USD stage can be authored in centimetres (metersPerUnit = 0.01), and
#: Isaac will then report 650.0 for a station 6.5 m up without complaining
#: about anything. Read UsdGeom.GetStageMetersPerUnit() and scale in `sim/`.
#: Nothing downstream rescales, because nothing downstream can tell.
LENGTH_UNIT: Final[str] = "metre"

#: What the `depth` payload key holds: the EUCLIDEAN distance from the sensor
#: origin to the surface, in metres. The length of the ray -- not its
#: component along the optical axis.
#:
#: Isaac's `distance_to_camera` annotator is euclidean and `core/mock_source`
#: fills it euclidean, so the two sources we have agree today. Habitat's depth
#: sensor is axial z by default: identical at the principal point and smaller
#: towards the corners, which is the worst possible shape for a mismatch --
#: invisible wherever you would first look. For a pinhole camera with focal
#: length f and principal point (cx, cy) the adapter converts with
#:
#:     euclidean(u, v) = axial(u, v) * sqrt(1 + ((u-cx)/f)^2 + ((v-cy)/f)^2)
#:
#: `inf` is the legal value for "this ray hit nothing". NaN never is.
DEPTH_CONVENTION: Final[str] = "euclidean_range_from_sensor_origin"

#: The frame the `points` payload key arrives in: WORLD, in metres, the same
#: frame and units as `Pose.position`. Not the sensor frame.
#:
#: This was the last open question on the list below and is now closed, in the
#: direction the rest of this file already pointed: every other quantity that
#: crosses the boundary is world-frame and metric, and a cloud that is not is
#: the odd one out. Isaac hands you the opposite -- `generic-model-output` is
#: sensor-local by default -- so `sim/observation_adapter.py` applies the
#: sensor-to-world matrix, and `core/mock_source.py` adds the mount position.
#:
#: What it costs to skip is specifically FUSION. One cloud in the sensor frame
#: is self-consistent, varies with the scene, and plots correctly; it is only
#: when two stations are combined that both land on the origin, overlapping,
#: and neither looks wrong on its own. Checked by
#: `test_range_clouds_are_in_the_world_frame` in tests/contract.py, which needs
#: a sensor away from the origin and a target of known world position to see
#: the difference at all.
POINTS_FRAME: Final[str] = "world"

#: What the `semantic` payload key holds: CLASS ids. One id per category,
#: shared by every instance of it -- two people are both id 1 -- and every id
#: that appears in the map has a name in `semantic_labels`.
#:
#: Isaac's `semantic_segmentation` annotator is class-based already. Habitat's
#: semantic sensor returns INSTANCE ids: two chairs get two ids, and turning
#: them into categories needs `sim.semantic_scene`, per dataset. An adapter
#: that skipped that step would hand over a map that looks right, segments
#: correctly, and quietly means something else.
#:
#: The completeness half is what makes this checkable, and it is checked --
#: see `test_every_semantic_id_in_the_map_has_a_name` in tests/contract.py.
#: Raw instance ids fail it immediately: thousands of ids against a mapping of
#: a few dozen categories.
SEMANTIC_ID_CONVENTION: Final[str] = "class_ids_with_complete_label_mapping"


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

# THE SHAPE AND DTYPE BEHIND EACH KEY. The array *library* is the source's
# business; these are not. W and H are the sensor's declared `resolution`.
#
#   rgb        (H, W, 3) uint8    THREE channels. Both Isaac's rgb annotator
#                                 and Habitat's color sensor hand you (H, W, 4)
#                                 RGBA, so both adapters slice the alpha off.
#                                 Nobody reading only the key name would know
#                                 to, which is why it is written here.
#   depth      (H, W)   float     metres, per DEPTH_CONVENTION. `inf` for a
#                                 ray that hit nothing; never NaN.
#   semantic   (H, W)   integer   class ids, per SEMANTIC_ID_CONVENTION.
#   points     (N, 3)   float     metres. N varies tick to tick -- that is the
#                                 signal, not a defect.
#
# Enforced by tests/contract.py, which is where a source finds out.

# NOTHING LEFT UNPINNED -- and the last entry is worth recording, because of
# how it closed. Renaming an Isaac annotator to a portable key buys
# portability and spends the definition that used to live in the name; units,
# up-axis, depth semantics and semantic ids were the earlier entries and are
# all fixed above.
#
# `points` was the last, and stayed open on the grounds that pinning it "would
# be a claim no suite could check" -- true only for as long as the check was
# imagined as a property of one cloud. It is not. It needs a sensor at a known
# non-origin pose and a target at a known world position, and then it is
# decidable in one tick. See POINTS_FRAME above and
# test_range_clouds_are_in_the_world_frame in tests/contract.py. The
# generalisation is the useful part: "no suite could check it" is worth
# doubting once, in case what it means is that the fixture was too symmetric.

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
    """
    World-frame pose. Metres, z-up (`UP_AXIS`), quaternion (w, x, y, z).

    All three of those are Isaac Sim's conventions and all three are things a
    Habitat or ROS adapter has to convert to rather than report in. See the
    conventions block above -- it is the whole checklist.
    """

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

    #: What the mount did to arrive at this reading, if it did anything.
    #:
    #: `MountType.AVATAR` is documented -- in CLAUDE.md, in `MountType` above,
    #: and in `core.memory.interfaces.Evidence` -- as producing embodied
    #: EXPERIENCE rather than allocentric state, on the grounds that an
    #: experience has an action before it and a consequence after it. Until
    #: this field existed the contract could represent neither, so every
    #: consumer was told it held experience and handed a reading indis-
    #: tinguishable from state. A documented claim with no representation is
    #: the gap; this closes it.
    #:
    #: Deliberately untyped and optional. What an action IS depends on the
    #: simulator -- a keyboard event here, a discrete `move_forward` in
    #: Habitat, a velocity command on a real robot -- and inventing a taxonomy
    #: before there is code that consumes one would be guessing. It stays
    #: `Any` until something in Layer 4 needs it to be more.
    #:
    #: A FIXED sensor must leave this None. Infrastructure does not act, and a
    #: source that stamps the avatar's action onto a ceiling camera has
    #: destroyed the distinction the field exists to preserve. Checked by
    #: `test_a_fixed_sensor_never_carries_an_action` in tests/contract.py.
    action: Any | None = None

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
        if self.action is not None:
            # Same discipline as the payload below: printable or named, never
            # dragged along whole, because `action` is Any and could be a
            # tensor as easily as a string.
            out["action"] = (
                self.action
                if isinstance(self.action, (str, int, float, bool))
                else type(self.action).__name__
            )
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
