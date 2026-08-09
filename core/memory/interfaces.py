"""
The closed loop, as interfaces. Layer 4.

    persistent memory ──> prediction ──> observation ──> residual ──┐
            ^                                                       │
            └───────────────────── revision ────────────────────────┘

Every persistent-memory system in the literature is write-only from
perception: perception writes, planning reads, and nothing ever writes back.
The arrow that closes the circle above is the contribution, and it is the only
reason this package exists.

A residual is therefore not an error to be minimised away. It is the
measurement, and it means two different things depending on the world:

    static scene   ->  residual == model noise
    dynamic scene  ->  residual == change detection

The same number, both times. That is why the loop has to exist and be
runnable before anything interesting can be said about either half -- and why
`tests/test_memory.py` asserts exactly those two sentences.

NOTHING HERE IS CLEVER, DELIBERATELY. This file names the parts and fixes the
signatures between them. `core/memory/baseline.py` is the dumbest thing that
makes the arrows connect; it is a placeholder for research code, not the
research code.

Dependencies: `core.observation` and nothing else. Not the registry (memory
does not need to know how sensors were declared), not the mock, and -- rule 2
-- never omni, pxr, or isaacsim.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Protocol, runtime_checkable

from core.observation import Modality, MountType, Observation, Pose

# --- State vs. experience ----------------------------------------------------


class Evidence(str, Enum):
    """
    What kind of thing a reading *is* -- a different question from which
    sensor produced it.

    A ceiling camera watches an event happen. An avatar camera is carried
    through it. Those two readings can be pixel-identical and still not be
    interchangeable: only the second one has an action before it and a
    consequence after it. A fixed camera never acts, never collides, and never
    pays a traversal cost, so it can only ever report where things are, never
    what it costs to go there.

    Fusing the two without knowing which is which is how a map of a room
    becomes indistinguishable from a record of walking around one. So it is
    carried on every residual and the two are stored apart.
    """

    STATE = "state"            # allocentric: a viewpoint that never acts
    EXPERIENCE = "experience"  # egocentric: a viewpoint carried through acting


_EVIDENCE_BY_MOUNT: dict[MountType, Evidence] = {
    MountType.FIXED: Evidence.STATE,
    # A robot platform is STATE here, and that is a fact about this scene
    # rather than about robots: ours are static observation posts (CLAUDE.md),
    # so they never act. The day a locomotion policy is switched on, this line
    # is the one that has to change -- and readings that silently kept arriving
    # as STATE would be exactly the bug that hid it.
    MountType.ROBOT: Evidence.STATE,
    MountType.AVATAR: Evidence.EXPERIENCE,
}


def evidence_of(mount: MountType) -> Evidence:
    """Which kind of evidence a mount can produce. No silent default."""
    try:
        return _EVIDENCE_BY_MOUNT[mount]
    except KeyError:
        raise ValueError(
            f"no evidence kind declared for mount {mount!r}. Adding a mount "
            f"type is a claim about whether it can act; say which it is in "
            f"core/memory/interfaces.py rather than letting it default."
        ) from None


# --- The query ---------------------------------------------------------------


@dataclass(frozen=True)
class Viewpoint:
    """
    Where a prediction is asked from, and through what.

    M5 specifies `predict(pose)`. A pose alone turns out not to be a query: a
    lidar and a camera at the same pose expect entirely different things and
    the modality is not recoverable from the position. `sensor_id` is here so
    a per-sensor implementation has something to key on -- but a real spatial
    memory must be able to answer for a viewpoint no sensor has ever occupied,
    which is what this type leaves room for and what the baseline cannot do.
    """

    sensor_id: str
    modality: Modality
    mount: MountType
    pose: Pose

    @property
    def evidence(self) -> Evidence:
        return evidence_of(self.mount)

    @classmethod
    def of(cls, obs: Observation) -> Viewpoint:
        """The viewpoint a reading was taken from. How the loop asks."""
        return cls(obs.sensor_id, obs.modality, obs.mount, obs.pose)


# --- What comes back ---------------------------------------------------------


@dataclass(frozen=True)
class Residual:
    """
    How wrong the prediction was, and about what.

    `magnitude` is normalised: 0.0 means the prediction was exact, 1.0 means
    nothing in the reading was recognisable from memory. Normalising is what
    lets a lidar residual and a camera residual be compared at all, and the
    per-key breakdown in `by_key` is kept because "which channel disagreed" is
    usually the actionable half.
    """

    sensor_id: str
    evidence: Evidence
    timestamp: float
    magnitude: float
    actual: Observation
    expected: Observation | None = None
    by_key: dict[str, float] = field(default_factory=dict)

    @property
    def novel(self) -> bool:
        """
        Nothing was predicted. Either memory has never seen this sensor, or it
        holds it at a pose too far from this one to extrapolate from.

        Novel is NOT change: there was no claim to contradict. Conflating the
        two makes a memory that "detects change" every time it looks somewhere
        new, which is the failure mode of most naive change detectors.
        """
        return self.expected is None

    def exceeds(self, threshold: float) -> bool:
        return self.magnitude > threshold


class RevisionKind(str, Enum):
    """
    What the residual did to memory. The three cases are meaningfully
    different and the demo highlights only the third.
    """

    WROTE = "wrote"            # nothing was predicted; no claim was contradicted
    REINFORCED = "reinforced"  # the prediction held; residual was model noise
    UPDATED = "updated"        # the prediction was wrong: the world changed


@dataclass(frozen=True)
class Revision:
    """The write-back half of the loop, made inspectable."""

    sensor_id: str
    evidence: Evidence
    kind: RevisionKind
    magnitude: float
    observations: int   # readings folded into this record so far

    @property
    def changed(self) -> bool:
        return self.kind is RevisionKind.UPDATED


@dataclass
class MemoryRecord:
    """
    One thing memory persists. Mutable on purpose: revision is the point.

    `observations` is the crudest possible confidence -- how many readings
    have agreed with this record. A real implementation replaces it with
    something that has a distribution behind it.
    """

    sensor_id: str
    evidence: Evidence
    observation: Observation
    observations: int = 1
    revisions: int = 0
    last_magnitude: float = 0.0


# --- The interface itself ----------------------------------------------------


@runtime_checkable
class SpatialMemory(Protocol):
    """
    Persistent memory that predicts, is contradicted, and revises.

    Three methods, one per arrow. Implement these and `MemoryLoop` drives you;
    nothing in `core/memory/loop.py` needs to change to swap the baseline for
    a real model.

    `residual` is a method rather than a free function because a memory that
    knows its own uncertainty should be allowed to weight the comparison by
    it -- a residual of 0.3 from a confident model is a different event from
    the same number out of a model that has seen one frame.
    """

    def predict(self, viewpoint: Viewpoint) -> Observation | None:
        """
        What memory expects to see from here, or None if it cannot say.

        Returning None is a legitimate and important answer. A memory that
        always produces something makes every residual meaningful-looking and
        the novel case indistinguishable from the confident-and-wrong case.
        """
        ...

    def residual(
        self, expected: Observation | None, actual: Observation
    ) -> Residual:
        """How far the reading fell from the prediction. Writes nothing."""
        ...

    def revise(self, residual: Residual) -> Revision:
        """
        Fold the residual back into memory. THE arrow that does not exist in
        the write-only systems this project is arguing with.
        """
        ...
