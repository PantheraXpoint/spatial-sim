"""
The dumbest memory that closes the loop. Layer 4.

Remember the last reading from each viewpoint; predict it will happen again.
That is the entire model.

WHY BOTHER. Because every arrow in `core/memory/interfaces.py` then becomes
executable, and the two claims the project rests on become things a test suite
can check rather than things a paragraph can assert:

    a static world reinforces memory and produces residual == noise
    a changed world contradicts memory and produces residual == the change

Both are already true of this class, which is worth noticing: the interesting
part of the research is not detecting that *something* changed, it is having a
memory whose predictions are worth contradicting.

WHAT IT IS NOT: a map. It stores readings, not structure. It cannot predict
for a viewpoint no sensor has occupied, cannot compose two views of the same
shelf, cannot say anything about the half of the warehouse nobody is looking
at, and forgets the old world completely the moment it is contradicted. Those
are the actual problems. This class solves none of them and is written so that
replacing it touches no other file.

NOTHING IN THIS MODULE MAY IMPORT omni, pxr, OR isaacsim.
"""

from __future__ import annotations

import math

from core.memory.interfaces import (
    Evidence,
    MemoryRecord,
    Residual,
    Revision,
    RevisionKind,
    Viewpoint,
    evidence_of,
)
from core.memory.residual import magnitude, payload_residual
from core.observation import Observation, Pose

#: A prediction is only offered from within this much of where the reading was
#: taken. A fixed sensor in a live simulator jitters in the last float digit;
#: an avatar two centimetres down the corridor is somewhere else.
DEFAULT_POSE_TOLERANCE_M = 0.01
DEFAULT_POSE_TOLERANCE_RAD = 0.01

#: Above this, the residual is treated as the world having changed rather than
#: the model being noisy. One global number is obviously too crude -- the right
#: threshold is per-modality and probably learned -- but it has to be *some*
#: number before the distinction can be tested at all.
DEFAULT_CHANGE_THRESHOLD = 0.05


class LastObservationMemory:
    """
    Satisfies `core.memory.interfaces.SpatialMemory`.

    State and experience live in two separate stores. That is structural, not
    decorative: with one store keyed by sensor id, the only thing stopping a
    consumer from averaging a ceiling camera together with a first-person view
    is that it remembered not to.
    """

    def __init__(
        self,
        *,
        change_threshold: float = DEFAULT_CHANGE_THRESHOLD,
        pose_tolerance_m: float = DEFAULT_POSE_TOLERANCE_M,
        pose_tolerance_rad: float = DEFAULT_POSE_TOLERANCE_RAD,
    ) -> None:
        self.change_threshold = float(change_threshold)
        self.pose_tolerance_m = float(pose_tolerance_m)
        self.pose_tolerance_rad = float(pose_tolerance_rad)
        self._stores: dict[Evidence, dict[str, MemoryRecord]] = {
            Evidence.STATE: {},
            Evidence.EXPERIENCE: {},
        }

    # --- SpatialMemory -------------------------------------------------------

    def predict(self, viewpoint: Viewpoint) -> Observation | None:
        record = self._stores[viewpoint.evidence].get(viewpoint.sensor_id)
        if record is None:
            return None
        remembered = record.observation
        if remembered.modality is not viewpoint.modality:
            raise ValueError(
                f"'{viewpoint.sensor_id}' was remembered as "
                f"{remembered.modality.value} and is now asking as "
                f"{viewpoint.modality.value}. A sensor does not change "
                f"modality; two sensors are sharing an id."
            )
        if not self._pose_matches(remembered.pose, viewpoint.pose):
            # Memory holds one viewpoint per sensor and cannot extrapolate to
            # another, so the honest answer is "I don't know".
            #
            # It is also the interesting answer: this branch is taken on every
            # single tick for an avatar-mounted sensor, because embodied
            # experience is never twice from the same place. A memory that can
            # only predict where it has already stood is not a spatial memory.
            # Closing that gap IS the work; the gap is visible here.
            return None
        return remembered

    def residual(
        self, expected: Observation | None, actual: Observation
    ) -> Residual:
        evidence = evidence_of(actual.mount)
        if expected is None:
            return Residual(
                sensor_id=actual.sensor_id,
                evidence=evidence,
                timestamp=actual.timestamp,
                magnitude=1.0,
                actual=actual,
                expected=None,
            )
        if expected.sensor_id != actual.sensor_id:
            raise ValueError(
                f"comparing a prediction for '{expected.sensor_id}' against a "
                f"reading from '{actual.sensor_id}'"
            )
        by_key = payload_residual(expected.data, actual.data)
        return Residual(
            sensor_id=actual.sensor_id,
            evidence=evidence,
            timestamp=actual.timestamp,
            magnitude=magnitude(by_key),
            actual=actual,
            expected=expected,
            by_key=by_key,
        )

    def revise(self, residual: Residual) -> Revision:
        sensor_id = residual.sensor_id
        self._check_evidence_is_stable(sensor_id, residual.evidence)
        store = self._stores[residual.evidence]
        record = store.get(sensor_id)

        if record is None or residual.novel:
            # No prediction was made, so nothing was contradicted. Writing
            # this down is bookkeeping, not change detection -- see
            # `Residual.novel`.
            store[sensor_id] = MemoryRecord(
                sensor_id=sensor_id,
                evidence=residual.evidence,
                observation=residual.actual,
                observations=1,
                revisions=record.revisions if record else 0,
                last_magnitude=residual.magnitude,
            )
            return Revision(
                sensor_id, residual.evidence, RevisionKind.WROTE,
                residual.magnitude, 1,
            )

        record.last_magnitude = residual.magnitude
        if residual.exceeds(self.change_threshold):
            record.observation = residual.actual
            record.observations = 1
            record.revisions += 1
            kind = RevisionKind.UPDATED
        else:
            # The prediction held. Keep the reading already stored rather than
            # the new one: under noise, a memory that overwrites itself every
            # tick drifts, and "nothing changed" stops being checkable against
            # anything older than one frame. Averaging would be better and is
            # exactly the kind of clever this file is not.
            record.observations += 1
            kind = RevisionKind.REINFORCED
        return Revision(
            sensor_id, residual.evidence, kind,
            residual.magnitude, record.observations,
        )

    # --- Inspecting it -------------------------------------------------------

    def records(self, evidence: Evidence) -> dict[str, MemoryRecord]:
        """
        Everything remembered of one kind. A copy -- revision goes through
        `revise`, so that every write to memory is on the residual path.
        """
        return dict(self._stores[evidence])

    def forget(self) -> None:
        """Empty. The next reading from every viewpoint will be novel again."""
        for store in self._stores.values():
            store.clear()

    def __len__(self) -> int:
        return sum(len(store) for store in self._stores.values())

    # --- Internals -----------------------------------------------------------

    def _check_evidence_is_stable(
        self, sensor_id: str, evidence: Evidence
    ) -> None:
        other = (
            Evidence.EXPERIENCE if evidence is Evidence.STATE else Evidence.STATE
        )
        if sensor_id in self._stores[other]:
            raise ValueError(
                f"'{sensor_id}' produced {other.value} and is now producing "
                f"{evidence.value}. A sensor cannot change what kind of "
                f"evidence it is: a fixed camera that started moving is a "
                f"parenting bug in the scene, not a memory event."
            )

    def _pose_matches(self, remembered: Pose, asked: Pose) -> bool:
        moved = math.dist(remembered.position, asked.position)
        if moved > self.pose_tolerance_m:
            return False
        dot = sum(a * b for a, b in zip(remembered.orientation, asked.orientation))
        # q and -q are the same rotation, hence the abs.
        angle = 2.0 * math.acos(min(1.0, abs(dot)))
        return angle <= self.pose_tolerance_rad
