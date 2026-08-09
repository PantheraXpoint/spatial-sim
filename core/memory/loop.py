"""
The driver. Layer 4.

Three lines of substance, and that is the point: every decision -- what to
predict, how to compare, when to rewrite -- belongs to the `SpatialMemory`
being driven. Swapping the baseline for a research model must not touch this
file, and if it does, policy has leaked out of the model.

The loop takes an `ObservationSource` and nothing else, so it runs against
`core.mock_source.MockObservationSource` on a laptop today and against
`sim/observation_adapter.py` (server task S11) unchanged. That is the whole
purpose of the contract in `tests/contract.py`.
"""

from __future__ import annotations

from dataclasses import dataclass

from core.memory.interfaces import (
    Residual,
    Revision,
    SpatialMemory,
    Viewpoint,
)
from core.observation import Observation, ObservationSource


@dataclass(frozen=True)
class Turn:
    """One full trip around the loop, for one reading."""

    residual: Residual
    revision: Revision

    @property
    def sensor_id(self) -> str:
        return self.residual.sensor_id

    @property
    def changed(self) -> bool:
        """Memory predicted, memory was wrong, memory was rewritten."""
        return self.revision.changed


class MemoryLoop:
    """
    persistent memory -> prediction -> observation -> residual -> revision

    Two ways to use it:

      `ingest` / `tick` / `run`  close the loop -- memory learns as it goes.
      `observe`                  predicts and compares but writes nothing.

    The second is not a convenience. Freezing memory after it has learned the
    empty warehouse and then watching the residual light up around the one
    moving body is the demo, and it needs a memory that does *not* quietly
    absorb the person into its model of the room.
    """

    def __init__(self, memory: SpatialMemory) -> None:
        self._memory = memory

    @property
    def memory(self) -> SpatialMemory:
        return self._memory

    def observe(self, obs: Observation) -> Residual:
        """Predict, compare. Read-only: memory is not touched."""
        expected = self._memory.predict(Viewpoint.of(obs))
        return self._memory.residual(expected, obs)

    def ingest(self, obs: Observation) -> Turn:
        """Predict, compare, revise. The closed loop, for one reading."""
        residual = self.observe(obs)
        return Turn(residual, self._memory.revise(residual))

    def tick(self, source: ObservationSource, dt: float | None = None) -> list[Turn]:
        """Advance the source one step and fold the whole frame in."""
        return [self.ingest(obs) for obs in source.step(dt)]

    def run(
        self, source: ObservationSource, ticks: int, dt: float | None = None
    ) -> list[list[Turn]]:
        """`ticks` frames, one inner list each."""
        return [self.tick(source, dt) for _ in range(ticks)]


def changes(turns: list[Turn]) -> list[Turn]:
    """
    The turns where memory was contradicted. What a change detector reports,
    and -- deliberately -- not the same set as "the residual was nonzero".
    """
    return [turn for turn in turns if turn.changed]
