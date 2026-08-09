"""
Layer 4. The research code goes here -- this is the skeleton it hangs on.

    persistent memory ──> prediction ──> observation ──> residual ──┐
            ^                                                       │
            └───────────────────── revision ────────────────────────┘

Read `interfaces.py` first: it explains why the revision arrow is the point
and why the residual means "model noise" and "change detected" with the same
number. `baseline.py` is a placeholder implementation that exists so the loop
can be run and tested before any of the interesting parts are written.

    from core.memory import LastObservationMemory, MemoryLoop

    loop = MemoryLoop(LastObservationMemory())
    loop.run(source, ticks=20)          # memory learns the room
    residuals = [loop.observe(o) for o in source.step()]   # then watches it

Nothing in this package imports omni, pxr, or isaacsim, and nothing in it
imports the simulator's registry either -- only `core.observation`. It runs on
a laptop, and it will run against Habitat.
"""

from core.memory.baseline import (
    DEFAULT_CHANGE_THRESHOLD,
    LastObservationMemory,
)
from core.memory.interfaces import (
    Evidence,
    MemoryRecord,
    Residual,
    Revision,
    RevisionKind,
    SpatialMemory,
    Viewpoint,
    evidence_of,
)
from core.memory.loop import MemoryLoop, Turn, changes
from core.memory.residual import magnitude, payload_residual

__all__ = [
    "DEFAULT_CHANGE_THRESHOLD",
    "Evidence",
    "LastObservationMemory",
    "MemoryLoop",
    "MemoryRecord",
    "Residual",
    "Revision",
    "RevisionKind",
    "SpatialMemory",
    "Turn",
    "Viewpoint",
    "changes",
    "evidence_of",
    "magnitude",
    "payload_residual",
]
