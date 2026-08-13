"""
Adapters from a simulator's native observation format to `core.observation`.

One module per simulator, and none of them may import their simulator at
module scope -- see the note in `habitat.py`. The Isaac adapter deliberately
does NOT live here: it is `sim/observation_adapter.py`, server-side, because
it imports isaacsim and rule 2 keeps that out of `core/`.

That asymmetry is the whole architecture in one sentence. An adapter for a
simulator that runs on a laptop can live in `core/`; one for a simulator that
needs an RTX GPU cannot. The contract they meet at is the same either way.
"""
