"""The shared contract, pointed at the live simulator. Server task S11.

``tests/contract.py`` is the handoff: it imports neither the mock nor the
simulator, and ``core.mock_source.MockObservationSource`` already passes it on
a laptop with no GPU. This file is the other half -- the same suite, the same
assertions, run against ``sim/observation_adapter.IsaacObservationSource``.

Nothing here may reach for an adapter-only attribute. The moment it does, this
stops being a contract test and becomes a second set of adapter tests, and the
one question worth asking -- *do the two sources actually agree?* -- goes
unanswered.

This cannot run in the dev container: it needs a stage, a renderer and Kit's
python. It runs inside the simulator, on a worker thread, driven by
``sim/observation_adapter.py``'s exec-mode entry point::

    docker compose -f docker/docker-compose.yml run --rm sim \\
        ./runheadless.sh --exec /workspace/sim/observation_adapter.py

Run it any other way and ``live_source()`` raises, saying so.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
for _p in (str(REPO), str(REPO / "sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Importing the adapter must not start a second run -- see
# observation_adapter._exec_entrypoint. The driver has already set this; it is
# repeated because an import that opened a stage and post_quit() the session
# would be a miserable thing to debug from a pytest traceback.
os.environ["OA_NO_AUTORUN"] = "1"

import observation_adapter as oa  # noqa: E402  -- sibling module, see sys.path

from core.observation import ObservationSource  # noqa: E402
from core.registry import SensorRegistry  # noqa: E402
from tests.contract import ObservationSourceContract  # noqa: E402

SENSORS = REPO / "config" / "sensors.yaml"
SCENE = REPO / "config" / "scene.yaml"


class TestIsaacSource(ObservationSourceContract):
    """The contract, run against the live simulator. See tests/contract.py.

    ``STEPS``/``STEP_DT`` are the contract's own knobs for how much simulated
    time the motion tests get, and they are retuned rather than defaulted:
    16 x 0.6 s at 1.4 m/s is 13.4 m of walking -- the same distance the
    defaults buy -- in 16 ticks instead of 40. Each tick here costs real
    rendered frames and holds a full set of camera buffers in memory for the
    whole trace, and 40 of those is roughly a gigabyte of RGB to prove
    something 16 already prove.

    The two path fixtures are overridden because the contract addresses its
    config files relative to the working directory, and Kit's is /isaac-sim.
    Same files, named absolutely; the suite itself is untouched.
    """

    STEPS = 16
    STEP_DT = 0.6

    def make_source(self) -> ObservationSource:
        return oa.live_source()

    @pytest.fixture
    def registry(self) -> SensorRegistry:
        return SensorRegistry.from_yaml(SENSORS, SCENE)

    @pytest.fixture
    def avatar_eye_height(self) -> float:
        return float(yaml.safe_load(SCENE.read_text())["avatar"]["eye_height"])
