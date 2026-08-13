"""
Keeps the M4 sketch honest against the contract it is a claim about.

Nothing here tests Habitat -- Habitat is not installed and must not be. These
check that `core/adapters/habitat.py` still says something true about
`core/observation.py`, because a sketch that silently falls out of date with
the contract is worse than no sketch: it reads like evidence of portability
long after it has stopped being any.

The one that earns its place is
`test_every_modality_is_either_servable_or_has_a_written_reason`. Add a
modality to the contract and it fails until somebody says what Habitat would
do about it -- which is the question M4 exists to keep asking.
"""

from __future__ import annotations

import pytest

from core.adapters.habitat import (
    HABITAT_PAYLOAD_SOURCES,
    HABITAT_UP_AXIS,
    MOUNT_STRATEGY,
    SERVABLE_MODALITIES,
    UNSERVABLE_MODALITIES,
    HabitatObservationSource,
)
from core.observation import (
    ANNOTATOR_DATA_KEYS,
    MODALITY_DATA_KEYS,
    UP_AXIS,
    Modality,
    MountType,
    ObservationSource,
)


@pytest.fixture
def sketch() -> HabitatObservationSource:
    """Constructible on purpose: the shape is the deliverable."""
    return HabitatObservationSource(sim=None, bindings={})


# --- The claim: the contract is a shape another simulator could satisfy -------


def test_the_sketch_has_the_shape_of_an_observation_source(sketch):
    """
    The whole M4 question, as one assertion. If the protocol had an
    Isaac-specific method on it, this file could not have been written.
    """
    assert isinstance(sketch, ObservationSource)


def test_nothing_that_would_need_habitat_pretends_to_work(sketch):
    """
    A stub that returns something plausible is how a sketch becomes a fake
    that a suite passes against. Everything that would have to talk to a
    simulator raises, and says which task it belongs to.

    `sensor_ids` and `time` are answerable from the constructor alone, so
    they are real -- and they have to be, or the isinstance check above dies
    inside hasattr instead of returning False.
    """
    with pytest.raises(NotImplementedError, match="sketch"):
        sketch.step(0.1)
    with pytest.raises(NotImplementedError, match="sketch"):
        sketch.close()
    assert sketch.sensor_ids == ()
    assert sketch.time == 0.0


def test_the_sketch_does_not_reach_for_a_simulator_that_is_not_installed():
    """
    Rule 2's neighbour: `scripts/check_layer_boundary.sh` imports everything
    under core/ on a machine with neither simulator and fails on any missing
    module, not only omni/pxr/isaacsim. A module-scope `import habitat_sim`
    would break `make check` everywhere, including CI.
    """
    import core.adapters.habitat as sketch_module

    source = sketch_module.__doc__ or ""
    assert "habitat_sim" in source, "the docstring should name what it avoids"
    assert not hasattr(sketch_module, "habitat_sim")


# --- The mapping tables still describe the contract ---------------------------


def test_every_payload_key_habitat_would_fill_is_one_the_contract_defines():
    """The adapter's job description cannot invent keys no consumer reads."""
    contract_keys = set(ANNOTATOR_DATA_KEYS.values())
    unknown = set(HABITAT_PAYLOAD_SOURCES) - contract_keys
    assert not unknown, (
        f"the sketch promises to fill {sorted(unknown)}, which no annotator "
        f"in core/observation.py maps to. Known: {sorted(contract_keys)}"
    )


def test_every_servable_modality_is_fully_covered_by_those_keys():
    """
    Claiming a modality means claiming every payload key it obliges. Half a
    modality is the silent-annotator failure with extra steps.
    """
    for modality in sorted(SERVABLE_MODALITIES, key=lambda m: m.value):
        missing = MODALITY_DATA_KEYS[modality] - set(HABITAT_PAYLOAD_SOURCES)
        assert not missing, (
            f"the sketch claims Habitat can serve {modality.value} but says "
            f"nothing about where {sorted(missing)} would come from"
        )


def test_every_modality_is_either_servable_or_has_a_written_reason():
    """
    THE test in this file. Adding a modality to the contract has to force a
    decision about Habitat at the moment it is added, not years later when
    somebody tries the comparison and finds out.
    """
    accounted = SERVABLE_MODALITIES | set(UNSERVABLE_MODALITIES)
    unaccounted = set(Modality) - accounted
    assert not unaccounted, (
        f"{sorted(m.value for m in unaccounted)} appeared in the contract "
        f"and core/adapters/habitat.py does not say whether Habitat could "
        f"serve it. Add it to SERVABLE_MODALITIES or give it a reason in "
        f"UNSERVABLE_MODALITIES."
    )
    assert not (SERVABLE_MODALITIES & set(UNSERVABLE_MODALITIES))
    for modality, reason in UNSERVABLE_MODALITIES.items():
        assert len(reason) > 40, f"{modality.value}: give an actual reason"


def test_every_mount_type_has_a_realisation():
    """
    Habitat hangs every sensor off an agent, so each of our mount types needs
    a story. If one appears without one, the state/experience split silently
    stops being expressible over there.
    """
    assert set(MOUNT_STRATEGY) == set(MountType)


def test_the_two_up_axes_actually_differ():
    """
    Guards the sketch's whole reason for discussing swizzles. If UP_AXIS ever
    changed to match Habitat's, the conversion notes would become wrong rather
    than unnecessary.
    """
    assert HABITAT_UP_AXIS != UP_AXIS
