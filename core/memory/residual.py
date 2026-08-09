"""
How far a reading fell from a prediction. Layer 4.

One number per payload key, each normalised to [0, 1] so that a lidar cloud
and an RGB frame can be put on the same axis. The keys are the ones fixed by
`core.observation.ANNOTATOR_DATA_KEYS`; comparing payloads is only possible at
all because that vocabulary is portable rather than Isaac's.

WHAT THIS IS NOT, and it matters more than what it is:

  * A frame-wide RMS dilutes a small object across a megapixel. A crate that
    fills 7% of an image moves the depth residual by about 0.15, and a coffee
    cup would not move it measurably at all. The fix is a *spatial* residual --
    a per-region or per-voxel field, not a scalar -- and it is deliberately
    not done here, because picking that representation is research and this
    file exists so the loop can run before the research starts.

  * Point clouds of different lengths are compared by size only. A cloud that
    keeps its point count but moves every point is scored 0 by that path,
    which is wrong; the real answer is a Chamfer distance or an occupancy-grid
    IoU. Same reason: placeholder.

Both limitations bite in the direction of *under*-reporting change, which is
the safe direction for a placeholder: it makes the residual test harder to
pass, not easier.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

_EPS = 1e-9

#: Payload keys whose values are labels, not quantities. Subtracting label ids
#: is meaningless -- id 7 is not "further" from id 1 than id 2 is -- so these
#: are scored as the fraction of elements that disagree.
CATEGORICAL_PAYLOAD_KEYS: frozenset[str] = frozenset({"semantic"})


def payload_residual(
    expected: Mapping[str, Any], actual: Mapping[str, Any]
) -> dict[str, float]:
    """Per-key disagreement between two `Observation.data` payloads."""
    out: dict[str, float] = {}
    for key in sorted(set(expected) | set(actual)):
        if key not in expected or key not in actual:
            # A payload key that appears or vanishes is not a small change.
            # Every consumer indexes on these keys, and one going missing is
            # the silent-annotator failure the registry exists to catch,
            # surfacing a layer late.
            out[key] = 1.0
            continue
        out[key] = key_residual(key, expected[key], actual[key])
    return out


def magnitude(by_key: Mapping[str, float]) -> float:
    """
    Collapse the per-key residuals into one number.

    Max, not mean. A lidar frame that changed completely while its unchanged
    intensity array sat next to it should read as "changed", and averaging
    across channels is how a real signal gets buried under the channels that
    happened to stay still.
    """
    return max(by_key.values(), default=0.0)


def key_residual(key: str, expected: Any, actual: Any) -> float:
    """Disagreement for one payload key, in [0, 1]."""
    if key in CATEGORICAL_PAYLOAD_KEYS:
        return _categorical(expected, actual)
    if isinstance(expected, bool) or isinstance(actual, bool):
        return _exact(expected, actual)
    if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        return _scalar(float(expected), float(actual))

    left, right = _as_array(expected), _as_array(actual)
    if left is None or right is None:
        return _exact(expected, actual)
    return _numeric(left, right)


# --- The comparisons ---------------------------------------------------------


def _numeric(a: np.ndarray, b: np.ndarray) -> float:
    """Normalised RMS difference, or a size ratio when the shapes disagree."""
    if a.shape != b.shape:
        return _by_size(a, b)
    if a.size == 0:
        return 0.0

    # Real depth buffers carry inf where the ray hit nothing. Subtracting
    # those gives nan, and a nan residual poisons every number downstream
    # while looking like an ordinary float. Compare the pattern separately.
    finite = np.isfinite(a) & np.isfinite(b)
    disagreement = float(np.mean(np.isfinite(a) != np.isfinite(b)))
    if not finite.any():
        return disagreement

    diff = _rms(a[finite] - b[finite])
    scale = max(_rms(a[finite]), _rms(b[finite]), _EPS)
    return min(1.0, max(diff / scale, disagreement))


def _by_size(a: np.ndarray, b: np.ndarray) -> float:
    """
    Different shapes: score by how much of the payload is new or gone.

    This is the point-cloud case -- N returns became M -- and it is the only
    thing a scalar can honestly say about two clouds it cannot correspond.
    """
    biggest = max(a.size, b.size)
    if biggest == 0:
        return 0.0
    return min(1.0, abs(a.size - b.size) / biggest)


def _categorical(expected: Any, actual: Any) -> float:
    a, b = _as_array(expected), _as_array(actual)
    if a is None or b is None:
        return _exact(expected, actual)
    if a.shape != b.shape:
        return 1.0
    if a.size == 0:
        return 0.0
    return float(np.mean(a != b))


def _scalar(a: float, b: float) -> float:
    scale = max(abs(a), abs(b), _EPS)
    return min(1.0, abs(a - b) / scale)


def _exact(a: Any, b: Any) -> float:
    """Anything with no metric on it: a label mapping, a string, an enum."""
    try:
        return 0.0 if bool(a == b) else 1.0
    except (TypeError, ValueError):
        return 0.0 if a is b else 1.0


# --- Coercion ----------------------------------------------------------------


def _as_array(value: Any) -> np.ndarray | None:
    """
    A numeric array, or None for anything that is not one.

    Note the float64 cast: `rgb` arrives as uint8, and subtracting uint8 250
    from uint8 10 wraps to 16 rather than giving -240. A residual metric that
    silently reports a black frame as a near-perfect match for a white one is
    worse than no metric.
    """
    if isinstance(value, np.ndarray):
        if value.dtype.kind not in "biufc":
            return None
        return value.astype(np.float64, copy=False)
    if isinstance(value, (list, tuple)):
        try:
            array = np.asarray(value, dtype=np.float64)
        except (TypeError, ValueError):
            return None
        return array if array.dtype.kind in "biufc" else None
    return None


def _rms(x: np.ndarray) -> float:
    if x.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(np.square(x))))
