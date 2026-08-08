#!/usr/bin/env bash
# =============================================================================
# Enforces the single most important architectural decision in the project:
# core/ must never import simulator APIs.
#
# Why this is worth a CI job: the day core/ imports `omni`, the same memory
# module can no longer run on Habitat for benchmark comparison, and the
# MacBook stops being a development machine. Both losses are silent and both
# are expensive to reverse. This script makes them loud and immediate.
#
# Run: ./scripts/check_layer_boundary.sh
# =============================================================================
set -uo pipefail

FORBIDDEN='^[[:space:]]*(import|from)[[:space:]]+(omni|pxr|isaacsim)\b'
STATUS=0

if [ ! -d core ]; then
    echo "FAIL: core/ not found. Run from the repository root."
    exit 1
fi

echo "Checking core/ for simulator imports..."
if HITS=$(grep -rnE "$FORBIDDEN" core/ --include='*.py' 2>/dev/null); then
    echo ""
    echo "FAIL: simulator imports found in core/"
    echo "$HITS" | sed 's/^/    /'
    echo ""
    echo "  core/ is Layers 3-4: it consumes observation dicts and nothing"
    echo "  else. Simulator-specific code belongs in sim/."
    STATUS=1
else
    echo "  OK -- core/ is clean."
fi

# A second, sharper check: core/ must actually IMPORT on a machine with no
# simulator installed. A dependency can leak in transitively without the
# grep above ever firing.
echo "Checking core/ imports without a simulator present..."
if PYTHONPATH=. python3 -c "
import importlib, pkgutil, sys
failed = []
for m in pkgutil.walk_packages(['core'], prefix='core.'):
    try:
        importlib.import_module(m.name)
    except Exception as exc:
        failed.append((m.name, type(exc).__name__, str(exc)))
if failed:
    for name, kind, msg in failed:
        print(f'    {name}: {kind}: {msg}')
    sys.exit(1)
" 2>&1; then
    echo "  OK -- core/ imports standalone."
else
    echo ""
    echo "FAIL: core/ could not be imported without the simulator."
    STATUS=1
fi

echo ""
if [ "$STATUS" -eq 0 ]; then
    echo "Layer boundary intact."
else
    echo "Layer boundary VIOLATED."
fi
exit "$STATUS"
