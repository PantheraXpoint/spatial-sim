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
#
# This distinguishes two failure modes that look identical if you're careless:
#   - a FORBIDDEN module is missing  -> a real leak
#   - any other module is missing    -> a declared dependency isn't installed,
#                                       i.e. you're outside the dev container
# Conflating them sends people hunting for a leak that isn't there.
echo "Checking core/ imports without a simulator present..."
if PYTHONPATH=. python3 -c "
import importlib, pkgutil, sys

FORBIDDEN = {'omni', 'pxr', 'isaacsim'}
leaks, missing_deps, other = [], [], []

for m in pkgutil.walk_packages(['core'], prefix='core.'):
    try:
        importlib.import_module(m.name)
    except ModuleNotFoundError as exc:
        root = (exc.name or '').split('.')[0]
        (leaks if root in FORBIDDEN else missing_deps).append((m.name, root))
    except Exception as exc:
        other.append((m.name, f'{type(exc).__name__}: {exc}'))

if leaks:
    print('    SIMULATOR LEAK -- core/ transitively imports a simulator module:')
    for mod, root in leaks:
        print(f'      {mod} requires {root!r}')
if missing_deps:
    print('    MISSING DEPENDENCY (not a leak) -- these are declared in')
    print('    docker/requirements-dev.txt but absent from this environment.')
    print('    You are probably running outside the dev container. Use:')
    print('      make check')
    for mod, root in missing_deps:
        print(f'      {mod} requires {root!r}')
if other:
    print('    IMPORT ERROR:')
    for mod, msg in other:
        print(f'      {mod}: {msg}')

sys.exit(1 if (leaks or missing_deps or other) else 0)
" 2>&1; then
    echo "  OK -- core/ imports standalone."
else
    echo ""
    echo "FAIL: core/ did not import cleanly. See the diagnosis above."
    STATUS=1
fi

echo ""
if [ "$STATUS" -eq 0 ]; then
    echo "Layer boundary intact."
else
    echo "Layer boundary VIOLATED."
fi
exit "$STATUS"