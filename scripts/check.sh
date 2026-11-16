#!/bin/bash
# The development loop's verification step. CI runs exactly this script, so a green run here
# means a green run on GitHub.
#
#   ./scripts/check.sh            lint + types + fast tests            (~2 min)
#   ./scripts/check.sh --quick    lint + types only                    (seconds)
#   ./scripts/check.sh --full     + slow tests (whole missions)        (~10 min)
#   ./scripts/check.sh --bench N  + benchmark, N seeds per preset, explore and prior mode
#   ./scripts/check.sh --fix      let ruff fix what it can first
#
# Exit code is non-zero as soon as one step fails.
set -euo pipefail
cd "$(dirname "$0")/../aufdeckung"
PY=${PYTHON:-python3}
QUICK=0 FULL=0 FIX=0 BENCH=0
while [ $# -gt 0 ]; do
    case "$1" in
        --quick) QUICK=1 ;;
        --full) FULL=1 ;;
        --fix) FIX=1 ;;
        --bench) BENCH=${2:?--bench needs a number of seeds}; shift ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "unknown option $1"; exit 2 ;;
    esac
    shift
done

step() { printf '\n\033[1m== %s\033[0m\n' "$1"; }

step "lint (ruff)"
[ $FIX -eq 1 ] && $PY -m ruff check --fix src tests ../src/aufdeckung_ros
$PY -m ruff check src tests ../src/aufdeckung_ros

step "types (mypy)"
$PY -m mypy src

if [ $QUICK -eq 0 ]; then
    step "tests"
    $PY -m pytest -q -m "not slow"
    $PY -m pytest -q ../src/aufdeckung_ros/test
fi
if [ $FULL -eq 1 ]; then
    step "slow tests (whole missions)"
    $PY -m pytest -q -m slow
fi
if [ "$BENCH" -gt 0 ]; then
    mkdir -p out
    step "benchmark: explore mode, $BENCH seeds per preset"
    $PY -m aufdeckung benchmark --presets all --seeds "$BENCH" --json out/bench_explore.json
    step "benchmark: prior mode, $BENCH seeds per preset"
    $PY -m aufdeckung benchmark --presets all --seeds "$BENCH" --prior --json out/bench_prior.json
fi
printf '\n\033[1;32mall checks passed\033[0m\n'
