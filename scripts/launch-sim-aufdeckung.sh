#!/bin/bash
# Full SITL run: random arena -> Gazebo -> drone -> PX4 -> ROS bridges -> mission node (tmux).
#   ./scripts/launch-sim-aufdeckung.sh [preset] [seed] [mode]
# mode: explore (default, map from the lidar), prior (layout known, lidar keeps mapping),
#       plan-only (layout known, lidar mapping off: tests the mission itself)
# Presets: sparse urban_ring dense_urban center_tower street mixed
set -e
# An old Gazebo or PX4 still running publishes on the same topics: PX4 then reports sensor
# timestamp errors and TIMEOUTs. Start from a clean slate.
"$(dirname "$0")/stop-sim.sh" >/dev/null
export AUFDECKUNG_PRESET=${1:-mixed}
export AUFDECKUNG_SEED=${2:-0}
AUFDECKUNG_MODE=${3:-explore}
export AUFDECKUNG_REPO=$PWD
export AUFDECKUNG_WORLD_NAME=aufdeckung
python3 -m aufdeckung generate --preset "$AUFDECKUNG_PRESET" --seed "$AUFDECKUNG_SEED" --sdf --plot \
    --world-name "$AUFDECKUNG_WORLD_NAME" --out-dir simulation/worlds
export AUFDECKUNG_SCENARIO=$PWD/simulation/worlds/${AUFDECKUNG_PRESET}_${AUFDECKUNG_SEED}.yaml
export AUFDECKUNG_WORLD=$PWD/simulation/worlds/${AUFDECKUNG_PRESET}_${AUFDECKUNG_SEED}.sdf
case "$AUFDECKUNG_MODE" in
    explore) export AUFDECKUNG_LAUNCH_ARGS="" ;;
    prior) export AUFDECKUNG_LAUNCH_ARGS="prior_scenario:=$AUFDECKUNG_SCENARIO" ;;
    plan-only) export AUFDECKUNG_LAUNCH_ARGS="prior_scenario:=$AUFDECKUNG_SCENARIO use_lidar:=false" ;;
    *) echo "unknown mode '$AUFDECKUNG_MODE' (explore, prior, plan-only)"; exit 2 ;;
esac
echo "mode:     $AUFDECKUNG_MODE"
read -r START_X START_Y < <(python3 -c "import yaml,sys; s=yaml.safe_load(open('$AUFDECKUNG_SCENARIO'))['start']; print(s[0], s[1])")
export START_X START_Y
tmuxinator start aufdeckung-sim -p tmux/aufdeckung-sim.yml
