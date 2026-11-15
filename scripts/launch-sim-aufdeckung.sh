#!/bin/bash
# Full SITL run: random arena -> Gazebo -> drone -> PX4 -> ROS bridges -> mission node (tmux).
#   ./scripts/launch-sim-aufdeckung.sh [preset] [seed]
# Presets: sparse urban_ring dense_urban center_tower street mixed
set -e
export AUFDECKUNG_PRESET=${1:-mixed}
export AUFDECKUNG_SEED=${2:-0}
export AUFDECKUNG_REPO=$PWD
export AUFDECKUNG_WORLD_NAME=aufdeckung
python3 -m aufdeckung generate --preset "$AUFDECKUNG_PRESET" --seed "$AUFDECKUNG_SEED" --sdf --plot \
    --world-name "$AUFDECKUNG_WORLD_NAME" --out-dir simulation/worlds
export AUFDECKUNG_SCENARIO=$PWD/simulation/worlds/${AUFDECKUNG_PRESET}_${AUFDECKUNG_SEED}.yaml
export AUFDECKUNG_WORLD=$PWD/simulation/worlds/${AUFDECKUNG_PRESET}_${AUFDECKUNG_SEED}.sdf
read -r START_X START_Y < <(python3 -c "import yaml,sys; s=yaml.safe_load(open('$AUFDECKUNG_SCENARIO'))['start']; print(s[0], s[1])")
export START_X START_Y
tmuxinator start aufdeckung-sim -p tmux/aufdeckung-sim.yml
