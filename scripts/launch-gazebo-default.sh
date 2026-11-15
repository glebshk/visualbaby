#!/bin/bash
# Start Gazebo with an aufdeckung world.
#   ./scripts/launch-gazebo-default.sh [world.sdf]
# Without an argument a random "mixed" arena (seed 0) is generated first.
set -e
export PX4_SOURCE_DIR=${PX4_SOURCE_DIR:-/sim/PX4-Autopilot}
export PX4_GZ_WORLDS=$PX4_SOURCE_DIR/Tools/simulation/gz/worlds
export ADDITIONAL_GZ_MODELS=$PWD/simulation
export GZ_SIM_RESOURCE_PATH=$PX4_GZ_WORLDS:$ADDITIONAL_GZ_MODELS:$PX4_SOURCE_DIR/Tools/simulation/gz/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4_SOURCE_DIR/build/px4_sitl_default/build_gzsim:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins

WORLD=${1:-}
if [ -z "$WORLD" ]; then
    python3 -m aufdeckung generate --preset mixed --seed 0 --sdf --out-dir simulation/worlds
    WORLD=simulation/worlds/mixed_0.sdf
fi
gz sim "$WORLD" -r -v 4
