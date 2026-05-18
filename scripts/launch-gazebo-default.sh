export PX4_SOURCE_DIR=/sim/PX4-Autopilot
export PX4_GZ_WORLDS=$PX4_SOURCE_DIR/Tools/simulation/gz/worlds
export ADDITIONAL_GZ_MODELS=$PWD/simulation
export GZ_SIM_RESOURCE_PATH=$PX4_GZ_WORLDS:$ADDITIONAL_GZ_MODELS
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4_SOURCE_DIR/build/px4_sitl_default/build_gzsim:/usr/lib/x86_64-linux-gnu/gz-sim-7/plugins

gz sim $PWD/simulation/imav_2026.sdf -r -v 4
