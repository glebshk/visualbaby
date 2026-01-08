PX4_SOURCE_DIR=/sim/PX4-Autopilot \
    #PX4_GZ_MODELS=$PX4_SOURCE_DIR/Tools/simulation/gz/models \
    PX4_GZ_WORLDS=$PX4_SOURCE_DIR/Tools/simulation/gz/worlds \
    ADDITIONAL_GZ_MODELS=$PWD/simulation \
    PX4_SIM_MODEL=iris_opticalflow \
    GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_GZ_WORLDS:$ADDITIONAL_GZ_MODELS \
    gz sim -r $PWD/simulation/world_indoor.sdf
    