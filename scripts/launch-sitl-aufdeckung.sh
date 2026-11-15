#!/bin/bash
# Start PX4 SITL and attach it to the already spawned "x500" model in the aufdeckung world.
MAX_RETRIES=5
RETRY_COUNT=0
WORLD=${AUFDECKUNG_WORLD_NAME:-aufdeckung}

start_px4() {
    cd ${PX4_SOURCE_DIR:-/sim/PX4-Autopilot}
    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_MODEL_NAME=x500 \
    PX4_GZ_WORLD=$WORLD \
    make px4_sitl gz_x500
}

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    start_px4 && break
    echo "PX4 start failed (attempt $((RETRY_COUNT+1))/$MAX_RETRIES), retrying in 5 s..."
    sleep 5
    RETRY_COUNT=$((RETRY_COUNT+1))
done
[ $RETRY_COUNT -lt $MAX_RETRIES ] || { echo "PX4 could not be started"; exit 1; }
