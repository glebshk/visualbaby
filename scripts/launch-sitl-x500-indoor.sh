#!/bin/bash

MAX_RETRIES=5
RETRY_COUNT=0

start_px4() {
    echo "Starte PX4..."
    PX4_SYS_AUTOSTART=4001 \
        PX4_GZ_MODEL_NAME=x500 \
        PX4_GZ_WORLD=imav2026 \
        /sim/PX4-Autopilot/build/px4_sitl_default/bin/px4

}

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    start_px4
    STATUS=$?
    if [ $STATUS -eq 0 ]; then
        echo "PX4 erfolgreich gestartet!"
        break
    else
        echo "PX4 Start fehlgeschlagen (Versuch $((RETRY_COUNT+1))/$MAX_RETRIES). Neustart in 5 Sekunden..."
        sleep 5
        RETRY_COUNT=$((RETRY_COUNT+1))
    fi
done

if [ $RETRY_COUNT -eq $MAX_RETRIES ]; then
    echo "PX4 konnte nach $MAX_RETRIES Versuchen nicht gestartet werden. Beende Script."
    exit 1
fi
