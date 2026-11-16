#!/bin/bash
# Spawn the aufdeckung drone at the scenario start pad (cage ENU, metres).
#   ./scripts/spawn-x500-aufdeckung.sh [x] [y]
# Waits until the Gazebo world accepts new entities (a first start can take a minute,
# e.g. under WSL while shaders compile), retries, and exits non-zero if it never works,
# so PX4 is not started against a world without a drone.
X=${1:-0.0}
Y=${2:--18.0}
WORLD=${AUFDECKUNG_WORLD_NAME:-aufdeckung}
SDF="$(cd "$(dirname "$0")/.." && pwd)/simulation/x500_aufdeckung/model.sdf"
TRIES=${SPAWN_TRIES:-90}

has_drone() { gz model --list 2>/dev/null | grep -qw x500; }

for i in $(seq 1 "$TRIES"); do
    if has_drone; then
        echo "x500 is in world '$WORLD'"
        exit 0
    fi
    if gz service -l 2>/dev/null | grep -qx "/world/$WORLD/create"; then
        OUT=$(gz service -s "/world/$WORLD/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
            --timeout 5000 --req "sdf_filename: \"$SDF\", name: \"x500\", pose: {position: {x: $X, y: $Y, z: 0.25}}" 2>&1)
        if echo "$OUT" | grep -q "data: true"; then
            echo "x500 spawned at ($X, $Y) in world '$WORLD'"
            exit 0
        fi
        echo "spawn attempt $i/$TRIES failed: ${OUT:-no reply}"
    else
        echo "waiting for Gazebo world '$WORLD' ($i/$TRIES)..."
    fi
    sleep 2
done
echo "ERROR: could not spawn x500 into world '$WORLD'. Is Gazebo running with that world?"
exit 1
