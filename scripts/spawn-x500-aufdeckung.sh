#!/bin/bash
# Spawn the aufdeckung drone at the scenario start pad (cage ENU, metres).
#   ./scripts/spawn-x500-aufdeckung.sh [x] [y]
X=${1:-0.0}
Y=${2:--18.0}
WORLD=${AUFDECKUNG_WORLD_NAME:-aufdeckung}
gz service -s /world/$WORLD/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 3000 --req "sdf_filename: \"simulation/x500_aufdeckung/model.sdf\", name: \"x500\", pose: {position: {x: $X, y: $Y, z: 0.25}}"
