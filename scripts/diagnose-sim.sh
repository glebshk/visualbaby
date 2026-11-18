#!/bin/bash
# Checks every link of the running simulation, one after the other, and prints where data stops:
#   Gazebo (drone, sensors) <-> PX4 <-> px4_to_ros2 (MAVSDK) <-> ROS <-> aufdeckung mission node
# Run it in a second terminal inside the container while ./scripts/launch-sim-aufdeckung.sh runs.
# Paste the whole output when asking for help.
cd "$(dirname "$0")/.." || exit 1
[ -f install/setup.bash ] && source install/setup.bash >/dev/null 2>&1
WORLD=${AUFDECKUNG_WORLD_NAME:-aufdeckung}
PX4_BIN=${PX4_SOURCE_DIR:-/sim/PX4-Autopilot}/build/px4_sitl_default/bin
FIRST_BAD=""
ok()   { printf "  [ OK ] %s\n" "$1"; }
bad()  { printf "  [FAIL] %s\n" "$1"; [ -z "$FIRST_BAD" ] && FIRST_BAD="$1"; }
info() { printf "  [info] %s\n" "$1"; }

# Prints the first message on a ROS topic (or nothing within the timeout).
once() { timeout "${2:-5}" ros2 topic echo --once "$1" 2>/dev/null | grep -v "^WARNING" | head -"${3:-20}"; }
field() { grep -m1 -E "^ *$1:" | sed "s/^ *$1: *//"; }

echo "== 1. Gazebo"
N_GZ=$(pgrep -fc "gz sim" 2>/dev/null); N_GZ=${N_GZ:-0}
# One run shows up as up to two processes (server + GUI); more hints at a leftover run.
if [ "$N_GZ" -gt 2 ]; then bad "$N_GZ Gazebo processes: probably an old run is still alive (./scripts/stop-sim.sh, relaunch)"
elif [ "$N_GZ" -gt 0 ]; then ok "gz sim is running ($N_GZ processes)"; else bad "Gazebo is not running"; fi
if gz model --list 2>/dev/null | grep -qw x500; then ok "drone x500 is in the world"
else bad "no x500 in world '$WORLD' (spawn failed: scroll up in the PX4 pane)"; fi
STATS=$(timeout 5 gz topic -e -t "/world/$WORLD/stats" -n 1 2>/dev/null)
RTF=$(echo "$STATS" | field real_time_factor)
PAUSED=$(echo "$STATS" | field paused)
if [ -z "$STATS" ]; then bad "no world statistics from Gazebo"
elif [ "$PAUSED" = true ]; then bad "simulation is PAUSED: press the play button (bottom left in Gazebo)"
else ok "simulation running, real-time factor ${RTF:-?}"; fi
if timeout 5 gz topic -e -t /lidar/points -n 1 >/dev/null 2>&1; then ok "lidar publishes in Gazebo"
else bad "no data on Gazebo topic /lidar/points"; fi

echo "== 2. PX4"
N_PX4=$(pgrep -xc px4 2>/dev/null); N_PX4=${N_PX4:-0}
if [ "$N_PX4" -gt 1 ]; then bad "$N_PX4 PX4 instances running: an old run is still alive (./scripts/stop-sim.sh, then relaunch)"
elif [ "$N_PX4" -eq 1 ]; then ok "PX4 SITL is running"; else bad "PX4 is not running (PX4 pane)"; fi
if [ -x "$PX4_BIN/px4-listener" ]; then
    VS=$("$PX4_BIN/px4-listener" vehicle_status -n 1 2>/dev/null)
    ARM=$(echo "$VS" | field arming_state)
    NAV=$(echo "$VS" | field nav_state)
    info "PX4 vehicle_status: arming_state=${ARM:-?} (1 = disarmed, 2 = armed), nav_state=${NAV:-?} (14 = offboard)"
    echo "  -- PX4 preflight checks (commander check):"
    "$PX4_BIN/px4-commander" check 2>&1 | sed 's/^/     /' | head -25
else
    info "PX4 client tools not found in $PX4_BIN (run checks in the pxh> pane: commander check)"
fi

echo "== 3. ROS nodes"
NODES=$(timeout 10 ros2 node list 2>/dev/null)
for n in px4_ros2 aufdeckung_mission; do
    echo "$NODES" | grep -q "/$n$" && ok "node /$n is running" || bad "node /$n is not running (ros2 window, top pane)"
done
echo "$NODES" | grep -qE "ros_gz_bridge|parameter_bridge" && ok "gz-ROS bridge is running" \
    || bad "gz-ROS bridge (ros_gz_bridge) is not running"

echo "== 4. PX4 -> ROS (px4_to_ros2 telemetry)"
POS=$(once /drone/out/pos/ned)
if [ -n "$POS" ]; then ok "position arrives on /drone/out/pos/ned (z = $(echo "$POS" | field z))"
else bad "no position on /drone/out/pos/ned: px4_to_ros2 is not connected to PX4"; fi
HEALTH=$(once /drone/out/health)
if [ -n "$HEALTH" ]; then
    MISSING=$(echo "$HEALTH" | grep -E ": false" | sed 's/:.*//' | tr -d ' ' | tr '\n' ' ')
    [ -z "$MISSING" ] && ok "PX4 health all true" || bad "PX4 health not ready: ${MISSING}"
else bad "no health report on /drone/out/health"; fi
info "armed: $(once /drone/out/armed | field data)   flight mode: $(once /drone/out/flightmode | field data)"
if [ -n "$(once /lidar/points 5 3)" ]; then ok "lidar arrives in ROS (/lidar/points)"
else bad "no lidar in ROS: gz bridge config (simulation/gz-bridge.yaml)"; fi

echo "== 5. mission node -> PX4"
PHASE=$(once /aufdeckung/phase | field data)
[ -n "$PHASE" ] && ok "mission phase: $PHASE" || bad "no mission phase: the mission node has not started the mission (see its 'waiting:' log line)"
SP=$(once /drone/in/offboard/pos/ned)
echo "$SP" | grep -q "x:" && ok "setpoints sent (n=$(echo "$SP" | field x) e=$(echo "$SP" | field y) alt=$(echo "$SP" | field z))" \
    || bad "no setpoints on /drone/in/offboard/pos/ned"

echo "== 6. last log lines of the mission node and the PX4 bridge"
LOGDIR=$(ls -td ~/.ros/log/*/ 2>/dev/null | head -1)
if [ -n "$LOGDIR" ]; then
    grep -hE "waiting:|failed|armed|flight mode|not airborne|PX4 connected|Connect|Error|error|Traceback|died" \
        "$LOGDIR"/*.log 2>/dev/null | tail -15 | sed 's/^/     /'
else info "no ROS log directory found"; fi
if tmux has-session -t aufdeckung-sim 2>/dev/null; then
    echo "  -- mission pane (ros2 window, top), last lines:"
    tmux capture-pane -p -t aufdeckung-sim:ros2.0 -S -200 2>/dev/null | grep -v "^$" | tail -30 | sed 's/^/     /'
fi

echo
if [ -z "$FIRST_BAD" ]; then echo "All links OK."
else echo "FIRST BROKEN LINK: $FIRST_BAD"; fi
