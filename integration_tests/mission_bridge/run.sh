#!/usr/bin/env bash
# Integration test: thesis mission bridge + safety_node selector + controller
# /controller/target, against a minimal simulated vehicle. See README.md.
#
#   NEREO_GUI_WS=<gui_ws install>  CONTROLLER_WS=<ros2_controller_tuning_aid install> \
#   TESI_WS=<Tesi-Magistrale ros2 install>  ./run.sh
#
# Defaults: the install folders next to the sources.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
: "${NEREO_GUI_WS:=$REPO/gui_ws/install}"
: "${CONTROLLER_WS:=$REPO/../ros2_controller_tuning_aid/install}"
: "${TESI_WS:=$REPO/../Tesi-Magistrale/ros2/install}"
: "${ROS_DOMAIN_ID:=77}"
export ROS_DOMAIN_ID
LOG="${LOG_DIR:-$(mktemp -d)}"
mkdir -p "$LOG"

set +u   # the colcon setup scripts use unset variables
source "$NEREO_GUI_WS/setup.bash"
source "$CONTROLLER_WS/setup.bash"
source "$TESI_WS/setup.bash"
set -u

python3 "$HERE/fake_vehicle.py" > "$LOG/fake_vehicle.log" 2>&1 & VEHICLE=$!
ros2 run web_pkg safety_node > "$LOG/safety_node.log" 2>&1 & SAFETY=$!
ros2 run nereo_controller_node nereo_controller_node --ros-args -p control_mode:=1 \
  > "$LOG/controller.log" 2>&1 & CONTROLLER=$!
# chart + bridge, bench mode (FpgaBus from /rov_armed, fixed battery)
ros2 launch nereo_mission_bridge nereo_mission_bridge.launch.py modalita_banco:=true \
  > "$LOG/bridge.log" 2>&1 & BRIDGE=$!

timeout 120 python3 "$HERE/pilot.py" | tee "$LOG/pilot.log"
RESULT=${PIPESTATUS[0]}

# background jobs of a non-interactive shell ignore SIGINT: stop with SIGTERM
kill -TERM $BRIDGE $CONTROLLER $SAFETY $VEHICLE 2>/dev/null
wait $BRIDGE $CONTROLLER $SAFETY $VEHICLE 2>/dev/null
echo "logs in $LOG"
grep -q '^RESULT PASS' "$LOG/pilot.log" && [ "$RESULT" -eq 0 ]
