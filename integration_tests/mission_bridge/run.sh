#!/usr/bin/env bash
# Integration test: thesis mission bridge + safety_node selector + controller
# /controller/target, against a simulated vehicle. See README.md.
#
#   NEREO_GUI_WS=<gui_ws install>  CONTROLLER_WS=<ros2_controller_tuning_aid install> \
#   TESI_WS=<Tesi-Magistrale ros2 install>  [SIM=fake|dinamica|stonefish] [SCENARIO=...] [BAG=0|1] \
#   ./run.sh
#
# SIM=fake (default): fake_vehicle.py, sensors fixed at the surface; scenario
#   pilot.py (SCENARIO=base, the only one).
# SIM=dinamica: sim_dinamica of gui_pkg, closed loop; SCENARIO=base, completa
#   (default), batteria, barometro or override (missione.py).
# SIM=stonefish: Stonefish with Tesi-Magistrale/stonefish/data/nereo.scn and
#   sim_stonefish.py (same interface and node name as sim_dinamica), same scenarios;
#   GUI=1 opens the Stonefish window (default: no window). TESI_DIR: the
#   Tesi-Magistrale clone (default ../Tesi-Magistrale next to this repo, or ~/Tesi-Magistrale).
# BAG=1 (default with SIM=dinamica or stonefish) records all topics with ros2 bag in
#   $LOG/bag_<scenario>.
# Defaults: the install folders next to the sources.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
: "${NEREO_GUI_WS:=$REPO/gui_ws/install}"
: "${CONTROLLER_WS:=$REPO/../ros2_controller_tuning_aid/install}"
: "${TESI_WS:=$REPO/../Tesi-Magistrale/ros2/install}"
: "${ROS_DOMAIN_ID:=77}"
: "${SIM:=fake}"
if [ "$SIM" = dinamica ] || [ "$SIM" = stonefish ]; then : "${SCENARIO:=completa}" "${BAG:=1}"; else : "${SCENARIO:=base}" "${BAG:=0}"; fi
export ROS_DOMAIN_ID
LOG="${LOG_DIR:-$(mktemp -d)}"
mkdir -p "$LOG"
case "$SIM/$SCENARIO" in
  fake/base|dinamica/base|dinamica/completa|dinamica/batteria|dinamica/barometro|dinamica/override) ;;
  stonefish/base|stonefish/completa|stonefish/batteria|stonefish/barometro|stonefish/override) ;;
  *) echo "unknown SIM/SCENARIO: $SIM/$SCENARIO"; exit 2 ;;
esac

set +u   # the colcon setup scripts use unset variables
# a single colcon workspace (all packages together) is fine too: missing folders are skipped
for ws in "$NEREO_GUI_WS" "$CONTROLLER_WS" "$TESI_WS"; do
  [ -f "$ws/setup.bash" ] && source "$ws/setup.bash"
done
set -u
if [ "$SIM" = stonefish ]; then
  : "${TESI_DIR:=$REPO/../Tesi-Magistrale}"
  [ -d "$TESI_DIR/stonefish" ] || TESI_DIR="$HOME/Tesi-Magistrale"
  SF_DATA="$TESI_DIR/stonefish/data"
  [ -f "$SF_DATA/nereo.scn" ] || { echo "nereo.scn not found in $SF_DATA (TESI_DIR)"; exit 2; }
fi

# each process in its own session, stopped as a whole group at the end: `ros2 run`
# does not pass SIGTERM on to the node it starts, which would be left running
PIDS=()
start() {   # start <log> <command...>
  local log=$1; shift
  setsid "$@" > "$LOG/$log" 2>&1 &
  PIDS+=($!)
}
if [ "$SIM" = dinamica ] || [ "$SIM" = stonefish ]; then
  if [ "$SIM" = stonefish ]; then
    if [ "${GUI:-0}" = 1 ]; then
      start stonefish.log ros2 launch stonefish_ros2 stonefish_simulator.launch.py \
        simulation_data:="$SF_DATA" scenario_desc:="$SF_DATA/nereo.scn" \
        rendering_quality:=low window_res_x:=1280 window_res_y:=720
    else
      start stonefish.log ros2 launch stonefish_ros2 stonefish_simulator_nogpu.launch.py \
        simulation_data:="$SF_DATA" scenario_desc:="$SF_DATA/nereo.scn"
    fi
    start sim_dinamica.log python3 "$TESI_DIR/stonefish/sim_stonefish.py"
  else
    start sim_dinamica.log ros2 run gui_pkg sim_dinamica
  fi
  # test gains for sim_dinamica (not tuned on the vehicle): incremental PID at the
  # 10 Hz of the bridge, anti-windup, correction capped at 0.8
  CTRL_ARGS=(-p control_mode:=2 -p authority_cap:=0.8
             -p "kp:=[1.5, 0.5, 0.5, 1.5]" -p "ki:=[0.03, 0.0, 0.0, 0.05]"
             -p "kd:=[0.0, 0.0, 0.0, 0.0]")
else
  start fake_vehicle.log python3 "$HERE/fake_vehicle.py"
  CTRL_ARGS=(-p control_mode:=1)
fi
start safety_node.log ros2 run web_pkg safety_node
start controller.log ros2 run nereo_controller_node nereo_controller_node --ros-args "${CTRL_ARGS[@]}"
# chart + bridge, bench mode (FpgaBus from /rov_armed, bench battery)
start bridge.log ros2 launch nereo_mission_bridge nereo_mission_bridge.launch.py modalita_banco:=true
BAG_PID=
if [ "$BAG" = 1 ]; then
  rm -rf "$LOG/bag_$SCENARIO"
  setsid ros2 bag record -a -o "$LOG/bag_$SCENARIO" > "$LOG/bag_$SCENARIO.log" 2>&1 &
  BAG_PID=$!
fi

if [ "$SCENARIO" = base ]; then
  timeout 120 python3 "$HERE/pilot.py" | tee "$LOG/pilot_$SCENARIO.log"
else
  timeout 400 python3 "$HERE/missione.py" "$SCENARIO" | tee "$LOG/pilot_$SCENARIO.log"
fi
RESULT=${PIPESTATUS[0]}

# background jobs of a non-interactive shell ignore SIGINT: stop with SIGTERM
# (ros2 bag closes the bag on SIGTERM too), to each process group
if [ -n "$BAG_PID" ]; then kill -TERM -- "-$BAG_PID" 2>/dev/null; wait "$BAG_PID" 2>/dev/null; fi
for p in "${PIDS[@]}"; do kill -TERM -- "-$p" 2>/dev/null; done
wait "${PIDS[@]}" 2>/dev/null
sleep 1
for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
echo "logs in $LOG"
grep -q '^RESULT PASS' "$LOG/pilot_$SCENARIO.log" && [ "$RESULT" -eq 0 ]
