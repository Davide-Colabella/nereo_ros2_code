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
#   SCN=<file> another scenario of stonefish/data (default nereo.scn, e.g. a current from
#   genera_scenario.py --corrente); GUI=1 opens the Stonefish window (default: no window); fault scenarios guasto_*,
#   comandi_persi, imu_persa, sonar_perso (missione.py, FAULTS); YAW_KP, YAW_KI, YAW_KD
#   override the yaw gains of the controller. PLOT=1 opens PlotJuggler with the
#   Nereo layout (Tesi-Magistrale/stonefish/plotjuggler_nereo.xml). TESI_DIR: the
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
  stonefish/guasto_*|stonefish/nessun_guasto|stonefish/ritardo_*|stonefish/corrente_*|stonefish/ostacolo|stonefish/falsi_allarmi|stonefish/comandi_persi|stonefish/imu_persa|stonefish/sonar_perso) ;;
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
  : "${SCN:=nereo.scn}"   # another scenario of the same folder, e.g. nereo_corrente_0_0.1.scn
  [ -f "$SF_DATA/$SCN" ] || { echo "$SCN not found in $SF_DATA (TESI_DIR)"; exit 2; }
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
    # PING360=1: obstacles from the Ping360 of the scenario (genera_scenario.py --ping360):
    # it needs the GPU, so the simulator with the window; ping360_ostacoli.py turns the echo
    # profiles into /ostacoli/scan (use with OSTACOLI=1)
    if [ "${GUI:-0}" = 1 ] || [ "${PING360:-0}" = 1 ]; then
      start stonefish.log ros2 launch stonefish_ros2 stonefish_simulator.launch.py \
        simulation_data:="$SF_DATA" scenario_desc:="$SF_DATA/$SCN" \
        rendering_quality:=low window_res_x:=1280 window_res_y:=720
    else
      start stonefish.log ros2 launch stonefish_ros2 stonefish_simulator_nogpu.launch.py \
        simulation_data:="$SF_DATA" scenario_desc:="$SF_DATA/$SCN"
    fi
    # RITARDO=<s>: depth and IMU late by that much from the start (sim_stonefish ritardo_sensori_s)
    SIM_ARGS=()
    [ -n "${RITARDO:-}" ] && SIM_ARGS+=(-p ritardo_sensori_s:=$RITARDO)
    [ "${PING360:-0}" = 1 ] && SIM_ARGS+=(-p ostacoli_da_multibeam:=false)
    start sim_dinamica.log python3 "$TESI_DIR/stonefish/sim_stonefish.py" \
      ${SIM_ARGS[@]+--ros-args "${SIM_ARGS[@]}"}
    # PING360_ETA=<s>: oldest echo still used (3 s; ~12 s for the full rotation)
    [ "${PING360:-0}" = 1 ] && start ping360.log python3 "$TESI_DIR/stonefish/ping360_ostacoli.py" \
      --ros-args -p eta_max_s:=${PING360_ETA:-3.0}
    export YAW_TURN_S=30 YAW_TURN_TOL=0.02
  else
    start sim_dinamica.log ros2 run gui_pkg sim_dinamica
  fi
  # test gains for sim_dinamica (not tuned on the vehicle): incremental PID at the
  # 10 Hz of the bridge, anti-windup, correction capped at 0.8
  CTRL_ARGS=(-p control_mode:=2 -p authority_cap:=0.8
             -p "kp:=[1.5, 0.5, 0.5, 1.5]" -p "ki:=[0.03, 0.0, 0.0, 0.05]"
             -p "kd:=[0.0, 0.0, 0.0, 0.0]")
  if [ "$SIM" = stonefish ]; then
    # Stonefish has real yaw inertia and 4 vectored T200 on yaw: with the sim_dinamica
    # gains yaw oscillates. First guess (not tuned): lower yaw P, a little yaw D, lower yaw I.
    # The PID is incremental with D on the error: a large D gives a derivative kick at
    # every heading step that the anti-windup cuts only on one side (kd 3 turned the
    # vehicle the wrong way in TrAlign).
    # Order of the arrays: depth, roll, pitch, yaw.
    CTRL_ARGS=(-p control_mode:=2 -p authority_cap:=0.8
               -p "kp:=[1.5, 0.5, 0.5, ${YAW_KP:-0.6}]" -p "ki:=[0.03, 0.0, 0.0, ${YAW_KI:-0.01}]"
               -p "kd:=[0.0, 0.0, 0.0, ${YAW_KD:-0.5}]")
  fi
else
  start fake_vehicle.log python3 "$HERE/fake_vehicle.py"
  CTRL_ARGS=(-p control_mode:=1)
fi
start safety_node.log ros2 run web_pkg safety_node
start controller.log ros2 run nereo_controller_node nereo_controller_node --ros-args "${CTRL_ARGS[@]}"
# chart + bridge, bench mode (FpgaBus from /rov_armed, bench battery)
# OSTACOLI=1: the bridge reads the forward scan (/ostacoli/scan, from sim_stonefish) and
# publishes the obstacle bus; default off, as on the vehicle (no Ping360 yet)
OST_ARGS=()
[ "${OSTACOLI:-0}" = 1 ] && OST_ARGS=(topic_ostacoli:=/ostacoli/scan)
# PONTE_YAML=<file>: another parameter file for the bridge (e.g. other avoidance commands)
[ -n "${PONTE_YAML:-}" ] && OST_ARGS+=(parametri_ponte:="$PONTE_YAML")
start bridge.log ros2 launch nereo_mission_bridge nereo_mission_bridge.launch.py modalita_banco:=true "${OST_ARGS[@]}"
# PLOT=1: PlotJuggler with the Nereo layout, left open at the end to look at the run
if [ "${PLOT:-0}" = 1 ]; then
  : "${TESI_DIR:=$REPO/../Tesi-Magistrale}"
  [ -d "$TESI_DIR/stonefish" ] || TESI_DIR="$HOME/Tesi-Magistrale"
  setsid ros2 run plotjuggler plotjuggler -l "$TESI_DIR/stonefish/plotjuggler_nereo.xml" \
    > "$LOG/plotjuggler.log" 2>&1 &
fi
BAG_PID=
if [ "$BAG" = 1 ]; then
  rm -rf "$LOG/bag_$SCENARIO"
  setsid ros2 bag record -a -o "$LOG/bag_$SCENARIO" > "$LOG/bag_$SCENARIO.log" 2>&1 &
  BAG_PID=$!
fi

if [ "$SCENARIO" = base ]; then
  timeout 120 python3 "$HERE/pilot.py" | tee "$LOG/pilot_$SCENARIO.log"
else
  timeout 600 python3 "$HERE/missione.py" "$SCENARIO" | tee "$LOG/pilot_$SCENARIO.log"
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
