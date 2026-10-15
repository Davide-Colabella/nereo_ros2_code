# Integration test: mission bridge + guidance mode selector + /controller/target

Checks the thesis mission bridge (`nereo_mission_bridge`, repository
`Davide-Colabella/Tesi-Magistrale`, branch `modello-stateflow`) together with
`safety_node` (guidance mode selector) and `nereo_controller_node`
(`/controller/target`, repository `PoliTOcean/ros2_controller_tuning_aid`,
branch `tesi-davide`), against a simulated vehicle.

| File | Role |
|------|------|
| `run.sh` | starts everything, runs a scenario, stops everything; exit status 0 only on `RESULT PASS` |
| `fake_vehicle.py` | `SIM=fake`: sensors fixed at the surface (depth 0, IMU, diagnostics), firmware arming `/set_arm_mode` → `/rov_armed`, tare services |
| `sim_dinamica` (`gui_pkg`) | `SIM=dinamica`: closed-loop simulator, see below |
| `pilot.py` | scenario `base` |

Every scenario prints `PASS`/`FAIL` per check and `RESULT PASS`/`RESULT FAIL`.

## Running

Build the three workspaces (ROS 2 Jazzy), then:

```bash
export NEREO_GUI_WS=~/nereo_ros2_code/gui_ws/install
export CONTROLLER_WS=~/ros2_controller_tuning_aid/install
export TESI_WS=~/Tesi-Magistrale/ros2/install
./integration_tests/mission_bridge/run.sh                                  # SIM=fake, base
SIM=dinamica ./integration_tests/mission_bridge/run.sh                     # closed loop
```

| Variable | Default | |
|---|---|---|
| `SIM` | `fake` | `fake` or `dinamica` |
| `SCENARIO` | `base` | the scenario |
| `BAG` | 0 (fake), 1 (dinamica) | 1: `ros2 bag record -a` into `$LOG_DIR/bag_<scenario>` |
| `LOG_DIR` | a temporary folder | node logs, `pilot_<scenario>.log`, bags |
| `ROS_DOMAIN_ID` | 77 | keeps the test off the vehicle network |

The workspace defaults are the `install` folders next to this repository.

## Scenario `base` (both simulators)

1. At start `/modalita_guida` is MANUAL (0).
2. The chart reaches Ready. AUTONOMOUS is requested through `/modalita_guida/richiedi_autonoma` and accepted.
3. In AUTONOMOUS a joystick command does not reach `/nereo_cmd_vel`.
4. LOAD and START. Arming goes through the bridge (`/set_arm_mode_auto` → `safety_node` → `/set_arm_mode`), the chart reaches Mission, and the controller follows `/controller/target`.
5. The joystick override button (`Joy` button 1) brings the mode back to MANUAL, and the chart goes to Abort with reason OPERATOR. The pilot's controller-mode input is forwarded again, and the controller depth goes back to the measured depth.
6. AUTONOMOUS requested again in Abort is refused.

## Closed loop: `sim_dinamica`

`sim_dinamica` (package `gui_pkg`) takes the place of the flight-controller firmware and of the Raspberry Pi sensors, on the same topics and QoS:
- **In:** `/nereo_cmd_vel`, applied only while armed; `/set_arm_mode`.
- **Out:** `/rov_armed` at 40 Hz, `/barometer_depth`, `/imu_data` (yaw), the two diagnostics, `sonar/distance` and `sonar/confidence` (Ping1D on a seabed at `fondale_m`).
- **Services:** the tare services.
- **Ground truth:** `/sim_dinamica/stato` publishes `[t, depth, yaw, x, y, w, u, v, r, armed]`, for the bags.

Each axis is a velocity following its command with a first-order lag (`gui_pkg/sim_model.py`, unit-tested in `gui_pkg/test/test_sim_model.py`):

| Value | | Source |
|---|---|---|
| seabed `fondale_m` | 4.5 m | `mission_env.m`, `BOTTOM` |
| maximum vertical speed | 0.3 m/s | `mission_env.m`, `V_MAX` (also the default `vAsc`) |
| buoyancy, rise with no thrust | 0.1 m/s | `mission_env.m`, disarmed vehicle |
| maximum surge and sway | 0.5 m/s | chosen: `comandi.velocita_max_m_s` of the bridge |
| maximum yaw rate | 0.5 rad/s | chosen |
| time constants | 1 s (vertical, surge, sway), 0.5 s (yaw) | chosen |

`mission_env.m` moves the vehicle kinematically (velocity towards the setpoint), without thrusters, heading or speed. The values it lacks were chosen here and are not vehicle measurements. One difference: in `mission_env.m` an armed vehicle in NEUTRAL rises at 0.05 m/s; here buoyancy is the same armed or disarmed, and the controller compensates it.

Faults at run time:

```bash
ros2 param set /sim_dinamica barometro_attivo false
```

The same works for `imu_attiva`, `sonar_attivo` and `fondale_m`. The bench battery of the bridge is changed with `banco.batteria_pct`.

With `SIM=dinamica` the controller runs with **test gains** for the simulator, not tuned on the vehicle:
- `control_mode` 2 (PID with anti-windup);
- `authority_cap` 0.8;
- `kp` [1.5, 0.5, 0.5, 1.5], `ki` [0.03, 0, 0, 0.05], `kd` 0.

The PID is incremental, at the 10 Hz of the bridge. Its default gains are 0, which means no feedback.
