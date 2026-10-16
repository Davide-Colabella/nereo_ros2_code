# Integration test: mission bridge + guidance mode selector + /controller/target

Checks the thesis mission bridge (`nereo_mission_bridge`, repository
`Davide-Colabella/Tesi-Magistrale`, branch `modello-stateflow`) together with
`safety_node` (guidance mode selector) and `nereo_controller_node`
(`/controller/target`, repository `Davide-Colabella/ros2_controller_tuning_aid`,
branch `tesi-davide`), against a simulated vehicle.

| File | Role |
|------|------|
| `run.sh` | starts everything, runs a scenario, stops everything; exit status 0 only on `RESULT PASS` |
| `fake_vehicle.py` | `SIM=fake`: sensors fixed at the surface (depth 0, IMU, diagnostics), firmware arming `/set_arm_mode` → `/rov_armed`, tare services |
| `sim_dinamica` (`gui_pkg`) | `SIM=dinamica`: closed-loop simulator, see below |
| `pilot.py` | scenario `base` |
| `missione.py` | scenarios `completa`, `batteria`, `barometro`, `override` (closed loop) |

Every scenario prints `PASS`/`FAIL` per check, the states the chart went through
(`STATES Boot/WaitLink > ...`) and `RESULT PASS`/`RESULT FAIL`.

## Running

Build the three workspaces (ROS 2 Jazzy), then:

```bash
export NEREO_GUI_WS=~/nereo_ros2_code/gui_ws/install
export CONTROLLER_WS=~/ros2_controller_tuning_aid/install
export TESI_WS=~/Tesi-Magistrale/ros2/install
./integration_tests/mission_bridge/run.sh                                  # SIM=fake, base
SIM=dinamica SCENARIO=completa ./integration_tests/mission_bridge/run.sh   # closed loop
```

| Variable | Default | |
|---|---|---|
| `SIM` | `fake` | `fake` or `dinamica` |
| `SCENARIO` | `base` (fake), `completa` (dinamica) | `base` runs with both simulators, the others only with `dinamica` |
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

The same works for `imu_attiva`, `sonar_attivo` and `fondale_m`. `yaw_iniziale` (rad, world frame, default 0) is the yaw at start; set at run time it turns the vehicle there at once, as a pilot would, and the IMU reads it relative to its tare. The bench battery of the bridge is changed with `banco.batteria_pct`.

With `SIM=dinamica` the controller runs with **test gains** for the simulator, not tuned on the vehicle:
- `control_mode` 2 (PID with anti-windup);
- `authority_cap` 0.8;
- `kp` [1.5, 0.5, 0.5, 1.5], `ki` [0.03, 0, 0, 0.05], `kd` 0.

The PID is incremental, at the 10 Hz of the bridge. Its default gains are 0, which means no feedback.

## Scenarios `completa`, `batteria`, `barometro`, `override` (`SIM=dinamica`)

Plan, published by the scenario (seabed at 4.5 m, `max_depth` 4 m):

| # | Task | Depth | Heading | Speed | Duration | Altitude |
|---|---|---|---|---|---|---|
| 1 | HOLD | 2.0 m | — | — | 20 s | — |
| 2 | TRANSECT | 2.0 m | 0.8 rad | 0.25 m/s | 20 s | — |
| 3 | ALTITUDE_HOLD | — | 0.8 rad | — | 20 s | 2.0 m |

| Scenario | Event | Expected |
|---|---|---|
| `completa` | none | Ready → Mission (the three tasks) → Surfaced → Ready, `mission_done`, no abort reason. After the tare, in Ready, the vehicle is turned to 1.0 rad (`yaw_iniziale`). Checks on the simulator: yaw within 0.05 rad of 1.0 rad in the descent (LeaveSurface, DescendToDepth, SettleAtDepth), depth within 0.2 m in HOLD, turn to the transect heading in TrAlign, heading and course of the transect, altitude at the end of ALTITUDE_HOLD |
| `batteria` | bench battery at 20 % (`P_BATT_ABORT` 25 %) during HOLD | watchdog → Abort with reason BATTERY → emergency ascent → Surfaced → Ready |
| `barometro` | barometer silent during HOLD | health CRITICAL → FaultSafe with reason CRITICAL → disarm (FsLatched). The vehicle floats up; RESET is refused while the barometer is missing. Barometer back → RESET → FsResetting → Boot → Ready |
| `override` | joystick override during the transect | MANUAL → Abort with reason OPERATOR, AUTONOMOUS refused. The pilot brings the vehicle up with the joystick (direct mode) and disarms at the surface → Surfaced → Ready |

## Results (15/10/2026, ROS 2 Jazzy via RoboStack)

Run on 15/10/2026, 22:36–22:43 UTC, with the bridge of `Tesi-Magistrale` at `a87ed7c`
(`DEPTH_HOLD` follows `setpoint.heading`) and the controller of
`Davide-Colabella/ros2_controller_tuning_aid` at `8a6e150` (`tesi-davide`). All six runs passed
(`RESULT PASS`: `base` with both simulators and the four closed-loop scenarios), each
node running once, no process left after the run. States as printed by the scenarios
(top state / substate, abort reason in brackets; Boot omitted):

- **`completa`** (111 s): Ready/Idle > PlanLoaded > PreDive/RequestArm > WaitArmAck > Mission/LeaveSurface > DescendToDepth > SettleAtDepth > SelectTask > PrepareTask > Hold > SelectTask > PrepareTask > TrAlign > TrRun > SelectTask > PrepareTask > AltAcquire > AltTrack > SelectTask > AscToSafety > AscSlowFinal > Surfaced/SfConfirm > SfDisarm > SfReport > Ready/Idle, no abort reason, `mission_done`. On the simulator:
  - **HOLD:** depth 1.99 to 2.01 m.
  - **TrAlign:** the vehicle turns to the transect heading before moving: yaw from 0.00 to 0.88 rad (plan 0.80) in the 5 s of TrAlign, within 0.05 rad of the plan after 2.9 s, peak 0.884 rad (overshoot 0.08 rad) at 4.6 s, within 0.02 rad 1.8 s into TrRun.
  - **Transect:** yaw 0.80 rad, 4.6 m on course 0.80 rad.
  - **ALTITUDE_HOLD:** altitude 2.00 m.
  - **End:** disarmed at the surface.
- **`batteria`** (46 s): … Mission/Hold > Abort/AbStopHorizontal (BATTERY) > AbEmergencyAscent > Surfaced/SfConfirm > SfDisarm > SfReport > Ready/Idle. Abort 0.3 s after the battery change; Surfaced 11.4 s later.
- **`barometro`** (59 s): … Mission/Hold > FaultSafe/FsDisarm (CRITICAL) > FsLatched > [RESET refused without barometer] > FsResetting > Boot/WaitLink > CheckSensors > CheckBattery > Calibrate > Ready/Idle.
  - **Detection:** FaultSafe 0.8 s after the barometer went silent.
  - **Ascent:** the vehicle, disarmed, floated from 2.0 m to the surface in about 20 s.
- **`override`** (67 s): … Mission/TrAlign > TrRun > Abort/AbStopHorizontal (OPERATOR) > AbEmergencyAscent > Surfaced/SfConfirm > SfDisarm > SfReport > Ready/Idle.
  - **Selector:** AUTONOMOUS refused in Abort.
  - **Pilot:** brought the vehicle to the surface in 8.6 s with the joystick, then disarmed.

`mission_status.abort_reason` is the reason of the last interruption, not of the current
state: it stays set after the return to Ready, even through RESET and Boot (`barometro`:
CRITICAL in Boot and Ready), and the chart clears it only on entering PreDive, at the next
START. A reason seen in Ready is the one of the last mission.

Bags (`ros2 bag record -a`, mcap, 1 to 3 MB per run) are kept with the thesis:
`ros2/prove_integrazione` in `Tesi-Magistrale`, branch `modello-stateflow`.

What the closed loop does not show:
- **Heading in the descent:** in the runs above the generated code wrote `setpoint.heading` only in Hold, TrAlign, TrRun and GtAlign, so in the descent the bridge followed 0, the heading at the IMU tare; the vehicle started at yaw 0 and no turn showed.

### Descent heading (16/10/2026)

The chart specification (`spec.py` in Tesi-Magistrale, `modello-stateflow`) now sets `setpoint.heading = nav.yaw` on entering LeaveSurface and the task heading in AltAcquire; `completa` turns the vehicle to 1.0 rad after the tare and checks the descent yaw. **Until `modello/codegen` is regenerated in MATLAB, `completa` fails on that check**: with the current generated code the vehicle turns back to the tare heading in the descent (deviation 1.045 rad). With a copy of `mission_chart.cpp` patched by hand with the two assignments, outside the repository (`TESI_WS` built with `-DMISSION_CHART_CODEGEN_DIR`), `completa` passes with deviation 0.000 rad over 190 samples, and `batteria`, `barometro` and `override` pass too.
- **Simulator values:** the values not taken from `mission_env.m` and the controller gains are not vehicle values: the times and margins above are about the software chain, not the vehicle.
- **Roll and pitch:** not simulated.
- **Noise and losses:** no sensor noise and no packet loss, apart from the faults injected.
