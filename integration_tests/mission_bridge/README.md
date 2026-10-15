# Integration test: mission bridge + guidance mode selector + /controller/target

Checks the thesis mission bridge (`nereo_mission_bridge`, repository
`Davide-Colabella/Tesi-Magistrale`, branch `modello-stateflow`) together with
`safety_node` (guidance mode selector) and `nereo_controller_node`
(`/controller/target`, repository `PoliTOcean/ros2_controller_tuning_aid`,
branch `tesi-davide`), against a minimal simulated vehicle.

| File | Role |
|------|------|
| `fake_vehicle.py` | sensors (depth 0, IMU, diagnostics), firmware arming `/set_arm_mode` → `/rov_armed`, tare services, counts `/nereo_cmd_vel` |
| `pilot.py` | the scenario; prints `PASS`/`FAIL` per check and `RESULT PASS`/`RESULT FAIL` |
| `run.sh` | starts everything, runs the pilot, stops everything; exit status 0 only on `RESULT PASS` |

Scenario:

1. at start `/modalita_guida` is MANUAL (0);
2. the chart reaches Ready, AUTONOMOUS is requested through `/modalita_guida/richiedi_autonoma` and accepted;
3. in AUTONOMOUS a joystick command does not reach `/nereo_cmd_vel`;
4. LOAD and START: arming goes through the bridge (`/set_arm_mode_auto` → `safety_node` → `/set_arm_mode`), the chart reaches Mission, the controller follows `/controller/target`;
5. the joystick override button (`Joy` button 1) brings the mode back to MANUAL and the chart goes to Abort with reason OPERATOR; the pilot's controller-mode input is forwarded again;
6. AUTONOMOUS requested again in Abort is refused.

## Running

Build the three workspaces (ROS 2 Jazzy), then:

```bash
NEREO_GUI_WS=~/nereo_ros2_code/gui_ws/install \
CONTROLLER_WS=~/ros2_controller_tuning_aid/install \
TESI_WS=~/Tesi-Magistrale/ros2/install \
./integration_tests/mission_bridge/run.sh
```

The defaults are the `install` folders next to this repository. `ROS_DOMAIN_ID`
defaults to 77 to keep the test off the vehicle network; `LOG_DIR` sets where
the node logs go (a temporary folder otherwise).
