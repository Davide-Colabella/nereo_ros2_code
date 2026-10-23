"""
Closed-loop scenarios of the mission bridge integration test, against
sim_dinamica (gui_pkg). Each check prints PASS or FAIL; the run ends with the
list of states the chart went through and "RESULT PASS" or "RESULT FAIL".

  python3 missione.py <scenario>

  completa  plan of three tasks (HOLD, TRANSECT, ALTITUDE_HOLD) from Ready to
            Surfaced and back to Ready, all driven by the bridge in AUTONOMOUS;
            the vehicle is turned to YAW_START after the tare and must keep that
            yaw in the descent
  batteria  battery under P_BATT_ABORT (25 %) during HOLD: watchdog, Abort
            with reason BATTERY, emergency ascent, Surfaced, Ready
  barometro barometer lost during HOLD: health CRITICAL, FaultSafe with reason
            CRITICAL, disarm, the vehicle floats up; barometer back, RESET,
            Boot, Ready
  override  pilot override during the transect: MANUAL, Abort with reason
            OPERATOR; the pilot brings the vehicle up with the joystick and
            disarms at the surface: Surfaced, Ready

  guasto_*  SIM=stonefish only: the completa plan with one fault injected during a
            phase (FAULTS below: thrusters lost or weak, a sensor lost, commands
            lost). Checks that the vehicle ends at the surface, disarmed; prints the
            chart's reaction (DETECTION) and what the vehicle did after the fault
            (EFFECT), against what the spec makes us expect (EXPECTED)

Faults are injected through parameters: banco.batteria_pct of the bridge (bench
battery), barometro_attivo and yaw_iniziale of sim_dinamica; with Stonefish also
motori_guasti, efficienza_guasto, imu_attiva, sonar_attivo, comandi_attivi.
"""
import math
import os
import sys
import time

import rclpy
from nereo_interfaces.msg import CommandVelocity
from nereo_mission_msgs.msg import Cmd, MissionStatusBus, NavBus, ObstacleBus, PlanBus
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, Float64MultiArray, UInt8
from std_srvs.srv import Trigger

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

# from Tesi-Magistrale modello/mission_substates.m and generatore_python/spec.py
TOP = ['Boot', 'Ready', 'PreDive', 'Mission', 'Abort', 'Surfaced', 'FaultSafe', 'Shutdown']
SUB = ['WaitLink', 'CheckSensors', 'CheckBattery', 'Calibrate', 'Idle', 'PlanLoaded',
       'RequestArm', 'WaitArmAck', 'ThrusterCheck', 'LeaveSurface', 'DescendToDepth',
       'SettleAtDepth', 'SelectTask', 'PrepareTask', 'TaskFailed', 'Hold', 'TrAlign', 'TrRun',
       'AltAcquire', 'AltTrack', 'AltFallback', 'GtAlign', 'GtApproach', 'VisSearch', 'VisTrack',
       'Paused', 'NavDegraded', 'AvStop', 'AvBackOff', 'AvScanYaw', 'AvDetour', 'AvResumeCheck',
       'AscToSafety', 'AscSlowFinal', 'AbStopHorizontal', 'AbEmergencyAscent', 'SfConfirm',
       'SfDisarm', 'SfReport', 'FsDisarm', 'FsLatched', 'FsResetting', 'SdCloseLogs', 'SdAck']
REASON = ['NONE', 'OPERATOR', 'HEALTH', 'BATTERY', 'DEPTH_LIMIT', 'ALTITUDE_LIMIT', 'TIMEOUT',
          'GEOFENCE', 'ATTITUDE', 'TASK_FAILED', 'NO_ASCENT', 'FPGA_OVERRIDE', 'LINK_LOST',
          'INCOHERENT', 'CRITICAL', 'SELFTEST', 'ARM_TIMEOUT', 'ARM_REFUSED', 'THRUSTER_CHECK',
          'DISARM_TIMEOUT', 'SHUTDOWN', 'TRACKING']
READY, MISSION, ABORT, SURFACED, FAULTSAFE = 1, 3, 4, 5, 6

# plan: (type, depth, heading, speed, duration, altitude); TaskType HOLD 1,
# TRANSECT 2, ALTITUDE_HOLD 3. Seabed of sim_dinamica at 4.5 m.
HEADING = 0.8
PLAN = [(1, 2.0, 0.0, 0.0, 20.0, 0.0),
        (2, 2.0, HEADING, 0.25, 20.0, 0.0),
        (3, 0.0, HEADING, 0.0, 20.0, 2.0)]
OVERRIDE_BUTTON = 1
YAW_START = 1.0   # completa: yaw set after the tare, kept in the descent
DESCENT = ('LeaveSurface', 'DescendToDepth', 'SettleAtDepth')


NODES = ('safety_node', 'nereo_controller_node', 'nereo_mission_bridge', 'nereo_mission')


def check_single_nodes(node, check, nodes=NODES, seconds=3.0, timeout=15.0):
    """Every node of the test runs once: a node left over from an earlier run
    (same domain) would publish on the same topics. Waits at least seconds, so
    that a duplicate has time to show up, and then up to timeout for the nodes
    discovery has not reported yet (slow after a fresh build)."""
    start = time.monotonic()
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        names = [n for n, _ in node.get_node_names_and_namespaces()]
        elapsed = time.monotonic() - start
        if elapsed >= timeout or (elapsed >= seconds and all(n in names for n in nodes)):
            break
    dup = sorted({n for n in names if names.count(n) > 1})
    missing = [n for n in nodes if n not in names]
    return check(not dup and not missing, 'each node once (duplicates %s, missing %s)' % (
        dup, missing))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def name(sub):
    return SUB[sub] if sub < len(SUB) else str(sub)


# sim_dinamica turns the vehicle at once; a physical simulator (SIM=stonefish) turns it
# with the thrusters, as a pilot would: run.sh widens time and tolerance there
TURN_S = float(os.environ.get('YAW_TURN_S', '2.0'))
TURN_TOL = float(os.environ.get('YAW_TURN_TOL', '1e-3'))


def param(name_, value):
    p = Parameter(name=name_)
    if isinstance(value, bool):
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    elif isinstance(value, str):
        p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
    else:
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
    return p


class Scenario(Node):
    def __init__(self):
        super().__init__('integration_missione')
        self.t0 = time.monotonic()
        self.ok = True
        self.mode = None
        self.status = None
        self.path = []          # (t, state, substate, reason) at every change
        self.truth = None       # sim_dinamica: [t, depth, yaw, x, y, w, u, v, r, armed]
        self.depth_meas = None
        self.setpoints = None
        self.avoid = False      # plan.avoid_enabled
        self.plan = None        # another plan than PLAN
        self.nav = None         # missione/nav, the bridge estimate
        self.obstacle = None    # missione/obstacle, if the bridge publishes it
        self.create_subscription(UInt8, '/modalita_guida',
                                 lambda m: setattr(self, 'mode', m.data), LATCHED)
        self.create_subscription(MissionStatusBus, 'missione/mission_status', self._status, 10)
        self.create_subscription(Float64MultiArray, '/sim_dinamica/stato',
                                 lambda m: setattr(self, 'truth', list(m.data)), 10)
        self.create_subscription(Float32, '/barometer_depth',
                                 lambda m: setattr(self, 'depth_meas', m.data), BEST_EFFORT)
        self.create_subscription(Float64MultiArray, '/controller/setpoints',
                                 lambda m: setattr(self, 'setpoints', list(m.data)), 10)
        self.create_subscription(NavBus, 'missione/nav', lambda m: setattr(self, 'nav', m), 10)
        self.create_subscription(ObstacleBus, 'missione/obstacle',
                                 lambda m: setattr(self, 'obstacle', m), 10)
        self.p_plan = self.create_publisher(PlanBus, 'missione/plan', LATCHED)
        self.p_cmd = self.create_publisher(Cmd, 'missione/cmd', 10)
        self.p_joy = self.create_publisher(Joy, 'joy', 10)
        self.p_joy_direct = self.create_publisher(CommandVelocity, '/nereo_cmd_vel_joy', 10)
        self.p_arm_pilot = self.create_publisher(Bool, '/set_arm_mode_manual', 10)
        self.auto_srv = self.create_client(Trigger, '/modalita_guida/richiedi_autonoma')
        self.bridge_params = self.create_client(
            SetParameters, '/nereo_mission_bridge/set_parameters')
        self.sim_params = self.create_client(SetParameters, '/sim_dinamica/set_parameters')

    # ── bookkeeping ───────────────────────────────────────────────────────

    def now(self):
        return time.monotonic() - self.t0

    def log(self, text):
        print('%6.1f %s' % (self.now(), text), flush=True)

    def check(self, cond, text):
        self.ok = self.ok and bool(cond)
        self.log(('PASS ' if cond else 'FAIL ') + text)
        return bool(cond)

    def _status(self, m):
        key = (m.state, m.substate, m.abort_reason)
        if not self.path or self.path[-1][1:] != key:
            self.path.append((self.now(),) + key)
            extra = ''
            if self.truth:
                extra = ' (depth %.2f m, yaw %.2f rad, armed %d)' % (
                    self.truth[1], self.truth[2], self.truth[9])
            self.log('chart: %s/%s reason %s%s' % (
                TOP[m.state] if m.state < len(TOP) else m.state, name(m.substate),
                REASON[m.abort_reason] if m.abort_reason < len(REASON) else m.abort_reason,
                extra))
        self.status = m

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, cond, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if cond():
                return True
        return False

    def in_sub(self, *names):
        return self.status is not None and name(self.status.substate) in names

    def visited(self):
        """Substates in order, consecutive repeats merged."""
        out = []
        for _, _, sub, _ in self.path:
            if not out or out[-1] != name(sub):
                out.append(name(sub))
        return out

    def went_through(self, sequence):
        """True if sequence is a subsequence of the visited substates."""
        it = iter(self.visited())
        return all(any(s == v for v in it) for s in sequence)

    def reasons(self):
        return sorted({REASON[r] for _, _, _, r in self.path if 0 < r < len(REASON)})

    # ── actions ───────────────────────────────────────────────────────────

    def call(self, client, request, seconds=5.0):
        if not client.wait_for_service(timeout_sec=seconds):
            return None
        f = client.call_async(request)
        rclpy.spin_until_future_complete(self, f, timeout_sec=seconds)
        return f.result()

    def set_params(self, client, **values):
        r = self.call(client, SetParameters.Request(
            parameters=[param(k, v) for k, v in values.items()]))
        return r is not None and all(x.successful for x in r.results)

    def command(self, cmd, cond, seconds=2.0):
        for _ in range(5):
            self.p_cmd.publish(Cmd(cmd=cmd))
            if self.wait_for(cond, seconds):
                return True
        return False

    def publish_plan(self):
        piano = self.plan or PLAN
        plan = PlanBus(valid=True, len=len(piano), max_depth=4.0, max_time=900.0,
                       avoid_enabled=self.avoid)
        for i, task in enumerate(piano):
            typ, depth, heading, speed, duration, altitude = task[:6]
            plan.type[i] = typ
            plan.depth[i] = depth
            plan.heading[i] = heading
            plan.speed[i] = speed
            plan.duration[i] = duration
            plan.altitude[i] = altitude
            if len(task) > 6:   # GOTO: (..., x, y) in the bridge frame (origin at the tare)
                plan.x[i], plan.y[i] = task[6], task[7]
        self.p_plan.publish(plan)

    def override_button(self):
        buttons = [0] * 12
        buttons[OVERRIDE_BUTTON] = 1
        self.p_joy.publish(Joy(buttons=buttons))
        self.spin_for(0.1)
        self.p_joy.publish(Joy(buttons=[0] * 12))

    # ── common part: from start to Mission ────────────────────────────────

    def start_mission(self, yaw=None):
        """From the start to Mission; yaw: turn the vehicle there in Ready, after the tare."""
        check_single_nodes(self, self.check, NODES + ('sim_dinamica',))
        self.check(self.wait_for(lambda: self.mode is not None, 10.0) and self.mode == 0,
                   'guidance mode MANUAL at start')
        self.check(self.wait_for(lambda: self.status is not None and self.status.state == READY,
                                 60.0), 'chart in Ready')
        if yaw is not None:
            ok = self.set_params(self.sim_params, yaw_iniziale=yaw) and self.wait_for(
                lambda: self.truth and abs(wrap(self.truth[2] - yaw)) < TURN_TOL, TURN_S)
            self.check(ok, 'vehicle turned to %.2f rad after the tare (yaw %.3f rad)' % (
                yaw, self.truth[2] if self.truth else float('nan')))
        self.publish_plan()
        self.spin_for(0.5)
        r = self.call(self.auto_srv, Trigger.Request())
        self.check(r is not None and r.success,
                   'AUTONOMOUS accepted: %s' % (r.message if r else 'no reply'))
        self.check(self.wait_for(lambda: self.mode == 1, 2.0), '/modalita_guida = 1')
        self.check(self.command(Cmd.LOAD, lambda: self.in_sub('PlanLoaded')), 'LOAD: PlanLoaded')
        self.check(self.command(Cmd.START, lambda: self.status.state in (2, 3)), 'START: PreDive')
        return self.check(self.wait_for(lambda: self.status.state == MISSION, 15.0),
                          'armed through the bridge, chart in Mission')

    def track(self, sub, seconds, sample):
        """Wait for substate sub, then sample(truth) every 0.5 s while in it."""
        if not self.wait_for(lambda: self.in_sub(sub), seconds):
            return []
        out = []
        while self.in_sub(sub):
            self.spin_for(0.5)
            if self.truth and self.in_sub(sub):
                out.append(sample(self.truth))
        return out


# ── scenarios ─────────────────────────────────────────────────────────────

def completa(s):
    if not s.start_mission(yaw=YAW_START):
        return
    # the chart descends on the current heading (LeaveSurface: setpoint.heading = nav.yaw)
    descent = []
    while s.in_sub(*DESCENT):
        s.spin_for(0.1)
        if s.truth and s.in_sub(*DESCENT):
            descent.append(s.truth[2])
    # reference: the true heading entering LeaveSurface, not YAW_START (the turn in Ready
    # stops within YAW_TURN_TOL of it, which is not the chart's doing)
    yaw0 = descent[0] if descent else YAW_START
    dev = max((abs(wrap(y - yaw0)) for y in descent), default=float('inf'))
    s.check(len(descent) > 10 and dev < 0.05,
            'descent: yaw within 0.05 rad of %.2f, the heading at LeaveSurface '
            '(max deviation %.3f rad, %d samples)' % (yaw0, dev, len(descent)))
    depth = s.track('Hold', 60.0, lambda x: x[1])
    s.check(depth and max(abs(d - 2.0) for d in depth[2:]) < 0.2,
            'HOLD: depth within 0.2 m of 2.0 (%.2f .. %.2f m)' % (
                min(depth or [0]), max(depth or [0])))
    # the bridge maps DEPTH_HOLD to setpoint.heading: the turn happens in TrAlign
    yaw_in = s.truth[2] if s.wait_for(lambda: s.in_sub('TrAlign'), 60.0) and s.truth else None
    align = s.track('TrAlign', 1.0, lambda x: x[2])
    # TrAlign ends after P_ALIGN_TIME, not on the heading: the chart promises a turn
    # in the right direction, not that it is over (the heading is checked at the end
    # of TrRun). On Stonefish the turn stalls at 79..91 % of the step after 5 s.
    if yaw_in is not None and align:
        done = wrap(align[-1] - yaw_in) / wrap(HEADING - yaw_in)
        s.check(abs(yaw_in) < 0.1 and done > 0.75 and wrap(align[-1] - HEADING) < 0.1,
                'TrAlign: yaw from %.2f to %.2f rad (plan %.2f), %.0f %% of the turn before TrRun' % (
                    yaw_in, align[-1], HEADING, 100 * done))
    else:
        s.check(False, 'TrAlign reached')
    run = s.track('TrRun', 60.0, lambda x: (x[2], x[3], x[4], x[1]))
    if run:
        yaw_end = run[-1][0]
        dist = math.hypot(run[-1][1] - run[0][1], run[-1][2] - run[0][2])
        course = math.atan2(run[-1][2] - run[0][2], run[-1][1] - run[0][1])
        s.check(abs(yaw_end - HEADING) < 0.1 and dist > 2.0 and abs(course - HEADING) < 0.35,
                'TRANSECT: yaw %.2f rad (plan %.2f), %.1f m on course %.2f rad, depth %.2f m' % (
                    yaw_end, HEADING, dist, course, run[-1][3]))
    else:
        s.check(False, 'TRANSECT: TrRun reached')
    alt = s.track('AltTrack', 60.0, lambda x: 4.5 - x[1])
    s.check(alt and abs(alt[-1] - 2.0) < 0.3,
            'ALTITUDE_HOLD: altitude %.2f m at the end (plan 2.0)' % (alt[-1] if alt else -1))
    s.check(s.wait_for(lambda: s.status.state == SURFACED, 90.0), 'ascent: chart in Surfaced')
    s.check(s.wait_for(lambda: s.status.state == READY, 20.0) and s.status.mission_done,
            'disarmed at the surface, chart back in Ready, mission_done')
    s.check(s.truth[9] == 0.0 and s.truth[1] < 0.3,
            'vehicle disarmed at %.2f m' % s.truth[1])
    s.check(s.went_through(
        ['Idle', 'PlanLoaded', 'RequestArm', 'WaitArmAck', 'LeaveSurface', 'DescendToDepth',
         'SettleAtDepth', 'SelectTask', 'PrepareTask', 'Hold', 'SelectTask', 'PrepareTask',
         'TrAlign', 'TrRun', 'SelectTask', 'PrepareTask', 'AltAcquire', 'AltTrack',
         'SelectTask', 'AscToSafety', 'AscSlowFinal', 'SfConfirm', 'SfDisarm', 'SfReport',
         'Idle']), 'expected sequence of substates')
    s.check(s.reasons() == [] and 'TaskFailed' not in s.visited(),
            'no abort reason, no task failed (%s)' % s.reasons())


def batteria(s):
    if not s.start_mission():
        return
    s.check(s.wait_for(lambda: s.in_sub('Hold'), 60.0), 'HOLD reached')
    s.spin_for(3.0)
    s.check(s.set_params(s.bridge_params, **{'banco.batteria_pct': 20.0}),
            'bench battery set to 20 % (P_BATT_ABORT 25 %)')
    s.check(s.wait_for(lambda: s.status.state == ABORT, 3.0) and
            REASON[s.status.abort_reason] == 'BATTERY', 'Abort with reason BATTERY')
    s.check(s.wait_for(lambda: s.status.state == SURFACED, 60.0), 'emergency ascent: Surfaced')
    s.check(s.wait_for(lambda: s.status.state == READY, 20.0), 'disarmed, back in Ready')
    s.check(s.went_through(['Hold', 'AbStopHorizontal', 'AbEmergencyAscent', 'SfConfirm',
                            'SfDisarm', 'SfReport', 'Idle']), 'expected sequence of substates')
    s.check(s.reasons() == ['BATTERY'], 'only reason BATTERY (%s)' % s.reasons())


def barometro(s):
    if not s.start_mission():
        return
    s.check(s.wait_for(lambda: s.in_sub('Hold'), 60.0), 'HOLD reached')
    s.spin_for(3.0)
    d0 = s.truth[1]
    s.check(s.set_params(s.sim_params, barometro_attivo=False),
            'barometer stops publishing at %.2f m' % d0)
    s.check(s.wait_for(lambda: s.status.state == FAULTSAFE, 5.0) and
            REASON[s.status.abort_reason] == 'CRITICAL', 'FaultSafe with reason CRITICAL')
    s.check(s.wait_for(lambda: s.in_sub('FsLatched'), 5.0) and
            s.wait_for(lambda: s.truth[9] == 0.0, 2.0), 'disarmed: FsLatched')
    s.check(s.wait_for(lambda: s.truth[1] < 0.05, 60.0),
            'vehicle floats up by buoyancy to the surface (%.2f m)' % s.truth[1])
    # RESET refused while the barometer is still missing (health CRITICAL)
    s.p_cmd.publish(Cmd(cmd=Cmd.RESET))
    s.spin_for(2.0)
    s.check(s.in_sub('FsLatched'), 'RESET without barometer: still FsLatched')
    s.check(s.set_params(s.sim_params, barometro_attivo=True), 'barometer back')
    s.spin_for(2.0)
    s.check(s.command(Cmd.RESET, lambda: s.status.state in (0, READY), 3.0),
            'RESET at the surface: FsResetting, Boot')
    s.check(s.wait_for(lambda: s.status.state == READY, 60.0), 'Boot again, chart in Ready')
    s.check(s.went_through(['Hold', 'FsDisarm', 'FsLatched', 'FsResetting', 'WaitLink',
                            'Calibrate', 'Idle']), 'expected sequence of substates')
    s.check(s.reasons() == ['CRITICAL'], 'only reason CRITICAL (%s)' % s.reasons())


def override(s):
    if not s.start_mission():
        return
    s.check(s.wait_for(lambda: s.in_sub('TrRun'), 90.0), 'TRANSECT running')
    s.spin_for(3.0)
    s.override_button()
    s.check(s.wait_for(lambda: s.mode == 0, 2.0), 'override button: MANUAL')
    s.check(s.wait_for(lambda: s.status.state == ABORT, 3.0) and
            REASON[s.status.abort_reason] == 'OPERATOR', 'Abort with reason OPERATOR')
    r = s.call(s.auto_srv, Trigger.Request())
    s.check(r is not None and not r.success, 'AUTONOMOUS refused in Abort: %s' % (
        r.message if r else 'no reply'))
    # the bridge no longer commands: the pilot brings the vehicle up
    s.log('pilot: full heave up with the joystick (direct mode)')
    end = time.monotonic() + 60.0
    while time.monotonic() < end and s.status.state != SURFACED:
        s.p_joy_direct.publish(CommandVelocity(cmd_vel=[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
        s.spin_for(0.05)
    s.check(s.status.state == SURFACED, 'pilot at the surface: chart in Surfaced')
    for _ in range(10):
        s.p_arm_pilot.publish(Bool(data=False))
        s.spin_for(0.05)
    s.check(s.wait_for(lambda: s.status.state == READY, 20.0), 'pilot disarms: chart in Ready')
    s.check(s.went_through(['TrRun', 'AbStopHorizontal', 'AbEmergencyAscent', 'SfConfirm',
                            'SfDisarm', 'SfReport', 'Idle']), 'expected sequence of substates')
    s.check(s.reasons() == ['OPERATOR'], 'only reason OPERATOR (%s)' % s.reasons())


# ── faults (SIM=stonefish) ────────────────────────────────────────────────

# name: (phase, seconds into it, sim parameters, what the spec makes us expect)
# Thrusters in firmware order: 1..4 horizontal (FR, FL, RR, RL), 5..8 vertical
# (FR, FL, RR, RL). The chart has no per-thruster signal: it can only see the
# effects (Descend timeout, NO_ASCENT, ATTITUDE, AltAcquire timeout) or the
# firmware state (SAFE_STOP, LATCHED_FAULT), which the bridge does not get yet.
FAULTS = {
    'nessun_guasto': ('Hold', 3.0, {}, 'none: reference for the EFFECT lines of the others'),
    'guasto_discesa': ('DescendToDepth', 1.0, {'motori_guasti': '5', 'efficienza_guasto': 0.0},
                       'none or Descend TIMEOUT (60 s) if the depth is not reached'),
    'guasto_hold': ('Hold', 3.0, {'motori_guasti': '5', 'efficienza_guasto': 0.0},
                    'none (no guard on the depth error in a task)'),
    'guasto_assetto': ('Hold', 3.0, {'motori_guasti': '5,7', 'efficienza_guasto': 0.0},
                       'Abort ATTITUDE if roll or pitch stays over 30 deg for 3 s'),
    'guasto_verticali_hold': ('Hold', 3.0, {'motori_guasti': '5,6,7,8', 'efficienza_guasto': 0.0},
                              'Abort TRACKING (the vehicle floats up off the task depth), '
                              'with the chart of 22/10/2026; none before it'),
    'guasto_verticali_deboli': ('Hold', 3.0,
                                {'motori_guasti': '5,6,7,8', 'efficienza_guasto': 0.3},
                                'none if 30 % of vertical thrust still holds the depth, '
                                'else Abort TRACKING'),
    'guasto_transetto': ('TrRun', 3.0, {'motori_guasti': '1', 'efficienza_guasto': 0.0},
                         'none (no guard on heading or track in a task)'),
    'guasto_debole': ('TrRun', 3.0, {'motori_guasti': '1,2,3,4', 'efficienza_guasto': 0.5},
                      'none (half thrust on all horizontals: slower transect)'),
    'guasto_risalita': ('AscToSafety', 1.0,
                        {'motori_guasti': '5,6,7,8', 'efficienza_guasto': 0.0},
                        'NO_ASCENT if under 0.5 m of ascent in 20 s; buoyancy alone may do'),
    'comandi_persi': ('TrRun', 3.0, {'comandi_attivi': False},
                      'Abort TRACKING once the depth error stays over P_TRACK_DEPTH (0.5 m) for '
                      'P_TRACK_TIME (5 s), with the chart of 22/10/2026; none before it'),
    'ritardo_200ms': ('LeaveSurface', 0.0, {'ritardo_sensori_s': 0.2},
                      'none: depth and IMU 0.2 s late from the descent on (compare EFFECT)'),
    'ritardo_500ms': ('LeaveSurface', 0.0, {'ritardo_sensori_s': 0.5},
                      'FaultSafe CRITICAL: switching the delay on leaves the IMU silent for '
                      '0.5 s plus a period, over the bridge freshness of 0.6 s (the steady '
                      'delay from the start is run.sh RITARDO=)'),
    # the current of the scenario file (run.sh SCN=nereo_corrente_<vx>_<vy>.scn) switched on
    'corrente_hold': ('Hold', 3.0, {'correnti_attive': True},
                      'none: no position, the chart cannot see the drift (EFFECT track)'),
    'corrente_transetto': ('TrRun', 0.5, {'correnti_attive': True},
                           'none: the heading is held, the course drifts (EFFECT track)'),
    'imu_persa': ('Hold', 3.0, {'imu_attiva': False},
                  'FaultSafe CRITICAL (IMU stale), disarm, float up'),
    'sonar_perso': ('AltTrack', 3.0, {'sonar_attivo': False},
                    'AltFallback after P_ALT_LOSS (2 s), TaskFailed after 20 s, policy'),
}


def guasto(s, which):
    phase, after, params, expected = FAULTS[which]
    s.log('EXPECTED %s' % expected)
    if not s.start_mission():
        return
    if not s.check(s.wait_for(lambda: s.in_sub(phase), 150.0), '%s reached' % phase):
        return
    s.spin_for(after)
    t_f = s.now()
    n_path = len(s.path)
    ok = s.set_params(s.sim_params, **params)
    s.check(ok, 'fault injected in %s: %s' % (phase, params))
    # after the fault, in every substate of Mission, after its first SETTLE_S (the
    # transient of the step in the setpoints): attitude, and the depth and heading
    # errors against the controller setpoints; track in the faulty phase; until the
    # vehicle is up and disarmed
    SETTLE_S = 3.0
    stats = {}     # substate -> [samples, max |roll|,|pitch|, max depth err, max heading err]
    xy = []
    up = False
    sub_in, t_in = None, 0.0
    end = time.monotonic() + 300.0
    while time.monotonic() < end:
        s.spin_for(0.1)
        x = s.truth
        if not x:
            continue
        sub = name(s.status.substate)
        if sub != sub_in:
            sub_in, t_in = sub, s.now()
        if (s.status.state == MISSION and s.setpoints and len(x) >= 12 and
                s.now() - t_in > SETTLE_S):
            st = stats.setdefault(sub, [0, 0.0, 0.0, 0.0])
            st[0] += 1
            st[1] = max(st[1], abs(x[10]), abs(x[11]))
            st[2] = max(st[2], abs(x[1] - s.setpoints[0]))
            st[3] = max(st[3], abs(wrap(x[2] - s.setpoints[3])))
        if s.in_sub(phase):
            xy.append((x[3], x[4]))
        if x[9] == 0.0 and x[1] < 0.3 and s.status.state in (READY, FAULTSAFE):
            up = True
            break
    reacted = [(t, st, sub, r) for t, st, sub, r in s.path[n_path:]
               if st in (ABORT, FAULTSAFE) or name(sub) in ('TaskFailed', 'AltFallback')]
    if reacted:
        t, st, sub, r = reacted[0]
        s.log('DETECTION %s/%s reason %s, %.1f s after the fault' % (
            TOP[st], name(sub), REASON[r], t - t_f))
    else:
        s.log('DETECTION none: the chart did not leave the mission plan')
    track = ''
    if len(xy) > 1:
        (x0, y0), (x1, y1) = xy[0], xy[-1]
        track = ': %.1f m on course %.2f rad' % (
            math.hypot(x1 - x0, y1 - y0), math.atan2(y1 - y0, x1 - x0))
    s.log('EFFECT track in %s%s; up and disarmed %.1f s after the fault' % (
        phase, track or ': none', s.now() - t_f))
    # depth error not meaningful where the depth setpoint is not what the vehicle follows
    no_depth = ('AltAcquire', 'AltTrack', 'AltFallback', 'AscToSafety', 'AscSlowFinal')
    for sub, (n, att, dep, yaw_e) in stats.items():
        s.log('EFFECT %-13s %3d samples: max |roll|,|pitch| %4.1f deg, depth error %s, '
              'heading error %.3f rad' % (sub, n, math.degrees(att),
                                          '  -   ' if sub in no_depth else '%.2f m' % dep, yaw_e))
    s.check(up, 'vehicle at the surface and disarmed (chart %s, depth %.2f m)' % (
        TOP[s.status.state], s.truth[1] if s.truth else float('nan')))
    s.log('path after the fault: %s' % ' > '.join(
        name(sub) + ('(%s)' % REASON[r] if r else '') for _, _, sub, r in _merged(s.path[n_path:])))


# ── obstacle (SIM=stonefish, OSTACOLI=1, scenario with a pole) ────────────

def ostacolo(s):
    """completa plan with avoid_enabled and a pole on the transect: the bridge sees it in the
    forward scan, the chart goes through AvoidObstacle. OSTACOLO_XY=x,y is where the pole is
    in the Stonefish scenario (genera_scenario.py --ostacolo X Y), for the checks on the true
    geometry: sign of the bearing and closest distance."""
    px, py = (float(v) for v in os.environ.get('OSTACOLO_XY', '3.15,2.52').split(','))
    s.avoid = True
    if not s.start_mission():
        return
    s.check(s.wait_for(lambda: s.in_sub('TrRun'), 150.0), 'TrRun reached')
    first = None
    dmin = float('inf')
    avoid_seen = False
    end = time.monotonic() + 300.0
    while time.monotonic() < end:
        s.spin_for(0.1)
        x = s.truth
        if not x:
            continue
        dmin = min(dmin, math.hypot(px - x[3], py - x[4]))
        o = s.obstacle
        if first is None and o is not None and o.valid:
            # bearing of the pole from the true pose, NED: positive to starboard
            true_b = wrap(math.atan2(py - x[4], px - x[3]) - x[2])
            first = (s.now(), o.dist, o.bearing, true_b, math.hypot(px - x[3], py - x[4]))
        if name(s.status.substate).startswith('Av'):
            avoid_seen = True
        if x[9] == 0.0 and x[1] < 0.3 and s.status.state in (READY, FAULTSAFE):
            break
    if first:
        t, d, b, tb, dc = first
        s.log('first obstacle at %.1f s: dist %.2f m (pole centre %.2f m, radius 0.3), bearing '
              '%.2f rad, true %.2f rad' % (t, d, dc, b, tb))
        s.check(b * tb > 0 or abs(tb) < 0.03, 'obstacle bearing on the right side (%.2f vs true %.2f)'
                % (b, tb))
    else:
        s.check(False, 'obstacle seen by the bridge')
    s.check(avoid_seen, 'chart went through AvoidObstacle')
    s.check(dmin > 0.3 + 0.35, 'no contact: closest distance pole centre - vehicle %.2f m' % dmin)
    s.check(s.truth[9] == 0.0 and s.truth[1] < 0.3, 'vehicle at the surface and disarmed')
    s.log('path: %s' % ' > '.join(
        name(sub) + ('(%s)' % REASON[r] if r else '') for _, _, sub, r in _merged(s.path)))


def falsi_allarmi(s):
    """completa plan with avoid_enabled and no obstacle in the scene: every obstacle the
    bridge reports is a false alarm (echoes from the seabed, the surface, noise)."""
    s.avoid = True
    if not s.start_mission():
        return
    n = n_valid = 0
    dmin = float('inf')
    first = None
    avoid_seen = False
    end = time.monotonic() + 300.0
    while time.monotonic() < end:
        s.spin_for(0.1)
        o = s.obstacle
        if o is not None and s.status.state == MISSION:
            n += 1
            if o.valid:
                n_valid += 1
                dmin = min(dmin, o.dist)
                if first is None:
                    first = (s.now(), o.dist, o.bearing, name(s.status.substate))
        if name(s.status.substate).startswith('Av'):
            avoid_seen = True
        x = s.truth
        if x and x[9] == 0.0 and x[1] < 0.3 and s.status.state in (READY, FAULTSAFE):
            break
    s.log('obstacle bus in Mission: %d samples, %d valid (closest %.2f m)%s' % (
        n, n_valid, dmin, '' if first is None else
        ', first at %.1f s: %.2f m, bearing %.2f rad, in %s' % first))
    s.check(n > 0, 'the bridge publishes the obstacle bus')
    s.check(not avoid_seen, 'no AvoidObstacle without obstacles')
    s.check(s.truth[9] == 0.0 and s.truth[1] < 0.3, 'vehicle at the surface and disarmed')


# ── GOTO with the DVL (SIM=stonefish, DVL=1) ──────────────────────────────

# HOLD 10 s at 2 m, GOTO (4, 3) m at 0.25 m/s within 60 s, HOLD 10 s; TaskType GOTO 4
GOTO_XY = (4.0, 3.0)
PLAN_GOTO = [(1, 2.0, 0.0, 0.0, 10.0, 0.0),
             (4, 2.0, 0.0, 0.25, 60.0, 0.0, GOTO_XY[0], GOTO_XY[1]),
             (1, 2.0, 0.0, 0.0, 10.0, 0.0)]


def goto(s, perdi_dvl=False):
    """The bridge estimates the position from the DVL; the chart goes to GOTO_XY (frame of the
    tare: origin where the vehicle was, axes of its heading). Checks the arrival, the true
    position there and the error of the estimate. perdi_dvl: the DVL stops during GtApproach,
    the chart must go to NavDegraded."""
    s.plan = PLAN_GOTO
    if not s.start_mission():
        return
    origin = (s.truth[3], s.truth[4]) if s.truth else (0.0, 0.0)
    s.check(s.wait_for(lambda: s.in_sub('GtApproach'), 120.0), 'GtApproach reached')
    t0 = s.now()
    if os.environ.get('CORRENTE') == '1':   # the current of the scenario (run.sh SCN=)
        s.check(s.set_params(s.sim_params, correnti_attive=True), 'current switched on')
    if perdi_dvl:
        s.spin_for(5.0)
        s.check(s.set_params(s.sim_params, dvl_attivo=False), 'DVL lost in GtApproach')
        s.check(s.wait_for(lambda: s.in_sub('NavDegraded'), 5.0),
                'NavDegraded after P_NAV_LOSS (%s)' % name(s.status.substate))
        s.check(s.wait_for(lambda: s.in_sub('TaskFailed') or s.status.state != MISSION, 60.0),
                'task failed after P_NAVDEG_TIMEOUT')
    else:
        arrived = s.wait_for(lambda: not s.in_sub('GtApproach', 'GtAlign'), 90.0)
        x = s.truth
        n = s.nav
        tx, ty = x[3] - origin[0], x[4] - origin[1]
        s.check(arrived and s.in_sub('SelectTask', 'PrepareTask', 'Hold'),
                'GOTO arrived in %.1f s (chart %s)' % (s.now() - t0, name(s.status.substate)))
        s.log('at the arrival: estimate (%.2f, %.2f) m, true (%.2f, %.2f) m from the start, '
              'target (%.1f, %.1f)' % (n.x, n.y, tx, ty, GOTO_XY[0], GOTO_XY[1]))
        s.check(math.hypot(tx - GOTO_XY[0], ty - GOTO_XY[1]) < 1.0,
                'true position within 1 m of the target (%.2f m)' % math.hypot(
                    tx - GOTO_XY[0], ty - GOTO_XY[1]))
        s.log('error of the DVL estimate: %.2f m' % math.hypot(n.x - tx, n.y - ty))
    s.check(s.wait_for(lambda: s.truth[9] == 0.0 and s.truth[1] < 0.3, 200.0),
            'vehicle at the surface and disarmed')
    s.check(s.reasons() == [], 'no abort reason (%s)' % s.reasons())
    s.log('path: %s' % ' > '.join(
        name(sub) + ('(%s)' % REASON[r] if r else '') for _, _, sub, r in _merged(s.path)))


SCENARIOS = {'completa': completa, 'batteria': batteria, 'barometro': barometro,
             'override': override}
SCENARIOS.update({f: (lambda s, f=f: guasto(s, f)) for f in FAULTS})
SCENARIOS['ostacolo'] = ostacolo
SCENARIOS['falsi_allarmi'] = falsi_allarmi
SCENARIOS['goto'] = goto
SCENARIOS['goto_dvl_perso'] = lambda s: goto(s, perdi_dvl=True)


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else 'completa'
    if which not in SCENARIOS:
        print('scenarios: %s' % ', '.join(SCENARIOS))
        sys.exit(2)
    rclpy.init()
    s = Scenario()
    s.log('scenario %s' % which)
    try:
        SCENARIOS[which](s)
    except Exception as e:   # a crash is a failure, with the path so far
        s.check(False, 'exception: %r' % e)
    print('STATES ' + ' > '.join(
        '%s/%s%s' % (TOP[st] if st < len(TOP) else st, name(sub),
                     '(%s)' % REASON[r] if r else '')
        for _, st, sub, r in _merged(s.path)), flush=True)
    print('RESULT', 'PASS' if s.ok else 'FAIL', flush=True)
    s.destroy_node()
    rclpy.try_shutdown()


def _merged(path):
    out = []
    for p in path:
        if not out or out[-1][1:] != p[1:]:
            out.append(p)
    return out


if __name__ == '__main__':
    main()
