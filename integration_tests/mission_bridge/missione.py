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

Faults are injected through parameters: banco.batteria_pct of the bridge (bench
battery), barometro_attivo and yaw_iniziale of sim_dinamica.
"""
import math
import sys
import time

import rclpy
from nereo_interfaces.msg import CommandVelocity
from nereo_mission_msgs.msg import Cmd, MissionStatusBus, PlanBus
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
          'DISARM_TIMEOUT', 'SHUTDOWN']
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


def param(name_, value):
    p = Parameter(name=name_)
    if isinstance(value, bool):
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
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
        self.create_subscription(UInt8, '/modalita_guida',
                                 lambda m: setattr(self, 'mode', m.data), LATCHED)
        self.create_subscription(MissionStatusBus, 'missione/mission_status', self._status, 10)
        self.create_subscription(Float64MultiArray, '/sim_dinamica/stato',
                                 lambda m: setattr(self, 'truth', list(m.data)), 10)
        self.create_subscription(Float32, '/barometer_depth',
                                 lambda m: setattr(self, 'depth_meas', m.data), BEST_EFFORT)
        self.create_subscription(Float64MultiArray, '/controller/setpoints',
                                 lambda m: setattr(self, 'setpoints', list(m.data)), 10)
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
        plan = PlanBus(valid=True, len=len(PLAN), max_depth=4.0, max_time=900.0)
        for i, (typ, depth, heading, speed, duration, altitude) in enumerate(PLAN):
            plan.type[i] = typ
            plan.depth[i] = depth
            plan.heading[i] = heading
            plan.speed[i] = speed
            plan.duration[i] = duration
            plan.altitude[i] = altitude
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
                lambda: self.truth and abs(wrap(self.truth[2] - yaw)) < 1e-3, 2.0)
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
    dev = max((abs(wrap(y - YAW_START)) for y in descent), default=float('inf'))
    s.check(len(descent) > 10 and dev < 0.05,
            'descent: yaw within 0.05 rad of %.2f (max deviation %.3f rad, %d samples)' % (
                YAW_START, dev, len(descent)))
    depth = s.track('Hold', 60.0, lambda x: x[1])
    s.check(depth and max(abs(d - 2.0) for d in depth[2:]) < 0.2,
            'HOLD: depth within 0.2 m of 2.0 (%.2f .. %.2f m)' % (
                min(depth or [0]), max(depth or [0])))
    # the bridge maps DEPTH_HOLD to setpoint.heading: the turn happens in TrAlign
    yaw_in = s.truth[2] if s.wait_for(lambda: s.in_sub('TrAlign'), 60.0) and s.truth else None
    align = s.track('TrAlign', 1.0, lambda x: x[2])
    if yaw_in is not None and align:
        s.check(abs(yaw_in) < 0.1 and abs(align[-1] - HEADING) < 0.1,
                'TrAlign: yaw from %.2f to %.2f rad (plan %.2f), turned before TrRun' % (
                    yaw_in, align[-1], HEADING))
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


SCENARIOS = {'completa': completa, 'batteria': batteria, 'barometro': barometro,
             'override': override}


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
