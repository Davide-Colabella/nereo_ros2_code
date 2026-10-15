"""
Scenario of the mission bridge integration test. Each check prints PASS or
FAIL; the last line is "RESULT PASS" or "RESULT FAIL".

  1. start: guidance mode MANUAL
  2. the chart reaches Ready; AUTONOMOUS requested through the service
  3. in AUTONOMOUS a joystick command does not reach the firmware
  4. LOAD, START: arming through the bridge, the chart reaches Mission and the
     controller follows /controller/target (depth 0.5 m, roll and pitch 0)
  5. joystick override: back to MANUAL, the chart goes to Abort (OPERATOR);
     with the pilot's controller-mode input forwarded again, the controller
     gives the depth back to the parameters
  6. AUTONOMOUS requested again in Abort: refused
"""
import time

import rclpy
from nereo_interfaces.msg import CommandVelocity
from nereo_mission_msgs.msg import Cmd, MissionStatusBus, PlanBus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Float64MultiArray, UInt8
from std_srvs.srv import Trigger

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
OVERRIDE_BUTTON = 1
OPERATOR = 1   # AbortReason.OPERATOR


NODES = ('safety_node', 'nereo_controller_node', 'nereo_mission_bridge', 'nereo_mission')


def check_single_nodes(node, check, nodes=NODES, seconds=3.0):
    """Every node of the test runs once: a node left over from an earlier run
    (same domain) would publish on the same topics."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    names = [n for n, _ in node.get_node_names_and_namespaces()]
    dup = sorted({n for n in names if names.count(n) > 1})
    missing = [n for n in nodes if n not in names]
    return check(not dup and not missing, 'each node once (duplicates %s, missing %s)' % (
        dup, missing))


class Pilot(Node):
    def __init__(self):
        super().__init__('integration_pilot')
        self.t0 = time.monotonic()
        self.mode = None
        self.status = None
        self.setpoints = None
        self.fw_cmds = []
        self.depth = None
        self.ok = True
        self.create_subscription(UInt8, '/modalita_guida',
                                 lambda m: setattr(self, 'mode', m.data), LATCHED)
        self.create_subscription(MissionStatusBus, 'missione/mission_status', self._status, 10)
        self.create_subscription(Float64MultiArray, '/controller/setpoints',
                                 lambda m: setattr(self, 'setpoints', list(m.data)), 10)
        self.create_subscription(CommandVelocity, '/nereo_cmd_vel',
                                 lambda m: self.fw_cmds.append(list(m.cmd_vel)), 10)
        self.create_subscription(Float32, '/barometer_depth',
                                 lambda m: setattr(self, 'depth', m.data), BEST_EFFORT)
        self.p_plan = self.create_publisher(PlanBus, 'missione/plan', LATCHED)
        self.p_cmd = self.create_publisher(Cmd, 'missione/cmd', 10)
        self.p_joy = self.create_publisher(Joy, 'joy', 10)
        self.p_joy_direct = self.create_publisher(CommandVelocity, '/nereo_cmd_vel_joy', 10)
        self.p_joy_ctrl = self.create_publisher(CommandVelocity, '/nereo_cmd_vel_no_fb_manual', 10)
        self.auto_srv = self.create_client(Trigger, '/modalita_guida/richiedi_autonoma')

    def _status(self, m):
        if self.status is None or (m.state, m.substate) != (self.status.state, self.status.substate):
            self.log('chart: state %d substate %d abort_reason %d' % (m.state, m.substate, m.abort_reason))
        self.status = m

    def log(self, text):
        print('%6.1f %s' % (time.monotonic() - self.t0, text), flush=True)

    def check(self, cond, text):
        self.ok = self.ok and bool(cond)
        self.log(('PASS ' if cond else 'FAIL ') + text)

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

    def request_autonomous(self):
        f = self.auto_srv.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        return f.result()

    def command(self, cmd, cond, seconds=2.0):
        for _ in range(5):
            self.p_cmd.publish(Cmd(cmd=cmd))
            if self.wait_for(cond, seconds):
                return True
        return False


def main():
    rclpy.init()
    p = Pilot()

    # 1. start
    check_single_nodes(p, p.check)
    p.check(p.wait_for(lambda: p.mode is not None, 10.0) and p.mode == 0,
            'guidance mode MANUAL at start')
    p.check(p.auto_srv.wait_for_service(timeout_sec=10.0), 'selector service available')

    # 2. Ready, then AUTONOMOUS
    p.check(p.wait_for(lambda: p.status is not None and p.status.state == 1, 40.0),
            'chart in Ready')
    plan = PlanBus(valid=True, len=1, max_depth=5.0, max_time=600.0)
    plan.type[0] = 1          # HOLD
    plan.depth[0] = 2.0
    plan.duration[0] = 30.0
    p.p_plan.publish(plan)
    p.spin_for(0.5)   # mission state reaches safety_node
    r = p.request_autonomous()
    p.check(r is not None and r.success, 'AUTONOMOUS accepted in Ready: %s' % (r.message if r else 'no reply'))
    p.check(p.wait_for(lambda: p.mode == 1, 2.0), '/modalita_guida = 1 (AUTONOMOUS)')

    # 3. joystick ignored in AUTONOMOUS
    p.fw_cmds.clear()
    end = time.monotonic() + 0.6
    while time.monotonic() < end:
        p.p_joy_direct.publish(CommandVelocity(cmd_vel=[0.9, 0.0, 0.0, 0.0, 0.0, 0.0]))
        p.spin_for(0.05)
    p.spin_for(0.3)
    p.check(p.fw_cmds and all(abs(c[0] - 0.9) > 1e-3 for c in p.fw_cmds),
            'joystick command (surge 0.9) does not reach the firmware in AUTONOMOUS '
            '(%d commands seen)' % len(p.fw_cmds))

    # 4. LOAD, START, Mission
    p.check(p.command(Cmd.LOAD, lambda: p.status.substate == 5), 'LOAD: PlanLoaded')
    p.check(p.command(Cmd.START, lambda: p.status.state in (2, 3)), 'START: PreDive')
    p.check(p.wait_for(lambda: p.status.state == 3, 15.0),
            'armed through the bridge and safety_node, chart in Mission')
    p.check(p.wait_for(lambda: p.setpoints is not None and abs(p.setpoints[0] - 0.5) < 1e-6
                       and abs(p.setpoints[1]) < 1e-9 and abs(p.setpoints[2]) < 1e-9, 3.0),
            'controller follows /controller/target: setpoints %s' % (
                [round(x, 3) for x in (p.setpoints or [])]))

    # 5. override from the joystick
    buttons = [0] * 12
    buttons[OVERRIDE_BUTTON] = 1
    p.p_joy.publish(Joy(buttons=buttons))
    p.spin_for(0.1)
    p.p_joy.publish(Joy(buttons=[0] * 12))
    p.check(p.wait_for(lambda: p.mode == 0, 2.0), 'override button: /modalita_guida = 0 (MANUAL)')
    p.check(p.wait_for(lambda: p.status.state == 4, 3.0) and p.status.abort_reason == OPERATOR,
            'chart in Abort with reason OPERATOR')
    # the controller computes (and publishes its setpoints) only when it gets an
    # input: in MANUAL that is the joystick in controller mode, through safety_node.
    # Target released (mask 0): the depth goes back to the parameters, i.e. the
    # measured depth captured at the release (0 with fake_vehicle)
    back = False
    end = time.monotonic() + 3.0
    while time.monotonic() < end and not back:
        p.p_joy_ctrl.publish(CommandVelocity())
        p.spin_for(0.1)
        back = (p.setpoints is not None and p.depth is not None and
                abs(p.setpoints[0] - p.depth) < 0.1 and p.setpoints[0] != 0.5)
    p.check(back, 'pilot input forwarded in MANUAL, controller depth back to the parameters '
            '(measured %s): setpoints %s' % (
                None if p.depth is None else round(p.depth, 3),
                [round(x, 3) for x in (p.setpoints or [])]))

    # 6. AUTONOMOUS refused in Abort
    r = p.request_autonomous()
    p.check(r is not None and not r.success and p.mode == 0,
            'AUTONOMOUS refused in Abort: %s' % (r.message if r else 'no reply'))

    print('RESULT', 'PASS' if p.ok else 'FAIL', flush=True)
    p.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
