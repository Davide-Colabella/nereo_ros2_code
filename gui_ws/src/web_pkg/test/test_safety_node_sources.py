"""
Node-level test of safety_node: which pilot sources reach the firmware and the
controller in each guidance mode.

The GUI (gui_node) and the web page (index.html, through rosbridge) command the
vehicle on two topics: /web_cmd_vel (web joystick) and /set_arm_mode_manual
(arm button of both). In AUTONOMOUS neither may reach /nereo_cmd_vel or
/set_arm_mode; the joystick neither; the bridge does. Runs safety_node in
process on its own ROS domain.
"""
import pathlib
import random
import re
import time

import pytest
import rclpy
from nereo_interfaces.msg import CommandVelocity
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, UInt8
from std_srvs.srv import Trigger

from web_pkg.safety_node import SafetyNode

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
READY = 1
SURGE = 0.4


class Harness(Node):
    """Plays GUI, web, joystick and bridge, and records what safety_node forwards."""

    def __init__(self):
        super().__init__('safety_harness')
        self.mode = None
        self.fw = []          # /nereo_cmd_vel
        self.arm = []         # /set_arm_mode
        self.ctrl_in = []     # /nereo_cmd_vel_no_fb
        self.create_subscription(UInt8, '/modalita_guida',
                                 lambda m: setattr(self, 'mode', m.data), LATCHED)
        self.create_subscription(CommandVelocity, '/nereo_cmd_vel',
                                 lambda m: self.fw.append(list(m.cmd_vel)), 50)
        self.create_subscription(Bool, '/set_arm_mode', lambda m: self.arm.append(m.data), 50)
        self.create_subscription(CommandVelocity, '/nereo_cmd_vel_no_fb',
                                 lambda m: self.ctrl_in.append(list(m.cmd_vel)), 50)
        self.web = self.create_publisher(CommandVelocity, '/web_cmd_vel', 10)
        self.arm_pilot = self.create_publisher(Bool, '/set_arm_mode_manual', 10)
        self.joy_direct = self.create_publisher(CommandVelocity, '/nereo_cmd_vel_joy', 10)
        self.joy_ctrl = self.create_publisher(CommandVelocity, '/nereo_cmd_vel_no_fb_manual', 10)
        self.joy = self.create_publisher(Joy, 'joy', 10)
        self.bridge_ctrl = self.create_publisher(CommandVelocity, '/nereo_cmd_vel_no_fb_auto', 10)
        self.bridge_arm = self.create_publisher(Bool, '/set_arm_mode_auto', 10)
        self.mission_state = self.create_publisher(UInt8, '/missione/stato', 10)
        self.auto_srv = self.create_client(Trigger, '/modalita_guida/richiedi_autonoma')

    def clear(self):
        self.fw.clear()
        self.arm.clear()
        self.ctrl_in.clear()


@pytest.fixture
def ros():
    # own ROS domain, so a running stack does not interfere
    rclpy.init(domain_id=random.randint(150, 230))
    safety = SafetyNode()
    harness = Harness()
    executor = SingleThreadedExecutor()
    executor.add_node(safety)
    executor.add_node(harness)

    def spin(seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
            if until is not None and until():
                return True
        return until is None

    yield harness, spin
    executor.shutdown()
    safety.destroy_node()
    harness.destroy_node()
    rclpy.try_shutdown()


def publish_for(spin, seconds, *pubs_msgs):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for pub, msg in pubs_msgs:
            pub.publish(msg)
        spin(0.05)


def go_autonomous(h, spin):
    assert spin(5.0, lambda: h.auto_srv.service_is_ready())
    publish_for(spin, 0.3, (h.mission_state, UInt8(data=READY)))
    future = h.auto_srv.call_async(Trigger.Request())
    assert spin(5.0, future.done)
    assert future.result().success, future.result().message
    assert spin(2.0, lambda: h.mode == 1)


def surge(cmds):
    return [c[0] for c in cmds]


def test_manual_forwards_gui_and_web(ros):
    h, spin = ros
    assert spin(5.0, lambda: h.mode == 0)
    h.clear()
    publish_for(spin, 0.5, (h.web, CommandVelocity(cmd_vel=[SURGE, 0, 0, 0, 0, 0])),
                (h.arm_pilot, Bool(data=True)))
    assert any(abs(s - SURGE) < 1e-6 for s in surge(h.fw)), 'web joystick not forwarded in MANUAL'
    assert True in h.arm, 'GUI/web arm request not forwarded in MANUAL'


def test_autonomous_blocks_gui_web_and_joystick(ros):
    h, spin = ros
    assert spin(5.0, lambda: h.mode == 0)
    # web active right before the switch: its hold must not leak into AUTONOMOUS
    publish_for(spin, 0.3, (h.web, CommandVelocity(cmd_vel=[SURGE, 0, 0, 0, 0, 0])))
    go_autonomous(h, spin)
    spin(0.1)
    h.clear()
    publish_for(spin, 1.0,
                (h.web, CommandVelocity(cmd_vel=[SURGE, 0, 0, 0, 0, 0])),
                (h.arm_pilot, Bool(data=False)),
                (h.joy_direct, CommandVelocity(cmd_vel=[0.9, 0, 0, 0, 0, 0])),
                (h.joy_ctrl, CommandVelocity(cmd_vel=[0.7, 0, 0, 0, 0, 0])))
    assert h.fw, 'safety_node stopped publishing /nereo_cmd_vel'
    assert all(abs(s) < 1e-9 for s in surge(h.fw)), (
        'pilot command reached the firmware in AUTONOMOUS: %s' % sorted(set(surge(h.fw))))
    assert h.arm == [], 'GUI/web arm request reached the firmware in AUTONOMOUS'
    assert h.ctrl_in == [], 'joystick reached the controller input in AUTONOMOUS'


def test_autonomous_forwards_the_bridge(ros):
    h, spin = ros
    go_autonomous(h, spin)
    h.clear()
    publish_for(spin, 0.5, (h.bridge_ctrl, CommandVelocity(cmd_vel=[0.2, 0, 0, 0, 0, 0])),
                (h.bridge_arm, Bool(data=True)))
    assert any(abs(c[0] - 0.2) < 1e-6 for c in h.ctrl_in)
    assert True in h.arm


def test_override_gives_gui_and_web_back(ros):
    h, spin = ros
    go_autonomous(h, spin)
    buttons = [0] * 12
    buttons[1] = 1
    publish_for(spin, 0.2, (h.joy, Joy(buttons=buttons)))
    publish_for(spin, 0.1, (h.joy, Joy(buttons=[0] * 12)))
    assert spin(2.0, lambda: h.mode == 0)
    h.clear()
    publish_for(spin, 0.5, (h.web, CommandVelocity(cmd_vel=[SURGE, 0, 0, 0, 0, 0])),
                (h.arm_pilot, Bool(data=False)), (h.bridge_arm, Bool(data=True)))
    assert any(abs(s - SURGE) < 1e-6 for s in surge(h.fw))
    assert h.arm and all(a is False for a in h.arm), 'bridge arm request forwarded in MANUAL'


# ── the GUI and the web page publish only on the pilot topics ─────────────

SRC = pathlib.Path(__file__).resolve().parents[2]   # gui_ws/src


# every topic named in gui_node publishers and in index.html (publishers and
# subscribers alike): none may be a topic that safety_node owns
@pytest.mark.parametrize('path, pattern', [
    ('gui_pkg/gui_pkg/gui_node.py', r"create_publisher\(\s*\w+\s*,\s*'([^']+)'"),
    ('web_pkg/web_pkg/static/index.html', r"name:\s*'([^']+)'"),
])
def test_gui_and_web_do_not_publish_firmware_topics(path, pattern):
    f = SRC / path
    if not f.exists():
        pytest.skip('%s not in this checkout' % path)
    topics = set(re.findall(pattern, f.read_text()))
    assert topics, 'no topic found in %s' % path
    forbidden = {'/nereo_cmd_vel', '/set_arm_mode', '/nereo_cmd_vel_no_fb',
                 '/nereo_cmd_vel_joy', '/nereo_cmd_vel_ctrl', '/modalita_guida'}
    assert not topics & forbidden, '%s uses %s' % (path, sorted(topics & forbidden))
