"""
Safety arbitration node — merges commands from the physical controller and
the web interface, enforces priority and timeout-based stop, and owns the
guidance mode (MANUAL / AUTONOMOUS) of the thesis mission software.

Topic layout:

  Physical controller (joy_to_cmdvel):
    /nereo_cmd_vel_joy      → direct mode  (priority 1 — highest)

  Controller node (nereo_controller_node, closed-loop):
    /nereo_cmd_vel_ctrl     → controller mode (priority 1 — highest)

  Web interface:
    /web_cmd_vel            → web joystick  (priority 2)

  Output (to ROV firmware — microROS subscribes to this):
    /nereo_cmd_vel          → arbitrated, timeout-guarded command

Guidance mode (published latched on /modalita_guida, std_msgs/UInt8:
0 MANUAL, 1 AUTONOMOUS; rules in mode_selector.py):

  MANUAL      pilot side only: joystick (direct and through the controller),
              web joystick, arm requests from joystick, GUI and web
  AUTONOMOUS  mission bridge (nereo_mission_bridge) only: controller input
              and arm requests; joystick and web commands are dropped

  The safety node is the single publisher of the controller input and of the
  arm requests to the firmware, forwarding only the source of the mode:
    /nereo_cmd_vel_no_fb_manual (joystick) | /nereo_cmd_vel_no_fb_auto (bridge)
        → /nereo_cmd_vel_no_fb  (input of nereo_controller_node)
    /set_arm_mode_manual (joystick, GUI, web) | /set_arm_mode_auto (bridge)
        → /set_arm_mode         (firmware)
  The controller output /nereo_cmd_vel_ctrl passes in both modes: in
  AUTONOMOUS the controller is fed only by the bridge.

  Start: MANUAL. Override: joystick button `override_button`, or the service
  /modalita_guida/richiedi_manuale, always back to MANUAL. AUTONOMOUS only on
  request (service /modalita_guida/richiedi_autonoma, or the optional
  `autonomous_button`) and only if the mission state (`mission_state_topic`,
  std_msgs/UInt8 from the bridge) is fresh and not Mission or Abort.

Note: /nereo_cmd_vel_no_fb is the input of nereo_controller_node, which then
republishes the closed-loop command on /nereo_cmd_vel_ctrl. The safety node
remains the single publisher of /nereo_cmd_vel.

Priority rules:
  - If the physical controller has sent a command within CONTROLLER_TIMEOUT,
    it wins — web commands are ignored.
  - If only the web is active, its commands are forwarded.
  - If no source has sent a command within its respective timeout, zeros are sent.
  - On web disconnect (no message for WEB_HOLD_SECS), hold the last command
    for WEB_HOLD_SECS then ramp to zero over RAMP_SECS.
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nereo_interfaces.msg import CommandVelocity
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, UInt8
from std_srvs.srv import Trigger

from web_pkg.mode_selector import (
    ModeSelector, SOURCE_BRIDGE, SOURCE_PILOT, mode_name)

CONTROLLER_TIMEOUT = 0.5   # s — controller considered lost after this
WEB_HOLD_SECS      = 1.0   # s — hold last web command before stopping
RAMP_SECS          = 0.5   # s — ramp down to zero after hold expires
PUBLISH_RATE       = 20    # Hz
WEB_ACTIVE_WINDOW  = 2.0   # s — web considered "recently active" within this window


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self._controller_cmd  = CommandVelocity()
        self._web_cmd         = CommandVelocity()
        self._last_controller = None
        self._last_web        = None
        self._web_hold_start  = None   # when the web timeout began

        self._pub = self.create_publisher(
            CommandVelocity, '/nereo_cmd_vel', 10)

        # controller sources — both treated as highest priority:
        #   - /nereo_cmd_vel_joy:  raw joystick passthrough (DIRECT mode),
        #                          pilot side: dropped in AUTONOMOUS
        #   - /nereo_cmd_vel_ctrl: closed-loop output from nereo_controller_node
        self.create_subscription(
            CommandVelocity, '/nereo_cmd_vel_joy',
            lambda m: self._on_controller(m, SOURCE_PILOT), 10)
        self.create_subscription(
            CommandVelocity, '/nereo_cmd_vel_ctrl',
            lambda m: self._on_controller(m, None), 10)

        # ── guidance mode ────────────────────────────────────────────────
        self.declare_parameter('override_button', 1)      # Xbox B / DS5 Circle
        self.declare_parameter('autonomous_button', -1)   # -1: service only
        self.declare_parameter('mission_state_topic', '/missione/stato')
        self.declare_parameter('mission_state_timeout', 2.0)
        self._override_button = self.get_parameter('override_button').value
        self._autonomous_button = self.get_parameter('autonomous_button').value
        if self._autonomous_button >= 0 and self._autonomous_button == self._override_button:
            self.get_logger().error(
                'autonomous_button equals override_button: autonomous button disabled')
            self._autonomous_button = -1
        self._selector = ModeSelector(
            self.get_parameter('mission_state_timeout').value)

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._mode_pub = self.create_publisher(UInt8, '/modalita_guida', latched)
        self.create_subscription(
            UInt8, self.get_parameter('mission_state_topic').value,
            self._on_mission_state, 10)
        self.create_subscription(Joy, 'joy', self._on_joy, best_effort)
        self.create_service(
            Trigger, '/modalita_guida/richiedi_autonoma', self._srv_autonomous)
        self.create_service(
            Trigger, '/modalita_guida/richiedi_manuale', self._srv_manual)

        # controller input and arm requests: forwarded only from the source of
        # the current mode
        self._ctrl_in_pub = self.create_publisher(
            CommandVelocity, '/nereo_cmd_vel_no_fb', 10)
        self._arm_pub = self.create_publisher(Bool, '/set_arm_mode', 10)
        self.create_subscription(
            CommandVelocity, '/nereo_cmd_vel_no_fb_manual',
            lambda m: self._forward(self._ctrl_in_pub, m, SOURCE_PILOT), 10)
        self.create_subscription(
            CommandVelocity, '/nereo_cmd_vel_no_fb_auto',
            lambda m: self._forward(self._ctrl_in_pub, m, SOURCE_BRIDGE), 10)
        self.create_subscription(
            Bool, '/set_arm_mode_manual',
            lambda m: self._forward_arm(m, SOURCE_PILOT), 10)
        self.create_subscription(
            Bool, '/set_arm_mode_auto',
            lambda m: self._forward_arm(m, SOURCE_BRIDGE), 10)
        self._publish_mode()

        # web source
        self.create_subscription(
            CommandVelocity, '/web_cmd_vel',
            self._on_web, 10)

        self.create_timer(1.0 / PUBLISH_RATE, self._publish)
        self.get_logger().info('Safety node active — publishing on /nereo_cmd_vel (firmware topic)')
        self.get_logger().info(
            'Guidance mode MANUAL; override button %d, autonomous button %s' % (
                self._override_button,
                self._autonomous_button if self._autonomous_button >= 0 else 'none (service only)'))

    # ── guidance mode ─────────────────────────────────────────────────────

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_mode(self):
        self._mode_pub.publish(UInt8(data=self._selector.mode))

    def _mode_changed(self, why):
        # commands of the previous mode must not leak into the new one
        self._last_controller = None
        self._last_web = None
        self._web_hold_start = None
        self._publish_mode()
        self.get_logger().warning(
            'Guidance mode %s (%s)' % (mode_name(self._selector.mode), why))

    def _on_mission_state(self, msg: UInt8):
        self._selector.set_mission_state(msg.data, self._now_s())

    def _on_joy(self, msg: Joy):
        before = self._selector.mode
        for event, ok, reason in self._selector.on_buttons(
                msg.buttons, self._override_button, self._autonomous_button, self._now_s()):
            if event == 'autonomous' and not ok:
                self.get_logger().warning('AUTONOMOUS request refused: %s' % reason)
        if self._selector.mode != before:
            self._mode_changed('joystick button')

    def _srv_autonomous(self, _request, response):
        before = self._selector.mode
        response.success, response.message = self._selector.request_autonomous(self._now_s())
        if self._selector.mode != before:
            self._mode_changed('service request')
        elif not response.success:
            self.get_logger().warning('AUTONOMOUS request refused: %s' % response.message)
        return response

    def _srv_manual(self, _request, response):
        if self._selector.override():
            self._mode_changed('service request')
        response.success = True
        response.message = 'MANUAL'
        return response

    def _forward(self, pub, msg, source):
        if self._selector.accepts(source):
            pub.publish(msg)

    def _forward_arm(self, msg: Bool, source):
        if self._selector.accepts(source):
            self._arm_pub.publish(msg)
        else:
            self.get_logger().warning(
                'Arm request from %s dropped in %s mode' % (
                    source, mode_name(self._selector.mode)),
                throttle_duration_sec=2.0)

    # ── callbacks ─────────────────────────────────────────────────────────

    def _on_controller(self, msg: CommandVelocity, source):
        if source is not None and not self._selector.accepts(source):
            return
        self._controller_cmd = msg
        self._last_controller = self.get_clock().now()

    def _on_web(self, msg: CommandVelocity):
        if not self._selector.accepts(SOURCE_PILOT):
            return
        self._web_cmd = msg
        self._last_web = self.get_clock().now()
        self._web_hold_start = None

    # ── arbitration ───────────────────────────────────────────────────────

    def _publish(self):
        now = self.get_clock().now()
        out = CommandVelocity()

        controller_alive = (
            self._last_controller is not None and
            (now - self._last_controller) < Duration(seconds=CONTROLLER_TIMEOUT)
        )

        if controller_alive:
            # controller has priority — pass through directly
            out = self._controller_cmd
            self._web_hold_start = None  # reset web hold while controller active
            # if web is actively publishing while controller is alive, keep
            # _last_web fresh so handover is immediate when controller drops
            if self._last_web is not None:
                web_age_now = (now - self._last_web).nanoseconds / 1e9
                if web_age_now < WEB_ACTIVE_WINDOW:
                    self._last_web = now   # pretend web just sent a message

        else:
            # no controller — use web with hold + ramp-down
            web_age = (
                (now - self._last_web).nanoseconds / 1e9
                if self._last_web is not None else float('inf')
            )

            if web_age < WEB_HOLD_SECS:
                # web is fresh or within hold window
                out = self._web_cmd
                self._web_hold_start = None

            elif web_age < WEB_HOLD_SECS + RAMP_SECS:
                # ramp down — linearly interpolate toward zero
                if self._web_hold_start is None:
                    self._web_hold_start = now
                ramp_elapsed = (now - self._web_hold_start).nanoseconds / 1e9
                scale = max(0.0, 1.0 - ramp_elapsed / RAMP_SECS)
                out = self._scale_cmd(self._web_cmd, scale)

            else:
                # fully timed out — send zeros
                if web_age < float('inf'):
                    self.get_logger().warning(
                        'Web client timed out — sending zero command', once=True)

        self._pub.publish(out)

    @staticmethod
    def _scale_cmd(msg: CommandVelocity, scale: float) -> CommandVelocity:
        out = CommandVelocity()
        for i in range(6):
            out.cmd_vel[i] = msg.cmd_vel[i] * scale
        return out


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
