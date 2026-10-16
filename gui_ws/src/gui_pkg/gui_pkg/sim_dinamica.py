"""
sim_dinamica: closed-loop simulator of the vehicle, for bench tests of the
mission software without the ROV.

Stands in for the flight-controller firmware and for the Raspberry Pi sensors,
on the same topics and QoS:

  in   /nereo_cmd_vel (CommandVelocity)   thruster command, as the firmware;
                                          applied only while armed, zero after
                                          cmd_timeout_s without messages
       /set_arm_mode (Bool)               arm / disarm, as the firmware
  out  /rov_armed (Bool, 40 Hz)           firmware state
       /barometer_depth (Float32, ~3.3 Hz)        depth after the tare
       /imu_data (Imu, 5 Hz)              orientation (yaw only), relative to
                                          the IMU tare
       /barometer_diagnostic, /imu_diagnostic (DiagnosticArray, 1 Hz)
       /sonar/distance (Float32), /sonar/confidence (Int32), 10 Hz: Ping1D
                                          at ping1d_angolo_rad from the vertical,
                                          seabed at fondale_m
       /sim_dinamica/stato (Float64MultiArray, 10 Hz): ground truth
                                          [t, depth, yaw, x, y, w, u, v, r, armed]
  srv  /barometer_reset_reference, /imu_reset_reference (Trigger): tare

Dynamics: gui_pkg/sim_model.py (first order per axis, values from
modello/mission_env.m of the thesis). Sensors are best effort, as on the Pi.

Faults and changes at run time, with ros2 param set /sim_dinamica <name> <value>:
  barometro_attivo, imu_attiva, sonar_attivo   false: the sensor stops publishing
  fondale_m                                    seabed depth
  yaw_iniziale                                 yaw of the vehicle [rad, world frame]: at
                                               start, and when set the vehicle is turned
                                               there at once (yaw rate 0), as a pilot
                                               would; the IMU reads it relative to its tare
"""
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nereo_interfaces.msg import CommandVelocity
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32
from std_srvs.srv import Trigger

from gui_pkg.sim_model import Model, Parameters, quaternion_from_yaw, wrap

BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
PING1D_RANGE_M = (0.3, 30.0)


class SimDinamica(Node):
    def __init__(self):
        super().__init__('sim_dinamica')
        d = Parameters()
        for name, value in (
                ('fondale_m', d.bottom_m), ('v_vert_max_m_s', d.v_vert_max_m_s),
                ('galleggiamento_m_s', d.buoyancy_m_s),
                ('v_avanzamento_max_m_s', d.v_surge_max_m_s),
                ('v_laterale_max_m_s', d.v_sway_max_m_s),
                ('imbardata_max_rad_s', d.yaw_rate_max_rad_s),
                ('tau_verticale_s', d.tau_vert_s), ('tau_avanzamento_s', d.tau_surge_s),
                ('tau_laterale_s', d.tau_sway_s), ('tau_imbardata_s', d.tau_yaw_s),
                ('profondita_iniziale_m', 0.0), ('yaw_iniziale', 0.0),
                ('ping1d_angolo_rad', 0.0), ('cmd_timeout_s', 0.5), ('passo_s', 0.02)):
            self.declare_parameter(name, value)
        for name in ('barometro_attivo', 'imu_attiva', 'sonar_attivo'):
            self.declare_parameter(name, True)
        g = lambda n: self.get_parameter(n).value   # noqa: E731
        self.model = Model(Parameters(
            bottom_m=g('fondale_m'), v_vert_max_m_s=g('v_vert_max_m_s'),
            buoyancy_m_s=g('galleggiamento_m_s'), v_surge_max_m_s=g('v_avanzamento_max_m_s'),
            v_sway_max_m_s=g('v_laterale_max_m_s'), yaw_rate_max_rad_s=g('imbardata_max_rad_s'),
            tau_vert_s=g('tau_verticale_s'), tau_surge_s=g('tau_avanzamento_s'),
            tau_sway_s=g('tau_laterale_s'), tau_yaw_s=g('tau_imbardata_s')))
        self.model.s.depth = g('profondita_iniziale_m')
        self.model.s.yaw = wrap(g('yaw_iniziale'))
        self.active = {n: g(n) for n in ('barometro_attivo', 'imu_attiva', 'sonar_attivo')}
        self.add_on_set_parameters_callback(self._on_params)

        self.armed = False
        self.cmd = [0.0] * 6
        self.cmd_t = None
        self.bar_offset = 0.0
        self.yaw_ref = 0.0
        self.t0 = time.monotonic()
        self.t_prev = self.t0

        self.p_armed = self.create_publisher(Bool, '/rov_armed', BEST_EFFORT)
        self.p_depth = self.create_publisher(Float32, '/barometer_depth', BEST_EFFORT)
        self.p_imu = self.create_publisher(Imu, '/imu_data', BEST_EFFORT)
        self.p_diag_bar = self.create_publisher(DiagnosticArray, '/barometer_diagnostic', BEST_EFFORT)
        self.p_diag_imu = self.create_publisher(DiagnosticArray, '/imu_diagnostic', BEST_EFFORT)
        self.p_dist = self.create_publisher(Float32, 'sonar/distance', BEST_EFFORT)
        self.p_conf = self.create_publisher(Int32, 'sonar/confidence', BEST_EFFORT)
        self.p_state = self.create_publisher(Float64MultiArray, '/sim_dinamica/stato', 10)
        self.create_subscription(CommandVelocity, '/nereo_cmd_vel', self._on_cmd, 10)
        self.create_subscription(Bool, '/set_arm_mode', self._on_arm, 10)
        self.create_service(Trigger, '/barometer_reset_reference', self._tare_bar)
        self.create_service(Trigger, '/imu_reset_reference', self._tare_imu)

        self.create_timer(g('passo_s'), self._step)
        self.create_timer(0.025, lambda: self.p_armed.publish(Bool(data=self.armed)))
        self.create_timer(0.3, self._barometer)
        self.create_timer(0.2, self._imu)
        self.create_timer(0.1, self._sonar)
        self.create_timer(0.1, self._state)
        self.create_timer(1.0, self._diagnostics)
        p = self.model.p
        self.get_logger().info(
            'sim_dinamica: seabed %.2f m, vertical %.2f m/s, buoyancy %.2f m/s, '
            'surge %.2f m/s, yaw %.2f rad/s' % (
                p.bottom_m, p.v_vert_max_m_s, p.buoyancy_m_s, p.v_surge_max_m_s,
                p.yaw_rate_max_rad_s))

    # ── parameters at run time ─────────────────────────────────────────────

    def _on_params(self, params):
        for prm in params:
            if prm.name in self.active:
                self.active[prm.name] = bool(prm.value)
                self.get_logger().warning('FAULT: %s = %s' % (prm.name, prm.value))
            elif prm.name == 'fondale_m':
                if prm.value <= 0.0:
                    return SetParametersResult(successful=False, reason='fondale_m > 0')
                self.model.p.bottom_m = float(prm.value)
            elif prm.name == 'yaw_iniziale':
                if not math.isfinite(prm.value):
                    return SetParametersResult(successful=False, reason='yaw_iniziale finite')
                self.model.s.yaw = wrap(float(prm.value))
                self.model.s.r = 0.0
                self.get_logger().info('yaw set to %.3f rad' % self.model.s.yaw)
        return SetParametersResult(successful=True)

    # ── firmware ───────────────────────────────────────────────────────────

    def _on_cmd(self, msg):
        self.cmd = list(msg.cmd_vel)
        self.cmd_t = time.monotonic()

    def _on_arm(self, msg):
        if msg.data != self.armed:
            self.get_logger().info('FIRMWARE: %s' % ('ARMED' if msg.data else 'DISARMED'))
        self.armed = msg.data

    def _step(self):
        now = time.monotonic()
        dt = now - self.t_prev
        self.t_prev = now
        fresh = self.cmd_t is not None and now - self.cmd_t <= self.get_parameter('cmd_timeout_s').value
        self.model.step(dt, self.cmd if fresh else [0.0] * 6, self.armed)

    # ── sensors ────────────────────────────────────────────────────────────

    def _barometer(self):
        if self.active['barometro_attivo']:
            self.p_depth.publish(Float32(data=float(self.model.s.depth - self.bar_offset)))

    def _imu(self):
        if not self.active['imu_attiva']:
            return
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'imu_link'
        w, x, y, z = quaternion_from_yaw(wrap(self.model.s.yaw - self.yaw_ref))
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = w, x, y, z
        m.angular_velocity.z = self.model.s.r
        self.p_imu.publish(m)

    def _sonar(self):
        if not self.active['sonar_attivo']:
            return
        angle = self.get_parameter('ping1d_angolo_rad').value
        dist = self.model.altitude() / max(math.cos(angle), 1e-3)
        ok = PING1D_RANGE_M[0] <= dist <= PING1D_RANGE_M[1]
        self.p_dist.publish(Float32(data=float(dist)))
        self.p_conf.publish(Int32(data=100 if ok else 0))

    def _diagnostics(self):
        for pub, key in ((self.p_diag_bar, 'barometro_attivo'), (self.p_diag_imu, 'imu_attiva')):
            if self.active[key]:
                a = DiagnosticArray()
                a.header.stamp = self.get_clock().now().to_msg()
                a.status = [DiagnosticStatus(level=DiagnosticStatus.OK, name='sim_dinamica')]
                pub.publish(a)

    def _state(self):
        s = self.model.s
        self.p_state.publish(Float64MultiArray(data=[
            time.monotonic() - self.t0, s.depth, s.yaw, s.x, s.y, s.w, s.u, s.v, s.r,
            1.0 if self.armed else 0.0]))

    def _tare_bar(self, _req, res):
        self.bar_offset = self.model.s.depth
        self.get_logger().info('TARE barometer at %.3f m' % self.bar_offset)
        res.success = True
        return res

    def _tare_imu(self, _req, res):
        self.yaw_ref = self.model.s.yaw
        self.get_logger().info('TARE IMU, yaw reference %.3f rad' % self.yaw_ref)
        res.success = True
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SimDinamica()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
