"""
Minimal vehicle simulator for the mission bridge integration test.

Stands in for the Raspberry Pi sensors and the flight-controller firmware:
  - /barometer_depth, /imu_data (best effort, like the Pi publishers),
    /barometer_diagnostic, /imu_diagnostic
  - firmware: /set_arm_mode -> /rov_armed at 40 Hz
  - tare services /barometer_reset_reference, /imu_reset_reference
  - counts the commands reaching the firmware on /nereo_cmd_vel
Depth is fixed at 0 (surface, after the tare with mounting_offset_m = 0).
"""
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nereo_interfaces.msg import CommandVelocity
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class FakeVehicle(Node):
    def __init__(self):
        super().__init__('fake_vehicle')
        self.armed = False
        self.cmd_count = 0
        self.p_depth = self.create_publisher(Float32, '/barometer_depth', BEST_EFFORT)
        self.p_imu = self.create_publisher(Imu, '/imu_data', BEST_EFFORT)
        self.p_diag_bar = self.create_publisher(DiagnosticArray, '/barometer_diagnostic', BEST_EFFORT)
        self.p_diag_imu = self.create_publisher(DiagnosticArray, '/imu_diagnostic', BEST_EFFORT)
        self.p_armed = self.create_publisher(Bool, '/rov_armed', BEST_EFFORT)
        self.create_subscription(Bool, '/set_arm_mode', self._arm, 10)
        self.create_subscription(CommandVelocity, '/nereo_cmd_vel', self._cmd, 10)
        self.create_service(Trigger, '/barometer_reset_reference', self._tare('barometer'))
        self.create_service(Trigger, '/imu_reset_reference', self._tare('IMU'))
        self.create_timer(0.3, lambda: self.p_depth.publish(Float32(data=0.0)))
        self.create_timer(0.2, self._imu)
        self.create_timer(1.0, self._diag)
        self.create_timer(0.025, lambda: self.p_armed.publish(Bool(data=self.armed)))

    def _imu(self):
        m = Imu()
        m.orientation.w = 1.0
        self.p_imu.publish(m)

    def _diag(self):
        for p in (self.p_diag_bar, self.p_diag_imu):
            a = DiagnosticArray()
            a.status = [DiagnosticStatus(level=DiagnosticStatus.OK)]
            p.publish(a)

    def _arm(self, msg):
        self.armed = msg.data
        self.get_logger().info('FIRMWARE: armed=%s' % msg.data)

    def _cmd(self, _msg):
        self.cmd_count += 1

    def _tare(self, name):
        def cb(_req, res):
            self.get_logger().info('TARE %s' % name)
            res.success = True
            return res
        return cb


def main():
    rclpy.init()
    node = FakeVehicle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
