import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from protocol.communication import Receiver
from protocol.tests import TestReceiver
from msur_msgs.msg import DevConfig, DevError, PidsStatus, Leaks
from protocol.model import Telemetry
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Header
import numpy as np


def to_quaternion(pitch, yaw, roll) -> list:
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2)
    return [qx, qy, qz, qw]


class ReceiverNode(Node):
    header = Header()
    header.frame_id = 'telemetry'

    def __init__(self):
        super().__init__('telemetry_node')
        self. receiver = TestReceiver(('127.0.0.1', 2065))
        self.receiver.logger = self.get_logger()
        self.timer = Time()

        # self.position_publisher = self.create_publisher(TelemetryPosition, 'telemetry/position', 10)

        self.config_publisher = self.create_publisher(DevConfig, 'telemetry/devices', 10)
        self.leaks_publisher = self.create_publisher(Leaks, 'telemetry/leaks', 10)
        self.pid_publisher = self.create_publisher(PidsStatus, 'telemetry/pid', 10)
        self.errors_publisher = self.create_publisher(DevError, 'telemetry/errors', 10)
        self.battery_publisher = self.create_publisher(BatteryState, 'telemetry/battery', 10)
        self.imu_publisher = self.create_publisher(Imu, 'telemetry/orientation', 10)

        self.i = 0
        self.get_logger().info('Running telemetry node')
        self.receiver.loop(self.callback)

    def callback(self, telemetry: Telemetry):
        if telemetry is None:
            return
        self.header.stamp = self.get_clock().now().to_msg()

        self.imu_publisher.publish(Imu(header=self.header, angular_velocity=telemetry.get_vector(Vector3)))
        self.battery_publisher.publish(BatteryState(voltage=telemetry.voltage, current=telemetry.current, present=True))
        self.errors_publisher.publish(
            DevError(pressure=telemetry.device_error.pressure_sensor, imu=telemetry.device_error.navigation_module))
        self.pid_publisher.publish(PidsStatus(roll=telemetry.pid_stat.roll, pitch=telemetry.pid_stat.pitch,
                                              yaw=telemetry.pid_stat.yaw, depth=telemetry.pid_stat.depth,
                                              altitude=telemetry.pid_stat.altitude, spd_x=telemetry.pid_stat.speed_x,
                                              spd_y=telemetry.pid_stat.speed_y))
        self.leaks_publisher.publish(Leaks(main=telemetry.leak.main, imu=telemetry.leak.navigation))
        self.config_publisher.publish(DevConfig(em_1=telemetry.devices_stat.em_1, em_2=telemetry.devices_stat.em_2))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = ReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
