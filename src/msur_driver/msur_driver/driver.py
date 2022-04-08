import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from protocol.communication import Receiver, Sender
from protocol.tests import TestReceiver
from msur_msgs.msg import DevConfig, DevError, PidsStatus, Leaks, UpdatePid, Reboot
from geometry_msgs.msg import PoseStamped, Vector3, Twist
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Header, Bool
import numpy as np
from protocol.model import *


def to_quaternion(pitch, yaw, roll) -> list:
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2)
    return [qx, qy, qz, qw]


class Driver(Node):
    header = Header()
    header.frame_id = 'driver'
    pid_types = {
        0: RollPidConfig,
        1: PitchPidConfig,
        2: YawPidConfig,
        3: DepthPidConfig,
        4: AltitudePidConfig,
        5: VelXPidConfig,
        6: VelYPidConfig
    }
    reboot_types = {
        0: RebootConfig(stm=True),
        1: RebootConfig(pc=True),
        2: RebootConfig(hudro=True),
    }

    def __init__(self):
        super().__init__('driver')
        self. receiver = TestReceiver(('127.0.0.1', 2065))
        # self.receiver = Receiver(('192.168.90.77', 2065))
        self.receiver.logger = self.get_logger()
        self.timer = Time()
        self.send_buffer = []

        # self.position_publisher = self.create_publisher(TelemetryPosition, 'telemetry/position', 10)

        self.config_publisher = self.create_publisher(DevConfig, 'telemetry/devices', 10)
        self.leaks_publisher = self.create_publisher(Leaks, 'telemetry/leaks', 10)
        self.pid_publisher = self.create_publisher(PidsStatus, 'telemetry/pid', 10)
        self.errors_publisher = self.create_publisher(DevError, 'telemetry/errors', 10)
        self.battery_publisher = self.create_publisher(BatteryState, 'telemetry/battery', 10)
        self.imu_publisher = self.create_publisher(Imu, 'telemetry/orientation', 10)

        self.i = 0

        self.sender = Sender(('192.168.90.1', 2030))
        self.pid_subscriber = self.create_subscription(UpdatePid, 'config/pid', self.update_pid, 10)
        self.rewrite = self.create_subscription(Bool, 'config/rewrite', self.rewrite, 10)
        self.reboot_subscriber = self.create_subscription(Reboot, 'config/reboot', self.reboot_device, 10)
        self.motion = self.create_subscription(Twist, 'cmd_vel', self.motion, 10)

        self.get_logger().info('Running driver node')
        self.timer = self.create_timer(0.1, self.driver_loop)

    def driver_loop(self):
        self.receiver.ones(self.publish_telemetry)
        self.send_data()
        self.send_buffer = []

    def publish_telemetry(self, telemetry: Telemetry):
        if telemetry is None:
            return
        self.header.stamp = self.get_clock().now().to_msg()

        self.imu_publisher.publish(Imu(header=self.header, angular_velocity=telemetry.get_vector(Vector3)))
        self.battery_publisher.publish(BatteryState(voltage=telemetry.voltage, current=telemetry.current, present=True))
        self.errors_publisher.publish(DevError(**telemetry.device_error.dict()))
        self.pid_publisher.publish(PidsStatus(**telemetry.pid_stat.dict()))
        self.leaks_publisher.publish(Leaks(**telemetry.leak.dict()))
        self.config_publisher.publish(DevConfig(**telemetry.devices_stat.dict()))
        self.i += 1

    def send_data(self):
        deprecated = []
        for item in reversed(self.send_buffer):
            if item.__class__ in deprecated:
                continue
            deprecated.append(item.__class__)
        self.sender.ones(lambda: self.send_buffer)

    def update_pid(self, msg: UpdatePid):
        pid = self.pid_types[msg.type](p=msg.p, i=msg.i, d=msg.d)
        self.get_logger().info(f'Update pid settings for\ntype: {pid}, p:{msg.p}, i:{msg.i}, d:{msg.d}')
        self.send_buffer.append(pid)

    def reboot_device(self, msg: Reboot):
        reboot = self.reboot_types[msg.device]
        self.get_logger().info(f'Reboot: {reboot}')
        self.send_buffer.append(reboot)

    def rewrite(self, msg: Bool):
        if msg.data:
            self.get_logger().info(f'Write pid parameters in rom')
            self.send_buffer.append(WriteConfig(pid=True))

    def motion(self, msg: Twist):
        self.get_logger().info(f'Send motion data')
        self.send_buffer += [XThrust(value=msg.linear.x), YThrust(value=msg.linear.y), ZThrust(value=msg.linear.z)]


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
