import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from protocol.communication import Sender
from protocol.tests import TestReceiver
from msur_msgs.msg import DevConfig, DevError, PidsStatus, Leaks
from protocol.model import *
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Header
from rclpy.action import ActionServer
import numpy as np
from msur_msgs.srv import WritePid


class WritePidService(Node):

    def __init__(self):
        super().__init__('pid_writer')
        self.sender = Sender(('192.168.90.1', 2030))
        self.srv = self.create_service(WritePid, 'pid_writer', self.update_pid)
        self.get_logger().info('pid writer is running')

    def update_pid(self, request, response):
        self.get_logger().info(f'Write pid parameters in rom')
        self.sender.ones(lambda: (RebootConfig(write_pid=True),))
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    pid_service = WritePidService()
    rclpy.spin(pid_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
