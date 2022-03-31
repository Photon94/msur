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
from msur_msgs.srv import Reboot


class RebootService(Node):
    types = {
            0: RebootConfig(stm=True),
            1: RebootConfig(pc=True),
            2: RebootConfig(hudro=True),
        }

    def __init__(self):
        super().__init__('reboot_service')
        self.sender = Sender(('192.168.90.1', 2030))
        self.srv = self.create_service(Reboot, 'reboot_service', self.reboot)
        self.get_logger().info('reboot service is running')

    def reboot(self, request, response):
        type_ = self.types[request.device]
        self.get_logger().info(f'Reboot: {type_}')
        self.sender.ones(lambda: (type_,))
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    reboot_service = RebootService()
    rclpy.spin(reboot_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
