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
from msur_msgs.srv import UpdatePid


class UpdatePidService(Node):
    types = {
            0: RollPidConfig,
            1: PitchPidConfig,
            2: YawPidConfig,
            3: DepthPidConfig,
            4: AltitudePidConfig,
            5: VelXPidConfig,
            6: VelYPidConfig
        }

    def __init__(self):
        super().__init__('pid_updator')
        self.sender = Sender(('192.168.90.1', 2030))
        self.srv = self.create_service(UpdatePid, 'pid_updator', self.update_pid)
        self.get_logger().info('pid updator is running')

    def update_pid(self, request, response):
        type_ = self.types[request.type](p=request.p, i=request.i, d=request.d)
        self.get_logger().info(f'Update pid settings for\ntype: {type_}, p:{request.p}, i:{request.i}, d:{request.d}')
        self.sender.ones(lambda: (type_,))
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    pid_service = UpdatePidService()
    rclpy.spin(pid_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
