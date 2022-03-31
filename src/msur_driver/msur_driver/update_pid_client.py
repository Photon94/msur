import sys

from msur_msgs.srv import UpdatePid
import rclpy
from rclpy.node import Node


class UpdatePidClient(Node):

    def __init__(self):
        super().__init__('pid_updator_client')
        self.cli = self.create_client(UpdatePid, 'pid_updator')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = UpdatePid.Request()
        # self.get_logger().info('pid types:\nroll:0 pitch:1 yaw:2, depth:3 altitude:4 speed x:5 speed y:6')

    def send_request(self):
        self.req.type = int(sys.argv[1])
        self.req.p = float(sys.argv[2])
        self.req.i = float(sys.argv[3])
        self.req.d = float(sys.argv[4])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    pid_updator = UpdatePidClient()
    pid_updator.send_request()

    while rclpy.ok():
        rclpy.spin_once(pid_updator)
        if pid_updator.future.done():
            try:
                response = pid_updator.future.result()
            except Exception as e:
                pid_updator.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                pid_updator.get_logger().info(f'Result of update pid: {response.success}')
            break

    pid_updator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
