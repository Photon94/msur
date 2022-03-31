import sys

from msur_msgs.srv import WritePid
import rclpy
from rclpy.node import Node


class WritePidClient(Node):

    def __init__(self):
        super().__init__('pid_writer_client')
        self.cli = self.create_client(WritePid, 'pid_writer')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = WritePid.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    pid_updator = WritePidClient()
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
                pid_updator.get_logger().info(f'Result of write pid: {response.success}')
            break

    pid_updator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
