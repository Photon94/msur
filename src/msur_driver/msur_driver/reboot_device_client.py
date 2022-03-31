import sys

from msur_msgs.srv import UpdatePid
import rclpy
from rclpy.node import Node


class RebootClient(Node):

    def __init__(self):
        super().__init__('reboot_service_client')
        self.cli = self.create_client(UpdatePid, 'reboot_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = UpdatePid.Request()

    def send_request(self):
        self.req.device = int(sys.argv[1])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    reboot_client = RebootClient()
    reboot_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(reboot_client)
        if reboot_client.future.done():
            try:
                response = reboot_client.future.result()
            except Exception as e:
                reboot_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                reboot_client.get_logger().info(f'Result for reboot: {response.success}')
            break

    reboot_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
