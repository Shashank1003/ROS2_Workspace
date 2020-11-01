#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from functools import partial


class ResetCounterClientNode(Node):

    def __init__(self):
        super().__init__("reset_counter_client")
        self.call_reset_counter(True)

    def call_reset_counter(self, data):
        client = self.create_client(SetBool, "reset_counter")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server Reset Counter...")

        request = SetBool.Request()
        request.data= True

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_reset_counter_client, data=data))

    def callback_reset_counter_client(self, future, data):
        try:
            response = future.result()
            self.get_logger().info(str(response))
        except Exception as e:
            self.get_logger().error("service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = ResetCounterClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
