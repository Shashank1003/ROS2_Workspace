#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node):

    def __init__(self):
        super().__init__("number_counter")

        self.counter_ = 0

        self.counter_publisher_ = self.create_publisher(
            Int64, "number_count", 10)
        self.get_logger().info("Number Counter Publisher has been stated")

        self.number_subscriber_ = self.create_subscription(
            Int64, "number", self.callback_number, 10)
        self.get_logger().info("Number Counter Subscriber has been started")

        self.counter_reset_ = self.create_service(
            SetBool, "reset_counter", self.callback_reset_counter)
        self.get_logger().info("Counter Reset Service has been started")

    def callback_number(self, msg):
        self.counter_ += msg.data

        new_msg = Int64()
        new_msg.data = self.counter_
        self.counter_publisher_.publish(new_msg)
        self.get_logger().info(str(self.counter_))

    def callback_reset_counter(self, request, response):
        if request.data:
            self.counter_= 0
            response.success= True
            response.message= "Counter has been reset to zero"
        else:
            response.success= False
            response.message= "Counter  has not been reset"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
