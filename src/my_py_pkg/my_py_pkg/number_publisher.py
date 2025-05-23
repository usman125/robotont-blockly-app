#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisherNode(Node):

    def __init__(self):
        super().__init__("number_publisher")
        self.counter_ = 0
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer = self.create_timer(0.5, self.publish_number)
        self.get_logger().info("Number publisher has started")

    def publish_number(self):
        counter = Int64()
        self.counter_ += 1
        counter.data = self.counter_
        self.publisher_.publish(counter)

def main(args=None):
    rclpy.init(args=args) 
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()