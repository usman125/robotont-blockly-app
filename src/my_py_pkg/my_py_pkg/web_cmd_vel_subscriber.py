#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

#ros2 launch rosbridge_server rosbridge_websocket_launch.xml

class WebCmdVelSubscriber(Node):

    def __init__(self):
        super().__init__("web_cmd_vel_subscriber")
        self.subsciber_ = self.create_subscription(
            Twist, "cmd_vel", self.callback_robot_news, 10)
        self.get_logger().info("Web cmd vel subscriber started now")

    def callback_robot_news(self, msg: Twist):
        self.get_logger().info(msg.linear.x)

def main(args=None):
    rclpy.init(args=args) 
    node = WebCmdVelSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()