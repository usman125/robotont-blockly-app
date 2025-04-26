#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time, math

class RectangleController(Node):
    def __init__(self):
        super().__init__('rectangle_controller')

        # Publisher to the /turtle1/cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Rectangle Controller Node has been started.")

        # Add an initial delay to ensure turtlesim is ready
        self.get_logger().info("Waiting for turtlesim to be ready...")
        time.sleep(2.0)  # Wait for 2 seconds to let turtlesim initialize

        # Draw the rectangle
        self.draw_rectangle()

    def draw_rectangle(self):
        """
        Function to make the turtle draw a rectangle.
        """
        # Rectangle parameters
        forward_speed = 1.0   # Linear speed (m/s)
        turn_speed = 4.0      # Angular speed (rad/s)
        short_side_duration = 2.0  # Time to move along short side (seconds)
        long_side_duration = 4.0   # Time to move along long side (seconds)
        turn_duration = 1.629 / turn_speed  # Time to turn 90 degrees (1.57 radians)

        # Create Twist message for forward motion
        move_forward = Twist()
        move_forward.linear.x = forward_speed
        move_forward.angular.z = 0.0

        # Create Twist message for turning
        turn = Twist()
        turn.linear.x = 0.0
        turn.angular.z = turn_speed

        for _ in range(2):  # Loop twice to complete the rectangle
            # Move forward along the long side
            self.get_logger().info("Moving along the long side...")
            self.cmd_vel_publisher.publish(move_forward)
            time.sleep(long_side_duration)

            # Turn 90 degrees
            self.get_logger().info("Turning 90 degrees...")
            self.cmd_vel_publisher.publish(turn)
            time.sleep(turn_duration)

            # Move forward along the short side
            self.get_logger().info("Moving along the short side...")
            self.cmd_vel_publisher.publish(move_forward)
            time.sleep(short_side_duration)

            # Turn 90 degrees
            self.get_logger().info("Turning 90 degrees...")
            self.cmd_vel_publisher.publish(turn)
            time.sleep(turn_duration)

        # Stop the turtle after drawing the rectangle
        self.get_logger().info("Stopping the turtle...")
        stop = Twist()
        self.cmd_vel_publisher.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = RectangleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()