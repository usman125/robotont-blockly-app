#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import numpy as np

class PerfectCircleController(Node):
    def __init__(self):
        super().__init__("perfect_circle_controller")
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Circle parameters (adjustable)
        self.target_radius = 0.5  # meters
        self.linear_speed = 0.5    # m/s
        self.angular_speed = self.linear_speed / self.target_radius  # rad/s
        
        # Control parameters for precision
        self.correction_gain = 0.15  # Tune this (higher = more aggressive correction)
        self.position_error = 0.0
        self.prev_error = 0.0
        
        # Motion timing
        self.circle_duration = (2 * math.pi / self.angular_speed)   # Full rotation time
        self.start_time = self.get_clock().now()
        
        # Start control loop
        self.timer = self.create_timer(0.05, self.update_motion)  # 20Hz control loop

    def update_motion(self):
        # Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        if elapsed_time >= self.circle_duration:
            self.stop_robot()
            self.get_logger().info("Circle completed! Shutting down...")
            self.timer.cancel()
            self.destroy_node()
            return
        
        # Simulate position error (replace with real odometry in practice)
        self.position_error += (np.random.randn() * 0.001)  # Simulate 1mm noise
        
        # Apply corrective angular velocity
        corrective_angular = self.correction_gain * self.position_error
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed + corrective_angular
        
        self.cmd_vel_pub.publish(twist)
        self.prev_error = self.position_error

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())  # Zero motion

def main(args=None):
    rclpy.init(args=args)
    node = PerfectCircleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()