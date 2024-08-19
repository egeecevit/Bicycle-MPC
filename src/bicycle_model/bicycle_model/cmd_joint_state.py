#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class BicycleControl(Node):

    def __init__(self):
        super().__init__('cmd_joint_state')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Define joint names
        self.joint_state = JointState()
        self.joint_state.name = ['rear_wheel_joint', 'front_wheel_joint', 'steering_joint']
        self.joint_state.position = [0.0, 0.0, 0.0]  # Initial positions
        self.joint_state.velocity = [0.0, 0.0, 0.0]  # Initial velocities
        self.joint_state.effort = [0.0, 0.0, 0.0]  # Initial efforts

        self.get_logger().info("Bicycle control initialized.")
    def timer_callback(self):
        # Example: Control the rear and front wheel velocity, and steering angle
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Velocity control for wheels (e.g., moving forward)
        self.joint_state.velocity[0] = 10.0  # Rear wheel velocity in radians/second
        self.joint_state.velocity[1] = 10.0  # Front wheel velocity in radians/second
        
        # Steering control (e.g., sinusoidal steering)
        self.joint_state.position[2] = math.sin(5*current_time) * 0.1  # Steering angle in radians

        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    bicycle_control = BicycleControl()
    rclpy.spin(bicycle_control)
    bicycle_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
