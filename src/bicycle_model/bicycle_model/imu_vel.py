#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuVelocityCalculator(Node):
    def __init__(self):
        super().__init__('imu_velocity_calculator')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.listener_callback,
            10)
        self.velocity = [0.0, 0.0, 0.0]
        self.last_time = self.get_clock().now()

        self.get_logger().info('Imu Velocity Calculator has been started.')

    def listener_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.velocity[0] += msg.linear_acceleration.x * dt
        self.velocity[1] += msg.linear_acceleration.y * dt
        self.velocity[2] += msg.linear_acceleration.z * dt
        self.last_time = current_time
        self.get_logger().info(f'Estimated Velocity: {self.velocity}')

def main(args=None):
    rclpy.init(args=args)
    imu_velocity_calculator = ImuVelocityCalculator()
    rclpy.spin(imu_velocity_calculator)
    imu_velocity_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
