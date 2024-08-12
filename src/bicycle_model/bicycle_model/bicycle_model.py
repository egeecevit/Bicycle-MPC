#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped


class BicycleModel(Node):
    def __init__(self):
        super().__init__('bicycle_model')

        self.cmd_subscriber_ = self.create_subscription(
            Float64,
            '/cmd_steer_angle',
            self.model_callback,
            10
        )

        self.pose_publisher_ = self.create_publisher(
            PointStamped,
            '/bicycle_pose',
            10
        )

        self.v = 10
        self.theta = 90*math.pi/180
        self.bicyc_length = 1
        self.theta_dot = 0.0
        self.dt = 0.01
        self.x_t = 0.0
        self.y_t = -5.0
        self.timer = self.create_timer(0.005, self.publish_pose)
        self.get_logger().info("Bicycle model node has been started.")


    def publish_pose(self):
        point = PointStamped()
        point.header.frame_id = 'map'
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = self.x_t
        point.point.y = self.y_t
        point.point.z = 0.0

        self.pose_publisher_.publish(point)


    def model_callback(self,msg):
        steering_ang = msg.data
        x_dot = self.v * math.cos(self.theta)
        y_dot = self.v * math.sin(self.theta)
        self.theta_dot = self.v * math.tan(steering_ang) / self.bicyc_length

        self.x_t += x_dot * self.dt
        self.y_t += y_dot * self.dt
        self.theta += self.theta_dot * self.dt


def main(args=None):
    rclpy.init(args=args)
    node = BicycleModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


