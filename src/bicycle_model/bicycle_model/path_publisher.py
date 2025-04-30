#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        self.publisher_ = self.create_publisher(
            Path,
            '/path',
            10
        )

        self.x = list(np.arange(0.0, 50.01, 0.01))
        self.y = []
        for num in self.x:
            self.y.append(3*math.sin(num * 2.0 * math.pi / 10.0))
        #     #self.y.append(3*math.cos(num * 2.0 * math.pi / 7.5))
        #     #self.y.append(0.0)
        #     # self.y.append(num)
        #     #self.y.append(num*math.tan(30*math.pi/180))

        self.path_data = []

        for i, v in enumerate(self.x):
            self.path_data.append([v, self.y[i]])
        self.x = []
        self.y = []

        # # Create the horizontal part of the "J"
        # horizontal_length = 15.0  # Length of the horizontal part
        # self.x.extend(np.arange(0.0, horizontal_length, 0.01))
        # self.y.extend([0.0] * len(self.x))  # y stays constant along the horizontal part

        # # Create the 1/8 circle part of the "J" that extends towards increasing x
        # radius = 5.0  # Radius of the 1/8 circle
        # num_points = 100  # Number of points to approximate the circle
        # angles = np.linspace(0, math.pi / 4, num_points)  # Angles for 1/8 of the circle (π/4 = 45 degrees)

        # for angle in angles:
        #     self.x.append(horizontal_length + radius * math.sin(angle))  # x increases as sin(angle)
        #     self.y.append(radius * (1 - math.cos(angle)))  # y forms the 1/8 circle, increasing in y direction

        # # Create the 45-degree slope line
        # line_length = 50.0  # Length of the line
        # last_x = self.x[-1]
        # last_y = self.y[-1]

        # for i in np.arange(1.0, line_length, 0.01):
        #     self.x.append(last_x + i)
        #     self.y.append(last_y + i)  # Slope of 45 degrees means x and y increase by the same amount

        # self.path_data = []

        # for i, v in enumerate(self.x):
        #     self.path_data.append([v, self.y[i]])

        self.timer = self.create_timer(1.0/30.0, self.publish_path)
        self.get_logger().info("Path published.")
        self.get_logger().info(f"x = {self.x}")

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in self.path_data:
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0 
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
