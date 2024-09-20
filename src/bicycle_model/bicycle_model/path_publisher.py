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

        self.x = list(np.arange(-30.1, 50.1, 0.1))
        self.y = []
        for num in self.x:
            self.y.append(3*math.sin(num * 2.0 * math.pi / 20.0))
            #self.y.append(0.0)
            # self.y.append(num)
            #self.y.append(num*math.tan(30*math.pi/180))

        self.path_data = []

        for i, v in enumerate(self.x):
            self.path_data.append([v, self.y[i]])

        self.timer = self.create_timer(1.0/30.0, self.publish_path)
        self.get_logger().info("Path published.")

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
