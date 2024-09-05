#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class Circles(Node):
    def __init__(self):
        super().__init__('circles')

        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.marker_publisher_ = self.create_publisher(
            MarkerArray,
            '/circles_marker_array',
            10
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_status = None
        self.odom_timestamp = None

        #Parameters for the circle
        self.angle_increment = 0.1 # For smoothness
        self.radius = 5.0

        self.timer = self.create_timer(1.0/30.0, self.publish_circle_marker_array)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y   
        self.current_theta = self.quat_to_yaw(msg.pose.pose.orientation)
        self.odom_status = True
        self.odom_timestamp = msg.header.stamp

    def quat_to_yaw(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Calculate yaw (rotation around z-axis)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return yaw_z  # in radians

    def publish_circle_marker_array(self):
        if self.odom_status is None:
            return

        marker_array = MarkerArray()

        # Calculate the circle centers on both sides of the robot's position and heading
        x_r, y_r = self.current_x, self.current_y
        theta = self.current_theta

        # Center on the left
        x_c_left = x_r + self.radius * math.cos(theta + math.pi/2)
        y_c_left = y_r + self.radius * math.sin(theta + math.pi/2)

        # Center on the right
        x_c_right = x_r + self.radius * math.cos(theta - math.pi/2)
        y_c_right = y_r + self.radius * math.sin(theta - math.pi/2)

        # Create markers for left and right circles
        left_marker = self.create_circle_marker(x_c_left, y_c_left, self.odom_timestamp, 0, [1.0, 1.0, 1.0])
        right_marker = self.create_circle_marker(x_c_right, y_c_right, self.odom_timestamp, 1, [1.0, 1.0, 1.0])

        # Append markers to the MarkerArray
        marker_array.markers.append(left_marker)
        marker_array.markers.append(right_marker)

        # Publish the MarkerArray
        self.marker_publisher_.publish(marker_array)

    def create_circle_marker(self, x_center, y_center, timestamp, marker_id, color):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = timestamp
        marker.ns = "circle"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width

        # Generate points for the circle
        for i in range(int(2 * math.pi / self.angle_increment) + 2):
            phi = i * self.angle_increment
            x = x_center + self.radius * math.cos(phi)
            y = y_center + self.radius * math.sin(phi)

            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0  # Assuming 2D circle in XY plane
            marker.points.append(point)

        marker.color.a = 1.0  # Alpha
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = Circles()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
