#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64


class VisualizeRoute(Node):
    def __init__(self):
        super().__init__('visualize_routes')

        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.marker_publisher_ = self.create_publisher(
            MarkerArray,
            '/marker_array',
            10
        )

        self.vel_subscriber_ = self.create_subscription(
            Float64,
            '/vel_topic',
            self.vel_callback,
            10
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.max_steering_angle = 35 * math.pi / 180  # Maximum steering angle
        self.steering_rate = 6 * math.pi / 180  # Rate at which the steering angle changes per time step
        self.min_steering_angle = -self.max_steering_angle # Minimum steering angle
        self.time_steps = 1000  # Number of time steps to simulate
        self.odom_status = None # Flag to check if the odometry message has been received
        self.velocities = None
        self.odom_timestamp = None
        self.dt = 0.1
        self.l = 3.30  # Wheelbase

        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

        self.get_logger().info("Visualize Route node has been started.")


    def vel_callback(self, msg):
        self.velocities = [msg.data/3.6, 5*msg.data/3.6, 10*msg.data/3.6]

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
    
    def calculate_paths(self, start_x, start_y, start_theta):
        all_paths = []
        if self.velocities is not None:
            for v in self.velocities:
                # For each velocity, calculate paths with increasing and decreasing steering angles
                increasing_points = []
                decreasing_points = []

                x_inc, y_inc, theta_inc = start_x, start_y, start_theta
                x_dec, y_dec, theta_dec = start_x, start_y, start_theta
                #print(f'start_x: {start_x}, start_y: {start_y}, start_theta: {start_theta}')
                
                steering_angle_inc = 0.0 * math.pi / 180  # Start with 0 steering angle for increasing
                steering_angle_dec = 0.0 * math.pi / 180  # Start with 0 steering angle for decreasing
                
                for step in range(self.time_steps):
                    # Increasing steering angle
                    point_inc = Point()
                    point_inc.x = x_inc
                    point_inc.y = y_inc
                    increasing_points.append(point_inc)
                    
                    if steering_angle_inc < self.max_steering_angle:
                        steering_angle_inc += self.steering_rate * self.dt
                    
                    x_inc += v * np.cos(theta_inc) * self.dt
                    y_inc += v * np.sin(theta_inc) * self.dt
                    theta_inc += v * np.tan(steering_angle_inc) * self.dt / self.l
                    
                    # Decreasing steering angle
                    point_dec = Point()
                    point_dec.x = x_dec
                    point_dec.y = y_dec
                    decreasing_points.append(point_dec)
                    
                    if steering_angle_dec > self.min_steering_angle:
                        steering_angle_dec -= self.steering_rate * self.dt
                    
                    x_dec += v * np.cos(theta_dec) * self.dt
                    y_dec += v * np.sin(theta_dec) * self.dt
                    theta_dec += v * np.tan(steering_angle_dec) * self.dt / self.l
                
                all_paths.append((increasing_points, decreasing_points))
        
        return all_paths
    
    def timer_callback(self):
        if self.odom_status and self.velocities is not None:
        
            marker_array = MarkerArray()

            # Calculate paths for different velocities with increasing and decreasing steering angles
            all_paths = self.calculate_paths(self.current_x, self.current_y, self.current_theta)
            
            # Create markers for each path with different velocities
            for i, (inc_points, dec_points) in enumerate(all_paths):
                # Marker for increasing steering angle path
                inc_marker = Marker()
                inc_marker.header.frame_id = "world"
                inc_marker.header.stamp = self.odom_timestamp
                inc_marker.type = Marker.LINE_STRIP
                inc_marker.action = Marker.ADD
                inc_marker.scale.x = 0.1  # Line width
                inc_marker.color.a = 1.0  # Alpha (transparency)

                # Assign different colors to each path
                if i == 0:
                    inc_marker.color.r = 1.0
                    inc_marker.color.g = 0.0
                    inc_marker.color.b = 0.0  # Red
                elif i == 1:
                    inc_marker.color.r = 0.0
                    inc_marker.color.g = 1.0
                    inc_marker.color.b = 0.0  # Green
                else:
                    inc_marker.color.r = 0.0
                    inc_marker.color.g = 0.0
                    inc_marker.color.b = 1.0  # Blue

                inc_marker.id = i * 2
                inc_marker.points = inc_points
                #inc_marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

                marker_array.markers.append(inc_marker)

                # Marker for decreasing steering angle path
                dec_marker = Marker()
                dec_marker.header.frame_id = "world"
                dec_marker.header.stamp = self.odom_timestamp
                dec_marker.type = Marker.LINE_STRIP
                dec_marker.action = Marker.ADD
                dec_marker.scale.x = 0.1  # Line width
                dec_marker.color.a = 1.0  # Alpha (transparency)

                # Use the same color as the increasing path for consistency
                dec_marker.color = inc_marker.color

                dec_marker.id = i * 2 + 1
                dec_marker.points = dec_points
                #dec_marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

                marker_array.markers.append(dec_marker)

            # Publish the marker array
            self.marker_publisher_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizeRoute()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
