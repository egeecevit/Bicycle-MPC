#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64MultiArray, Float64
import numpy as np
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

class Controller(Node):
    def __init__(self):
        super().__init__('sim_controller')

        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.position_cmd_publisher_ = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )

        self.velocity_cmd_publisher_ = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10
        )

        self.path_subcriber_ = self.create_subscription(
            Path,
            "/path",
            self.path_callback,
            10
        )

        self.vel_subscriber_ = self.create_subscription(
            Float64,
            '/vel_topic',
            self.vel_callback,
            10
        )

        self.x_t = []
        self.y_t = []
        self.x = []
        self.y = []
        self.x_tp = 0
        self.y_tp = 0
        self.alpha = 0
        self.bicyc_length = 0.4
        self.wheel_rad = 0.05
        self.v = None
        self.w = None
        self.theta = 0

        self.timer_ = self.create_timer(0.005, self.controller_callback)
        self.get_logger().info("Controller node has been started.")

    def vel_callback(self, msg):
        self.v = msg.data
        self.w = self.v / self.wheel_rad

    def odom_callback(self,msg):
        self.x_t.append(msg.pose.pose.position.x)
        self.y_t.append(msg.pose.pose.position.y)
        self.theta = self.quat_to_yaw(msg.pose.pose.orientation)

    def path_callback(self, msg):
        for idx, pose in enumerate(msg.poses):
            self.x.append(pose.pose.position.x)
            self.y.append(pose.pose.position.y)

    def quat_to_yaw(self,quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Calculate yaw (rotation around z-axis)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return yaw_z  # in radians

    def find_largest(self, x, xtp, y, ytp):
        largestx = None
        largesty = None

        for i in range(len(x)-1):
            if x[i] < xtp:
                if largestx is None or x[i] > largestx:
                    largestx = x[i]
            if y[i] < ytp:
                if largesty is None or y[i] > largesty:
                    largesty = y[i]
        
        return largestx, largesty
    
    def find_smallest(self, x, x_p, y, y_p):
        smallestx = None
        smallesty = None

        for i in range(len(x)-1):
            if x[i] > x_p:
                if smallestx is None or x[i] < smallestx:
                    smallestx = x[i]
                    smallesty = y[i]

        return smallestx, smallesty
    

    def controller_callback(self):
        if len(self.x) == 0 and len(self.y) == 0:
            return

        if len(self.x_t) < 2:
            return
        
        if self.v is None:
            return
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        steering_ang = 0.0
        float64_msg = Float64MultiArray()
        float64_msg.data = [steering_ang]

        self.position_cmd_publisher_.publish(float64_msg)

        v_r = self.w * math.cos(steering_ang)
        v_f = self.w
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [v_r, v_f]
        self.velocity_cmd_publisher_.publish(velocity_msg)



def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
