#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64MultiArray
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

        self.x_t = []
        self.y_t = []
        self.x = []
        self.y = []
        self.x_in_circle = []
        self.y_in_circle = []
        self.x_tp = 0
        self.y_tp = 0
        self.alpha = 0
        self.bicyc_length = 0.4
        self.v = 40.0
        self.theta = None
        self.Kdd = 0.07
        self.ld = self.Kdd * self.v

        self.timer_ = self.create_timer(0.005, self.controller_callback)
        self.get_logger().info("Controller node has been started.")

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

    def data_in_circle(self, x, x_t, y, y_t):
        for i in range(len(x)-1):
            distance = math.sqrt((x[i] - x_t)**2 + (y[i] - y_t)**2)
            if distance <= self.ld:
                self.x_in_circle.append(x[i])
                self.y_in_circle.append(y[i])
            if len(self.x_in_circle) > 2 and len(self.y_in_circle) > 2:
                self.x_in_circle.pop(0)
                self.y_in_circle.pop(0)

    
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
        self.theta = np.arctan2(self.y_t[-1] - self.y_t[-2], self.x_t[-1] - self.x_t[-2])
        self.data_in_circle(self.x, self.x_t[-1], self.y, self.y_t[-1])
        #self.circle = self.update_circle(self.circle)

        dist_p = math.sqrt((self.x_t[-1])**2 + (self.y_t[-1])**2)
        dist_tp = math.sqrt((self.x_tp)**2 + (self.y_tp)**2)
        dist_max = math.sqrt(self.x[-2]**2 + self.y[-2]**2)

        if dist_p > dist_max:
            self.alpha = np.arctan2((self.y[-2] - self.y_t[-1]), (self.x[-2] - self.x_t[-1])) - self.theta
            
        if len(self.x_in_circle) < 1:
            self.x_tp, self.y_tp = self.find_smallest(self.x, self.x_t[-1], self.y, self.y_t[-1])
            self.alpha = np.arctan2((self.y_tp - self.y_t[-1]), (self.x_tp - self.x_t[-1])) - self.theta
            #print(f'alpha: {self.alpha}, xtp: {self.x_tp}, ytp: {self.y_tp}')
            #print(f'theta-alpha: {self.theta - self.alpha}')

        if len(self.x_in_circle) > 0:
            self.x_tp, self.y_tp = self.x_in_circle[-1], self.y_in_circle[-1]
            self.alpha = np.arctan2((self.y_tp - self.y_t[-1]), (self.x_tp - self.x_t[-1])) - self.theta
            # print(f'xtp: {self.x_tp}, ytp: {self.y_tp}, alpha: {self.alpha}\n')
            # print(f'x_in_circle: {self.x_in_circle}, y_in_circle: {self.y_in_circle}')
            if dist_p > dist_tp:
                self.x_tp, self.y_tp = self.x[-1], self.y[-1]
                self.alpha = np.arctan2((self.y_tp - self.y_t[-1]), (self.x_tp - self.x_t[-1])) - self.theta

        if self.alpha > math.pi/4:
            self.alpha = math.pi/4

        steering_ang = math.atan((2 * self.bicyc_length * math.sin(self.alpha)) / self.ld)
        float64_msg = Float64MultiArray()
        float64_msg.data = [steering_ang]

        self.position_cmd_publisher_.publish(float64_msg)

        v_r = -self.v
        v_f = -self.v * math.cos(steering_ang)
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
    
