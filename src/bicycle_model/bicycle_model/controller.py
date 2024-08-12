#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.cmd_publisher_ = self.create_publisher(
            Float64,
            '/cmd_steer_angle',
            10
        )

        self.path_subcriber_ = self.create_subscription(
            Path,
            "/path",
            self.path_callback,
            10
        )

        self.pose_subscriber_ = self.create_subscription(
            PointStamped,
            '/bicycle_pose',
            self.pose_callback,
            10
        )

        self.x_t = []
        self.y_t = []
        self.x = None
        self.y = None
        self.bicyc_length = 1
        self.v = 20
        self.theta = 0 * math.pi / 180
        self.Kdd = 0.1
        self.ld = self.Kdd * self.v
        self.x = []
        self.y = []

        
        self.x_tp = 0
        self.y_tp = 0
        self.alpha = 0
        self.start = True
        self.x_in_circle = []
        self.y_in_circle = []
        self.circle = False

        self.timer_ = self.create_timer(0.005, self.controller_callback)
        self.get_logger().info("Controller node has been started.")

    def pose_callback(self,msg):
        self.x_t.append(msg.point.x)
        self.y_t.append(msg.point.y)


    def path_callback(self, msg):
        for idx, pose in enumerate(msg.poses):
            self.x.append(pose.pose.position.x)
            self.y.append(pose.pose.position.y)
            #print(f'x: {pose.pose.position.x}, y: {pose.pose.position.y}')
            #PATH DOĞRU GELİYO

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
    
    # def update_circle(self, circle):
    #     if circle:
    #         circle.remove()
    #     circle = plt.Circle( (self.x_t, self.y_t), self.ld, fill = False )
    #     self.ax.add_artist(circle)
    #     plt.draw()
    #     return circle
    
    
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
            
        if len(self.x_in_circle) == 0:
            self.x_tp, self.y_tp = self.find_smallest(self.x, self.x_t[-1], self.y, self.y_t[-1])
            self.alpha = np.arctan2((self.y_tp - self.y_t[-1]), (self.x_tp - self.x_t[-1])) - self.theta
            #print(f'alpha: {self.alpha}, xtp: {self.x_tp}, ytp: {self.y_tp}')
            print(f'theta-alpha: {self.theta - self.alpha}')

        if len(self.x_in_circle) > 0:
            self.x_tp, self.y_tp = max(self.x_in_circle), max(self.y_in_circle)
            self.alpha = np.arctan2((self.y_tp - self.y_t[-1]), (self.x_tp - self.x_t[-1])) - self.theta
            print(f'xtp: {self.x_tp}, ytp: {self.y_tp}, alpha: {self.alpha}')
            if dist_p > dist_tp:
                self.x_tp, self.y_tp = self.find_smallest(self.x, self.x_t[-1], self.y, self.y_t[-1])
                self.alpha = np.arctan2((self.y_tp - self.y_t[-1]), (self.x_tp - self.x_t[-1])) - self.theta

        steering_ang = math.atan((2 * self.bicyc_length * math.sin(self.alpha)) / self.ld)
        float64_msg = Float64()
        float64_msg.data = steering_ang

        self.cmd_publisher_.publish(float64_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()