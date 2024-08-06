#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import matplotlib.pyplot as plt
import rclpy.node
import numpy as np



class BicycleControl(Node):
    def __init__(self):
        super().__init__('bicycle_control')

        self.bicyc_length = 1
        self.v = 10
        self.gamma_dot = 0
        self.theta = 0
        self.dt = 0.01
        self.x_t = 7
        self.y_t = 7


        self.declare_parameter("steering_angle",30)

        self.timer = self.create_timer(0.01,self.timer_callback)

        # Initialize plot
        self.fig, self.ax = plt.subplots()
        self.x_data, self.y_data = [], []
        self.line, = self.ax.plot(self.x_data, self.y_data, 'r-')

        #plt.axhline(y=15, color = 'b')
        x = np.linspace(0,30,100)
        y = -x + 15
        plt.plot(x, y, '-g')
        plt.ion()
        plt.show()

    def timer_callback(self):
        # steering_ang = self.get_parameter("steering_angle").value

        # steering_ang = steering_ang * math.pi / 180

        # line equation ax+by+c=0
        a = 1
        b = 1
        c = -15

        Kd = 0.5
        Kh = 1

        theta_s = math.atan2(-a, b) #radians
        line_array = np.array([a, b, c])
        position_array = np.array([self.x_t, self.y_t, 1])
        d = np.dot(line_array, position_array) / math.sqrt(a**2 + b**2)
        print(f'distance from the line : {d}\n')
        
        steering_ang = -Kd * d + Kh * (theta_s - self.theta)
        print(f'theta: {self.theta}\n')
        print(f'steering angle : {steering_ang}')

        #steering_ang = -14 * math.pi /180
        # Rear wheel is the reference Bicycle Kinematic
        x_dot = self.v * math.cos(self.theta)
        y_dot = self.v * math.sin(self.theta)
        self.theta_dot = self.v * math.tan(steering_ang) / self.bicyc_length
        self.theta += self.theta_dot * self.dt



        self.x_t += x_dot * self.dt
        self.y_t += y_dot * self.dt

        """------------------------------------------------------------------------------------------------------------------"""

        # self.get_logger().info(f'x_dot: {x_dot}, y_dot: {y_dot}, theta_dot: {self.theta_dot}, gamma_dot: {self.gamma_dot}\n')
        # self.get_logger().info(f'x: {self.x_t} y: {self.y_t}\n')
        


        # Update plot
        self.x_data.append(self.x_t)
        self.y_data.append(self.y_t)
        self.line.set_xdata(self.x_data)
        self.line.set_ydata(self.y_data)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = BicycleControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




