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
        self.theta = 45 * math.pi / 180
        self.dt = 0.005
        self.x_t = 0
        self.y_t = -2
        self.Kdd = 0.03
        self.ld = self.Kdd * self.v
        self.x = list(np.arange(0, 100.1, 0.1))
        self.y = []
        for num in self.x:
            self.y.append(math.sin(num*2.0*math.pi/40.0))
        
        self.x_tp = 0
        self.y_tp = 0
        self.alpha = 0
        self.start = True
        self.x_in_circle = []
        self.y_in_circle = []
        self.circle = False


        self.timer = self.create_timer(0.005,self.timer_callback)

        # Initialize plot
        self.fig, self.ax = plt.subplots()
        self.x_data, self.y_data = [self.x_t], [self.y_t]
        self.line, = self.ax.plot(self.x_data, self.y_data, 'r-', linewidth=2)
        plt.ion()
        plt.show()
    
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
    
    def update_circle(self, circle):
        if circle:
            circle.remove()
        circle = plt.Circle( (self.x_t, self.y_t), self.ld, fill = False )
        self.ax.add_artist(circle)
        plt.draw()
        return circle


    def timer_callback(self):
        #----------------------Pure Pursuit------------------------------------------------------
        #self.ld = self.Kdd * self.v
        self.data_in_circle(self.x, self.x_t, self.y, self.y_t)
        # print(f'x_in_circle: {self.x_in_circle}, y_in_circle: {self.y_in_circle}')
        self.circle = self.update_circle(self.circle)
        self.theta = np.arctan2(self.y[-2] - self.y[-1], self.x[-2] - self.x[-1])
        x_p, y_p = self.find_smallest(self.x, self.x_tp, self.y, self.y_tp)
        dist_p = math.sqrt((self.x_t)**2 + (self.y_t)**2)
        dist_tp = math.sqrt((self.x_tp)**2 + (self.y_tp)**2)
        dist_max = math.sqrt(self.x[-2]**2 + self.y[-2]**2)

        if dist_p > dist_max:
            self.alpha = np.arctan2((self.y[-2] - self.y_t), (self.x[-2] - self.x_t)) - self.theta
            
        elif len(self.x_in_circle) == 0:
            self.x_tp, self.y_tp = self.find_smallest(self.x, self.x_t, self.y, self.y_t)
            self.alpha = np.arctan2((self.y_tp - self.y_t), (self.x_tp - self.x_t)) - self.theta

        elif len(self.x_in_circle) > 0:
            self.x_tp, self.y_tp = max(self.x_in_circle), max(self.y_in_circle)
            self.alpha = np.arctan2((self.y_tp - self.y_t), (self.x_tp - self.x_t)) - self.theta
            if dist_p > dist_tp:
                self.x_tp, self.y_tp = self.find_smallest(self.x, self.x_t, self.y, self.y_t)
                self.alpha = np.arctan2((self.y_tp - self.y_t), (self.x_tp - self.x_t)) - self.theta
        print(f'xtp: {self.x_tp}, ytp: {self.y_tp}\n x_in_circle: {self.x_in_circle}, y_in_circle: {self.y_in_circle}\n x_t: {self.x_t}, y_t: {self.y_t}')

        # elif dist1 < dist2:
        #     print("burdayım")
        #     self.x_tp, self.y_tp = max(self.x_in_circle), max(self.y_in_circle)
        #     self.alpha = np.arctan2((self.y_tp - self.y_t), (self.x_tp - self.x_t)) - self.theta
        #     print(f'xtp: {self.x_tp}, ytp: {self.y_tp}\n x_in_circle: {self.x_in_circle}, y_in_circle: {self.y_in_circle}\n x_t: {self.x_t}, y_t: {self.y_t}')
            # else:
            #     print(f'xtp: {self.x_tp}, ytp: {self.y_tp}\n x_in_circle: {self.x_in_circle}, y_in_circle: {self.y_in_circle}\n x_t: {self.x_t}, y_t: {self.y_t}')
            #     self.x_tp, self.y_tp = max(self.x_in_circle), max(self.y_in_circle)
            #     self.alpha = np.arctan2((self.y_tp - self.y_t), (self.x_tp - self.x_t)) - self.theta


        steering_ang = math.atan((2 * self.bicyc_length * math.sin(self.alpha)) / self.ld)
        steering_ang = 0 * math.pi / 180
        #----------------------------------------------------------------------------------------------

        # ----------------Rear wheel is the reference Bicycle Kinematic----------------------------------
        x_dot = self.v * math.cos(self.theta)
        y_dot = self.v * math.sin(self.theta)
        self.theta_dot = self.v * math.tan(steering_ang) / self.bicyc_length

        self.x_t += x_dot * self.dt
        self.y_t += y_dot * self.dt
        self.theta += self.theta_dot * self.dt
        #------------------------------------------------------------------------------------------------------------------        


        # Update plot
        self.x_data.append(self.x_t)
        self.y_data.append(self.y_t)
        self.line.set_xdata(self.x_data)
        self.line.set_ydata(self.y_data)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.plot(self.x_tp, self.y_tp, 'ro')
        plt.plot(self.x, self.y, 'b', linewidth=0.5)
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




