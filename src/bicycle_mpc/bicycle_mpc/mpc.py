#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64MultiArray, Float64
import numpy as np
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from bicycle_mpc.model_class import BicycleModelMPC
from scipy.interpolate import InterpolatedUnivariateSpline
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker, MarkerArray

class MPC(Node):
    def __init__(self):
        super().__init__('MPC')

        self.mpc = BicycleModelMPC(dt=0.05, L=0.4, N=60)
        

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

        self.marker_array_publisher = self.create_publisher(
            MarkerArray,
            '/predicted_trajectory',
            10
        )

        # self.path_subcriber_ = self.create_subscription(
        #     Path,
        #     "/path",
        #     self.path_callback,
        #     10
        # )

        self.x_t = []
        self.y_t = []
        # self.x = []
        # self.y = []

        # Don't know why but published path has duplicate values
        self.x = list(np.arange(0, 50.1, 0.1))
        self.y = []
        for num in self.x:
            self.y.append(3*math.sin(num * 2.0 * math.pi / 40.0))
            #self.y.append(0.0)

        self.bicyc_length = 0.4
        self.wheel_rad = 0.05
        self.v = None
        self.w = None
        self.odom_status = None
        self.theta = 0
        self.spline_func = None
        self.derivative_func = None
        self.odom_timestamp = None
        self.initial_state = np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0])  # Example initial state

        self.timer_ = self.create_timer(1.0/30.0, self.controller_callback)
        self.get_logger().info("Controller node has been started.")


    def control_loop(self):
        optimal_controls, predicted_vel, predicted_states = self.mpc.solve(self.initial_state)

        control_input = optimal_controls[:, 0]
        delta = control_input[0]  # Steering angle
        a = control_input[1] # Acceleration
        self.v = predicted_vel[0]  # Predicted velocity
        # print(f'predicted_x = {predicted_states[0,0]}\n, predicted_y = {predicted_states[1,0]}\n, predicted_psi = {predicted_states[2,0]}\n, predicted_v = {predicted_states[3,0]}\n, predicted_cte = {predicted_states[4,0]}\n, predicted_epsi = {predicted_states[5,0]}\n')
        # print("-----------------------------------")

        return delta, a, predicted_states

    def odom_callback(self,msg):
        self.x_t.append(msg.pose.pose.position.x)
        self.y_t.append(msg.pose.pose.position.y)
        self.theta = self.quat_to_yaw(msg.pose.pose.orientation)
        self.odom_status = True
        self.odom_timestamp = msg.header.stamp

    # def path_callback(self, msg):
    #     for idx, pose in enumerate(msg.poses):
    #         self.x.append(pose.pose.position.x)
    #         self.y.append(pose.pose.position.y)

    def path_interpolation(self):
        spline_func = InterpolatedUnivariateSpline(self.x, self.y, k=3)
        derivative_func = spline_func.derivative()

        return spline_func, derivative_func

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
    
    def find_closest(self):
        # Ensure that there are positions to compare
        if not self.x_t or not self.y_t:
            raise ValueError("Robot's current position is not available.")

        closest = None
        idx = 0
        for i in range(len(self.x)):
            dist = math.sqrt((self.x[i] - self.x_t[-1])**2 + (self.y[i] - self.y_t[-1])**2)
            if closest is None or dist < closest:
                closest = dist
                idx = i

        return idx
    
    def create_marker(self, id, position, color=(0.0, 1.0, 0.0)):
        """
        Creates a single marker for RViz visualization.

        Parameters:
        - id: Unique identifier for the marker.
        - position: A tuple (x, y, z) of the marker position.
        - color: RGB tuple for marker color (default is green).

        Returns:
        - marker: A Marker message.
        """
        marker = Marker()
        marker.header.frame_id = "world"  # Adjust to your frame of reference
        marker.header.stamp = self.odom_timestamp
        marker.ns = "predicted_trajectory"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the position of the marker
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.0

        # Set marker scale
        marker.scale.x = 0.1  # Size of the marker
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set marker color (RGBA)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0  # Fully opaque

        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()  # Keep marker alive for 0.1 seconds

        return marker
    
    def publish_predicted_states(self, predicted_states):
        """
        Publishes the predicted trajectory as a MarkerArray.

        Parameters:
        - predicted_states: A 2D numpy array with predicted states from MPC.
        """
        marker_array = MarkerArray()
        for i in range(predicted_states.shape[1]):  # Loop through each predicted state
            x = predicted_states[0, i]
            y = predicted_states[1, i]
            marker = self.create_marker(i, (x, y, 0))
            marker_array.markers.append(marker)

        self.marker_array_publisher.publish(marker_array)


    def calculate_errors(self):
        spline_func, derivative_func = self.path_interpolation()

        # Find the closest point on the path
        idx = self.find_closest()

        # Get the slope of the path at the closest point
        slope = derivative_func(self.x[idx])
        path_angle = math.atan(slope)

        # Calculate heading error
        epsi = self.theta - path_angle

        # Normalize epsi within [-pi, pi]
        epsi = (epsi + np.pi) % (2 * np.pi) - np.pi

        # Calculate the signed cross-track error
        dx = self.x_t[-1] - self.x[idx]
        dy = self.y_t[-1] - self.y[idx]
        cte = -dx * math.sin(path_angle) + dy * math.cos(path_angle)

        print(f'path_x = {self.x[idx]}, path_y = {self.y[idx]}')
        print("-----------------------------------")
        print(f'cte = {cte}, epsi = {epsi}')

        return cte, epsi



    
    def controller_callback(self):
        if len(self.x) == 0 and len(self.y) == 0:
            return

        if len(self.x_t) < 2:
            return
        
        # if self.v is None:
        #     return
        #print(f'lenght of x: {len(self.x)}, lenght of y: {len(self.y)}')
        # Get the optimal control inputs
        delta, a, predicted_states = self.control_loop() # Get the optimal control inputs

        cte, epsi = self.calculate_errors()

        # Publish the steering angle
        float64_msg = Float64MultiArray()
        float64_msg.data = [delta]
        self.position_cmd_publisher_.publish(float64_msg)

        # Publish the velocity
        velocity_msg = Float64MultiArray()
        self.w = self.v[0] / self.bicyc_length # Don't know why but self.v is a list !!!!!!
        w_r = self.w * math.cos(delta)
        w_f = self.w
        #print(f'v = {self.v}')
        velocity_msg.data = [float(w_r), float(w_f)]
        self.velocity_cmd_publisher_.publish(velocity_msg)

        self.initial_state = np.array([self.x_t[-1], self.y_t[-1], self.theta, self.v[0], cte, epsi])
        self.publish_predicted_states(predicted_states)



def main(args=None):
    rclpy.init(args=args)
    node = MPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
