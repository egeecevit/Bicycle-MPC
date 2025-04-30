#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64MultiArray, Float64
import numpy as np
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker, MarkerArray
from scipy.interpolate import InterpolatedUnivariateSpline
import casadi as ca


class MPC(Node):
    def __init__(self):
        super().__init__('MPC')
        
        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1
        )

        self.position_cmd_publisher_ = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )

        self.velocity_cmd_publisher_ = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            1
        )

        self.marker_array_publisher = self.create_publisher(
            MarkerArray,
            '/predicted_trajectory',
            1
        )

        self.vel_publisher = self.create_publisher(
            Float64,
            '/vel_topic',
            1
        )

        self.rate_publisher = self.create_publisher(
            Float64,
            '/rate_topic',
            1
        )

        self.steer_publisher = self.create_publisher(
            Float64,
            '/steer_topic',
            1
        )

        self.dt = 0.05
        self.L = 0.4
        self.N = 30
        
        

        self.x_t = []
        self.y_t = []

        # Don't know why but published path has duplicate values
        ### PATH DEFINITION ###
        # self.x = []
        # self.y = []

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

        #---------------------------------#
        self.x = list(np.arange(0.0, 50.01, 0.01))
        self.y = []
        #self.v_ref = [1.0] * (len(self.x) - 50) + [0.0] * 50
        for num in self.x:
            self.y.append(3*math.sin(num * 2.0 * math.pi / 10.0))
        #     #self.y.append(3*math.cos(num * 2.0 * math.pi / 7.5))
        #     #self.y.append(0.0)
        #     #self.y.append(num)
        #     #self.y.append(num*math.tan(30*math.pi/180))

        self.bicyc_length = 0.4
        self.wheel_rad = 0.05
        self.v = None
        self.w = None
        self.odom_status = None
        self.theta = 0.0
        self.odom_timestamp = None
        self.v_init = 0.0
        self.delta_init = 0.0
        self.predicted_states = None
        self.initial_state = None
        self.spline_func = None
        self.derivative_func = None
        self.spline_and_slope_func = None
        # self.a_c = [0.0] * self.N
        # self.steer_c = [0.0] * self.N
        self.a_c = 0.0
        self.steer_c = 0.0
        self.v_ref = 5.0
        self.v_limit = [0.2, 5.0]
        self.cte_values = []
        self.v_values = []

        self.define_variables()
        self.setup_model()
        self.spline(self.x, self.y)

        self.timer_ = self.create_timer(1.0/30.0, self.controller_callback)
        self.get_logger().info("Controller node has been started.")

    #------------------------------------------------------------------------------------------------#
    #CASADI NONLINEAR SOLVER PART#
    def define_variables(self):
        """Define state and control variables."""
        self.x_m = ca.MX.sym('x')
        self.y_m = ca.MX.sym('y')
        self.psi_m = ca.MX.sym('psi')
        self.v_m = ca.MX.sym('v')
        self.delta = ca.MX.sym('delta')
        
        # State vector
        self.state = ca.vertcat(self.x_m, self.y_m, self.psi_m, self.v_m, self.delta)
        
        # Control variables
        self.steer_rate = ca.MX.sym('steer_rate')  # Steering angle
        self.a = ca.MX.sym('a')          # Acceleration
        
        # Control vector
        self.control = ca.vertcat(self.steer_rate, self.a)

    def setup_model(self):
        # Define state transition equations
        x_next = self.x_m + self.v_m * ca.cos(self.psi_m) * self.dt
        y_next = self.y_m + self.v_m * ca.sin(self.psi_m) * self.dt
        delta_next = self.delta + self.steer_rate * self.dt
        psi_next = self.psi_m + (self.v_m / self.L) * self.delta * self.dt
        v_next = self.v_m + self.a * self.dt

        # Update the state transition function to include cte and epsi
        self.f = ca.Function('f', [self.state, self.control], 
                            [ca.vertcat(x_next, y_next, psi_next, v_next, delta_next)])
        

    def solve(self, initial_state):
        # Define prediction horizon variables
        X = ca.MX.sym('X', 5, self.N+1)  # State trajectory
        U = ca.MX.sym('U', 2, self.N)    # Control trajectory

        # Initialize cost and constraints
        cost = 0
        g = []

        # Define the Q and R matrices for state and control penalties
        #Q = ca.diagcat(0.0, 0.0, 0.0, 1.0)  # cte and epsi are weighted more because they are the main things to be penalized
        R = ca.diagcat(10.0, 2.0)  # Adjust weights for control inputs

        # Initial state constraint
        g.append(X[:, 0] - initial_state)  # Ensure X[:, 0] == initial_state

        # Loop through each step in the prediction horizon
        for k in range(self.N):
            # State cost: penalize deviations from desired state
            x = X[:, k]

            cost += 10.0 * (x[3] - self.v_ref)**2  # Penalize velocity deviation from 1.2

            cost += 50.0 * ca.mtimes(x[4],x[4].T)  # Penalize steering angle

            cte, epsi = self.calculate_errors_symbolic_spline(x[0], x[1], x[2]) 
            
            cost += 50 * cte**2 + 100 * epsi**2  # Penalize cte and epsi 

            # Control cost: penalize magnitude of control inputs
            u = U[:, k]
            cost += ca.mtimes([u.T, R, u])

            # State transition constraints
            state_next = self.f(X[:, k], U[:, k])
            g.append(X[:, k+1] - state_next)

        # Combine constraints and decision variables
        # Constraints are more complex than bounds for decision variables
        # Constraints are like inequalities or equalities that must be satisfied
        # Bounds are like limits on the decision variables
        g = ca.vertcat(*g)
        decision_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))

        # Define lower and upper bounds for decision variables
        lb_decision_vars = -ca.inf * ca.DM.ones(decision_vars.shape)
        ub_decision_vars = ca.inf * ca.DM.ones(decision_vars.shape)

        # Define bounds
        # Velocity, CTE, and EPSI limits
        vel_limit = self.v_limit
        steering_angle_limit = np.deg2rad(30.0)
        #steering_angle_limit = ca.inf

        # State bounds: [x, y, psi, v]
        lb_states = ca.DM([-ca.inf, -ca.inf, -ca.inf, vel_limit[0], -steering_angle_limit])  
        ub_states = ca.DM([ca.inf, ca.inf, ca.inf, vel_limit[1], steering_angle_limit])

        # Control bounds: [steering angle rate, a (acceleration)]
        steering_rate_limit = np.deg2rad(10.0)  # 10 degrees/sec in radians
        #steering_rate_limit = ca.inf
        lb_controls = ca.DM([-steering_rate_limit, -1])  # [delta_min, a_min]
        ub_controls = ca.DM([steering_rate_limit, 1])    # [delta_max, a_max]

        # Apply state bounds only from the second state onward (skip initial state)
        lb_states_full = ca.repmat(lb_states, self.N, 1).reshape((-1, 1))  # N steps only
        ub_states_full = ca.repmat(ub_states, self.N, 1).reshape((-1, 1))

        # Control bounds as before
        lb_controls_full = ca.repmat(lb_controls, self.N, 1).reshape((-1, 1))
        ub_controls_full = ca.repmat(ub_controls, self.N, 1).reshape((-1, 1))

        # Adjust to apply state bounds starting from the second predicted state onward
        num_states = 5 * (self.N + 1)  # 6 states per step, N steps
        lb_decision_vars[5: num_states] = lb_states_full  # Skip initial state (first 6 entries)
        ub_decision_vars[5: num_states] = ub_states_full

        # Apply control bounds as usual
        lb_decision_vars[num_states:] = lb_controls_full
        ub_decision_vars[num_states:] = ub_controls_full

        # Define lower and upper bounds for constraints i.e g(x) = 0 constraints are tried to be satisfied
        lb_g = ca.DM.zeros(g.shape)
        ub_g = ca.DM.zeros(g.shape)

        # Define the optimization problem
        nlp = {'x': decision_vars, 'f': cost, 'g': g}
        # Set IPOPT options to suppress output
        options = {
            'ipopt': {
                'print_level': 0,  # Suppresses IPOPT output
                'sb': 'yes',       # Suppress banner
                'tol': 1e-6,
                'max_iter': 2000,
                #'print_timing_statistics': 'yes',
                #'timing_statistics': 'yes',
            },
            'print_time': 0
        }
        solver = ca.nlpsol('solver', 'ipopt', nlp, options)

        # Initial guess for the decision variables
        #x0_decision_vars = ca.DM.zeros(decision_vars.shape)
        state_guess = [initial_state]

        # Propagate the initial state forward using zero control inputs
        for _ in range(self.N):
            # Predict next state assuming zero steering rate (delta_dot) and zero acceleration (a)
            next_state = self.f(state_guess[-1], [0, 0])  # Zero control input
            #next_state = self.f(state_guess[-1], [self.steer_c, self.a_c]) # Use the last control input
            state_guess.append(next_state.full().flatten())

        # Flatten the state trajectory guesses into a single column vector
        state_guess = np.hstack(state_guess).flatten()

        # Control input guess: All zeros (no control action)
        control_guess = np.zeros(2 * self.N)  # [delta, a] for N steps

        # Combine state and control guesses into the decision variable vector
        x0_decision_vars = np.hstack([state_guess, control_guess]).reshape(-1, 1)

        '''-------------------------'''

        # Solve the optimization problem
        solution = solver(x0=x0_decision_vars, 
                        lbx=lb_decision_vars,
                        ubx=ub_decision_vars, 
                        lbg=lb_g, 
                        ubg=ub_g)

        stats = solver.stats()
        #print(f"Solver iterations: {stats['iter_count']}")

        # Extract the optimal control inputs
        optimal_controls = solution['x'][num_states:num_states + 2 * self.N]
        optimal_controls = ca.reshape(optimal_controls, 2, self.N)

        # Extract the predicted state trajectory (including velocity)
        predicted_states = solution['x'][:num_states]
        predicted_states = ca.reshape(predicted_states, 5, self.N + 1)

        # Extract the velocity trajectory from the predicted states
        predicted_velocity = predicted_states[3, :]  # Extract the velocity component (4th row)

        return optimal_controls.full(), predicted_velocity.full(), predicted_states.full() # Convert to a numpy array optimal_controls[:,0] is the optimal control for the first step


    #------------------------------------------------------------------------------------------------#

    def odom_callback(self,msg):
        self.x_t.append(msg.pose.pose.position.x)
        self.y_t.append(msg.pose.pose.position.y)
        self.theta = self.quat_to_yaw(msg.pose.pose.orientation)
        self.odom_status = True
        self.odom_timestamp = msg.header.stamp

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
    
    def create_marker(self, id, position, color=(0.0, 1.0, 0.0)):
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
        marker_array = MarkerArray()
        for i in range(predicted_states.shape[1]):  # Loop through each predicted state
            x = predicted_states[0, i]
            y = predicted_states[1, i]
            marker = self.create_marker(i, (x, y, 0))
            marker_array.markers.append(marker)

        self.marker_array_publisher.publish(marker_array)


    def calculate_errors(self, x_pred, y_pred,psi_pred):
        distances = np.sqrt((np.array(self.x) - x_pred)**2 + (np.array(self.y) - y_pred)**2)
        idx = np.argmin(distances)

        # Get the slope of the path at the closest point
        path_slope = np.gradient(self.y)[idx] / np.gradient(self.x)[idx]
        path_angle = np.arctan(path_slope)

        # Calculate heading error
        epsi = psi_pred - path_angle

        # Normalize epsi within [-pi, pi]
        epsi = (epsi + np.pi) % (2 * np.pi) - np.pi

        # Calculate the signed cross-track error
        dx = x_pred - self.x[idx]
        dy = y_pred - self.y[idx]
        cte = -dx * np.sin(path_angle) + dy * np.cos(path_angle)

        return cte, epsi
    
    def spline(self, x, y):
        # Define symbolic interpolation for path using interp1d
        path_spline_y = ca.interpolant('spline', 'bspline', [x], y)

        x_sym = ca.MX.sym('x_sym')
        
        # Manually compute the slope by differentiating the spline function
        spline_value = path_spline_y(x_sym)
        spline_slope = ca.jacobian(spline_value, x_sym)

        self.spline_and_slope_func = ca.Function('spline_and_slope', [x_sym], [spline_value, spline_slope])
    
    def calculate_errors_symbolic_spline(self, x_pred, y_pred, psi_pred):
        spline_value, spline_slope = self.spline_and_slope_func(x_pred)

        #spline_slope = ca.if_else(ca.fabs(spline_slope) > 1.1, 1.1, spline_slope)

        path_angle = ca.atan(spline_slope)  # Path angle from slope

        # Calculate heading error (epsi)
        epsi = psi_pred - path_angle

        # Normalize EPSI within [-pi, pi] using CasADi functions
        epsi = ca.fmod(epsi + ca.pi, 2 * ca.pi) - ca.pi

        # Calculate cross-track error (cte)
        cte = spline_value - y_pred  # Signed cross-track error

        return cte, epsi
    
    def plot_cte_vs_velocity(self):
        cte = max(self.cte_values)
        plt.figure()
        plt.scatter(self.v_ref, cte, color='r', label="CTE vs Velocity", marker='o')
        plt.xlabel("Velocity (m/s)")
        plt.ylabel("Cross Track Error (CTE)")
        plt.title("CTE vs Velocity")
        plt.legend()
        plt.grid(True)
        plt.show()

    def controller_callback(self):
        
        if len(self.x) == 0 and len(self.y) == 0:
            return
        if self.odom_status is None:
            return

        if len(self.x_t) < 2 and len(self.y_t) < 2:
            return
        
        self.initial_state = np.array([self.x_t[-1], self.y_t[-1], self.theta, self.v_init, self.delta_init])  # Example initial state
        
        optimal_controls, predicted_vel, predicted_states = self.solve(self.initial_state)

        control_input = optimal_controls[:, 0]
        steering_rate = control_input[0]  # Steering angle
        a = control_input[1] # Acceleration
        self.v = predicted_vel[0]  # Predicted velocity

        self.predicted_states = predicted_states
        # orientation = predicted_states[2,:]
        # x_pose = predicted_states[0,:]
        # tot = 0.0
        # for i in range(len(x_pose)-1):
        #     path_slope = 3*2.0*math.pi*math.cos(x_pose[i]*2.0*math.pi/7.5)/7.5
        #     path_slope = math.atan(path_slope)
        #     tot += path_slope - orientation[i]
        # self.get_logger().info(f'orientation: {tot}')
        self.a_c = a
        self.steer_c = steering_rate
        #self.get_logger().info(f'shape a_c: {self.a_c[0]}\nshape steer_c: {self.steer_c[0]}')

        self.v_init += a * self.dt
        vel_input = self.v_init
        #self.get_logger().info(f'\nv: {self.v}\n a: {a}\n vel_input: {vel_input}\n')

        if steering_rate > np.deg2rad(10.0):
            steering_rate = np.deg2rad(10.0)
        elif steering_rate < -np.deg2rad(10.0):
            steering_rate = -np.deg2rad(10.0)
        else:
            steering_rate = steering_rate

        self.delta_init += steering_rate * self.dt
        delta_input = self.delta_init
        #self.get_logger().info(f'\nsteering_rate: {steering_rate*180/np.pi}\n delta_input: {delta_input*180/np.pi}\n')

        # Publish the steering angle
        float64_msg = Float64MultiArray()
        float64_msg.data = [delta_input]
        self.position_cmd_publisher_.publish(float64_msg)
        self.rate_publisher.publish(Float64(data=steering_rate*180/np.pi))
        self.steer_publisher.publish(Float64(data=delta_input*180/np.pi))

        # Publish the velocity
        velocity_msg = Float64MultiArray()
        self.w = vel_input / self.wheel_rad
        w_r = self.w * math.cos(delta_input)
        w_f = self.w
        velocity_msg.data = [float(w_r), float(w_f)]
        self.velocity_cmd_publisher_.publish(velocity_msg)
        self.vel_publisher.publish(Float64(data=vel_input))

        self.initial_state = np.array([self.x_t[-1], self.y_t[-1], self.theta, vel_input, delta_input])
        self.publish_predicted_states(self.predicted_states)

        cte, epsi = self.calculate_errors(self.x_t[-1], self.y_t[-1], self.theta)

        self.cte_values.append(cte)

        if len(self.cte_values) > 1:  # Change to a condition that suits your case
            print(f"Velocity: {self.v_ref}, Max CTE: {max(self.cte_values)}")

        

def main(args=None):
    rclpy.init(args=args)
    node = MPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
