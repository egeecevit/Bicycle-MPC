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
        self.dt = 0.1
        self.L = 0.4
        self.N = 50
        
        

        self.x_t = []
        self.y_t = []

        # Don't know why but published path has duplicate values
        self.x = list(np.arange(-30.1, 50.1, 0.1))
        self.y = []
        self.v_ref = [1.0] * (len(self.x) - 50) + [0.0] * 50
        for num in self.x:
            self.y.append(3*math.sin(num * 2.0 * math.pi / 20.0))
            #self.y.append(0.0)
            #self.y.append(num)
            #self.y.append(num*math.tan(30*math.pi/180))

        self.bicyc_length = 0.4
        self.wheel_rad = 0.05
        self.v = None
        self.w = None
        self.odom_status = None
        self.theta = 0.0
        self.odom_timestamp = None
        self.delta_c = 0.0
        self.a_c = 0.0
        self.v_init = 0.0
        self.predicted_states = None
        self.initial_state = None
        self.spline_func = None
        self.derivative_func = None
        self.spline_and_slope_func = None

        self.define_variables()
        self.setup_model()
        self.spline(self.x, self.y)

        self.timer_ = self.create_timer(1.0/30.0, self.controller_callback)
        self.get_logger().info("Controller node has been started.")


    # def control_loop(self):
    #     start_time = time.time()
    #     optimal_controls, predicted_vel, predicted_states = self.mpc.solve(self.initial_state)

    #     control_input = optimal_controls[:, 0]
    #     delta = control_input[0]  # Steering angle
    #     a = control_input[1] # Acceleration
    #     self.v = predicted_vel[0]  # Predicted velocity
    #     # print(f'predicted_x = {predicted_states[0,0]}\n, predicted_y = {predicted_states[1,0]}\n, predicted_psi = {predicted_states[2,0]}\n, predicted_v = {predicted_states[3,0]}\n, predicted_cte = {predicted_states[4,0]}\n, predicted_epsi = {predicted_states[5,0]}\n')
    #     # print("-----------------------------------")
    #     end_time = time.time()
    #     #self.get_logger().info(f"Control loop duration: {end_time - start_time:.4f} seconds")

    #     self.delta = delta
    #     self.a = a
    #     self.predicted_states = predicted_states

    #     return delta, a, predicted_states

    #------------------------------------------------------------------------------------------------#
    #CASADI NONLINEAR SOLVER PART#
    def define_variables(self):
        """Define state and control variables."""
        self.x_m = ca.MX.sym('x')
        self.y_m = ca.MX.sym('y')
        self.psi_m = ca.MX.sym('psi')
        self.v_m = ca.MX.sym('v')
        
        # State vector
        self.state = ca.vertcat(self.x_m, self.y_m, self.psi_m, self.v_m)
        
        # Control variables
        self.delta = ca.MX.sym('delta')  # Steering angle
        self.a = ca.MX.sym('a')          # Acceleration
        
        # Control vector
        self.control = ca.vertcat(self.delta, self.a)

    def setup_model(self):
        # Define state transition equations
        x_next = self.x_m + self.v_m * ca.cos(self.psi_m) * self.dt
        y_next = self.y_m + self.v_m * ca.sin(self.psi_m) * self.dt
        psi_next = self.psi_m + (self.v_m / self.L) * self.delta * self.dt
        v_next = self.v_m + self.a * self.dt

        # Expicitly define the cte and epsi which is not ideal
        # cte_next = 3*ca.sin(x_next * 2.0 * math.pi / 40.0) - y_next
        # epsi_next = ca.atan((2.0 * math.pi / 40.0) * 3 * ca.cos(x_next * 2.0 * math.pi / 40.0)) - psi_next
        # cte_next = self.y - self.y_m
        # epsi_next = ca.atan((2.0 * math.pi / 40.0) * 3 * ca.cos(x_next * 2.0 * math.pi / 40.0)) - psi_next
        # Include cte and epsi in the next state calculation
        # predicted_cte, predicted_epsi = self.calculate_errors_symbolic(x_next, y_next, psi_next)
        # cte_next = predicted_cte
        #epsi_next = predicted_epsi
        # self.get_logger().info(f'cte_next = {cte_next.shape}, epsi_next = {epsi_next.shape}, x_next = {x_next.shape}')
        #self.get_logger().info(f'y_next = {y_next.shape}, y_m = {self.y_m.shape}, path_y = {len(self.y)}')

        # Update the state transition function to include cte and epsi
        self.f = ca.Function('f', [self.state, self.control], 
                            [ca.vertcat(x_next, y_next, psi_next, v_next)])
        

    def solve(self, initial_state):
        """
        Solve the MPC problem given the initial state.

        Parameters:
        initial_state (list or np.array): Initial state [x, y, psi, v, cte, epsi].

        Returns:
        np.array: Optimal control inputs trajectory.
        """

        # Define prediction horizon variables
        X = ca.MX.sym('X', 4, self.N+1)  # State trajectory
        U = ca.MX.sym('U', 2, self.N)    # Control trajectory

        # Initialize cost and constraints
        cost = 0
        g = []

        # Define the Q and R matrices for state and control penalties
        Q = ca.diagcat(0.0, 0.0, 0.0, 1.0)  # cte and epsi are weighted more because they are the main things to be penalized
        R = ca.diagcat(10.0, 2.0)  # Adjust weights for control inputs

        # Initial state constraint
        g.append(X[:, 0] - initial_state)  # Ensure X[:, 0] == initial_state

        # Loop through each step in the prediction horizon
        for k in range(self.N):
            # State cost: penalize deviations from desired state
            x = X[:, k]

            cost += 5.0 * (x[3] - 1.2)**2  # Penalize velocity deviation from 1.0
            # cost += ca.mtimes([x.T, Q, x])


            # cte= 3*ca.sin(x[0] * 2.0 * math.pi / 40.0) - x[1]
            # epsi = ca.atan((2.0 * math.pi / 40.0) * 3 * ca.cos(x[0] * 2.0 * math.pi / 40.0)) - x[2]

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
        vel_limit = [0, 1.5]
        # cte_limit = 0.5
        # epsi_limit = np.deg2rad(10)

        # State bounds: [x, y, psi, v, cte, epsi]
        # lb_states = ca.DM([-ca.inf, -ca.inf, -ca.inf, vel_limit[0], -cte_limit, -epsi_limit])  
        # ub_states = ca.DM([ca.inf, ca.inf, ca.inf, vel_limit[1], cte_limit, epsi_limit])
        lb_states = ca.DM([-ca.inf, -ca.inf, -ca.inf, vel_limit[0]])  
        ub_states = ca.DM([ca.inf, ca.inf, ca.inf, vel_limit[1]])

        # Control bounds: [delta (steering angle), a (acceleration)]
        steering_angle_limit = np.deg2rad(30)  # 30 degrees in radians
        lb_controls = ca.DM([-steering_angle_limit, -1])  # [delta_min, a_min]
        ub_controls = ca.DM([steering_angle_limit, 1])    # [delta_max, a_max]

        # Apply state bounds only from the second state onward (skip initial state)
        lb_states_full = ca.repmat(lb_states, self.N, 1).reshape((-1, 1))  # N steps only
        ub_states_full = ca.repmat(ub_states, self.N, 1).reshape((-1, 1))

        # Control bounds as before
        lb_controls_full = ca.repmat(lb_controls, self.N, 1).reshape((-1, 1))
        ub_controls_full = ca.repmat(ub_controls, self.N, 1).reshape((-1, 1))

        # Adjust to apply state bounds starting from the second predicted state onward
        num_states = 4 * (self.N + 1)  # 6 states per step, N steps
        lb_decision_vars[4: num_states] = lb_states_full  # Skip initial state (first 6 entries)
        ub_decision_vars[4: num_states] = ub_states_full

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
        '''-------------------------THIS IS BETTER THAN ABOVE ONE-------------------------'''
        # # Create a list to hold the state trajectory guesses, starting with the initial state
        state_guess = [initial_state]

        # Propagate the initial state forward using zero control inputs
        for _ in range(self.N):
            # Predict next state assuming zero steering (delta) and zero acceleration (a)
            next_state = self.f(state_guess[-1], [0, 0])  # Zero control input # TODO current input? 
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
        predicted_states = ca.reshape(predicted_states, 4, self.N + 1)

        # Extract the velocity trajectory from the predicted states
        predicted_velocity = predicted_states[3, :]  # Extract the velocity component (4th row)

        return optimal_controls.full(), predicted_velocity.full(), predicted_states.full() # Convert to a numpy array optimal_controls[:,0] is the optimal control for the first step


    #------------------------------------------------------------------------------------------------#
    # def spline(self, x, y):
    #     self.spline_func = InterpolatedUnivariateSpline(x, y)
    #     self.derivative_func = self.spline_func.derivative()

        #return self.spline_func, self.derivative_func
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
        # self.get_logger().info(f'predicted_states[0]= {predicted_states[0,0]}, predicted_states[1]= {predicted_states[1,0]}')
        # self.get_logger().info(f'current_x = {self.x_t[-1]}, current_y = {self.y_t[-1]}')
        for i in range(predicted_states.shape[1]):  # Loop through each predicted state
            x = predicted_states[0, i]
            y = predicted_states[1, i]
            #self.get_logger().info(f'i = {i} predicted_cte = {predicted_states[4,i]}, predicted_epsi = {predicted_states[5,i]}')
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
    
    # def calculate_errors_symbolic(self, x_pred, y_pred, psi_pred):
    #     # Convert path coordinates to CasADi MX types for symbolic calculations
    #     path_x = ca.MX(self.x)  # CasADi symbolic path coordinates
    #     path_y = ca.MX(self.y)

    #     # Find the index of the closest point on the path symbolically
    #     min_distance = ca.inf
    #     min_index = 0

    #     for i in range(len(self.x)):
    #         distance = ca.sqrt((path_x[i] - x_pred)**2 + (path_y[i] - y_pred)**2)
    #         min_index = ca.if_else(distance < min_distance, i, min_index)
    #         min_distance = ca.if_else(distance < min_distance, distance, min_distance)

    #     # Calculate the slope and angle at the closest point symbolically
    #     idx_next = min_index + 1
    #     #idx_prev = ca.if_else(min_index > 0, min_index - 1, min_index)

    #     # path_slope = ca.if_else(
    #     #     min_index < len(self.x) - 1,
    #     #     (path_y[idx_next] - path_y[min_index]) / (path_x[idx_next] - path_x[min_index]),
    #     #     (path_y[min_index] - path_y[idx_prev]) / (path_x[min_index] - path_x[idx_prev])
    #     # )
    #     path_slope = ca.if_else(ca.fabs(path_x[idx_next] - path_x[min_index]) > 1e-3,  # Avoid division by very small numbers
    #         (path_y[idx_next] - path_y[min_index]) / (path_x[idx_next] - path_x[min_index]),
    #         0.0)
        
    #     path_slope = ca.if_else(path_slope == ca.inf, 0.0, path_slope)  # Avoid infinite slopes
        
    #     path_angle = ca.atan(path_slope)

    #     # Calculate heading error
    #     epsi = psi_pred - path_angle

    #     # Normalize EPSI within [-pi, pi] using CasADi functions
    #     epsi = ca.fmod(epsi + ca.pi, 2 * ca.pi) - ca.pi

    #     # Calculate the signed cross-track error
    #     dx = x_pred - path_x[min_index]
    #     dy = y_pred - path_y[min_index]
    #     cte = -dx * ca.sin(path_angle) + dy * ca.cos(path_angle)

    #     return cte, epsi
    
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
        #self.get_logger().info(f'slope = {spline_slope}')

        path_angle = ca.atan(spline_slope)  # Path angle from slope
        

        # Calculate heading error (epsi)
        epsi = psi_pred - path_angle

        # Normalize EPSI within [-pi, pi] using CasADi functions
        epsi = ca.fmod(epsi + ca.pi, 2 * ca.pi) - ca.pi


        # Calculate cross-track error (cte)
        cte = spline_value - y_pred  # Signed cross-track error

        return cte, epsi

    
    def controller_callback(self):
        
        if len(self.x) == 0 and len(self.y) == 0:
            return
        if self.odom_status is None:
            return
        # if self.v is None:
        #     return
        if len(self.x_t) < 2 and len(self.y_t) < 2:
            return
        
        # current_cte, current_epsi = self.calculate_errors(self.x_t[-1], self.y_t[-1], self.theta)
        self.initial_state = np.array([self.x_t[-1], self.y_t[-1], self.theta, self.v_init])  # Example initial state
        #self.get_logger().info(f'x_t = {self.x_t[-1]}, y_t = {self.y_t[-1]}, theta = {self.theta}, v = {self.v_init}')
        
        
        optimal_controls, predicted_vel, predicted_states = self.solve(self.initial_state)

        control_input = optimal_controls[:, 0]
        delta = control_input[0]  # Steering angle
        a = control_input[1] # Acceleration
        self.v = predicted_vel[0]  # Predicted velocity
        # print(f'predicted_x = {predicted_states[0,0]}\n, predicted_y = {predicted_states[1,0]}\n, predicted_psi = {predicted_states[2,0]}\n, predicted_v = {predicted_states[3,0]}\n, predicted_cte = {predicted_states[4,0]}\n, predicted_epsi = {predicted_states[5,0]}\n')
        # print("-----------------------------------")
        #self.get_logger().info(f'control input: {optimal_controls}')
        #self.get_logger().info(f'predicted_cte = {predicted_states[4,0]}')

        # self.delta = delta
        # self.a = a
        self.delta_c = delta
        self.a_c = a
        self.predicted_states = predicted_states

        # vel_input = (self.v[0] + self.v[-1])/2
        # vel_input += a * self.dt
        self.v_init += a * self.dt
        vel_input = self.v_init
        self.get_logger().info(f'\nv: {self.v}\n a: {a}\n vel_input: {vel_input}')

        # if self.predicted_states is None:
        #     return
        
        # if self.v is None:
        #     return
        #print(f'lenght of x: {len(self.x)}, lenght of y: {len(self.y)}')
        # Get the optimal control inputs
        #delta, a, predicted_states = self.control_loop() # Get the optimal control inputs

        # Publish the steering angle
        float64_msg = Float64MultiArray()
        float64_msg.data = [delta]
        self.position_cmd_publisher_.publish(float64_msg)

        # Publish the velocity
        velocity_msg = Float64MultiArray()
        self.w = vel_input / self.wheel_rad
        w_r = self.w * math.cos(delta)
        w_f = self.w
        velocity_msg.data = [float(w_r), float(w_f)]
        self.velocity_cmd_publisher_.publish(velocity_msg)

        self.initial_state = np.array([self.x_t[-1], self.y_t[-1], self.theta, vel_input])
        self.publish_predicted_states(self.predicted_states)

        #self.get_logger().info(f'steering angle: {delta}, predicted velocity: {self.v[0]}')



def main(args=None):
    rclpy.init(args=args)
    node = MPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
