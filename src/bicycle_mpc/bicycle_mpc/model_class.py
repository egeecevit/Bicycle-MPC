import casadi as ca
import numpy as np

class BicycleModelMPC:
    def __init__(self, dt=0.1, L=0.4, N=10):
        """
        Initialize the Bicycle Model MPC class.

        Parameters:
        dt (float): Time step for discrete updates.
        Lf (float): Distance between the front wheel and rear wheel.
        N (int): Prediction horizon.
        v_ref (float): Reference velocity.
        """
        self.dt = dt
        self.L = L
        self.N = N
        
        # Define the state and control variables
        self.define_variables()
        
        # Define the model equations and constraints
        self.setup_model()

    def define_variables(self):
        """Define state and control variables."""
        self.x = ca.MX.sym('x')
        self.y = ca.MX.sym('y')
        self.psi = ca.MX.sym('psi')
        self.v = ca.MX.sym('v')
        self.cte = ca.MX.sym('cte')
        self.epsi = ca.MX.sym('epsi')
        
        # State vector
        self.state = ca.vertcat(self.x, self.y, self.psi, self.v, self.cte, self.epsi)
        
        # Control variables
        self.delta = ca.MX.sym('delta')  # Steering angle
        self.a = ca.MX.sym('a')          # Acceleration
        
        # Control vector
        self.control = ca.vertcat(self.delta, self.a)

    def setup_model(self):
        """Set up the model equations and constraints."""
        # Define state transition equations
        x_next = self.x + self.v * ca.cos(self.psi) * self.dt
        y_next = self.y + self.v * ca.sin(self.psi) * self.dt
        psi_next = self.psi + (self.v / self.L) * self.delta * self.dt
        v_next = self.v + self.a * self.dt
        cte_next = self.cte + (self.v * ca.sin(self.epsi) * self.dt)
        epsi_next = self.epsi + (self.v / self.L) * self.delta * self.dt
        
        # Create the state transition function
        self.f = ca.Function('f', [self.state, self.control], 
                             [ca.vertcat(x_next, y_next, psi_next, v_next, cte_next, epsi_next)])
    
    
    def solve(self, initial_state):
        """
        Solve the MPC problem given the initial state.

        Parameters:
        initial_state (list or np.array): Initial state [x, y, psi, v, cte, epsi].

        Returns:
        np.array: Optimal control inputs trajectory.
        """
        # Define prediction horizon variables
        X = ca.MX.sym('X', 6, self.N+1)  # State trajectory
        U = ca.MX.sym('U', 2, self.N)    # Control trajectory

        # Initialize cost and constraints
        cost = 0
        g = []

        # Define the Q and R matrices for state and control penalties
        Q = ca.diagcat(1.0, 1.0, 1.0, 1.0, 50.0, 50.0)  # cte and epsi are weighted more because they are the main things to be penalized
        R = ca.diagcat(0.1, 0.1)  # Adjust weights for control inputs

        # Initial state constraint
        g.append(X[:, 0] - initial_state)  # Ensure X[:, 0] == initial_state

        # Loop through each step in the prediction horizon
        for k in range(self.N):
            # State cost: penalize deviations from desired state
            x = X[:, k]
            cost += ca.mtimes([x.T, Q, x])

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
        vel_limit = [0, 5.0]
        cte_limit = [0, 0.5]
        epsi_limit = np.deg2rad(10)

        # State bounds: [x, y, psi, v, cte, epsi]
        lb_states = ca.DM([-ca.inf, -ca.inf, -ca.inf, vel_limit[0], cte_limit[0], -epsi_limit])  
        ub_states = ca.DM([ca.inf, ca.inf, ca.inf, vel_limit[1], cte_limit[1], epsi_limit])

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
        num_states = 6 * (self.N + 1)  # 6 states per step, N steps
        lb_decision_vars[6: num_states] = lb_states_full  # Skip initial state (first 6 entries)
        ub_decision_vars[6: num_states] = ub_states_full

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
            },
            'print_time': 0
        }
        solver = ca.nlpsol('solver', 'ipopt', nlp, options)

        # Initial guess for the decision variables
        x0_decision_vars = ca.DM.zeros(decision_vars.shape)
        '''-------------------------'''
        # Create a list to hold the state trajectory guesses, starting with the initial state
        # state_guess = [initial_state]

        # # Propagate the initial state forward using zero control inputs
        # for _ in range(self.N):
        #     # Predict next state assuming zero steering (delta) and zero acceleration (a)
        #     next_state = self.f(state_guess[-1], [0, 0])  # Zero control input
        #     state_guess.append(next_state.full().flatten())

        # # Flatten the state trajectory guesses into a single column vector
        # state_guess = np.hstack(state_guess).flatten()

        # # Control input guess: All zeros (no control action)
        # control_guess = np.zeros(2 * self.N)  # [delta, a] for N steps

        # # Combine state and control guesses into the decision variable vector
        # x0_decision_vars = np.hstack([state_guess, control_guess]).reshape(-1, 1)

        '''-------------------------'''

        # Solve the optimization problem
        solution = solver(x0=x0_decision_vars, 
                        lbx=lb_decision_vars, 
                        ubx=ub_decision_vars, 
                        lbg=lb_g, 
                        ubg=ub_g)

        # Extract the optimal control inputs
        optimal_controls = solution['x'][num_states:num_states + 2 * self.N]
        optimal_controls = ca.reshape(optimal_controls, 2, self.N)

        # Extract the predicted state trajectory (including velocity)
        predicted_states = solution['x'][:num_states]
        predicted_states = ca.reshape(predicted_states, 6, self.N + 1)

        # Extract the velocity trajectory from the predicted states
        predicted_velocity = predicted_states[3, :]  # Extract the velocity component (4th row)

        return optimal_controls.full(), predicted_velocity.full(), predicted_states.full() # Convert to a numpy array optimal_controls[:,0] is the optimal control for the first step
        
       

