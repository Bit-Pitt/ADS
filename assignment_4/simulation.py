import numpy as np

class Simulation:
    def __init__(self, lf, lr, mass, Iz, dt, integrator="euler", model="kinematic"):
        """
        Initialize the simulation parameters.
        """
        self.l_f = lf                   # Distance to front axle (m)
        self.l_r = lr                   # Distance to rear axle (m)
        self.l_wb = lf + lr
        self.mass = mass                # Vehicle mass (kg)
        self.I_z = Iz                   # Yaw moment of inertia (kg*m^2)
        self.dt = dt                    # Time step (s)
        self.integrator = integrator    # Integrator choice
        self.model = model              # Model choice
        
        # Aerodynamic and rolling resistance parameters
        self.rho = 1.225               # Air density (kg/m^3)
        self.C_d = 0.3                 # Drag coefficient (typical for cars)
        self.A = 2.2                   # Frontal area (m^2)
        self.C_rr = 0.015              # Rolling resistance coefficient

        # Initialize states
        self.x = 0                      # X position (m)
        self.y = 0                      # Y position (m)
        self.theta = 0                  # Heading angle (rad)
        self.vx = 0.0                     # Longitudinal velocity (m/s)
        self.vy = 0                     # Lateral velocity (m/s)
        self.r = 0                      # Yaw rate (rad/s)

        # Pacejka's Magic Formula coefficients
        self.B, self.C, self.D, self.E = 0, 0 , 0, 0
        self.B_f, self.C_f, self.D_f, self.E_f = self.B, self.C, self.D, self.E
        self.B_r, self.C_r, self.D_r, self.E_r = self.B, self.C, self.D, self.E
        
        self.Cf, self.Cr = self.B_f*self.C_f*self.D_f, self.B_r*self.C_r*self.D_r  # Cornering stiffness front/rear (N/rad)

    def kinematic_model(self, ax, delta):
        """ Kinematic single-track model equations of motion. """
        
        # Aerodynamic drag and rolling resistance forces
        F_aero = 0.0
        F_roll = self.C_rr * self.mass * 9.81
        
        dx = np.array([
            0,
            0,
            0,
            0,
            0,
            0
        ])
        return dx

    def linear_single_track_model(self, ax, delta):
        """ Linear single-track model with aerodynamic and rolling resistance. """
        
        # Tire slip angles
        alpha_f = 0#
        alpha_r = 0#

        # Vertical forces (nominal vertical load)
        Fz_f_nominal = 0#
        Fz_r_nominal = 0#

        # Front and rear lateral forces
        Fyf = 0#
        Fyr = 0#

        # Aerodynamic drag and rolling resistance forces
        F_aero = 0#
        F_roll = self.C_rr * self.mass * 9.81

        # Dynamics equations
        dx = np.array([
            0,  # dx/dt
            0,  # dy/dt
            0,                                                      # dtheta/dt
            0,       # dvx/dt with resistive forces
            0,                  # dvy/dt
            0                # dr/dt
        ])
        
        return dx

    def nonlinear_single_track_model(self, ax, delta):
        """ Nonlinear single-track model with aerodynamic and rolling resistance. """
        
        # Tire slip angles
        alpha_f = 0#
        alpha_r = 0#

        # Vertical forces (nominal vertical load)
        Fz_f_nominal = 0#
        Fz_r_nominal = 0#

        # Front and rear lateral forces
        Fyf = 0#
        Fyr = 0#

        # Aerodynamic drag and rolling resistance forces
        F_aero = 0#
        F_roll = self.C_rr * self.mass * 9.81

        # Dynamics equations
        dx = np.array([
            0,  # dx/dt
            0,  # dy/dt
            0,                                                      # dtheta/dt
            0,       # dvx/dt with resistive forces
            0,                  # dvy/dt
            0                # dr/dt
        ])
        
        return dx

    def integrate(self, ax, delta):
        """ Select the integrator method and apply it to update the state. """
        if self.integrator == "euler":
            self.euler_step(ax, delta)
        elif self.integrator == "rk4":
            self.rk4_step(ax, delta)

    def euler_step(self, ax, delta):
        """ Euler integration method. """
        dx = self.compute_dx(ax, delta)
        self.update_state(dx)

    def rk4_step(self, ax, delta):
        """ Runge-Kutta 4th order integration method. """
        k1 = self.compute_dx(ax, delta)
        self.update_state(k1, scale=0.5)
        
        k2 = self.compute_dx(ax, delta)
        self.update_state(k2, scale=0.5, revert=k1)
        
        k3 = self.compute_dx(ax, delta)
        self.update_state(k3, scale=1, revert=k2)

        k4 = self.compute_dx(ax, delta)
        
        # Combine k1, k2, k3, k4 for RK4 update
        dx = (k1 + 2*k2 + 2*k3 + k4) / 6
        self.update_state(dx)

    def compute_dx(self, ax, delta):
        """ Compute the state derivatives using the chosen model. """
        if self.model == "kinematic":
            return self.kinematic_model(ax, delta)
        elif self.model == "linear":
            return self.linear_single_track_model(ax, delta)
        elif self.model == "nonlinear":
            return self.nonlinear_single_track_model(ax, delta)

    def update_state(self, dx, scale=1, revert=None):
        """ Update state with scaled dx. Optionally revert previous state for RK4. """
        if revert is not None:
            self.x -= revert[0] * self.dt
            self.y -= revert[1] * self.dt
            self.theta -= revert[2] * self.dt
            self.vx -= revert[3] * self.dt
            self.vy -= revert[4] * self.dt
            self.r -= revert[5] * self.dt

        self.x += dx[0] * self.dt * scale
        self.y += dx[1] * self.dt * scale
        self.theta += dx[2] * self.dt * scale
        self.vx += dx[3] * self.dt * scale
        self.vy += dx[4] * self.dt * scale
        self.r += dx[5] * self.dt * scale
