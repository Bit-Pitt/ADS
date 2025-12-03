import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from simulation import Simulation

def plot_comparison(results, labels, title, xlabel, ylabel):
    """ Plot comparison of results for a specific state variable. """
    plt.figure(figsize=(10, 6))
    for i, result in enumerate(results):
        plt.plot(result, label=labels[i])
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_trajectory(x_vals, y_vals, labels):
    """ Plot 2D trajectory (x vs y) for all simulation configurations. """
    plt.figure(figsize=(10, 6))
    for i in range(len(x_vals)):
        plt.plot(x_vals[i], y_vals[i], label=labels[i])
    plt.title("2D Trajectory Comparison")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()

def run_simulation(ax, steer, dt, integrator, model, steps=500):
    """ Run a simulation with the given parameters and return all states. """
    # Vehicle parameters
    lf = 1.156          # Distance from COG to front axle (m)
    lr = 1.42           # Distance from COG to rear axle (m)
    mass = 1200         # Vehicle mass (kg)
    Iz = 1792           # Yaw moment of inertia (kg*m^2)

    # Initialize the simulation
    sim = Simulation(lf, lr, mass, Iz, dt, integrator=integrator, model=model)

    # Storage for state variables and slip angles
    x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals = [], [], [], [], [], []
    alpha_f_vals, alpha_r_vals = [], []  # Slip angles

    # Max steer and frequency for sinusoidal steer commands
    steer_max = 0.1
    frequency = 0.5

    for step in range(steps):

        # Make one step simulation via model integration
        # Calculate sinusoidal steering angle
        #time = step * dt
        # steer = steer_max * np.sin(2 * np.pi * frequency * time)  # Sinusoidal steering angle

        sim.integrate(ax, steer)
        
        # Append each state to corresponding list
        x_vals.append(sim.x)
        y_vals.append(sim.y)
        theta_vals.append(sim.theta)
        vx_vals.append(sim.vx)
        vy_vals.append(sim.vy)
        r_vals.append(sim.r)

        # Calculate slip angles for front and rear tires
        alpha_f = 0.0  # Front tire slip angle
        alpha_r = 0.0         # Rear tire slip angle

        alpha_f_vals.append(alpha_f)
        alpha_r_vals.append(alpha_r)

    return x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals, alpha_f_vals, alpha_r_vals

def main():
    # Simulation parameters
    dt = 0.0        # Time step (s)
    ax = 0.0            # Constant longitudinal acceleration (m/s^2)
    steer = 0.0         # Constant steering angle (rad)
    sim_time = 0.0      # Simulation duration in seconds
    steps = int(sim_time / dt)  # Simulation steps (30 seconds)

    # List of configurations
    configs = [
        ("rk4", "kinematic"),
        ("rk4", "linear"),
        ("rk4", "nonlinear"),
        ("euler", "kinematic"),
        ("euler", "linear"),
        ("euler", "nonlinear")
    ]

    # Run each simulation and store the results
    all_results = []
    labels = []
    for integrator, model in configs:
        results = run_simulation(ax, steer, dt, integrator, model, steps)
        all_results.append(results)
        labels.append(f"{integrator.capitalize()} - {model.capitalize()}")

    # Separate each state for plotting
    x_results = [result[0] for result in all_results]
    y_results = [result[1] for result in all_results]
    theta_results = [result[2] for result in all_results]
    vx_results = [result[3] for result in all_results]
    vy_results = [result[4] for result in all_results]
    r_results = [result[5] for result in all_results]
    alpha_f_results = [result[6] for result in all_results]
    alpha_r_results = [result[7] for result in all_results]

    # Plot comparisons for each state variable
    plot_trajectory(x_results, y_results, labels)
    plot_comparison(theta_results, labels, "Heading Angle Comparison", "Time Step", "Heading Angle (rad)")
    plot_comparison(vx_results, labels, "Longitudinal Velocity Comparison", "Time Step", "Velocity (m/s)")
    plot_comparison(vy_results, labels, "Lateral Velocity Comparison", "Time Step", "Lateral Velocity (m/s)")
    plot_comparison(r_results, labels, "Yaw Rate Comparison", "Time Step", "Yaw Rate (rad/s)")
    plot_comparison(alpha_f_results, labels, "Front Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Front")
    plot_comparison(alpha_r_results, labels, "Rear Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Rear")

if __name__ == "__main__":
    main()
