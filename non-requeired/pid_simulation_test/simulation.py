"""
PID Simulation Runner

This script lets you run PID control simulations with different systems
and visualize the results to understand how PID works.

Usage:
    python simulation.py

You can modify the parameters at the bottom to experiment with different
PID gains and systems!
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from pid_controller import PIDController, PIDControllerWithVelocity
from plant_models import (
    FirstOrderSystem,
    SecondOrderSystem,
    PumpPressureSystem,
    WaterTankSystem,
    MotorSpeedSystem
)


class PIDSimulation:
    """
    Main simulation class that runs PID control loop and visualizes results
    """

    def __init__(self, controller, plant, sample_time=0.01, duration=10.0):
        """
        Args:
            controller: PIDController instance
            plant: Plant model instance (system to control)
            sample_time: Time step for simulation (seconds)
            duration: Total simulation time (seconds)
        """
        self.controller = controller
        self.plant = plant
        self.sample_time = sample_time
        self.duration = duration

        # Time array
        self.num_steps = int(duration / sample_time)
        self.time = np.arange(0, duration, sample_time)

        # Data storage
        self.setpoint_history = []
        self.process_value_history = []

    def run(self, setpoint_profile=None, disturbance_profile=None, noise_level=0.0):
        """
        Run the PID control simulation

        Args:
            setpoint_profile: Function of time that returns setpoint
                             If None, uses controller's fixed setpoint
            disturbance_profile: Function of time that returns disturbance
                                (added to plant output)
            noise_level: Standard deviation of measurement noise

        Returns:
            Dictionary with time, setpoint, and process value arrays
        """
        # Reset controller and plant
        self.controller.reset()
        self.plant.reset()

        self.setpoint_history = []
        self.process_value_history = []

        # Run simulation loop
        for i, t in enumerate(self.time):
            # Update setpoint if profile is provided
            if setpoint_profile is not None:
                self.controller.set_setpoint(setpoint_profile(t))

            # Get current setpoint
            current_setpoint = self.controller.setpoint
            self.setpoint_history.append(current_setpoint)

            # Get current process value (with optional noise)
            current_pv = self.plant.add_noise(noise_level)

            # Calculate control output
            control_output = self.controller.update(current_pv)

            # Update plant with control output
            next_pv = self.plant.update(control_output)

            # Add disturbance if provided
            if disturbance_profile is not None:
                disturbance = disturbance_profile(t)
                self.plant.value += disturbance

            self.process_value_history.append(current_pv)

        return {
            'time': self.time,
            'setpoint': np.array(self.setpoint_history),
            'process_value': np.array(self.process_value_history)
        }

    def plot_results(self, title="PID Control Simulation", save_path=None):
        """
        Create comprehensive plots of the simulation results

        Args:
            title: Plot title
            save_path: Optional path to save figure
        """
        fig = plt.figure(figsize=(14, 10))
        gs = GridSpec(3, 2, figure=fig, hspace=0.3, wspace=0.3)

        # 1. Main plot: Setpoint vs Process Value
        ax1 = fig.add_subplot(gs[0, :])
        ax1.plot(self.time, self.setpoint_history, 'r--', linewidth=2, label='Setpoint', alpha=0.7)
        ax1.plot(self.time, self.process_value_history, 'b-', linewidth=1.5, label='Process Value')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Value')
        ax1.set_title(f'{title}\nKp={self.controller.Kp}, Ki={self.controller.Ki}, Kd={self.controller.Kd}')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # 2. Error over time
        ax2 = fig.add_subplot(gs[1, 0])
        ax2.plot(self.time, self.controller.error_history, 'g-', linewidth=1.5)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Error')
        ax2.set_title('Tracking Error')
        ax2.grid(True, alpha=0.3)

        # 3. Control output
        ax3 = fig.add_subplot(gs[1, 1])
        ax3.plot(self.time, self.controller.output_history, 'm-', linewidth=1.5)
        ax3.axhline(y=self.controller.output_limits[0], color='r', linestyle='--', alpha=0.5, label='Limits')
        ax3.axhline(y=self.controller.output_limits[1], color='r', linestyle='--', alpha=0.5)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Control Output')
        ax3.set_title('Controller Output')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # 4. P, I, D components
        ax4 = fig.add_subplot(gs[2, 0])
        ax4.plot(self.time, self.controller.p_term_history, label='P term', alpha=0.7)
        ax4.plot(self.time, self.controller.i_term_history, label='I term', alpha=0.7)
        ax4.plot(self.time, self.controller.d_term_history, label='D term', alpha=0.7)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Component Value')
        ax4.set_title('PID Components')
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        # 5. Performance metrics
        ax5 = fig.add_subplot(gs[2, 1])
        ax5.axis('off')

        # Calculate performance metrics
        error_array = np.array(self.controller.error_history)
        pv_array = np.array(self.process_value_history)
        sp_array = np.array(self.setpoint_history)

        # Rise time (10% to 90% of final value)
        final_sp = sp_array[-1]
        rise_start = np.argmax(pv_array > 0.1 * final_sp) if final_sp != 0 else 0
        rise_end = np.argmax(pv_array > 0.9 * final_sp) if final_sp != 0 else 0
        rise_time = (rise_end - rise_start) * self.sample_time if rise_end > rise_start else 0

        # Settling time (within 5% of setpoint)
        tolerance = 0.05 * abs(final_sp) if final_sp != 0 else 0.05
        settled = np.abs(error_array) < tolerance
        if np.any(settled):
            settling_idx = np.where(settled)[0][-1]
            # Check if it stays settled
            if np.all(settled[settling_idx:]):
                settling_time = settling_idx * self.sample_time
            else:
                settling_time = self.duration
        else:
            settling_time = self.duration

        # Overshoot
        if final_sp != 0:
            overshoot = (np.max(pv_array) - final_sp) / final_sp * 100
        else:
            overshoot = 0

        # Steady-state error
        steady_state_error = np.mean(error_array[-int(1.0/self.sample_time):])  # Last 1 second

        # IAE (Integral Absolute Error)
        iae = np.sum(np.abs(error_array)) * self.sample_time

        # ITAE (Integral Time-weighted Absolute Error)
        itae = np.sum(self.time * np.abs(error_array)) * self.sample_time

        metrics_text = f"""
        Performance Metrics:

        Rise Time: {rise_time:.3f} s
        Settling Time: {settling_time:.3f} s
        Overshoot: {overshoot:.2f} %
        Steady-State Error: {steady_state_error:.4f}

        IAE: {iae:.3f}
        ITAE: {itae:.3f}

        Max Control Output: {np.max(self.controller.output_history):.2f}
        Min Control Output: {np.min(self.controller.output_history):.2f}
        """

        ax5.text(0.1, 0.5, metrics_text, fontsize=10, family='monospace',
                verticalalignment='center')

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Plot saved to {save_path}")

        plt.tight_layout()
        return fig


def step_setpoint(step_time=1.0, initial_value=0.0, final_value=100.0):
    """Generate a step change in setpoint"""
    def profile(t):
        return final_value if t >= step_time else initial_value
    return profile


def ramp_setpoint(start_time=0.0, ramp_rate=10.0, initial_value=0.0):
    """Generate a ramp setpoint"""
    def profile(t):
        if t < start_time:
            return initial_value
        return initial_value + ramp_rate * (t - start_time)
    return profile


def sine_setpoint(amplitude=50.0, frequency=0.5, offset=50.0):
    """Generate a sinusoidal setpoint"""
    def profile(t):
        return offset + amplitude * np.sin(2 * np.pi * frequency * t)
    return profile


# Example simulation functions
def example_temperature_control():
    """Example: Temperature control with first-order system"""
    print("\n=== Temperature Control Example ===")
    print("System: Heating element with thermal mass")
    print("Goal: Heat to 100°C and maintain temperature\n")

    # Create plant (heater with time constant of 2 seconds)
    plant = FirstOrderSystem(time_constant=2.0, gain=1.0, initial_value=20.0)

    # Create PID controller
    # Start with just P control
    pid = PIDController(Kp=2.0, Ki=0.0, Kd=0.0, setpoint=100.0,
                       output_limits=(0, 200), sample_time=0.01)

    # Run simulation
    sim = PIDSimulation(pid, plant, sample_time=0.01, duration=15.0)
    results = sim.run()

    fig = sim.plot_results("Temperature Control - P Only")

    return sim, fig


def example_motor_position():
    """Example: Motor position control with second-order system"""
    print("\n=== Motor Position Control Example ===")
    print("System: DC motor with inertia and friction")
    print("Goal: Move to position 50 units\n")

    # Create plant (motor with inertia)
    plant = SecondOrderSystem(mass=1.0, damping=5.0, stiffness=20.0,
                             gain=1.0, initial_value=0.0)

    # Create PID controller with all three terms
    pid = PIDController(Kp=15.0, Ki=2.0, Kd=8.0, setpoint=50.0,
                       output_limits=(-100, 100), sample_time=0.01)

    # Run simulation
    sim = PIDSimulation(pid, plant, sample_time=0.01, duration=10.0)
    results = sim.run()

    fig = sim.plot_results("Motor Position Control - PID")

    return sim, fig


def example_pump_pressure():
    """Example: Pump pressure control (like your hardware!)"""
    print("\n=== Pump Pressure Control Example ===")
    print("System: Hydraulic pump with leakage")
    print("Goal: Maintain pressure at 60 bar\n")

    # Create plant (pump system)
    plant = PumpPressureSystem(pump_efficiency=0.8, leakage_coefficient=0.05,
                              pressure_capacity=100.0, initial_pressure=0.0)

    # Create PID controller
    pid = PIDController(Kp=5.0, Ki=1.5, Kd=0.5, setpoint=60.0,
                       output_limits=(0, 100), sample_time=0.01)

    # Run simulation with step change at t=5s
    sim = PIDSimulation(pid, plant, sample_time=0.01, duration=15.0)
    results = sim.run(setpoint_profile=step_setpoint(step_time=5.0, initial_value=60.0, final_value=80.0))

    fig = sim.plot_results("Pump Pressure Control - Setpoint Change")

    return sim, fig


def example_comparison():
    """Compare different PID tunings side by side"""
    print("\n=== PID Tuning Comparison ===")
    print("Comparing P, PI, and PID control on same system\n")

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    configs = [
        ("P Control Only", {"Kp": 10.0, "Ki": 0.0, "Kd": 0.0}),
        ("PI Control", {"Kp": 10.0, "Ki": 2.0, "Kd": 0.0}),
        ("PID Control", {"Kp": 10.0, "Ki": 2.0, "Kd": 5.0}),
    ]

    for idx, (name, gains) in enumerate(configs):
        # Create fresh plant and controller
        plant = SecondOrderSystem(mass=1.0, damping=3.0, stiffness=15.0)
        pid = PIDController(**gains, setpoint=50.0, output_limits=(-100, 100))

        # Run simulation
        sim = PIDSimulation(pid, plant, sample_time=0.01, duration=8.0)
        results = sim.run()

        # Plot on subplot
        ax = axes[idx]
        ax.plot(sim.time, sim.setpoint_history, 'r--', label='Setpoint', linewidth=2, alpha=0.7)
        ax.plot(sim.time, sim.process_value_history, 'b-', label='Process Value', linewidth=1.5)
        ax.set_ylabel('Value')
        ax.set_title(f'{name} (Kp={gains["Kp"]}, Ki={gains["Ki"]}, Kd={gains["Kd"]})')
        ax.legend()
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()

    return fig


if __name__ == "__main__":
    print("=" * 60)
    print("PID Controller Educational Simulation")
    print("=" * 60)

    # Run examples
    sim1, fig1 = example_temperature_control()
    sim2, fig2 = example_motor_position()
    sim3, fig3 = example_pump_pressure()
    fig4 = example_comparison()

    print("\n" + "=" * 60)
    print("Simulations complete! Close the plot windows to exit.")
    print("=" * 60)

    plt.show()
