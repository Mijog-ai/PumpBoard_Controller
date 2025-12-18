"""
Quick Start Example - PID Simulation

This is the simplest way to get started with PID simulation.
Just run this file to see a basic example!

Usage:
    python quick_start.py
"""

import numpy as np
import matplotlib.pyplot as plt
from pid_controller import PIDController
from plant_models import PumpPressureSystem


def main():
    print("=" * 60)
    print("Quick Start - PID Controller Simulation")
    print("=" * 60)
    print()
    print("This example simulates a pump pressure control system")
    print("similar to your PumpBoard controller hardware!")
    print()

    # Simulation parameters
    sample_time = 0.01  # 10ms (100Hz like many embedded systems)
    duration = 15.0     # 15 seconds
    time = np.arange(0, duration, sample_time)

    # Create the pump system (like your hardware!)
    pump = PumpPressureSystem(
        pump_efficiency=0.8,        # How well pump converts input to pressure
        leakage_coefficient=0.05,   # System leakage rate
        pressure_capacity=100.0,    # Max pressure (bar)
        initial_pressure=0.0        # Starting at 0
    )

    # Create PID controller
    # These gains are tuned for this pump system
    pid = PIDController(
        Kp=5.0,                     # Proportional gain
        Ki=1.5,                     # Integral gain
        Kd=0.5,                     # Derivative gain
        setpoint=60.0,              # Target pressure = 60 bar
        output_limits=(0, 100),     # Pump power range
        sample_time=sample_time
    )

    print(f"PID Parameters: Kp={pid.Kp}, Ki={pid.Ki}, Kd={pid.Kd}")
    print(f"Target Pressure: {pid.setpoint} bar")
    print(f"Simulating for {duration} seconds...")
    print()

    # Storage for results
    setpoint_history = []
    pressure_history = []
    control_history = []

    # Simulation loop
    for t in time:
        # Change setpoint at t=10s to test tracking
        if t >= 10.0:
            pid.set_setpoint(80.0)  # Increase to 80 bar

        # Record setpoint
        setpoint_history.append(pid.setpoint)

        # Get current pressure
        current_pressure = pump.value

        # Calculate control output from PID
        control_output = pid.update(current_pressure)

        # Apply control to pump
        pump.update(control_output)

        # Record data
        pressure_history.append(current_pressure)
        control_history.append(control_output)

    # Convert to numpy arrays for easier plotting
    setpoint_history = np.array(setpoint_history)
    pressure_history = np.array(pressure_history)
    control_history = np.array(control_history)

    print("Simulation complete!")
    print()
    print("Performance Summary:")
    print(f"  Final Pressure: {pressure_history[-1]:.2f} bar")
    print(f"  Final Setpoint: {setpoint_history[-1]:.2f} bar")
    print(f"  Final Error: {setpoint_history[-1] - pressure_history[-1]:.2f} bar")
    print()

    # Create plots
    fig, axes = plt.subplots(3, 1, figsize=(12, 9))
    fig.suptitle('Pump Pressure Control - PID Simulation', fontsize=14, fontweight='bold')

    # Plot 1: Pressure vs Setpoint
    axes[0].plot(time, setpoint_history, 'r--', linewidth=2, label='Setpoint (Target)', alpha=0.7)
    axes[0].plot(time, pressure_history, 'b-', linewidth=1.5, label='Actual Pressure')
    axes[0].set_ylabel('Pressure (bar)', fontsize=11)
    axes[0].set_title('System Response: Pressure Tracking')
    axes[0].legend(loc='best')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlim([0, duration])

    # Plot 2: Tracking Error
    error = setpoint_history - pressure_history
    axes[1].plot(time, error, 'g-', linewidth=1.5)
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[1].set_ylabel('Error (bar)', fontsize=11)
    axes[1].set_title('Tracking Error')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_xlim([0, duration])

    # Plot 3: Control Output (Pump Power)
    axes[2].plot(time, control_history, 'm-', linewidth=1.5)
    axes[2].set_xlabel('Time (seconds)', fontsize=11)
    axes[2].set_ylabel('Pump Power (%)', fontsize=11)
    axes[2].set_title('Controller Output')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_xlim([0, duration])

    plt.tight_layout()

    print("=" * 60)
    print("Close the plot window to exit.")
    print()
    print("Next steps:")
    print("  - Try running: python interactive_tuning.py")
    print("    to experiment with different PID gains!")
    print()
    print("  - Run: python simulation.py")
    print("    to see more examples with different systems!")
    print("=" * 60)

    plt.show()


if __name__ == "__main__":
    main()
