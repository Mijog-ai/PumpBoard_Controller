"""
Interactive PID Tuning Tool

This tool provides real-time sliders to adjust PID parameters and see
the effects immediately. Great for learning and experimentation!

Usage:
    python interactive_tuning.py

Use the sliders to adjust Kp, Ki, Kd and see how the system responds!
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from pid_controller import PIDController
from plant_models import (
    FirstOrderSystem,
    SecondOrderSystem,
    PumpPressureSystem,
    WaterTankSystem,
    MotorSpeedSystem
)


class InteractivePIDTuner:
    """
    Interactive PID tuning interface with real-time visualization
    """

    def __init__(self):
        # Initial PID parameters
        self.Kp = 5.0
        self.Ki = 1.0
        self.Kd = 2.0
        self.setpoint = 50.0

        # Simulation parameters
        self.sample_time = 0.01
        self.duration = 10.0
        self.time = np.arange(0, self.duration, self.sample_time)

        # Current plant type
        self.plant_type = "Second Order System"
        self.plant = self._create_plant(self.plant_type)

        # Run initial simulation
        self.run_simulation()

        # Create the GUI
        self.create_gui()

    def _create_plant(self, plant_type):
        """Create a plant based on the selected type"""
        plants = {
            "First Order System": FirstOrderSystem(time_constant=2.0, gain=1.0, initial_value=0.0),
            "Second Order System": SecondOrderSystem(mass=1.0, damping=3.0, stiffness=15.0, initial_value=0.0),
            "Pump Pressure": PumpPressureSystem(pump_efficiency=0.8, leakage_coefficient=0.05, initial_pressure=0.0),
            "Water Tank": WaterTankSystem(tank_area=1.0, outlet_coefficient=0.1, initial_level=0.0),
            "Motor Speed": MotorSpeedSystem(inertia=0.01, friction=0.1, initial_speed=0.0)
        }
        return plants.get(plant_type, SecondOrderSystem())

    def run_simulation(self):
        """Run the PID simulation with current parameters"""
        # Create controller
        pid = PIDController(
            Kp=self.Kp,
            Ki=self.Ki,
            Kd=self.Kd,
            setpoint=self.setpoint,
            output_limits=(-100, 100),
            sample_time=self.sample_time
        )

        # Reset plant
        self.plant.reset()

        # Arrays to store results
        self.setpoint_array = []
        self.pv_array = []
        self.error_array = []
        self.output_array = []
        self.p_array = []
        self.i_array = []
        self.d_array = []

        # Simulation loop
        for t in self.time:
            # Update setpoint (step at t=1s)
            if t >= 1.0:
                pid.set_setpoint(self.setpoint)
            else:
                pid.set_setpoint(0.0)

            self.setpoint_array.append(pid.setpoint)

            # Get current value
            current_pv = self.plant.value

            # PID update
            control_output = pid.update(current_pv)

            # Update plant
            self.plant.update(control_output)

            # Store data
            self.pv_array.append(current_pv)
            self.error_array.append(pid.current_error)
            self.output_array.append(pid.output)

            components = pid.get_components()
            self.p_array.append(components['P'])
            self.i_array.append(components['I'])
            self.d_array.append(components['D'])

        # Convert to numpy arrays
        self.setpoint_array = np.array(self.setpoint_array)
        self.pv_array = np.array(self.pv_array)
        self.error_array = np.array(self.error_array)
        self.output_array = np.array(self.output_array)
        self.p_array = np.array(self.p_array)
        self.i_array = np.array(self.i_array)
        self.d_array = np.array(self.d_array)

    def create_gui(self):
        """Create the interactive GUI with sliders"""
        # Create figure with subplots
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.canvas.manager.set_window_title('Interactive PID Tuner')

        # Main plot area
        gs = self.fig.add_gridspec(4, 2, left=0.1, right=0.95, bottom=0.25,
                                  top=0.95, hspace=0.4, wspace=0.3)

        # Plot 1: Response
        self.ax_response = self.fig.add_subplot(gs[0:2, 0])
        self.line_setpoint, = self.ax_response.plot(self.time, self.setpoint_array, 'r--',
                                                    linewidth=2, label='Setpoint', alpha=0.7)
        self.line_pv, = self.ax_response.plot(self.time, self.pv_array, 'b-',
                                              linewidth=1.5, label='Process Value')
        self.ax_response.set_xlabel('Time (s)')
        self.ax_response.set_ylabel('Value')
        self.ax_response.set_title('System Response')
        self.ax_response.legend()
        self.ax_response.grid(True, alpha=0.3)

        # Plot 2: Error
        self.ax_error = self.fig.add_subplot(gs[2, 0])
        self.line_error, = self.ax_error.plot(self.time, self.error_array, 'g-', linewidth=1.5)
        self.ax_error.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.ax_error.set_xlabel('Time (s)')
        self.ax_error.set_ylabel('Error')
        self.ax_error.set_title('Tracking Error')
        self.ax_error.grid(True, alpha=0.3)

        # Plot 3: Control Output
        self.ax_output = self.fig.add_subplot(gs[3, 0])
        self.line_output, = self.ax_output.plot(self.time, self.output_array, 'm-', linewidth=1.5)
        self.ax_output.set_xlabel('Time (s)')
        self.ax_output.set_ylabel('Control Output')
        self.ax_output.set_title('Controller Output')
        self.ax_output.grid(True, alpha=0.3)

        # Plot 4: PID Components
        self.ax_components = self.fig.add_subplot(gs[0:2, 1])
        self.line_p, = self.ax_components.plot(self.time, self.p_array, 'r-', label='P', alpha=0.7)
        self.line_i, = self.ax_components.plot(self.time, self.i_array, 'g-', label='I', alpha=0.7)
        self.line_d, = self.ax_components.plot(self.time, self.d_array, 'b-', label='D', alpha=0.7)
        self.ax_components.set_xlabel('Time (s)')
        self.ax_components.set_ylabel('Component Value')
        self.ax_components.set_title('PID Components')
        self.ax_components.legend()
        self.ax_components.grid(True, alpha=0.3)

        # Plot 5: Info text
        self.ax_info = self.fig.add_subplot(gs[2:4, 1])
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.1, 0.5, '', fontsize=10,
                                          family='monospace', verticalalignment='center')

        # Create sliders
        slider_color = 'lightgoldenrodyellow'

        # Kp slider
        ax_kp = plt.axes([0.15, 0.15, 0.65, 0.02], facecolor=slider_color)
        self.slider_kp = Slider(ax_kp, 'Kp', 0.0, 50.0, valinit=self.Kp, valstep=0.1)
        self.slider_kp.on_changed(self.update)

        # Ki slider
        ax_ki = plt.axes([0.15, 0.11, 0.65, 0.02], facecolor=slider_color)
        self.slider_ki = Slider(ax_ki, 'Ki', 0.0, 20.0, valinit=self.Ki, valstep=0.1)
        self.slider_ki.on_changed(self.update)

        # Kd slider
        ax_kd = plt.axes([0.15, 0.07, 0.65, 0.02], facecolor=slider_color)
        self.slider_kd = Slider(ax_kd, 'Kd', 0.0, 30.0, valinit=self.Kd, valstep=0.1)
        self.slider_kd.on_changed(self.update)

        # Setpoint slider
        ax_sp = plt.axes([0.15, 0.03, 0.65, 0.02], facecolor=slider_color)
        self.slider_sp = Slider(ax_sp, 'Setpoint', 0.0, 100.0, valinit=self.setpoint, valstep=1.0)
        self.slider_sp.on_changed(self.update)

        # Plant type selector
        ax_plant = plt.axes([0.85, 0.03, 0.13, 0.15], facecolor=slider_color)
        self.radio = RadioButtons(ax_plant, [
            'First Order System',
            'Second Order System',
            'Pump Pressure',
            'Water Tank',
            'Motor Speed'
        ], active=1)
        self.radio.on_clicked(self.change_plant)

        # Reset button
        ax_reset = plt.axes([0.85, 0.19, 0.05, 0.03])
        self.btn_reset = Button(ax_reset, 'Reset', color=slider_color)
        self.btn_reset.on_clicked(self.reset)

        # Update info text
        self.update_info()

    def update(self, val):
        """Called when any slider changes"""
        # Get new values
        self.Kp = self.slider_kp.val
        self.Ki = self.slider_ki.val
        self.Kd = self.slider_kd.val
        self.setpoint = self.slider_sp.val

        # Re-run simulation
        self.run_simulation()

        # Update plots
        self.line_setpoint.set_ydata(self.setpoint_array)
        self.line_pv.set_ydata(self.pv_array)
        self.line_error.set_ydata(self.error_array)
        self.line_output.set_ydata(self.output_array)
        self.line_p.set_ydata(self.p_array)
        self.line_i.set_ydata(self.i_array)
        self.line_d.set_ydata(self.d_array)

        # Rescale axes
        self.ax_response.relim()
        self.ax_response.autoscale_view()
        self.ax_error.relim()
        self.ax_error.autoscale_view()
        self.ax_output.relim()
        self.ax_output.autoscale_view()
        self.ax_components.relim()
        self.ax_components.autoscale_view()

        # Update info
        self.update_info()

        # Redraw
        self.fig.canvas.draw_idle()

    def change_plant(self, label):
        """Called when plant type is changed"""
        self.plant_type = label
        self.plant = self._create_plant(label)
        self.update(None)

    def reset(self, event):
        """Reset sliders to default values"""
        self.slider_kp.reset()
        self.slider_ki.reset()
        self.slider_kd.reset()
        self.slider_sp.reset()

    def update_info(self):
        """Update the information text box"""
        # Calculate metrics
        if len(self.error_array) > 0:
            # Overshoot
            if self.setpoint != 0:
                overshoot = (np.max(self.pv_array) - self.setpoint) / self.setpoint * 100
            else:
                overshoot = 0

            # Steady-state error (last 1 second)
            last_second_idx = int(1.0 / self.sample_time)
            ss_error = np.mean(self.error_array[-last_second_idx:])

            # Settling time (within 5%)
            tolerance = 0.05 * abs(self.setpoint)
            settled = np.abs(self.error_array) < tolerance
            if np.any(settled) and np.all(settled[np.where(settled)[0][-1]:]):
                settling_time = np.where(settled)[0][-1] * self.sample_time
            else:
                settling_time = self.duration

            info = f"""
PID Parameters:
  Kp = {self.Kp:.2f}
  Ki = {self.Ki:.2f}
  Kd = {self.Kd:.2f}

Setpoint: {self.setpoint:.1f}

Performance:
  Overshoot: {overshoot:.2f}%
  Settling Time: {settling_time:.2f}s
  Steady-State Error: {ss_error:.3f}

Plant Type:
  {self.plant_type}

Tips:
  - Kp: Faster response, but can overshoot
  - Ki: Eliminates steady-state error
  - Kd: Reduces overshoot & oscillation
            """
        else:
            info = "No data"

        self.info_text.set_text(info)

    def show(self):
        """Display the interactive tuner"""
        plt.show()


def main():
    """Main function to run the interactive tuner"""
    print("=" * 60)
    print("Interactive PID Tuner")
    print("=" * 60)
    print("\nInstructions:")
    print("- Use the sliders to adjust Kp, Ki, Kd parameters")
    print("- Change the setpoint to see tracking performance")
    print("- Select different plant types to see how PID behaves")
    print("- Watch how P, I, D components contribute to control")
    print("\nTips for tuning:")
    print("1. Start with Kp only (Ki=0, Kd=0)")
    print("2. Increase Kp until you see fast response with some overshoot")
    print("3. Add Ki to eliminate steady-state error")
    print("4. Add Kd to reduce overshoot and oscillation")
    print("=" * 60)
    print()

    tuner = InteractivePIDTuner()
    tuner.show()


if __name__ == "__main__":
    main()
