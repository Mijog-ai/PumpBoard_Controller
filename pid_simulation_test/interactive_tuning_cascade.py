"""
Interactive Three-Tab PID Tuning Tool - Cascade Control
========================================================

This tool simulates the cascade control structure used in the actual PumpBoard controller:
  Pressure Loop (outer) → Angle Loop (middle) → Current Loop (inner)

Each tab allows tuning of one control loop while seeing how it affects the others.

Usage:
    python interactive_tuning_cascade.py

The three tabs represent:
1. PRESSURE LOOP: Outer loop controlling hydraulic pressure
2. ANGLE LOOP: Middle loop controlling pump swash plate angle
3. CURRENT LOOP: Inner loop controlling solenoid valve current

This matches the real controller cascade structure!
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.gridspec import GridSpec
from pid_controller import PIDController


class PlantBase:
    """Base class for plant models"""
    pass


class CurrentPlant(PlantBase):
    """
    Solenoid valve current control plant (innermost loop)

    Models the electrical dynamics of the solenoid valve:
    - Fast response (100us control period)
    - First-order lag from inductance
    - PWM to current conversion
    """

    def __init__(self, time_constant=0.002, gain=0.8, initial_value=0.0, sample_time=0.0001):
        """
        Args:
            time_constant: Electrical time constant (L/R) in seconds
            gain: PWM to current conversion
            initial_value: Starting current
            sample_time: 100us = 0.0001s (matches real controller)
        """
        self.value = initial_value
        self.sample_time = sample_time
        self.history = [initial_value]
        self.tau = time_constant
        self.gain = gain

    def update(self, pwm_input):
        """Update current based on PWM input"""
        # First-order lag: dI/dt = (PWM*gain - I) / tau
        dI = (self.gain * pwm_input - self.value) / self.tau
        self.value += dI * self.sample_time
        self.value = max(0, self.value)  # Current can't be negative
        self.history.append(self.value)
        return self.value

    def reset(self, initial_value=0.0):
        self.value = initial_value
        self.history = [initial_value]


class AnglePlant(PlantBase):
    """
    Swash plate angle control plant (middle loop)

    Models the mechanical dynamics of the pump swash plate:
    - Controlled by solenoid valve current
    - Has inertia and friction
    - Medium speed (1ms control period)
    """

    def __init__(self, inertia=0.05, damping=5.0, stiffness=80.0,
                 gain=2.5, initial_value=0.0, sample_time=0.001):
        """
        Args:
            inertia: Rotational inertia of swash plate
            damping: Friction and viscous damping
            stiffness: Spring return force
            gain: Current to torque conversion
            initial_value: Starting angle
            sample_time: 1ms = 0.001s (matches real controller)
        """
        self.value = initial_value
        self.sample_time = sample_time
        self.history = [initial_value]
        self.inertia = inertia
        self.damping = damping
        self.stiffness = stiffness
        self.gain = gain
        self.velocity = 0.0

    def update(self, current_input):
        """Update angle based on solenoid current"""
        # Torque from current
        torque = self.gain * current_input

        # Second-order dynamics: I*d²θ/dt² = torque - damping*dθ/dt - stiffness*θ
        acceleration = (torque - self.damping * self.velocity - self.stiffness * self.value) / self.inertia

        self.velocity += acceleration * self.sample_time
        self.value += self.velocity * self.sample_time

        # Physical limits (0 to 100 represents angle range)
        self.value = np.clip(self.value, 0, 100)

        self.history.append(self.value)
        return self.value

    def reset(self, initial_value=0.0):
        self.value = initial_value
        self.velocity = 0.0
        self.history = [initial_value]


class PressurePlant(PlantBase):
    """
    Hydraulic pressure control plant (outermost loop)

    Models the pump pressure system:
    - Pressure depends on pump angle
    - Has leakage (pressure drop)
    - Slow dynamics (2.5ms control period)
    """

    def __init__(self, pump_efficiency=0.6, leakage=0.08,
                 initial_value=0.0, sample_time=0.0025):
        """
        Args:
            pump_efficiency: Angle to pressure conversion
            leakage: Pressure leakage coefficient
            initial_value: Starting pressure
            sample_time: 2.5ms = 0.0025s (matches real controller)
        """
        self.value = initial_value
        self.sample_time = sample_time
        self.history = [initial_value]
        self.efficiency = pump_efficiency
        self.leakage = leakage

    def update(self, angle_input):
        """Update pressure based on pump angle"""
        # Pressure generation from angle
        pump_flow = self.efficiency * angle_input

        # Leakage proportional to pressure
        leakage_flow = self.leakage * self.value

        # Net pressure change
        dP = pump_flow - leakage_flow
        self.value += dP * self.sample_time

        # Physical limits
        self.value = np.clip(self.value, 0, 100)

        self.history.append(self.value)
        return self.value

    def reset(self, initial_value=0.0):
        self.value = initial_value
        self.history = [initial_value]


class CascadePIDTuner:
    """
    Three-tab interactive PID tuner for cascade control

    Simulates the real controller structure:
    Pressure → Angle → Current → PWM
    """

    def __init__(self):
        # Control periods (matching real controller)
        self.dt_current = 0.0001   # 100us - innermost loop
        self.dt_angle = 0.001       # 1ms - middle loop
        self.dt_pressure = 0.0025   # 2.5ms - outer loop

        # Use the fastest rate for simulation
        self.sample_time = self.dt_current
        self.duration = 5.0
        self.time = np.arange(0, self.duration, self.sample_time)

        # PID Parameters (initial values from real controller)
        # Current Loop (from PID.h lines 34-43)
        self.current_Kp = 0.0244  # 50/2048
        self.current_Ki = 0.7935  # 6500/8192
        self.current_Kd = 0.0

        # Angle Loop (from PID.h lines 46-58)
        self.angle_Kp = 0.375     # 150/400
        self.angle_Ki = 0.0       # No integral
        self.angle_Kd = 3.333     # 1500/450

        # Pressure Loop (from PID.h lines 63-78)
        self.pressure_Kp = 5.333  # 1600/300
        self.pressure_Ki = 0.0    # No integral
        self.pressure_Kd = 5.15   # 515/100

        # Setpoints
        self.pressure_setpoint = 50.0
        self.angle_setpoint = 40.0
        self.current_setpoint = 30.0

        # Create plants
        self.current_plant = CurrentPlant(sample_time=self.dt_current)
        self.angle_plant = AnglePlant(sample_time=self.dt_angle)
        self.pressure_plant = PressurePlant(sample_time=self.dt_pressure)

        # Control mode: 'cascade' or 'independent'
        self.cascade_mode = True

        # Counters for control loop timing
        self.angle_counter = 0
        self.pressure_counter = 0

        # Run initial simulation
        self.run_simulation()

        # Create GUI
        self.create_gui()

    def run_simulation(self):
        """Run cascade PID simulation"""
        # Create PID controllers
        pid_current = PIDController(
            Kp=self.current_Kp,
            Ki=self.current_Ki,
            Kd=self.current_Kd,
            setpoint=0.0,
            output_limits=(0, 100),
            sample_time=self.dt_current
        )

        pid_angle = PIDController(
            Kp=self.angle_Kp,
            Ki=self.angle_Ki,
            Kd=self.angle_Kd,
            setpoint=0.0,
            output_limits=(0, 100),
            sample_time=self.dt_angle
        )

        pid_pressure = PIDController(
            Kp=self.pressure_Kp,
            Ki=self.pressure_Ki,
            Kd=self.pressure_Kd,
            setpoint=0.0,
            output_limits=(0, 100),
            sample_time=self.dt_pressure
        )

        # Reset plants
        self.current_plant.reset()
        self.angle_plant.reset()
        self.pressure_plant.reset()

        # Storage arrays
        self.pressure_sp_array = []
        self.pressure_pv_array = []
        self.angle_sp_array = []
        self.angle_pv_array = []
        self.current_sp_array = []
        self.current_pv_array = []
        self.pwm_array = []

        # Control loop timing counters
        angle_cnt = 0
        pressure_cnt = 0

        # Simulation loop
        for i, t in enumerate(self.time):
            # Update setpoints (step at t=1s)
            if t >= 1.0:
                pressure_sp = self.pressure_setpoint
                angle_sp = self.angle_setpoint if not self.cascade_mode else 0.0
                current_sp = self.current_setpoint if not self.cascade_mode else 0.0
            else:
                pressure_sp = 0.0
                angle_sp = 0.0
                current_sp = 0.0

            # PRESSURE LOOP (runs every 2.5ms = 25 cycles)
            if pressure_cnt == 0 and self.cascade_mode:
                pid_pressure.set_setpoint(pressure_sp)
                pressure_output = pid_pressure.update(self.pressure_plant.value)
                # Pressure output becomes angle setpoint
                angle_sp = pressure_output

            pressure_cnt = (pressure_cnt + 1) % int(self.dt_pressure / self.sample_time)

            # ANGLE LOOP (runs every 1ms = 10 cycles)
            if angle_cnt == 0:
                pid_angle.set_setpoint(angle_sp)
                angle_output = pid_angle.update(self.angle_plant.value)
                # Angle output becomes current setpoint
                if self.cascade_mode:
                    current_sp = angle_output

            angle_cnt = (angle_cnt + 1) % int(self.dt_angle / self.sample_time)

            # CURRENT LOOP (runs every cycle - 100us)
            pid_current.set_setpoint(current_sp)
            pwm_output = pid_current.update(self.current_plant.value)

            # Update plants (cascade structure)
            current_value = self.current_plant.update(pwm_output)
            angle_value = self.angle_plant.update(current_value)
            pressure_value = self.pressure_plant.update(angle_value)

            # Store data
            self.pressure_sp_array.append(pressure_sp)
            self.pressure_pv_array.append(pressure_value)
            self.angle_sp_array.append(angle_sp)
            self.angle_pv_array.append(angle_value)
            self.current_sp_array.append(current_sp)
            self.current_pv_array.append(current_value)
            self.pwm_array.append(pwm_output)

        # Convert to numpy arrays
        self.pressure_sp_array = np.array(self.pressure_sp_array)
        self.pressure_pv_array = np.array(self.pressure_pv_array)
        self.angle_sp_array = np.array(self.angle_sp_array)
        self.angle_pv_array = np.array(self.angle_pv_array)
        self.current_sp_array = np.array(self.current_sp_array)
        self.current_pv_array = np.array(self.current_pv_array)
        self.pwm_array = np.array(self.pwm_array)

    def create_gui(self):
        """Create three-tab GUI"""
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.canvas.manager.set_window_title('Cascade PID Tuner - Pressure → Angle → Current')

        # Create main layout
        gs_main = GridSpec(2, 1, figure=self.fig, height_ratios=[4, 1],
                          hspace=0.4, top=0.95, bottom=0.30)

        # Top: plots area
        gs_plots = gs_main[0].subgridspec(2, 3, hspace=0.35, wspace=0.3)

        # Bottom: control sliders area (will be updated based on active tab)
        gs_controls = gs_main[1].subgridspec(1, 1)

        # Create all plots
        # Row 1: Individual loop responses
        self.ax_pressure = self.fig.add_subplot(gs_plots[0, 0])
        self.ax_angle = self.fig.add_subplot(gs_plots[0, 1])
        self.ax_current = self.fig.add_subplot(gs_plots[0, 2])

        # Row 2: Combined view and cascade flow
        self.ax_combined = self.fig.add_subplot(gs_plots[1, :2])
        self.ax_info = self.fig.add_subplot(gs_plots[1, 2])
        self.ax_info.axis('off')

        # Plot Pressure Loop
        self.line_pressure_sp, = self.ax_pressure.plot(self.time, self.pressure_sp_array,
                                                       'r--', linewidth=2, label='Setpoint', alpha=0.7)
        self.line_pressure_pv, = self.ax_pressure.plot(self.time, self.pressure_pv_array,
                                                       'b-', linewidth=1.5, label='Pressure')
        self.ax_pressure.set_xlabel('Time (s)')
        self.ax_pressure.set_ylabel('Pressure')
        self.ax_pressure.set_title('PRESSURE LOOP (Outer - 2.5ms)', fontweight='bold')
        self.ax_pressure.legend()
        self.ax_pressure.grid(True, alpha=0.3)

        # Plot Angle Loop
        self.line_angle_sp, = self.ax_angle.plot(self.time, self.angle_sp_array,
                                                 'r--', linewidth=2, label='Setpoint', alpha=0.7)
        self.line_angle_pv, = self.ax_angle.plot(self.time, self.angle_pv_array,
                                                 'g-', linewidth=1.5, label='Angle')
        self.ax_angle.set_xlabel('Time (s)')
        self.ax_angle.set_ylabel('Angle')
        self.ax_angle.set_title('ANGLE LOOP (Middle - 1ms)', fontweight='bold')
        self.ax_angle.legend()
        self.ax_angle.grid(True, alpha=0.3)

        # Plot Current Loop
        self.line_current_sp, = self.ax_current.plot(self.time, self.current_sp_array,
                                                     'r--', linewidth=2, label='Setpoint', alpha=0.7)
        self.line_current_pv, = self.ax_current.plot(self.time, self.current_pv_array,
                                                     'm-', linewidth=1.5, label='Current')
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current')
        self.ax_current.set_title('CURRENT LOOP (Inner - 100us)', fontweight='bold')
        self.ax_current.legend()
        self.ax_current.grid(True, alpha=0.3)

        # Combined plot
        self.line_combined_pressure, = self.ax_combined.plot(self.time, self.pressure_pv_array,
                                                             'b-', linewidth=1.5, label='Pressure', alpha=0.8)
        self.line_combined_angle, = self.ax_combined.plot(self.time, self.angle_pv_array,
                                                          'g-', linewidth=1.5, label='Angle', alpha=0.8)
        self.line_combined_current, = self.ax_combined.plot(self.time, self.current_pv_array,
                                                            'm-', linewidth=1.5, label='Current', alpha=0.8)
        self.ax_combined.set_xlabel('Time (s)')
        self.ax_combined.set_ylabel('Value')
        self.ax_combined.set_title('CASCADE CONTROL - All Three Loops', fontweight='bold')
        self.ax_combined.legend()
        self.ax_combined.grid(True, alpha=0.3)

        # Info text
        self.info_text = self.ax_info.text(0.05, 0.5, '', fontsize=9,
                                           family='monospace', verticalalignment='center')
        self.update_info()

        # Create control area for sliders
        self.ax_controls = self.fig.add_subplot(gs_controls[0])
        self.ax_controls.axis('off')

        # Tab buttons
        tab_y = 0.25
        tab_height = 0.04
        tab_width = 0.15

        ax_tab_pressure = plt.axes([0.15, tab_y, tab_width, tab_height])
        ax_tab_angle = plt.axes([0.15 + tab_width + 0.01, tab_y, tab_width, tab_height])
        ax_tab_current = plt.axes([0.15 + 2*(tab_width + 0.01), tab_y, tab_width, tab_height])
        ax_cascade_mode = plt.axes([0.70, tab_y, tab_width, tab_height])
        ax_reset = plt.axes([0.87, tab_y, 0.08, tab_height])

        self.btn_tab_pressure = Button(ax_tab_pressure, 'PRESSURE', color='lightblue')
        self.btn_tab_angle = Button(ax_tab_angle, 'ANGLE', color='lightgray')
        self.btn_tab_current = Button(ax_tab_current, 'CURRENT', color='lightgray')
        self.btn_cascade_mode = Button(ax_cascade_mode, 'CASCADE: ON', color='lightgreen')
        self.btn_reset = Button(ax_reset, 'Reset')

        self.btn_tab_pressure.on_clicked(lambda x: self.switch_tab('pressure'))
        self.btn_tab_angle.on_clicked(lambda x: self.switch_tab('angle'))
        self.btn_tab_current.on_clicked(lambda x: self.switch_tab('current'))
        self.btn_cascade_mode.on_clicked(self.toggle_cascade_mode)
        self.btn_reset.on_clicked(self.reset)

        # Current tab
        self.current_tab = 'pressure'

        # Create sliders for all tabs (will show/hide based on active tab)
        slider_y_start = 0.18
        slider_spacing = 0.035
        slider_color = 'lightgoldenrodyellow'

        # Pressure sliders
        self.ax_pressure_kp = plt.axes([0.15, slider_y_start, 0.65, 0.02], facecolor=slider_color)
        self.ax_pressure_ki = plt.axes([0.15, slider_y_start - slider_spacing, 0.65, 0.02], facecolor=slider_color)
        self.ax_pressure_kd = plt.axes([0.15, slider_y_start - 2*slider_spacing, 0.65, 0.02], facecolor=slider_color)
        self.ax_pressure_sp = plt.axes([0.15, slider_y_start - 3*slider_spacing, 0.65, 0.02], facecolor=slider_color)

        self.slider_pressure_kp = Slider(self.ax_pressure_kp, 'Kp', 0.0, 20.0,
                                         valinit=self.pressure_Kp, valstep=0.1)
        self.slider_pressure_ki = Slider(self.ax_pressure_ki, 'Ki', 0.0, 10.0,
                                         valinit=self.pressure_Ki, valstep=0.1)
        self.slider_pressure_kd = Slider(self.ax_pressure_kd, 'Kd', 0.0, 15.0,
                                         valinit=self.pressure_Kd, valstep=0.1)
        self.slider_pressure_sp = Slider(self.ax_pressure_sp, 'Setpoint', 0.0, 100.0,
                                         valinit=self.pressure_setpoint, valstep=1.0)

        # Angle sliders
        self.ax_angle_kp = plt.axes([0.15, slider_y_start, 0.65, 0.02], facecolor=slider_color)
        self.ax_angle_ki = plt.axes([0.15, slider_y_start - slider_spacing, 0.65, 0.02], facecolor=slider_color)
        self.ax_angle_kd = plt.axes([0.15, slider_y_start - 2*slider_spacing, 0.65, 0.02], facecolor=slider_color)
        self.ax_angle_sp = plt.axes([0.15, slider_y_start - 3*slider_spacing, 0.65, 0.02], facecolor=slider_color)

        self.slider_angle_kp = Slider(self.ax_angle_kp, 'Kp', 0.0, 5.0,
                                      valinit=self.angle_Kp, valstep=0.05)
        self.slider_angle_ki = Slider(self.ax_angle_ki, 'Ki', 0.0, 2.0,
                                      valinit=self.angle_Ki, valstep=0.05)
        self.slider_angle_kd = Slider(self.ax_angle_kd, 'Kd', 0.0, 10.0,
                                      valinit=self.angle_Kd, valstep=0.1)
        self.slider_angle_sp = Slider(self.ax_angle_sp, 'Setpoint', 0.0, 100.0,
                                      valinit=self.angle_setpoint, valstep=1.0)

        # Current sliders
        self.ax_current_kp = plt.axes([0.15, slider_y_start, 0.65, 0.02], facecolor=slider_color)
        self.ax_current_ki = plt.axes([0.15, slider_y_start - slider_spacing, 0.65, 0.02], facecolor=slider_color)
        self.ax_current_kd = plt.axes([0.15, slider_y_start - 2*slider_spacing, 0.65, 0.02], facecolor=slider_color)
        self.ax_current_sp = plt.axes([0.15, slider_y_start - 3*slider_spacing, 0.65, 0.02], facecolor=slider_color)

        self.slider_current_kp = Slider(self.ax_current_kp, 'Kp', 0.0, 1.0,
                                        valinit=self.current_Kp, valstep=0.01)
        self.slider_current_ki = Slider(self.ax_current_ki, 'Ki', 0.0, 3.0,
                                        valinit=self.current_Ki, valstep=0.05)
        self.slider_current_kd = Slider(self.ax_current_kd, 'Kd', 0.0, 1.0,
                                        valinit=self.current_Kd, valstep=0.01)
        self.slider_current_sp = Slider(self.ax_current_sp, 'Setpoint', 0.0, 100.0,
                                        valinit=self.current_setpoint, valstep=1.0)

        # Connect slider callbacks
        self.slider_pressure_kp.on_changed(self.update_pressure)
        self.slider_pressure_ki.on_changed(self.update_pressure)
        self.slider_pressure_kd.on_changed(self.update_pressure)
        self.slider_pressure_sp.on_changed(self.update_pressure)

        self.slider_angle_kp.on_changed(self.update_angle)
        self.slider_angle_ki.on_changed(self.update_angle)
        self.slider_angle_kd.on_changed(self.update_angle)
        self.slider_angle_sp.on_changed(self.update_angle)

        self.slider_current_kp.on_changed(self.update_current)
        self.slider_current_ki.on_changed(self.update_current)
        self.slider_current_kd.on_changed(self.update_current)
        self.slider_current_sp.on_changed(self.update_current)

        # Show initial tab
        self.switch_tab('pressure')

    def switch_tab(self, tab_name):
        """Switch between tabs"""
        self.current_tab = tab_name

        # Update button colors
        self.btn_tab_pressure.color = 'lightblue' if tab_name == 'pressure' else 'lightgray'
        self.btn_tab_angle.color = 'lightblue' if tab_name == 'angle' else 'lightgray'
        self.btn_tab_current.color = 'lightblue' if tab_name == 'current' else 'lightgray'

        # Hide all sliders
        for ax in [self.ax_pressure_kp, self.ax_pressure_ki, self.ax_pressure_kd, self.ax_pressure_sp,
                   self.ax_angle_kp, self.ax_angle_ki, self.ax_angle_kd, self.ax_angle_sp,
                   self.ax_current_kp, self.ax_current_ki, self.ax_current_kd, self.ax_current_sp]:
            ax.set_visible(False)

        # Show sliders for current tab
        if tab_name == 'pressure':
            self.ax_pressure_kp.set_visible(True)
            self.ax_pressure_ki.set_visible(True)
            self.ax_pressure_kd.set_visible(True)
            self.ax_pressure_sp.set_visible(True)
        elif tab_name == 'angle':
            self.ax_angle_kp.set_visible(True)
            self.ax_angle_ki.set_visible(True)
            self.ax_angle_kd.set_visible(True)
            self.ax_angle_sp.set_visible(True)
        elif tab_name == 'current':
            self.ax_current_kp.set_visible(True)
            self.ax_current_ki.set_visible(True)
            self.ax_current_kd.set_visible(True)
            self.ax_current_sp.set_visible(True)

        self.fig.canvas.draw_idle()

    def toggle_cascade_mode(self, event):
        """Toggle between cascade and independent modes"""
        self.cascade_mode = not self.cascade_mode
        self.btn_cascade_mode.label.set_text(f'CASCADE: {"ON" if self.cascade_mode else "OFF"}')
        self.btn_cascade_mode.color = 'lightgreen' if self.cascade_mode else 'salmon'
        self.update_all()

    def update_pressure(self, val):
        """Update pressure loop parameters"""
        self.pressure_Kp = self.slider_pressure_kp.val
        self.pressure_Ki = self.slider_pressure_ki.val
        self.pressure_Kd = self.slider_pressure_kd.val
        self.pressure_setpoint = self.slider_pressure_sp.val
        self.update_all()

    def update_angle(self, val):
        """Update angle loop parameters"""
        self.angle_Kp = self.slider_angle_kp.val
        self.angle_Ki = self.slider_angle_ki.val
        self.angle_Kd = self.slider_angle_kd.val
        self.angle_setpoint = self.slider_angle_sp.val
        self.update_all()

    def update_current(self, val):
        """Update current loop parameters"""
        self.current_Kp = self.slider_current_kp.val
        self.current_Ki = self.slider_current_ki.val
        self.current_Kd = self.slider_current_kd.val
        self.current_setpoint = self.slider_current_sp.val
        self.update_all()

    def update_all(self):
        """Re-run simulation and update all plots"""
        self.run_simulation()

        # Update plots
        self.line_pressure_sp.set_ydata(self.pressure_sp_array)
        self.line_pressure_pv.set_ydata(self.pressure_pv_array)
        self.line_angle_sp.set_ydata(self.angle_sp_array)
        self.line_angle_pv.set_ydata(self.angle_pv_array)
        self.line_current_sp.set_ydata(self.current_sp_array)
        self.line_current_pv.set_ydata(self.current_pv_array)

        self.line_combined_pressure.set_ydata(self.pressure_pv_array)
        self.line_combined_angle.set_ydata(self.angle_pv_array)
        self.line_combined_current.set_ydata(self.current_pv_array)

        # Rescale axes
        for ax in [self.ax_pressure, self.ax_angle, self.ax_current, self.ax_combined]:
            ax.relim()
            ax.autoscale_view()

        # Update info
        self.update_info()

        # Redraw
        self.fig.canvas.draw_idle()

    def update_info(self):
        """Update information text"""
        mode_text = "CASCADE MODE" if self.cascade_mode else "INDEPENDENT MODE"

        info = f"""
{'='*35}
{mode_text}
{'='*35}

Control Flow:
  Pressure → Angle → Current → PWM

Loop Periods:
  Pressure: 2.5ms (slowest)
  Angle:    1.0ms (medium)
  Current:  0.1ms (fastest)

Current Parameters:
┌─ PRESSURE LOOP ────────┐
│ Kp = {self.pressure_Kp:6.3f}           │
│ Ki = {self.pressure_Ki:6.3f}           │
│ Kd = {self.pressure_Kd:6.3f}           │
│ SP = {self.pressure_setpoint:6.1f}           │
└────────────────────────┘

┌─ ANGLE LOOP ───────────┐
│ Kp = {self.angle_Kp:6.3f}           │
│ Ki = {self.angle_Ki:6.3f}           │
│ Kd = {self.angle_Kd:6.3f}           │
│ SP = {self.angle_setpoint:6.1f}           │
└────────────────────────┘

┌─ CURRENT LOOP ─────────┐
│ Kp = {self.current_Kp:6.3f}           │
│ Ki = {self.current_Ki:6.3f}           │
│ Kd = {self.current_Kd:6.3f}           │
│ SP = {self.current_setpoint:6.1f}           │
└────────────────────────┘

Tips:
• Use CASCADE mode to see
  how loops interact
• Tune inner loops first
  (Current → Angle → Pressure)
• Watch combined plot for
  overall system behavior
        """

        self.info_text.set_text(info)

    def reset(self, event):
        """Reset all parameters to defaults"""
        # Reset to real controller values
        self.slider_current_kp.set_val(0.0244)
        self.slider_current_ki.set_val(0.7935)
        self.slider_current_kd.set_val(0.0)

        self.slider_angle_kp.set_val(0.375)
        self.slider_angle_ki.set_val(0.0)
        self.slider_angle_kd.set_val(3.333)

        self.slider_pressure_kp.set_val(5.333)
        self.slider_pressure_ki.set_val(0.0)
        self.slider_pressure_kd.set_val(5.15)

    def show(self):
        """Display the tuner"""
        plt.show()


def main():
    """Main function"""
    print("=" * 70)
    print("CASCADE PID TUNER - Three-Tab Interactive Tool")
    print("=" * 70)
    print()
    print("This tool simulates the cascade control structure from the real controller:")
    print()
    print("  ┌──────────────────────────────────────────────────────────┐")
    print("  │  PRESSURE LOOP (Outer, 2.5ms)                            │")
    print("  │    ↓ output → angle setpoint                             │")
    print("  │  ┌────────────────────────────────────────────────────┐  │")
    print("  │  │  ANGLE LOOP (Middle, 1ms)                          │  │")
    print("  │  │    ↓ output → current setpoint                     │  │")
    print("  │  │  ┌──────────────────────────────────────────────┐  │  │")
    print("  │  │  │  CURRENT LOOP (Inner, 100us)                 │  │  │")
    print("  │  │  │    ↓ output → PWM                            │  │  │")
    print("  │  │  │  SOLENOID VALVE                              │  │  │")
    print("  │  │  └──────────────────────────────────────────────┘  │  │")
    print("  │  └────────────────────────────────────────────────────┘  │")
    print("  └──────────────────────────────────────────────────────────┘")
    print()
    print("Instructions:")
    print("  1. Click tabs to switch between PRESSURE, ANGLE, and CURRENT loops")
    print("  2. Use sliders to adjust PID parameters (Kp, Ki, Kd) and setpoints")
    print("  3. Toggle CASCADE mode to see loop correlation")
    print("  4. Watch all three loops interact in the combined plot")
    print()
    print("Tuning Tips:")
    print("  • Start with CASCADE mode ON")
    print("  • Tune CURRENT loop first (innermost, fastest)")
    print("  • Then tune ANGLE loop (middle)")
    print("  • Finally tune PRESSURE loop (outermost, slowest)")
    print("  • Inner loops should be 3-5x faster than outer loops")
    print()
    print("=" * 70)
    print()

    tuner = CascadePIDTuner()
    tuner.show()


if __name__ == "__main__":
    main()
