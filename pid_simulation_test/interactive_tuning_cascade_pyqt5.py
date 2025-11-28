"""
Interactive Three-Tab PID Tuning Tool - Cascade Control (PyQt5 Version)
========================================================================

High-performance PyQt5 version of the cascade PID tuner.
Much faster than the matplotlib-only version!

This tool simulates the cascade control structure used in the actual PumpBoard controller:
  Pressure Loop (outer) → Angle Loop (middle) → Current Loop (inner)

Each tab allows tuning of one control loop while seeing how it affects the others.

Usage:
    python interactive_tuning_cascade_pyqt5.py

The three tabs represent:
1. PRESSURE LOOP: Outer loop controlling hydraulic pressure
2. ANGLE LOOP: Middle loop controlling pump swash plate angle
3. CURRENT LOOP: Inner loop controlling solenoid valve current

This matches the real controller cascade structure!
"""

import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
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


class MplCanvas(FigureCanvas):
    """Matplotlib canvas for embedding in PyQt5"""

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super(MplCanvas, self).__init__(self.fig)
        self.setParent(parent)


class CascadePIDTunerPyQt5(QtWidgets.QMainWindow):
    """
    PyQt5-based Three-tab interactive PID tuner for cascade control

    Much faster than matplotlib-only version!
    """

    def __init__(self):
        super().__init__()

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

        # Update timer to debounce slider changes
        self.update_timer = QtCore.QTimer()
        self.update_timer.setSingleShot(True)
        self.update_timer.timeout.connect(self.update_simulation)

        # Run initial simulation
        self.run_simulation()

        # Create GUI
        self.init_ui()

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('Cascade PID Tuner - Pressure → Angle → Current (PyQt5)')
        self.setGeometry(100, 100, 1600, 900)

        # Central widget
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QtWidgets.QVBoxLayout(central_widget)

        # Top: Plots area
        plots_widget = self.create_plots_widget()
        main_layout.addWidget(plots_widget, stretch=3)

        # Bottom: Control tabs
        tabs_widget = self.create_tabs_widget()
        main_layout.addWidget(tabs_widget, stretch=1)

        # Status bar
        self.statusBar().showMessage('Ready - Adjust sliders to tune PID parameters')

    def create_plots_widget(self):
        """Create the plots area"""
        plots_widget = QtWidgets.QWidget()
        plots_layout = QtWidgets.QGridLayout(plots_widget)

        # Create matplotlib canvases
        self.canvas_pressure = MplCanvas(self, width=5, height=3, dpi=100)
        self.canvas_angle = MplCanvas(self, width=5, height=3, dpi=100)
        self.canvas_current = MplCanvas(self, width=5, height=3, dpi=100)
        self.canvas_combined = MplCanvas(self, width=10, height=3, dpi=100)

        # Add canvases to layout
        plots_layout.addWidget(self.canvas_pressure, 0, 0)
        plots_layout.addWidget(self.canvas_angle, 0, 1)
        plots_layout.addWidget(self.canvas_current, 0, 2)
        plots_layout.addWidget(self.canvas_combined, 1, 0, 1, 3)

        # Create plots
        self.setup_plots()

        return plots_widget

    def setup_plots(self):
        """Setup the matplotlib plots"""
        # Pressure plot
        self.ax_pressure = self.canvas_pressure.fig.add_subplot(111)
        self.line_pressure_sp, = self.ax_pressure.plot(self.time, self.pressure_sp_array,
                                                       'r--', linewidth=2, label='Setpoint', alpha=0.7)
        self.line_pressure_pv, = self.ax_pressure.plot(self.time, self.pressure_pv_array,
                                                       'b-', linewidth=1.5, label='Pressure')
        self.ax_pressure.set_xlabel('Time (s)')
        self.ax_pressure.set_ylabel('Pressure')
        self.ax_pressure.set_title('PRESSURE LOOP (Outer - 2.5ms)', fontweight='bold')
        self.ax_pressure.legend()
        self.ax_pressure.grid(True, alpha=0.3)

        # Angle plot
        self.ax_angle = self.canvas_angle.fig.add_subplot(111)
        self.line_angle_sp, = self.ax_angle.plot(self.time, self.angle_sp_array,
                                                 'r--', linewidth=2, label='Setpoint', alpha=0.7)
        self.line_angle_pv, = self.ax_angle.plot(self.time, self.angle_pv_array,
                                                 'g-', linewidth=1.5, label='Angle')
        self.ax_angle.set_xlabel('Time (s)')
        self.ax_angle.set_ylabel('Angle')
        self.ax_angle.set_title('ANGLE LOOP (Middle - 1ms)', fontweight='bold')
        self.ax_angle.legend()
        self.ax_angle.grid(True, alpha=0.3)

        # Current plot
        self.ax_current = self.canvas_current.fig.add_subplot(111)
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
        self.ax_combined = self.canvas_combined.fig.add_subplot(111)
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

        # Tight layout
        self.canvas_pressure.fig.tight_layout()
        self.canvas_angle.fig.tight_layout()
        self.canvas_current.fig.tight_layout()
        self.canvas_combined.fig.tight_layout()

    def create_tabs_widget(self):
        """Create the tabbed control panel"""
        # Tab widget
        self.tabs = QtWidgets.QTabWidget()

        # Create tabs
        self.pressure_tab = self.create_control_tab('pressure')
        self.angle_tab = self.create_control_tab('angle')
        self.current_tab = self.create_control_tab('current')

        self.tabs.addTab(self.pressure_tab, "PRESSURE LOOP")
        self.tabs.addTab(self.angle_tab, "ANGLE LOOP")
        self.tabs.addTab(self.current_tab, "CURRENT LOOP")

        # Container with additional controls
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)
        layout.addWidget(self.tabs)

        # Bottom buttons
        button_layout = QtWidgets.QHBoxLayout()

        self.cascade_button = QtWidgets.QPushButton('CASCADE MODE: ON')
        self.cascade_button.setCheckable(True)
        self.cascade_button.setChecked(True)
        self.cascade_button.setStyleSheet("QPushButton:checked { background-color: lightgreen; }")
        self.cascade_button.clicked.connect(self.toggle_cascade_mode)

        reset_button = QtWidgets.QPushButton('Reset to Defaults')
        reset_button.clicked.connect(self.reset_parameters)

        button_layout.addWidget(self.cascade_button)
        button_layout.addWidget(reset_button)
        button_layout.addStretch()

        layout.addLayout(button_layout)

        return container

    def create_control_tab(self, loop_name):
        """Create a control tab for a specific loop"""
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QFormLayout(tab)

        if loop_name == 'pressure':
            kp_slider = self.create_slider(0, 200, int(self.pressure_Kp * 10),
                                           lambda v: self.update_parameter('pressure_Kp', v/10))
            ki_slider = self.create_slider(0, 100, int(self.pressure_Ki * 10),
                                           lambda v: self.update_parameter('pressure_Ki', v/10))
            kd_slider = self.create_slider(0, 150, int(self.pressure_Kd * 10),
                                           lambda v: self.update_parameter('pressure_Kd', v/10))
            sp_slider = self.create_slider(0, 100, int(self.pressure_setpoint),
                                           lambda v: self.update_parameter('pressure_setpoint', v))

            self.pressure_kp_label = QtWidgets.QLabel(f'{self.pressure_Kp:.3f}')
            self.pressure_ki_label = QtWidgets.QLabel(f'{self.pressure_Ki:.3f}')
            self.pressure_kd_label = QtWidgets.QLabel(f'{self.pressure_Kd:.3f}')
            self.pressure_sp_label = QtWidgets.QLabel(f'{self.pressure_setpoint:.1f}')

            layout.addRow(f'Kp (0-20):', self.create_slider_row(kp_slider, self.pressure_kp_label))
            layout.addRow(f'Ki (0-10):', self.create_slider_row(ki_slider, self.pressure_ki_label))
            layout.addRow(f'Kd (0-15):', self.create_slider_row(kd_slider, self.pressure_kd_label))
            layout.addRow(f'Setpoint (0-100):', self.create_slider_row(sp_slider, self.pressure_sp_label))

        elif loop_name == 'angle':
            kp_slider = self.create_slider(0, 50, int(self.angle_Kp * 10),
                                           lambda v: self.update_parameter('angle_Kp', v/10))
            ki_slider = self.create_slider(0, 20, int(self.angle_Ki * 10),
                                           lambda v: self.update_parameter('angle_Ki', v/10))
            kd_slider = self.create_slider(0, 100, int(self.angle_Kd * 10),
                                           lambda v: self.update_parameter('angle_Kd', v/10))
            sp_slider = self.create_slider(0, 100, int(self.angle_setpoint),
                                           lambda v: self.update_parameter('angle_setpoint', v))

            self.angle_kp_label = QtWidgets.QLabel(f'{self.angle_Kp:.3f}')
            self.angle_ki_label = QtWidgets.QLabel(f'{self.angle_Ki:.3f}')
            self.angle_kd_label = QtWidgets.QLabel(f'{self.angle_Kd:.3f}')
            self.angle_sp_label = QtWidgets.QLabel(f'{self.angle_setpoint:.1f}')

            layout.addRow(f'Kp (0-5):', self.create_slider_row(kp_slider, self.angle_kp_label))
            layout.addRow(f'Ki (0-2):', self.create_slider_row(ki_slider, self.angle_ki_label))
            layout.addRow(f'Kd (0-10):', self.create_slider_row(kd_slider, self.angle_kd_label))
            layout.addRow(f'Setpoint (0-100):', self.create_slider_row(sp_slider, self.angle_sp_label))

        elif loop_name == 'current':
            kp_slider = self.create_slider(0, 100, int(self.current_Kp * 100),
                                           lambda v: self.update_parameter('current_Kp', v/100))
            ki_slider = self.create_slider(0, 300, int(self.current_Ki * 100),
                                           lambda v: self.update_parameter('current_Ki', v/100))
            kd_slider = self.create_slider(0, 100, int(self.current_Kd * 100),
                                           lambda v: self.update_parameter('current_Kd', v/100))
            sp_slider = self.create_slider(0, 100, int(self.current_setpoint),
                                           lambda v: self.update_parameter('current_setpoint', v))

            self.current_kp_label = QtWidgets.QLabel(f'{self.current_Kp:.4f}')
            self.current_ki_label = QtWidgets.QLabel(f'{self.current_Ki:.4f}')
            self.current_kd_label = QtWidgets.QLabel(f'{self.current_Kd:.4f}')
            self.current_sp_label = QtWidgets.QLabel(f'{self.current_setpoint:.1f}')

            layout.addRow(f'Kp (0-1):', self.create_slider_row(kp_slider, self.current_kp_label))
            layout.addRow(f'Ki (0-3):', self.create_slider_row(ki_slider, self.current_ki_label))
            layout.addRow(f'Kd (0-1):', self.create_slider_row(kd_slider, self.current_kd_label))
            layout.addRow(f'Setpoint (0-100):', self.create_slider_row(sp_slider, self.current_sp_label))

        # Add info label
        info_label = QtWidgets.QLabel()
        if loop_name == 'pressure':
            info_text = "Outer loop (slowest) - Controls hydraulic pressure\nRuns at 2.5ms period"
        elif loop_name == 'angle':
            info_text = "Middle loop - Controls pump swash plate angle\nRuns at 1ms period"
        else:
            info_text = "Inner loop (fastest) - Controls solenoid valve current\nRuns at 100us period"
        info_label.setText(info_text)
        info_label.setStyleSheet("color: gray; font-style: italic;")
        layout.addRow(info_label)

        return tab

    def create_slider(self, min_val, max_val, init_val, callback):
        """Create a slider with callback"""
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setValue(init_val)
        slider.valueChanged.connect(callback)
        return slider

    def create_slider_row(self, slider, label):
        """Create a row with slider and value label"""
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(slider, stretch=4)
        layout.addWidget(label, stretch=1)
        return widget

    def update_parameter(self, param_name, value):
        """Update a parameter and schedule simulation update"""
        setattr(self, param_name, value)

        # Update label
        if 'pressure' in param_name:
            if 'Kp' in param_name:
                self.pressure_kp_label.setText(f'{value:.3f}')
            elif 'Ki' in param_name:
                self.pressure_ki_label.setText(f'{value:.3f}')
            elif 'Kd' in param_name:
                self.pressure_kd_label.setText(f'{value:.3f}')
            elif 'setpoint' in param_name:
                self.pressure_sp_label.setText(f'{value:.1f}')
        elif 'angle' in param_name:
            if 'Kp' in param_name:
                self.angle_kp_label.setText(f'{value:.3f}')
            elif 'Ki' in param_name:
                self.angle_ki_label.setText(f'{value:.3f}')
            elif 'Kd' in param_name:
                self.angle_kd_label.setText(f'{value:.3f}')
            elif 'setpoint' in param_name:
                self.angle_sp_label.setText(f'{value:.1f}')
        elif 'current' in param_name:
            if 'Kp' in param_name:
                self.current_kp_label.setText(f'{value:.4f}')
            elif 'Ki' in param_name:
                self.current_ki_label.setText(f'{value:.4f}')
            elif 'Kd' in param_name:
                self.current_kd_label.setText(f'{value:.4f}')
            elif 'setpoint' in param_name:
                self.current_sp_label.setText(f'{value:.1f}')

        # Debounce: restart timer (only update after 100ms of no changes)
        self.update_timer.start(100)

    def toggle_cascade_mode(self):
        """Toggle cascade mode"""
        self.cascade_mode = not self.cascade_mode
        if self.cascade_mode:
            self.cascade_button.setText('CASCADE MODE: ON')
            self.cascade_button.setStyleSheet("QPushButton:checked { background-color: lightgreen; }")
        else:
            self.cascade_button.setText('CASCADE MODE: OFF')
            self.cascade_button.setStyleSheet("QPushButton:checked { background-color: salmon; }")
        self.update_simulation()

    def reset_parameters(self):
        """Reset all parameters to defaults"""
        # Reset to real controller values
        self.current_Kp = 0.0244
        self.current_Ki = 0.7935
        self.current_Kd = 0.0

        self.angle_Kp = 0.375
        self.angle_Ki = 0.0
        self.angle_Kd = 3.333

        self.pressure_Kp = 5.333
        self.pressure_Ki = 0.0
        self.pressure_Kd = 5.15

        # Update UI (this will recreate all sliders)
        self.tabs.clear()
        self.pressure_tab = self.create_control_tab('pressure')
        self.angle_tab = self.create_control_tab('angle')
        self.current_tab = self.create_control_tab('current')

        self.tabs.addTab(self.pressure_tab, "PRESSURE LOOP")
        self.tabs.addTab(self.angle_tab, "ANGLE LOOP")
        self.tabs.addTab(self.current_tab, "CURRENT LOOP")

        self.update_simulation()

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

    def update_simulation(self):
        """Re-run simulation and update plots"""
        self.statusBar().showMessage('Updating simulation...')

        # Run simulation
        self.run_simulation()

        # Update plot data (fast - just update data, don't recreate plots)
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
        self.ax_pressure.relim()
        self.ax_pressure.autoscale_view()
        self.ax_angle.relim()
        self.ax_angle.autoscale_view()
        self.ax_current.relim()
        self.ax_current.autoscale_view()
        self.ax_combined.relim()
        self.ax_combined.autoscale_view()

        # Redraw canvases
        self.canvas_pressure.draw()
        self.canvas_angle.draw()
        self.canvas_current.draw()
        self.canvas_combined.draw()

        self.statusBar().showMessage('Ready')


def main():
    """Main function"""
    print("=" * 70)
    print("CASCADE PID TUNER - PyQt5 High-Performance Version")
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
    print("Performance Notes:")
    print("  • PyQt5 version - much faster than matplotlib-only!")
    print("  • Debounced updates for smooth slider operation")
    print("  • Real-time visualization with minimal lag")
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

    app = QtWidgets.QApplication(sys.argv)
    window = CascadePIDTunerPyQt5()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
