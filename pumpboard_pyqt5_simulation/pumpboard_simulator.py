"""
PumpBoard Controller Simulator - PyQt5 GUI
Replicates PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27 functionality
"""
import sys
import numpy as np
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QSpinBox, QPushButton,
                             QGroupBox, QGridLayout, QCheckBox, QTabWidget,
                             QTableWidget, QTableWidgetItem)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg

from pid_controller import CurrentPIDController, AnglePIDController, PressurePIDController
from pump_plant_model import HydraulicPumpPlant, SensorSimulator


class CascadeControlSystem:
    """Cascade PID control system: Pressure -> Angle -> Current"""

    def __init__(self):
        # Create PID controllers
        self.current_pid_a = CurrentPIDController()
        self.current_pid_b = CurrentPIDController()
        self.angle_pid = AnglePIDController()
        self.pressure_pid = PressurePIDController()

        # Create plant and sensor models
        self.plant = HydraulicPumpPlant(dt=0.0001)  # 100us time step
        self.sensor = SensorSimulator()

        # Control enables (from firmware)
        self.pump_start_enabled = False
        self.pressure_loop_enabled = False
        self.angle_loop_enabled = True
        self.power_loop_enabled = False

        # Setpoints
        self.angle_setpoint = 2500  # Per unit (0-5000)
        self.pressure_setpoint = 300  # bar (0-600)
        self.current_a_setpoint = 1500  # mA
        self.current_b_setpoint = 1500  # mA

        # Feedback values
        self.angle_feedback = 0
        self.pressure_feedback = 0
        self.current_a_feedback = 0
        self.current_b_feedback = 0

        # PWM outputs
        self.pwm_a = 0
        self.pwm_b = 0

        # Simulation mode
        self.simulation_running = False

    def reset(self):
        """Reset all controllers and plant"""
        self.current_pid_a.reset()
        self.current_pid_b.reset()
        self.angle_pid.reset()
        self.pressure_pid.reset()
        self.plant.reset()

    def update(self):
        """
        Update cascade control system (one iteration)
        Matches firmware control structure
        """
        if not self.simulation_running:
            return None

        # Update plant model with current PWM commands
        plant_state = self.plant.update(self.pwm_a, self.pwm_b)

        # Get sensor feedback (with filtering)
        self.current_a_feedback = plant_state['current_a']
        self.current_b_feedback = plant_state['current_b']
        raw_angle = plant_state['angle']
        raw_pressure = plant_state['pressure']

        # Apply sensor filtering
        self.angle_feedback = self.sensor.filter_angle(raw_angle)
        self.pressure_feedback = self.sensor.filter_pressure(raw_pressure)

        # Cascade control logic
        if self.pressure_loop_enabled:
            # Outer loop: Pressure controller -> Angle setpoint
            self.angle_setpoint = self.pressure_pid.update(
                self.pressure_setpoint * 10,  # Scale to match firmware
                self.pressure_feedback * 10
            )
            # Clamp angle setpoint
            self.angle_setpoint = max(0, min(5000, self.angle_setpoint))

        if self.angle_loop_enabled:
            # Middle loop: Angle controller -> Current setpoint
            angle_output = self.angle_pid.update(
                self.angle_setpoint,
                self.angle_feedback
            )

            # Convert angle PID output to differential current
            # Angle output range: -2500 to +2500
            # Map to current setpoints: A and B
            if angle_output >= 0:
                self.current_a_setpoint = 1500 + angle_output
                self.current_b_setpoint = 1500
            else:
                self.current_a_setpoint = 1500
                self.current_b_setpoint = 1500 - angle_output

            # Clamp current setpoints
            self.current_a_setpoint = max(0, min(3000, self.current_a_setpoint))
            self.current_b_setpoint = max(0, min(3000, self.current_b_setpoint))

        # Inner loop: Current controllers -> PWM output
        self.pwm_a = self.current_pid_a.update(
            self.current_a_setpoint,
            self.current_a_feedback
        )
        self.pwm_b = self.current_pid_b.update(
            self.current_b_setpoint,
            self.current_b_feedback
        )

        # Return current state for plotting
        return {
            'angle_setpoint': self.angle_setpoint,
            'angle_feedback': self.angle_feedback,
            'pressure_setpoint': self.pressure_setpoint,
            'pressure_feedback': self.pressure_feedback,
            'current_a_setpoint': self.current_a_setpoint,
            'current_a_feedback': self.current_a_feedback,
            'current_b_setpoint': self.current_b_setpoint,
            'current_b_feedback': self.current_b_feedback,
            'pwm_a': self.pwm_a,
            'pwm_b': self.pwm_b
        }


class PumpBoardSimulatorGUI(QMainWindow):
    """Main GUI window for PumpBoard simulator"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("PumpBoard PPQ1 Simulator - PyQt5")
        self.setGeometry(100, 100, 1400, 900)

        # Create control system
        self.control_system = CascadeControlSystem()

        # Data buffers for plotting
        self.max_points = 1000
        self.time_data = deque(maxlen=self.max_points)
        self.angle_sp_data = deque(maxlen=self.max_points)
        self.angle_fb_data = deque(maxlen=self.max_points)
        self.pressure_sp_data = deque(maxlen=self.max_points)
        self.pressure_fb_data = deque(maxlen=self.max_points)
        self.current_a_sp_data = deque(maxlen=self.max_points)
        self.current_a_fb_data = deque(maxlen=self.max_points)
        self.current_b_sp_data = deque(maxlen=self.max_points)
        self.current_b_fb_data = deque(maxlen=self.max_points)
        self.pwm_a_data = deque(maxlen=self.max_points)
        self.pwm_b_data = deque(maxlen=self.max_points)

        self.time_counter = 0
        self.dt = 0.001  # 1ms update rate for GUI

        # Setup UI
        self.setup_ui()

        # Timer for simulation updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.update_interval = 1  # ms

    def setup_ui(self):
        """Setup the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Left panel: Controls
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, stretch=1)

        # Right panel: Plots
        right_panel = self.create_plot_panel()
        main_layout.addWidget(right_panel, stretch=3)

    def create_control_panel(self):
        """Create control panel with inputs"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Title
        title = QLabel("PumpBoard Controller")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # System Enable Controls
        enable_group = QGroupBox("System Enable")
        enable_layout = QVBoxLayout()

        self.pump_start_cb = QCheckBox("Pump Start")
        self.pump_start_cb.setChecked(False)
        self.pump_start_cb.stateChanged.connect(self.on_enable_changed)

        self.angle_loop_cb = QCheckBox("Angle Loop Enable")
        self.angle_loop_cb.setChecked(True)
        self.angle_loop_cb.stateChanged.connect(self.on_enable_changed)

        self.pressure_loop_cb = QCheckBox("Pressure Loop Enable")
        self.pressure_loop_cb.setChecked(False)
        self.pressure_loop_cb.stateChanged.connect(self.on_enable_changed)

        self.power_loop_cb = QCheckBox("Power Loop Enable")
        self.power_loop_cb.setChecked(False)
        self.power_loop_cb.stateChanged.connect(self.on_enable_changed)

        enable_layout.addWidget(self.pump_start_cb)
        enable_layout.addWidget(self.angle_loop_cb)
        enable_layout.addWidget(self.pressure_loop_cb)
        enable_layout.addWidget(self.power_loop_cb)
        enable_group.setLayout(enable_layout)
        layout.addWidget(enable_group)

        # Setpoint Controls
        setpoint_group = QGroupBox("Setpoints")
        setpoint_layout = QGridLayout()

        # Angle setpoint
        setpoint_layout.addWidget(QLabel("Angle:"), 0, 0)
        self.angle_sp_spin = QSpinBox()
        self.angle_sp_spin.setRange(0, 5000)
        self.angle_sp_spin.setValue(2500)
        self.angle_sp_spin.setSuffix(" pu")
        setpoint_layout.addWidget(self.angle_sp_spin, 0, 1)

        # Pressure setpoint
        setpoint_layout.addWidget(QLabel("Pressure:"), 1, 0)
        self.pressure_sp_spin = QSpinBox()
        self.pressure_sp_spin.setRange(0, 600)
        self.pressure_sp_spin.setValue(300)
        self.pressure_sp_spin.setSuffix(" bar")
        setpoint_layout.addWidget(self.pressure_sp_spin, 1, 1)

        # Current A setpoint
        setpoint_layout.addWidget(QLabel("Current A:"), 2, 0)
        self.current_a_sp_spin = QSpinBox()
        self.current_a_sp_spin.setRange(0, 3000)
        self.current_a_sp_spin.setValue(1500)
        self.current_a_sp_spin.setSuffix(" mA")
        setpoint_layout.addWidget(self.current_a_sp_spin, 2, 1)

        # Current B setpoint
        setpoint_layout.addWidget(QLabel("Current B:"), 3, 0)
        self.current_b_sp_spin = QSpinBox()
        self.current_b_sp_spin.setRange(0, 3000)
        self.current_b_sp_spin.setValue(1500)
        self.current_b_sp_spin.setSuffix(" mA")
        setpoint_layout.addWidget(self.current_b_sp_spin, 3, 1)

        setpoint_group.setLayout(setpoint_layout)
        layout.addWidget(setpoint_group)

        # Control Buttons
        button_group = QGroupBox("Control")
        button_layout = QVBoxLayout()

        self.start_btn = QPushButton("Start Simulation")
        self.start_btn.clicked.connect(self.start_simulation)
        button_layout.addWidget(self.start_btn)

        self.stop_btn = QPushButton("Stop Simulation")
        self.stop_btn.clicked.connect(self.stop_simulation)
        self.stop_btn.setEnabled(False)
        button_layout.addWidget(self.stop_btn)

        self.reset_btn = QPushButton("Reset")
        self.reset_btn.clicked.connect(self.reset_simulation)
        button_layout.addWidget(self.reset_btn)

        button_group.setLayout(button_layout)
        layout.addWidget(button_group)

        # Status Display
        status_group = QGroupBox("Status")
        status_layout = QGridLayout()

        status_layout.addWidget(QLabel("Angle FB:"), 0, 0)
        self.angle_fb_label = QLabel("0 pu")
        status_layout.addWidget(self.angle_fb_label, 0, 1)

        status_layout.addWidget(QLabel("Pressure FB:"), 1, 0)
        self.pressure_fb_label = QLabel("0 bar")
        status_layout.addWidget(self.pressure_fb_label, 1, 1)

        status_layout.addWidget(QLabel("Current A FB:"), 2, 0)
        self.current_a_fb_label = QLabel("0 mA")
        status_layout.addWidget(self.current_a_fb_label, 2, 1)

        status_layout.addWidget(QLabel("Current B FB:"), 3, 0)
        self.current_b_fb_label = QLabel("0 mA")
        status_layout.addWidget(self.current_b_fb_label, 3, 1)

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        layout.addStretch()
        return panel

    def create_plot_panel(self):
        """Create plotting panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Create tab widget for different plots
        tabs = QTabWidget()

        # Angle plot
        self.angle_plot = pg.PlotWidget(title="Angle Control")
        self.angle_plot.setLabel('left', 'Angle', units='pu')
        self.angle_plot.setLabel('bottom', 'Time', units='s')
        self.angle_plot.addLegend()
        self.angle_sp_curve = self.angle_plot.plot(pen='r', name='Setpoint')
        self.angle_fb_curve = self.angle_plot.plot(pen='g', name='Feedback')
        tabs.addTab(self.angle_plot, "Angle")

        # Pressure plot
        self.pressure_plot = pg.PlotWidget(title="Pressure Control")
        self.pressure_plot.setLabel('left', 'Pressure', units='bar')
        self.pressure_plot.setLabel('bottom', 'Time', units='s')
        self.pressure_plot.addLegend()
        self.pressure_sp_curve = self.pressure_plot.plot(pen='r', name='Setpoint')
        self.pressure_fb_curve = self.pressure_plot.plot(pen='g', name='Feedback')
        tabs.addTab(self.pressure_plot, "Pressure")

        # Current plot
        self.current_plot = pg.PlotWidget(title="Current Control")
        self.current_plot.setLabel('left', 'Current', units='mA')
        self.current_plot.setLabel('bottom', 'Time', units='s')
        self.current_plot.addLegend()
        self.current_a_sp_curve = self.current_plot.plot(pen='r', name='A Setpoint')
        self.current_a_fb_curve = self.current_plot.plot(pen='g', name='A Feedback')
        self.current_b_sp_curve = self.current_plot.plot(pen='m', name='B Setpoint')
        self.current_b_fb_curve = self.current_plot.plot(pen='c', name='B Feedback')
        tabs.addTab(self.current_plot, "Current")

        # PWM plot
        self.pwm_plot = pg.PlotWidget(title="PWM Output")
        self.pwm_plot.setLabel('left', 'PWM', units='counts')
        self.pwm_plot.setLabel('bottom', 'Time', units='s')
        self.pwm_plot.addLegend()
        self.pwm_a_curve = self.pwm_plot.plot(pen='r', name='PWM A')
        self.pwm_b_curve = self.pwm_plot.plot(pen='b', name='PWM B')
        tabs.addTab(self.pwm_plot, "PWM")

        layout.addWidget(tabs)
        return panel

    def on_enable_changed(self):
        """Handle enable checkbox changes"""
        self.control_system.pump_start_enabled = self.pump_start_cb.isChecked()
        self.control_system.angle_loop_enabled = self.angle_loop_cb.isChecked()
        self.control_system.pressure_loop_enabled = self.pressure_loop_cb.isChecked()
        self.control_system.power_loop_enabled = self.power_loop_cb.isChecked()

    def start_simulation(self):
        """Start the simulation"""
        self.control_system.simulation_running = True
        self.timer.start(self.update_interval)
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def stop_simulation(self):
        """Stop the simulation"""
        self.control_system.simulation_running = False
        self.timer.stop()
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

    def reset_simulation(self):
        """Reset the simulation"""
        self.stop_simulation()
        self.control_system.reset()

        # Clear data buffers
        self.time_data.clear()
        self.angle_sp_data.clear()
        self.angle_fb_data.clear()
        self.pressure_sp_data.clear()
        self.pressure_fb_data.clear()
        self.current_a_sp_data.clear()
        self.current_a_fb_data.clear()
        self.current_b_sp_data.clear()
        self.current_b_fb_data.clear()
        self.pwm_a_data.clear()
        self.pwm_b_data.clear()

        self.time_counter = 0

        # Update plots
        self.update_plots()

    def update_simulation(self):
        """Update simulation (called by timer)"""
        # Update setpoints from GUI
        self.control_system.angle_setpoint = self.angle_sp_spin.value()
        self.control_system.pressure_setpoint = self.pressure_sp_spin.value()
        self.control_system.current_a_setpoint = self.current_a_sp_spin.value()
        self.control_system.current_b_setpoint = self.current_b_sp_spin.value()

        # Run control system update
        state = self.control_system.update()

        if state is not None:
            # Store data for plotting
            self.time_data.append(self.time_counter)
            self.angle_sp_data.append(state['angle_setpoint'])
            self.angle_fb_data.append(state['angle_feedback'])
            self.pressure_sp_data.append(state['pressure_setpoint'])
            self.pressure_fb_data.append(state['pressure_feedback'])
            self.current_a_sp_data.append(state['current_a_setpoint'])
            self.current_a_fb_data.append(state['current_a_feedback'])
            self.current_b_sp_data.append(state['current_b_setpoint'])
            self.current_b_fb_data.append(state['current_b_feedback'])
            self.pwm_a_data.append(state['pwm_a'])
            self.pwm_b_data.append(state['pwm_b'])

            self.time_counter += self.dt

            # Update status labels
            self.angle_fb_label.setText(f"{state['angle_feedback']:.1f} pu")
            self.pressure_fb_label.setText(f"{state['pressure_feedback']:.1f} bar")
            self.current_a_fb_label.setText(f"{state['current_a_feedback']:.1f} mA")
            self.current_b_fb_label.setText(f"{state['current_b_feedback']:.1f} mA")

            # Update plots
            self.update_plots()

    def update_plots(self):
        """Update all plots"""
        if len(self.time_data) > 0:
            time_array = np.array(self.time_data)

            # Angle plot
            self.angle_sp_curve.setData(time_array, np.array(self.angle_sp_data))
            self.angle_fb_curve.setData(time_array, np.array(self.angle_fb_data))

            # Pressure plot
            self.pressure_sp_curve.setData(time_array, np.array(self.pressure_sp_data))
            self.pressure_fb_curve.setData(time_array, np.array(self.pressure_fb_data))

            # Current plot
            self.current_a_sp_curve.setData(time_array, np.array(self.current_a_sp_data))
            self.current_a_fb_curve.setData(time_array, np.array(self.current_a_fb_data))
            self.current_b_sp_curve.setData(time_array, np.array(self.current_b_sp_data))
            self.current_b_fb_curve.setData(time_array, np.array(self.current_b_fb_data))

            # PWM plot
            self.pwm_a_curve.setData(time_array, np.array(self.pwm_a_data))
            self.pwm_b_curve.setData(time_array, np.array(self.pwm_b_data))


def main():
    app = QApplication(sys.argv)
    window = PumpBoardSimulatorGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
