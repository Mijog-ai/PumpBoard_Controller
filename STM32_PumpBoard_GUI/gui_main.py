"""
Main GUI Application for STM32F407 PumpBoard Controller
PyQt5-based interface for monitoring and controlling hydraulic pump parameters
"""

import sys
import time
from collections import deque
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QComboBox, QSpinBox, QDoubleSpinBox,
    QCheckBox, QTabWidget, QGridLayout, QMessageBox, QProgressBar,
    QTableWidget, QTableWidgetItem, QHeaderView
)
from PyQt5.QtCore import QTimer, Qt, pyqtSlot
from PyQt5.QtGui import QPalette, QColor, QFont
import pyqtgraph as pg

from serial_comm import SerialProtocol
from parameter_defs import ParameterDefinitions, ParameterInfo


class PumpBoardGUI(QMainWindow):
    """Main application window"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32F407 PumpBoard Controller - Monitoring & Control")
        self.setGeometry(100, 100, 1400, 900)

        # Communication handler
        self.serial = SerialProtocol()
        self.serial.connected.connect(self.on_connected)
        self.serial.disconnected.connect(self.on_disconnected)
        self.serial.data_received.connect(self.on_data_received)
        self.serial.error_occurred.connect(self.on_error)

        # Data storage for plotting
        self.max_plot_points = 500
        self.plot_data = {
            'time': deque(maxlen=self.max_plot_points),
            'angle_fb': deque(maxlen=self.max_plot_points),
            'pressure_fb': deque(maxlen=self.max_plot_points),
            'current_a_fb': deque(maxlen=self.max_plot_points),
            'current_b_fb': deque(maxlen=self.max_plot_points),
        }
        self.start_time = time.time()

        # Parameter cache
        self.param_cache = {}

        # Setup UI
        self.init_ui()

        # Setup timers
        self.monitor_timer = QTimer()
        self.monitor_timer.timeout.connect(self.update_monitoring)
        self.monitor_interval = 100  # ms

    def init_ui(self):
        """Initialize user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # Connection panel
        connection_group = self.create_connection_panel()
        main_layout.addWidget(connection_group)

        # Tab widget for different panels
        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)

        # Create tabs
        self.tab_widget.addTab(self.create_dashboard_tab(), "Dashboard")
        self.tab_widget.addTab(self.create_control_tab(), "Control")
        self.tab_widget.addTab(self.create_pid_tab(), "PID Tuning")
        self.tab_widget.addTab(self.create_calibration_tab(), "Calibration")

        # Status bar
        self.statusBar().showMessage("Disconnected")

    def create_connection_panel(self):
        """Create connection control panel"""
        group = QGroupBox("Connection")
        layout = QHBoxLayout()

        # Port selection
        layout.addWidget(QLabel("Serial Port:"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        layout.addWidget(self.port_combo)

        # Refresh button
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh_btn)

        # Baudrate selection
        layout.addWidget(QLabel("Baudrate:"))
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400"])
        self.baudrate_combo.setCurrentText("115200")
        layout.addWidget(self.baudrate_combo)

        # Connect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)

        layout.addStretch()

        group.setLayout(layout)
        return group

    def create_dashboard_tab(self):
        """Create real-time monitoring dashboard"""
        widget = QWidget()
        layout = QVBoxLayout()

        # Real-time values display
        values_group = QGroupBox("Real-time Parameters")
        values_layout = QGridLayout()

        # Create value displays
        self.value_labels = {}

        row = 0
        for key in ['angle_feedback', 'pressure_feedback', 'current_a_feedback',
                    'current_b_feedback', 'pwm_a', 'pwm_b']:
            param = ParameterInfo.PARAMETERS[key]

            label = QLabel(f"{param['name']}:")
            label.setFont(QFont("Arial", 10, QFont.Bold))
            values_layout.addWidget(label, row, 0)

            value_label = QLabel("-- " + param['unit'])
            value_label.setFont(QFont("Arial", 12))
            value_label.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 5px; }")
            value_label.setMinimumWidth(150)
            values_layout.addWidget(value_label, row, 1)

            self.value_labels[key] = value_label
            row += 1

        values_group.setLayout(values_layout)
        layout.addWidget(values_group)

        # Plotting area
        plot_group = QGroupBox("Real-time Plots")
        plot_layout = QVBoxLayout()

        # Create plot widget
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground('w')

        # Create plots
        self.plots = {}
        plot_configs = [
            ('angle_fb', 'Angle Feedback', '%', 'b'),
            ('pressure_fb', 'Pressure Feedback', 'bar', 'r'),
            ('current_a_fb', 'Current A', 'mA', 'g'),
            ('current_b_fb', 'Current B', 'mA', 'm'),
        ]

        for i, (key, title, unit, color) in enumerate(plot_configs):
            if i > 0:
                self.plot_widget.nextRow()

            plot = self.plot_widget.addPlot(title=title)
            plot.setLabel('left', unit)
            plot.setLabel('bottom', 'Time', units='s')
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setYRange(0, 100)  # Will be adjusted dynamically

            curve = plot.plot(pen=pg.mkPen(color, width=2))
            self.plots[key] = {'plot': plot, 'curve': curve}

        plot_layout.addWidget(self.plot_widget)
        plot_group.setLayout(plot_layout)
        layout.addWidget(plot_group)

        widget.setLayout(layout)
        return widget

    def create_control_tab(self):
        """Create control panel tab"""
        widget = QWidget()
        layout = QVBoxLayout()

        # Enable flags
        enable_group = QGroupBox("Enable Controls")
        enable_layout = QGridLayout()

        self.enable_checkboxes = {}
        row = 0

        for key in ['enable_pump_start', 'enable_pressure_loop',
                    'enable_angle_leak', 'enable_power_loop']:
            param = ParameterInfo.PARAMETERS[key]

            checkbox = QCheckBox(param['name'])
            checkbox.setFont(QFont("Arial", 10))
            checkbox.stateChanged.connect(
                lambda state, k=key: self.on_enable_changed(k, state)
            )
            enable_layout.addWidget(checkbox, row // 2, row % 2)

            self.enable_checkboxes[key] = checkbox
            row += 1

        enable_group.setLayout(enable_layout)
        layout.addWidget(enable_group)

        # Setpoint controls
        setpoint_group = QGroupBox("Setpoint Controls")
        setpoint_layout = QGridLayout()

        self.setpoint_spinboxes = {}
        row = 0

        for key in ['angle_reference', 'pressure_reference',
                    'current_a_reference', 'current_b_reference', 'torque_limit']:
            param = ParameterInfo.PARAMETERS[key]

            label = QLabel(f"{param['name']} ({param['unit']}):")
            setpoint_layout.addWidget(label, row, 0)

            # Use QDoubleSpinBox if scale is not 1.0, otherwise use QSpinBox
            if param['scale'] != 1.0:
                spinbox = QDoubleSpinBox()
                spinbox.setDecimals(2)
                spinbox.setSuffix(" " + param['unit'])
                spinbox.setMinimum(param['min'] * param['scale'])
                spinbox.setMaximum(param['max'] * param['scale'])
            else:
                spinbox = QSpinBox()
                spinbox.setSuffix(" " + param['unit'])
                spinbox.setMinimum(int(param['min'] * param['scale']))
                spinbox.setMaximum(int(param['max'] * param['scale']))

            spinbox.setValue(0)
            setpoint_layout.addWidget(spinbox, row, 1)

            apply_btn = QPushButton("Apply")
            apply_btn.clicked.connect(
                lambda checked, k=key: self.apply_setpoint(k)
            )
            setpoint_layout.addWidget(apply_btn, row, 2)

            self.setpoint_spinboxes[key] = spinbox
            row += 1

        setpoint_group.setLayout(setpoint_layout)
        layout.addWidget(setpoint_group)

        # Quick control buttons
        quick_group = QGroupBox("Quick Actions")
        quick_layout = QHBoxLayout()

        stop_all_btn = QPushButton("EMERGENCY STOP")
        stop_all_btn.setStyleSheet("QPushButton { background-color: red; color: white; font-weight: bold; padding: 10px; }")
        stop_all_btn.clicked.connect(self.emergency_stop)
        quick_layout.addWidget(stop_all_btn)

        zero_all_btn = QPushButton("Zero All Setpoints")
        zero_all_btn.clicked.connect(self.zero_all_setpoints)
        quick_layout.addWidget(zero_all_btn)

        read_all_btn = QPushButton("Read All Parameters")
        read_all_btn.clicked.connect(self.read_all_parameters)
        quick_layout.addWidget(read_all_btn)

        quick_group.setLayout(quick_layout)
        layout.addWidget(quick_group)

        layout.addStretch()
        widget.setLayout(layout)
        return widget

    def create_pid_tab(self):
        """Create PID tuning tab"""
        widget = QWidget()
        layout = QVBoxLayout()

        # Angle Loop PID
        angle_group = QGroupBox("Angle Loop PID")
        angle_layout = QGridLayout()

        self.pid_spinboxes = {}

        pid_params = [
            ('angle_kp', 'Kp', 0, 10000),
            ('angle_ki', 'Ki', 0, 10000),
            ('angle_kd', 'Kd', 0, 10000),
            ('angle_kv', 'Kv', 0, 10000),
        ]

        row = 0
        for key, name, min_val, max_val in pid_params:
            label = QLabel(f"{name}:")
            angle_layout.addWidget(label, row, 0)

            spinbox = QSpinBox()
            spinbox.setMinimum(min_val)
            spinbox.setMaximum(max_val)
            spinbox.setValue(0)
            angle_layout.addWidget(spinbox, row, 1)

            read_btn = QPushButton("Read")
            read_btn.clicked.connect(lambda checked, k=key: self.read_parameter(k))
            angle_layout.addWidget(read_btn, row, 2)

            write_btn = QPushButton("Write")
            write_btn.clicked.connect(lambda checked, k=key: self.write_parameter(k))
            angle_layout.addWidget(write_btn, row, 3)

            self.pid_spinboxes[key] = spinbox
            row += 1

        angle_group.setLayout(angle_layout)
        layout.addWidget(angle_group)

        # Pressure Loop PID
        pressure_group = QGroupBox("Pressure Loop PID")
        pressure_layout = QGridLayout()

        pid_params = [
            ('pressure_kp', 'Kp', 0, 10000),
            ('pressure_ki', 'Ki', 0, 10000),
            ('pressure_kd', 'Kd', 0, 10000),
            ('pressure_kv', 'Kv', 0, 10000),
        ]

        row = 0
        for key, name, min_val, max_val in pid_params:
            label = QLabel(f"{name}:")
            pressure_layout.addWidget(label, row, 0)

            spinbox = QSpinBox()
            spinbox.setMinimum(min_val)
            spinbox.setMaximum(max_val)
            spinbox.setValue(0)
            pressure_layout.addWidget(spinbox, row, 1)

            read_btn = QPushButton("Read")
            read_btn.clicked.connect(lambda checked, k=key: self.read_parameter(k))
            pressure_layout.addWidget(read_btn, row, 2)

            write_btn = QPushButton("Write")
            write_btn.clicked.connect(lambda checked, k=key: self.write_parameter(k))
            pressure_layout.addWidget(write_btn, row, 3)

            self.pid_spinboxes[key] = spinbox
            row += 1

        pressure_group.setLayout(pressure_layout)
        layout.addWidget(pressure_group)

        # Current Loop A PID
        current_group = QGroupBox("Current Loop A PID")
        current_layout = QGridLayout()

        pid_params = [
            ('current_a_kp', 'Kp', 0, 10000),
            ('current_a_ki', 'Ki', 0, 30000),
            ('current_a_kd', 'Kd', 0, 10000),
        ]

        row = 0
        for key, name, min_val, max_val in pid_params:
            label = QLabel(f"{name}:")
            current_layout.addWidget(label, row, 0)

            spinbox = QSpinBox()
            spinbox.setMinimum(min_val)
            spinbox.setMaximum(max_val)
            spinbox.setValue(0)
            current_layout.addWidget(spinbox, row, 1)

            read_btn = QPushButton("Read")
            read_btn.clicked.connect(lambda checked, k=key: self.read_parameter(k))
            current_layout.addWidget(read_btn, row, 2)

            write_btn = QPushButton("Write")
            write_btn.clicked.connect(lambda checked, k=key: self.write_parameter(k))
            current_layout.addWidget(write_btn, row, 3)

            self.pid_spinboxes[key] = spinbox
            row += 1

        current_group.setLayout(current_layout)
        layout.addWidget(current_group)

        layout.addStretch()
        widget.setLayout(layout)
        return widget

    def create_calibration_tab(self):
        """Create calibration tab"""
        widget = QWidget()
        layout = QVBoxLayout()

        info_label = QLabel("Calibration parameters (Advanced users only)")
        info_label.setStyleSheet("QLabel { color: red; font-weight: bold; }")
        layout.addWidget(info_label)

        layout.addWidget(QLabel("This tab will contain ADC calibration values."))
        layout.addWidget(QLabel("Implementation coming soon..."))

        layout.addStretch()
        widget.setLayout(layout)
        return widget

    # Connection handlers
    def refresh_ports(self):
        """Refresh available serial ports"""
        self.port_combo.clear()
        ports = self.serial.list_ports()
        for port, desc in ports:
            self.port_combo.addItem(f"{port} - {desc}", port)

    def toggle_connection(self):
        """Toggle serial connection"""
        if not self.serial.is_connected:
            port = self.port_combo.currentData()
            baudrate = int(self.baudrate_combo.currentText())

            if port:
                self.serial.connect(port, baudrate)
            else:
                QMessageBox.warning(self, "Error", "Please select a serial port")
        else:
            self.serial.disconnect()

    @pyqtSlot()
    def on_connected(self):
        """Handle connection established"""
        self.connect_btn.setText("Disconnect")
        self.connect_btn.setStyleSheet("QPushButton { background-color: green; color: white; }")
        self.statusBar().showMessage("Connected")
        self.monitor_timer.start(self.monitor_interval)

    @pyqtSlot()
    def on_disconnected(self):
        """Handle disconnection"""
        self.connect_btn.setText("Connect")
        self.connect_btn.setStyleSheet("")
        self.statusBar().showMessage("Disconnected")
        self.monitor_timer.stop()

    @pyqtSlot(int, int)
    def on_data_received(self, index, value):
        """Handle received data"""
        # Update parameter cache
        self.param_cache[index] = value

    @pyqtSlot(str)
    def on_error(self, error_msg):
        """Handle communication error"""
        self.statusBar().showMessage(f"Error: {error_msg}", 5000)

    # Monitoring
    def update_monitoring(self):
        """Update real-time monitoring displays"""
        if not self.serial.is_connected:
            return

        # Read feedback parameters
        feedback_params = ['angle_feedback', 'pressure_feedback',
                          'current_a_feedback', 'current_b_feedback',
                          'pwm_a', 'pwm_b']

        for key in feedback_params:
            param = ParameterInfo.PARAMETERS[key]
            value = self.serial.read_parameter(param['index'])

            if value is not None:
                scaled_value = value * param['scale']

                # Update display label
                if key in self.value_labels:
                    self.value_labels[key].setText(f"{scaled_value:.2f} {param['unit']}")

                # Update plot data
                current_time = time.time() - self.start_time
                if key == 'angle_feedback':
                    self.plot_data['time'].append(current_time)
                    self.plot_data['angle_fb'].append(scaled_value)
                elif key == 'pressure_feedback':
                    self.plot_data['pressure_fb'].append(scaled_value)
                elif key == 'current_a_feedback':
                    self.plot_data['current_a_fb'].append(scaled_value)
                elif key == 'current_b_feedback':
                    self.plot_data['current_b_fb'].append(scaled_value)

        # Update plots
        self.update_plots()

    def update_plots(self):
        """Update real-time plots"""
        if len(self.plot_data['time']) == 0:
            return

        time_array = list(self.plot_data['time'])

        plot_keys = ['angle_fb', 'pressure_fb', 'current_a_fb', 'current_b_fb']

        for key in plot_keys:
            if key in self.plots:
                data = list(self.plot_data[key])
                if len(data) > 0:
                    self.plots[key]['curve'].setData(time_array, data)

    # Control handlers
    def on_enable_changed(self, key, state):
        """Handle enable checkbox changed"""
        param = ParameterInfo.PARAMETERS[key]
        value = 1 if state == Qt.Checked else 0
        success = self.serial.write_parameter(param['index'], value)

        if not success:
            # Revert checkbox if write failed
            self.enable_checkboxes[key].blockSignals(True)
            self.enable_checkboxes[key].setChecked(not bool(value))
            self.enable_checkboxes[key].blockSignals(False)
            QMessageBox.warning(self, "Error", f"Failed to write {param['name']}")

    def apply_setpoint(self, key):
        """Apply setpoint value"""
        param = ParameterInfo.PARAMETERS[key]
        spinbox = self.setpoint_spinboxes[key]

        # Convert scaled value back to raw value
        scaled_value = spinbox.value()
        raw_value = int(scaled_value / param['scale'])

        success = self.serial.write_parameter(param['index'], raw_value)

        if success:
            self.statusBar().showMessage(f"{param['name']} set to {scaled_value:.2f}", 3000)
        else:
            QMessageBox.warning(self, "Error", f"Failed to write {param['name']}")

    def emergency_stop(self):
        """Emergency stop - disable all loops and zero setpoints"""
        reply = QMessageBox.question(
            self, "Emergency Stop",
            "This will disable all control loops and zero all setpoints. Continue?",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # Disable all enable flags
            for key in self.enable_checkboxes.keys():
                param = ParameterInfo.PARAMETERS[key]
                self.serial.write_parameter(param['index'], 0)
                self.enable_checkboxes[key].setChecked(False)

            # Zero all setpoints
            self.zero_all_setpoints()

            self.statusBar().showMessage("EMERGENCY STOP EXECUTED", 5000)

    def zero_all_setpoints(self):
        """Zero all setpoint values"""
        for key in self.setpoint_spinboxes.keys():
            self.setpoint_spinboxes[key].setValue(0)
            param = ParameterInfo.PARAMETERS[key]
            self.serial.write_parameter(param['index'], 0)

    def read_all_parameters(self):
        """Read all parameters from controller"""
        if not self.serial.is_connected:
            QMessageBox.warning(self, "Error", "Not connected")
            return

        # This could take a while...
        self.statusBar().showMessage("Reading all parameters...")

        # Read enable flags
        for key, checkbox in self.enable_checkboxes.items():
            param = ParameterInfo.PARAMETERS[key]
            value = self.serial.read_parameter(param['index'])
            if value is not None:
                checkbox.setChecked(bool(value))

        # Read setpoints
        for key, spinbox in self.setpoint_spinboxes.items():
            param = ParameterInfo.PARAMETERS[key]
            value = self.serial.read_parameter(param['index'])
            if value is not None:
                scaled_value = value * param['scale']
                spinbox.setValue(scaled_value)

        # Read PID parameters
        for key, spinbox in self.pid_spinboxes.items():
            param = ParameterInfo.PARAMETERS[key]
            value = self.serial.read_parameter(param['index'])
            if value is not None:
                spinbox.setValue(value)

        self.statusBar().showMessage("All parameters read", 3000)

    def read_parameter(self, key):
        """Read single parameter"""
        param = ParameterInfo.PARAMETERS[key]
        value = self.serial.read_parameter(param['index'])

        if value is not None:
            if key in self.pid_spinboxes:
                self.pid_spinboxes[key].setValue(value)
            self.statusBar().showMessage(f"{param['name']}: {value}", 3000)
        else:
            QMessageBox.warning(self, "Error", f"Failed to read {param['name']}")

    def write_parameter(self, key):
        """Write single parameter"""
        param = ParameterInfo.PARAMETERS[key]

        if key in self.pid_spinboxes:
            value = self.pid_spinboxes[key].value()
        else:
            return

        success = self.serial.write_parameter(param['index'], value)

        if success:
            self.statusBar().showMessage(f"{param['name']} written: {value}", 3000)
        else:
            QMessageBox.warning(self, "Error", f"Failed to write {param['name']}")

    def closeEvent(self, event):
        """Handle window close"""
        if self.serial.is_connected:
            self.serial.disconnect()
        event.accept()


def main():
    """Main entry point"""
    app = QApplication(sys.argv)

    # Set application style
    app.setStyle('Fusion')

    # Create and show main window
    window = PumpBoardGUI()
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
