"""
Hydraulic Pump Plant Model
Simulates the physical behavior of the hydraulic pump system
"""
import numpy as np


class HydraulicPumpPlant:
    """
    Simulates a variable displacement hydraulic pump with:
    - Current-controlled solenoid valves
    - Swash plate angle control
    - Pressure generation based on displacement and load
    """

    def __init__(self, dt=0.0001):
        """
        Initialize pump plant model

        Args:
            dt: Simulation time step (100us = 0.0001s matches firmware)
        """
        self.dt = dt

        # Physical parameters
        self.max_displacement = 28  # cc/rev (from firmware PUMP_MAX_CC_VAL)
        self.max_angle = 5000  # Per unit (0-5000 from firmware ANG_PERUNIT_SCOPE)
        self.max_pressure = 600  # bar (from firmware)
        self.max_current = 3000  # mA

        # Current dynamics (solenoid valve response)
        self.current_a = 0  # mA
        self.current_b = 0  # mA
        self.current_time_constant = 0.005  # 5ms time constant

        # Angle dynamics (mechanical swash plate)
        self.angle = 0  # Per unit (0-5000)
        self.angle_velocity = 0  # Per unit/s
        self.angle_time_constant = 0.02  # 20ms time constant
        self.angle_inertia = 0.01  # Mechanical inertia

        # Pressure dynamics (hydraulic system)
        self.pressure = 0  # bar
        self.pressure_velocity = 0  # bar/s
        self.pressure_time_constant = 0.05  # 50ms time constant

        # System parameters
        self.leakage_coefficient = 0.0005  # Internal leakage
        self.load_torque = 0  # External load
        self.pump_speed = 1500  # RPM (assumed constant)

        # Noise and disturbances
        self.noise_enabled = True
        self.noise_amplitude = 0.01  # 1% noise

    def update(self, pwm_a, pwm_b):
        """
        Update plant model for one time step

        Args:
            pwm_a: PWM command for solenoid A (0-21000)
            pwm_b: PWM command for solenoid B (0-21000)

        Returns:
            dict with current state: current_a, current_b, angle, pressure
        """
        # Convert PWM to current command
        # PWM range: 0-21000, Current range: 0-3000mA
        current_a_cmd = (pwm_a / 21000.0) * self.max_current
        current_b_cmd = (pwm_b / 21000.0) * self.max_current

        # Simulate current dynamics (first-order lag)
        self.current_a += (current_a_cmd - self.current_a) * self.dt / self.current_time_constant
        self.current_b += (current_b_cmd - self.current_b) * self.dt / self.current_time_constant

        # Calculate net current (differential control)
        net_current = self.current_a - self.current_b

        # Convert current to angle command (solenoid force)
        # Net current range: -3000 to +3000 mA
        # Angle range: 0 to 5000 per unit
        # Map: -3000mA -> 0, 0mA -> 2500, +3000mA -> 5000
        angle_cmd = 2500 + (net_current / 3000.0) * 2500

        # Clamp angle command
        angle_cmd = max(0, min(self.max_angle, angle_cmd))

        # Simulate angle dynamics (second-order system)
        angle_error = angle_cmd - self.angle
        angle_accel = (angle_error / self.angle_time_constant - self.angle_velocity) / self.angle_inertia
        self.angle_velocity += angle_accel * self.dt
        self.angle += self.angle_velocity * self.dt

        # Clamp angle
        self.angle = max(0, min(self.max_angle, self.angle))

        # Calculate displacement (proportional to angle)
        displacement = (self.angle / self.max_angle) * self.max_displacement

        # Simulate pressure generation
        # Pressure depends on displacement, speed, and load
        flow_rate = (displacement * self.pump_speed) / 1000  # L/min
        leakage = self.pressure * self.leakage_coefficient

        # Pressure change rate
        pressure_cmd = flow_rate * 10 - leakage * 100  # Simplified model

        # Pressure dynamics (first-order lag)
        pressure_error = pressure_cmd - self.pressure
        self.pressure += pressure_error * self.dt / self.pressure_time_constant

        # Clamp pressure
        self.pressure = max(0, min(self.max_pressure, self.pressure))

        # Add noise if enabled
        if self.noise_enabled:
            current_noise = np.random.normal(0, self.noise_amplitude * self.max_current)
            angle_noise = np.random.normal(0, self.noise_amplitude * self.max_angle)
            pressure_noise = np.random.normal(0, self.noise_amplitude * self.max_pressure)

            current_a_meas = self.current_a + current_noise
            current_b_meas = self.current_b + current_noise
            angle_meas = self.angle + angle_noise
            pressure_meas = self.pressure + pressure_noise
        else:
            current_a_meas = self.current_a
            current_b_meas = self.current_b
            angle_meas = self.angle
            pressure_meas = self.pressure

        return {
            'current_a': max(0, current_a_meas),
            'current_b': max(0, current_b_meas),
            'angle': max(0, min(self.max_angle, angle_meas)),
            'pressure': max(0, min(self.max_pressure, pressure_meas)),
            'pwm_a': pwm_a,
            'pwm_b': pwm_b
        }

    def set_disturbance(self, load_torque=None, noise_enabled=None):
        """Set external disturbances and noise"""
        if load_torque is not None:
            self.load_torque = load_torque
        if noise_enabled is not None:
            self.noise_enabled = noise_enabled

    def reset(self):
        """Reset plant to initial state"""
        self.current_a = 0
        self.current_b = 0
        self.angle = 0
        self.angle_velocity = 0
        self.pressure = 0
        self.pressure_velocity = 0


class SensorSimulator:
    """Simulate ADC sensor readings with filtering"""

    def __init__(self):
        # From firmware calibration values
        self.ang_adc_min = 8902
        self.ang_adc_max = 15155
        self.ang_per_unit_scope = 5000

        self.prs_adc_min = 2000
        self.prs_adc_max = 34344
        self.prs_max_bar = 600

        # Current sensing parameters
        self.current_adc_gain = 1.0  # ADC counts per mA

        # Filter state
        self.ang_filtered = 0
        self.prs_filtered = 0
        self.ang_filter_const = 0.015  # From firmware
        self.prs_filter_const = 0.059  # From firmware

    def convert_angle_to_adc(self, angle_per_unit):
        """Convert angle in per-unit to ADC counts"""
        # Linear mapping
        adc = self.ang_adc_min + (angle_per_unit / self.ang_per_unit_scope) * (self.ang_adc_max - self.ang_adc_min)
        return int(adc)

    def convert_adc_to_angle(self, adc):
        """Convert ADC counts to angle in per-unit"""
        if adc <= self.ang_adc_min:
            return 0
        if adc >= self.ang_adc_max:
            return self.ang_per_unit_scope
        angle = ((adc - self.ang_adc_min) / (self.ang_adc_max - self.ang_adc_min)) * self.ang_per_unit_scope
        return angle

    def convert_pressure_to_adc(self, pressure_bar):
        """Convert pressure in bar to ADC counts"""
        adc = self.prs_adc_min + (pressure_bar / self.prs_max_bar) * (self.prs_adc_max - self.prs_adc_min)
        return int(adc)

    def convert_adc_to_pressure(self, adc):
        """Convert ADC counts to pressure in bar"""
        if adc <= self.prs_adc_min:
            return 0
        if adc >= self.prs_adc_max:
            return self.prs_max_bar
        pressure = ((adc - self.prs_adc_min) / (self.prs_adc_max - self.prs_adc_min)) * self.prs_max_bar
        return pressure

    def filter_angle(self, angle_raw):
        """Apply low-pass filter to angle feedback"""
        self.ang_filtered = self.ang_filtered + self.ang_filter_const * (angle_raw - self.ang_filtered)
        return self.ang_filtered

    def filter_pressure(self, pressure_raw):
        """Apply low-pass filter to pressure feedback"""
        self.prs_filtered = self.prs_filtered + self.prs_filter_const * (pressure_raw - self.prs_filtered)
        return self.prs_filtered
