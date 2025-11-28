"""
Plant Models - Different Systems to Control with PID

This module contains various "plants" (systems) that you can control with PID.
Each plant has different dynamics and characteristics to help you learn how
PID behaves with different types of systems.

Plant Types:
1. Simple First-Order System (e.g., temperature control)
2. Second-Order System with inertia (e.g., motor position)
3. Pump Pressure System (similar to your actual controller)
4. Water Tank Level System
"""

import numpy as np


class PlantBase:
    """Base class for all plant models"""

    def __init__(self, initial_value=0.0, sample_time=0.01):
        self.value = initial_value
        self.sample_time = sample_time
        self.history = [initial_value]

    def update(self, control_input):
        """Update plant state based on control input"""
        raise NotImplementedError

    def reset(self, initial_value=0.0):
        """Reset plant to initial state"""
        self.value = initial_value
        self.history = [initial_value]

    def add_noise(self, noise_level=0.0):
        """Add measurement noise to simulate real sensors"""
        if noise_level > 0:
            return self.value + np.random.normal(0, noise_level)
        return self.value


class FirstOrderSystem(PlantBase):
    """
    Simple First-Order System (like temperature control)

    This is the simplest dynamic system, described by:
    dy/dt = (input - y) / tau

    Where tau is the time constant (how fast the system responds).

    Good for learning:
    - Basic PID tuning
    - Understanding time constants
    - No overshoot (unless PID is poorly tuned)

    Example: Heating element - when you apply power, temperature rises gradually
    """

    def __init__(self, time_constant=1.0, gain=1.0, initial_value=0.0, sample_time=0.01):
        """
        Args:
            time_constant: How fast the system responds (seconds)
                          Larger = slower response
            gain: How much the input affects the output
            initial_value: Starting value
            sample_time: Simulation time step
        """
        super().__init__(initial_value, sample_time)
        self.tau = time_constant
        self.gain = gain

    def update(self, control_input):
        """
        Update the system state

        Args:
            control_input: Control signal from PID controller

        Returns:
            Current value of the system
        """
        # First-order differential equation: dy/dt = (K*u - y) / tau
        # Discrete approximation: y[n+1] = y[n] + dt/tau * (K*u - y[n])
        dydt = (self.gain * control_input - self.value) / self.tau
        self.value += dydt * self.sample_time
        self.history.append(self.value)
        return self.value


class SecondOrderSystem(PlantBase):
    """
    Second-Order System (like motor position or mass-spring-damper)

    This system has inertia and can oscillate, described by:
    m * d²y/dt² + c * dy/dt + k * y = input

    Good for learning:
    - Dealing with overshoot and oscillations
    - Effect of D term on damping
    - More realistic mechanical systems

    Example: Motor rotating to a position - has inertia and friction
    """

    def __init__(self, mass=1.0, damping=2.0, stiffness=10.0,
                 gain=1.0, initial_value=0.0, sample_time=0.01):
        """
        Args:
            mass: Inertia of the system (larger = slower, more overshoot)
            damping: Friction/resistance (larger = less oscillation)
            stiffness: Spring constant (larger = faster response)
            gain: Input to force conversion
            initial_value: Starting position
            sample_time: Simulation time step
        """
        super().__init__(initial_value, sample_time)
        self.mass = mass
        self.damping = damping
        self.stiffness = stiffness
        self.gain = gain
        self.velocity = 0.0
        self.velocity_history = [0.0]

    def update(self, control_input):
        """
        Update position and velocity based on control input

        Second order dynamics:
        acceleration = (force - damping*velocity - stiffness*position) / mass
        """
        force = self.gain * control_input

        # Calculate acceleration from forces
        acceleration = (force - self.damping * self.velocity - self.stiffness * self.value) / self.mass

        # Update velocity and position
        self.velocity += acceleration * self.sample_time
        self.value += self.velocity * self.sample_time

        self.history.append(self.value)
        self.velocity_history.append(self.velocity)

        return self.value

    def reset(self, initial_value=0.0):
        super().reset(initial_value)
        self.velocity = 0.0
        self.velocity_history = [0.0]


class PumpPressureSystem(PlantBase):
    """
    Pump Pressure Control System (similar to your actual controller!)

    This models a pump that builds pressure in a system with leakage.
    - Control input = pump speed/power
    - Output = pressure in the system
    - Leakage causes pressure to drop when pump is off

    Good for learning:
    - Real-world nonlinear behavior
    - Systems with disturbances (leakage)
    - Pressure control like in your hardware
    """

    def __init__(self, pump_efficiency=0.5, leakage_coefficient=0.1,
                 pressure_capacity=100.0, initial_pressure=0.0, sample_time=0.01):
        """
        Args:
            pump_efficiency: How effectively pump input creates pressure
            leakage_coefficient: How fast pressure leaks out
            pressure_capacity: Maximum pressure the system can hold
            initial_pressure: Starting pressure
            sample_time: Simulation time step
        """
        super().__init__(initial_pressure, sample_time)
        self.efficiency = pump_efficiency
        self.leakage = leakage_coefficient
        self.max_pressure = pressure_capacity
        self.flow_history = []

    def update(self, control_input):
        """
        Update pressure based on pump input and leakage

        dP/dt = pump_flow - leakage_flow
        pump_flow = efficiency * control_input
        leakage_flow = leakage_coefficient * current_pressure
        """
        # Pump adds pressure (proportional to control input)
        pump_flow = self.efficiency * max(0, control_input)

        # Leakage removes pressure (proportional to current pressure)
        leakage_flow = self.leakage * self.value

        # Net change in pressure
        dP = pump_flow - leakage_flow

        # Update pressure
        self.value += dP * self.sample_time

        # Clamp to physical limits
        self.value = np.clip(self.value, 0, self.max_pressure)

        self.history.append(self.value)
        self.flow_history.append(pump_flow - leakage_flow)

        return self.value


class WaterTankSystem(PlantBase):
    """
    Water Tank Level Control System

    Control the water level in a tank with:
    - Inlet flow (controlled by valve/pump)
    - Outlet flow (drain at bottom)

    Good for learning:
    - Nonlinear behavior (outlet flow depends on level)
    - Practical fluid system control
    - Integration behavior
    """

    def __init__(self, tank_area=1.0, outlet_coefficient=0.1,
                 max_level=10.0, initial_level=0.0, sample_time=0.01):
        """
        Args:
            tank_area: Cross-sectional area of tank (larger = slower level change)
            outlet_coefficient: How fast water drains (larger = faster drain)
            max_level: Maximum water level
            initial_level: Starting water level
            sample_time: Simulation time step
        """
        super().__init__(initial_level, sample_time)
        self.area = tank_area
        self.outlet_coef = outlet_coefficient
        self.max_level = max_level
        self.inflow_history = []
        self.outflow_history = []

    def update(self, control_input):
        """
        Update water level based on inflow and outflow

        dh/dt = (inlet_flow - outlet_flow) / area
        outlet_flow = outlet_coefficient * sqrt(level)  # Torricelli's law
        """
        # Inlet flow from control (valve opening)
        inlet_flow = max(0, control_input)

        # Outlet flow depends on water level (sqrt relationship)
        outlet_flow = self.outlet_coef * np.sqrt(max(0, self.value))

        # Net change in level
        dh = (inlet_flow - outlet_flow) / self.area

        # Update level
        self.value += dh * self.sample_time

        # Clamp to physical limits
        self.value = np.clip(self.value, 0, self.max_level)

        self.history.append(self.value)
        self.inflow_history.append(inlet_flow)
        self.outflow_history.append(outlet_flow)

        return self.value


class MotorSpeedSystem(PlantBase):
    """
    DC Motor Speed Control

    Simple motor model with:
    - Electrical time constant (current buildup)
    - Mechanical inertia
    - Friction/load torque

    Good for learning:
    - Current control (inner loop)
    - Speed control (outer loop)
    - Cascaded control systems
    """

    def __init__(self, inertia=0.01, friction=0.1, motor_constant=1.0,
                 initial_speed=0.0, sample_time=0.01):
        """
        Args:
            inertia: Rotational inertia (larger = harder to change speed)
            friction: Friction coefficient (larger = more resistance)
            motor_constant: Voltage to torque conversion
            initial_speed: Starting speed (RPM or rad/s)
            sample_time: Simulation time step
        """
        super().__init__(initial_speed, sample_time)
        self.inertia = inertia
        self.friction = friction
        self.Kt = motor_constant  # Torque constant
        self.current_history = []

    def update(self, control_input):
        """
        Update motor speed based on applied voltage

        Torque = Kt * current ≈ Kt * voltage (simplified)
        d(speed)/dt = (Torque - friction*speed) / inertia
        """
        # Simplified: assume current follows voltage instantly
        torque = self.Kt * control_input

        # Angular acceleration
        friction_torque = self.friction * self.value
        acceleration = (torque - friction_torque) / self.inertia

        # Update speed
        self.value += acceleration * self.sample_time

        # Speed can't be negative in this simple model
        self.value = max(0, self.value)

        self.history.append(self.value)
        self.current_history.append(control_input)

        return self.value
