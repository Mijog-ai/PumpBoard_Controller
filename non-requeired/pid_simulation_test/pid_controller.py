"""
PID Controller Implementation for Educational Testing

This module provides a flexible PID controller class that helps you understand
how Proportional-Integral-Derivative control works.

PID Control Basics:
- P (Proportional): Reacts to current error (fast response, but may oscillate)
- I (Integral): Eliminates steady-state error (but can cause overshoot)
- D (Derivative): Reduces overshoot and oscillation (but sensitive to noise)
"""

import numpy as np


class PIDController:
    """
    A Position-form PID Controller

    This controller calculates the control output based on the error between
    the setpoint (target) and the process variable (current value).

    The control equation is:
    output = Kp * error + Ki * sum(error) + Kd * d(error)/dt

    Attributes:
        Kp: Proportional gain
        Ki: Integral gain
        Kd: Derivative gain
        setpoint: Target value we want to reach
        output_limits: Tuple (min, max) for output saturation
        integral_limits: Tuple (min, max) to prevent integral windup
    """

    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0.0,
                 output_limits=(-100, 100), integral_limits=None,
                 sample_time=0.01):
        """
        Initialize the PID controller

        Args:
            Kp: Proportional gain (how much to react to current error)
            Ki: Integral gain (how much to react to accumulated error)
            Kd: Derivative gain (how much to react to error rate of change)
            setpoint: The target value
            output_limits: (min, max) tuple for output clamping
            integral_limits: (min, max) tuple for integral term (prevents windup)
            sample_time: Time step for discrete control (seconds)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        # State variables
        self.current_error = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.output = 0.0

        # Limits
        self.output_limits = output_limits
        if integral_limits is None:
            # Default integral limits based on output limits and Ki
            if Ki != 0:
                self.integral_limits = (output_limits[0] / Ki, output_limits[1] / Ki)
            else:
                self.integral_limits = (-1000, 1000)
        else:
            self.integral_limits = integral_limits

        # History for analysis
        self.error_history = []
        self.output_history = []
        self.p_term_history = []
        self.i_term_history = []
        self.d_term_history = []

    def update(self, current_value):
        """
        Calculate PID output based on current process value

        Args:
            current_value: Current measured value from the system

        Returns:
            Control output value (clamped to output_limits)
        """
        # Calculate error
        self.current_error = self.setpoint - current_value

        # Proportional term
        p_term = self.Kp * self.current_error

        # Integral term (with anti-windup)
        self.integral += self.current_error * self.sample_time
        self.integral = np.clip(self.integral, self.integral_limits[0], self.integral_limits[1])
        i_term = self.Ki * self.integral

        # Derivative term (rate of error change)
        self.derivative = (self.current_error - self.last_error) / self.sample_time
        d_term = self.Kd * self.derivative

        # Calculate total output
        self.output = p_term + i_term + d_term

        # Clamp output to limits
        self.output = np.clip(self.output, self.output_limits[0], self.output_limits[1])

        # Update last error
        self.last_error = self.current_error

        # Store history
        self.error_history.append(self.current_error)
        self.output_history.append(self.output)
        self.p_term_history.append(p_term)
        self.i_term_history.append(i_term)
        self.d_term_history.append(d_term)

        return self.output

    def set_tunings(self, Kp=None, Ki=None, Kd=None):
        """Update PID gains on the fly"""
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd

    def set_setpoint(self, setpoint):
        """Change the target value"""
        self.setpoint = setpoint

    def reset(self):
        """Reset the controller state (useful when starting new test)"""
        self.current_error = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.output = 0.0
        self.error_history = []
        self.output_history = []
        self.p_term_history = []
        self.i_term_history = []
        self.d_term_history = []

    def get_components(self):
        """
        Get the individual P, I, D components for analysis

        Returns:
            Dictionary with current P, I, D values
        """
        return {
            'P': self.Kp * self.current_error if self.error_history else 0,
            'I': self.Ki * self.integral if self.error_history else 0,
            'D': self.Kd * self.derivative if self.error_history else 0,
            'error': self.current_error,
            'output': self.output
        }

    def __repr__(self):
        return f"PIDController(Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}, setpoint={self.setpoint})"


class PIDControllerWithVelocity(PIDController):
    """
    Extended PID Controller with Velocity term (similar to the pump controller)

    This adds a Kv term that reacts to the rate of change of the setpoint,
    which can help with feed-forward control.
    """

    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, Kv=0.0, setpoint=0.0,
                 output_limits=(-100, 100), integral_limits=None,
                 sample_time=0.01):
        super().__init__(Kp, Ki, Kd, setpoint, output_limits, integral_limits, sample_time)
        self.Kv = Kv
        self.last_setpoint = setpoint
        self.v_term_history = []

    def update(self, current_value):
        """
        Calculate PID output with velocity term
        """
        # Get base PID output
        base_output = super().update(current_value)

        # Velocity term (feed-forward based on setpoint change rate)
        setpoint_velocity = (self.setpoint - self.last_setpoint) / self.sample_time
        v_term = self.Kv * setpoint_velocity

        # Add velocity term to output
        self.output = base_output + v_term
        self.output = np.clip(self.output, self.output_limits[0], self.output_limits[1])

        # Update state
        self.last_setpoint = self.setpoint
        self.v_term_history.append(v_term)
        self.output_history[-1] = self.output

        return self.output

    def reset(self):
        super().reset()
        self.last_setpoint = self.setpoint
        self.v_term_history = []
