"""
PID Controller Implementation
Based on PumpBoard_Car_PPQ1_Single firmware
"""

class PIDController:
    """Position-based PID Controller with anti-windup and output limiting"""

    def __init__(self, kp, ki, kd, kv=0,
                 kp_div=1, ki_div=1, kd_div=1, kv_div=1,
                 output_min=0, output_max=100,
                 integral_area=0, integral_area_lower=0,
                 area_i=0, err_d_period=1):
        """
        Initialize PID controller

        Args:
            kp: Proportional gain coefficient
            ki: Integral gain coefficient
            kd: Derivative gain coefficient
            kv: Velocity gain coefficient (for feedforward)
            kp_div, ki_div, kd_div, kv_div: Divisors for gain scaling
            output_min, output_max: Output limits
            integral_area: Error threshold for integral accumulation (upper)
            integral_area_lower: Error threshold for integral accumulation (lower)
            area_i: Alternative integral gain within area
            err_d_period: Derivative error period
        """
        self.kp = kp / kp_div if kp_div != 0 else kp
        self.ki = ki / ki_div if ki_div != 0 else ki
        self.kd = kd / kd_div if kd_div != 0 else kd
        self.kv = kv / kv_div if kv_div != 0 else kv

        self.output_min = output_min
        self.output_max = output_max

        # Integral limits
        if ki != 0:
            self.sum_max = output_max * ki_div / ki
            self.sum_min = output_min * ki_div / ki
        else:
            self.sum_max = output_max
            self.sum_min = output_min

        self.integral_area = integral_area
        self.integral_area_lower = integral_area_lower
        self.area_i = area_i
        self.err_d_period = err_d_period

        # State variables
        self.cur_error = 0
        self.last_error = 0
        self.sum_error = 0
        self.kv_err = 0
        self.output = 0

    def reset(self):
        """Reset controller state"""
        self.cur_error = 0
        self.last_error = 0
        self.sum_error = 0
        self.kv_err = 0
        self.output = 0

    def update(self, setpoint, feedback, derivative_feedback=None):
        """
        Update PID controller

        Args:
            setpoint: Desired value
            feedback: Current measured value
            derivative_feedback: Optional separate derivative term

        Returns:
            Control output
        """
        # Calculate error
        self.last_error = self.cur_error
        self.cur_error = setpoint - feedback

        # Proportional term
        p_term = self.kp * self.cur_error

        # Integral term with anti-windup
        # Only accumulate if error is within specified area
        if self.integral_area > 0:
            if abs(self.cur_error) <= self.integral_area and abs(self.cur_error) >= self.integral_area_lower:
                if self.area_i > 0:
                    self.sum_error += self.cur_error * self.area_i / 10
                else:
                    self.sum_error += self.cur_error
            # Clamp integral
            self.sum_error = max(min(self.sum_error, self.sum_max), self.sum_min)
        else:
            self.sum_error += self.cur_error
            # Clamp integral
            self.sum_error = max(min(self.sum_error, self.sum_max), self.sum_min)

        i_term = self.ki * self.sum_error

        # Derivative term
        if derivative_feedback is not None:
            d_term = self.kd * derivative_feedback
            v_term = self.kv * derivative_feedback
        else:
            d_error = self.cur_error - self.last_error
            d_term = self.kd * d_error
            v_term = self.kv * d_error

        # Calculate output
        self.output = p_term + i_term + d_term + v_term

        # Clamp output
        self.output = max(min(self.output, self.output_max), self.output_min)

        return self.output

    def get_components(self):
        """Return individual PID components for debugging"""
        return {
            'p': self.kp * self.cur_error,
            'i': self.ki * self.sum_error,
            'd': self.kd * (self.cur_error - self.last_error),
            'error': self.cur_error,
            'integral': self.sum_error
        }


class CurrentPIDController(PIDController):
    """Current loop PID controller (inner loop)"""

    def __init__(self):
        # From firmware: CUR_A_KP=50, KI=6500, KD=0
        # Dividers: KP_DIV=2048, KI_DIV=8192, KD_DIV=10
        # Output: 0 to 16800 (80% of PWM_ARR=21000)
        super().__init__(
            kp=50, ki=6500, kd=0, kv=0,
            kp_div=2048, ki_div=8192, kd_div=10, kv_div=1,
            output_min=0, output_max=16800,
            integral_area=0, integral_area_lower=0,
            area_i=0, err_d_period=1
        )


class AnglePIDController(PIDController):
    """Angle loop PID controller (middle loop)"""

    def __init__(self):
        # From firmware: ANGLE_A_KP=150, KI=0, KD=1500, KV=1000
        # Dividers: KP_DIV=400, KI_DIV=32000, KD_DIV=450, KV_DIV=350
        # Output: -2500 to 2500 (current setpoint)
        super().__init__(
            kp=150, ki=0, kd=1500, kv=1000,
            kp_div=400, ki_div=32000, kd_div=450, kv_div=350,
            output_min=-2500, output_max=2500,
            integral_area=200, integral_area_lower=0,
            area_i=20, err_d_period=1
        )


class PressurePIDController(PIDController):
    """Pressure loop PID controller (outer loop)"""

    def __init__(self):
        # From firmware: PRS_A_KP=1600, KI=0, KD=515, KV=515
        # Dividers: KP_DIV=300, KI_DIV=200, KD_DIV=100, KV_DIV=100
        # Output: 0 to 5000 (angle setpoint in per-unit)
        super().__init__(
            kp=1600, ki=0, kd=515, kv=515,
            kp_div=300, ki_div=200, kd_div=100, kv_div=100,
            output_min=0, output_max=5000,
            integral_area=100, integral_area_lower=0,
            area_i=10, err_d_period=1
        )
