"""
Test script for PumpBoard simulation
Tests basic functionality without GUI
"""
from pid_controller import CurrentPIDController, AnglePIDController, PressurePIDController
from pump_plant_model import HydraulicPumpPlant, SensorSimulator


def test_pid_controllers():
    """Test PID controller instantiation"""
    print("Testing PID Controllers...")

    current_pid = CurrentPIDController()
    print(f"✓ Current PID: Kp={current_pid.kp:.4f}, Ki={current_pid.ki:.4f}, Kd={current_pid.kd:.4f}")

    angle_pid = AnglePIDController()
    print(f"✓ Angle PID: Kp={angle_pid.kp:.4f}, Ki={angle_pid.ki:.4f}, Kd={angle_pid.kd:.4f}")

    pressure_pid = PressurePIDController()
    print(f"✓ Pressure PID: Kp={pressure_pid.kp:.4f}, Ki={pressure_pid.ki:.4f}, Kd={pressure_pid.kd:.4f}")

    print()


def test_plant_model():
    """Test plant model"""
    print("Testing Plant Model...")

    plant = HydraulicPumpPlant(dt=0.0001)
    sensor = SensorSimulator()

    # Run a few steps
    for i in range(10):
        state = plant.update(pwm_a=10000, pwm_b=5000)

    print(f"✓ Plant state after 10 steps:")
    print(f"  Current A: {state['current_a']:.2f} mA")
    print(f"  Current B: {state['current_b']:.2f} mA")
    print(f"  Angle: {state['angle']:.2f} pu")
    print(f"  Pressure: {state['pressure']:.2f} bar")

    print()


def test_cascade_control():
    """Test cascade control system"""
    print("Testing Cascade Control...")

    # Create controllers
    current_pid_a = CurrentPIDController()
    current_pid_b = CurrentPIDController()
    angle_pid = AnglePIDController()

    # Create plant
    plant = HydraulicPumpPlant(dt=0.0001)

    # Setpoints
    angle_setpoint = 3000  # Target 3000 pu
    current_a_setpoint = 1500
    current_b_setpoint = 1500

    # Run simulation for 100 steps
    pwm_a = 0
    pwm_b = 0

    print(f"  Target angle: {angle_setpoint} pu")

    for step in range(100):
        # Update plant
        state = plant.update(pwm_a, pwm_b)

        # Angle control
        angle_feedback = state['angle']
        angle_output = angle_pid.update(angle_setpoint, angle_feedback)

        # Convert to current setpoints
        if angle_output >= 0:
            current_a_setpoint = 1500 + angle_output
            current_b_setpoint = 1500
        else:
            current_a_setpoint = 1500
            current_b_setpoint = 1500 - angle_output

        # Clamp
        current_a_setpoint = max(0, min(3000, current_a_setpoint))
        current_b_setpoint = max(0, min(3000, current_b_setpoint))

        # Current control
        current_a_feedback = state['current_a']
        current_b_feedback = state['current_b']

        pwm_a = current_pid_a.update(current_a_setpoint, current_a_feedback)
        pwm_b = current_pid_b.update(current_b_setpoint, current_b_feedback)

        # Print progress every 25 steps
        if step % 25 == 0:
            error = angle_setpoint - angle_feedback
            print(f"  Step {step:3d}: Angle={angle_feedback:6.1f} pu, Error={error:6.1f}, PWM_A={pwm_a:6.1f}")

    print(f"✓ Final angle: {state['angle']:.1f} pu (target: {angle_setpoint} pu)")
    print()


def main():
    print("=" * 60)
    print("PumpBoard Simulation Test Suite")
    print("=" * 60)
    print()

    test_pid_controllers()
    test_plant_model()
    test_cascade_control()

    print("=" * 60)
    print("All tests completed successfully!")
    print("=" * 60)


if __name__ == '__main__':
    main()
