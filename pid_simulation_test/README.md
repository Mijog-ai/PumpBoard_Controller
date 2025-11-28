# PID Controller Simulation & Learning Tool

A comprehensive Python-based simulation environment for learning and experimenting with PID (Proportional-Integral-Derivative) controllers. This project helps you understand how PID control works without needing physical hardware.

## 📚 What is PID Control?

PID control is one of the most widely used control algorithms in industrial applications. It continuously calculates an error value as the difference between a desired setpoint and a measured process variable, then applies a correction based on:

- **P (Proportional)**: Reacts to the current error
  - Larger Kp → Faster response, but may overshoot and oscillate
  - Too small → Slow response, may not reach setpoint

- **I (Integral)**: Accumulates past errors to eliminate steady-state error
  - Removes offset between setpoint and actual value
  - Too large → Overshoot and instability
  - Too small → Slow to eliminate steady-state error

- **D (Derivative)**: Predicts future errors based on rate of change
  - Reduces overshoot and oscillation
  - Adds damping to the system
  - Too large → Sensitive to noise

## 🚀 Quick Start

### Installation

1. Make sure you have Python 3.7+ installed
2. Install dependencies:

```bash
cd pid_simulation_test
pip install -r requirements.txt
```

### Running the Simulations

#### 1. Pre-configured Examples
Run the main simulation with several examples:

```bash
python simulation.py
```

This will show you:
- Temperature control (First-order system)
- Motor position control (Second-order system)
- Pump pressure control (Similar to your hardware!)
- Comparison of P, PI, and PID control

#### 2. Interactive Tuning Tool
Launch the interactive tuner to experiment in real-time:

```bash
python interactive_tuning.py
```

Use the sliders to adjust:
- Kp, Ki, Kd parameters
- Setpoint value
- Plant type (different systems)

Watch how the system responds immediately to your changes!

#### 3. Cascade PID Tuning Tool (Three-Loop Control)
Two versions available - choose based on your performance needs:

**Standard Version** (Matplotlib-based):
```bash
python interactive_tuning_cascade.py
```

**High-Performance Version** (PyQt5-based) ⚡ **RECOMMENDED**:
```bash
python interactive_tuning_cascade_pyqt5.py
```

The PyQt5 version is much faster and more responsive!

Features:
- Three correlated control loops: Pressure → Angle → Current
- Real-time visualization of cascade effects
- Tab-based interface for easy loop selection
- Toggle between CASCADE and INDEPENDENT modes
- Based on actual PumpBoard controller structure

Perfect for understanding:
- How cascade control works
- Loop interaction and timing
- Multi-loop PID tuning strategies

## 📁 Project Structure

```
pid_simulation_test/
├── README.md                          # This file
├── requirements.txt                   # Python dependencies
├── pid_controller.py                  # PID controller implementation
├── plant_models.py                    # Different systems to control
├── simulation.py                      # Pre-configured simulation examples
├── interactive_tuning.py              # Interactive tuning tool with sliders
├── interactive_tuning_cascade.py      # Cascade PID tuner (matplotlib)
└── interactive_tuning_cascade_pyqt5.py # Cascade PID tuner (PyQt5, faster!) ⚡
```

## 🎯 Available Plant Models

### 1. First-Order System
**Example:** Temperature control, heating elements

Simple exponential response. Good for learning basic PID concepts.

```python
plant = FirstOrderSystem(time_constant=2.0, gain=1.0)
```

### 2. Second-Order System
**Example:** Motor position, mass-spring-damper

Has inertia and can oscillate. More realistic for mechanical systems.

```python
plant = SecondOrderSystem(mass=1.0, damping=5.0, stiffness=20.0)
```

### 3. Pump Pressure System
**Example:** Hydraulic pump with leakage (like your controller!)

Non-linear system with continuous disturbance (leakage).

```python
plant = PumpPressureSystem(pump_efficiency=0.8, leakage_coefficient=0.05)
```

### 4. Water Tank System
**Example:** Liquid level control

Non-linear outflow (depends on square root of level).

```python
plant = WaterTankSystem(tank_area=1.0, outlet_coefficient=0.1)
```

### 5. Motor Speed System
**Example:** DC motor speed control

With inertia and friction, good for understanding velocity control.

```python
plant = MotorSpeedSystem(inertia=0.01, friction=0.1)
```

## 🔧 Basic Usage Examples

### Example 1: Simple Temperature Control

```python
from pid_controller import PIDController
from plant_models import FirstOrderSystem
from simulation import PIDSimulation

# Create a heating system (time constant = 2 seconds)
heater = FirstOrderSystem(time_constant=2.0, gain=1.0, initial_value=20.0)

# Create PID controller to reach 100°C
pid = PIDController(Kp=2.0, Ki=0.5, Kd=0.1, setpoint=100.0)

# Run simulation
sim = PIDSimulation(pid, heater, duration=15.0)
results = sim.run()
sim.plot_results("My Temperature Controller")

import matplotlib.pyplot as plt
plt.show()
```

### Example 2: Experimenting with Different Gains

```python
from pid_controller import PIDController
from plant_models import SecondOrderSystem
from simulation import PIDSimulation
import matplotlib.pyplot as plt

# Create motor system
motor = SecondOrderSystem(mass=1.0, damping=3.0, stiffness=15.0)

# Try different PID settings
configs = [
    ("P only", 10.0, 0.0, 0.0),
    ("PI", 10.0, 2.0, 0.0),
    ("PID", 10.0, 2.0, 5.0),
]

fig, axes = plt.subplots(len(configs), 1, figsize=(10, 8))

for idx, (name, Kp, Ki, Kd) in enumerate(configs):
    motor.reset()
    pid = PIDController(Kp=Kp, Ki=Ki, Kd=Kd, setpoint=50.0)
    sim = PIDSimulation(pid, motor, duration=8.0)
    results = sim.run()

    axes[idx].plot(sim.time, sim.setpoint_history, 'r--', label='Setpoint')
    axes[idx].plot(sim.time, sim.process_value_history, 'b-', label='Response')
    axes[idx].set_title(f'{name}: Kp={Kp}, Ki={Ki}, Kd={Kd}')
    axes[idx].legend()
    axes[idx].grid(True)

plt.tight_layout()
plt.show()
```

### Example 3: Testing with Disturbances

```python
from pid_controller import PIDController
from plant_models import PumpPressureSystem
from simulation import PIDSimulation

# Create pump system
pump = PumpPressureSystem(pump_efficiency=0.8, leakage_coefficient=0.05)

# PID controller
pid = PIDController(Kp=5.0, Ki=1.5, Kd=0.5, setpoint=60.0)

# Run with changing setpoint
sim = PIDSimulation(pid, pump, duration=20.0)

# Step change: 60 bar at t=0, then 80 bar at t=10s
def changing_setpoint(t):
    return 60.0 if t < 10.0 else 80.0

results = sim.run(setpoint_profile=changing_setpoint)
sim.plot_results("Pump Pressure - Setpoint Change")

import matplotlib.pyplot as plt
plt.show()
```

## 📊 Understanding the Plots

When you run a simulation, you'll see these plots:

1. **System Response**: Shows setpoint (red dashed) vs actual value (blue)
   - Good control → blue line follows red line closely

2. **Tracking Error**: Difference between setpoint and actual value
   - Good control → error approaches zero quickly

3. **Controller Output**: The control signal sent to the system
   - Watch for saturation (hitting limits)

4. **PID Components**: Individual P, I, D contributions
   - See which term is doing what!

5. **Performance Metrics**:
   - **Rise Time**: How fast the system responds
   - **Overshoot**: How much it overshoots the target
   - **Settling Time**: How long until it stays at target
   - **Steady-State Error**: Final error that remains

## 🎓 PID Tuning Tips

### Manual Tuning (Ziegler-Nichols Method)

1. **Start with P only** (Ki=0, Kd=0)
   - Increase Kp until you get sustained oscillation
   - Note the critical gain (Kp_crit) and oscillation period (T_crit)

2. **Calculate starting values**:
   - Kp = 0.6 × Kp_crit
   - Ki = 2 × Kp / T_crit
   - Kd = Kp × T_crit / 8

3. **Fine-tune from there**:
   - Increase Kp for faster response
   - Increase Ki to eliminate steady-state error
   - Increase Kd to reduce overshoot

### Quick Tuning Rules

- **If system is too slow**: Increase Kp
- **If system oscillates**: Decrease Kp, increase Kd
- **If system doesn't reach setpoint**: Increase Ki
- **If system overshoots a lot**: Increase Kd
- **If response is too aggressive**: Decrease Kp

## 🔬 Advanced Features

### Custom Plant Models

Create your own plant by extending `PlantBase`:

```python
from plant_models import PlantBase

class MyCustomPlant(PlantBase):
    def __init__(self, custom_param=1.0, initial_value=0.0):
        super().__init__(initial_value)
        self.param = custom_param

    def update(self, control_input):
        # Your custom dynamics here
        self.value += control_input * self.param * self.sample_time
        self.history.append(self.value)
        return self.value
```

### Adding Noise and Disturbances

```python
# Add measurement noise
results = sim.run(noise_level=1.0)  # Standard deviation of noise

# Add disturbance
def disturbance(t):
    return 10.0 if 5.0 < t < 6.0 else 0.0  # Pulse disturbance

results = sim.run(disturbance_profile=disturbance)
```

## 🔗 Connection to Your Hardware Controller

This simulation mirrors the concepts in your STM32 PumpBoard controller:

- **PID Structure**: Same position-form PID as in `PID.h`
- **Pump Pressure Model**: Similar to your actual pressure control
- **Integral Anti-windup**: Prevents integral term from growing too large
- **Output Clamping**: Matches PWM limits in hardware

### Mapping to Hardware Parameters

Your C code has scaled gains. To convert:

```
C code:        Kp_actual = CUR_A_KP / CUR_A_KP_DIV
Python:        Kp = Kp_actual  (already in real units)

Example from your code:
CUR_A_KP = 50, CUR_A_KP_DIV = 2048
→ Kp_actual = 50/2048 = 0.0244

Use Kp=0.0244 in Python simulation
```

## 📝 Learning Path

### Beginner
1. Run `simulation.py` to see examples
2. Try `interactive_tuning.py` with "First Order System"
3. Experiment with only Kp (set Ki=0, Kd=0)
4. Gradually add Ki and Kd

### Intermediate
1. Try different plant types
2. Compare P, PI, and PID control
3. Experiment with setpoint changes
4. Add noise and disturbances

### Advanced
1. Create custom plant models
2. Implement cascaded control (two PIDs)
3. Try feed-forward control with Kv
4. Optimize for different performance metrics

## 🐛 Troubleshooting

**Plots don't show up?**
- Make sure matplotlib is installed
- Try running with `python -i simulation.py`

**System goes unstable?**
- Reduce Kp or increase Kd
- Check that plant parameters are reasonable

**No response from controller?**
- Check if Kp, Ki, Kd are too small
- Verify output_limits aren't too restrictive

## 📚 Further Reading

- [PID Controller (Wikipedia)](https://en.wikipedia.org/wiki/PID_controller)
- [PID Tuning Methods](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)
- Control Systems Engineering textbooks

## 💡 Tips for Learning

1. **Start Simple**: Begin with first-order systems and P control only
2. **One Parameter at a Time**: Change one gain at a time to see its effect
3. **Watch the Components**: See how P, I, D contribute differently
4. **Try Different Systems**: Each plant type teaches something new
5. **Experiment Freely**: This is a safe environment - you can't break anything!

## 🎉 Have Fun Learning!

PID control is both an art and a science. Use this tool to build intuition about how different parameters affect system behavior. The more you experiment, the better you'll understand!

---

**Created for learning PID control concepts**
Compatible with concepts from the PumpBoard_Controller project
