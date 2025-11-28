# Cascade PID Tuning Tool

## Overview

The **interactive_tuning_cascade.py** tool provides a three-tab interface for tuning the cascade control structure used in the PumpBoard controller. This tool simulates the exact control hierarchy from the real embedded system:

```
┌─────────────────────────────────────────────────────────┐
│  PRESSURE LOOP (Outer, 2.5ms)                           │
│    ↓ output → angle setpoint                            │
│  ┌───────────────────────────────────────────────────┐  │
│  │  ANGLE LOOP (Middle, 1ms)                         │  │
│  │    ↓ output → current setpoint                    │  │
│  │  ┌─────────────────────────────────────────────┐  │  │
│  │  │  CURRENT LOOP (Inner, 100us)                │  │  │
│  │  │    ↓ output → PWM                           │  │  │
│  │  │  SOLENOID VALVE → ANGLE → PRESSURE          │  │  │
│  │  └─────────────────────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

## Features

### Three Interactive Tabs

1. **PRESSURE TAB** - Tune the outer loop (slowest, 2.5ms period)
   - Controls hydraulic pressure
   - Output sets angle setpoint
   - Default PID: Kp=5.333, Ki=0.0, Kd=5.15

2. **ANGLE TAB** - Tune the middle loop (medium, 1ms period)
   - Controls pump swash plate angle
   - Output sets current setpoint
   - Default PID: Kp=0.375, Ki=0.0, Kd=3.333

3. **CURRENT TAB** - Tune the inner loop (fastest, 100us period)
   - Controls solenoid valve current
   - Output drives PWM
   - Default PID: Kp=0.0244, Ki=0.7935, Kd=0.0

### Real-Time Visualization

- **Individual Loop Plots**: See each control loop's response separately
- **Combined View**: Watch how all three loops interact together
- **Cascade Mode Toggle**: Switch between cascade and independent modes
- **Performance Metrics**: Monitor system behavior in real-time

## Installation

```bash
cd pid_simulation_test
pip install -r requirements.txt
```

## Usage

### Basic Usage

```bash
python interactive_tuning_cascade.py
```

### Interface Controls

1. **Tab Buttons**: Click PRESSURE, ANGLE, or CURRENT to switch tabs
2. **Sliders**: Adjust PID parameters (Kp, Ki, Kd) and setpoint for active tab
3. **CASCADE Toggle**: Enable/disable cascade control mode
4. **Reset Button**: Return all parameters to default values

### Understanding the Display

#### Top Row - Individual Loops
- Each plot shows one control loop's setpoint (red dashed) and actual value (colored solid)
- Watch how adjusting parameters affects each loop's response

#### Bottom Left - Combined View
- Shows all three loops on the same plot
- See the cascade effect when pressure changes propagate through angle to current

#### Bottom Right - Information Panel
- Displays current PID parameters
- Shows control loop periods
- Provides tuning tips

## Tuning Guide

### Step 1: Tune Current Loop (Inner Loop First)

1. Switch to **CURRENT** tab
2. Start with default values (Kp=0.0244, Ki=0.7935, Kd=0.0)
3. Increase Kp for faster response (watch for oscillations)
4. Adjust Ki to eliminate steady-state error
5. Current loop should respond within ~5-10ms

**Goal**: Fast, stable current tracking with minimal overshoot

### Step 2: Tune Angle Loop (Middle Loop)

1. Switch to **ANGLE** tab
2. Start with default values (Kp=0.375, Ki=0.0, Kd=3.333)
3. Increase Kp until you see good tracking
4. Increase Kd to reduce overshoot
5. Angle loop should respond within ~50-100ms

**Goal**: Smooth angle control without oscillations

### Step 3: Tune Pressure Loop (Outer Loop)

1. Switch to **PRESSURE** tab
2. Start with default values (Kp=5.333, Ki=0.0, Kd=5.15)
3. Adjust Kp for desired speed vs stability trade-off
4. Increase Kd to dampen oscillations
5. Pressure loop should reach setpoint within ~500ms-1s

**Goal**: Stable pressure control with acceptable settling time

### Tuning Rules of Thumb

1. **Loop Speed Hierarchy**: Inner loops should be 3-10x faster than outer loops
   - Current (100us) is 10x faster than Angle (1ms)
   - Angle (1ms) is 2.5x faster than Pressure (2.5ms)

2. **Stability**: Tune inner loops first, outer loops last
   - Unstable inner loops make outer loop tuning impossible
   - Well-tuned inner loops simplify outer loop tuning

3. **Cascade Benefits**:
   - Better disturbance rejection
   - Faster overall response
   - More robust to parameter variations

## Control Loop Details

### Current Loop (Innermost - 100us)

**Plant Model**: First-order electrical system (solenoid coil)
- Time constant: 2ms (L/R)
- Models inductance and resistance
- Output: Valve current (drives swash plate actuator)

**From Real Controller** (`PID.h` lines 34-43):
```c
#define Cur_B_Kp          50
#define Cur_B_Kp_Div      2048
#define Cur_B_Ki          6500
#define Cur_B_Ki_Div      8192
```

### Angle Loop (Middle - 1ms)

**Plant Model**: Second-order mechanical system (swash plate)
- Inertia, damping, and spring forces
- Models physical pump mechanics
- Output: Swash plate angle (controls pump displacement)

**From Real Controller** (`PID.h` lines 46-58):
```c
#define Ang_Kp            150
#define Ang_Kp_Div        400
#define Ang_Kd            1500
#define Ang_Kd_Div        450
```

### Pressure Loop (Outermost - 2.5ms)

**Plant Model**: Hydraulic system with leakage
- Pressure generation from pump
- Leakage proportional to pressure
- Output: Hydraulic pressure

**From Real Controller** (`PID.h` lines 63-78):
```c
#define Prs_Kp            1600
#define Prs_Kp_Div        300
#define Prs_Kd            515
#define Prs_Kd_Div        100
```

## Cascade Mode vs Independent Mode

### CASCADE Mode (Default)
- Pressure output → Angle setpoint
- Angle output → Current setpoint
- **Use this mode** for realistic system behavior

### INDEPENDENT Mode
- Each loop tracks its own independent setpoint
- Useful for testing individual loop performance
- Not representative of real system operation

## Tips for Learning

1. **Start with Cascade ON**: See how loops interact naturally
2. **Change Pressure Setpoint**: Watch the cascade effect flow through all loops
3. **Compare Loop Speeds**: Notice how current responds fastest, pressure slowest
4. **Experiment with Ki**: Most loops use Ki=0 (PD control) - see why!
5. **Watch Combined Plot**: Best view of overall system dynamics

## Common Observations

- **No Integral Term**: Real controller uses Ki=0 for angle and pressure loops
  - Prevents windup in cascade systems
  - Outer loop handles steady-state error

- **High Kd Values**: Derivative gain is high relative to Kp
  - Provides damping for mechanical systems
  - Reduces overshoot and oscillations

- **Fast Inner Loop**: Current loop has high Ki (0.7935)
  - Eliminates electrical steady-state error quickly
  - Innermost loop can use integral without cascade issues

## Troubleshooting

### Oscillations
- Reduce Kp (proportional gain)
- Increase Kd (derivative gain)
- Check that inner loops are stable first

### Slow Response
- Increase Kp (but watch for instability)
- Verify cascade mode is enabled
- Check that setpoint isn't too aggressive

### Steady-State Error
- Increase Ki (integral gain)
- Note: Real controller uses Ki=0 for outer loops
- Cascade structure naturally eliminates some steady-state error

## Related Files

- **interactive_tuning.py**: Single-loop PID tuning tool
- **simulation.py**: Pre-configured simulation examples
- **pid_controller.py**: Core PID implementation
- **plant_models.py**: System dynamics models

## References

- Actual controller code: `Function/application.c` lines 631-664
- PID constants: `Function/PID.h`
- Control structure documentation: `SIMULATOR_LEARNING_GUIDE.md`

## Next Steps

After tuning in simulation:
1. Compare your parameters to real controller defaults
2. Test on hardware (if available)
3. Fine-tune based on actual system response
4. Document your tuning results

---

**Note**: This simulation uses simplified plant models. Real hardware may have additional dynamics (friction, dead zones, saturation, etc.) that require further tuning adjustments.
