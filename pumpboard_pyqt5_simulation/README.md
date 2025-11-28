# PumpBoard PPQ1 Simulator - PyQt5

This is a Python/PyQt5 simulation of the **PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27** embedded firmware.

## Overview

This simulator replicates the cascade PID control system used in the hydraulic pump controller:

- **Outer Loop**: Pressure Control → Angle Setpoint
- **Middle Loop**: Angle Control → Current Setpoint
- **Inner Loop**: Current Control → PWM Output

## Features

✅ Cascade PID control with configurable gains
✅ Realistic hydraulic pump plant model
✅ Sensor simulation with filtering
✅ Real-time plotting of all control variables
✅ Individual loop enable/disable
✅ Adjustable setpoints during runtime

## Architecture

### Files

- `pid_controller.py` - PID controller implementations (Current, Angle, Pressure)
- `pump_plant_model.py` - Hydraulic pump physical model and sensor simulation
- `pumpboard_simulator.py` - Main PyQt5 GUI application
- `requirements.txt` - Python dependencies

### Control Structure

```
Pressure Setpoint ──> [Pressure PID] ──> Angle Setpoint
                                              ↓
                      Angle Feedback <── [Angle PID] ──> Current Setpoint
                                                               ↓
                      Current Feedback <── [Current PID] ──> PWM Output
                                                                  ↓
                                                          [Hydraulic Pump Plant]
```

## Installation

### Requirements

- Python 3.7+
- PyQt5
- NumPy
- pyqtgraph

### Install Dependencies

```bash
cd pumpboard_pyqt5_simulation
pip install -r requirements.txt
```

## Usage

### Run the Simulator

```bash
python pumpboard_simulator.py
```

### GUI Controls

**System Enable:**
- ☐ Pump Start - Enable pump operation
- ☑ Angle Loop Enable - Enable angle PID control (default on)
- ☐ Pressure Loop Enable - Enable pressure cascade control
- ☐ Power Loop Enable - Enable power limiting (not implemented)

**Setpoints:**
- Angle: 0-5000 per-unit (represents swash plate angle)
- Pressure: 0-600 bar
- Current A/B: 0-3000 mA (solenoid valve currents)

**Control Buttons:**
- **Start Simulation** - Begin real-time simulation
- **Stop Simulation** - Pause simulation
- **Reset** - Reset all controllers and clear plots

### Tabs

1. **Angle** - Swash plate angle control performance
2. **Pressure** - Hydraulic pressure control performance
3. **Current** - Solenoid current tracking
4. **PWM** - PWM duty cycle outputs

## PID Parameters

### Current Loop (Inner Loop)
- **Kp**: 50 / 2048 = 0.0244
- **Ki**: 6500 / 8192 = 0.794
- **Kd**: 0
- **Update Rate**: 100 µs (10 kHz)

### Angle Loop (Middle Loop)
- **Kp**: 150 / 400 = 0.375
- **Ki**: 0
- **Kd**: 1500 / 450 = 3.33
- **Kv**: 1000 / 350 = 2.86 (velocity feedforward)
- **Update Rate**: 1 ms (1 kHz)

### Pressure Loop (Outer Loop)
- **Kp**: 1600 / 300 = 5.33
- **Ki**: 0
- **Kd**: 515 / 100 = 5.15
- **Kv**: 515 / 100 = 5.15
- **Update Rate**: 2.5 ms (400 Hz)

## Firmware Mapping

This simulation directly replicates firmware behavior from:

- **main.c** - Main control loop structure
- **application.h/c** - System parameters and control functions
- **PID.h/c** - PID controller implementation
- **Task.h/c** - Periodic task scheduling

### Key Parameters Replicated

| Parameter | Firmware Value | Simulation |
|-----------|----------------|------------|
| PWM Frequency | 200 Hz | 200 Hz |
| Control Loop | 100 µs | 100 µs |
| Angle Range | 0-5000 pu | 0-5000 pu |
| Pressure Range | 0-600 bar | 0-600 bar |
| Current Range | 0-3000 mA | 0-3000 mA |
| Max Displacement | 28 cc/rev | 28 cc/rev |

## Testing

### Test Scenario 1: Angle Step Response

1. Enable "Angle Loop Enable" only
2. Set Angle setpoint to 2500 pu
3. Start simulation
4. Observe angle tracking and PWM response

### Test Scenario 2: Pressure Cascade Control

1. Enable both "Angle Loop Enable" and "Pressure Loop Enable"
2. Set Pressure setpoint to 300 bar
3. Start simulation
4. Observe cascade control: Pressure → Angle → Current → PWM

### Test Scenario 3: Disturbance Rejection

1. Start with stable angle control
2. Change angle setpoint during runtime
3. Observe transient response and settling time

## Troubleshooting

### Simulation is unstable
- Check PID gains are correct
- Reduce plant noise in `pump_plant_model.py`
- Increase loop update rates

### No response when starting
- Verify "Angle Loop Enable" is checked
- Check setpoints are non-zero
- Ensure Python dependencies are installed

## License

This simulation is for educational and testing purposes, replicating the embedded firmware functionality.

## References

- Firmware: `PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/`
- Target MCU: STM32F407ZE
- Development Environment: Keil µVision
