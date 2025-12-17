# STM32F407 PumpBoard Controller - PyQt5 GUI

A comprehensive Python-based graphical user interface for monitoring and controlling the STM32F407ZE hydraulic pump controller via USB serial connection.

## Features

### Real-time Monitoring
- **Live parameter display**: Angle, pressure, current (A & B), PWM outputs
- **Real-time plotting**: Continuous data visualization with automatic scrolling
- **Update rate**: Configurable (default 100ms)

### Control Interface
- **Enable/Disable Controls**: Pump start, pressure loop, angle leakage compensation, power loop
- **Setpoint Adjustment**: Angle reference, pressure reference, current references, torque limit
- **Safety Features**: Emergency stop button, zero all setpoints

### PID Tuning
- **Angle Loop**: Kp, Ki, Kd, Kv parameters
- **Pressure Loop**: Kp, Ki, Kd, Kv parameters
- **Current Loop A**: Kp, Ki, Kd parameters
- **Individual parameter read/write**

### Communication Protocol
- **Frame-based protocol** matching STM32 firmware (ParseFrame.c/h)
- **CRC16 error checking** for data integrity
- **Automatic frame parsing** and validation
- **Command acknowledgment** and timeout handling

## System Requirements

### Software
- Python 3.7 or higher
- PyQt5 5.15.0+
- pyqtgraph 0.12.0+
- pyserial 3.5+
- numpy 1.20.0+

### Hardware
- STM32F407ZE PumpBoard Controller
- USB Type-B cable
- Windows/Linux/macOS PC

## Installation

### 1. Clone or Download the GUI Package

```bash
cd STM32_PumpBoard_GUI/
```

### 2. Install Python Dependencies

#### Using pip (Recommended)
```bash
pip install -r requirements.txt
```

#### Manual Installation
```bash
pip install PyQt5>=5.15.0
pip install pyqtgraph>=0.12.0
pip install pyserial>=3.5
pip install numpy>=1.20.0
```

### 3. Verify Installation

```bash
python -c "import PyQt5; import pyqtgraph; import serial; print('All dependencies installed!')"
```

## Usage

### 1. Connect Hardware

1. Connect the STM32F407ZE controller to your PC using a USB Type-B cable
2. Wait for the virtual COM port driver to install (Windows may require STM32 VCP driver)
3. Note the COM port number (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)

### 2. Launch the GUI

```bash
python gui_main.py
```

### 3. Connect to Controller

1. **Select Serial Port**: Choose the correct COM port from the dropdown
2. **Set Baudrate**: Default is 115200 (must match STM32 firmware)
3. **Click "Connect"**: Button turns green when connected

### 4. Monitor Parameters

Navigate to the **Dashboard** tab to view:
- Real-time parameter values (updated every 100ms)
- Live plots of angle, pressure, and current feedback
- PWM output duty cycles

### 5. Control the System

Navigate to the **Control** tab to:

#### Enable/Disable Loops
- Check/uncheck boxes to enable/disable:
  - Pump Start
  - Pressure Loop
  - Angle Leakage Compensation
  - Power Loop

#### Set Reference Values
- Adjust sliders or enter values for:
  - Angle Reference (0-100%)
  - Pressure Reference (0-600 bar)
  - Current A/B References (0-3000 mA)
  - Torque Limit (0-250 Nm)
- Click **Apply** to send to controller

#### Safety Controls
- **EMERGENCY STOP**: Disables all loops and zeros all setpoints
- **Zero All Setpoints**: Resets all references to zero
- **Read All Parameters**: Reads current values from controller

### 6. Tune PID Parameters

Navigate to the **PID Tuning** tab to:

1. **Read current values**: Click "Read" button for each parameter
2. **Adjust values**: Enter new PID gains
3. **Write to controller**: Click "Write" to update
4. **Test performance**: Monitor response in Dashboard tab

**Warning**: Improper PID tuning can cause system instability. Make small changes and test carefully.

## Protocol Details

### Frame Structure

```
[0xAA] [CMD] [INDEX] [LEN] [PAYLOAD...] [CRC16_L] [CRC16_H] [0x55]
```

| Field | Size | Description |
|-------|------|-------------|
| Header | 1 byte | 0xAA - Start of frame |
| Command | 1 byte | 0x01=Read, 0x02=Write, 0x03=Wave |
| Index | 1 byte | Parameter index (0-255) |
| Length | 1 byte | Payload length in bytes |
| Payload | 0-N bytes | Data (typically 4 bytes for int32) |
| CRC16 Low | 1 byte | CRC lower byte |
| CRC16 High | 1 byte | CRC upper byte |
| Tail | 1 byte | 0x55 - End of frame |

### CRC16 Calculation

- **Algorithm**: CRC16 with lookup table
- **Polynomial**: 0x8005
- **Initial Value**: 0xFFFF
- **CRC Data**: CMD + INDEX + LEN + PAYLOAD

### Command Types

#### CMD_READ (0x01)
Request to read a parameter value from the controller.

**Request Frame**:
```
[0xAA] [0x01] [INDEX] [0x00] [CRC_L] [CRC_H] [0x55]
```

**Response Frame**:
```
[0xAA] [0x01] [INDEX] [0x04] [VALUE_0] [VALUE_1] [VALUE_2] [VALUE_3] [CRC_L] [CRC_H] [0x55]
```

#### CMD_WRITE (0x02)
Write a parameter value to the controller.

**Request Frame**:
```
[0xAA] [0x02] [INDEX] [0x04] [VALUE_0] [VALUE_1] [VALUE_2] [VALUE_3] [CRC_L] [CRC_H] [0x55]
```

**Response Frame**:
```
[0xAA] [0x02] [INDEX] [0x00] [CRC_L] [CRC_H] [0x55]
```

## Parameter Mapping

Parameters are accessed by index matching the `g_HLCmdMap[]` array in the STM32 firmware.

### Feedback Parameters (Read-Only)

| Index | Parameter | Unit | Range |
|-------|-----------|------|-------|
| 0x08 | Angle Feedback | % | 0-100 |
| 0x09 | Pressure Feedback | bar | 0-600 |
| 0x0A | Current A Feedback | mA | 0-3000 |
| 0x0B | Current B Feedback | mA | 0-3000 |
| 0x06 | PWM A Output | % | 0-100 |
| 0x07 | PWM B Output | % | 0-100 |

### Enable Flags (Read/Write)

| Index | Parameter |
|-------|-----------|
| 0x20 | Enable Pump Start |
| 0x21 | Enable Angle Leakage |
| 0x22 | Enable Pressure Loop |
| 0x23 | Enable Power Loop |

### Control Setpoints (Read/Write)

| Index | Parameter | Unit | Range |
|-------|-----------|------|-------|
| 0x30 | Tilt Angle Reference | Per-unit | 0-10000 |
| 0x31 | Pressure Reference | Per-unit | 0-10000 |
| 0x32 | Current A Reference | mA | 0-3000 |
| 0x33 | Current B Reference | mA | 0-3000 |
| 0x34 | Torque Limit | Nm | 0-250 |

### PID Parameters (Read/Write)

#### Angle Loop (0x50-0x5F)
- 0x50: Kp
- 0x51: Ki
- 0x52: Kd
- 0x53: Kv (feedforward)

#### Pressure Loop (0x60-0x6F)
- 0x60: Kp
- 0x61: Ki
- 0x62: Kd
- 0x63: Kv

#### Current Loop A (0x40-0x4F)
- 0x40: Kp
- 0x41: Ki
- 0x42: Kd

## File Structure

```
STM32_PumpBoard_GUI/
├── gui_main.py          # Main application window and UI
├── serial_comm.py       # Serial protocol handler
├── parameter_defs.py    # Parameter definitions and mappings
├── requirements.txt     # Python dependencies
└── README.md           # This file
```

## Troubleshooting

### Connection Issues

**Problem**: "Connection failed" error

**Solutions**:
1. Check USB cable connection
2. Verify correct COM port selected
3. Ensure no other program is using the COM port
4. On Linux, check permissions: `sudo chmod 666 /dev/ttyUSB0`
5. Install STM32 VCP driver (Windows)

### Communication Errors

**Problem**: "CRC error" messages

**Solutions**:
1. Check baudrate matches firmware (default: 115200)
2. Verify cable quality (try different cable)
3. Reduce baud rate to 57600 or 38400
4. Check for electrical noise/interference

**Problem**: "Timeout reading/writing" errors

**Solutions**:
1. Verify STM32 firmware is running correctly
2. Check that UART is initialized in firmware
3. Increase timeout value in `serial_comm.py` (default: 1.0s)
4. Ensure firmware is responding to commands

### Data Display Issues

**Problem**: Values not updating

**Solutions**:
1. Check connection status (button should be green)
2. Verify monitoring timer is running (Dashboard tab)
3. Check that firmware is sending data
4. Try "Read All Parameters" button

**Problem**: Plots not displaying

**Solutions**:
1. Ensure pyqtgraph is installed correctly
2. Check that data is being received (view status bar)
3. Restart application

### Control Issues

**Problem**: Setpoints not applying

**Solutions**:
1. Verify write commands succeed (check status bar)
2. Ensure parameter is not read-only
3. Check value is within valid range
4. Verify enable flags are set correctly

## Safety Considerations

### IMPORTANT WARNINGS

1. **Emergency Stop**: Always have the emergency stop button visible and accessible
2. **Safe Startup**: Start with all loops disabled and zero setpoints
3. **Gradual Changes**: Make small adjustments to setpoints and monitor response
4. **Pressure Limits**: Never exceed the maximum rated pressure (600 bar)
5. **Current Limits**: Monitor solenoid currents to prevent overheating (max 3000 mA)
6. **PID Tuning**: Improper tuning can cause instability or damage
7. **Calibration**: Do not modify calibration values unless you know what you're doing

## Advanced Features

### Custom Parameters

To add custom parameters:

1. **Add to `parameter_defs.py`**:
```python
'my_parameter': {
    'index': 0xC0,  # Choose unused index
    'name': 'My Parameter',
    'unit': 'units',
    'scale': 1.0,
    'min': 0,
    'max': 1000,
    'readonly': False,
    'category': 'Custom'
}
```

2. **Add UI element in `gui_main.py`** (as needed)

### Data Logging

To add data logging:

```python
# In update_monitoring() method
with open('data_log.csv', 'a') as f:
    f.write(f"{time.time()},{angle},{pressure},{current_a},{current_b}\n")
```

### Custom Plots

To add additional plots:

```python
# In create_dashboard_tab() method
plot = self.plot_widget.addPlot(title='My Plot')
curve = plot.plot(pen='y')
self.plots['my_plot'] = {'plot': plot, 'curve': curve}
```

## Development

### Testing Without Hardware

For development/testing without hardware:

1. Use a serial port loopback adapter (TX->RX)
2. Implement a simulation mode in firmware
3. Use virtual serial port software (e.g., com0com on Windows)

### Debugging

Enable debug output:

```python
# Add to SerialProtocol class
import logging
logging.basicConfig(level=logging.DEBUG)
self.logger = logging.getLogger(__name__)

# Add debug prints in _receive_thread()
self.logger.debug(f"Received frame: {frame.hex()}")
```

## Support and Contribution

### Issues

Report issues including:
- Python version
- OS and version
- Error messages and stack traces
- Steps to reproduce

### Future Enhancements

Planned features:
- Data logging to CSV/HDF5
- Configuration profiles (save/load settings)
- Waveform recording and playback
- Advanced diagnostics and error reporting
- CAN bus support
- Multi-controller support

## License

This GUI application is provided as-is for use with the STM32F407 PumpBoard Controller.

## References

- STM32F407 Reference Manual: [RM0090](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- PyQt5 Documentation: [https://www.riverbankcomputing.com/static/Docs/PyQt5/](https://www.riverbankcomputing.com/static/Docs/PyQt5/)
- pyqtgraph Documentation: [https://pyqtgraph.readthedocs.io/](https://pyqtgraph.readthedocs.io/)
- pyserial Documentation: [https://pyserial.readthedocs.io/](https://pyserial.readthedocs.io/)

## Contact

For technical support or questions about the GUI application, please refer to the main project repository.

---

**Version**: 1.0.0
**Date**: 2025-12-17
**Author**: Auto-generated for STM32F407 PumpBoard Controller
