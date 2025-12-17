# PumpBoard Controller GUI

A Qt-based C++ GUI application for monitoring and controlling the STM32F407xx PumpBoard hydraulic pump controller via USB connection.

## Screenshots

The GUI provides:
- **Dashboard**: Real-time gauges showing angle, pressure, current, and PWM outputs
- **Control Panel**: Sliders and spinboxes to set references and enable control loops
- **PID Tuning**: Parameter adjustment for angle and pressure PID controllers
- **Charts**: Real-time data plotting

## System Requirements

### PC (GUI Application)
- Windows 10/11, Linux (Ubuntu 20.04+), or macOS 10.15+
- Qt 5.15+ or Qt 6.2+ with:
  - Qt Core
  - Qt GUI
  - Qt Widgets
  - Qt SerialPort
  - Qt Charts
- CMake 3.16+
- C++17 compatible compiler

### Hardware
- STM32F407xx PumpBoard Controller
- USB Type-B cable
- USB-to-UART adapter (if not using native USB)

## Installation

### Ubuntu/Debian Linux
```bash
# Install Qt5 and dependencies
sudo apt-get update
sudo apt-get install -y \
    qtbase5-dev \
    libqt5serialport5-dev \
    libqt5charts5-dev \
    qttools5-dev \
    cmake \
    build-essential

# OR for Qt6:
sudo apt-get install -y \
    qt6-base-dev \
    qt6-serialport-dev \
    qt6-charts-dev \
    cmake \
    build-essential
```

### Windows
1. Download Qt Online Installer from https://www.qt.io/download-qt-installer
2. Install Qt 6.x (or 5.15) with these components:
   - MSVC 2019/2022 64-bit (or MinGW)
   - Qt SerialPort
   - Qt Charts
3. Install CMake from https://cmake.org/download/
4. Install Visual Studio 2019/2022 (for MSVC) or MinGW

### macOS
```bash
# Using Homebrew
brew install qt@6 cmake

# Add Qt to PATH
echo 'export PATH="/opt/homebrew/opt/qt@6/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
```

## Building the Application

### Linux/macOS
```bash
cd PumpBoard_GUI
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run the application
./PumpBoardGUI
```

### Windows (Command Prompt)
```batch
cd PumpBoard_GUI
mkdir build && cd build

# For MSVC:
cmake -G "Visual Studio 17 2022" -A x64 ..
cmake --build . --config Release

# For MinGW:
cmake -G "MinGW Makefiles" ..
mingw32-make

# Run:
Release\PumpBoardGUI.exe
```

### Windows (Qt Creator)
1. Open Qt Creator
2. File → Open File or Project
3. Select `PumpBoard_GUI/CMakeLists.txt`
4. Configure the project with your Qt kit
5. Click Build → Run

## Firmware Integration

### Adding USB Protocol to Your Keil Project

1. **Copy files** to your project:
   - `Function/usb_protocol.h`
   - `Function/usb_protocol.c`

2. **Add to Keil project**:
   - Right-click on Function group → Add Existing Files
   - Select both files

3. **Add include path**:
   - Options for Target → C/C++ → Include Paths
   - Add path to Function folder

4. **Initialize in main.c**:
```c
#include "usb_protocol.h"

// Add system tick variable (if not present)
volatile uint32_t g_system_tick_ms = 0;

// In SysTick_Handler or TIM interrupt:
void SysTick_Handler(void) {
    g_system_tick_ms++;
}

int main(void) {
    // ... existing initialization ...

    // Initialize USB protocol (115200 baud on USART1: PA9/PA10)
    USB_Protocol_Init();
    USB_UART_Init(115200);

    while(1) {
        // ... existing main loop ...
    }
}

// In your 10ms or 100ms periodic task:
void Task_10ms(void) {
    USB_Protocol_Task();  // Handle streaming and timeouts
}
```

5. **Hardware connection**:
   - USART1 TX: PA9 → USB-UART RX
   - USART1 RX: PA10 → USB-UART TX
   - GND → GND

### USB-to-UART Adapters
Recommended adapters:
- FTDI FT232RL
- CH340G
- CP2102
- PL2303

## Using the GUI

### Connecting to Controller

1. Connect USB cable from PC to PumpBoard (via USB-UART adapter)
2. Launch `PumpBoardGUI`
3. Select COM port from dropdown (click "Refresh" if not visible)
4. Set baud rate to 115200
5. Click "Connect"

### Dashboard Tab
- **Angle Gauge**: Shows swash plate angle (0-100%)
- **Pressure Gauge**: Shows hydraulic pressure (0-600 bar)
- **Current Bars**: Solenoid A/B current (0-3000 mA)
- **PWM Output**: Duty cycle percentage
- **Status Indicators**: Control loop enables, errors

### Control Panel Tab
- **Angle Reference**: Drag slider or enter value (0-10000 = 0-100%)
- **Pressure Reference**: Drag slider or enter value (0-10000)
- **Current References**: Set solenoid A/B current (0-3000 mA)
- **Enable Flags**: Toggle control loops
- Click **"Apply All"** to send changes to controller

### PID Tuning Tab
- **Angle PID**: Kp, Ki, Kd, Kv gains with divisors
- **Pressure PID**: Same structure
- **Calibration**: ADC min/mid/max values for sensors
- **Filters**: Low-pass filter time constants

### Charts Tab
- Real-time plotting of angle, pressure, current
- Toggle channels with checkboxes
- Adjust time window (5-300 seconds)
- Pause/Resume and Clear controls

### Data Logging
1. Click "Start Log" in toolbar
2. Choose filename (CSV format)
3. Data is recorded continuously
4. Click "Stop Log" to finish
5. CSV can be opened in Excel/LibreOffice

## Communication Protocol

### Frame Format
```
[0xAA][CMD][LEN][PAYLOAD...][CRC16_L][CRC16_H][0x55]
```

### Key Commands
| Command | Code | Description |
|---------|------|-------------|
| Heartbeat | 0x01 | Keep connection alive |
| Get Status | 0x04 | Read firmware version and status |
| Get All Feedback | 0x10 | Read all sensor values |
| Set Angle Ref | 0x30 | Set angle reference (0-10000) |
| Set Pressure Ref | 0x31 | Set pressure reference (0-10000) |
| Set Enable Flags | 0x33 | Enable/disable control loops |
| Start Stream | 0x90 | Begin continuous data streaming |
| Stop Stream | 0x91 | Stop streaming |

## Troubleshooting

### GUI won't start
- Verify Qt is properly installed
- Check Qt plugins are in PATH
- On Windows, copy Qt DLLs to executable folder

### Can't find COM port
- Check USB cable connection
- Install USB-UART driver (FTDI/CH340/CP210x)
- On Linux, add user to `dialout` group:
  ```bash
  sudo usermod -a -G dialout $USER
  # Log out and back in
  ```

### No data received
- Verify baud rate matches (115200)
- Check TX/RX wiring (may need to swap)
- Verify firmware has USB protocol integrated
- Check USART1 GPIO configuration

### CRC errors
- Check for electrical noise
- Try shorter USB cable
- Verify both ends use same CRC algorithm

## Testing Without Hardware

The firmware includes a simulation mode. To test:

1. In firmware, set simulation flag:
```c
extern u8 g_u8_SimulationMode;
g_u8_SimulationMode = 1;  // Enable simulation
```

2. Simulated values:
   - Angle feedback: 2000 (20%)
   - Current feedback: 800 mA
   - All control loops functional

## License

This project is provided for PumpBoard controller development.

## Support

For issues, check:
1. Hardware connections
2. COM port permissions
3. Firmware integration
4. Qt installation
