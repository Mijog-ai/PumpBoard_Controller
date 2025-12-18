# PumpBoard Controller Deployment Guide

## Overview
This guide provides step-by-step instructions for deploying firmware to the PumpBoard Controller hardware.

**Target Hardware:** STM32F407ZE Microcontroller (Cortex-M4 with FPU)
**Flash Memory:** 1024KB (0x08000000 - 0x08100000)
**Project:** PumpBoard_Car_PPQ1_Single V0.0.0.2

---

## Table of Contents
1. [Hardware Requirements](#hardware-requirements)
2. [Software Requirements](#software-requirements)
3. [Build the Firmware](#build-the-firmware)
4. [Deploy to Controller - Method 1: Using J-Link](#method-1-using-j-link-recommended)
5. [Deploy to Controller - Method 2: Using ST-Link](#method-2-using-st-link)
6. [Deploy to Controller - Method 3: Using Command Line](#method-3-using-command-line-tools)
7. [Verification and Testing](#verification-and-testing)
8. [Troubleshooting](#troubleshooting)

---

## Hardware Requirements

### Required Items:
1. **PumpBoard Controller Body** - Target hardware with STM32F407ZE
2. **Programmer/Debugger** - One of the following:
   - J-Link debugger (JTAG/SWD) - RECOMMENDED
   - ST-Link V2 or V3 (SWD)
   - Other JTAG/SWD compatible programmers
3. **Connection Cable** - 10-pin or 20-pin JTAG/SWD cable
4. **Power Supply** - Appropriate power supply for the controller board
5. **USB Cable** - For connecting debugger to PC

### Connection Points on Controller:
- Locate the **SWD/JTAG programming header** on the controller PCB
- Common pinout:
  - SWDIO (Data)
  - SWCLK (Clock)
  - GND (Ground)
  - VCC/VDD (3.3V - may be optional)
  - NRST (Reset - optional)

---

## Software Requirements

### Install the following software:

1. **Keil MDK-ARM (uVision)** - Version 5.x or later
   - Download from: https://www.keil.com/download/product/
   - Install ARM Compiler 6.24 or compatible
   - Install STM32F4xx Device Family Pack (DFP 3.1.1 or later)

2. **Programmer Software** - Choose based on your hardware:

   **Option A: J-Link Software Pack** (if using J-Link)
   - Download from: https://www.segger.com/downloads/jlink/
   - Includes J-Link drivers and J-Flash utility

   **Option B: STM32 ST-Link Utility** (if using ST-Link)
   - Download from: https://www.st.com/en/development-tools/stsw-link004.html
   - Or use STM32CubeProgrammer (newer alternative)

3. **USB Drivers** - For your specific debugger

---

## Build the Firmware

### Step 1: Open the Project

1. Navigate to the project directory:
   ```
   PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/
   ```

2. Double-click `PumpCtrl.uvprojx` to open in Keil uVision

### Step 2: Configure Build Settings

1. In Keil uVision menu: **Project → Options for Target 'PPQ_Ctrl'**
2. Verify settings:
   - **Device:** STM32F407ZE
   - **Target:** ARM Compiler 6
   - **Output:** Check "Create HEX File"

### Step 3: Build the Project

1. **Clean the project first:**
   - Menu: **Project → Clean Targets**
   - Or press: `Ctrl + F7`

2. **Rebuild all:**
   - Menu: **Project → Rebuild all target files**
   - Or press: `F7`

3. **Check build output:**
   - Look for "0 Error(s), 0 Warning(s)" in the Build Output window
   - Note the output files created:
     - `../OBJ/Template.hex` (Intel HEX format)
     - `PumpCtrl_V0.1.0.4.bin` (Binary format)

### Step 4: Locate Binary Files

After successful build, find the compiled files at:
```
PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/OBJ/Template.hex
PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/PumpCtrl_V0.1.0.4.bin
```

---

## Method 1: Using J-Link (Recommended)

### Hardware Setup:

1. **Power OFF the controller board**
2. **Connect J-Link to PC** via USB
3. **Connect J-Link to controller:**
   - Connect SWDIO, SWCLK, GND, and optionally NRST
   - Use the SWD header on the controller PCB
4. **Power ON the controller board**

### Option A: Flash from Keil uVision (Easiest)

1. In Keil uVision menu: **Project → Options for Target**
2. Select **Debug** tab
3. Select **J-Link/J-Trace Cortex** from the dropdown
4. Click **Settings** button:
   - Port: **SW** (for SWD interface)
   - Max Clock: **4 MHz** (can increase to 10MHz if stable)
5. Click **OK** to close all dialogs

6. **Flash and Run:**
   - Menu: **Flash → Download**
   - Or press: `F8`
   - Wait for "Application running..." message

7. **Reset and Run:**
   - Menu: **Debug → Start/Stop Debug Session** (`Ctrl + F5`)
   - Then **Debug → Run** (`F5`)

### Option B: Using J-Flash Standalone

1. **Launch J-Flash** application

2. **Create new project:**
   - File → New Project
   - Device: Select **STM32F407ZE**
   - Interface: **SWD**
   - Speed: **4000 kHz**

3. **Open firmware file:**
   - File → Open data file
   - Browse to: `OBJ/Template.hex` or `USER/PumpCtrl_V0.1.0.4.bin`
   - For .bin file, set start address: `0x08000000`

4. **Connect to target:**
   - Target → Connect
   - Should show "Connected successfully"

5. **Program the flash:**
   - Target → Production Programming (`F7`)
   - Or: Target → Program & Verify (`F6`)
   - Wait for "Programming completed successfully" message

6. **Reset controller:**
   - Target → Reset
   - Or power cycle the board

---

## Method 2: Using ST-Link

### Hardware Setup:

1. **Power OFF the controller board**
2. **Connect ST-Link to PC** via USB
3. **Connect ST-Link to controller:**
   - SWDIO → SWDIO
   - SWCLK → SWCLK
   - GND → GND
   - 3.3V → VDD (if needed)
4. **Power ON the controller board**

### Option A: Using STM32 ST-Link Utility

1. **Launch STM32 ST-Link Utility**

2. **Connect to target:**
   - Menu: **Target → Connect**
   - Or click **Connect** button
   - Should show device information (STM32F407ZE)

3. **Load firmware file:**
   - Menu: **Target → Program & Verify**
   - Click **Browse**, select: `OBJ/Template.hex`
   - Start Address: `0x08000000` (should auto-fill)
   - Check options:
     - ☑ Verify programming
     - ☑ Reset after programming
     - ☐ Run after programming (optional)

4. **Flash the firmware:**
   - Click **Start** button
   - Wait for "Verification...OK" message

5. **Disconnect and reset:**
   - Menu: **Target → Disconnect**
   - Power cycle the board or press reset button

### Option B: Using STM32CubeProgrammer

1. **Launch STM32CubeProgrammer**

2. **Connect to board:**
   - Select **ST-LINK** interface on the right panel
   - Port: **SWD**
   - Frequency: **4000 kHz**
   - Mode: **Normal**
   - Click **Connect** button

3. **Erase flash (optional but recommended):**
   - Go to **Erasing & Programming** section
   - Click **Full chip erase**
   - Wait for completion

4. **Download firmware:**
   - Browse and select: `OBJ/Template.hex` or `USER/PumpCtrl_V0.1.0.4.bin`
   - For .bin file, Start address: `0x08000000`
   - Check **Verify programming after download**
   - Check **Run after programming** (optional)
   - Click **Start Programming**

5. **Disconnect:**
   - Click **Disconnect** button
   - Power cycle the board

---

## Method 3: Using Command Line Tools

### Using J-Link Commander (JLinkExe)

1. **Create a J-Link script file** `flash_script.jlink`:
   ```
   device STM32F407ZE
   si SWD
   speed 4000
   connect
   r
   h
   loadfile OBJ/Template.hex
   r
   g
   exit
   ```

2. **Run the flashing command:**
   ```bash
   JLinkExe -commanderscript flash_script.jlink
   ```

### Using OpenOCD (Advanced)

1. **Install OpenOCD** (if not already installed)

2. **Create configuration file** `stm32f407.cfg`:
   ```
   source [find interface/stlink.cfg]
   source [find target/stm32f4x.cfg]

   init
   reset init
   flash write_image erase OBJ/Template.hex
   reset run
   shutdown
   ```

3. **Run flashing command:**
   ```bash
   openocd -f stm32f407.cfg
   ```

### Using STM32_Programmer_CLI

```bash
STM32_Programmer_CLI -c port=SWD freq=4000 -w OBJ/Template.hex -v -rst
```

---

## Verification and Testing

### Step 1: Check Programming Success

1. **LED Indicators:**
   - Check if status LEDs on the controller are functioning
   - Observe expected startup behavior

2. **Serial/Debug Output:**
   - If UART debug is enabled, connect serial terminal
   - Baud rate: Check application settings (typically 115200)
   - Look for startup messages

### Step 2: Functional Testing

1. **Power Supply:**
   - Verify input voltage is within specifications
   - Check current consumption is normal

2. **Sensor Initialization:**
   - AD7689 ADC should initialize
   - Internal ADC1 should initialize
   - Check SPI3 communication

3. **PWM Output:**
   - TIM2 PWM should be operating
   - Verify output signals with oscilloscope if available

4. **Watchdog:**
   - Independent watchdog (IWDG) is enabled
   - System should feed watchdog regularly

### Step 3: Test Control Loops

The firmware includes several control loops that can be tested:
- Pump start enable
- Pressure loop control
- Angle leak control
- Power loop control

**Note:** The current code has simulation mode enabled by default (line 113 in main.c). For real hardware testing, change:
```c
#if 1  // Set to 0 for real hardware
```
to:
```c
#if 0  // Set to 0 for real hardware
```

Then rebuild and re-flash the firmware.

---

## Troubleshooting

### Issue: Cannot Connect to Target

**Possible Causes & Solutions:**

1. **Power Supply:**
   - ✓ Verify controller board is powered
   - ✓ Check voltage is 3.3V on VDD pins
   - ✓ Ensure sufficient current capability

2. **Cable Connections:**
   - ✓ Check all SWD connections (SWDIO, SWCLK, GND)
   - ✓ Ensure cables are properly seated
   - ✓ Try a different cable

3. **NRST Pin:**
   - ✓ If using NRST, ensure it's connected
   - ✓ Try connecting/disconnecting NRST
   - ✓ Check if NRST has pull-up resistor

4. **SWD Interface Disabled:**
   - ✓ Previous firmware might have disabled SWD
   - ✓ Try connecting under reset:
     - Hold NRST low (reset pressed)
     - Initiate connection
     - Release NRST
   - ✓ As last resort, use BOOT0 pin to enter bootloader mode

### Issue: Programming Failed

**Solutions:**

1. **Reduce Programming Speed:**
   - Try 1000 kHz instead of 4000 kHz
   - Some boards have signal integrity issues at high speeds

2. **Check Flash Protection:**
   - Flash might be read/write protected
   - Use programmer tool to disable protection:
     - J-Link: `unlock kinetis` command
     - ST-Link Utility: Option Bytes, disable RDP

3. **Erase Before Programming:**
   - Perform full chip erase first
   - Then program fresh firmware

4. **Power Cycle:**
   - Disconnect programmer
   - Power cycle board
   - Reconnect and retry

### Issue: Firmware Not Running After Flash

**Solutions:**

1. **Verify Programming:**
   - Re-read flash memory
   - Compare with original HEX file
   - Check for verification errors

2. **Check Reset Vector:**
   - First location at 0x08000000 should contain stack pointer
   - Second location at 0x08000004 should contain reset handler address

3. **Check Boot Configuration:**
   - BOOT0 pin should be LOW for normal operation
   - BOOT1 setting depends on your board design

4. **Clock Configuration:**
   - Ensure external crystal is oscillating (if used)
   - Check HSE (High Speed External) clock source

5. **Debug Mode:**
   - Use debugger to step through startup code
   - Check if code reaches main()
   - Verify SystemInit() executes properly

### Issue: Watchdog Resets

**Solutions:**

1. **Disable Watchdog During Development:**
   - Comment out `IWDG_Init()` call in main.c (line 108)
   - Rebuild and re-flash

2. **Check Feed Function:**
   - Ensure `IWDG_Feed()` is called regularly
   - Should be called at least every few hundred milliseconds

### Issue: Communication Errors

**Solutions:**

1. **CAN Bus:**
   - Check CAN_H and CAN_L connections
   - Verify 120Ω termination resistors
   - Check CAN transceiver power

2. **SPI (AD7689):**
   - Verify SPI3 connections
   - Check chip select signal
   - Use oscilloscope to verify clock and data

3. **ADC:**
   - Check analog input ranges
   - Verify reference voltage (VREF+)

### Getting More Help

**Debug Output:**
- Enable UART debug output if available
- Use printf redirection to UART for debugging

**Debugger:**
- Use Keil uVision debugger to step through code
- Set breakpoints in main() and initialization functions
- Watch variables in real-time

**Hardware Inspection:**
- Visual inspection for solder bridges
- Check for proper component placement
- Verify crystal oscillator is soldering correctly

**Contact Support:**
- Document error messages and symptoms
- Provide hardware revision information
- Include debugger connection logs

---

## Quick Reference Commands

### Keil uVision Shortcuts:
- `F7` - Build project
- `F8` - Flash download
- `Ctrl + F5` - Start debug session
- `F5` - Run program
- `F10` - Step over
- `F11` - Step into

### J-Link Commands:
```
connect          - Connect to target
r               - Reset target
h               - Halt CPU
g               - Go/Run
loadfile <file> - Program flash
mem <addr> <bytes> - Read memory
erase           - Erase flash
exit            - Quit
```

### Important Addresses:
```
Flash Start:    0x08000000
Flash End:      0x080FFFFF (1024KB)
SRAM Start:     0x20000000
SRAM Size:      128KB
CCM RAM:        0x10000000 (64KB)
```

---

## Safety Notes

⚠️ **IMPORTANT SAFETY WARNINGS:**

1. **Power Supply:**
   - Verify voltage levels before connecting
   - Never exceed maximum ratings
   - Use current-limited power supply during development

2. **Disconnect High Power:**
   - Disconnect pump motors and actuators during initial programming
   - Test control signals with LEDs or oscilloscope first
   - Only connect high-power loads after verification

3. **ESD Protection:**
   - Use ESD wrist strap when handling controller board
   - Work on ESD-safe mat
   - Controller contains sensitive CMOS circuitry

4. **Short Circuit Protection:**
   - Inspect board for solder bridges before power-on
   - Check for shorts between VDD and GND

5. **Backup:**
   - Always backup working firmware before flashing new code
   - Keep known-good binary files archived with version numbers

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-10-27 | Initial deployment guide |

---

## Additional Resources

- **STM32F407 Reference Manual:** [RM0090](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- **STM32F407 Datasheet:** [DS8626](https://www.st.com/resource/en/datasheet/stm32f407ze.pdf)
- **Keil uVision User Guide:** Available in IDE Help menu
- **J-Link User Guide:** https://www.segger.com/downloads/jlink/UM08001
- **STM32CubeProgrammer Manual:** https://www.st.com/resource/en/user_manual/dm00403500.pdf

---

**Document End**
