# STM32CubeIDE Setup Guide for PumpBoard Controller

## Overview
This guide provides step-by-step instructions for setting up the PumpBoard Controller project in STM32CubeIDE. The project is currently configured for Keil uVision, so this guide will help you import and configure it for STM32CubeIDE.

**Target Hardware:** STM32F407ZE Microcontroller
**Current Project Format:** Keil MDK-ARM (uVision)
**Target IDE:** STM32CubeIDE
**Project:** PumpBoard_Car_PPQ1_Single V0.0.0.2

---

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Install STM32CubeIDE](#install-stm32cubeide)
3. [Method 1: Import Existing Keil Project](#method-1-import-existing-keil-project-recommended)
4. [Method 2: Create New STM32CubeIDE Project from Scratch](#method-2-create-new-stm32cubeide-project-from-scratch)
5. [Project Configuration](#project-configuration)
6. [Build the Project](#build-the-project)
7. [Flash and Debug](#flash-and-debug)
8. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Required Software:
1. **STM32CubeIDE** - Version 1.13.0 or later
2. **STM32CubeMX** - Integrated within STM32CubeIDE
3. **ST-Link drivers** - Usually installed with STM32CubeIDE

### Hardware:
1. PumpBoard Controller with STM32F407ZE
2. ST-Link V2/V3 or J-Link debugger
3. USB cable for debugger connection
4. Power supply for the controller board

---

## Install STM32CubeIDE

### Step 1: Download STM32CubeIDE

1. Visit: https://www.st.com/en/development-tools/stm32cubeide.html
2. Click "Get Software" or "Download"
3. Select your operating system:
   - Windows (installer or portable)
   - Linux (Debian/RPM or generic)
   - macOS

4. You may need to create a free ST account

### Step 2: Install STM32CubeIDE

**For Windows:**
```
1. Run the installer executable
2. Follow the installation wizard
3. Accept the license agreement
4. Choose installation directory (default: C:\ST\STM32CubeIDE)
5. Install ST-Link drivers when prompted
6. Complete the installation
```

**For Linux:**
```bash
# For Debian/Ubuntu (.deb package)
sudo dpkg -i st-stm32cubeide_*.deb
sudo apt-get install -f  # Install dependencies if needed

# Or for generic Linux (.sh installer)
chmod +x st-stm32cubeide_*.sh
sudo ./st-stm32cubeide_*.sh

# Install udev rules for ST-Link
sudo cp ~/st/stm32cubeide_*/stlinkrules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**For macOS:**
```
1. Open the .dmg file
2. Drag STM32CubeIDE to Applications folder
3. Launch and allow security permissions if prompted
```

### Step 3: First Launch

1. Launch STM32CubeIDE
2. Select a workspace directory (e.g., `~/STM32CubeIDE/workspace`)
3. Wait for the IDE to initialize
4. Close the welcome screen

---

## Method 1: Import Existing Keil Project (Recommended)

This method attempts to import the existing Keil project structure into STM32CubeIDE.

### Step 1: Create New STM32 Project

1. **Open STM32CubeIDE**

2. **Create New Project:**
   - File → New → STM32 Project
   - Wait for the target selection window

3. **Select MCU:**
   - In the "MCU/MPU Selector" tab
   - Commercial Part Number: Type `STM32F407ZE`
   - Select `STM32F407ZETx` from the list
   - Click "Next"

4. **Project Setup:**
   - Project Name: `PumpBoard_Controller`
   - Targeted Language: `C`
   - Targeted Binary Type: `Executable`
   - Targeted Project Type: `STM32Cube`
   - Click "Finish"

5. **Initialize Peripherals:**
   - When prompted to initialize peripherals to default mode
   - Select **"No"** (we'll configure manually based on the Keil project)

### Step 2: Configure Clock and Peripherals

Based on the existing Keil project, configure the following:

1. **Open .ioc file** (should open automatically)

2. **Configure RCC (Clock):**
   - Navigate to: System Core → RCC
   - High Speed Clock (HSE): Select `Crystal/Ceramic Resonator`
   - Low Speed Clock (LSE): `Disable` (unless you need RTC)

3. **Configure SYS:**
   - Navigate to: System Core → SYS
   - Debug: `Serial Wire` (for SWD debugging)

4. **Configure Clock Tree:**
   - Click "Clock Configuration" tab at the top
   - Set HCLK to `168 MHz` (maximum for STM32F407)
   - Input frequency: `8 MHz` or `25 MHz` (check your board's crystal)
   - PLL Source: `HSE`
   - Configure PLL to achieve 168 MHz:
     - PLLM: `8` (if using 8MHz crystal) or `25` (if using 25MHz)
     - PLLN: `336`
     - PLLP: `2`
     - System Clock Mux: `PLLCLK`

5. **Configure Peripherals Based on Keil Project:**

   **TIM2 (PWM for pump control):**
   - Navigate to: Timers → TIM2
   - Clock Source: `Internal Clock`
   - Channel: Enable required channels for PWM
   - Mode: `PWM Generation CHx`

   **SPI3 (for AD7689 ADC):**
   - Navigate to: Connectivity → SPI3
   - Mode: `Full-Duplex Master`
   - Hardware NSS Signal: `Disable` (using software CS)
   - Configuration:
     - Frame Format: `Motorola`
     - Data Size: `16 Bits`
     - First Bit: `MSB First`

   **ADC1 (Internal ADC):**
   - Navigate to: Analog → ADC1
   - Mode: Enable required channels
   - Configuration: Based on your requirements

   **CAN1 (CAN Bus communication):**
   - Navigate to: Connectivity → CAN1
   - Mode: `Activated`
   - Configuration: Set baud rate as needed

   **GPIO:**
   - Configure GPIO pins as needed for your hardware
   - Refer to original Keil project for pin mappings

6. **Configure NVIC:**
   - Navigate to: System Core → NVIC
   - Enable interrupts for:
     - TIM2 (if using interrupts)
     - SPI3 (if using interrupts)
     - ADC1 (if using interrupts)
     - CAN1
     - Set priorities as needed

7. **Generate Code:**
   - Click "Project" menu → "Generate Code"
   - Or click the gear icon in the toolbar
   - Wait for code generation to complete

### Step 3: Copy Source Files from Keil Project

Now we need to copy the application code from the Keil project:

1. **Navigate to your project directory:**
   ```
   workspace/PumpBoard_Controller/
   ```

2. **Copy source files:**

   ```bash
   # From the Keil project directory:
   PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/

   # Copy to STM32CubeIDE project:
   workspace/PumpBoard_Controller/
   ```

   **Files to copy:**

   - **Application code:**
     - `HardWare_Init/Src/*.c` → `Core/Src/` or create `HardWare_Init/Src/`
     - `HardWare_Init/Inc/*.h` → `Core/Inc/` or create `HardWare_Init/Inc/`
     - `Function/*.c` and `*.h` → Create `Function/` directory
     - `USER/main.c` content → Merge with generated `Core/Src/main.c`

   - **Library files:**
     - `FWLIB/` → Copy entire directory to project root
     - `LIB/` → Copy if needed

3. **Create directory structure:**

   In STM32CubeIDE project, create these directories:
   ```
   PumpBoard_Controller/
   ├── Core/
   │   ├── Inc/
   │   └── Src/
   ├── HardWare_Init/
   │   ├── Inc/
   │   └── Src/
   ├── Function/
   ├── FWLIB/
   └── Drivers/  (auto-generated)
   ```

### Step 4: Update Include Paths

1. **Right-click project** → Properties

2. **Navigate to:**
   - C/C++ Build → Settings → Tool Settings
   - MCU GCC Compiler → Include paths

3. **Add include paths:**
   ```
   ../Core/Inc
   ../HardWare_Init/Inc
   ../Function
   ../FWLIB/inc
   ../Drivers/STM32F4xx_HAL_Driver/Inc
   ../Drivers/CMSIS/Device/ST/STM32F4xx/Include
   ../Drivers/CMSIS/Include
   ```

4. Click "Apply"

### Step 5: Add Source Files to Build

1. **In Project Explorer**, right-click project → Properties

2. **C/C++ Build → Settings → Tool Settings**

3. **MCU GCC Compiler → Preprocessor**
   - Add required defines:
     ```
     STM32F407xx
     USE_HAL_DRIVER
     ```

4. **Make sure all .c files are included:**
   - Check that new directories appear in Project Explorer
   - If not, refresh (F5) or close and reopen project

### Step 6: Resolve Code Conflicts

The Keil project likely uses SPL (Standard Peripheral Library), while STM32CubeIDE generates HAL (Hardware Abstraction Layer) code. You have two options:

**Option A: Keep SPL and disable HAL**
1. Remove HAL driver files
2. Use SPL library from FWLIB
3. Update preprocessor defines

**Option B: Migrate to HAL** (Recommended for new projects)
1. Gradually replace SPL calls with HAL equivalents
2. Use both libraries during transition
3. Eventually remove SPL

For this guide, we'll keep SPL since the existing code uses it.

### Step 7: Configure Linker Script

1. **Check the linker script:**
   - File: `STM32F407ZETX_FLASH.ld`
   - Located in project root

2. **Verify memory configuration:**
   ```ld
   MEMORY
   {
     RAM (xrw)    : ORIGIN = 0x20000000, LENGTH = 128K
     CCMRAM (rw)  : ORIGIN = 0x10000000, LENGTH = 64K
     FLASH (rx)   : ORIGIN = 0x08000000, LENGTH = 512K
   }
   ```

3. **Update if needed** based on your memory requirements

---

## Method 2: Create New STM32CubeIDE Project from Scratch

If Method 1 is too complex, you can create a minimal project and add features incrementally.

### Step 1: Create Minimal Project

1. Follow Method 1, Step 1 to create basic project

2. Configure only essential peripherals:
   - RCC (clock)
   - SYS (debug)
   - GPIO (basic I/O)

3. Generate code

### Step 2: Build and Test

1. Build the project (Ctrl+B)
2. Fix any errors
3. Flash to board
4. Verify basic functionality

### Step 3: Add Features Incrementally

Add peripherals one by one:
1. Add TIM2, generate code, test
2. Add SPI3, generate code, test
3. Add ADC1, generate code, test
4. And so on...

This approach is slower but more controlled.

---

## Project Configuration

### Compiler Settings

1. **Right-click project → Properties**

2. **C/C++ Build → Settings → MCU GCC Compiler**

   **Optimization:**
   - Optimization level: `-O2` or `-O3` for release
   - Debug level: `-g3` for debugging

   **Warnings:**
   - Enable: `-Wall`

   **Preprocessor defines:**
   ```
   STM32F407xx
   USE_STDPERIPH_DRIVER
   ARM_MATH_CM4
   __FPU_PRESENT=1
   ```

3. **MCU GCC Linker → General**
   - Linker script: Verify correct `.ld` file is selected

4. **Apply and Close**

### Debug Configuration

1. **Run → Debug Configurations**

2. **Create new STM32 C/C++ Application**

3. **Debugger tab:**
   - Debug probe: `ST-LINK (ST-LINK GDB server)`
   - Interface: `SWD`
   - Frequency: `4000 kHz`
   - Reset Mode: `Software system reset`

4. **Apply**

---

## Build the Project

### Step 1: Clean Project

1. **Project → Clean...**
2. Select your project
3. Click "Clean"

### Step 2: Build

1. **Project → Build All** (Ctrl+B)
2. Check Console for errors/warnings
3. Build should complete with:
   ```
   Finished building target: PumpBoard_Controller.elf
   arm-none-eabi-objcopy -O ihex "PumpBoard_Controller.elf" "PumpBoard_Controller.hex"
   arm-none-eabi-objcopy -O binary "PumpBoard_Controller.elf" "PumpBoard_Controller.bin"
   ```

### Step 3: Verify Output Files

Check that these files were created in `Debug/` or `Release/` folder:
- `PumpBoard_Controller.elf` - ELF executable
- `PumpBoard_Controller.hex` - Intel HEX file
- `PumpBoard_Controller.bin` - Binary file
- `PumpBoard_Controller.map` - Memory map

---

## Flash and Debug

### Hardware Connection

1. **Connect ST-Link or J-Link to PC**
2. **Connect debugger to board:**
   - SWDIO
   - SWCLK
   - GND
   - 3.3V (optional, if board needs power from debugger)
3. **Power on the board**

### Flash the Firmware

**Method 1: Using Debug Configuration (Recommended)**

1. Click debug icon or **Run → Debug** (F11)
2. STM32CubeIDE will:
   - Build the project (if needed)
   - Flash the firmware
   - Start debug session
   - Stop at main()
3. Click Resume (F8) to run

**Method 2: Using Run Configuration**

1. **Run → Run Configurations**
2. Create new **STM32 C/C++ Application**
3. Click "Run"
4. Firmware will be flashed and executed

**Method 3: Using STM32CubeProgrammer**

1. **Run → Run Configurations**
2. Select **STM32 Cortex-M C/C++ Application**
3. Right-click → "Download"
4. Or use external STM32CubeProgrammer GUI

### Debug Features

**Breakpoints:**
- Double-click in left margin of code editor
- Or right-click line → Toggle Breakpoint

**Step Commands:**
- F5: Step Into
- F6: Step Over
- F7: Step Return
- F8: Resume

**Watch Variables:**
- Right-click variable → Add Watch Expression
- View in "Expressions" tab

**Live Expressions:**
- Window → Show View → Live Expressions
- Add variables to monitor in real-time

**Memory View:**
- Window → Show View → Memory
- Enter address to monitor (e.g., `0x20000000`)

**Peripheral Registers:**
- Window → Show View → SFRs (Special Function Registers)
- View and modify peripheral registers in real-time

---

## Troubleshooting

### Issue: "No ST-LINK detected"

**Solutions:**
1. Check USB connection
2. Install ST-Link drivers:
   - Windows: Use STM32CubeIDE installer or download separately
   - Linux: Install udev rules (see installation section)
3. Try different USB port
4. Update ST-Link firmware using STM32CubeProgrammer

### Issue: "Error in final launch sequence"

**Solutions:**
1. Check debugger connections (SWDIO, SWCLK, GND)
2. Verify board is powered
3. Try lower SWD frequency (1000 kHz instead of 4000 kHz):
   - Debug Configurations → Debugger → Frequency
4. Try "Connect under reset":
   - Debug Configurations → Debugger → Mode Setup
   - Enable "Connect under reset"

### Issue: Build errors with HAL/SPL conflicts

**Solutions:**
1. Decide on one library (HAL or SPL)
2. Remove conflicting headers:
   - For SPL: Remove HAL includes, keep `stm32f4xx_conf.h`
   - For HAL: Remove SPL includes, keep `stm32f4xx_hal_conf.h`
3. Update preprocessor defines accordingly

### Issue: Undefined reference errors

**Solutions:**
1. Check all source files are included in build:
   - Project Properties → C/C++ Build → Settings → Tool Settings
   - MCU GCC Linker → Source folders
2. Verify include paths are correct
3. Check that libraries are linked:
   - MCU GCC Linker → Libraries
   - Add: `m` (math library) if using math functions

### Issue: Hard Fault after flashing

**Solutions:**
1. Check clock configuration:
   - Verify HSE frequency matches crystal on board
   - Ensure PLL settings are correct
2. Verify linker script memory regions
3. Check stack size is sufficient:
   - Edit `.ld` file, increase `_Min_Stack_Size`
4. Enable fault handlers for debugging:
   ```c
   void HardFault_Handler(void) {
       while(1) {
           // Set breakpoint here
       }
   }
   ```

### Issue: Watchdog resets during debugging

**Solutions:**
1. Disable IWDG during development:
   - Comment out watchdog initialization in main.c
2. Or configure debugger to disable watchdog:
   - Debug Configurations → Debugger → Debug Options
   - Add init command: `monitor reset init`

### Issue: Cannot program flash - "Flash Loader"

**Solutions:**
1. Erase flash first:
   - Use STM32CubeProgrammer to perform full chip erase
2. Check flash protection:
   - Read Option Bytes
   - Disable read/write protection if enabled
3. Verify target voltage (should be 3.3V)

---

## Additional Tips

### Using J-Link Instead of ST-Link

1. **Install J-Link software pack:**
   - Download from: https://www.segger.com/downloads/jlink/

2. **Configure debug probe:**
   - Debug Configurations → Debugger
   - Debug probe: `J-Link (J-Link GDB Server)`
   - Interface: `SWD`

3. **Test connection:**
   - Use J-Link Commander to verify communication

### Project Conversion Checklist

When converting from Keil to STM32CubeIDE, check:

- [ ] All source files copied
- [ ] All header files copied
- [ ] Include paths configured
- [ ] Preprocessor defines set
- [ ] Library files included (SPL or HAL)
- [ ] Linker script configured
- [ ] Startup file appropriate for device
- [ ] Clock configuration matches
- [ ] Peripheral initialization matches
- [ ] Interrupt handlers configured
- [ ] Build completes without errors
- [ ] Firmware flashes successfully
- [ ] Basic functionality verified

### Performance Optimization

For release builds:

1. **Compiler optimization:**
   - `-O3` or `-Ofast` for maximum speed
   - `-Os` for size optimization

2. **Link-Time Optimization (LTO):**
   - Enable: `-flto`

3. **Remove debug info:**
   - Set debug level to `None`

4. **Use FPU:**
   - FPU: `-mfpu=fpv4-sp-d16`
   - Float ABI: `-mfloat-abi=hard`

---

## Quick Reference

### STM32CubeIDE Shortcuts

- `Ctrl+B` - Build project
- `F11` - Debug
- `Ctrl+F11` - Run
- `Ctrl+Space` - Auto-complete
- `F3` - Open declaration
- `Ctrl+Shift+R` - Open resource
- `Ctrl+/` - Toggle comment

### Important Directories

```
PumpBoard_Controller/
├── Core/              - Generated HAL code
│   ├── Inc/          - Headers
│   ├── Src/          - Sources
│   └── Startup/      - Startup files
├── Drivers/          - HAL/LL drivers
│   ├── STM32F4xx_HAL_Driver/
│   └── CMSIS/
├── Debug/            - Debug build output
└── Release/          - Release build output
```

### Memory Map (STM32F407ZE)

| Region | Start Address | Size | Description |
|--------|--------------|------|-------------|
| Flash  | 0x08000000  | 512KB | Program memory |
| SRAM   | 0x20000000  | 128KB | Data memory |
| CCM RAM| 0x10000000  | 64KB  | Core-coupled memory |
| Peripherals | 0x40000000 | - | Peripheral registers |

---

## Resources

- **STM32CubeIDE User Guide:** https://www.st.com/resource/en/user_manual/um2609-stm32cubeide-user-guide-stmicroelectronics.pdf
- **STM32F407 Reference Manual:** https://www.st.com/resource/en/reference_manual/dm00031020.pdf
- **STM32F407 Datasheet:** https://www.st.com/resource/en/datasheet/stm32f407ze.pdf
- **STM32CubeMX User Guide:** https://www.st.com/resource/en/user_manual/um1718-stm32cubemx-for-stm32-configuration-and-initialization-c-code-generation-stmicroelectronics.pdf

---

**Last Updated:** 2025-10-27
**Version:** 1.0
