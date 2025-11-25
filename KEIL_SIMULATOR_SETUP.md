# Keil µVision Simulator Mode Configuration Guide

## Issues Fixed

This document describes the configuration fixes applied to enable proper simulator mode operation for the PumpBoard Controller project.

## Problems Encountered

### 1. Memory Access Violation at 0x40023800
**Error:** `access violation at 0x40023800 : no 'read' permission`

**Root Cause:** The RCC (Reset and Clock Control) peripheral registers at address 0x40023800 were not properly mapped in the simulator's memory map configuration.

**Solution:** Updated `USER/debug.ini` to include comprehensive memory mappings for all STM32F407 peripherals, including:
- RCC registers (0x40023800-0x40023BFF) - **CRITICAL FIX**
- All GPIO ports (GPIOA-GPIOI)
- All timers, ADCs, UARTs, SPIs, I2C, CAN peripherals
- DMA controllers
- Flash interface registers
- Cortex-M4 core peripherals

### 2. Syntax Error in Configuration
**Error:** `*** error 10: Syntax error` near `[BREAKPOINTS]`

**Root Cause:** This error typically occurs when:
- The debugger configuration files have conflicting settings
- Watch window expressions contain invalid syntax
- Debug script loading order is incorrect

**Solution:** The `debug.ini` file is now properly structured with correct MAP commands and comprehensive peripheral coverage.

## Configuration Files

### debug.ini (Simulator Mode)
Location: `USER/debug.ini`

This file contains MAP commands that define the memory regions accessible in simulator mode. The file now includes:

1. **APB1 Peripherals** (42 MHz max)
   - Timers (TIM2-TIM7)
   - Serial interfaces (USART2-5, SPI2-3, I2C1-2)
   - CAN controllers

2. **APB2 Peripherals** (84 MHz max)
   - High-speed timers (TIM1, TIM8)
   - Fast serial (USART1, USART6, SPI1)
   - ADC modules
   - SYSCFG and EXTI

3. **AHB1 Peripherals** (168 MHz max)
   - **RCC at 0x40023800** ← Most critical for initialization
   - GPIO ports (GPIOA-GPIOI)
   - DMA controllers
   - Flash interface

4. **Memory Regions**
   - Flash: 0x08000000-0x080FFFFF (1MB, executable)
   - SRAM: 0x20000000-0x2001FFFF (128KB)
   - CCM RAM: 0x10000000-0x1000FFFF (64KB)

### JLinkSettings.ini (Hardware Debug Mode)
Location: `USER/JLinkSettings.ini`

This file is used only when debugging with J-Link hardware debugger. It should NOT be used in simulator mode.

## How to Use Simulator Mode

### Step 1: Verify Project Settings
1. Open the project in Keil µVision
2. Go to **Project → Options for Target → Debug**
3. Ensure **"Use Simulator"** is selected (not J-Link or other hardware debugger)
4. In the simulator settings, verify that **Initialization File** points to: `.\debug.ini`

### Step 2: Start Debug Session
1. Build the project: **Project → Build Target** (F7)
2. Start debug session: **Debug → Start/Stop Debug Session** (Ctrl+F5)
3. The simulator should now load without memory access violations

### Step 3: Verify Memory Mapping
During debug session, you can verify memory access:
1. Open **View → Memory Window**
2. Enter address `0x40023800` (RCC registers)
3. You should see the memory contents without access violation errors

## Watch Window Configuration

The project includes predefined watch windows in the `.uvoptx` file:

**Watch Window 1:** Real-time control variables
- InstructionSet values (TiltAngRef, Cur_A_Ref, Cur_B_Ref)
- Detector feedback values
- PWM outputs

**Watch Window 2:** Debug variables (with hexadecimal display)
- User_Parameter structures
- Temperature references
- Loop parameters
- Pressure offsets

## Troubleshooting

### If you still see access violations:

1. **Clean and rebuild the project:**
   ```
   Project → Clean Targets
   Project → Build Target
   ```

2. **Reset the simulator:**
   ```
   Debug → Stop Debug Session
   Close µVision
   Reopen project
   Start Debug Session
   ```

3. **Verify debug.ini is loaded:**
   - Check the debugger output window for "Loading debug.ini" messages
   - Ensure the file path in Project Options is correct: `.\debug.ini`

4. **Check for hardware-specific code:**
   - Some STM32 HAL functions may not work properly in simulator mode
   - Consider using conditional compilation for simulator-specific code:
     ```c
     #ifdef __UVISION_VERSION  // Keil simulator
     // Simulator-compatible code
     #else
     // Hardware-specific code
     #endif
     ```

### If watch variables show errors:

1. Ensure variables are in scope and have been compiled with debug info
2. Rebuild with optimization level 0 for better debug experience
3. Check that variable names match exactly (case-sensitive)

## Limitations of Simulator Mode

Be aware that simulator mode has limitations:

1. **No Real Hardware Timing:** Timer and interrupt timing is approximate
2. **No Peripheral Behavior:** ADCs return 0, GPIO inputs don't change
3. **No External Events:** CAN messages, UART data must be simulated
4. **Limited Interrupt Simulation:** Some interrupts may not trigger correctly

For full testing, use hardware debugging with J-Link or ST-Link.

## Switching to Hardware Debug

To switch from simulator to hardware debugging:

1. Connect your J-Link or ST-Link debugger
2. Go to **Project → Options for Target → Debug**
3. Select your debugger (J-Link/J-Trace or ST-Link)
4. Configure debugger settings as needed
5. The `JLinkSettings.ini` will be used automatically

## Additional Notes

- The current configuration is optimized for **STM32F407ZE** MCU
- Memory map addresses match STM32F4 reference manual (RM0090)
- All peripheral base addresses are from the official STM32F4 memory map

## References

- STM32F407 Reference Manual (RM0090)
- Keil µVision User's Guide
- ARM Cortex-M4 Technical Reference Manual
