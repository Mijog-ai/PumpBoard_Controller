# PumpCtrl Build & Flash Guide

## Quick Start

### Build the Project
```cmd
build.bat
```
Or manually:
```cmd
cbuild PumpCtrl.csolution.yml
```

### Flash to Hardware
```cmd
flash.bat
```
Choose your programmer (ST-Link or J-Link) from the menu.

## Build Output
- `out/PumpCtrl/PPQ_Ctrl/PumpCtrl.axf` - ELF executable
- `out/PumpCtrl/PPQ_Ctrl/PumpCtrl.bin` - Binary file for flashing
- `out/PumpCtrl/PPQ_Ctrl/PumpCtrl.axf.map` - Memory map

## Manual Flashing

### Using ST-Link
```cmd
STM32_Programmer_CLI -c port=SWD -w out\PumpCtrl\PPQ_Ctrl\PumpCtrl.bin 0x08000000 -v -rst
```

### Using J-Link
```cmd
JLink.exe -device STM32F407ZE -if SWD -speed 4000
```
Then in J-Link Commander:
```
loadbin out\PumpCtrl\PPQ_Ctrl\PumpCtrl.bin 0x08000000
r
g
```

## Project Configuration
- Target: STM32F407ZETx
- Compiler: ARM Compiler 6 (AC6)
- Optimization: Debug
- FPU: Single Precision (SP)

## Changes Made
1. Fixed device variant: `STM32F407ZE` → `STM32F407ZETx`
2. Disabled simulation mode in `main.c` for real hardware
3. Fixed assembly file compilation flags
