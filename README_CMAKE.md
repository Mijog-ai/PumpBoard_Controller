# STM32 PumpBoard Controller - CMake Build System

This project has been converted from Keil µVision to CMake for cross-platform development.

## Quick Start

```bash
# Build the project
./build.sh

# Clean and rebuild
./build.sh rebuild

# Flash to device
./build.sh flash

# Build and flash
./build.sh all
```

## What's New

The following files have been added to support CMake builds:

1. **CMakeLists.txt** - Main CMake configuration file
2. **arm-none-eabi-gcc.cmake** - ARM GCC toolchain file
3. **STM32F407ZE_FLASH.ld** - GCC linker script (converted from Keil scatter file)
4. **startup_stm32f407xx_gcc.S** - GCC-compatible startup file
5. **build.sh** - Convenient build script
6. **CMAKE_BUILD_GUIDE.md** - Comprehensive build guide
7. **README_CMAKE.md** - This file

## Key Features

- ✅ STM32F407ZE (Cortex-M4 with FPU)
- ✅ STM32 Standard Peripheral Library
- ✅ Hardware FPU support
- ✅ Optimization level O2
- ✅ Debug symbols included
- ✅ Generates .elf, .bin, .hex, .map, and .lst files
- ✅ Prebuilt library support
- ✅ Memory usage reporting

## Prerequisites

Install the ARM GCC toolchain and CMake:

```bash
# Ubuntu/Debian
sudo apt-get install gcc-arm-none-eabi cmake make

# For flashing (optional)
sudo apt-get install stlink-tools  # or openocd
```

## Manual Build

If you prefer not to use the build script:

```bash
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi-gcc.cmake ..
make -j$(nproc)
```

## Documentation

See [CMAKE_BUILD_GUIDE.md](CMAKE_BUILD_GUIDE.md) for:
- Detailed build instructions
- Debugging setup
- Flashing options
- Customization guide
- Troubleshooting

## Project Structure

```
PumpBoard_Controller/
├── CMakeLists.txt                 # CMake configuration
├── arm-none-eabi-gcc.cmake       # Toolchain file
├── STM32F407ZE_FLASH.ld          # Linker script
├── build.sh                      # Build script
├── CMAKE_BUILD_GUIDE.md          # Full documentation
└── PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/
    ├── CORE/                     # Startup & system
    ├── FWLIB/                    # STM32 peripherals
    ├── Function/                 # Application
    ├── HardWare_Init/           # Hardware init
    ├── LIB/                     # Libraries
    └── USER/                    # Main application
```

## Configuration Summary

| Item | Value |
|------|-------|
| MCU | STM32F407ZE |
| Core | Cortex-M4 |
| Flash | 256KB (configurable to 512KB) |
| RAM | 128KB |
| CCM RAM | 64KB |
| FPU | Hardware (fpv4-sp-d16) |
| Optimization | -O2 |
| C Standard | C99 |

## Build Outputs

After building, you'll find in the `build/` directory:

- **PumpCtrl.elf** - Executable with debug symbols
- **PumpCtrl.bin** - Raw binary for flashing
- **PumpCtrl.hex** - Intel HEX format
- **PumpCtrl.map** - Memory map
- **PumpCtrl.lst** - Disassembly listing

## Compatibility

This CMake configuration replicates the original Keil µVision project settings as closely as possible:

- Same source files
- Same compiler defines
- Same include paths
- Same memory layout
- Same optimization level

## Support

For issues or questions:
1. Check [CMAKE_BUILD_GUIDE.md](CMAKE_BUILD_GUIDE.md) troubleshooting section
2. Verify ARM toolchain installation
3. Ensure CMake version >= 3.22

## Original Project

The original Keil µVision project files are preserved:
- `PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/PumpCtrl.uvprojx`

You can continue using Keil µVision alongside this CMake setup.
