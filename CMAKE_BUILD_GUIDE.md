# STM32 CMake Build Guide

This guide explains how to build the PumpBoard Controller firmware using CMake instead of Keil µVision.

## Prerequisites

1. **ARM GCC Toolchain**: Install the ARM GNU toolchain
   ```bash
   # Ubuntu/Debian
   sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi

   # Or download from: https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm
   ```

2. **CMake**: Version 3.22 or higher
   ```bash
   sudo apt-get install cmake
   ```

3. **Build Tools**: Make or Ninja
   ```bash
   sudo apt-get install make
   # or
   sudo apt-get install ninja-build
   ```

## Project Structure

```
PumpBoard_Controller/
├── CMakeLists.txt                  # Main CMake configuration
├── arm-none-eabi-gcc.cmake        # ARM GCC toolchain file
├── STM32F407ZE_FLASH.ld           # GCC linker script
├── CMAKE_BUILD_GUIDE.md           # This file
└── PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/
    ├── CORE/                      # Startup and system files
    │   ├── startup_stm32f407xx_gcc.S   # GCC startup file
    │   └── system_stm32f4xx.c
    ├── FWLIB/                     # STM32 Standard Peripheral Library
    ├── Function/                  # Application logic
    ├── HardWare_Init/            # Hardware initialization
    ├── LIB/                      # Prebuilt libraries
    └── USER/                     # User application (main.c)
```

## Configuration Details

### MCU Configuration
- **Device**: STM32F407ZE
- **Core**: ARM Cortex-M4 with FPU
- **Flash**: 512KB (configured as 256KB in linker script to match Keil project)
- **RAM**: 128KB
- **CCM RAM**: 64KB

### Compiler Definitions
- `STM32F40_41xxx`
- `STM32F407xx`
- `USE_STDPERIPH_DRIVER`

### Optimization
- Optimization level: `-O2`
- Debug information: `-g3 -gdwarf-2`
- Link-time optimization: Disabled (can be enabled if needed)

## Building the Project

### Step 1: Create Build Directory
```bash
cd /home/user/PumpBoard_Controller
mkdir build
cd build
```

### Step 2: Configure with CMake
```bash
# Configure the project with ARM toolchain
cmake -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi-gcc.cmake ..

# Or use Ninja for faster builds
cmake -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi-gcc.cmake -G Ninja ..
```

### Step 3: Build
```bash
# Using Make
make

# Or using Ninja
ninja

# Build with verbose output to see compiler commands
make VERBOSE=1
# or
ninja -v
```

### Step 4: Build Outputs

After a successful build, you'll find the following files in the `build/` directory:

- `PumpCtrl.elf` - Executable with debug symbols
- `PumpCtrl.bin` - Binary file for flashing
- `PumpCtrl.hex` - Intel HEX file for flashing
- `PumpCtrl.map` - Memory map file
- `PumpCtrl.lst` - Extended listing file

## Flashing the Firmware

### Using OpenOCD
```bash
# Flash the binary
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "program build/PumpCtrl.bin 0x08000000 verify reset exit"
```

### Using st-flash (from stlink tools)
```bash
st-flash write build/PumpCtrl.bin 0x08000000
```

### Using STM32CubeProgrammer
1. Open STM32CubeProgrammer
2. Connect to the board
3. Load `PumpCtrl.hex` or `PumpCtrl.bin`
4. Program the device

## Debugging

### Using GDB with OpenOCD
Terminal 1 - Start OpenOCD:
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
```

Terminal 2 - Start GDB:
```bash
arm-none-eabi-gdb build/PumpCtrl.elf
(gdb) target extended-remote localhost:3333
(gdb) monitor reset halt
(gdb) load
(gdb) continue
```

### Using VSCode
Create `.vscode/launch.json`:
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug STM32",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/build/PumpCtrl.elf",
            "cwd": "${workspaceFolder}",
            "MIMode": "gdb",
            "miDebuggerPath": "arm-none-eabi-gdb",
            "miDebuggerServerAddress": "localhost:3333",
            "setupCommands": [
                {
                    "text": "target extended-remote localhost:3333"
                },
                {
                    "text": "monitor reset halt"
                },
                {
                    "text": "load"
                }
            ]
        }
    ]
}
```

## Clean Build
```bash
# Remove build directory
rm -rf build

# Or from within build directory
make clean
# or
ninja clean
```

## Customization

### Change Optimization Level
Edit `CMakeLists.txt` and modify:
```cmake
-O2  # Change to -O0, -O1, -O3, -Os, or -Og
```

### Enable USB Support
Uncomment USB-related lines in `CMakeLists.txt`:
```cmake
add_compile_definitions(
    STM32F40_41xxx
    STM32F407xx
    USE_STDPERIPH_DRIVER
    USE_USB_FS  # Uncomment this
)
```

And uncomment USB include paths:
```cmake
include_directories(
    # ... existing paths ...
    ${PROJECT_DIR}/USB/STM32_USB_Device_Library/Class/CDC/Inc
    ${PROJECT_DIR}/USB/STM32_USB_Device_Library/Core/Inc
    ${PROJECT_DIR}/USB/USB_APP
    ${PROJECT_DIR}/USB/USB_HAL
)
```

### Modify Memory Configuration
Edit `STM32F407ZE_FLASH.ld` to change flash/RAM sizes:
```ld
MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K   /* Change as needed */
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 128K
  CCMRAM (rw)     : ORIGIN = 0x10000000, LENGTH = 64K
}
```

## Comparing with Keil Build

The CMake configuration replicates the Keil µVision project settings:

| Setting | Keil | CMake/GCC |
|---------|------|-----------|
| Compiler | ARMCLANG v6.24 | arm-none-eabi-gcc |
| MCU | STM32F407ZE | STM32F407ZE |
| FPU | Hardware FP | -mfpu=fpv4-sp-d16 -mfloat-abi=hard |
| Optimization | -O2 | -O2 |
| C Standard | C99 | C99 |
| Defines | STM32F40_41xxx, STM32F407xx, USE_STDPERIPH_DRIVER | Same |

## Troubleshooting

### Toolchain Not Found
```
CMake Error: CMAKE_C_COMPILER not found
```
**Solution**: Install `gcc-arm-none-eabi` or add it to your PATH.

### Undefined Reference Errors
If you see undefined references to standard library functions:
- Check that `-specs=nano.specs -specs=nosys.specs` are in linker flags
- Verify all source files are included in `CMakeLists.txt`

### Library Compatibility Issues
The prebuilt libraries (`Deadzone_Current.lib`, `PID_AREA.lib`) must be compatible with ARM GCC. If they were built with ARMCC, you may need to:
1. Request GCC-compatible versions
2. Rebuild them from source
3. Link errors will indicate incompatibility

### Flash Size Mismatch
If the binary is larger than 256KB, edit `STM32F407ZE_FLASH.ld`:
```ld
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K
```

## Additional Resources

- [STM32F407 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [ARM GCC Toolchain Documentation](https://gcc.gnu.org/onlinedocs/gcc/)
- [CMake Documentation](https://cmake.org/documentation/)
- [OpenOCD User Guide](http://openocd.org/doc/html/index.html)

## Notes

- The original Keil project used ARMCC compiler; this CMake setup uses GCC
- A new GCC-compatible startup file was created (`startup_stm32f407xx_gcc.S`)
- The linker script was converted from Keil's scatter file format to GCC's linker script format
- USB support is disabled by default (can be enabled if USB source files are available)
