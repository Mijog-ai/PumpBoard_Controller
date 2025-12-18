cmake_minimum_required(VERSION 3.22)

# Set project name and languages
project(PumpCtrl C ASM)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Enable export of compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Define the MCU
set(MCU_FAMILY STM32F4xx)
set(MCU_MODEL STM32F407xx)
set(CPU_TYPE cortex-m4)
set(FPU_TYPE fpv4-sp-d16)
set(FLOAT_ABI hard)

# Project paths
set(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27)

# Compiler options
add_compile_options(
    -mcpu=${CPU_TYPE}
    -mthumb
    -mfpu=${FPU_TYPE}
    -mfloat-abi=${FLOAT_ABI}

    # Optimization and debugging
    -O2
    -g3
    -gdwarf-2

    # Warnings
    -Wall
    -Wextra
    -Wno-unused-parameter

    # Code generation
    -fdata-sections
    -ffunction-sections
    -fstack-usage
)

# Linker options
add_link_options(
    -mcpu=${CPU_TYPE}
    -mthumb
    -mfpu=${FPU_TYPE}
    -mfloat-abi=${FLOAT_ABI}

    # Linker script
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F407ZE_FLASH.ld

    # Link options
    -specs=nano.specs
    -specs=nosys.specs
    -lc
    -lm
    -lnosys

    # Remove unused sections
    -Wl,--gc-sections

    # Generate map file
    -Wl,-Map=${PROJECT_NAME}.map,--cref

    # Print memory usage
    -Wl,--print-memory-usage
)

# Preprocessor definitions
add_compile_definitions(
    STM32F40_41xxx
    STM32F407xx
    USE_STDPERIPH_DRIVER
    # USB support - uncomment if USB files are added
    # USE_USB_FS
)

# Include directories
include_directories(
    ${PROJECT_DIR}/CORE
    ${PROJECT_DIR}/Function
    ${PROJECT_DIR}/FWLIB/inc
    ${PROJECT_DIR}/HardWare_Init/Inc
    # USB includes - uncomment if USB is needed
    # ${PROJECT_DIR}/USB/STM32_USB_Device_Library/Class/CDC/Inc
    # ${PROJECT_DIR}/USB/STM32_USB_Device_Library/Core/Inc
    # ${PROJECT_DIR}/USB/USB_APP
    # ${PROJECT_DIR}/USB/USB_HAL
)

# Source files - USER
set(USER_SOURCES
    ${PROJECT_DIR}/USER/main.c
)

# Source files - CORE
set(CORE_SOURCES
    ${PROJECT_DIR}/CORE/startup_stm32f407xx_gcc.S
    ${PROJECT_DIR}/CORE/system_stm32f4xx.c
)

# Source files - FWLIB (STM32 Standard Peripheral Library)
file(GLOB FWLIB_SOURCES
    ${PROJECT_DIR}/FWLIB/src/*.c
)

# Source files - HardWare_Init
file(GLOB HARDWARE_SOURCES
    ${PROJECT_DIR}/HardWare_Init/Src/*.c
)

# Source files - Function
file(GLOB FUNCTION_SOURCES
    ${PROJECT_DIR}/Function/*.c
)

# Combine all sources
set(PROJECT_SOURCES
    ${USER_SOURCES}
    ${CORE_SOURCES}
    ${FWLIB_SOURCES}
    ${HARDWARE_SOURCES}
    ${FUNCTION_SOURCES}
)

# Create executable
add_executable(${PROJECT_NAME}.elf ${PROJECT_SOURCES})

# Link prebuilt libraries
target_link_libraries(${PROJECT_NAME}.elf
    ${PROJECT_DIR}/LIB/Deadzone_Current.lib
    ${PROJECT_DIR}/LIB/PID_AREA.lib
)

# Print executable size
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
    COMMENT "Calculating size of ${PROJECT_NAME}.elf"
)

# Create hex file
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Creating hex file ${PROJECT_NAME}.hex"
)

# Create binary file
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMENT "Creating binary file ${PROJECT_NAME}.bin"
)

# Create extended listing
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJDUMP} -h -S ${PROJECT_NAME}.elf > ${PROJECT_NAME}.lst
    COMMENT "Creating extended listing ${PROJECT_NAME}.lst"
)
