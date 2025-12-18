#!/bin/bash
###############################################################################
# Build script for STM32 PumpBoard Controller CMake project
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Project root directory
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"

# Function to print colored output
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if ARM toolchain is installed
check_toolchain() {
    print_info "Checking for ARM GCC toolchain..."
    if ! command -v arm-none-eabi-gcc &> /dev/null; then
        print_error "arm-none-eabi-gcc not found!"
        print_error "Please install the ARM GCC toolchain:"
        print_error "  Ubuntu/Debian: sudo apt-get install gcc-arm-none-eabi"
        print_error "  Or download from: https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm"
        exit 1
    fi
    print_info "ARM GCC version: $(arm-none-eabi-gcc --version | head -n1)"
}

# Check if CMake is installed
check_cmake() {
    print_info "Checking for CMake..."
    if ! command -v cmake &> /dev/null; then
        print_error "cmake not found!"
        print_error "Please install CMake: sudo apt-get install cmake"
        exit 1
    fi
    print_info "CMake version: $(cmake --version | head -n1)"
}

# Clean build directory
clean_build() {
    print_info "Cleaning build directory..."
    if [ -d "$BUILD_DIR" ]; then
        rm -rf "$BUILD_DIR"
        print_info "Build directory cleaned"
    else
        print_warn "Build directory does not exist, nothing to clean"
    fi
}

# Configure CMake
configure() {
    print_info "Configuring CMake..."
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"

    if command -v ninja &> /dev/null; then
        print_info "Using Ninja build system"
        cmake -DCMAKE_TOOLCHAIN_FILE="${PROJECT_ROOT}/arm-none-eabi-gcc.cmake" \
              -DCMAKE_BUILD_TYPE=Debug \
              -G Ninja \
              "$PROJECT_ROOT"
    else
        print_info "Using Make build system"
        cmake -DCMAKE_TOOLCHAIN_FILE="${PROJECT_ROOT}/arm-none-eabi-gcc.cmake" \
              -DCMAKE_BUILD_TYPE=Debug \
              "$PROJECT_ROOT"
    fi

    print_info "Configuration complete"
}

# Build project
build() {
    print_info "Building project..."
    cd "$BUILD_DIR"

    if [ -f "build.ninja" ]; then
        ninja
    else
        make -j$(nproc)
    fi

    print_info "Build complete!"
}

# Display build artifacts
show_artifacts() {
    print_info "Build artifacts:"
    echo ""
    if [ -f "$BUILD_DIR/PumpCtrl.elf" ]; then
        ls -lh "$BUILD_DIR"/PumpCtrl.* | awk '{print "  " $9 " (" $5 ")"}'
        echo ""
        print_info "Memory usage:"
        arm-none-eabi-size "$BUILD_DIR/PumpCtrl.elf"
    else
        print_warn "No build artifacts found"
    fi
}

# Flash firmware (optional)
flash() {
    print_info "Flashing firmware..."

    if command -v st-flash &> /dev/null; then
        st-flash write "$BUILD_DIR/PumpCtrl.bin" 0x08000000
    elif command -v openocd &> /dev/null; then
        openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
            -c "program $BUILD_DIR/PumpCtrl.bin 0x08000000 verify reset exit"
    else
        print_error "Neither st-flash nor openocd found!"
        print_error "Please install one of them to flash the firmware"
        exit 1
    fi

    print_info "Flashing complete!"
}

# Print usage
usage() {
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  build          Configure and build the project (default)"
    echo "  clean          Clean the build directory"
    echo "  rebuild        Clean and rebuild the project"
    echo "  flash          Flash the firmware to the device"
    echo "  all            Build and flash the firmware"
    echo "  help           Display this help message"
    echo ""
}

# Main script
main() {
    cd "$PROJECT_ROOT"

    case "${1:-build}" in
        build)
            check_toolchain
            check_cmake
            if [ ! -d "$BUILD_DIR" ]; then
                configure
            fi
            build
            show_artifacts
            ;;
        clean)
            clean_build
            ;;
        rebuild)
            check_toolchain
            check_cmake
            clean_build
            configure
            build
            show_artifacts
            ;;
        flash)
            if [ ! -f "$BUILD_DIR/PumpCtrl.bin" ]; then
                print_error "No binary file found. Please build first."
                exit 1
            fi
            flash
            ;;
        all)
            check_toolchain
            check_cmake
            if [ ! -d "$BUILD_DIR" ]; then
                configure
            fi
            build
            show_artifacts
            flash
            ;;
        help|--help|-h)
            usage
            ;;
        *)
            print_error "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
}

main "$@"
