#!/bin/bash
# Build script for Pico C++ Drive Controller
# 
# Prerequisites:
#   - Pico SDK installed
#   - PICO_SDK_PATH environment variable set
#   - arm-none-eabi-gcc toolchain installed
#
# On Ubuntu/Debian:
#   sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi
#   git clone https://github.com/raspberrypi/pico-sdk.git
#   export PICO_SDK_PATH=/path/to/pico-sdk

set -e

echo "============================================"
echo "Building Pico C++ Drive Controller"
echo "============================================"

# Check for PICO_SDK_PATH
if [ -z "$PICO_SDK_PATH" ]; then
    echo "ERROR: PICO_SDK_PATH not set!"
    echo ""
    echo "Please set PICO_SDK_PATH to the Pico SDK location:"
    echo "  export PICO_SDK_PATH=/path/to/pico-sdk"
    echo ""
    echo "Or install the SDK:"
    echo "  git clone https://github.com/raspberrypi/pico-sdk.git"
    echo "  cd pico-sdk && git submodule update --init"
    exit 1
fi

echo "Using PICO_SDK_PATH: $PICO_SDK_PATH"

# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

echo ""
echo "============================================"
echo "Build complete!"
echo ""
echo "Output files:"
echo "  - pico_drive.uf2  (drag-and-drop to Pico)"
echo "  - pico_drive.elf  (for debugging)"
echo ""
echo "To flash:"
echo "  1. Hold BOOTSEL button on Pico"
echo "  2. Connect USB"
echo "  3. Copy pico_drive.uf2 to RPI-RP2 drive"
echo "============================================"
