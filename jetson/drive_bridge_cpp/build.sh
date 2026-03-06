#!/bin/bash
# Build script for C++ Drive Bridge
# Run on Jetson: ./build.sh

set -e

echo "============================================"
echo "Building C++ Drive Bridge"
echo "============================================"

# Install dependencies if needed
if ! pkg-config --exists libzmq; then
    echo "Installing libzmq-dev..."
    sudo apt-get update
    sudo apt-get install -y libzmq3-dev
fi

# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
make -j$(nproc)

echo ""
echo "============================================"
echo "Build complete!"
echo "Binary: $(pwd)/drive_bridge"
echo ""
echo "Usage:"
echo "  ./drive_bridge                    # Default settings"
echo "  ./drive_bridge -p /dev/ttyACM0    # Specify serial port"
echo "  ./drive_bridge -z 5555            # Specify ZMQ port"
echo "============================================"
