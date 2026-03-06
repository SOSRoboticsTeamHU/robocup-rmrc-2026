cmake_minimum_required(VERSION 3.13)

# Initialize Pico SDK (must be before project())
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_drive_controller C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Main executable
add_executable(pico_drive
    src/main.cpp
    src/motor.cpp
    src/serial_parser.cpp
)

# Link libraries
target_link_libraries(pico_drive
    pico_stdlib
    hardware_pwm
    hardware_gpio
)

# Enable USB serial (CDC)
pico_enable_stdio_usb(pico_drive 1)
pico_enable_stdio_uart(pico_drive 0)

# Create UF2 file for drag-and-drop upload
pico_add_extra_outputs(pico_drive)

# Print memory usage
target_compile_options(pico_drive PRIVATE -Os -Wall -Wextra)
