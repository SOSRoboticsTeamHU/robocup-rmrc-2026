cmake_minimum_required(VERSION 3.16)
project(drive_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Optimization flags
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

# Find packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# Main executable
add_executable(drive_bridge
    src/main.cpp
    src/serial_port.cpp
    src/zmq_receiver.cpp
    src/drive_mixer.cpp
)

target_include_directories(drive_bridge PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${ZMQ_INCLUDE_DIRS}
)

target_link_libraries(drive_bridge
    ${ZMQ_LIBRARIES}
    pthread
)

# Install
install(TARGETS drive_bridge DESTINATION bin)
