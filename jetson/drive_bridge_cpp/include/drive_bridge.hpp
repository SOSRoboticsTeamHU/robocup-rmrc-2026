/**
 * RoboCupRescue RMRC 2026 - C++ Drive Bridge
 * ==========================================
 * Zero-latency ZMQ -> Serial bridge for Pico motor controller.
 * 
 * Performance targets:
 * - <1ms end-to-end latency
 * - No memory allocations in hot path
 * - Lock-free where possible
 */

#pragma once

#include <string>
#include <atomic>
#include <cstdint>
#include <chrono>

namespace drive_bridge {

// =============================================================================
// CONFIGURATION
// =============================================================================

constexpr const char* DEFAULT_SERIAL_PORT = "/dev/ttyACM0";
constexpr int SERIAL_BAUD = 115200;
constexpr int ZMQ_PORT = 5555;
constexpr int WATCHDOG_MS = 150;
constexpr int LOOP_HZ = 200;  // 5ms loop

// =============================================================================
// DRIVE COMMAND
// =============================================================================

struct DriveCommand {
    int16_t y{0};   // Forward/back (-100 to 100)
    int16_t x{0};   // Curve turn left/right (-100 to 100)
    int16_t z{0};   // Spot rotate left/right (-100 to 100)
    
    bool operator==(const DriveCommand& other) const {
        return y == other.y && x == other.x && z == other.z;
    }
    bool operator!=(const DriveCommand& other) const {
        return !(*this == other);
    }
};

struct MotorSpeeds {
    int16_t left{0};
    int16_t right{0};
};

// =============================================================================
// SERIAL PORT - Non-blocking, minimal overhead
// =============================================================================

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    
    bool open(const std::string& port, int baud = SERIAL_BAUD);
    void close();
    bool isOpen() const { return fd_ >= 0; }
    
    // Send "left,right\n" - returns true if successful
    bool sendMotorCommand(int16_t left, int16_t right);
    
    // Send stop command
    bool sendStop();
    
private:
    int fd_{-1};
    char write_buf_[32];  // Pre-allocated buffer
};

// =============================================================================
// ZMQ RECEIVER - CONFLATE mode for latest-only
// =============================================================================

class ZmqReceiver {
public:
    ZmqReceiver() = default;
    ~ZmqReceiver();
    
    bool bind(int port = ZMQ_PORT);
    void close();
    
    // Non-blocking receive, returns true if new command received
    bool receive(DriveCommand& cmd);
    
private:
    void* context_{nullptr};
    void* socket_{nullptr};
};

// =============================================================================
// DRIVE MIXER - Arcade to tank conversion
// =============================================================================

class DriveMixer {
public:
    // Convert joystick input to motor speeds
    static MotorSpeeds mix(const DriveCommand& cmd);
    
    // Clamp value to range
    static int16_t clamp(int value, int min_val = -100, int max_val = 100);
};

// =============================================================================
// MAIN BRIDGE CLASS
// =============================================================================

class Bridge {
public:
    Bridge() = default;
    ~Bridge();
    
    bool init(const std::string& serial_port = DEFAULT_SERIAL_PORT, 
              int zmq_port = ZMQ_PORT);
    void run();
    void stop();
    
    // Statistics
    uint64_t getCmdCount() const { return cmd_count_; }
    uint64_t getErrorCount() const { return error_count_; }
    
private:
    SerialPort serial_;
    ZmqReceiver zmq_;
    
    std::atomic<bool> running_{false};
    
    // Watchdog
    std::chrono::steady_clock::time_point last_cmd_time_;
    bool motors_stopped_{true};
    
    // Last command (for change detection)
    MotorSpeeds last_speeds_{0, 0};
    
    // Stats
    uint64_t cmd_count_{0};
    uint64_t error_count_{0};
    
    void checkWatchdog();
};

} // namespace drive_bridge
