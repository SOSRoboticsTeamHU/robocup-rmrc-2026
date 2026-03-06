/**
 * RoboCupRescue RMRC 2026 - C++ Drive Bridge
 * ==========================================
 * Industry-grade, zero-latency motor control bridge.
 * 
 * Usage:
 *   ./drive_bridge [--port /dev/ttyACM0] [--zmq 5555] [--debug]
 * 
 * Performance:
 *   - 200Hz main loop (5ms)
 *   - <1ms ZMQ receive + serial write
 *   - No heap allocations in hot path
 *   - CONFLATE socket = always latest command
 */

#include "drive_bridge.hpp"

#include <csignal>
#include <cstdio>
#include <cstring>
#include <thread>
#include <getopt.h>

namespace drive_bridge {

// Global bridge pointer for signal handler
static Bridge* g_bridge = nullptr;

void signalHandler(int sig) {
    (void)sig;
    if (g_bridge) {
        g_bridge->stop();
    }
}

Bridge::~Bridge() {
    stop();
}

bool Bridge::init(const std::string& serial_port, int zmq_port) {
    std::printf("============================================================\n");
    std::printf("DRIVE BRIDGE C++ - ZERO LATENCY MOTOR CONTROL\n");
    std::printf("============================================================\n");
    
    // Open serial port
    if (!serial_.open(serial_port)) {
        std::fprintf(stderr, "[BRIDGE] WARNING: Serial not connected\n");
        // Continue anyway - might connect later
    }
    
    // Bind ZMQ socket
    if (!zmq_.bind(zmq_port)) {
        std::fprintf(stderr, "[BRIDGE] FATAL: ZMQ bind failed\n");
        return false;
    }
    
    last_cmd_time_ = std::chrono::steady_clock::now();
    motors_stopped_ = true;
    
    std::printf("[BRIDGE] Initialized (watchdog: %dms, loop: %dHz)\n", 
                WATCHDOG_MS, LOOP_HZ);
    
    return true;
}

void Bridge::run() {
    running_ = true;
    g_bridge = this;
    
    // Install signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::printf("[BRIDGE] Running... Press Ctrl+C to stop\n");
    
    constexpr auto loop_period = std::chrono::microseconds(1000000 / LOOP_HZ);
    
    DriveCommand cmd{};
    
    while (running_) {
        auto loop_start = std::chrono::steady_clock::now();
        
        // Receive command from ZMQ (non-blocking)
        if (zmq_.receive(cmd)) {
            // Mix to motor speeds
            auto speeds = DriveMixer::mix(cmd);
            
            // Send only if changed
            if (speeds.left != last_speeds_.left || speeds.right != last_speeds_.right) {
                if (serial_.sendMotorCommand(speeds.left, speeds.right)) {
                    last_speeds_ = speeds;
                    motors_stopped_ = (speeds.left == 0 && speeds.right == 0);
                } else {
                    error_count_++;
                }
            }
            
            last_cmd_time_ = loop_start;
            cmd_count_++;
        }
        
        // Watchdog check
        checkWatchdog();
        
        // Maintain loop rate
        auto elapsed = std::chrono::steady_clock::now() - loop_start;
        if (elapsed < loop_period) {
            std::this_thread::sleep_for(loop_period - elapsed);
        }
    }
    
    // Final stop
    serial_.sendStop();
    std::printf("[BRIDGE] Stopped. Commands: %lu, Errors: %lu\n", 
                cmd_count_, error_count_);
}

void Bridge::stop() {
    running_ = false;
}

void Bridge::checkWatchdog() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_cmd_time_).count();
    
    if (elapsed > WATCHDOG_MS && !motors_stopped_) {
        std::printf("[WATCHDOG] No command for %ldms - STOP\n", elapsed);
        serial_.sendStop();
        last_speeds_ = {0, 0};
        motors_stopped_ = true;
    }
}

} // namespace drive_bridge


// =============================================================================
// MAIN
// =============================================================================

void printUsage(const char* prog) {
    std::printf("Usage: %s [options]\n", prog);
    std::printf("Options:\n");
    std::printf("  -p, --port PORT    Serial port (default: /dev/ttyACM0)\n");
    std::printf("  -z, --zmq PORT     ZMQ port (default: 5555)\n");
    std::printf("  -d, --debug        Enable debug output\n");
    std::printf("  -h, --help         Show this help\n");
}

int main(int argc, char* argv[]) {
    std::string serial_port = drive_bridge::DEFAULT_SERIAL_PORT;
    int zmq_port = drive_bridge::ZMQ_PORT;
    bool debug = false;
    
    // Parse arguments
    static struct option long_options[] = {
        {"port",  required_argument, 0, 'p'},
        {"zmq",   required_argument, 0, 'z'},
        {"debug", no_argument,       0, 'd'},
        {"help",  no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };
    
    int opt;
    while ((opt = getopt_long(argc, argv, "p:z:dh", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'p':
                serial_port = optarg;
                break;
            case 'z':
                zmq_port = std::atoi(optarg);
                break;
            case 'd':
                debug = true;
                break;
            case 'h':
            default:
                printUsage(argv[0]);
                return opt == 'h' ? 0 : 1;
        }
    }
    
    (void)debug;  // TODO: Add debug logging
    
    // Create and run bridge
    drive_bridge::Bridge bridge;
    
    if (!bridge.init(serial_port, zmq_port)) {
        return 1;
    }
    
    bridge.run();
    
    return 0;
}
