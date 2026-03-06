/**
 * RoboCupRescue RMRC 2026 - Pico Drive Controller v2.0
 * =====================================================
 * PRODUCTION-READY firmware for Raspberry Pi Pico.
 * 
 * Safety Features:
 * - Hardware watchdog (auto-reset if firmware hangs)
 * - Software watchdog (stop motors if no commands)
 * - Speed ramping (smooth acceleration/deceleration)
 * - Error recovery (USB disconnect handling)
 * - Graceful degradation
 * 
 * Protocol: "left,right\n" where values are -100 to 100
 * Special:  "STOP\n" for emergency stop
 * Response: "OK:left,right\n" on valid command
 * 
 * LED Status:
 * - Slow blink (1Hz): Waiting for commands
 * - Fast blink (5Hz): Active/running
 * - Solid ON: Error state
 * 
 * Hardware:
 * - GP6: Left PWM, GP7: Left DIR (Cytron MDD20A)
 * - GP8: Right PWM, GP9: Right DIR
 * - GP25: Status LED
 */

#pragma once

#include <cstdint>
#include "pico/types.h"

namespace pico_drive {

// =============================================================================
// CONFIGURATION - Easily tunable parameters
// =============================================================================

// Pin assignments (Cytron MDD20A)
constexpr uint PIN_LEFT_PWM   = 6;
constexpr uint PIN_LEFT_DIR   = 7;
constexpr uint PIN_RIGHT_PWM  = 8;
constexpr uint PIN_RIGHT_DIR  = 9;
constexpr uint PIN_LED        = 25;

// PWM Configuration
constexpr uint32_t PWM_FREQ_HZ = 20000;  // 20kHz - silent operation
constexpr uint16_t PWM_WRAP    = 4095;   // 12-bit resolution

// Timing
constexpr uint32_t WATCHDOG_TIMEOUT_MS   = 250;   // Stop motors if no command
constexpr uint32_t HW_WATCHDOG_TIMEOUT_MS = 2000; // Reset Pico if firmware hangs

// Ramping (smooth acceleration) - CONFIGURABLE
constexpr int16_t RAMP_RATE = 8;           // Speed change per update cycle
                                            // Higher = faster response, lower = smoother
constexpr bool ENABLE_RAMPING = true;       // Set false for direct control

// Speed limits
constexpr int16_t MAX_SPEED = 100;
constexpr int16_t MIN_SPEED = -100;

// Serial buffer
constexpr uint8_t SERIAL_BUFFER_SIZE = 32;

// =============================================================================
// MOTOR CLASS - Hardware PWM with optional ramping
// =============================================================================

class Motor {
public:
    Motor(uint pwm_pin, uint dir_pin);
    
    void init();
    
    // Set target speed (-100 to 100)
    void setTarget(int16_t speed);
    
    // Update motor (apply ramping if enabled)
    void update();
    
    // Immediate stop (bypass ramping)
    void emergencyStop();
    
    // Getters
    int16_t getTarget() const { return target_speed_; }
    int16_t getCurrent() const { return current_speed_; }
    bool isAtTarget() const { return current_speed_ == target_speed_; }
    
private:
    uint pwm_pin_;
    uint dir_pin_;
    uint pwm_slice_;
    uint pwm_channel_;
    
    int16_t target_speed_{0};
    int16_t current_speed_{0};
    
    void applyToHardware();
    static int16_t clamp(int value);
};

// =============================================================================
// SERIAL PARSER - Robust command parsing
// =============================================================================

enum class ParseResult {
    INCOMPLETE,     // Need more data
    VALID_DRIVE,    // Got valid "left,right" command
    VALID_STOP,     // Got "STOP" command
    ERROR           // Parse error (will reset)
};

class SerialParser {
public:
    SerialParser() { reset(); }
    
    // Process one character
    ParseResult process(char c);
    
    // Get parsed values (valid after VALID_DRIVE)
    int16_t getLeft() const { return left_; }
    int16_t getRight() const { return right_; }
    
    // Reset parser state
    void reset();
    
private:
    char buffer_[SERIAL_BUFFER_SIZE];
    uint8_t pos_{0};
    int16_t left_{0};
    int16_t right_{0};
    
    bool parseCommand();
};

// =============================================================================
// DRIVE CONTROLLER - Main controller with all safety features
// =============================================================================

class DriveController {
public:
    DriveController();
    
    // Initialize all hardware (call once at startup)
    void init();
    
    // Main loop update (call in tight loop)
    void update();
    
    // Set motor targets
    void setMotors(int16_t left, int16_t right);
    
    // Emergency stop (immediate)
    void emergencyStop();
    
    // State
    bool isActive() const { return !motors_stopped_; }
    uint32_t getUptimeMs() const;
    
private:
    Motor left_motor_;
    Motor right_motor_;
    SerialParser parser_;
    
    // Timing
    uint32_t boot_time_us_{0};
    uint32_t last_cmd_time_us_{0};
    uint32_t last_update_us_{0};
    bool motors_stopped_{true};
    
    // LED state
    uint32_t led_toggle_time_us_{0};
    bool led_state_{false};
    
    // Methods
    void processSerial();
    void updateMotors();
    void checkWatchdog();
    void updateLed();
    void sendResponse(int16_t left, int16_t right);
    void kickHardwareWatchdog();
};

} // namespace pico_drive
