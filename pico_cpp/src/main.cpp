/**
 * RoboCupRescue RMRC 2026 - Pico Drive Controller v2.0
 * =====================================================
 * PRODUCTION-READY firmware for Raspberry Pi Pico.
 * 
 * Protocol: "left,right\n" where values are -100 to 100
 * Special:  "STOP\n" for emergency stop
 * Response: "OK:left,right\n" on valid command
 * 
 * Safety Features:
 * - Hardware watchdog (resets Pico if firmware hangs)
 * - Software watchdog (stops motors if no commands)
 * - Ramping (smooth acceleration/deceleration)
 * - USB disconnect recovery
 * 
 * Hardware:
 * - GP6: Left PWM, GP7: Left DIR
 * - GP8: Right PWM, GP9: Right DIR
 * - GP25: Status LED
 */

#include "pico_drive.hpp"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include <cstdio>

namespace pico_drive {

DriveController::DriveController()
    : left_motor_(PIN_LEFT_PWM, PIN_LEFT_DIR)
    , right_motor_(PIN_RIGHT_PWM, PIN_RIGHT_DIR)
{
}

void DriveController::init() {
    // Record boot time
    boot_time_us_ = time_us_32();
    
    // Initialize stdio for USB serial
    stdio_init_all();
    
    // Wait for USB connection (with timeout - don't block forever!)
    for (int i = 0; i < 30; i++) {
        if (stdio_usb_connected()) break;
        sleep_ms(100);
    }
    
    // Initialize LED
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);  // LED on during init
    
    // Initialize motors
    left_motor_.init();
    right_motor_.init();
    
    // Enable hardware watchdog
    watchdog_enable(HW_WATCHDOG_TIMEOUT_MS, true);
    
    // Initialize timing
    last_cmd_time_us_ = time_us_32();
    last_update_us_ = time_us_32();
    led_toggle_time_us_ = time_us_32();
    
    // Startup message
    printf("\n");
    printf("==========================================\n");
    printf("PICO_DRIVE_CPP v2.0 READY\n");
    printf("==========================================\n");
    printf("Protocol: left,right (e.g., 50,-50)\n");
    printf("Special:  STOP (emergency stop)\n");
    printf("Watchdog: %lu ms\n", WATCHDOG_TIMEOUT_MS);
    printf("Ramping:  %s (rate=%d)\n", 
           ENABLE_RAMPING ? "ON" : "OFF", RAMP_RATE);
    printf("==========================================\n");
    
    gpio_put(PIN_LED, 0);  // LED off, ready
}

void DriveController::update() {
    // Kick hardware watchdog (prevents reset if firmware hangs)
    kickHardwareWatchdog();
    
    // Process serial commands
    processSerial();
    
    // Update motors (applies ramping)
    updateMotors();
    
    // Check software watchdog
    checkWatchdog();
    
    // Update LED status
    updateLed();
}

void DriveController::setMotors(int16_t left, int16_t right) {
    left_motor_.setTarget(left);
    right_motor_.setTarget(right);
    last_cmd_time_us_ = time_us_32();
    motors_stopped_ = (left == 0 && right == 0);
}

void DriveController::emergencyStop() {
    left_motor_.emergencyStop();
    right_motor_.emergencyStop();
    motors_stopped_ = true;
    printf("OK:STOPPED\n");
}

uint32_t DriveController::getUptimeMs() const {
    return (time_us_32() - boot_time_us_) / 1000;
}

void DriveController::processSerial() {
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        ParseResult result = parser_.process(static_cast<char>(c));
        
        switch (result) {
            case ParseResult::VALID_DRIVE:
                setMotors(parser_.getLeft(), parser_.getRight());
                sendResponse(parser_.getLeft(), parser_.getRight());
                break;
                
            case ParseResult::VALID_STOP:
                emergencyStop();
                break;
                
            case ParseResult::ERROR:
                printf("ERR:PARSE\n");
                break;
                
            case ParseResult::INCOMPLETE:
                // Need more data
                break;
        }
    }
}

void DriveController::updateMotors() {
    // Update motors at consistent rate (for ramping)
    uint32_t now = time_us_32();
    if (now - last_update_us_ >= 10000) {  // Every 10ms
        left_motor_.update();
        right_motor_.update();
        last_update_us_ = now;
    }
}

void DriveController::checkWatchdog() {
    uint32_t now = time_us_32();
    uint32_t elapsed_ms = (now - last_cmd_time_us_) / 1000;
    
    if (elapsed_ms > WATCHDOG_TIMEOUT_MS && !motors_stopped_) {
        // No commands received - stop motors for safety
        left_motor_.setTarget(0);
        right_motor_.setTarget(0);
        
        // If ramping disabled, stop immediately
        if (!ENABLE_RAMPING) {
            left_motor_.emergencyStop();
            right_motor_.emergencyStop();
        }
        
        motors_stopped_ = true;
    }
}

void DriveController::updateLed() {
    uint32_t now = time_us_32();
    uint32_t blink_period;
    
    if (motors_stopped_) {
        blink_period = 500000;  // 1Hz - idle
    } else {
        blink_period = 100000;  // 5Hz - active
    }
    
    if (now - led_toggle_time_us_ >= blink_period) {
        led_state_ = !led_state_;
        gpio_put(PIN_LED, led_state_);
        led_toggle_time_us_ = now;
    }
}

void DriveController::sendResponse(int16_t left, int16_t right) {
    printf("OK:%d,%d\n", left, right);
}

void DriveController::kickHardwareWatchdog() {
    watchdog_update();
}

} // namespace pico_drive


// =============================================================================
// MAIN
// =============================================================================

int main() {
    pico_drive::DriveController controller;
    controller.init();
    
    // Main loop - runs continuously
    while (true) {
        controller.update();
        
        // Small sleep to prevent tight spinning (saves power)
        sleep_us(100);  // 10kHz loop
    }
    
    return 0;
}
