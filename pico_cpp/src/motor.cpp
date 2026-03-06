/**
 * Motor Implementation v2.0 - Hardware PWM with Ramping
 * ======================================================
 * Uses RP2040 hardware PWM for precise, jitter-free motor control.
 * Optional ramping for smooth acceleration/deceleration.
 */

#include "pico_drive.hpp"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

namespace pico_drive {

Motor::Motor(uint pwm_pin, uint dir_pin)
    : pwm_pin_(pwm_pin)
    , dir_pin_(dir_pin)
    , pwm_slice_(pwm_gpio_to_slice_num(pwm_pin))
    , pwm_channel_(pwm_gpio_to_channel(pwm_pin))
{
}

void Motor::init() {
    // Configure direction pin as output
    gpio_init(dir_pin_);
    gpio_set_dir(dir_pin_, GPIO_OUT);
    gpio_put(dir_pin_, 0);
    
    // Configure PWM pin
    gpio_set_function(pwm_pin_, GPIO_FUNC_PWM);
    
    // Calculate clock divider for desired frequency
    uint32_t clock_hz = clock_get_hz(clk_sys);
    float divider = (float)clock_hz / (float)(PWM_WRAP + 1) / (float)PWM_FREQ_HZ;
    
    pwm_set_clkdiv(pwm_slice_, divider);
    pwm_set_wrap(pwm_slice_, PWM_WRAP);
    pwm_set_chan_level(pwm_slice_, pwm_channel_, 0);
    pwm_set_enabled(pwm_slice_, true);
    
    target_speed_ = 0;
    current_speed_ = 0;
}

int16_t Motor::clamp(int value) {
    if (value > MAX_SPEED) return MAX_SPEED;
    if (value < MIN_SPEED) return MIN_SPEED;
    return static_cast<int16_t>(value);
}

void Motor::setTarget(int16_t speed) {
    target_speed_ = clamp(speed);
    
    // If ramping disabled, apply immediately
    if (!ENABLE_RAMPING) {
        current_speed_ = target_speed_;
        applyToHardware();
    }
}

void Motor::update() {
    if (!ENABLE_RAMPING) return;  // Already applied in setTarget
    
    // Ramp current speed toward target
    if (current_speed_ < target_speed_) {
        current_speed_ = clamp(current_speed_ + RAMP_RATE);
        if (current_speed_ > target_speed_) {
            current_speed_ = target_speed_;
        }
    } else if (current_speed_ > target_speed_) {
        current_speed_ = clamp(current_speed_ - RAMP_RATE);
        if (current_speed_ < target_speed_) {
            current_speed_ = target_speed_;
        }
    }
    
    applyToHardware();
}

void Motor::emergencyStop() {
    target_speed_ = 0;
    current_speed_ = 0;
    gpio_put(dir_pin_, 0);
    pwm_set_chan_level(pwm_slice_, pwm_channel_, 0);
}

void Motor::applyToHardware() {
    int16_t speed = current_speed_;
    
    // Set direction
    if (speed >= 0) {
        gpio_put(dir_pin_, 0);  // Forward
    } else {
        gpio_put(dir_pin_, 1);  // Reverse
        speed = -speed;
    }
    
    // Calculate PWM duty cycle (0-100 maps to 0-PWM_WRAP)
    uint16_t duty = static_cast<uint16_t>(
        static_cast<uint32_t>(speed) * PWM_WRAP / 100
    );
    
    pwm_set_chan_level(pwm_slice_, pwm_channel_, duty);
}

} // namespace pico_drive
