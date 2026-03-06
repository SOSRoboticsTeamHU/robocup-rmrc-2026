/**
 * Drive Mixer Implementation
 * Converts arcade-style input (Y, X, Z) to tank drive (left, right).
 *
 * Formula: left = Y + X + Z, right = Y - X - Z
 *   Y = forward/backward
 *   X = curved turning / left-right (íves kanyar)
 *   Z = spot rotation (egyhelyben forgás)
 */

#include "drive_bridge.hpp"

#include <algorithm>
#include <cmath>

namespace drive_bridge {

int16_t DriveMixer::clamp(int value, int min_val, int max_val) {
    if (value < min_val) return static_cast<int16_t>(min_val);
    if (value > max_val) return static_cast<int16_t>(max_val);
    return static_cast<int16_t>(value);
}

MotorSpeeds DriveMixer::mix(const DriveCommand& cmd) {
    MotorSpeeds speeds;
    int left  = cmd.y + cmd.x + cmd.z;
    int right = cmd.y - cmd.x - cmd.z;
    
    // Scale if exceeds limits (preserve ratio)
    int max_val = std::max(std::abs(left), std::abs(right));
    if (max_val > 100) {
        left  = left  * 100 / max_val;
        right = right * 100 / max_val;
    }
    
    speeds.left  = clamp(left, -100, 100);
    speeds.right = clamp(right, -100, 100);
    
    return speeds;
}

} // namespace drive_bridge
