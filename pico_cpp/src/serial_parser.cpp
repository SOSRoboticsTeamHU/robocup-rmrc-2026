/**
 * Serial Parser Implementation v2.0
 * ==================================
 * Robust parsing with STOP command support.
 * 
 * Commands:
 *   "left,right\n"  - Set motor speeds (-100 to 100)
 *   "STOP\n"        - Emergency stop
 */

#include "pico_drive.hpp"
#include <cstring>
#include <cstdlib>

namespace pico_drive {

ParseResult SerialParser::process(char c) {
    // Handle line endings
    if (c == '\n' || c == '\r') {
        if (pos_ > 0) {
            buffer_[pos_] = '\0';
            
            // Check for STOP command
            if (strcmp(buffer_, "STOP") == 0 || strcmp(buffer_, "stop") == 0) {
                reset();
                return ParseResult::VALID_STOP;
            }
            
            // Try to parse as drive command
            if (parseCommand()) {
                reset();
                return ParseResult::VALID_DRIVE;
            }
            
            // Parse failed
            reset();
            return ParseResult::ERROR;
        }
        return ParseResult::INCOMPLETE;
    }
    
    // Ignore carriage return mid-buffer
    if (c == '\r') {
        return ParseResult::INCOMPLETE;
    }
    
    // Add character to buffer (with overflow protection)
    if (pos_ < SERIAL_BUFFER_SIZE - 1) {
        buffer_[pos_++] = c;
    } else {
        // Buffer overflow - reset
        reset();
        return ParseResult::ERROR;
    }
    
    return ParseResult::INCOMPLETE;
}

void SerialParser::reset() {
    pos_ = 0;
    buffer_[0] = '\0';
}

bool SerialParser::parseCommand() {
    // Format: "left,right" where left and right are integers -100 to 100
    // Example: "50,-50" or "-100,100" or "0,0"
    
    // Find comma separator
    char* comma = strchr(buffer_, ',');
    if (!comma) {
        return false;
    }
    
    // Split at comma
    *comma = '\0';
    
    // Parse left value
    char* endptr;
    long left_val = strtol(buffer_, &endptr, 10);
    if (endptr == buffer_ || *endptr != '\0') {
        return false;  // Invalid left value
    }
    
    // Parse right value
    long right_val = strtol(comma + 1, &endptr, 10);
    if (endptr == comma + 1) {
        return false;  // Invalid right value
    }
    // Allow trailing whitespace
    while (*endptr == ' ' || *endptr == '\t') endptr++;
    if (*endptr != '\0') {
        return false;  // Garbage after number
    }
    
    // Clamp values to valid range
    if (left_val > MAX_SPEED) left_val = MAX_SPEED;
    if (left_val < MIN_SPEED) left_val = MIN_SPEED;
    if (right_val > MAX_SPEED) right_val = MAX_SPEED;
    if (right_val < MIN_SPEED) right_val = MIN_SPEED;
    
    left_ = static_cast<int16_t>(left_val);
    right_ = static_cast<int16_t>(right_val);
    
    return true;
}

} // namespace pico_drive
