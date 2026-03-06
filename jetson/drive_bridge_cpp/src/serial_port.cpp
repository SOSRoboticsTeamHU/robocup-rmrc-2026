/**
 * Serial Port Implementation - Linux termios
 * Non-blocking, minimal latency serial communication.
 */

#include "drive_bridge.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cstdio>

namespace drive_bridge {

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, int baud) {
    // Open port
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::fprintf(stderr, "[SERIAL] Failed to open %s\n", port.c_str());
        return false;
    }
    
    // Configure port
    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(fd_, &tty) != 0) {
        std::fprintf(stderr, "[SERIAL] tcgetattr failed\n");
        close();
        return false;
    }
    
    // Baud rate
    speed_t speed;
    switch (baud) {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        default:     speed = B115200; break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    
    // 8N1, no flow control
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem
    
    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    
    // Non-blocking read
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::fprintf(stderr, "[SERIAL] tcsetattr failed\n");
        close();
        return false;
    }
    
    // Flush buffers
    tcflush(fd_, TCIOFLUSH);
    
    std::printf("[SERIAL] Opened %s @ %d baud\n", port.c_str(), baud);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        // Send stop before closing
        sendStop();
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::sendMotorCommand(int16_t left, int16_t right) {
    if (fd_ < 0) return false;
    
    // Format: "left,right\n" - no dynamic allocation
    int len = std::snprintf(write_buf_, sizeof(write_buf_), "%d,%d\n", left, right);
    
    if (len <= 0 || len >= static_cast<int>(sizeof(write_buf_))) {
        return false;
    }
    
    ssize_t written = ::write(fd_, write_buf_, len);
    return written == len;
}

bool SerialPort::sendStop() {
    return sendMotorCommand(0, 0);
}

} // namespace drive_bridge
