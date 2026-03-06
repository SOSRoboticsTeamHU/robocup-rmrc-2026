/**
 * ZMQ Receiver Implementation
 * CONFLATE mode ensures we always get the latest message.
 */

#include "drive_bridge.hpp"

#include <zmq.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace drive_bridge {

ZmqReceiver::~ZmqReceiver() {
    close();
}

bool ZmqReceiver::bind(int port) {
    // Create context
    context_ = zmq_ctx_new();
    if (!context_) {
        std::fprintf(stderr, "[ZMQ] Failed to create context\n");
        return false;
    }
    
    // Create SUB socket
    socket_ = zmq_socket(context_, ZMQ_SUB);
    if (!socket_) {
        std::fprintf(stderr, "[ZMQ] Failed to create socket\n");
        zmq_ctx_destroy(context_);
        context_ = nullptr;
        return false;
    }
    
    // Subscribe to all messages
    zmq_setsockopt(socket_, ZMQ_SUBSCRIBE, "", 0);
    
    // CONFLATE = keep only latest message (critical for low latency!)
    int conflate = 1;
    zmq_setsockopt(socket_, ZMQ_CONFLATE, &conflate, sizeof(conflate));
    
    // Small receive buffer
    int rcvhwm = 1;
    zmq_setsockopt(socket_, ZMQ_RCVHWM, &rcvhwm, sizeof(rcvhwm));
    
    // Receive timeout 5ms
    int timeout = 5;
    zmq_setsockopt(socket_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    
    // Bind to port
    char addr[64];
    std::snprintf(addr, sizeof(addr), "tcp://0.0.0.0:%d", port);
    
    if (zmq_bind(socket_, addr) != 0) {
        std::fprintf(stderr, "[ZMQ] Failed to bind to %s: %s\n", addr, zmq_strerror(errno));
        zmq_close(socket_);
        zmq_ctx_destroy(context_);
        socket_ = nullptr;
        context_ = nullptr;
        return false;
    }
    
    std::printf("[ZMQ] Listening on port %d (CONFLATE=1)\n", port);
    return true;
}

void ZmqReceiver::close() {
    if (socket_) {
        zmq_close(socket_);
        socket_ = nullptr;
    }
    if (context_) {
        zmq_ctx_destroy(context_);
        context_ = nullptr;
    }
}

bool ZmqReceiver::receive(DriveCommand& cmd) {
    if (!socket_) return false;
    
    // Pre-allocated buffer for JSON parsing
    char buf[256];
    
    int len = zmq_recv(socket_, buf, sizeof(buf) - 1, ZMQ_DONTWAIT);
    if (len <= 0) {
        return false;  // No message or error
    }
    
    buf[len] = '\0';
    
    // Fast JSON parsing - we only care about y, x, z
    // Format: {"y": 50, "x": 10, "z": 0}
    // Use simple sscanf-style parsing to avoid JSON library overhead
    
    // Find "y":
    const char* py = std::strstr(buf, "\"y\"");
    if (py) {
        py = std::strchr(py, ':');
        if (py) cmd.y = static_cast<int16_t>(std::atoi(py + 1));
    }
    
    // Find "x":
    const char* px = std::strstr(buf, "\"x\"");
    if (px) {
        px = std::strchr(px, ':');
        if (px) cmd.x = static_cast<int16_t>(std::atoi(px + 1));
    }
    
    // Find "z":
    const char* pz = std::strstr(buf, "\"z\"");
    if (pz) {
        pz = std::strchr(pz, ':');
        if (pz) cmd.z = static_cast<int16_t>(std::atoi(pz + 1));
    }
    
    return true;
}

} // namespace drive_bridge
