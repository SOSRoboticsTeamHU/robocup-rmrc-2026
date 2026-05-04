#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Robot Status Publisher
Publishes robot status to laptop GUI via ZMQ
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
try:
    from shared.constants import ZMQ_PORT_STATUS, PICO_SERIAL_PORT
except ImportError:
    ZMQ_PORT_STATUS = 5559
    PICO_SERIAL_PORT = "/dev/ttyACM0"

import zmq
import json
import time
import serial
import threading

ZMQ_PORT = ZMQ_PORT_STATUS
PICO_PORT = PICO_SERIAL_PORT
PUBLISH_RATE = 10  # Hz


class StatusPublisher:
    """Publishes robot status via ZMQ."""
    
    def __init__(self, port: int = ZMQ_PORT):
        self.port = port
        self.running = False
        
        # ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SNDBUF, 64 * 1024)
        
        # Status data
        self.status = {
            "pico_connected": False,
            "arm_connected": False,
            "lidar_connected": False,
            "cameras_active": 0,
            "left_speed": 0,
            "right_speed": 0,
            "cpu_temp": 0,
            "gpu_temp": 0,
            "uptime": 0,
            "timestamp": 0,
            "heartbeat_seq": 0,
        }
        
        self.start_time = time.time()
        self.lock = threading.Lock()
    
    def update_pico_status(self, connected: bool, left: int = 0, right: int = 0):
        """Update Pico/motor status."""
        with self.lock:
            self.status["pico_connected"] = connected
            self.status["left_speed"] = left
            self.status["right_speed"] = right
    
    def update_arm_status(self, connected: bool):
        """Update arm status."""
        with self.lock:
            self.status["arm_connected"] = connected
    
    def update_lidar_status(self, connected: bool):
        """Update lidar status."""
        with self.lock:
            self.status["lidar_connected"] = connected
    
    def update_cameras(self, count: int):
        """Update active camera count."""
        with self.lock:
            self.status["cameras_active"] = count
    
    def _get_cpu_temp(self) -> float:
        """Read Jetson CPU temperature."""
        try:
            with open("/sys/devices/virtual/thermal/thermal_zone0/temp", "r") as f:
                return int(f.read().strip()) / 1000.0
        except Exception:
            return 0.0

    def _get_gpu_temp(self) -> float:
        """Read Jetson GPU temperature."""
        try:
            with open("/sys/devices/virtual/thermal/thermal_zone1/temp", "r") as f:
                return int(f.read().strip()) / 1000.0
        except Exception:
            return 0.0
    
    def _check_pico(self) -> bool:
        """Quick check if Pico is responsive."""
        try:
            import os
            return os.path.exists(PICO_PORT)
        except Exception:
            return False
    
    def start(self):
        """Start publishing status."""
        self.socket.bind(f"tcp://*:{self.port}")
        print(f"[STATUS] Publishing on port {self.port}")
        
        self.running = True
        interval = 1.0 / PUBLISH_RATE
        
        try:
            while self.running:
                loop_start = time.time()
                
                # Update system stats
                with self.lock:
                    self.status["cpu_temp"] = round(self._get_cpu_temp(), 1)
                    self.status["gpu_temp"] = round(self._get_gpu_temp(), 1)
                    self.status["uptime"] = round(time.time() - self.start_time, 1)
                    self.status["timestamp"] = time.time()
                    self.status["pico_connected"] = self._check_pico()
                    self.status["heartbeat_seq"] += 1
                    
                    # Send status (NOBLOCK can raise zmq.Again if would block)
                    try:
                        self.socket.send_json(self.status, zmq.NOBLOCK)
                    except zmq.Again:
                        pass
                
                # Rate limiting
                elapsed = time.time() - loop_start
                if interval > elapsed:
                    time.sleep(interval - elapsed)
                    
        except KeyboardInterrupt:
            print("\n[STATUS] Interrupted")
        finally:
            self.stop()
    
    def stop(self):
        """Stop publishing."""
        self.running = False
        self.socket.close()
        self.context.term()
        print("[STATUS] Stopped")


def main():
    publisher = StatusPublisher()
    publisher.start()


if __name__ == "__main__":
    main()
