#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Drive Bridge Node
===========================================
Bridges drive commands from laptop (via ZMQ) to Pico motor controller (via USB serial).

This node:
1. Receives drive commands from laptop over ZMQ
2. Forwards them to the Pico via USB serial
3. Publishes drive status for monitoring

Can run standalone or as a ROS2 node.
"""

import json
import time
import threading
import serial
import zmq
from typing import Optional, Callable
import sys
import os

# Add shared module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'shared'))

try:
    from shared.constants import (
        JETSON_IP, LAPTOP_IP,
        ZMQ_PORT_DRIVE, ZMQ_PORT_STATUS,
        PICO_SERIAL_PORT, PICO_SERIAL_BAUD,
        WATCHDOG_TIMEOUT_MS, DRIVE_CONTROL_HZ,
        zmq_bind_str, zmq_connect_str
    )
    from shared.protocol import DriveCommand, DriveStatus, MessageType
except ImportError:
    # Fallback defaults if shared module not available
    JETSON_IP = "192.168.1.1"
    LAPTOP_IP = "192.168.1.2"
    ZMQ_PORT_DRIVE = 5555
    ZMQ_PORT_STATUS = 5559
    PICO_SERIAL_PORT = "/dev/ttyACM2"
    PICO_SERIAL_BAUD = 115200
    WATCHDOG_TIMEOUT_MS = 500
    DRIVE_CONTROL_HZ = 50


class PicoSerialBridge:
    """Handles serial communication with Pico drive controller."""
    
    def __init__(self, port: str = PICO_SERIAL_PORT, baud: int = PICO_SERIAL_BAUD):
        self.port = port
        self.baud = baud
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self.last_response = None
        self.response_lock = threading.Lock()
        self.read_thread: Optional[threading.Thread] = None
        self.running = False
        
    def connect(self) -> bool:
        """Connect to Pico serial port."""
        try:
            # Try symlink first, then fall back to direct port
            ports_to_try = ["/dev/pico", self.port, "/dev/ttyACM0", "/dev/ttyACM1"]
            
            for port in ports_to_try:
                try:
                    self.serial = serial.Serial(
                        port=port,
                        baudrate=self.baud,
                        timeout=0.1,
                        write_timeout=0.1
                    )
                    self.port = port
                    break
                except (serial.SerialException, FileNotFoundError):
                    continue
            
            if self.serial is None or not self.serial.is_open:
                print(f"[DRIVE] Failed to open any serial port")
                return False
            
            # Clear buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # Start read thread
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            # Wait for Pico ready message
            time.sleep(1.0)
            
            # Send ping to verify connection
            if self.ping():
                self.connected = True
                print(f"[DRIVE] Connected to Pico on {self.port}")
                return True
            else:
                print(f"[DRIVE] Pico not responding on {self.port}")
                return False
                
        except Exception as e:
            print(f"[DRIVE] Serial connection error: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Pico."""
        self.running = False
        self.connected = False
        
        # Send stop command before disconnecting
        if self.serial and self.serial.is_open:
            try:
                self.send_command({"cmd": "stop"})
            except:
                pass
            self.serial.close()
        
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        
        print("[DRIVE] Disconnected from Pico")
    
    def _read_loop(self):
        """Background thread to read serial responses."""
        while self.running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        try:
                            response = json.loads(line)
                            with self.response_lock:
                                self.last_response = response
                        except json.JSONDecodeError:
                            pass  # Ignore malformed responses
            except Exception as e:
                if self.running:
                    print(f"[DRIVE] Serial read error: {e}")
            time.sleep(0.001)  # Small delay to prevent CPU spinning
    
    def send_command(self, cmd: dict) -> bool:
        """Send command to Pico."""
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            msg = json.dumps(cmd) + "\n"
            self.serial.write(msg.encode('utf-8'))
            self.serial.flush()
            return True
        except Exception as e:
            print(f"[DRIVE] Serial write error: {e}")
            return False
    
    def send_drive(self, y: int, x: int, z: int) -> bool:
        """Send drive command with Y, X, Z axes."""
        return self.send_command({
            "cmd": "drive",
            "y": int(y),
            "x": int(x),
            "z": int(z)
        })
    
    def stop(self) -> bool:
        """Send emergency stop."""
        return self.send_command({"cmd": "stop"})
    
    def ping(self) -> bool:
        """Ping Pico and wait for response."""
        self.send_command({"cmd": "ping"})
        time.sleep(0.1)
        
        with self.response_lock:
            if self.last_response and "pong" in self.last_response:
                return True
        return False
    
    def get_status(self) -> Optional[dict]:
        """Get last status from Pico."""
        self.send_command({"cmd": "status"})
        time.sleep(0.05)
        
        with self.response_lock:
            return self.last_response


class DriveBridge:
    """
    Main drive bridge class.
    Receives commands via ZMQ, forwards to Pico via serial.
    """
    
    def __init__(self):
        self.pico = PicoSerialBridge()
        self.zmq_context = zmq.Context()
        
        # ZMQ socket to receive drive commands from laptop
        self.cmd_socket = self.zmq_context.socket(zmq.SUB)
        self.cmd_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.cmd_socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        
        # ZMQ socket to publish status
        self.status_socket = self.zmq_context.socket(zmq.PUB)
        
        self.running = False
        self.last_cmd_time = 0
        self.watchdog_timeout = WATCHDOG_TIMEOUT_MS / 1000.0
        
        # Statistics
        self.cmd_count = 0
        self.error_count = 0
    
    def start(self):
        """Start the drive bridge."""
        print("[DRIVE] Starting drive bridge...")
        
        # Connect to Pico
        if not self.pico.connect():
            print("[DRIVE] WARNING: Pico not connected, will retry...")
        
        # Bind ZMQ sockets
        try:
            bind_addr = f"tcp://*:{ZMQ_PORT_DRIVE}"
            self.cmd_socket.bind(bind_addr)
            print(f"[DRIVE] Listening for commands on {bind_addr}")
            
            status_addr = f"tcp://*:{ZMQ_PORT_STATUS}"
            self.status_socket.bind(status_addr)
            print(f"[DRIVE] Publishing status on {status_addr}")
            
        except zmq.ZMQError as e:
            print(f"[DRIVE] ZMQ bind error: {e}")
            return
        
        self.running = True
        self.last_cmd_time = time.time()
        
        print("[DRIVE] Drive bridge started")
        self._run_loop()
    
    def stop(self):
        """Stop the drive bridge."""
        print("[DRIVE] Stopping drive bridge...")
        self.running = False
        
        # Stop motors
        if self.pico.connected:
            self.pico.stop()
        
        self.pico.disconnect()
        
        # Close ZMQ sockets
        self.cmd_socket.close()
        self.status_socket.close()
        self.zmq_context.term()
        
        print("[DRIVE] Drive bridge stopped")
    
    def _run_loop(self):
        """Main processing loop."""
        loop_period = 1.0 / DRIVE_CONTROL_HZ
        status_period = 0.1  # Publish status at 10Hz
        last_status_time = 0
        
        while self.running:
            loop_start = time.time()
            
            # Try to reconnect Pico if disconnected
            if not self.pico.connected:
                if self.pico.connect():
                    print("[DRIVE] Reconnected to Pico")
            
            # Receive commands from laptop
            try:
                msg = self.cmd_socket.recv(zmq.NOBLOCK)
                self._handle_command(msg)
            except zmq.Again:
                pass  # No message available
            except Exception as e:
                print(f"[DRIVE] ZMQ receive error: {e}")
                self.error_count += 1
            
            # Watchdog - stop if no commands received
            if time.time() - self.last_cmd_time > self.watchdog_timeout:
                if self.pico.connected:
                    self.pico.stop()
            
            # Publish status periodically
            if time.time() - last_status_time > status_period:
                self._publish_status()
                last_status_time = time.time()
            
            # Maintain loop rate
            elapsed = time.time() - loop_start
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)
    
    def _handle_command(self, msg: bytes):
        """Process incoming drive command."""
        try:
            data = json.loads(msg.decode('utf-8'))
            msg_type = data.get("msg_type", data.get("cmd", ""))
            
            if msg_type in ["drive_cmd", "drive"]:
                y = int(data.get("y", 0))
                x = int(data.get("x", 0))
                z = int(data.get("z", 0))
                emergency_stop = data.get("emergency_stop", False)
                
                if emergency_stop:
                    self.pico.stop()
                else:
                    self.pico.send_drive(y, x, z)
                
                self.last_cmd_time = time.time()
                self.cmd_count += 1
                
            elif msg_type == "stop":
                self.pico.stop()
                self.last_cmd_time = time.time()
                
        except Exception as e:
            print(f"[DRIVE] Command handling error: {e}")
            self.error_count += 1
    
    def _publish_status(self):
        """Publish drive status."""
        status = {
            "msg_type": "drive_status",
            "timestamp": time.time(),
            "pico_connected": self.pico.connected,
            "cmd_count": self.cmd_count,
            "error_count": self.error_count,
            "last_cmd_age_ms": int((time.time() - self.last_cmd_time) * 1000)
        }
        
        # Get Pico status if connected
        if self.pico.connected:
            pico_status = self.pico.get_status()
            if pico_status:
                status.update({
                    "left_speed": pico_status.get("left_current", 0),
                    "right_speed": pico_status.get("right_current", 0),
                    "left_target": pico_status.get("left_target", 0),
                    "right_target": pico_status.get("right_target", 0),
                    "watchdog_ok": pico_status.get("watchdog_ok", False)
                })
        
        try:
            self.status_socket.send_json(status)
        except Exception as e:
            print(f"[DRIVE] Status publish error: {e}")


# =============================================================================
# ROS2 NODE (optional)
# =============================================================================

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    
    class DriveBridgeNode(Node):
        """ROS2 wrapper for drive bridge."""
        
        def __init__(self):
            super().__init__('drive_bridge')
            
            self.bridge = DriveBridge()
            
            # ROS2 subscriptions
            self.twist_sub = self.create_subscription(
                Twist, '/cmd_vel', self.twist_callback, 10
            )
            
            # ROS2 publishers
            self.status_pub = self.create_publisher(String, '/drive/status', 10)
            
            # Timer for status publishing
            self.create_timer(0.1, self.publish_status)
            
            self.get_logger().info('Drive bridge node initialized')
        
        def twist_callback(self, msg: Twist):
            """Convert ROS Twist (m/s, rad/s) to drive command (-100..100).
            Nav2 sends linear.x in m/s (e.g. 0.3). Scale by max_linear_vel_mps and clamp.
            """
            try:
                from shared.constants import NAV2_MAX_LINEAR_VEL_MPS, NAV2_MAX_ANGULAR_VEL_RPS
            except ImportError:
                NAV2_MAX_LINEAR_VEL_MPS = 0.3
                NAV2_MAX_ANGULAR_VEL_RPS = 1.0
            # linear.x [m/s] -> Y [-100, 100]
            if NAV2_MAX_LINEAR_VEL_MPS > 0:
                y = int(msg.linear.x / NAV2_MAX_LINEAR_VEL_MPS * 100)
            else:
                y = int(msg.linear.x * 100)
            y = max(-100, min(100, y))
            # linear.y (strafe) -> X
            x = int(msg.linear.y * 100) if abs(msg.linear.y) < 1e-3 else int(msg.linear.y / 0.3 * 100)
            x = max(-100, min(100, x))
            # angular.z [rad/s] -> Z
            if NAV2_MAX_ANGULAR_VEL_RPS > 0:
                z = int(msg.angular.z / NAV2_MAX_ANGULAR_VEL_RPS * 100)
            else:
                z = int(msg.angular.z * 100)
            z = max(-100, min(100, z))

            if self.bridge.pico.connected:
                self.bridge.pico.send_drive(y, x, z)
                self.bridge.last_cmd_time = time.time()
        
        def publish_status(self):
            """Publish status to ROS topic."""
            status_msg = String()
            status_msg.data = json.dumps({
                "pico_connected": self.bridge.pico.connected,
                "cmd_count": self.bridge.cmd_count
            })
            self.status_pub.publish(status_msg)
    
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Drive Bridge - ZMQ to Pico')
    parser.add_argument('--ros2', action='store_true', help='Run as ROS2 node')
    parser.add_argument('--port', type=str, default=PICO_SERIAL_PORT, 
                        help='Pico serial port')
    args = parser.parse_args()
    
    if args.ros2 and ROS2_AVAILABLE:
        # Run as ROS2 node
        rclpy.init()
        node = DriveBridgeNode()
        
        # Start bridge in background
        bridge_thread = threading.Thread(target=node.bridge.start, daemon=True)
        bridge_thread.start()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.bridge.stop()
            node.destroy_node()
            rclpy.shutdown()
    else:
        # Run standalone
        bridge = DriveBridge()
        
        try:
            bridge.start()
        except KeyboardInterrupt:
            print("\n[DRIVE] Interrupted by user")
        finally:
            bridge.stop()


if __name__ == "__main__":
    main()
