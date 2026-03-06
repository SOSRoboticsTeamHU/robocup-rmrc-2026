#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Joystick Handler
==========================================
Handles Thrustmaster TCA Airbus Edition joystick input and sends
drive commands to the Jetson via ZMQ.

Control Mapping:
- Y axis (forward/back): Forward/backward movement
- X axis (left/right): Curved turning (arcade drive)
- Z axis (twist): Spot turn / rotate in place

Features:
- Deadzone filtering
- Smooth ramping
- Button mapping for emergency stop, arm control, etc.
- Haptic feedback (if supported)
"""

import time
import threading
import json
import os
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass
from enum import Enum

# pygame will be imported lazily to avoid segfault on macOS when PyQt5 is also used
# The import happens in JoystickHandler.initialize()
pygame = None
PYGAME_AVAILABLE = None  # Will be set on first use

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False
    print("WARNING: pyzmq not installed. Run: pip install pyzmq")

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'shared'))

try:
    from shared.constants import (
        JETSON_IP, ZMQ_PORT_DRIVE, 
        JOYSTICK_DEADZONE, MAX_DRIVE_SPEED,
        DRIVE_CONTROL_HZ
    )
except ImportError:
    JETSON_IP = "192.168.1.1"
    ZMQ_PORT_DRIVE = 5555
    JOYSTICK_DEADZONE = 0.05
    MAX_DRIVE_SPEED = 100
    DRIVE_CONTROL_HZ = 50


# =============================================================================
# JOYSTICK CONFIGURATION
# =============================================================================

@dataclass
class JoystickConfig:
    """Configuration for joystick axes and buttons."""
    # Axis indices (may need adjustment for your specific joystick)
    axis_y: int = 1          # Forward/backward (usually inverted)
    axis_x: int = 0          # Left/right turn
    axis_z: int = 2          # Twist/rotation (Z-axis or rudder)
    axis_throttle: int = 3   # Throttle (optional speed limiter)
    
    # Axis inversions (True = invert)
    invert_y: bool = True    # Usually needed for "forward = negative"
    invert_x: bool = False
    invert_z: bool = True    # Negate raw Z: twist right (raw neg) -> positive z -> robot turns right
    
    # Deadzone (0.0 - 1.0)
    deadzone: float = 0.08
    
    # Sensitivity curve exponent (1.0 = linear, 2.0 = quadratic)
    curve_exponent: float = 1.5
    
    # Button indices
    button_emergency_stop: int = 0   # Trigger or button 1
    button_arm_toggle: int = 1       # Button 2
    button_speed_boost: int = 2      # Button 3
    button_headlights: int = 3       # Button 4
    
    # Speed settings
    max_speed: int = 100
    normal_speed: int = 70
    slow_speed: int = 40


class JoystickButton(Enum):
    """Named button events."""
    EMERGENCY_STOP = "emergency_stop"
    ARM_TOGGLE = "arm_toggle"
    SPEED_BOOST = "speed_boost"
    HEADLIGHTS = "headlights"
    UNKNOWN = "unknown"


# =============================================================================
# JOYSTICK HANDLER
# =============================================================================

class JoystickHandler:
    """
    Handles joystick input and converts it to drive commands.
    Sends commands to Jetson via ZMQ.
    """
    
    def __init__(self, config: JoystickConfig = None, jetson_ip: str = JETSON_IP):
        self.config = config or JoystickConfig()
        self.jetson_ip = jetson_ip
        
        # Pygame joystick (type hint uses Any since pygame is lazy-loaded)
        self.joystick: Optional[Any] = None
        self.joystick_name = "Not connected"
        
        # ZMQ socket
        self.zmq_context: Optional[zmq.Context] = None
        self.zmq_socket: Optional[zmq.Socket] = None
        
        # State
        self.running = False
        self.connected = False
        self.current_speed_limit = self.config.normal_speed
        
        # Current values
        self.axis_values = {"y": 0.0, "x": 0.0, "z": 0.0, "throttle": 0.0}
        self.button_states: Dict[int, bool] = {}
        
        # Callbacks
        self.on_button_press: Optional[Callable[[JoystickButton], None]] = None
        self.on_button_release: Optional[Callable[[JoystickButton], None]] = None
        self.on_drive_command: Optional[Callable[[int, int, int], None]] = None
        
        # Statistics
        self.cmd_count = 0
        self.last_cmd_time = 0
    
    def initialize(self) -> bool:
        """Initialize pygame and ZMQ."""
        global pygame, PYGAME_AVAILABLE
        
        # Lazy import pygame to avoid segfault on macOS when PyQt5 is loaded first
        if pygame is None:
            # Set SDL environment variables BEFORE import to prevent Qt conflicts
            os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
            os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
            # Allow joystick events in background (important for Qt coexistence)
            os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
            # Disable SDL hint that can cause conflicts
            os.environ.setdefault("SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
            try:
                import pygame as _pygame
                pygame = _pygame
                PYGAME_AVAILABLE = True
            except ImportError:
                PYGAME_AVAILABLE = False
                print("[JOYSTICK] WARNING: pygame not installed. Run: pip install pygame")
        
        if not PYGAME_AVAILABLE:
            print("[JOYSTICK] pygame not available")
            return False
        
        # Initialize ONLY joystick subsystem to avoid conflict with Qt on macOS
        # Do NOT use pygame.init() as it initializes video which causes trace trap
        pygame.joystick.init()
        
        # In standalone mode (no Qt), we can pump events safely
        # When used with Qt (main.py combined mode), this will be set to False
        self._can_pump_events = True
        
        # Find and initialize joystick
        if pygame.joystick.get_count() == 0:
            print("[JOYSTICK] No joystick found")
            return False
        
        # Use first joystick (or find Thrustmaster specifically)
        for i in range(pygame.joystick.get_count()):
            js = pygame.joystick.Joystick(i)
            js.init()
            name = js.get_name().lower()
            print(f"[JOYSTICK] Found: {js.get_name()}")
            
            # Prefer Thrustmaster if found
            if "thrustmaster" in name or "tca" in name:
                self.joystick = js
                break
        
        if self.joystick is None:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        
        self.joystick_name = self.joystick.get_name()
        print(f"[JOYSTICK] Using: {self.joystick_name}")
        print(f"[JOYSTICK] Axes: {self.joystick.get_numaxes()}, Buttons: {self.joystick.get_numbuttons()}")
        
        # Initialize ZMQ
        if ZMQ_AVAILABLE:
            self.zmq_context = zmq.Context()
            self.zmq_socket = self.zmq_context.socket(zmq.PUB)
            
            # CRITICAL: Real-time settings - no buffering!
            self.zmq_socket.setsockopt(zmq.SNDHWM, 1)  # Only buffer 1 message
            self.zmq_socket.setsockopt(zmq.LINGER, 0)  # Don't wait on close
            
            connect_str = f"tcp://{self.jetson_ip}:{ZMQ_PORT_DRIVE}"
            
            try:
                self.zmq_socket.connect(connect_str)
                print(f"[JOYSTICK] Connected to Jetson at {connect_str}")
                self.connected = True
            except Exception as e:
                print(f"[JOYSTICK] ZMQ connection failed: {e}")
                self.connected = False
        
        return True
    
    def shutdown(self):
        """Clean up resources."""
        self.running = False
        
        if self.zmq_socket:
            # Send stop command before disconnecting
            self._send_drive_command(0, 0, 0, emergency_stop=True)
            self.zmq_socket.close()
        
        if self.zmq_context:
            self.zmq_context.term()
        
        if self.joystick:
            self.joystick.quit()
        
        pygame.joystick.quit()
        
        print("[JOYSTICK] Shutdown complete")
    
    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to axis value."""
        if abs(value) < self.config.deadzone:
            return 0.0
        
        # Scale remaining range to 0-1
        sign = 1 if value > 0 else -1
        scaled = (abs(value) - self.config.deadzone) / (1.0 - self.config.deadzone)
        return sign * scaled
    
    def apply_curve(self, value: float) -> float:
        """Apply sensitivity curve to axis value."""
        sign = 1 if value > 0 else -1
        return sign * (abs(value) ** self.config.curve_exponent)
    
    def process_axis(self, axis_index: int, invert: bool = False) -> float:
        """Read and process a joystick axis."""
        if self.joystick is None:
            return 0.0
        
        if axis_index >= self.joystick.get_numaxes():
            return 0.0
        
        value = self.joystick.get_axis(axis_index)
        
        if invert:
            value = -value
        
        value = self.apply_deadzone(value)
        value = self.apply_curve(value)
        
        return value
    
    def get_button(self, button_index: int) -> bool:
        """Get button state."""
        if self.joystick is None:
            return False
        
        if button_index >= self.joystick.get_numbuttons():
            return False
        
        return self.joystick.get_button(button_index) == 1
    
    def update(self):
        """
        Update joystick state and send drive commands.
        Should be called at regular intervals (e.g., 50Hz).
        """
        if self.joystick is None:
            return
        
        # Pump events only if safe (standalone mode, not with Qt)
        # On macOS with Qt, calling event.pump() causes segfault since video isn't initialized
        if getattr(self, '_can_pump_events', True):
            try:
                pygame.event.pump()
            except pygame.error:
                # Event module not initialized, disable pumping for future calls
                self._can_pump_events = False
        
        # Read axes
        y = self.process_axis(self.config.axis_y, self.config.invert_y)
        x = self.process_axis(self.config.axis_x, self.config.invert_x)
        z = self.process_axis(self.config.axis_z, self.config.invert_z)
        throttle = self.process_axis(self.config.axis_throttle, False)
        
        self.axis_values = {"y": y, "x": x, "z": z, "throttle": throttle}
        
        # Process buttons
        self._process_buttons()
        
        # Calculate speed limit based on throttle (optional)
        # Throttle at -1 = slow, at +1 = fast
        # speed_factor = (throttle + 1) / 2  # 0 to 1
        # self.current_speed_limit = int(self.config.slow_speed + 
        #     speed_factor * (self.config.max_speed - self.config.slow_speed))
        
        # Convert to integer drive values (-100 to 100)
        drive_y = int(y * self.current_speed_limit)
        drive_x = int(x * self.current_speed_limit)
        drive_z = int(z * self.current_speed_limit)
        
        # Send drive command
        self._send_drive_command(drive_y, drive_x, drive_z)
        
        # Callback
        if self.on_drive_command:
            self.on_drive_command(drive_y, drive_x, drive_z)
    
    def _process_buttons(self):
        """Process button presses and releases."""
        button_map = {
            self.config.button_emergency_stop: JoystickButton.EMERGENCY_STOP,
            self.config.button_arm_toggle: JoystickButton.ARM_TOGGLE,
            self.config.button_speed_boost: JoystickButton.SPEED_BOOST,
            self.config.button_headlights: JoystickButton.HEADLIGHTS,
        }
        
        for button_idx, button_type in button_map.items():
            current = self.get_button(button_idx)
            previous = self.button_states.get(button_idx, False)
            
            if current and not previous:
                # Button pressed
                self._handle_button_press(button_type)
                if self.on_button_press:
                    self.on_button_press(button_type)
            
            elif not current and previous:
                # Button released
                self._handle_button_release(button_type)
                if self.on_button_release:
                    self.on_button_release(button_type)
            
            self.button_states[button_idx] = current
    
    def _handle_button_press(self, button: JoystickButton):
        """Handle button press events."""
        if button == JoystickButton.EMERGENCY_STOP:
            print("[JOYSTICK] EMERGENCY STOP")
            self._send_drive_command(0, 0, 0, emergency_stop=True)
        
        elif button == JoystickButton.SPEED_BOOST:
            self.current_speed_limit = self.config.max_speed
            print(f"[JOYSTICK] Speed boost: {self.current_speed_limit}%")
    
    def _handle_button_release(self, button: JoystickButton):
        """Handle button release events."""
        if button == JoystickButton.SPEED_BOOST:
            self.current_speed_limit = self.config.normal_speed
            print(f"[JOYSTICK] Normal speed: {self.current_speed_limit}%")
    
    def _send_drive_command(self, y: int, x: int, z: int, emergency_stop: bool = False):
        """Send drive command via ZMQ."""
        if not self.zmq_socket:
            return
        
        cmd = {
            "msg_type": "drive_cmd",
            "timestamp": time.time(),
            "y": y,
            "x": x,
            "z": z,
            "emergency_stop": emergency_stop
        }
        
        try:
            self.zmq_socket.send_json(cmd, zmq.NOBLOCK)
            self.cmd_count += 1
            self.last_cmd_time = time.time()
        except zmq.Again:
            pass  # Socket buffer full, skip this command
        except Exception as e:
            print(f"[JOYSTICK] Send error: {e}")
    
    def run_loop(self, rate_hz: int = DRIVE_CONTROL_HZ):
        """
        Run the main joystick processing loop.
        Blocks until stop() is called or interrupted.
        """
        self.running = True
        loop_period = 1.0 / rate_hz
        
        print(f"[JOYSTICK] Starting control loop at {rate_hz}Hz")
        print("[JOYSTICK] Press Ctrl+C to stop")
        
        try:
            while self.running:
                loop_start = time.time()
                
                self.update()
                
                # Maintain loop rate
                elapsed = time.time() - loop_start
                if elapsed < loop_period:
                    time.sleep(loop_period - elapsed)
        
        except KeyboardInterrupt:
            print("\n[JOYSTICK] Interrupted")
        
        finally:
            self.running = False
    
    def stop(self):
        """Stop the control loop."""
        self.running = False
    
    def get_state(self) -> Dict[str, Any]:
        """Get current joystick state for GUI display."""
        return {
            "connected": self.joystick is not None,
            "name": self.joystick_name,
            "axes": self.axis_values.copy(),
            "speed_limit": self.current_speed_limit,
            "cmd_count": self.cmd_count,
            "zmq_connected": self.connected
        }


# =============================================================================
# CALIBRATION UTILITY
# =============================================================================

def calibrate_joystick():
    """Interactive joystick calibration utility."""
    if not PYGAME_AVAILABLE:
        print("pygame required for calibration")
        return
    
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No joystick found!")
        return
    
    js = pygame.joystick.Joystick(0)
    js.init()
    
    print(f"\nJoystick: {js.get_name()}")
    print(f"Axes: {js.get_numaxes()}")
    print(f"Buttons: {js.get_numbuttons()}")
    print(f"Hats: {js.get_numhats()}")
    print("\nMove axes and press buttons to see their indices.")
    print("Press Ctrl+C to exit.\n")
    
    try:
        while True:
            pygame.event.pump()
            
            # Print axis values
            axes_str = " | ".join([f"A{i}:{js.get_axis(i):+.2f}" 
                                   for i in range(js.get_numaxes())])
            
            # Print button values
            buttons_str = " ".join([f"B{i}:{js.get_button(i)}" 
                                    for i in range(min(8, js.get_numbuttons()))])
            
            print(f"\r{axes_str} || {buttons_str}    ", end="", flush=True)
            
            time.sleep(0.05)
    
    except KeyboardInterrupt:
        print("\n\nCalibration ended.")
    
    finally:
        js.quit()
        pygame.joystick.quit()
        pygame.quit()


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Joystick Handler for RoboCup Rescue Robot')
    parser.add_argument('--calibrate', action='store_true', help='Run calibration mode')
    parser.add_argument('--jetson-ip', type=str, default=JETSON_IP, help='Jetson IP address')
    parser.add_argument('--rate', type=int, default=50, help='Control loop rate (Hz)')
    args = parser.parse_args()
    
    if args.calibrate:
        calibrate_joystick()
        return
    
    # Create and run joystick handler
    handler = JoystickHandler(jetson_ip=args.jetson_ip)
    
    if not handler.initialize():
        print("Failed to initialize joystick")
        return
    
    # Add simple console output callback
    def on_drive(y, x, z):
        if y != 0 or x != 0 or z != 0:
            print(f"\rDrive: Y={y:+4d} X={x:+4d} Z={z:+4d}  ", end="", flush=True)
    
    handler.on_drive_command = on_drive
    
    try:
        handler.run_loop(rate_hz=args.rate)
    finally:
        handler.shutdown()


if __name__ == "__main__":
    main()
