#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Standalone Joystick Controller
=========================================================
Run this separately from the GUI to avoid pygame/Qt conflicts on macOS.

Publishes joystick state to local ZMQ socket for GUI to receive.
Sends drive commands to Jetson.

Usage:
    python run_joystick.py
    python run_joystick.py --calibrate
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'shared'))

try:
    from shared.constants import JETSON_IP, DRIVE_CONTROL_HZ
except ImportError:
    JETSON_IP = "192.168.2.100"
    DRIVE_CONTROL_HZ = 50

# Local port for GUI to subscribe to joystick state
ZMQ_PORT_JOYSTICK_STATE = 5560


def main():
    parser = argparse.ArgumentParser(
        description='Standalone Joystick Controller'
    )
    parser.add_argument('--calibrate', action='store_true',
                        help='Run joystick calibration')
    parser.add_argument('--jetson-ip', type=str, default=JETSON_IP,
                        help='Jetson IP address')
    parser.add_argument('--rate', type=int, default=DRIVE_CONTROL_HZ,
                        help='Control loop rate in Hz')
    parser.add_argument('--gui-port', type=int, default=ZMQ_PORT_JOYSTICK_STATE,
                        help='Port to publish joystick state for GUI')
    args = parser.parse_args()
    
    # Calibration mode
    if args.calibrate:
        from control.joystick_handler import calibrate_joystick
        calibrate_joystick()
        return
    
    # Import after args parsing
    from control.joystick_handler import JoystickHandler, JoystickConfig
    
    try:
        import zmq
        ZMQ_AVAILABLE = True
    except ImportError:
        ZMQ_AVAILABLE = False
        print("[JOYSTICK] Warning: pyzmq not installed, GUI won't receive state")
    
    # Setup joystick
    config = JoystickConfig()
    handler = JoystickHandler(config=config, jetson_ip=args.jetson_ip)
    
    if not handler.initialize():
        print("[JOYSTICK] Failed to initialize joystick")
        return 1
    
    # Setup ZMQ publisher for GUI
    gui_socket = None
    zmq_context = None
    if ZMQ_AVAILABLE:
        zmq_context = zmq.Context()
        gui_socket = zmq_context.socket(zmq.PUB)
        gui_socket.setsockopt(zmq.SNDHWM, 1)  # Only keep latest
        gui_socket.bind(f"tcp://127.0.0.1:{args.gui_port}")
        print(f"[JOYSTICK] Publishing state on tcp://127.0.0.1:{args.gui_port}")
    
    # Callback to print and publish state
    def on_drive(y, x, z):
        print(f"\rDrive: Y={y:+4d} X={x:+4d} Z={z:+4d}  ", end="", flush=True)
        
        # Publish state to GUI
        if gui_socket:
            state = handler.get_state()
            state['drive'] = {'y': y, 'x': x, 'z': z}
            try:
                gui_socket.send_json(state, zmq.NOBLOCK)
            except zmq.Again:
                pass
    
    handler.on_drive_command = on_drive
    
    print(f"[JOYSTICK] Starting control loop at {args.rate}Hz")
    print("[JOYSTICK] Press Ctrl+C to stop")
    
    try:
        handler.run_loop(rate_hz=args.rate)
    except KeyboardInterrupt:
        print("\n[JOYSTICK] Interrupted")
    finally:
        handler.shutdown()
        if gui_socket:
            gui_socket.close()
        if zmq_context:
            zmq_context.term()
    
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
