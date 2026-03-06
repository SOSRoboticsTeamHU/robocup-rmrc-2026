#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - GUI Operator Station
===============================================
Alternative entry point: GUI with camera; optional joystick via ZMQ, optional arm leader.

Usage:
    python run_gui.py                  # GUI + camera + arm leader (like main.py)
    python run_gui.py --no-arm         # GUI without arm leader
    
Start joystick in another terminal for drive control:
    python run_joystick.py
"""

import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'shared'))

try:
    from shared.constants import JETSON_IP
except ImportError:
    JETSON_IP = "192.168.2.100"

# Local port to receive joystick state
ZMQ_PORT_JOYSTICK_STATE = 5560


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='RoboCupRescue RMRC 2026 Operator Station GUI'
    )
    parser.add_argument('--jetson-ip', type=str, default=JETSON_IP,
                        help='Jetson IP address')
    parser.add_argument('--joystick-port', type=int, default=ZMQ_PORT_JOYSTICK_STATE,
                        help='Port to receive joystick state')
    parser.add_argument('--no-arm', action='store_true',
                        help='Do not start arm leader (teleop to Jetson)')
    parser.add_argument('--arm-no-lerobot', action='store_true',
                        help='Run arm leader without hardware (neutral positions)')
    args = parser.parse_args()
    
    # Import PyQt5
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QTimer
    from PyQt5.QtGui import QPalette, QColor
    
    from gui.main_window import OperatorStation
    
    try:
        import zmq
        ZMQ_AVAILABLE = True
    except ImportError:
        ZMQ_AVAILABLE = False
        print("[GUI] Warning: pyzmq not installed, won't receive joystick state")
    
    # Create Qt application
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    # Set dark palette
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(26, 26, 46))
    palette.setColor(QPalette.WindowText, QColor(224, 224, 224))
    palette.setColor(QPalette.Base, QColor(15, 15, 26))
    palette.setColor(QPalette.Text, QColor(224, 224, 224))
    palette.setColor(QPalette.Button, QColor(15, 52, 96))
    palette.setColor(QPalette.ButtonText, QColor(224, 224, 224))
    palette.setColor(QPalette.Highlight, QColor(78, 204, 163))
    app.setPalette(palette)
    
    # Create main window (camera receiver started inside OperatorStation)
    window = OperatorStation(jetson_ip=args.jetson_ip)
    
    # Start arm leader process (same as main.py)
    arm_leader_proc = None
    if not args.no_arm:
        try:
            from multiprocessing import Process
            from control.leader_arm import run_leader_arm
            try:
                from shared.constants import ARM_LEADER_SERIAL, ARM_LEADER_RATE_HZ
            except ImportError:
                ARM_LEADER_SERIAL = "/dev/tty.usbmodem5AE60833341"
                ARM_LEADER_RATE_HZ = 100
            arm_leader_proc = Process(
                target=run_leader_arm,
                kwargs={
                    "jetson_ip": args.jetson_ip,
                    "serial_port": ARM_LEADER_SERIAL,
                    "rate_hz": ARM_LEADER_RATE_HZ,
                    "no_lerobot": args.arm_no_lerobot,
                },
                daemon=True,
            )
            arm_leader_proc.start()
            print("[GUI] Arm leader started")
        except Exception as e:
            print(f"[GUI] Arm leader start error: {e}")
    
    # Setup ZMQ subscriber for joystick state
    joystick_socket = None
    zmq_context = None
    if ZMQ_AVAILABLE:
        zmq_context = zmq.Context()
        joystick_socket = zmq_context.socket(zmq.SUB)
        joystick_socket.setsockopt(zmq.SUBSCRIBE, b"")
        joystick_socket.setsockopt(zmq.RCVTIMEO, 10)  # 10ms timeout
        joystick_socket.setsockopt(zmq.CONFLATE, 1)   # Only keep latest
        joystick_socket.connect(f"tcp://127.0.0.1:{args.joystick_port}")
        print(f"[GUI] Subscribing to joystick on tcp://127.0.0.1:{args.joystick_port}")
    
    # Timer to poll joystick state
    def poll_joystick():
        if joystick_socket:
            try:
                state = joystick_socket.recv_json(zmq.NOBLOCK)
                window.update_joystick_state(state)
            except zmq.Again:
                pass  # No message available
            except Exception as e:
                pass  # Ignore errors
    
    joystick_timer = QTimer()
    joystick_timer.timeout.connect(poll_joystick)
    joystick_timer.start(20)  # 50Hz polling
    
    # Show window
    window.show()
    
    print("[GUI] Operator Station started")
    print("[GUI] Run 'python run_joystick.py' in another terminal for joystick control")
    print("[GUI] Press Ctrl+C or close window to exit")
    
    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        print("\n[GUI] Interrupted")
        exit_code = 0
    finally:
        if arm_leader_proc and arm_leader_proc.is_alive():
            arm_leader_proc.terminate()
            arm_leader_proc.join(timeout=1)
        if joystick_socket:
            joystick_socket.close()
        if zmq_context:
            zmq_context.term()
        print("[GUI] Shutdown complete")
    
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
