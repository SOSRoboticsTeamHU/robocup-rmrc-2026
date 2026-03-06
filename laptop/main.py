#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Laptop Operator Station
=================================================
Main entry point for the operator station.

Usage:
    python main.py                    # Run with GUI, joystick, camera, arm leader
    python main.py --no-gui           # Run joystick only (headless)
    python main.py --no-joystick      # Run GUI without joystick
    python main.py --no-arm           # Run GUI without arm leader (teleop)
    python main.py --calibrate        # Calibrate joystick (axes)
    python main.py --joy-buttons-debug   # GUI: show joystick button ID when pressed
    python -m control.joystick_button_debug   # Standalone: print button IDs (no GUI)
"""

import sys
import os
import time
import argparse

# Add paths
sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'shared'))


def main():
    parser = argparse.ArgumentParser(
        description='RoboCupRescue RMRC 2026 Operator Station'
    )
    parser.add_argument('--no-gui', action='store_true', 
                        help='Run without GUI (joystick only)')
    parser.add_argument('--no-joystick', action='store_true',
                        help='Run without joystick (GUI only)')
    parser.add_argument('--calibrate', action='store_true',
                        help='Run joystick calibration')
    parser.add_argument('--jetson-ip', type=str, default='192.168.2.100',
                        help='Jetson IP address')
    parser.add_argument('--rate', type=int, default=50,
                        help='Control loop rate in Hz')
    parser.add_argument('--logitech', action='store_true',
                        help='Use Logitech joystick axis mapping (fwd=axis2, rot=axis1)')
    parser.add_argument('--joystick-debug', action='store_true',
                        help='Print joystick axis values every 200ms')
    parser.add_argument('--joy-buttons-debug', action='store_true',
                        help='GUI: show joystick button ID on press (no action); use to find IDs for mapping')
    parser.add_argument('--no-arm', action='store_true',
                        help='Do not start arm leader (leader arm teleop to Jetson)')
    parser.add_argument('--arm-serial', type=str, default=None,
                        help='Leader arm serial port (default from shared.constants)')
    parser.add_argument('--arm-no-lerobot', action='store_true',
                        help='Run arm leader without hardware (send neutral positions)')
    args = parser.parse_args()
    
    # Calibration mode (raw axis print for mapping)
    if args.calibrate:
        from control.joystick_process import calibrate_joystick
        calibrate_joystick()
        return

    # Headless mode (joystick only - no GUI)
    if args.no_gui:
        run_headless(args)
        return

    # GUI mode: start camera (in GUI), joystick process (optional), arm leader process (optional)
    run_gui_mode(args)


def run_headless(args):
    """Run joystick-only mode without GUI."""
    from control.joystick_handler import JoystickHandler, JoystickConfig
    
    config = JoystickConfig()
    handler = JoystickHandler(config=config, jetson_ip=args.jetson_ip)
    
    if not handler.initialize():
        print("[MAIN] No joystick found!")
        return
    
    print("[MAIN] Running in headless mode (joystick only)")
    
    def on_drive(y, x, z):
        print(f"\rDrive: Y={y:+4d} X={x:+4d} Z={z:+4d}  ", end="", flush=True)
    
    handler.on_drive_command = on_drive
    
    try:
        handler.run_loop(rate_hz=args.rate)
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted")
    finally:
        handler.shutdown()


def run_gui_mode(args):
    """Run GUI with integrated joystick and arm leader (same pattern as camera/joystick)."""
    
    # STEP 1: Initialize Qt FIRST (before pygame)
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QTimer, Qt
    from PyQt5.QtGui import QPalette, QColor
    
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    # Dark palette
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(26, 26, 46))
    palette.setColor(QPalette.WindowText, QColor(224, 224, 224))
    palette.setColor(QPalette.Base, QColor(15, 15, 26))
    palette.setColor(QPalette.Text, QColor(224, 224, 224))
    palette.setColor(QPalette.Button, QColor(15, 52, 96))
    palette.setColor(QPalette.ButtonText, QColor(224, 224, 224))
    palette.setColor(QPalette.Highlight, QColor(78, 204, 163))
    app.setPalette(palette)
    
    # STEP 2: Create GUI window (camera receiver started inside OperatorStation)
    from gui.main_window import OperatorStation
    window = OperatorStation(
        jetson_ip=args.jetson_ip,
        joy_buttons_debug=getattr(args, "joy_buttons_debug", False),
    )
    
    # STEP 3: Start arm leader process (teleop to Jetson, like joystick for drive)
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
            serial_port = args.arm_serial if args.arm_serial else ARM_LEADER_SERIAL
            arm_leader_proc = Process(
                target=run_leader_arm,
                kwargs={
                    "jetson_ip": args.jetson_ip,
                    "serial_port": serial_port,
                    "rate_hz": ARM_LEADER_RATE_HZ,
                    "no_lerobot": args.arm_no_lerobot,
                },
                daemon=True,
            )
            arm_leader_proc.start()
            print("[MAIN] Arm leader started")
        except Exception as e:
            print(f"[MAIN] Arm leader start error: {e}")
            import traceback
            traceback.print_exc()
    
    # STEP 4: Initialize joystick (AFTER Qt)
    joystick = None
    pygame = None
    
    # Use multiprocessing for joystick to avoid pygame/Qt conflicts on macOS
    joystick_queue = None
    joystick_proc = None
    
    if not args.no_joystick:
        try:
            from multiprocessing import Process, Queue
            from control.joystick_process import joystick_worker
            
            joystick_queue = Queue(maxsize=1)
            joystick_proc = Process(
                target=joystick_worker,
                args=(joystick_queue, args.jetson_ip),
                kwargs={"logitech_axis_swap": args.logitech, "debug": args.joystick_debug},
                daemon=True
            )
            joystick_proc.start()
            print("[MAIN] Joystick process started")
            
        except Exception as e:
            print(f"[MAIN] Joystick init error: {e}")
            import traceback
            traceback.print_exc()
            joystick_queue = None
    
    # STEP 5: Timer to read joystick state from subprocess queue
    def update_joystick_gui():
        if joystick_queue is None:
            return
        try:
            # Get latest joystick state from queue
            state = joystick_queue.get_nowait()
            window.update_joystick_state({
                "connected": True,
                "axes": state
            })
        except:
            pass
    
    if joystick_queue:
        timer = QTimer()
        timer.timeout.connect(update_joystick_gui)
        timer.start(50)  # 20Hz is enough for GUI display
    
    # STEP 6: Show window and run
    window.show()
    print("[MAIN] Operator Station started")
    print("[MAIN] Press Ctrl+C or close window to exit")
    
    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted")
        exit_code = 0
    finally:
        if joystick_proc and joystick_proc.is_alive():
            joystick_proc.terminate()
            joystick_proc.join(timeout=1)
        if arm_leader_proc and arm_leader_proc.is_alive():
            arm_leader_proc.terminate()
            arm_leader_proc.join(timeout=1)

        print("[MAIN] Shutdown complete")
    
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
