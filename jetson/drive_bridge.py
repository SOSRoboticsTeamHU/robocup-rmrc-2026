#!/usr/bin/env python3
"""
=============================================================================
DRIVE BRIDGE - ZMQ -> Pico (left,right)
=============================================================================
Protokoll: y=előre/hátra, x=íves kanyar jobbra/balra, z=egyhelyben forgás jobbra/balra.
Mix: bal = y+x+z, jobb = y-x-z. Pico-nak "left,right\\n" formátumban küldjük.
=============================================================================
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
try:
    from shared.constants import PICO_SERIAL_PORT, PICO_SERIAL_BAUD, ZMQ_PORT_DRIVE
except ImportError:
    PICO_SERIAL_PORT = "/dev/ttyACM0"
    PICO_SERIAL_BAUD = 115200
    ZMQ_PORT_DRIVE = 5555

import zmq
import serial
import time

# =============================================================================
# CONFIGURATION
# =============================================================================

ZMQ_PORT = ZMQ_PORT_DRIVE
WATCHDOG_MS = 150  # Emergency stop timeout
DEBUG_INTERVAL = 0.3  # Print received/mixed values at most every N seconds when debug

# =============================================================================
# MAIN
# =============================================================================

def main():
    import argparse
    ap = argparse.ArgumentParser(description="ZMQ drive bridge -> Pico serial L,R")
    ap.add_argument("--debug", action="store_true", help="Print received y,x,z and mixed left,right every 300ms")
    ap.add_argument("--port", type=int, default=ZMQ_PORT, help="ZMQ port to bind")
    ap.add_argument("--serial", type=str, default=PICO_SERIAL_PORT, help="Pico serial port")
    args = ap.parse_args()
    debug = args.debug
    zmq_port = args.port
    serial_port = args.serial

    print("="*60)
    print("DRIVE BRIDGE - INDUSTRY-GRADE MOTOR CONTROLLER")
    print("="*60)
    
    # --- Serial Setup ---
    ser = None
    try:
        ser = serial.Serial(serial_port, PICO_SERIAL_BAUD, timeout=0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.2)
        print(f"[SERIAL] Connected: {serial_port} @ {PICO_SERIAL_BAUD}")
    except Exception as e:
        print(f"[SERIAL] ERROR: {e} (port {serial_port})")
    
    # --- ZMQ Setup (optimized for zero latency) ---
    ctx = zmq.Context(1)  # Single IO thread
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")      # Subscribe to all
    sock.setsockopt(zmq.RCVHWM, 1)           # Queue size = 1
    sock.setsockopt(zmq.CONFLATE, 1)         # Keep only latest
    sock.setsockopt(zmq.RCVTIMEO, 5)         # 5ms timeout (200Hz poll)
    sock.setsockopt(zmq.RCVBUF, 1024)        # Small receive buffer
    sock.bind(f"tcp://0.0.0.0:{zmq_port}")
    print(f"[ZMQ] Listening on port {zmq_port}")
    if debug:
        print("[BRIDGE] Debug ON: will print y,x,z -> left,right every 300ms when non-zero")
    
    # --- State ---
    last_cmd_time = time.perf_counter()
    last_left = 0
    last_right = 0
    stopped = True
    last_debug_t = 0.0
    
    # --- Helper Functions ---
    def send_motors(left, right):
        """Send motor command to Pico every time so motion stays continuous (Pico/watchdog expects steady stream)."""
        nonlocal last_left, last_right, stopped
        if ser is None:
            return
        cmd = f"{-left},{right}\n"
        ser.write(cmd.encode())
        last_left = left
        last_right = right
        stopped = (left == 0 and right == 0)
    
    def mix(fwd, turn, rot):
        """
        y=fwd előre/hátra, x=turn íves kanyar, z=rot egyhelyben forgás.
        Bal kerék = fwd+turn+rot, jobb kerék = fwd-turn-rot.
        turn>0 vagy rot>0 -> bal gyorsabb -> robot jobbra megy/fordul.
        """
        left = fwd - turn - rot
        right = fwd + turn + rot
        
        # Scale if exceeds limits
        max_val = max(abs(left), abs(right), 1)
        if max_val > 100:
            left = int(left * 100 / max_val)
            right = int(right * 100 / max_val)
        
        return int(left), int(right)
    
    # --- Main Loop ---
    print(f"[BRIDGE] Running (watchdog: {WATCHDOG_MS}ms)")
    print("[BRIDGE] Press Ctrl+C to stop")
    
    try:
        while True:
            try:
                # Receive command (5ms timeout)
                msg = sock.recv_json()
                last_cmd_time = time.perf_counter()
                
                # y=előre/hátra, x=íves kanyar, z=egyhelyben forgás
                fwd = int(msg.get("y", 0))
                turn = int(msg.get("x", 0))
                rot = -int(msg.get("z", 0))   # Negate: joystick Z sign inverted vs robot
                left, right = mix(fwd, turn, rot)
                send_motors(left, right)  # Pico: "left,right\n"

                # Debug: print received and mixed values periodically
                if debug and (fwd != 0 or turn != 0 or rot != 0):
                    t = time.perf_counter()
                    if t - last_debug_t >= DEBUG_INTERVAL:
                        print(f"[BRIDGE] y={fwd} x={turn} z={rot} -> left={left} right={right}")
                        last_debug_t = t
                
            except zmq.Again:
                # No message - check watchdog
                elapsed_ms = (time.perf_counter() - last_cmd_time) * 1000
                if elapsed_ms > WATCHDOG_MS and not stopped:
                    print(f"[WATCHDOG] No commands for {int(elapsed_ms)}ms - STOP")
                    send_motors(0, 0)
                    
    except KeyboardInterrupt:
        print("\n[BRIDGE] Interrupted")
    finally:
        # Safety: stop motors
        if ser:
            ser.write(b"0,0\n")
            time.sleep(0.02)
            ser.close()
        sock.close()
        ctx.term()
        print("[BRIDGE] Shutdown complete")


if __name__ == "__main__":
    main()
