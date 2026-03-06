#!/usr/bin/env python3
"""
Clean Joystick Controller - Simple and Stable
==============================================
Default mapping: Thrustmaster TCA Airbus Edition
  - Axis 0 = X (left/right) -> curve turn
  - Axis 1 = Y (forward/back, push forward = negative raw) -> forward/back
  - Axis 2 = Z (twist) -> spot rotation

For Logitech sticks (different axis order) use --logitech.
Use --calibrate to print raw axis values and identify which index is which.
"""

import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"

import time
import sys

# Axis mapping: Y=forward, X=curve turn (stick L/R), Z=spot rotate (twist)
# axis0 = stick left/right (X) -> curve turn
# axis1 = stick forward/back (Y) -> forward/back
# axis2 = twist (Z) -> spot rotate
AXIS_FWD = 1   # Y (forward/back)
AXIS_TURN = 0  # X (stick left/right) -> curve turn
AXIS_ROT = 2   # Z (twist) -> spot rotate


def joystick_worker(queue, jetson_ip, logitech_axis_swap=False, debug=False):
    import pygame
    import zmq

    # Default: Y=fwd, Z=curve turn, X=spot rotate. Logitech = swap fwd/rot axes.
    if logitech_axis_swap:
        axis_fwd, axis_turn, axis_rot = 2, 0, 1
        print("[JOYSTICK] Using Logitech: fwd=axis2, turn=axis0, rot=axis1")
    else:
        axis_fwd, axis_turn, axis_rot = AXIS_FWD, AXIS_TURN, AXIS_ROT
        print("[JOYSTICK] Mapping: Y(axis1)=fwd/back, X(axis0)=curve turn, Z(axis2)=spot rotate")

    pygame.init()

    if pygame.joystick.get_count() == 0:
        print("[JOYSTICK] No joystick found!")
        return

    # Find joystick (prefer Thrustmaster, else first)
    joystick = None
    for i in range(pygame.joystick.get_count()):
        js = pygame.joystick.Joystick(i)
        js.init()
        print(f"[JOYSTICK] Found: {js.get_name()}")
        if "thrustmaster" in js.get_name().lower() or "tca" in js.get_name().lower():
            joystick = js
            break
    if joystick is None:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

    print(f"[JOYSTICK] Using: {joystick.get_name()}")
    naxes = joystick.get_numaxes()
    nbuttons = joystick.get_numbuttons()
    print(f"[JOYSTICK] Axes: {naxes}, Buttons: {nbuttons}")

    # ZMQ
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f"tcp://{jetson_ip}:5555")

    # Settings
    DEADZONE = 0.05
    MAX_FWD = 100
    MAX_TURN = 80
    MAX_ROT = 90
    SMOOTH_ALPHA = 0.28   # Exponential smoothing: lower = smoother, less responsive
    SEND_INTERVAL = 0.02  # 50 Hz fixed rate = continuous stream, no stutter

    # State
    smooth_fwd, smooth_turn, smooth_rot = 0.0, 0.0, 0.0
    last_fwd, last_turn, last_rot = 0, 0, 0
    debug_interval = 0.2
    last_debug = 0.0
    last_send_time = 0.0

    def dz(v):
        if abs(v) < DEADZONE:
            return 0.0
        sign = 1 if v > 0 else -1
        return sign * (abs(v) - DEADZONE) / (1 - DEADZONE)

    def get_axis(idx):
        if idx >= naxes:
            return 0.0
        return joystick.get_axis(idx)

    def ema(smoothed, raw, alpha):
        return alpha * raw + (1.0 - alpha) * smoothed

    print("[JOYSTICK] Running... (smoothed @ 50 Hz for continuous drive)")

    while True:
        try:
            pygame.event.pump()

            raw_fwd = get_axis(axis_fwd)
            raw_turn = get_axis(axis_turn)
            raw_rot = get_axis(axis_rot)

            smooth_fwd = ema(smooth_fwd, raw_fwd, SMOOTH_ALPHA)
            smooth_turn = ema(smooth_turn, raw_turn, SMOOTH_ALPHA)
            smooth_rot = ema(smooth_rot, raw_rot, SMOOTH_ALPHA)

            y = dz(smooth_fwd)
            x = dz(smooth_turn)
            z = dz(smooth_rot)

            forward = int(-y * MAX_FWD)
            turn = int(x * MAX_TURN)
            rotate = int(-z * MAX_ROT)

            if debug:
                t = time.perf_counter()
                if t - last_debug >= debug_interval:
                    print(f"[JOYSTICK] raw axis{axis_fwd}={raw_fwd:.2f} axis{axis_turn}={raw_turn:.2f} axis{axis_rot}={raw_rot:.2f} -> y={forward} x={turn} z={rotate}")
                    last_debug = t

            t = time.perf_counter()
            if (t - last_send_time) >= SEND_INTERVAL:
                try:
                    sock.send_json({"y": forward, "x": turn, "z": rotate}, zmq.NOBLOCK)
                    last_fwd, last_turn, last_rot = forward, turn, rotate
                except zmq.Again:
                    pass
                last_send_time = t  # always advance so we keep sending at 50 Hz

            # GUI queue (axes + buttons for Start/Stop/Lap/Report/YOLO/QR)
            try:
                if queue is not None and queue.full():
                    queue.get_nowait()
                if queue is not None:
                    buttons = [1 if joystick.get_button(i) else 0 for i in range(nbuttons)] if nbuttons else []
                    # Negate z for display so GUI indicator matches robot (drive_bridge negates rot)
                    queue.put_nowait({"y": forward/100, "x": turn/100, "z": -rotate/100, "buttons": buttons})
            except Exception:
                pass

            time.sleep(0.01)  # 100Hz

        except Exception as e:
            print(f"[JOYSTICK] Error: {e}")
            time.sleep(0.1)


def calibrate_joystick():
    """Print raw axis values so you can see which physical axis is which index."""
    import pygame
    pygame.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick found.")
        return
    js = pygame.joystick.Joystick(0)
    js.init()
    n = js.get_numaxes()
    print(f"Joystick: {js.get_name()}, axes: {n}")
    print("Move ONE axis at a time and note which index changes.")
    print("Format: axis0 axis1 axis2 ... (values -1.0 to 1.0)")
    print("Press Ctrl+C to exit.\n")
    try:
        while True:
            pygame.event.pump()
            vals = [js.get_axis(i) for i in range(n)]
            # Only print if any axis outside deadzone
            if any(abs(v) > 0.05 for v in vals):
                print(f"  {' '.join(f'a{i}:{v:+.2f}' for i, v in enumerate(vals))}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nDone.")


if __name__ == "__main__":
    from multiprocessing import Queue
    import argparse
    ap = argparse.ArgumentParser(description="Joystick controller -> ZMQ 5555")
    ap.add_argument("--logitech", action="store_true", help="Use Logitech axis mapping only if not Thrustmaster")
    ap.add_argument("--debug", action="store_true", help="Print axis values every 200ms")
    ap.add_argument("--calibrate", action="store_true", help="Print raw axes to identify mapping (no ZMQ)")
    ap.add_argument("jetson_ip", nargs="?", default="192.168.2.100", help="Jetson IP")
    args = ap.parse_args()

    if args.calibrate:
        calibrate_joystick()
        sys.exit(0)

    q = Queue(maxsize=1)
    joystick_worker(q, args.jetson_ip, logitech_axis_swap=args.logitech, debug=args.debug)
