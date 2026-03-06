#!/usr/bin/env python3
"""
Clean Joystick Controller - Simple and Stable
==============================================
Y axis (1) = forward/back
X axis (0) = curve turning
Z axis (2) = spot rotation
"""

import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"

import time
import sys


def joystick_worker(queue, jetson_ip):
    import pygame
    import zmq
    
    pygame.init()
    
    if pygame.joystick.get_count() == 0:
        print("[JOYSTICK] No joystick found!")
        return
    
    # Find joystick
    joystick = None
    for i in range(pygame.joystick.get_count()):
        js = pygame.joystick.Joystick(i)
        js.init()
        print(f"[JOYSTICK] Found: {js.get_name()}")
        if "thrustmaster" in js.get_name().lower():
            joystick = js
            break
    if joystick is None:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
    
    print(f"[JOYSTICK] Using: {joystick.get_name()}")
    print(f"[JOYSTICK] Axes: {joystick.get_numaxes()}")
    
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
    
    # State
    last_fwd, last_turn, last_rot = 0, 0, 0
    
    print("[JOYSTICK] Axis mapping:")
    print("  Y (axis 1) = forward/back")
    print("  X (axis 0) = curve turn")
    print("  Z (axis 2) = spot rotate")
    print("[JOYSTICK] Running...")
    
    while True:
        try:
            pygame.event.pump()
            
            # Read axes
            raw_y = joystick.get_axis(1)  # Y = forward/back
            raw_x = joystick.get_axis(0)  # X = curve turn
            raw_z = joystick.get_axis(2) if joystick.get_numaxes() > 2 else 0
            
            # Apply deadzone
            def dz(v):
                if abs(v) < DEADZONE:
                    return 0.0
                sign = 1 if v > 0 else -1
                return sign * (abs(v) - DEADZONE) / (1 - DEADZONE)
            
            y = dz(raw_y)
            x = dz(raw_x)
            z = dz(raw_z)
            
            # Protocol: y=forward, x=curve turn, z=spot rotate
            forward = int(-y * MAX_FWD)
            turn = int(x * MAX_TURN)      # X (stick) -> curve turn
            rotate = int(-z * MAX_ROT)   # Z (twist) -> spot rotate
            
            if forward != last_fwd or turn != last_turn or rotate != last_rot:
                sock.send_json({"y": forward, "x": turn, "z": rotate}, zmq.NOBLOCK)
                last_fwd, last_turn, last_rot = forward, turn, rotate
            
            # GUI queue
            try:
                if queue.full():
                    queue.get_nowait()
                queue.put_nowait({"y": forward/100, "x": turn/100, "z": rotate/100})
            except:
                pass
            
            time.sleep(0.01)  # 100Hz
            
        except Exception as e:
            print(f"[JOYSTICK] Error: {e}")
            time.sleep(0.1)


if __name__ == "__main__":
    from multiprocessing import Queue
    q = Queue(maxsize=1)
    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.2.100"
    joystick_worker(q, ip)
