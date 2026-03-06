#!/usr/bin/env python3
"""
Simple Direct Joystick Control - NO LATENCY
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
    
    # ZMQ - direct connection
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f"tcp://{jetson_ip}:5555")
    
    DEADZONE = 0.05
    MAX_SPEED = 100
    
    print(f"[JOYSTICK] Running - DIRECT MODE, no latency")
    
    while True:
        try:
            pygame.event.pump()
            
            # Thrustmaster TCA axes:
            # Axis 0 = X (left/right tilt)
            # Axis 1 = Y (forward/back tilt, forward = negative)
            # Axis 2 = Z (twist)
            
            raw_x = joystick.get_axis(0)
            raw_y = joystick.get_axis(1)
            raw_z = joystick.get_axis(2) if joystick.get_numaxes() > 2 else 0
            
            # Simple deadzone
            def dz(v):
                return 0 if abs(v) < DEADZONE else v
            
            x = dz(raw_x)
            y = dz(raw_y)
            z = dz(raw_z)
            
            # Protocol: y=forward, x=curve turn, z=spot rotate
            forward = int(-y * MAX_SPEED)
            turn = int(x * MAX_SPEED)     # X (stick) -> curve turn
            rotate = int(z * MAX_SPEED)    # Z (twist) -> spot rotate
            
            sock.send_json({"y": forward, "x": turn, "z": rotate}, zmq.NOBLOCK)
            
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
    jetson_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.2.100"
    joystick_worker(q, jetson_ip)
