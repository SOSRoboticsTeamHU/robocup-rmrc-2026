#!/usr/bin/env python3
"""
Joystick Button ID Debug Tool
=============================
Run this to see which physical button corresponds to which ID (0, 1, 2, ...).
Press each button on your joystick; the script prints "Button N pressed" / "Button N released".

Usage:
    python -m control.joystick_button_debug
    python laptop/control/joystick_button_debug.py

Press Ctrl+C to exit.
"""

import os
import sys
import time

# Avoid opening a display window (pygame)
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

def main():
    try:
        import pygame
    except ImportError:
        print("Install pygame: pip install pygame")
        sys.exit(1)

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick found. Plug in your joystick and run again.")
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()
    nbuttons = js.get_numbuttons()
    naxes = js.get_numaxes()

    print(f"Joystick: {js.get_name()}")
    print(f"Buttons: {nbuttons}  |  Axes: {naxes}")
    print()
    print("Press each button to see its ID. You will see 'Button N pressed' when you press it.")
    print("Ctrl+C to exit.")
    print("-" * 50)

    prev = [0] * max(nbuttons, 16)
    try:
        while True:
            pygame.event.pump()
            for i in range(nbuttons):
                current = 1 if js.get_button(i) else 0
                if current != prev[i]:
                    prev[i] = current
                    if current:
                        print(f"  Button {i}  PRESSED")
                    else:
                        print(f"  Button {i}  released")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nDone. Use these IDs in the GUI button mapping (see JOY_BUTTON_* in main_window.py).")

if __name__ == "__main__":
    main()
