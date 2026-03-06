#!/usr/bin/env python3
"""
RViz Window Screenshot Stream (Jetson)
======================================
Captures the RViz2 window (or full screen) every 100-200ms, encodes JPEG,
and publishes via ZMQ (port 5572) for the laptop GUI.

Usage:
  # RViz already running (e.g. from stream_rviz.sh or manually):
  python3 rviz_streamer.py --port 5572 --interval-ms 150

  # Optional: launch RViz from this script:
  python3 rviz_streamer.py --launch-rviz --rviz-config jetson/rviz/slam_3rd_person.rviz --port 5572

Requires: pyzmq, opencv-python (cv2). Capture: pyautogui (preferred) or scrot.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

# Project root for shared constants
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    import zmq
except ImportError:
    print("ERROR: pip install pyzmq")
    sys.exit(1)

try:
    import cv2
    import numpy as np
except ImportError:
    print("ERROR: pip install opencv-python numpy")
    sys.exit(1)

# Capture: pyautogui, scrot, ImageMagick import, or xwd+convert
PYAUTOGUI_AVAILABLE = False
try:
    import pyautogui
    PYAUTOGUI_AVAILABLE = True
except ImportError:
    pass

_os_env = None

def _env():
    global _os_env
    if _os_env is None:
        import os
        _os_env = {**os.environ, "DISPLAY": os.environ.get("DISPLAY", ":0")}
    return _os_env


def capture_screen_scrot() -> bytes | None:
    """Capture full screen via scrot, return PNG bytes or None."""
    try:
        result = subprocess.run(
            ["scrot", "-o", "-"],
            capture_output=True,
            timeout=3,
            env=_env(),
        )
        if result.returncode == 0 and result.stdout:
            return result.stdout
        return None
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None


def capture_screen_import() -> bytes | None:
    """Capture root window via ImageMagick 'import', return PNG bytes or None."""
    try:
        # import -window root -  writes to stdout (PNG)
        result = subprocess.run(
            ["import", "-window", "root", "png:-"],
            capture_output=True,
            timeout=3,
            env=_env(),
        )
        if result.returncode == 0 and result.stdout:
            return result.stdout
        return None
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None


def capture_screen_xwd() -> bytes | None:
    """Capture via xwd -root, convert to PNG with ImageMagick convert."""
    try:
        xwd_proc = subprocess.Popen(
            ["xwd", "-root", "-silent"],
            stdout=subprocess.PIPE,
            env=_env(),
        )
        convert_proc = subprocess.run(
            ["convert", "xwd:-", "png:-"],
            stdin=xwd_proc.stdout,
            capture_output=True,
            timeout=3,
        )
        if xwd_proc.poll() is None:
            xwd_proc.kill()
        if convert_proc.returncode == 0 and convert_proc.stdout:
            return convert_proc.stdout
        return None
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None


def capture_screen_pyautogui() -> np.ndarray | None:
    """Capture full screen via pyautogui, return BGR numpy array or None."""
    if not PYAUTOGUI_AVAILABLE:
        return None
    try:
        img = pyautogui.screenshot()
        frame = np.array(img)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame
    except Exception:
        return None


def _decode_image_bytes(raw: bytes) -> np.ndarray | None:
    """Decode PNG/JPEG bytes to BGR array."""
    if not raw:
        return None
    arr = np.frombuffer(raw, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return frame


def capture_frame(use_fallbacks: bool = True) -> np.ndarray | None:
    """Capture screen; return BGR array (H, W, 3) or None."""
    frame = capture_screen_pyautogui()
    if frame is not None:
        return frame
    if not use_fallbacks:
        return None
    for capture_fn, name in [
        (capture_screen_scrot, "scrot"),
        (capture_screen_import, "ImageMagick import"),
        (capture_screen_xwd, "xwd+convert"),
    ]:
        raw = capture_fn()
        if raw:
            frame = _decode_image_bytes(raw)
            if frame is not None:
                return frame
    return None


def check_capture_once() -> bool:
    """Try capture once and print install hint if it fails. Returns True if capture works."""
    frame = capture_frame()
    if frame is not None:
        return True
    print(
        "[RViz] Screen capture failed. Install one of:\n"
        "  pip install pyautogui   (needs: sudo apt install python3-tk scrot)\n"
        "  sudo apt install scrot\n"
        "  sudo apt install imagemagick   (provides 'import')\n"
        "  sudo apt install x11-apps imagemagick   (xwd + convert)\n"
        "Then ensure DISPLAY is set (e.g. export DISPLAY=:0) and a display is available."
    )
    return False


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Capture RViz window (or full screen) and stream JPEG over ZMQ."
    )
    parser.add_argument("--port", type=int, default=5572, help="ZMQ PUB port (default: 5572)")
    parser.add_argument(
        "--interval-ms",
        type=int,
        default=150,
        help="Capture interval in ms (default: 150)",
    )
    parser.add_argument(
        "--launch-rviz",
        action="store_true",
        help="Launch rviz2 in background before streaming",
    )
    parser.add_argument(
        "--rviz-config",
        type=str,
        default=str(SCRIPT_DIR / "rviz" / "slam_3rd_person.rviz"),
        help="Path to RViz config (used if --launch-rviz)",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=75,
        help="JPEG quality 1-100 (default: 75)",
    )
    args = parser.parse_args()

    interval_sec = args.interval_ms / 1000.0
    jpeg_quality = max(1, min(100, args.jpeg_quality))

    if args.launch_rviz:
        config_path = Path(args.rviz_config)
        if not config_path.is_absolute():
            config_path = SCRIPT_DIR / args.rviz_config
        if config_path.exists():
            print(f"[RViz] Starting rviz2 -d {config_path}...")
            subprocess.Popen(
                ["rviz2", "-d", str(config_path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env={**__import__("os").environ},
            )
            time.sleep(3)
        else:
            print(f"[RViz] Config not found: {config_path}")

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 2)
    socket.setsockopt(zmq.LINGER, 0)
    bind_addr = f"tcp://*:{args.port}"
    socket.bind(bind_addr)
    print(f"[RViz] ZMQ PUB on {bind_addr} (interval {args.interval_ms} ms, JPEG quality {jpeg_quality})")

    if not check_capture_once():
        print("[RViz] Retrying capture every 5s until one of the tools is available...")
    frame_count = 0
    backoff = 0.0
    capture_hint_printed = False

    try:
        while True:
            loop_start = time.time()
            frame = capture_frame()
            if frame is None:
                if not capture_hint_printed:
                    capture_hint_printed = True
                    # check_capture_once already printed the install hint above
                backoff = min(backoff + 0.5, 5.0)
                time.sleep(backoff)
                continue
            backoff = 0.0

            _, jpeg_bytes = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
            )
            if jpeg_bytes is None:
                time.sleep(interval_sec)
                continue

            ts = time.time()
            header = json.dumps({
                "timestamp": ts,
                "frame_num": frame_count,
                "width": frame.shape[1],
                "height": frame.shape[0],
            }).encode("utf-8")

            try:
                socket.send_multipart([header, jpeg_bytes.tobytes()], zmq.NOBLOCK)
            except zmq.Again:
                pass
            frame_count += 1
            if frame_count <= 3 or frame_count % 100 == 0:
                print(f"[RViz] Frame #{frame_count} sent ({len(jpeg_bytes)} bytes)")

            elapsed = time.time() - loop_start
            if interval_sec > elapsed:
                time.sleep(interval_sec - elapsed)
    except KeyboardInterrupt:
        print("\n[RViz] Stopping...")
    finally:
        socket.close()
        context.term()
        print("[RViz] Done")


if __name__ == "__main__":
    main()
