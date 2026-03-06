#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Raspberry Pi Camera Streamer
=======================================================
Streams one or more USB cameras from Raspberry Pi to the operator station GUI.

Usage (on Raspberry Pi):
    python camera_stream.py                           # Two cameras: front (0), arm (1)
    python camera_stream.py --cameras "front:0"       # Single camera
    python camera_stream.py --cameras "front:0,arm:1,left:2"  # Three cameras

Arguments:
    --port      ZMQ port to bind (default: 5557)
    --cameras   Comma-separated "id:device" pairs (default: "front:0,arm:1")
    --fps       Target FPS per camera (default: 15)
    --width     Frame width (default: 640)
    --height    Frame height (default: 480)
    --quality   JPEG quality 0-100 (default: 65)
"""

import cv2
import zmq
import json
import time
import argparse
from typing import List, Tuple


def parse_cameras(s: str) -> List[Tuple[str, int]]:
    """Parse 'front:0,arm:1' into [('front', 0), ('arm', 1)]."""
    result = []
    for part in s.strip().split(","):
        part = part.strip()
        if ":" in part:
            name, dev = part.split(":", 1)
            result.append((name.strip(), int(dev.strip())))
        elif part:
            result.append((part, len(result)))  # default device index by order
    return result


def main():
    parser = argparse.ArgumentParser(description='Pi Camera Streamer')
    parser.add_argument('--port', type=int, default=5557,
                        help='ZMQ port to bind (default: 5557)')
    parser.add_argument('--cameras', type=str, default='left:0,right:1',
                        help='Comma-separated id:device (default: left:0,right:1 → Lower Forward / Backward in GUI)')
    parser.add_argument('--fps', type=int, default=15,
                        help='Target FPS per camera (default: 15)')
    parser.add_argument('--width', type=int, default=640,
                        help='Frame width (default: 640)')
    parser.add_argument('--height', type=int, default=480,
                        help='Frame height (default: 480)')
    parser.add_argument('--quality', type=int, default=65,
                        help='JPEG quality 0-100 (default: 65)')
    args = parser.parse_args()

    camera_specs = parse_cameras(args.cameras)
    if not camera_specs:
        print("[PI-CAM] ERROR: No cameras specified. Use --cameras e.g. 'front:0,arm:1'")
        return

    # Open all cameras
    caps = []  # list of (camera_id, VideoCapture, frame_count, last_fps_time, fps_calc)
    for camera_id, device in camera_specs:
        print(f"[PI-CAM] Opening camera '{camera_id}' on device {device}...")
        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            print(f"[PI-CAM] ERROR: Could not open camera {device} ({camera_id}), skipping")
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[PI-CAM]   {camera_id}: {w}x{h}")
        caps.append({
            "camera_id": camera_id,
            "cap": cap,
            "frame_count": 0,
            "last_fps_time": time.time(),
            "fps_calc": 0.0,
        })

    if not caps:
        print("[PI-CAM] No cameras opened. Try: ls /dev/video*")
        return

    # Setup ZMQ publisher
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 4)  # Buffer for multiple cameras
    socket.setsockopt(zmq.LINGER, 0)
    bind_str = f"tcp://*:{args.port}"
    print(f"[PI-CAM] Binding to {bind_str}...")
    socket.bind(bind_str)

    names = [c["camera_id"] for c in caps]
    print(f"[PI-CAM] Streaming {len(caps)} camera(s): {names} at {args.fps} FPS")
    print("[PI-CAM] Viewers can connect to tcp://<PI_IP>:5557")
    print("[PI-CAM] Press Ctrl+C to stop")

    frame_interval = 1.0 / args.fps

    try:
        while True:
            loop_start = time.time()

            for ent in caps:
                cap = ent["cap"]
                ret, frame = cap.read()
                if not ret:
                    continue

                ent["frame_count"] += 1
                now = time.time()
                if now - ent["last_fps_time"] >= 1.0:
                    ent["fps_calc"] = ent["frame_count"] / (now - ent["last_fps_time"])
                    ent["frame_count"] = 0
                    ent["last_fps_time"] = now

                _, jpeg = cv2.imencode('.jpg', frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, args.quality])
                header = {
                    "camera_id": ent["camera_id"],
                    "timestamp": time.time(),
                    "frame_num": ent["frame_count"],
                    "width": frame.shape[1],
                    "height": frame.shape[0],
                    "fps": round(ent["fps_calc"], 1),
                    "detections": [],
                    "qr_codes": [],
                }
                try:
                    socket.send_multipart([
                        json.dumps(header).encode(),
                        jpeg.tobytes(),
                    ], zmq.NOBLOCK)
                except zmq.Again:
                    pass

            elapsed = time.time() - loop_start
            if frame_interval > elapsed:
                time.sleep(frame_interval - elapsed)

    except KeyboardInterrupt:
        print("\n[PI-CAM] Stopping...")
    finally:
        for ent in caps:
            ent["cap"].release()
        socket.close()
        context.term()
        print("[PI-CAM] Stopped")


if __name__ == "__main__":
    main()
