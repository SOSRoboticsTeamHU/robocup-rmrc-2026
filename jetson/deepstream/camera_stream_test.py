#!/usr/bin/env python3
"""
RoboCup Rescue RMRC 2026 - Camera Stream Test (No YOLO)
=========================================================
Pure camera streaming for hardware testing - no inference.

Usage:
  python3 camera_stream_test.py
  python3 camera_stream_test.py --fps 30 --resolution 1280 720
  python3 camera_stream_test.py --cameras front arm
"""

import sys
import os
import json
import time
import threading
from pathlib import Path

# Add parent dirs
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
JETSON_DIR = os.path.dirname(SCRIPT_DIR)
REPO_ROOT = os.path.dirname(JETSON_DIR)
for p in [REPO_ROOT, JETSON_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from shared.constants import (
        ZMQ_PORT_CAMERA, CAMERA_WIDTH, CAMERA_HEIGHT,
        CAMERA_FPS, JPEG_QUALITY, CAMERAS
    )
except ImportError:
    ZMQ_PORT_CAMERA = 5557
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    CAMERA_FPS = 15
    JPEG_QUALITY = 65
    CAMERAS = {"front": "/dev/video0", "arm": "/dev/video2", "backward": "/dev/video4"}

try:
    import zmq
except ImportError:
    zmq = None

try:
    import numpy as np
    import cv2
except ImportError:
    np = None
    cv2 = None

# Import GStreamer camera reader
try:
    from deepstream_yolo_node import GstCameraReader, Gst, GLib, GST_AVAILABLE
except ImportError:
    print("[CAMTEST] Error: Cannot import GstCameraReader")
    sys.exit(1)


class SimpleCameraStream:
    """Simple multi-camera streaming without inference."""
    
    def __init__(self, port: int = ZMQ_PORT_CAMERA, camera_ids: tuple = None,
                 width: int = None, height: int = None, fps: int = None):
        self.port = port
        self.camera_ids = camera_ids or tuple(CAMERAS.keys())
        self.width = width or CAMERA_WIDTH
        self.height = height or CAMERA_HEIGHT
        self.target_fps = fps or min(CAMERA_FPS, 15)
        
        self.cameras: dict = {}
        self.context = None
        self.socket = None
        self.running = False
        self.stream_thread = None
        self.main_loop = None
        
        # Stats
        self.frame_counters = {}
        self.fps_counters = {}
        self.last_fps_print = time.time()
    
    def start(self):
        """Initialize cameras and start streaming."""
        if not GST_AVAILABLE:
            print("[CAMTEST] ❌ GStreamer not available!")
            return
        if not zmq:
            print("[CAMTEST] ❌ pyzmq not available!")
            return
        
        Gst.init(None)
        
        print("[CAMTEST] 🎥 Camera Stream Test (No YOLO)")
        print(f"[CAMTEST] Resolution: {self.width}x{self.height}")
        print(f"[CAMTEST] Target FPS: {self.target_fps}")
        print(f"[CAMTEST] Cameras: {list(self.camera_ids)}")
        print()
        
        # Initialize selected cameras
        OPEN_DELAY = 2.0
        for cam_id in self.camera_ids:
            device = CAMERAS.get(cam_id)
            if not device:
                print(f"[CAMTEST] ⚠️  Unknown camera ID: {cam_id}")
                continue
            
            if not os.path.exists(device):
                print(f"[CAMTEST] ❌ {cam_id} - device not found: {device}")
                continue
            
            if self.cameras:
                print(f"[CAMTEST] Waiting {OPEN_DELAY}s before next camera...")
                time.sleep(OPEN_DELAY)
            
            reader = GstCameraReader(cam_id, device, width=self.width, height=self.height)
            if reader.build_and_connect():
                self.cameras[cam_id] = reader
                self.frame_counters[cam_id] = 0
                self.fps_counters[cam_id] = []
                print(f"[CAMTEST] ✅ {cam_id} ready: {device}")
            else:
                print(f"[CAMTEST] ❌ {cam_id} failed: {device}")
        
        if not self.cameras:
            print("[CAMTEST] ❌ No cameras initialized!")
            return
        
        print(f"\n[CAMTEST] Starting {len(self.cameras)} camera(s)...")
        
        # Start camera pipelines
        for reader in self.cameras.values():
            reader.set_playing()
        
        # Setup ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 60)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f"tcp://*:{self.port}")
        print(f"[CAMTEST] 🌐 ZMQ streaming on port {self.port}")
        
        # Start streaming thread
        self.running = True
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()
        
        print("[CAMTEST] 🚀 Streaming active!")
        print("[CAMTEST] Press Ctrl+C to stop\n")
        
        # Main loop
        if GLib is not None:
            self.main_loop = GLib.MainLoop()
            try:
                self.main_loop.run()
            except KeyboardInterrupt:
                pass
        else:
            while self.running:
                time.sleep(1)
    
    def _stream_loop(self):
        """Main streaming loop."""
        frame_interval = 1.0 / self.target_fps
        
        while self.running:
            loop_start = time.time()
            
            for cam_id, reader in self.cameras.items():
                frame = reader.get_frame()
                if frame is None:
                    continue
                
                self.frame_counters[cam_id] += 1
                self.fps_counters[cam_id].append(time.time())
                
                # Keep only last 30 timestamps for FPS calculation
                if len(self.fps_counters[cam_id]) > 30:
                    self.fps_counters[cam_id].pop(0)
                
                # Encode JPEG
                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                
                # Build header
                header = {
                    "msg_type": "camera_frame",
                    "camera_id": cam_id,
                    "timestamp": time.time(),
                    "frame_num": self.frame_counters[cam_id],
                    "width": frame.shape[1],
                    "height": frame.shape[0],
                    "fps": self.target_fps,
                    "test_mode": True,  # Flag for GUI
                    "detections": [],
                    "qr_codes": [],
                }
                
                # Send via ZMQ
                try:
                    self.socket.send_multipart([json.dumps(header).encode(), jpeg.tobytes()], zmq.NOBLOCK)
                except zmq.Again:
                    pass
            
            # Print stats every 5 seconds
            now = time.time()
            if now - self.last_fps_print >= 5.0:
                self._print_stats()
                self.last_fps_print = now
            
            # Rate limiting
            elapsed = time.time() - loop_start
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)
    
    def _print_stats(self):
        """Print streaming statistics."""
        print("\n[CAMTEST] 📊 Stats:")
        for cam_id in self.cameras:
            frame_count = self.frame_counters[cam_id]
            timestamps = self.fps_counters[cam_id]
            
            if len(timestamps) >= 2:
                elapsed = timestamps[-1] - timestamps[0]
                actual_fps = len(timestamps) / elapsed if elapsed > 0 else 0
            else:
                actual_fps = 0
            
            print(f"  {cam_id:8s}: {frame_count:6d} frames @ {actual_fps:5.1f} FPS")
    
    def stop(self):
        """Cleanup and stop."""
        print("\n[CAMTEST] Stopping...")
        self.running = False
        
        if self.main_loop and GLib is not None:
            self.main_loop.quit()
        
        if self.stream_thread:
            self.stream_thread.join(timeout=2.0)
        
        for reader in self.cameras.values():
            reader.stop()
        
        if self.socket:
            self.socket.close()
        if self.context:
            self.context.term()
        
        print("[CAMTEST] ✅ Stopped")


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Simple camera stream test (no YOLO)")
    ap.add_argument("--port", type=int, default=ZMQ_PORT_CAMERA, help="ZMQ stream port")
    ap.add_argument("--cameras", nargs="+", default=None, help="Camera IDs to stream (default: all)")
    ap.add_argument("--resolution", nargs=2, type=int, metavar=("WIDTH", "HEIGHT"), 
                    default=None, help="Camera resolution (default: 640 480)")
    ap.add_argument("--fps", type=int, default=None, help="Target FPS (default: 15)")
    args = ap.parse_args()
    
    width, height = args.resolution if args.resolution else (CAMERA_WIDTH, CAMERA_HEIGHT)
    
    stream = SimpleCameraStream(
        port=args.port,
        camera_ids=tuple(args.cameras) if args.cameras else None,
        width=width,
        height=height,
        fps=args.fps
    )
    
    try:
        stream.start()
    except KeyboardInterrupt:
        print("\n[CAMTEST] Interrupt")
    finally:
        stream.stop()


if __name__ == "__main__":
    main()
