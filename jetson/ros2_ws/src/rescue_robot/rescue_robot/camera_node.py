#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Camera Streaming Node
================================================
Captures video from multiple cameras and streams via ZMQ.

Features:
- Multi-camera support (front, arm, backward)
- JPEG compression for bandwidth efficiency
- Configurable resolution and FPS
- Auto-reconnect on camera disconnect
- 3rd camera (backward): lower res (320x240) and longer delay to avoid USB "No space left on device".
  If 3rd camera still fails, try: start_robot.sh --gst-camera (GStreamer multi-cam).
"""

import cv2
import json
import time
import threading
import zmq
import numpy as np
from typing import Dict, Optional, List
from dataclasses import dataclass
import sys
import os

# Add shared module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'shared'))

try:
    from shared.constants import (
        CAMERAS, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
        JPEG_QUALITY, ZMQ_PORT_CAMERA, CAMERA_STREAM_HZ
    )
except ImportError:
    CAMERAS = {
        "front": "/dev/video0",
        "arm": "/dev/video2",
        "backward": "/dev/video4",
    }
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    CAMERA_FPS = 30
    JPEG_QUALITY = 80
    ZMQ_PORT_CAMERA = 5557
    CAMERA_STREAM_HZ = 30

# Slot names for auto-discovered cameras (order matters)
CAMERA_SLOT_NAMES = ["front", "arm", "backward"]


def discover_cameras(max_cameras: int = 4, try_indices: List[int] = None) -> Dict[str, str]:
    """
    Try /dev/video0, video1, ... and return a dict slot_name -> device for each that opens.
    Use this when fixed CAMERAS mapping doesn't match your system (e.g. only video0, video1, video2 exist).
    """
    if try_indices is None:
        try_indices = list(range(8))  # video0 .. video7
    result = {}
    for i in try_indices:
        if len(result) >= max_cameras:
            break
        device = f"/dev/video{i}"
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(device)
        if cap.isOpened():
            cap.release()
            name = CAMERA_SLOT_NAMES[len(result)]
            result[name] = device
            print(f"[CAMERA] Discovered {name} -> {device}")
    return result


@dataclass
class CameraConfig:
    """Configuration for a single camera."""
    device: str
    name: str
    width: int = CAMERA_WIDTH
    height: int = CAMERA_HEIGHT
    fps: int = CAMERA_FPS
    jpeg_quality: int = JPEG_QUALITY
    enabled: bool = True


class CameraCapture:
    """Handles capture from a single camera."""
    
    def __init__(self, config: CameraConfig):
        self.config = config
        self.cap: Optional[cv2.VideoCapture] = None
        self.connected = False
        self.frame_count = 0
        self.last_frame: Optional[np.ndarray] = None
        self.last_frame_time = 0
        self.lock = threading.Lock()
        
    def connect(self) -> bool:
        """Connect to camera device. Use 640x480 and low buffer to avoid USB 'No space left on device'."""
        try:
            self.cap = cv2.VideoCapture(self.config.device, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(self.config.device)
            if not self.cap.isOpened():
                print(f"[CAM:{self.config.name}] Failed to open {self.config.device}")
                return False
            # Reduce load: small buffer first, then resolution (many USB cams need this order)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
            self.cap.set(cv2.CAP_PROP_FPS, min(self.config.fps, 15))  # cap FPS for multi-cam USB
            # One dummy read so driver commits the pipeline (helps 3rd camera on Jetson)
            self.cap.read()
            time.sleep(0.2)  # let USB settle after first read
            actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            self.connected = True
            print(f"[CAM:{self.config.name}] Connected: {actual_w}x{actual_h}@{actual_fps}fps")
            return True
            
        except Exception as e:
            print(f"[CAM:{self.config.name}] Connection error: {e}")
            return False
    
    def disconnect(self):
        """Release camera."""
        if self.cap:
            self.cap.release()
        self.connected = False
        print(f"[CAM:{self.config.name}] Disconnected")
    
    def read_frame(self) -> Optional[np.ndarray]:
        """Read a frame from the camera."""
        if not self.cap or not self.connected:
            return None
        
        try:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.last_frame = frame
                    self.last_frame_time = time.time()
                    self.frame_count += 1
                return frame
            else:
                # Camera may have disconnected
                self.connected = False
                return None
        except Exception as e:
            print(f"[CAM:{self.config.name}] Read error: {e}")
            self.connected = False
            return None
    
    def get_last_frame(self) -> Optional[np.ndarray]:
        """Get the last captured frame."""
        with self.lock:
            return self.last_frame.copy() if self.last_frame is not None else None
    
    def encode_jpeg(self, frame: np.ndarray) -> Optional[bytes]:
        """Encode frame as JPEG; resize to config size if larger (saves USB/CPU with 3 cams)."""
        try:
            h, w = frame.shape[:2]
            if w > self.config.width or h > self.config.height:
                frame = cv2.resize(frame, (self.config.width, self.config.height), interpolation=cv2.INTER_AREA)
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality]
            ret, jpeg = cv2.imencode('.jpg', frame, encode_params)
            if ret:
                return jpeg.tobytes()
        except Exception as e:
            print(f"[CAM:{self.config.name}] Encode error: {e}")
        return None


class MultiCameraStreamer:
    """
    Manages multiple cameras and streams frames via ZMQ.
    """
    
    def __init__(self, camera_configs: Dict[str, CameraConfig] = None):
        # Default camera configurations
        if camera_configs is None:
            camera_configs = {
                name: CameraConfig(device=device, name=name)
                for name, device in CAMERAS.items()
            }
        
        self.configs = camera_configs
        self.cameras: Dict[str, CameraCapture] = {}
        
        # ZMQ setup (SNDHWM so we don't drop 3rd/4th camera when subscriber is slow)
        self.zmq_context = zmq.Context()
        self.pub_socket = self.zmq_context.socket(zmq.PUB)
        self.pub_socket.setsockopt(zmq.SNDHWM, 30)
        self.pub_socket.setsockopt(zmq.LINGER, 0)
        
        # Threading
        self.running = False
        self.capture_threads: Dict[str, threading.Thread] = {}
        self.stream_thread: Optional[threading.Thread] = None
        
        # Statistics
        self.total_frames = 0
        self.start_time = 0
    
    def start(self):
        """Start camera streaming."""
        print("[CAMERA] Starting multi-camera streamer...")
        
        # Bind ZMQ socket
        bind_addr = f"tcp://*:{ZMQ_PORT_CAMERA}"
        try:
            self.pub_socket.bind(bind_addr)
            print(f"[CAMERA] Publishing on {bind_addr}")
        except zmq.ZMQError as e:
            print(f"[CAMERA] ZMQ bind error: {e}")
            return
        
        # Initialize cameras one by one with delay (avoids "No space left on device" with 3+ USB cams)
        OPEN_DELAY = 2.0  # seconds between opening each device
        EXTRA_DELAY_BEFORE_THIRD = 1.5  # extra settle time before 3rd camera (USB bandwidth)
        for idx, (name, config) in enumerate(self.configs.items()):
            if config.enabled:
                if self.cameras:
                    time.sleep(OPEN_DELAY)
                    if len(self.cameras) == 2:
                        time.sleep(EXTRA_DELAY_BEFORE_THIRD)
                camera = CameraCapture(config)
                if camera.connect():
                    self.cameras[name] = camera
                else:
                    print(f"[CAMERA] Skipping {name} - not available")
        
        if not self.cameras:
            print("[CAMERA] WARNING: No cameras available!")
        
        self.running = True
        self.start_time = time.time()
        
        # Start capture threads (one per camera)
        for name, camera in self.cameras.items():
            thread = threading.Thread(
                target=self._capture_loop,
                args=(name, camera),
                daemon=True
            )
            thread.start()
            self.capture_threads[name] = thread
        
        # Start streaming thread
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()
        
        print(f"[CAMERA] Started with {len(self.cameras)} cameras")
    
    def stop(self):
        """Stop camera streaming."""
        print("[CAMERA] Stopping...")
        self.running = False
        
        # Wait for threads
        for thread in self.capture_threads.values():
            thread.join(timeout=1.0)
        
        if self.stream_thread:
            self.stream_thread.join(timeout=1.0)
        
        # Disconnect cameras
        for camera in self.cameras.values():
            camera.disconnect()
        
        # Close ZMQ
        self.pub_socket.close()
        self.zmq_context.term()
        
        # Print statistics
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            fps = self.total_frames / elapsed
            print(f"[CAMERA] Stopped. Streamed {self.total_frames} frames ({fps:.1f} fps avg)")
    
    def _capture_loop(self, name: str, camera: CameraCapture):
        """Continuous capture loop for a single camera."""
        reconnect_delay = 1.0
        
        while self.running:
            if not camera.connected:
                # Try to reconnect
                time.sleep(reconnect_delay)
                if camera.connect():
                    reconnect_delay = 1.0
                else:
                    reconnect_delay = min(reconnect_delay * 2, 10.0)
                continue
            
            # Capture frame
            frame = camera.read_frame()
            if frame is None:
                continue
            
            # Small sleep to prevent CPU spinning
            time.sleep(0.001)
    
    def _stream_loop(self):
        """Stream frames from all cameras via ZMQ."""
        stream_period = 1.0 / CAMERA_STREAM_HZ
        
        while self.running:
            loop_start = time.time()
            
            for name, camera in self.cameras.items():
                if not camera.connected:
                    continue
                
                frame = camera.get_last_frame()
                if frame is None:
                    continue
                
                # Encode to JPEG
                jpeg_data = camera.encode_jpeg(frame)
                if jpeg_data is None:
                    continue
                
                # Create message header
                header = {
                    "msg_type": "camera_frame",
                    "camera_id": name,
                    "timestamp": time.time(),
                    "width": frame.shape[1],
                    "height": frame.shape[0],
                    "format": "jpeg",
                    "size": len(jpeg_data),
                    "frame_num": camera.frame_count
                }
                
                # Send as multipart message [header_json, jpeg_data]
                try:
                    self.pub_socket.send_multipart([
                        json.dumps(header).encode('utf-8'),
                        jpeg_data
                    ], zmq.NOBLOCK)
                    self.total_frames += 1
                except zmq.Again:
                    pass  # Drop frame if can't send
                except Exception as e:
                    print(f"[CAMERA] Send error: {e}")
            
            # Maintain stream rate
            elapsed = time.time() - loop_start
            if elapsed < stream_period:
                time.sleep(stream_period - elapsed)
    
    def get_status(self) -> dict:
        """Get streamer status."""
        return {
            "cameras": {
                name: {
                    "connected": cam.connected,
                    "frame_count": cam.frame_count,
                    "device": cam.config.device
                }
                for name, cam in self.cameras.items()
            },
            "total_frames": self.total_frames,
            "uptime": time.time() - self.start_time if self.start_time else 0
        }


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Camera Streaming Node')
    parser.add_argument('--cameras', type=str, nargs='+', 
                        help='Camera names to enable (default: all)')
    parser.add_argument('--scan', action='store_true',
                        help='Auto-discover cameras: try /dev/video0, video1, ... and use first 3–4 that open')
    parser.add_argument('--width', type=int, default=CAMERA_WIDTH,
                        help='Frame width')
    parser.add_argument('--height', type=int, default=CAMERA_HEIGHT,
                        help='Frame height')
    parser.add_argument('--fps', type=int, default=CAMERA_FPS,
                        help='Target FPS')
    parser.add_argument('--quality', type=int, default=JPEG_QUALITY,
                        help='JPEG quality (0-100)')
    args = parser.parse_args()
    
    # Build camera configs: from discovery or from constants
    if args.scan:
        cameras_map = discover_cameras(max_cameras=4)
        if not cameras_map:
            print("[CAMERA] No cameras found during scan. Falling back to CAMERAS from constants.")
            cameras_map = CAMERAS
    else:
        cameras_map = CAMERAS

    # 3rd camera (backward): lower resolution to reduce USB load and avoid "No space left on device"
    REDUCE_RES_FROM_INDEX = 2  # backward = 320x240 @ 10 fps
    configs = {}
    for idx, (name, device) in enumerate(cameras_map.items()):
        enabled = args.cameras is None or name in args.cameras
        w, h, f = args.width, args.height, args.fps
        if idx >= REDUCE_RES_FROM_INDEX:
            w, h, f = 320, 240, min(10, args.fps)
        configs[name] = CameraConfig(
            device=device,
            name=name,
            width=w,
            height=h,
            fps=f,
            jpeg_quality=args.quality,
            enabled=enabled
        )
    
    # Create and start streamer
    streamer = MultiCameraStreamer(configs)
    
    try:
        streamer.start()
        
        # Keep running and print status periodically
        while True:
            time.sleep(10)
            status = streamer.get_status()
            connected = sum(1 for c in status['cameras'].values() if c['connected'])
            print(f"[CAMERA] Status: {connected}/{len(status['cameras'])} cameras, "
                  f"{status['total_frames']} frames")
            
    except KeyboardInterrupt:
        print("\n[CAMERA] Interrupted")
    finally:
        streamer.stop()


if __name__ == "__main__":
    main()
