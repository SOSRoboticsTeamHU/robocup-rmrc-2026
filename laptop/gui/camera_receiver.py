#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Camera Receiver
==========================================
Receives camera streams from Jetson via ZMQ and provides
frames to the GUI for display.

Features:
- Multi-camera stream reception
- JPEG decoding
- Frame rate limiting
- Detection overlay support
- Thread-safe frame access
"""

import cv2
import json
import time
import threading
import numpy as np
from typing import Dict, Optional, Callable, List
from dataclasses import dataclass
import sys
import os

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False
    print("[CAMERA] WARNING: pyzmq not available")

# Add shared module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'shared'))

try:
    from shared.constants import JETSON_IP, ZMQ_PORT_CAMERA, QR_DECODE_ON_LAPTOP, QR_DECODE_INTERVAL
except ImportError:
    JETSON_IP = "192.168.2.100"
    ZMQ_PORT_CAMERA = 5557
    QR_DECODE_ON_LAPTOP = True
    QR_DECODE_INTERVAL = 2

try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except ImportError:
    PYZBAR_AVAILABLE = False


@dataclass
class CameraFrame:
    """Container for a received camera frame."""
    camera_id: str
    frame: np.ndarray
    timestamp: float
    width: int
    height: int
    frame_num: int
    detections: List[dict] = None
    qr_codes: List[dict] = None
    landolt_readings: List[dict] = None  # Landolt-C direction readings


class CameraReceiver:
    """
    Receives camera streams from Jetson and manages frame buffers.
    ALL ZMQ on main thread only - polling via poll() to avoid libzmq fq.cpp crash.
    Call poll() from main window's _update_cameras (QTimer).
    """
    
    def __init__(
        self,
        jetson_ip: str = JETSON_IP,
        port: int = ZMQ_PORT_CAMERA,
        pi_ip: Optional[str] = None,
        decode_qr_on_receive: Optional[bool] = None,
        qr_decode_interval: Optional[int] = None,
        zmq_context: Optional["zmq.Context"] = None,
    ):
        self.jetson_ip = jetson_ip
        self.port = port
        self.pi_ip = None
        self.decode_qr_on_receive = decode_qr_on_receive if decode_qr_on_receive is not None else QR_DECODE_ON_LAPTOP
        self.qr_decode_interval = max(1, qr_decode_interval if qr_decode_interval is not None else QR_DECODE_INTERVAL)
        self._last_qr_codes: Dict[str, List[dict]] = {}
        self._shared_zmq_context = zmq_context
        self._owns_context = zmq_context is None

        if self.decode_qr_on_receive and not PYZBAR_AVAILABLE:
            print("[CAMERA] WARNING: QR decode on laptop enabled but pyzbar not installed. "
                  "Install: pip install pyzbar, and on macOS: brew install zbar")
        elif self.decode_qr_on_receive:
            print("[CAMERA] QR decoding on laptop (every {} frame(s))".format(self.qr_decode_interval))

        self.zmq_context: Optional[zmq.Context] = None
        self.socket: Optional[zmq.Socket] = None
        self.frames: Dict[str, CameraFrame] = {}
        self.frame_lock = threading.Lock()
        self.running = False
        self.connected = False
        self.total_frames = 0
        self.frames_per_camera: Dict[str, int] = {}
        self.last_frame_time: Dict[str, float] = {}
        self.start_time = 0
        self.on_frame_received: Optional[Callable[[str, np.ndarray], None]] = None
        self.on_detection: Optional[Callable[[str, List[dict]], None]] = None
    
    def disconnect(self):
        """Disconnect from camera stream."""
        self.running = False
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
            self.socket = None
        if self.zmq_context and self._owns_context:
            try:
                self.zmq_context.term()
            except Exception:
                pass
        self.zmq_context = None
        print("[CAMERA] Disconnected")
    
    def start(self):
        """Create socket on main thread. Call poll() from main thread (e.g. _update_cameras)."""
        if not ZMQ_AVAILABLE:
            print("[CAMERA] ZMQ not available")
            return
        self.stop()
        try:
            ctx = self._shared_zmq_context if self._shared_zmq_context is not None else zmq.Context()
            self.zmq_context = ctx
            self.socket = ctx.socket(zmq.SUB)
            self.socket.setsockopt(zmq.SUBSCRIBE, b"")
            self.socket.setsockopt(zmq.RCVTIMEO, 0)  # Non-blocking for poll()
            self.socket.setsockopt(zmq.RCVHWM, 9)   # ~3 per camera; prevents latency buildup
            self.socket.setsockopt(zmq.CONNECT_TIMEOUT, 5000)
            self.socket.connect(f"tcp://{self.jetson_ip}:{self.port}")
            self.running = True
            self.connected = True
            self.start_time = time.time()
            print(f"[CAMERA] Connected to tcp://{self.jetson_ip}:{self.port} (3 Jetson cameras)")
        except Exception as e:
            print(f"[CAMERA] Connection error: {e}")
            self.disconnect()
            return
        print("[CAMERA] Receiver started (main-thread polling)")
    
    def stop(self):
        """Stop receiving frames."""
        self.running = False
        self.disconnect()
    
    def poll(self) -> None:
        """Drain available frames; keep only latest per camera and decode those (avoids latency buildup)."""
        if not self.socket or not self.running:
            return
        try:
            # Collect latest message per camera (don't decode in loop — decode only last per camera)
            pending = {}  # camera_id -> (header_dict, jpeg_bytes)
            while True:
                try:
                    msg_parts = self.socket.recv_multipart(zmq.NOBLOCK)
                except zmq.Again:
                    break
                if len(msg_parts) != 2:
                    continue
                try:
                    header = json.loads(msg_parts[0].decode("utf-8"))
                except Exception:
                    continue
                camera_id = header.get("camera_id", "unknown")
                pending[camera_id] = (header, bytes(msg_parts[1]))

            for camera_id, (header, jpeg_data) in pending.items():
                frame = cv2.imdecode(
                    np.frombuffer(jpeg_data, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )
                if frame is None:
                    continue
                frame_num = header.get("frame_num", 0)
                qr_codes = header.get("qr_codes") or []
                if self.decode_qr_on_receive and PYZBAR_AVAILABLE and (frame_num % self.qr_decode_interval == 0):
                    try:
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        decoded = pyzbar.decode(gray)
                        qr_codes = [{
                            "data": obj.data.decode("utf-8", errors="ignore"),
                            "type": obj.type,
                            "bbox": [obj.rect.left, obj.rect.top,
                                     obj.rect.left + obj.rect.width,
                                     obj.rect.top + obj.rect.height],
                        } for obj in decoded]
                        self._last_qr_codes[camera_id] = qr_codes
                    except Exception:
                        qr_codes = self._last_qr_codes.get(camera_id, [])
                elif self.decode_qr_on_receive and camera_id in self._last_qr_codes:
                    qr_codes = self._last_qr_codes[camera_id]
                camera_frame = CameraFrame(
                    camera_id=camera_id,
                    frame=frame,
                    timestamp=header.get("timestamp", time.time()),
                    width=header.get("width", frame.shape[1]),
                    height=header.get("height", frame.shape[0]),
                    frame_num=frame_num,
                    detections=header.get("detections"),
                    qr_codes=qr_codes,
                    landolt_readings=header.get("landolt_readings"),
                )
                with self.frame_lock:
                    self.frames[camera_id] = camera_frame
                    self.total_frames += 1
                    self.frames_per_camera[camera_id] = self.frames_per_camera.get(camera_id, 0) + 1
                    self.last_frame_time[camera_id] = time.time()
                if self.on_frame_received:
                    self.on_frame_received(camera_id, frame)
        except zmq.ZMQError:
            pass
        except Exception as e:
            if self.running:
                print(f"[CAMERA] Poll error: {e}")
    
    def get_frame(self, camera_id: str) -> Optional[np.ndarray]:
        """Get the latest frame for a camera."""
        with self.frame_lock:
            if camera_id in self.frames:
                return self.frames[camera_id].frame.copy()
        return None
    
    def get_frame_data(self, camera_id: str) -> Optional[CameraFrame]:
        """Get the full frame data including metadata."""
        with self.frame_lock:
            if camera_id in self.frames:
                return self.frames[camera_id]
        return None
    
    def get_all_frames(self) -> Dict[str, np.ndarray]:
        """Get latest frames for all cameras."""
        with self.frame_lock:
            return {
                cam_id: frame.frame.copy()
                for cam_id, frame in self.frames.items()
            }
    
    def get_camera_ids(self) -> List[str]:
        """Get list of active camera IDs."""
        with self.frame_lock:
            return list(self.frames.keys())
    
    def get_fps(self, camera_id: str = None) -> float:
        """Get current FPS for a camera or overall."""
        elapsed = time.time() - self.start_time
        if elapsed <= 0:
            return 0
        
        with self.frame_lock:
            if camera_id:
                count = self.frames_per_camera.get(camera_id, 0)
            else:
                count = self.total_frames
        
        return count / elapsed
    
    def is_camera_active(self, camera_id: str, timeout: float = 2.0) -> bool:
        """Check if camera is actively sending frames."""
        with self.frame_lock:
            last_time = self.last_frame_time.get(camera_id, 0)
        return (time.time() - last_time) < timeout
    
    def get_status(self) -> dict:
        """Get receiver status."""
        elapsed = time.time() - self.start_time if self.start_time else 0
        
        with self.frame_lock:
            cameras = {
                cam_id: {
                    "frames": self.frames_per_camera.get(cam_id, 0),
                    "active": self.is_camera_active(cam_id),
                    "last_frame_age": time.time() - self.last_frame_time.get(cam_id, 0)
                }
                for cam_id in self.frames.keys()
            }
        
        return {
            "connected": self.connected,
            "running": self.running,
            "total_frames": self.total_frames,
            "uptime": elapsed,
            "overall_fps": self.total_frames / elapsed if elapsed > 0 else 0,
            "cameras": cameras
        }


def frame_to_qimage(frame: np.ndarray):
    """Convert OpenCV frame to QImage for Qt display."""
    from PyQt5.QtGui import QImage
    
    if frame is None:
        return None
    
    # Convert BGR to RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    h, w, ch = rgb_frame.shape
    bytes_per_line = ch * w
    
    return QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)


def frame_to_pixmap(frame: np.ndarray, max_width: int = None, max_height: int = None):
    """Convert OpenCV frame to QPixmap, optionally scaled."""
    from PyQt5.QtGui import QPixmap
    from PyQt5.QtCore import Qt
    
    qimage = frame_to_qimage(frame)
    if qimage is None:
        return None
    
    pixmap = QPixmap.fromImage(qimage)
    
    if max_width or max_height:
        pixmap = pixmap.scaled(
            max_width or pixmap.width(),
            max_height or pixmap.height(),
            Qt.KeepAspectRatio,
            Qt.FastTransformation
        )
    
    return pixmap


class CameraWidget:
    """
    Helper class to manage a camera display widget in PyQt.
    """
    
    def __init__(self, camera_id: str, receiver: CameraReceiver):
        self.camera_id = camera_id
        self.receiver = receiver
        self.last_update = 0
        self.min_update_interval = 1.0 / 30  # Max 30 fps updates
    
    def get_pixmap(self, max_width: int = None, max_height: int = None):
        """Get current frame as QPixmap."""
        # Rate limit updates
        now = time.time()
        if now - self.last_update < self.min_update_interval:
            return None
        
        frame = self.receiver.get_frame(self.camera_id)
        if frame is None:
            return None
        
        self.last_update = now
        return frame_to_pixmap(frame, max_width, max_height)
    
    def is_active(self) -> bool:
        """Check if camera is receiving frames."""
        return self.receiver.is_camera_active(self.camera_id)


def main():
    """Test camera receiver."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Camera Receiver Test')
    parser.add_argument('--ip', type=str, default=JETSON_IP,
                        help='Jetson IP address')
    parser.add_argument('--port', type=int, default=ZMQ_PORT_CAMERA,
                        help='Camera stream port')
    parser.add_argument('--display', action='store_true',
                        help='Display frames with OpenCV')
    args = parser.parse_args()
    
    receiver = CameraReceiver(jetson_ip=args.ip, port=args.port)
    
    def on_frame(camera_id, frame):
        if args.display:
            cv2.imshow(camera_id, frame)
            cv2.waitKey(1)
    
    receiver.on_frame_received = on_frame
    
    try:
        receiver.start()
        print("Receiving camera frames... Press Ctrl+C to stop")
        last_print = 0
        while True:
            receiver.poll()
            if args.display:
                cv2.waitKey(1)
            else:
                time.sleep(0.05)
            now = time.time()
            if now - last_print >= 5:
                last_print = now
                status = receiver.get_status()
                print(f"[STATUS] Frames: {status['total_frames']}, "
                      f"FPS: {status['overall_fps']:.1f}, "
                      f"Cameras: {list(status['cameras'].keys())}")
            
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        receiver.stop()
        if args.display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
