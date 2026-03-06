#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - TensorRT Camera Node
===============================================
High-performance camera streaming with TensorRT-accelerated detection.

Features:
- 3 cameras via GStreamer V4L2 (front, arm, backward)
- TensorRT YOLOv8 inference (HAZMAT + Landolt-C) on front + arm cameras
- QR code detection via zbar
- ZMQ streaming to laptop GUI
- ~25-30 FPS with full detection pipeline

Usage:
    python3 tensorrt_camera_node.py
    python3 tensorrt_camera_node.py --no-detect  # Stream only, no inference
"""

import os
import sys
import json
import time
import threading
from queue import Queue, Empty
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from pathlib import Path

# Configuration (repo-relative)
_JETSON_DIR = Path(__file__).resolve().parent
MODELS_DIR = _JETSON_DIR / "models"
ENGINES_DIR = MODELS_DIR / "engines"

# Default paths
HAZMAT_ENGINE = ENGINES_DIR / "hazmat_yolov8n.engine"
LANDOLT_ENGINE = ENGINES_DIR / "landolt_yolov8n.engine"

# Camera config
CAMERAS = {
    "front": "/dev/video0",
    "arm": "/dev/video2",
    "backward": "/dev/video4",
}
DETECTION_CAMERAS = ("front", "arm")  # Only these get YOLO inference

# Stream config
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
JPEG_QUALITY = 70

# ZMQ ports
ZMQ_PORT_CAMERA = 5557

# Import dependencies
try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False
    print("[TRT] Warning: zmq not available")

try:
    import numpy as np
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False
    print("[TRT] Warning: numpy/cv2 not available")

try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except ImportError:
    PYZBAR_AVAILABLE = False
    print("[TRT] Warning: pyzbar not available, QR detection disabled")

# TensorRT YOLO inference
TRT_AVAILABLE = False
try:
    from ultralytics import YOLO
    TRT_AVAILABLE = True
except ImportError:
    print("[TRT] Warning: ultralytics not available")


@dataclass
class Detection:
    """Single detection result"""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    source: str = "yolo"  # "yolo", "hazmat", "landolt", "qr"


class TensorRTDetector:
    """TensorRT-accelerated YOLO detector"""
    
    def __init__(self, hazmat_engine: Path = None, landolt_engine: Path = None):
        self.hazmat_model = None
        self.landolt_model = None
        self.hazmat_classes = []
        self.landolt_classes = []
        
        # Load HAZMAT model
        if hazmat_engine and hazmat_engine.exists():
            try:
                self.hazmat_model = YOLO(str(hazmat_engine), task="detect")
                self.hazmat_classes = list(self.hazmat_model.names.values())
                print(f"[TRT] HAZMAT engine loaded: {len(self.hazmat_classes)} classes")
            except Exception as e:
                print(f"[TRT] HAZMAT load error: {e}")
        else:
            print(f"[TRT] HAZMAT engine not found: {hazmat_engine}")
        
        # Load Landolt model
        if landolt_engine and landolt_engine.exists():
            try:
                self.landolt_model = YOLO(str(landolt_engine), task="detect")
                self.landolt_classes = list(self.landolt_model.names.values())
                print(f"[TRT] Landolt engine loaded: {len(self.landolt_classes)} classes")
            except Exception as e:
                print(f"[TRT] Landolt load error: {e}")
        else:
            print(f"[TRT] Landolt engine not found: {landolt_engine}")
    
    def detect(self, frame: np.ndarray, conf: float = 0.5) -> List[Detection]:
        """Run all detectors on frame"""
        detections = []
        
        # HAZMAT detection
        if self.hazmat_model is not None:
            try:
                results = self.hazmat_model(frame, conf=conf, verbose=False)
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        detections.append(Detection(
                            class_id=cls_id,
                            class_name=self.hazmat_classes[cls_id] if cls_id < len(self.hazmat_classes) else f"hazmat_{cls_id}",
                            confidence=float(box.conf[0]),
                            bbox=tuple(map(int, box.xyxy[0].tolist())),
                            source="hazmat"
                        ))
            except Exception as e:
                pass
        
        # Landolt-C detection
        if self.landolt_model is not None:
            try:
                results = self.landolt_model(frame, conf=conf, verbose=False)
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        detections.append(Detection(
                            class_id=cls_id,
                            class_name=self.landolt_classes[cls_id] if cls_id < len(self.landolt_classes) else f"landolt_{cls_id}",
                            confidence=float(box.conf[0]),
                            bbox=tuple(map(int, box.xyxy[0].tolist())),
                            source="landolt"
                        ))
            except Exception as e:
                pass
        
        return detections


def detect_qr(frame: np.ndarray) -> List[Detection]:
    """Detect QR codes in frame"""
    if not PYZBAR_AVAILABLE:
        return []
    
    try:
        decoded = pyzbar.decode(frame)
        return [
            Detection(
                class_id=0,
                class_name=obj.data.decode('utf-8', errors='ignore'),
                confidence=1.0,
                bbox=(obj.rect.left, obj.rect.top, 
                      obj.rect.left + obj.rect.width,
                      obj.rect.top + obj.rect.height),
                source="qr"
            )
            for obj in decoded
        ]
    except:
        return []


class GStreamerCamera:
    """GStreamer V4L2 camera capture"""
    
    def __init__(self, camera_id: str, device: str, width: int, height: int, fps: int):
        self.camera_id = camera_id
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self.frame_count = 0
        self.last_frame_time = 0
        self.actual_fps = 0.0
    
    def _build_pipeline(self) -> str:
        """Build GStreamer pipeline string"""
        # Try hardware-accelerated MJPEG first, fallback to raw
        pipeline = (
            f"v4l2src device={self.device} ! "
            f"image/jpeg,width={self.width},height={self.height},framerate={self.fps}/1 ! "
            f"jpegdec ! "
            f"videoconvert ! "
            f"video/x-raw,format=BGR ! "
            f"appsink drop=1 max-buffers=2"
        )
        return pipeline
    
    def open(self) -> bool:
        """Open camera with GStreamer"""
        try:
            # Try GStreamer first
            pipeline = self._build_pipeline()
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if not self.cap.isOpened():
                # Fallback to V4L2
                dev_idx = int(self.device.replace("/dev/video", ""))
                self.cap = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            
            if self.cap.isOpened():
                print(f"[TRT] Camera {self.camera_id} opened: {self.device}")
                return True
            return False
        except Exception as e:
            print(f"[TRT] Camera {self.camera_id} error: {e}")
            return False
    
    def read(self) -> Optional[np.ndarray]:
        """Read frame from camera"""
        if self.cap is None or not self.cap.isOpened():
            return None
        
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        # Update FPS stats
        self.frame_count += 1
        now = time.time()
        if self.last_frame_time > 0:
            dt = now - self.last_frame_time
            self.actual_fps = 0.9 * self.actual_fps + 0.1 * (1.0 / dt) if dt > 0 else self.actual_fps
        self.last_frame_time = now
        
        return frame
    
    def close(self):
        """Release camera"""
        if self.cap:
            self.cap.release()
            self.cap = None


class TensorRTCameraNode:
    """Main camera streaming node with TensorRT detection"""
    
    def __init__(self, port: int = ZMQ_PORT_CAMERA, enable_detection: bool = True):
        self.port = port
        self.enable_detection = enable_detection
        self.running = False
        
        # ZMQ setup
        self.context = None
        self.socket = None
        
        # Cameras
        self.cameras: Dict[str, GStreamerCamera] = {}
        
        # Detector
        self.detector = None
        if enable_detection and TRT_AVAILABLE:
            self.detector = TensorRTDetector(
                hazmat_engine=HAZMAT_ENGINE,
                landolt_engine=LANDOLT_ENGINE
            )
        
        # Stats
        self.stats = {
            "frames_sent": 0,
            "detections": 0,
            "start_time": 0,
        }
    
    def start(self):
        """Start camera streaming"""
        if not ZMQ_AVAILABLE:
            print("[TRT] ERROR: ZMQ not available")
            return
        
        # Setup ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 2)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f"tcp://*:{self.port}")
        print(f"[TRT] Streaming on port {self.port}")
        
        # Open cameras
        for cam_id, device in CAMERAS.items():
            cam = GStreamerCamera(cam_id, device, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS)
            if cam.open():
                self.cameras[cam_id] = cam
        
        if not self.cameras:
            print("[TRT] ERROR: No cameras opened!")
            return
        
        print(f"[TRT] {len(self.cameras)} cameras ready, detection on: {DETECTION_CAMERAS}")
        
        self.running = True
        self.stats["start_time"] = time.time()
        self._stream_loop()
    
    def _stream_loop(self):
        """Main streaming loop"""
        frame_interval = 1.0 / CAMERA_FPS
        
        try:
            while self.running:
                loop_start = time.time()
                
                for cam_id, cam in self.cameras.items():
                    frame = cam.read()
                    if frame is None:
                        continue
                    
                    # Detection (only on specified cameras)
                    detections = []
                    qr_codes = []
                    
                    if self.enable_detection and cam_id in DETECTION_CAMERAS:
                        # TensorRT YOLO
                        if self.detector:
                            detections = self.detector.detect(frame)
                        
                        # QR codes
                        qr_codes = detect_qr(frame)
                    
                    # Encode JPEG
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    
                    # Build message
                    header = {
                        "camera_id": cam_id,
                        "timestamp": time.time(),
                        "frame_num": cam.frame_count,
                        "width": frame.shape[1],
                        "height": frame.shape[0],
                        "fps": round(cam.actual_fps, 1),
                        "detections": [
                            {
                                "class_id": d.class_id,
                                "class_name": d.class_name,
                                "confidence": round(d.confidence, 3),
                                "bbox": list(d.bbox),
                                "source": d.source,
                            }
                            for d in detections
                        ],
                        "qr_codes": [
                            {
                                "data": d.class_name,
                                "bbox": list(d.bbox),
                            }
                            for d in qr_codes
                        ],
                    }
                    
                    # Send
                    try:
                        self.socket.send_multipart([
                            json.dumps(header).encode(),
                            jpeg.tobytes()
                        ], zmq.NOBLOCK)
                        self.stats["frames_sent"] += 1
                        self.stats["detections"] += len(detections) + len(qr_codes)
                    except zmq.Again:
                        pass
                
                # Rate limiting
                elapsed = time.time() - loop_start
                if elapsed < frame_interval:
                    time.sleep(frame_interval - elapsed)
                    
        except KeyboardInterrupt:
            print("\n[TRT] Interrupted")
        finally:
            self.stop()
    
    def stop(self):
        """Stop streaming"""
        self.running = False
        
        # Print stats
        elapsed = time.time() - self.stats["start_time"]
        if elapsed > 0:
            avg_fps = self.stats["frames_sent"] / elapsed / len(self.cameras) if self.cameras else 0
            print(f"[TRT] Stats: {self.stats['frames_sent']} frames, {self.stats['detections']} detections, {avg_fps:.1f} avg FPS")
        
        # Cleanup
        for cam in self.cameras.values():
            cam.close()
        
        if self.socket:
            self.socket.close()
        if self.context:
            self.context.term()
        
        print("[TRT] Stopped")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="TensorRT Camera Node")
    parser.add_argument("--port", type=int, default=ZMQ_PORT_CAMERA)
    parser.add_argument("--no-detect", action="store_true", help="Disable detection (stream only)")
    parser.add_argument("--hazmat", type=str, default=str(HAZMAT_ENGINE), help="HAZMAT engine path")
    parser.add_argument("--landolt", type=str, default=str(LANDOLT_ENGINE), help="Landolt engine path")
    args = parser.parse_args()
    
    # Override engine paths
    global HAZMAT_ENGINE, LANDOLT_ENGINE
    HAZMAT_ENGINE = Path(args.hazmat)
    LANDOLT_ENGINE = Path(args.landolt)
    
    node = TensorRTCameraNode(
        port=args.port,
        enable_detection=not args.no_detect
    )
    node.start()


if __name__ == "__main__":
    main()
