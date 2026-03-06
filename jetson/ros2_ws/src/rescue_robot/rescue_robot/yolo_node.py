#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - YOLO Detection Node
==============================================
Object detection using YOLOv8 with TensorRT optimization.
Also includes QR code reading with pyzbar.

Features:
- YOLOv8 inference (TensorRT on Jetson, ONNX fallback)
- Hazmat label detection
- Landolt-C detection
- QR code reading and decoding
- Detection result publishing via ZMQ
"""

import cv2
import json
import time
import threading
import zmq
import numpy as np
from typing import Dict, Optional, List, Tuple
from dataclasses import dataclass, asdict
import sys
import os

# Add shared module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'shared'))

try:
    from shared.constants import (
        ZMQ_PORT_CAMERA, YOLO_CONFIDENCE,
        YOLO_MODEL_HAZMAT, YOLO_MODEL_GENERAL,
        HAZMAT_LABELS, JETSON_IP
    )
except ImportError:
    ZMQ_PORT_CAMERA = 5557
    YOLO_CONFIDENCE = 0.5
    YOLO_MODEL_HAZMAT = "models/hazmat_yolov8n.engine"
    YOLO_MODEL_GENERAL = "models/yolov8n.engine"
    HAZMAT_LABELS = ["Explosive", "Flammable", "Corrosive", "Radioactive"]
    JETSON_IP = "192.168.1.1"

# Try to import detection libraries
try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except ImportError:
    PYZBAR_AVAILABLE = False
    print("[YOLO] WARNING: pyzbar not available, QR detection disabled")

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False
    print("[YOLO] WARNING: ultralytics not available")


@dataclass
class Detection:
    """Single detection result."""
    class_name: str
    class_id: int
    confidence: float
    bbox: List[int]  # [x1, y1, x2, y2]
    center: List[int]  # [cx, cy]


@dataclass
class QRCode:
    """QR code detection result."""
    data: str
    bbox: List[int]  # [x1, y1, x2, y2]
    polygon: List[List[int]]  # Corner points


class YOLODetector:
    """YOLO object detection wrapper."""
    
    def __init__(self, model_path: str = None, confidence: float = YOLO_CONFIDENCE):
        self.model = None
        self.model_path = model_path
        self.confidence = confidence
        self.class_names = []
        self.initialized = False
        
    def load(self) -> bool:
        """Load YOLO model."""
        if not ULTRALYTICS_AVAILABLE:
            print("[YOLO] Ultralytics not available")
            return False
        
        try:
            if self.model_path and os.path.exists(self.model_path):
                print(f"[YOLO] Loading model: {self.model_path}")
                self.model = YOLO(self.model_path)
            else:
                # Use default YOLOv8n
                print("[YOLO] Loading default YOLOv8n model")
                self.model = YOLO("yolov8n.pt")
            
            self.class_names = self.model.names
            self.initialized = True
            print(f"[YOLO] Model loaded with {len(self.class_names)} classes")
            return True
            
        except Exception as e:
            print(f"[YOLO] Failed to load model: {e}")
            return False
    
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Run detection on a frame."""
        if not self.initialized or self.model is None:
            return []
        
        try:
            results = self.model(frame, conf=self.confidence, verbose=False)
            
            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue
                
                for i, box in enumerate(boxes):
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    cls_name = self.class_names.get(cls_id, f"class_{cls_id}")
                    
                    detections.append(Detection(
                        class_name=cls_name,
                        class_id=cls_id,
                        confidence=conf,
                        bbox=[x1, y1, x2, y2],
                        center=[(x1 + x2) // 2, (y1 + y2) // 2]
                    ))
            
            return detections
            
        except Exception as e:
            print(f"[YOLO] Detection error: {e}")
            return []


class QRDetector:
    """QR code detector using pyzbar."""
    
    def __init__(self):
        self.available = PYZBAR_AVAILABLE
    
    def detect(self, frame: np.ndarray) -> List[QRCode]:
        """Detect QR codes in frame."""
        if not self.available:
            return []
        
        try:
            # Convert to grayscale for better detection
            if len(frame.shape) == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame
            
            # Detect QR codes
            codes = pyzbar.decode(gray)
            
            results = []
            for code in codes:
                if code.type == 'QRCODE':
                    # Get bounding box
                    x, y, w, h = code.rect
                    
                    # Get polygon points
                    polygon = [[p.x, p.y] for p in code.polygon]
                    
                    results.append(QRCode(
                        data=code.data.decode('utf-8'),
                        bbox=[x, y, x + w, y + h],
                        polygon=polygon
                    ))
            
            return results
            
        except Exception as e:
            print(f"[QR] Detection error: {e}")
            return []


class DetectionNode:
    """
    Main detection node that processes camera frames
    and publishes detection results.
    """
    
    def __init__(self):
        # Detectors
        self.yolo = YOLODetector()
        self.qr = QRDetector()
        
        # ZMQ setup
        self.zmq_context = zmq.Context()
        
        # Subscribe to camera frames
        self.cam_socket = self.zmq_context.socket(zmq.SUB)
        self.cam_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.cam_socket.setsockopt(zmq.RCVTIMEO, 100)
        
        # Publish detection results
        self.det_socket = self.zmq_context.socket(zmq.PUB)
        
        # State
        self.running = False
        self.frame_count = 0
        self.detection_count = 0
        self.qr_count = 0
        
        # Settings
        self.enable_yolo = True
        self.enable_qr = True
        self.process_interval = 3  # Process every Nth frame
    
    def start(self, camera_addr: str = None):
        """Start detection processing."""
        print("[DETECTION] Starting detection node...")
        
        # Connect to camera stream
        if camera_addr is None:
            camera_addr = f"tcp://{JETSON_IP}:{ZMQ_PORT_CAMERA}"
        
        try:
            self.cam_socket.connect(camera_addr)
            print(f"[DETECTION] Connected to camera at {camera_addr}")
        except zmq.ZMQError as e:
            print(f"[DETECTION] Camera connection error: {e}")
            return
        
        # Bind detection publisher
        det_port = ZMQ_PORT_CAMERA + 10  # 5567
        try:
            self.det_socket.bind(f"tcp://*:{det_port}")
            print(f"[DETECTION] Publishing detections on port {det_port}")
        except zmq.ZMQError as e:
            print(f"[DETECTION] Bind error: {e}")
            return
        
        # Load YOLO model
        if self.enable_yolo:
            self.yolo.load()
        
        self.running = True
        print("[DETECTION] Detection node started")
        self._run_loop()
    
    def stop(self):
        """Stop detection processing."""
        print("[DETECTION] Stopping...")
        self.running = False
        
        self.cam_socket.close()
        self.det_socket.close()
        self.zmq_context.term()
        
        print(f"[DETECTION] Stopped. Processed {self.frame_count} frames, "
              f"{self.detection_count} YOLO detections, {self.qr_count} QR codes")
    
    def _run_loop(self):
        """Main processing loop."""
        while self.running:
            try:
                # Receive camera frame
                msg_parts = self.cam_socket.recv_multipart(zmq.NOBLOCK)
                if len(msg_parts) != 2:
                    continue
                
                header = json.loads(msg_parts[0].decode('utf-8'))
                jpeg_data = msg_parts[1]
                
                self.frame_count += 1
                
                # Skip frames for performance
                if self.frame_count % self.process_interval != 0:
                    continue
                
                # Decode JPEG
                frame = cv2.imdecode(
                    np.frombuffer(jpeg_data, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )
                
                if frame is None:
                    continue
                
                camera_id = header.get("camera_id", "unknown")
                
                # Run detections
                yolo_detections = []
                qr_codes = []
                
                if self.enable_yolo and self.yolo.initialized:
                    yolo_detections = self.yolo.detect(frame)
                    self.detection_count += len(yolo_detections)
                
                if self.enable_qr:
                    qr_codes = self.qr.detect(frame)
                    self.qr_count += len(qr_codes)
                
                # Publish results
                if yolo_detections or qr_codes:
                    self._publish_results(camera_id, yolo_detections, qr_codes)
                    
            except zmq.Again:
                pass  # No message available
            except Exception as e:
                print(f"[DETECTION] Processing error: {e}")
    
    def _publish_results(self, camera_id: str, 
                         detections: List[Detection],
                         qr_codes: List[QRCode]):
        """Publish detection results."""
        msg = {
            "msg_type": "detection_result",
            "timestamp": time.time(),
            "camera_id": camera_id,
            "yolo_detections": [asdict(d) for d in detections],
            "qr_codes": [asdict(q) for q in qr_codes]
        }
        
        try:
            self.det_socket.send_json(msg, zmq.NOBLOCK)
        except zmq.Again:
            pass
        except Exception as e:
            print(f"[DETECTION] Publish error: {e}")


def draw_detections(frame: np.ndarray, 
                    detections: List[Detection],
                    qr_codes: List[QRCode]) -> np.ndarray:
    """Draw detection results on frame."""
    result = frame.copy()
    
    # Draw YOLO detections
    for det in detections:
        x1, y1, x2, y2 = det.bbox
        color = (0, 255, 0)  # Green
        
        # Special colors for hazmat
        if det.class_name.lower() in [h.lower() for h in HAZMAT_LABELS]:
            color = (0, 0, 255)  # Red for hazmat
        
        cv2.rectangle(result, (x1, y1), (x2, y2), color, 2)
        
        label = f"{det.class_name} {det.confidence:.2f}"
        cv2.putText(result, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    # Draw QR codes
    for qr in qr_codes:
        x1, y1, x2, y2 = qr.bbox
        color = (255, 0, 0)  # Blue
        
        cv2.rectangle(result, (x1, y1), (x2, y2), color, 2)
        cv2.putText(result, f"QR: {qr.data[:20]}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    return result


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='YOLO Detection Node')
    parser.add_argument('--camera', type=str, default=None,
                        help='Camera ZMQ address')
    parser.add_argument('--no-yolo', action='store_true',
                        help='Disable YOLO detection')
    parser.add_argument('--no-qr', action='store_true',
                        help='Disable QR detection')
    parser.add_argument('--interval', type=int, default=3,
                        help='Process every Nth frame')
    parser.add_argument('--model', type=str, default=None,
                        help='YOLO model path')
    args = parser.parse_args()
    
    node = DetectionNode()
    node.enable_yolo = not args.no_yolo
    node.enable_qr = not args.no_qr
    node.process_interval = args.interval
    
    if args.model:
        node.yolo.model_path = args.model
    
    try:
        node.start(args.camera)
    except KeyboardInterrupt:
        print("\n[DETECTION] Interrupted")
    finally:
        node.stop()


if __name__ == "__main__":
    main()
