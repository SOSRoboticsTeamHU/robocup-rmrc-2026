#!/usr/bin/env python3
"""
RoboCup Rescue RMRC 2026 - Multi-Model YOLO Inference Node
===========================================================
Task-based model switching: HAZMAT, Landolt-C, General

Features:
- Runtime model switching via ZMQ control
- TensorRT .engine support (Jetson Orin optimized)
- Checkpoint-based detection caching
- QR code detection (pyzbar)

Usage:
  python3 multi_model_yolo_node.py
  python3 multi_model_yolo_node.py --models-dir /home/jetson/models/engines
"""

import sys
import os
import json
import time
import threading
from pathlib import Path

# Add parent dirs to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
JETSON_DIR = os.path.dirname(SCRIPT_DIR)
REPO_ROOT = os.path.dirname(JETSON_DIR)
for p in [REPO_ROOT, JETSON_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from shared.constants import (
        ZMQ_PORT_CAMERA, ZMQ_PORT_CAMERA_CONTROL,
        CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
        JPEG_QUALITY, CAMERAS, YOLO_CONFIDENCE
    )
except ImportError:
    ZMQ_PORT_CAMERA = 5557
    ZMQ_PORT_CAMERA_CONTROL = 5561
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    CAMERA_FPS = 15
    JPEG_QUALITY = 65
    CAMERAS = {"front": "/dev/video0", "arm": "/dev/video2", "backward": "/dev/video4"}
    YOLO_CONFIDENCE = 0.5

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

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    YOLO = None

try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except ImportError:
    PYZBAR_AVAILABLE = False

# Import GStreamer camera reader from parent
try:
    from deepstream_yolo_node import GstCameraReader, detect_qr, Gst, GLib, GST_AVAILABLE
except ImportError:
    print("[MULTI] Error: Cannot import GstCameraReader from deepstream_yolo_node.py")
    sys.exit(1)


class ModelManager:
    """Manages multiple YOLO models for task-based switching."""
    
    def __init__(self, models_dir: Path, confidence: float = YOLO_CONFIDENCE):
        self.models_dir = Path(models_dir)
        self.confidence = confidence
        self.models = self._discover_models()
        self.current_task = None
        self.current_model = None
        self.ready = False
        
    def _discover_models(self) -> dict:
        """Auto-discover available .engine and .pt models."""
        models = {
            "hazmat": None,
            "landolt": None,
            "general": None
        }
        
        if not self.models_dir.exists():
            print(f"[MODEL] Models dir not found: {self.models_dir}")
            return models
        
        # Priority: .engine > .pt
        search_patterns = [
            ("hazmat", ["hazmat_yolo11n.engine", "hazmat_yolo11n.pt", "hazmat_yolov8n.engine"]),
            ("landolt", ["landolt_yolo11n.engine", "landolt_yolo11n.pt", "landolt_yolov8n.engine"]),
            ("general", ["yolo11n.engine", "yolo11n.pt", "yolov8n.pt"])
        ]
        
        for task, patterns in search_patterns:
            for pattern in patterns:
                candidate = self.models_dir / pattern
                if candidate.exists():
                    models[task] = str(candidate)
                    print(f"[MODEL] Found {task}: {candidate.name}")
                    break
        
        return models
    
    def switch_model(self, task: str) -> bool:
        """Switch to model for given task."""
        if task not in self.models:
            print(f"[MODEL] Unknown task: {task}")
            return False
        
        model_path = self.models[task]
        if not model_path:
            print(f"[MODEL] No model available for task: {task}")
            return False
        
        if self.current_task == task and self.ready:
            print(f"[MODEL] Already using {task} model")
            return True
        
        try:
            print(f"[MODEL] Loading {task} model: {model_path}")
            self.current_model = YOLO(model_path)
            self.current_task = task
            self.ready = True
            print(f"[MODEL] ✅ {task} model loaded")
            return True
        except Exception as e:
            print(f"[MODEL] ❌ Failed to load {task}: {e}")
            self.ready = False
            return False
    
    def detect(self, frame) -> list:
        """Run inference on frame."""
        if not self.ready or self.current_model is None or frame is None:
            return []
        
        try:
            results = self.current_model(frame, conf=self.confidence, verbose=False)
            out = []
            for r in results:
                if r.boxes is None:
                    continue
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    out.append({
                        "class_id": cls_id,
                        "class_name": r.names.get(cls_id, f"class_{cls_id}"),
                        "confidence": round(float(box.conf[0]), 3),
                        "bbox": list(map(int, box.xyxy[0].tolist())),
                        "center": [(int(box.xyxy[0][0]) + int(box.xyxy[0][2])) // 2, 
                                   (int(box.xyxy[0][1]) + int(box.xyxy[0][3])) // 2],
                    })
            return out
        except Exception as e:
            print(f"[MODEL] Detection error: {e}")
            return []


class MultiModelYOLONode:
    """Multi-camera streaming with task-based YOLO model switching."""
    
    def __init__(self, port: int = ZMQ_PORT_CAMERA, control_port: int = ZMQ_PORT_CAMERA_CONTROL,
                 models_dir: Path = None, yolo_cameras: tuple = ("front", "arm")):
        self.port = port
        self.control_port = control_port
        self.yolo_cameras = set(yolo_cameras)
        
        # Model manager
        if models_dir is None:
            models_dir = Path(JETSON_DIR) / "models" / "engines"
        self.model_manager = ModelManager(models_dir)
        
        # Camera readers
        self.cameras: dict = {}
        self.camera_res: dict = {}
        
        # ZMQ
        self.context = None
        self.socket = None
        self.control_socket = None
        
        # Threading
        self.running = False
        self.stream_thread = None
        self.control_thread = None
        self.main_loop = None
        
        # Runtime state
        self.target_fps = CAMERA_FPS
        self.detect_every_n = 3
        self.qr_every_n = 5
        self.last_detections: dict = {}
        self.last_qr_codes: dict = {}
        self.enable_yolo = True
        self.enable_qr = True
        self._control_lock = threading.Lock()
    
    def start(self):
        """Start camera streaming and inference."""
        if not GST_AVAILABLE:
            print("[MULTI] GStreamer not available!")
            return
        if not zmq:
            print("[MULTI] pyzmq not available!")
            return
        if not YOLO_AVAILABLE:
            print("[MULTI] Ultralytics YOLO not available!")
            return
        
        Gst.init(None)
        
        # Initialize cameras
        print("[MULTI] Initializing cameras...")
        OPEN_DELAY = 2.0
        for idx, (cam_id, device) in enumerate(CAMERAS.items()):
            if not os.path.exists(device):
                print(f"[MULTI] Skip {cam_id} - device not found: {device}")
                continue
            
            if self.cameras:
                time.sleep(OPEN_DELAY)
            
            w, h = CAMERA_WIDTH, CAMERA_HEIGHT
            self.camera_res[cam_id] = (w, h)
            needs_bgr = cam_id in self.yolo_cameras
            reader = GstCameraReader(cam_id, device, width=w, height=h, needs_bgr=needs_bgr)
            if reader.build_and_connect():
                self.cameras[cam_id] = reader
                print(f"[MULTI] ✅ {cam_id} ready: {device} ({w}x{h})")
            else:
                print(f"[MULTI] ❌ {cam_id} failed")
        
        if not self.cameras:
            print("[MULTI] No cameras available!")
            return
        
        # Start camera pipelines
        for reader in self.cameras.values():
            reader.set_playing()
        
        # Load default model (general)
        print("[MULTI] Loading default model...")
        self.model_manager.switch_model("general")
        
        # Setup ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 60)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f"tcp://*:{self.port}")
        
        self.control_socket = self.context.socket(zmq.SUB)
        self.control_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.control_socket.setsockopt(zmq.RCVTIMEO, 500)
        self.control_socket.bind(f"tcp://*:{self.control_port}")
        
        print(f"[MULTI] ZMQ camera port: {self.port}")
        print(f"[MULTI] ZMQ control port: {self.control_port}")
        print(f"[MULTI] YOLO cameras: {sorted(self.yolo_cameras)}")
        
        # Start threads
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()
        
        print("[MULTI] 🚀 Multi-model YOLO node running!")
        print("[MULTI] Available models:", list(self.model_manager.models.keys()))
        print("[MULTI] Control commands: {\"task\": \"hazmat|landolt|general\", \"enable_yolo\": true|false}")
        
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
    
    def _control_loop(self):
        """Handle control messages from laptop."""
        while self.running and self.control_socket:
            try:
                msg = self.control_socket.recv_string()
                data = json.loads(msg)
                
                with self._control_lock:
                    # Task switching
                    if "task" in data:
                        task = data["task"]
                        print(f"[MULTI] Control: switch to {task}")
                        if self.model_manager.switch_model(task):
                            print(f"[MULTI] ✅ Switched to {task} model")
                        else:
                            print(f"[MULTI] ❌ Failed to switch to {task}")
                    
                    # YOLO toggle
                    if "enable_yolo" in data:
                        self.enable_yolo = bool(data["enable_yolo"])
                        print(f"[MULTI] YOLO: {self.enable_yolo}")
                    
                    # QR toggle
                    if "enable_qr" in data:
                        self.enable_qr = bool(data["enable_qr"])
                        print(f"[MULTI] QR: {self.enable_qr}")
                        
            except (zmq.Again, json.JSONDecodeError, ValueError):
                pass
            except Exception as e:
                print(f"[MULTI] Control loop error: {e}")
    
    def _stream_loop(self):
        """Main streaming loop with inference."""
        frame_interval = 1.0 / self.target_fps
        frame_counters = {cid: 0 for cid in self.cameras}
        
        while self.running:
            loop_start = time.time()
            
            with self._control_lock:
                enable_yolo = self.enable_yolo
                enable_qr = self.enable_qr
                current_task = self.model_manager.current_task
            
            for cam_id, reader in self.cameras.items():
                frame = reader.get_frame() if getattr(reader, 'appsink', None) else None
                jpeg_bytes = reader.get_jpeg() if hasattr(reader, 'get_jpeg') else None
                if frame is None and jpeg_bytes is None:
                    continue
                
                frame_counters[cam_id] += 1
                
                # Run inference on selected cameras
                run_yolo = (
                    frame is not None
                    and cam_id in self.yolo_cameras
                    and (frame_counters[cam_id] % self.detect_every_n) == 0
                )
                run_qr = run_yolo and (frame_counters[cam_id] % self.qr_every_n) == 0
                if run_yolo and (enable_yolo or enable_qr):
                    detections = self.model_manager.detect(frame) if enable_yolo else self.last_detections.get(cam_id, [])
                    if enable_yolo:
                        self.last_detections[cam_id] = detections
                    qr_codes = detect_qr(frame) if (enable_qr and run_qr) else self.last_qr_codes.get(cam_id, [])
                    if enable_qr:
                        self.last_qr_codes[cam_id] = qr_codes
                else:
                    detections = self.last_detections.get(cam_id, [])
                    qr_codes = self.last_qr_codes.get(cam_id, [])
                
                # Prefer GPU-encoded JPEG (nvjpegenc) to reduce CPU load for SLAM
                if jpeg_bytes is not None:
                    jpeg = jpeg_bytes
                elif frame is not None:
                    _, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    jpeg = enc.tobytes()
                else:
                    continue
                
                # Build header
                header = {
                    "msg_type": "camera_frame",
                    "camera_id": cam_id,
                    "timestamp": time.time(),
                    "frame_num": reader.frame_count,
                    "width": frame.shape[1] if frame is not None else reader.width,
                    "height": frame.shape[0] if frame is not None else reader.height,
                    "fps": self.target_fps,
                    "task": current_task,  # Current YOLO task
                    "detections": detections,
                    "qr_codes": qr_codes,
                }
                
                # Send via ZMQ
                try:
                    data = jpeg if isinstance(jpeg, bytes) else jpeg.tobytes()
                    self.socket.send_multipart([json.dumps(header).encode(), data], zmq.NOBLOCK)
                except zmq.Again:
                    pass
            
            # Rate limiting
            elapsed = time.time() - loop_start
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)
    
    def stop(self):
        """Cleanup and stop."""
        self.running = False
        if self.main_loop and GLib is not None:
            self.main_loop.quit()
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        if self.stream_thread:
            self.stream_thread.join(timeout=2.0)
        for r in self.cameras.values():
            r.stop()
        if self.control_socket:
            self.control_socket.close()
        if self.socket:
            self.socket.close()
        if self.context:
            self.context.term()
        print("[MULTI] Stopped")


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Multi-model YOLO inference node with task switching")
    ap.add_argument("--port", type=int, default=ZMQ_PORT_CAMERA, help="ZMQ camera stream port")
    ap.add_argument("--control-port", type=int, default=ZMQ_PORT_CAMERA_CONTROL, help="ZMQ control port")
    ap.add_argument("--models-dir", type=str, default=None, help="Directory with .engine models")
    ap.add_argument("--yolo-cameras", nargs="+", default=["front", "arm"], help="Camera IDs for YOLO")
    ap.add_argument("--fps", type=int, default=CAMERA_FPS, help="Target FPS per camera (30-60)")
    args = ap.parse_args()
    
    node = MultiModelYOLONode(
        port=args.port,
        control_port=args.control_port,
        models_dir=Path(args.models_dir) if args.models_dir else None,
        yolo_cameras=tuple(args.yolo_cameras)
    )
    node.target_fps = args.fps
    
    try:
        node.start()
    except KeyboardInterrupt:
        print("\n[MULTI] Interrupt")
    finally:
        node.stop()


if __name__ == "__main__":
    main()
