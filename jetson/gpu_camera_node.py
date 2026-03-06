#!/usr/bin/env python3
"""
RoboCup Rescue RMRC 2026 - GPU Camera Node (Streaming + Optional YOLO)
======================================================================
Primary camera node: GPU streaming always, YOLO optional.

- **Without models:** Pure GPU streaming. All cameras use JPEG-only pipeline.
- **With models:** Copy .engine or .pt files to models/engines/ and restart.
  YOLO auto-enables on front+arm cameras. No code changes needed.

Usage:
  python3 gpu_camera_node.py
  python3 gpu_camera_node.py --models-dir /path/to/engines

Model directory (auto-discovered):
  jetson/models/engines/
    hazmat_yolo11n.engine  (or hazmat_yolov8n.engine)
    landolt_yolo11n.engine
    yolo11n.engine         (general fallback)
"""

import sys
import os
import json
import time
import threading
from pathlib import Path

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

try:
    from shared.constants import (
        ZMQ_PORT_CAMERA, ZMQ_PORT_CAMERA_CONTROL,
        CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
        JPEG_QUALITY, CAMERAS, YOLO_CONFIDENCE,
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

DEFAULT_MODELS_DIR = Path(SCRIPT_DIR) / "models" / "engines"
YOLO_CAMERAS = ("front", "arm")  # Cameras that get inference when YOLO is available

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

# GStreamer + camera reader (detect_qr handles missing pyzbar)
try:
    from deepstream_yolo_node import (
        GstCameraReader, build_gst_pipeline, detect_qr,
        Gst, GLib, GST_AVAILABLE,
    )
except ImportError:
    GST_AVAILABLE = False
    Gst = None
    GLib = None
    GstCameraReader = None
    build_gst_pipeline = None

    def detect_qr(frame):
        return []

# Optional: ModelManager for multi-model YOLO (HAZMAT, Landolt, General)
ModelManager = None
YOLO_AVAILABLE = False
try:
    if SCRIPT_DIR not in sys.path:
        sys.path.insert(0, SCRIPT_DIR)
    from deepstream.multi_model_yolo_node import ModelManager as _ModelManager
    ModelManager = _ModelManager
    YOLO_AVAILABLE = True
except ImportError as e:
    pass  # YOLO optional; streaming works without it

try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except ImportError:
    PYZBAR_AVAILABLE = False


def discover_yolo_models(models_dir: Path) -> bool:
    """Check if any YOLO model exists in models_dir. Returns True if YOLO can be enabled."""
    if not models_dir or not models_dir.exists():
        return False
    patterns = [
        "hazmat_yolo11n.engine", "hazmat_yolo11n.pt", "hazmat_yolov8n.engine", "hazmat_best.engine",
        "landolt_yolo11n.engine", "landolt_yolo11n.pt", "landolt_yolov8n.engine", "landolt_best.engine",
        "yolo11n.engine", "yolo11n.pt", "yolov8n.engine", "yolov8n.pt",
    ]
    for p in patterns:
        if (models_dir / p).exists():
            return True
    return False


class GPUCameraNode:
    """
    GPU camera streaming with optional YOLO.
    - No models: stream only (JPEG pipeline, low CPU)
    - Models in models/engines/: YOLO + QR on front+arm
    """
    def __init__(self, port: int = ZMQ_PORT_CAMERA, control_port: int = ZMQ_PORT_CAMERA_CONTROL,
                 models_dir: Path = None):
        self.port = port
        self.control_port = control_port
        self.models_dir = Path(models_dir) if models_dir else DEFAULT_MODELS_DIR

        self.yolo_available = False
        self.yolo_cameras = set()
        self.model_manager = None

        self.cameras = {}
        self.camera_res = {}
        self.context = None
        self.socket = None
        self.control_socket = None
        self.running = False
        self.stream_thread = None
        self.control_thread = None
        self.main_loop = None

        self.target_fps = CAMERA_FPS
        self.detect_every_n = 3
        self.qr_every_n = 5
        self.last_detections = {}
        self.last_qr_codes = {}
        self.enable_yolo = True
        self.enable_qr = True
        self._control_lock = threading.Lock()

    def _init_yolo(self) -> bool:
        """Discover and load YOLO models if available."""
        if not discover_yolo_models(self.models_dir):
            print("[GPU-CAM] No YOLO models found in", self.models_dir)
            print("[GPU-CAM] Copy .engine files to enable YOLO:")
            print("          hazmat_yolo11n.engine, landolt_yolo11n.engine, yolo11n.engine")
            return False
        if not ModelManager or not YOLO_AVAILABLE:
            print("[GPU-CAM] ModelManager not available (ultralytics?)")
            return False
        self.model_manager = ModelManager(self.models_dir, confidence=YOLO_CONFIDENCE)
        if not any(self.model_manager.models.values()):
            print("[GPU-CAM] No valid models in", self.models_dir)
            return False
        # Load default (general or first available)
        for task in ("general", "hazmat", "landolt"):
            if self.model_manager.switch_model(task):
                break
        self.yolo_available = True
        self.yolo_cameras = set(YOLO_CAMERAS)
        print("[GPU-CAM] YOLO enabled on cameras:", sorted(self.yolo_cameras))
        return True

    def start(self):
        if not GST_AVAILABLE or GstCameraReader is None:
            print("[GPU-CAM] GStreamer/camera reader not available")
            return
        if not zmq:
            print("[GPU-CAM] pyzmq not available")
            return

        # Check for YOLO models before creating cameras
        self._init_yolo()
        if not self.yolo_available:
            print("[GPU-CAM] Streaming only (no YOLO). Copy models to enable.")

        Gst.init(None)
        OPEN_DELAY = 2.0
        EXTRA_BEFORE_THIRD = 1.5
        slot_order = list(CAMERAS.keys())

        for idx, cam_id in enumerate(slot_order):
            device = CAMERAS.get(cam_id)
            if not device or not os.path.exists(device):
                print(f"[GPU-CAM] Skip {cam_id} ({device}) - missing device")
                continue
            if self.cameras:
                time.sleep(OPEN_DELAY)
                if len(self.cameras) == 2:
                    time.sleep(EXTRA_BEFORE_THIRD)
            w, h = CAMERA_WIDTH, CAMERA_HEIGHT
            self.camera_res[cam_id] = (w, h)
            needs_bgr = cam_id in self.yolo_cameras
            reader = GstCameraReader(cam_id, device, width=w, height=h, needs_bgr=needs_bgr)
            if reader.build_and_connect():
                self.cameras[cam_id] = reader
                mode = "YOLO" if needs_bgr else "stream"
                print(f"[GPU-CAM] {cam_id} ready: {device} ({w}x{h}) [{mode}]")
            else:
                print(f"[GPU-CAM] Skip {cam_id} ({device})")

        if not self.cameras:
            print("[GPU-CAM] No cameras started")
            return

        for r in self.cameras.values():
            r.set_playing()

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 60)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f"tcp://*:{self.port}")

        self.control_socket = self.context.socket(zmq.SUB)
        self.control_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.control_socket.setsockopt(zmq.RCVTIMEO, 500)
        self.control_socket.bind(f"tcp://*:{self.control_port}")
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

        print(f"[GPU-CAM] ZMQ port {self.port} | control {self.control_port}")
        self.running = True
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()
        print("[GPU-CAM] Running")

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
        while self.running and self.control_socket:
            try:
                msg = self.control_socket.recv_string()
                data = json.loads(msg)
                with self._control_lock:
                    if "enable_yolo" in data and self.yolo_available:
                        self.enable_yolo = bool(data["enable_yolo"])
                    if "enable_qr" in data:
                        self.enable_qr = bool(data["enable_qr"])
                    if "task" in data and self.model_manager:
                        self.model_manager.switch_model(data["task"])
            except (zmq.Again, json.JSONDecodeError, ValueError):
                pass
            except Exception:
                pass

    def _stream_loop(self):
        frame_interval = 1.0 / self.target_fps
        frame_counters = {cid: 0 for cid in self.cameras}
        current_task = "general" if self.model_manager else None

        while self.running:
            loop_start = time.time()
            with self._control_lock:
                enable_yolo = self.enable_yolo
                enable_qr = self.enable_qr
                if self.model_manager:
                    current_task = self.model_manager.current_task

            for cam_id, reader in self.cameras.items():
                frame = reader.get_frame() if getattr(reader, "appsink", None) else None
                jpeg_bytes = reader.get_jpeg() if hasattr(reader, "get_jpeg") else None
                if frame is None and jpeg_bytes is None:
                    continue
                frame_counters[cam_id] += 1

                run_yolo = (
                    frame is not None
                    and cam_id in self.yolo_cameras
                    and (frame_counters[cam_id] % self.detect_every_n) == 0
                )
                run_qr = run_yolo and (frame_counters[cam_id] % self.qr_every_n) == 0
                if run_yolo and (enable_yolo or enable_qr) and self.model_manager:
                    detections = self.model_manager.detect(frame) if enable_yolo else self.last_detections.get(cam_id, [])
                    if enable_yolo:
                        self.last_detections[cam_id] = detections
                    qr_codes = detect_qr(frame) if (enable_qr and run_qr) else self.last_qr_codes.get(cam_id, [])
                    if enable_qr:
                        self.last_qr_codes[cam_id] = qr_codes
                else:
                    detections = self.last_detections.get(cam_id, [])
                    qr_codes = self.last_qr_codes.get(cam_id, [])

                if jpeg_bytes is not None:
                    jpeg = jpeg_bytes
                elif frame is not None:
                    _, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    jpeg = enc.tobytes()
                else:
                    continue

                w = frame.shape[1] if frame is not None else reader.width
                h = frame.shape[0] if frame is not None else reader.height
                header = {
                    "msg_type": "camera_frame",
                    "camera_id": cam_id,
                    "timestamp": time.time(),
                    "frame_num": reader.frame_count,
                    "width": w,
                    "height": h,
                    "fps": self.target_fps,
                    "task": current_task,
                    "detections": detections,
                    "qr_codes": qr_codes,
                }
                try:
                    self.socket.send_multipart(
                        [json.dumps(header).encode(), jpeg if isinstance(jpeg, bytes) else jpeg.tobytes()],
                        zmq.NOBLOCK
                    )
                except zmq.Again:
                    pass

            elapsed = time.time() - loop_start
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)

    def stop(self):
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
        print("[GPU-CAM] Stopped")


def main():
    import argparse
    ap = argparse.ArgumentParser(description="GPU camera node: stream only, YOLO when models present")
    ap.add_argument("--port", type=int, default=ZMQ_PORT_CAMERA)
    ap.add_argument("--control-port", type=int, default=ZMQ_PORT_CAMERA_CONTROL)
    ap.add_argument("--models-dir", type=str, default=None, help="Path to engines (default: jetson/models/engines)")
    ap.add_argument("--fps", type=int, default=CAMERA_FPS, help="Target FPS per camera (30-60)")
    args = ap.parse_args()
    models_dir = Path(args.models_dir) if args.models_dir else None
    node = GPUCameraNode(port=args.port, control_port=args.control_port, models_dir=models_dir)
    node.target_fps = min(max(args.fps, 10), 60)
    try:
        node.start()
    except KeyboardInterrupt:
        print("\n[GPU-CAM] Interrupt")
    finally:
        node.stop()


if __name__ == "__main__":
    main()
