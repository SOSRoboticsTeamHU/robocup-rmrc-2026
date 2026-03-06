#!/usr/bin/env python3
"""
Simple Vision Node - RoboCupRescue RMRC 2026
Basic 3-camera setup without complex recovery logic
"""

import sys
import os
import json
import time
import threading
import io
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
JETSON_DIR = SCRIPT_DIR.parent
REPO_ROOT = JETSON_DIR.parent

for p in [str(REPO_ROOT), str(JETSON_DIR)]:
    if p not in sys.path:
        sys.path.insert(0, p)

# ====================== CONSTANTS ======================
try:
    from shared.constants import (
        ZMQ_PORT_CAMERA, ZMQ_PORT_CAMERA_CONTROL,
        ZMQ_PORT_SNAPSHOT_REQUEST, ZMQ_PORT_SNAPSHOT_RESULT,
        CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, JPEG_QUALITY,
        CAMERAS, YOLO_CONFIDENCE, YOLO_CAMERAS, JETSON_CAMERAS,
    )
except ImportError:
    ZMQ_PORT_CAMERA = 5557
    ZMQ_PORT_CAMERA_CONTROL = 5561
    ZMQ_PORT_SNAPSHOT_REQUEST = 5570
    ZMQ_PORT_SNAPSHOT_RESULT = 5571
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    CAMERA_FPS = 15
    JPEG_QUALITY = 50
    CAMERAS = {"front": "/dev/video4", "arm": "/dev/video2", "backward": "/dev/video0"}
    YOLO_CONFIDENCE = 0.5
    YOLO_CAMERAS = ("front", "arm")
    JETSON_CAMERAS = ("front", "arm", "backward")

import zmq
import numpy as np
from PIL import Image
import cv2

# Vision modules
try:
    from vision.yolo11_processor import YOLO11Processor, YOLO11Config
    from vision.landolt_classifier import LandoltClassifier
except ImportError:
    from jetson.vision.yolo11_processor import YOLO11Processor, YOLO11Config
    from jetson.vision.landolt_classifier import LandoltClassifier

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
Gst.init(None)


class SimpleCameraReader:
    """Simple camera reader using OpenCV - most reliable"""
    
    def __init__(self, camera_id: str, device: str, width: int = 640, height: int = 480, fps: int = 15):
        self.camera_id = camera_id
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self._last_jpeg = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def _open_capture(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap or not cap.isOpened():
            cap = cv2.VideoCapture(self.device)
        if not cap or not cap.isOpened():
            return None
        
        # Set MJPEG for better performance
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except Exception:
            pass
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        return cap

    def build_and_connect(self) -> bool:
        self.cap = self._open_capture()
        if not self.cap:
            print(f"[VISION] {self.camera_id} OpenCV failed: {self.device}")
            return False
        
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        print(f"[VISION] {self.camera_id} → OpenCV OK: {self.device} ({self.width}x{self.height}@{self.fps})")
        return True

    def _reader_loop(self):
        print(f"[VISION] {self.camera_id} reader loop starting...")
        frame_count = 0
        
        while self._running:
            if not self.cap or not self.cap.isOpened():
                print(f"[VISION] {self.camera_id} camera disconnected, attempting reconnect...")
                self.cap = self._open_capture()
                if not self.cap:
                    time.sleep(0.5)
                    continue
            
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            
            frame_count += 1
            if frame_count == 1:
                print(f"[VISION] {self.camera_id} captured first frame!")
            
            # Resize if needed
            h, w = frame.shape[:2]
            if w != self.width or h != self.height:
                frame = cv2.resize(frame, (self.width, self.height))
            
            # Encode to JPEG
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if ok:
                with self._lock:
                    self._last_jpeg = buf.tobytes()
            
            time.sleep(0.03)  # ~30fps max, let camera dictate actual rate

    def get_jpeg(self):
        with self._lock:
            return self._last_jpeg

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()


class SimpleVisionNode:
    def __init__(self):
        self.cameras = {}
        self.running = False
        self.stream_thread = None
        self.inference_thread = None
        self.main_loop = None

        # YOLO processor (only for front and arm)
        self.yolo = YOLO11Processor(
            models_dir=JETSON_DIR / "models" / "engines",
            config=YOLO11Config(confidence_threshold=YOLO_CONFIDENCE),
            default_task="detect"
        )

        self.landolt_classifier = LandoltClassifier(use_rule_based_fallback=True)

        self.context = None
        self.socket = None
        self.control_socket = None
        
        self.last_detections = {}
        self.last_landolt_readings = {}

    def start(self):
        print("[VISION] Starting simple 3-camera setup...")
        
        # Start cameras in order: front, arm, backward
        for cam_id in ["front", "arm", "backward"]:
            device = CAMERAS.get(cam_id)
            if device and os.path.exists(device):
                reader = SimpleCameraReader(cam_id, device)
                if reader.build_and_connect():
                    self.cameras[cam_id] = reader
                    time.sleep(1.0)  # Stagger startup
                else:
                    print(f"[VISION] Failed to start {cam_id} camera")
            else:
                print(f"[VISION] Device not found for {cam_id}: {device}")

        if not self.cameras:
            print("[VISION] No cameras started!")
            return

        # Setup ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 3)
        self.socket.setsockopt(zmq.SNDTIMEO, 0)
        self.socket.bind(f"tcp://*:{ZMQ_PORT_CAMERA}")

        self.control_socket = self.context.socket(zmq.SUB)
        self.control_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.control_socket.bind(f"tcp://*:{ZMQ_PORT_CAMERA_CONTROL}")

        self.running = True

        # Start threads
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

        self.inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self.inference_thread.start()

        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()

        print(f"[VISION] Simple vision node running - {len(self.cameras)} cameras")
        print(f"[VISION] YOLO active on: {YOLO_CAMERAS}")

        self.main_loop = GLib.MainLoop()
        try:
            self.main_loop.run()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def _control_loop(self):
        while self.running:
            try:
                msg = self.control_socket.recv_string()
                data = json.loads(msg)
                if "task" in data:
                    self.yolo.switch_task(data["task"])
            except:
                pass
            time.sleep(0.05)

    def _inference_loop(self):
        """Background YOLO + Landolt inference on front and arm"""
        while self.running:
            for cam_id in YOLO_CAMERAS:
                if cam_id not in self.cameras:
                    continue
                
                jpeg = self.cameras[cam_id].get_jpeg()
                if not jpeg:
                    continue
                
                frame_rgb = self._jpeg_to_rgb(jpeg)
                if frame_rgb is None:
                    continue

                # YOLO detection
                detections = self.yolo.detect_combined(frame_rgb) if self.yolo.combined_ready else self.yolo.detect(frame_rgb)

                # Landolt classification
                landolt_readings = []
                landolt_dets = [d for d in detections if "landolt" in d.get("class_name", "").lower() or "ring" in d.get("class_name", "").lower()]
                if landolt_dets:
                    landolt_readings = self.landolt_classifier.predict_from_detections(frame_rgb, landolt_dets)

                self.last_detections[cam_id] = detections
                self.last_landolt_readings[cam_id] = landolt_readings

            time.sleep(0.08)  # ~12 Hz inference

    def _jpeg_to_rgb(self, jpeg_bytes):
        if not jpeg_bytes:
            return None
        try:
            img = Image.open(io.BytesIO(jpeg_bytes)).convert("RGB")
            return np.array(img)
        except:
            return None

    def _stream_loop(self):
        """Send frames for all cameras"""
        frame_nums = {cid: 0 for cid in self.cameras}
        
        while self.running:
            for cam_id, reader in self.cameras.items():
                jpeg = reader.get_jpeg()
                if jpeg:
                    frame_nums[cam_id] += 1
                    
                    header = {
                        "msg_type": "camera_frame",
                        "camera_id": cam_id,
                        "timestamp": time.time(),
                        "width": CAMERA_WIDTH,
                        "height": CAMERA_HEIGHT,
                        "fps": CAMERA_FPS,
                        "frame_num": frame_nums[cam_id],
                        "task": self.yolo.get_current_task() if cam_id in YOLO_CAMERAS else "none",
                        "detections": self.last_detections.get(cam_id, []),
                        "qr_codes": [],
                        "landolt_readings": [
                            {"direction": getattr(r, "direction_name", str(r)), "confidence": getattr(r, "confidence", 0)}
                            for r in self.last_landolt_readings.get(cam_id, [])
                        ],
                    }
                    
                    try:
                        self.socket.send_multipart([json.dumps(header).encode(), jpeg], flags=zmq.NOBLOCK)
                    except zmq.Again:
                        pass
                    except Exception:
                        pass
            
            time.sleep(0.04)

    def stop(self):
        self.running = False
        for reader in self.cameras.values():
            reader.stop()
        if self.main_loop:
            self.main_loop.quit()
        print("[VISION] Stopped")


if __name__ == "__main__":
    node = SimpleVisionNode()
    try:
        node.start()
    except KeyboardInterrupt:
        node.stop()
