#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Camera Streaming Node
"""

import cv2
import zmq
import json
import time
from typing import Dict, List

# Configuration
ZMQ_PORT = 5557
JPEG_QUALITY = 65
TARGET_FPS = 15
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Known working cameras on this Jetson
DEFAULT_CAMERAS = {
    "front": 0,
    "arm": 2,
    "backward": 4,
}

# Detection libs
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except:
    YOLO_AVAILABLE = False

try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except:
    PYZBAR_AVAILABLE = False


class CameraStream:
    def __init__(self, camera_id: str, device_index: int):
        self.camera_id = camera_id
        self.device_index = device_index
        self.cap = None
        self.frame_count = 0
        self.last_frame_time = 0
        self.fps = 0
    
    def open(self) -> bool:
        try:
            self.cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
                self.cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                print(f"[CAM] {self.camera_id} opened on /dev/video{self.device_index}")
                return True
            return False
        except Exception as e:
            print(f"[CAM] {self.camera_id} error: {e}")
            return False
    
    def close(self):
        if self.cap:
            self.cap.release()
    
    def read(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.frame_count += 1
                now = time.time()
                if self.last_frame_time > 0:
                    dt = now - self.last_frame_time
                    self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt) if dt > 0 else self.fps
                self.last_frame_time = now
                return frame
        return None


class Detector:
    def __init__(self, yolo_model: str = "yolov8n.pt"):
        self.yolo = None
        self.rescue_classes = [0, 16, 17]  # person, dog, horse
        
        if YOLO_AVAILABLE:
            try:
                self.yolo = YOLO(yolo_model)
                print(f"[DET] YOLO loaded: {yolo_model}")
            except Exception as e:
                print(f"[DET] YOLO error: {e}")
    
    def detect_objects(self, frame) -> List[dict]:
        if self.yolo is None:
            return []
        try:
            results = self.yolo(frame, verbose=False, conf=0.5)
            detections = []
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    detections.append({
                        "class_id": cls_id,
                        "class_name": r.names[cls_id],
                        "confidence": round(float(box.conf[0]), 3),
                        "bbox": list(map(int, box.xyxy[0])),
                        "is_victim": cls_id in self.rescue_classes
                    })
            return detections
        except:
            return []
    
    def detect_qr(self, frame) -> List[dict]:
        if not PYZBAR_AVAILABLE:
            return []
        try:
            decoded = pyzbar.decode(frame)
            return [{
                "data": obj.data.decode('utf-8', errors='ignore'),
                "type": obj.type,
                "bbox": [obj.rect.left, obj.rect.top, 
                        obj.rect.left + obj.rect.width, 
                        obj.rect.top + obj.rect.height]
            } for obj in decoded]
        except:
            return []


class CameraStreamNode:
    def __init__(self, port: int = ZMQ_PORT, enable_detection: bool = False):
        self.port = port
        self.enable_detection = enable_detection
        self.running = False
        self.target_fps = TARGET_FPS
        
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 2)
        self.socket.setsockopt(zmq.LINGER, 0)
        
        self.cameras: Dict[str, CameraStream] = {}
        self.detector = Detector() if enable_detection else None
        
        self.detection_interval = 5
        self.detection_counter = 0
        self.last_detections: Dict[str, List[dict]] = {}
        self.last_qr_codes: Dict[str, List[dict]] = {}
    
    def start(self, camera_devices: Dict[str, int] = None):
        self.socket.bind(f"tcp://*:{self.port}")
        print(f"[CAM] Streaming on port {self.port}")
        
        if camera_devices is None:
            camera_devices = DEFAULT_CAMERAS
        
        print(f"[CAM] Using cameras: {camera_devices}")
        
        for cam_id, device_idx in camera_devices.items():
            cam = CameraStream(cam_id, device_idx)
            if cam.open():
                self.cameras[cam_id] = cam
        
        if not self.cameras:
            print("[CAM] No cameras opened!")
            return
        
        self.running = True
        self._stream_loop()
    
    def _stream_loop(self):
        frame_interval = 1.0 / self.target_fps
        print(f"[CAM] Streaming {len(self.cameras)} cameras at {self.target_fps} FPS")
        
        try:
            while self.running:
                loop_start = time.time()
                
                for cam_id, cam in self.cameras.items():
                    frame = cam.read()
                    if frame is None:
                        continue
                    
                    detections = []
                    qr_codes = []
                    
                    if self.enable_detection and self.detector:
                        self.detection_counter += 1
                        if self.detection_counter >= self.detection_interval:
                            self.detection_counter = 0
                            detections = self.detector.detect_objects(frame)
                            qr_codes = self.detector.detect_qr(frame)
                            self.last_detections[cam_id] = detections
                            self.last_qr_codes[cam_id] = qr_codes
                        else:
                            detections = self.last_detections.get(cam_id, [])
                            qr_codes = self.last_qr_codes.get(cam_id, [])
                    
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    
                    header = {
                        "camera_id": cam_id,
                        "timestamp": time.time(),
                        "frame_num": cam.frame_count,
                        "width": frame.shape[1],
                        "height": frame.shape[0],
                        "fps": round(cam.fps, 1),
                        "detections": detections,
                        "qr_codes": qr_codes
                    }
                    
                    try:
                        self.socket.send_multipart([
                            json.dumps(header).encode(),
                            jpeg.tobytes()
                        ], zmq.NOBLOCK)
                    except zmq.Again:
                        pass
                
                elapsed = time.time() - loop_start
                if frame_interval > elapsed:
                    time.sleep(frame_interval - elapsed)
                    
        except KeyboardInterrupt:
            print("\n[CAM] Interrupted")
        finally:
            self.stop()
    
    def stop(self):
        self.running = False
        for cam in self.cameras.values():
            cam.close()
        self.socket.close()
        self.context.term()
        print("[CAM] Stopped")


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=ZMQ_PORT)
    parser.add_argument('--no-detect', action='store_true')
    parser.add_argument('--detect', action='store_true')
    parser.add_argument('--fps', type=int, default=TARGET_FPS)
    args = parser.parse_args()
    
    # Default is no detection for performance
    enable_det = args.detect and not args.no_detect
    
    node = CameraStreamNode(port=args.port, enable_detection=enable_det)
    node.target_fps = args.fps
    node.start()


if __name__ == "__main__":
    main()
