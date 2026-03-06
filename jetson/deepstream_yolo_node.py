#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Kamera stream: DeepStream (GStreamer) minden kamerán, YOLO csak kiválasztottakon
=========================================================================================================
Minden kamera (front, arm, backward) GStreamer pipeline-pal; YOLOv8 TensorRT + QR csak a megadott
kamerákon (alapból front + arm). Lecseréli az OpenCV camera_stream-et.

Usage:
  python3 deepstream_yolo_node.py
  python3 deepstream_yolo_node.py --yolo-cameras front arm --model yolov8n.engine

Requires: GStreamer (gi.repository.Gst), ultralytics, pyzbar, zmq.
TensorRT: .engine a Jetsonon (yolo export model=yolov8n.pt format=engine).
"""

import sys
import os
import json
import time
import threading
import queue

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

try:
    from shared.constants import (
        ZMQ_PORT_CAMERA, ZMQ_PORT_CAMERA_CONTROL,
        CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
        JPEG_QUALITY, CAMERAS, YOLO_MODEL_GENERAL, YOLO_CONFIDENCE, YOLO_CAMERAS,
    )
except ImportError:
    ZMQ_PORT_CAMERA = 5557
    ZMQ_PORT_CAMERA_CONTROL = 5561
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    CAMERA_FPS = 15
    JPEG_QUALITY = 65
    CAMERAS = {"front": "/dev/video0", "arm": "/dev/video2", "backward": "/dev/video4"}
    YOLO_MODEL_GENERAL = "yolov8n.engine"
    YOLO_CONFIDENCE = 0.5
    YOLO_CAMERAS = ("front", "arm")

DEFAULT_YOLO_CAMERAS = YOLO_CAMERAS

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
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst, GLib
    GST_AVAILABLE = True
except (ImportError, ValueError):
    GST_AVAILABLE = False
    Gst = None
    GLib = None

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


def gst_to_numpy(sample):
    """Convert GstSample to BGR numpy array."""
    if sample is None:
        return None
    buf = sample.get_buffer()
    caps = sample.get_caps()
    if buf is None or caps is None:
        return None
    fmt = caps.get_structure(0)
    w = fmt.get_value("width")
    h = fmt.get_value("height")
    success, map_info = buf.map(Gst.MapFlags.READ)
    if not success:
        return None
    try:
        arr = np.ndarray((h, w, 3), dtype=np.uint8, buffer=map_info.data)
        return arr.copy()
    finally:
        buf.unmap(map_info)


def build_gst_pipeline(device: str, width: int, height: int, fps: int,
                      use_gpu_jpeg: bool = True, needs_bgr: bool = True) -> "Gst.Pipeline":
    """Build GStreamer pipeline.

    Use GST_USE_RAW_CAMERA=1 for raw YUYV (fallback when MJPEG fails).
    Use GST_MJPEG_RESOLUTION=640x480 if cameras only support 640x480 MJPEG.
    Default: MJPEG 1280x720 + GPU decode (nvjpegenc).

    GPU path (use_gpu_jpeg=True): nvjpegenc for hardware JPEG encoding.
    CPU path: Single BGR appsink, cv2.imencode for JPEG (fallback).
    """
    fps_val = fps or 30

    # Raw pipeline: set GST_USE_RAW_CAMERA=1 only when MJPEG fails
    if os.environ.get("GST_USE_RAW_CAMERA", "0") == "1":
        return _build_raw_pipeline(device, width, height, fps_val, needs_bgr)

    # MJPEG resolution: 1280x720 (default) or 640x480 via GST_MJPEG_RESOLUTION
    mjpeg_res = os.environ.get("GST_MJPEG_RESOLUTION", "1280x720").strip().lower()
    try:
        mjpeg_w, mjpeg_h = map(int, mjpeg_res.replace("x", " ").split())
    except (ValueError, AttributeError):
        mjpeg_w, mjpeg_h = 1280, 720

    if use_gpu_jpeg:
        # GPU decode + scale
        base = (
            f"v4l2src device={device} io-mode=2 ! "
            f"image/jpeg,width={mjpeg_w},height={mjpeg_h},framerate={fps_val}/1 ! "
            f"nvv4l2decoder mjpeg=1 ! nvvidconv ! "
            f"video/x-raw(memory:NVMM),width={width},height={height},format=NV12 ! "
        )
        if needs_bgr:
            # YOLO cameras: tee to both nvjpegenc and BGR
            src = (
                base + "tee name=t ! queue ! nvjpegenc quality=65 ! image/jpeg ! "
                f"appsink name=jpegsink emit-signals=true max-buffers=4 drop=true sync=false "
                f"t. ! queue ! nvvidconv ! video/x-raw,width={width},height={height},format=BGRx ! "
                f"videoconvert ! video/x-raw,format=BGR ! "
                f"appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false"
            )
        else:
            # Non-YOLO: JPEG only, no BGR = no GPU->CPU sync (reduces SLAM contention)
            src = base + "nvjpegenc quality=65 ! image/jpeg ! appsink name=jpegsink emit-signals=true max-buffers=4 drop=true sync=false"
        try:
            return Gst.parse_launch(src)
        except Exception as e:
            print(f"[DEEP] GPU JPEG pipeline failed ({e}), falling back to CPU encode")
            use_gpu_jpeg = False

    if not use_gpu_jpeg:
        # CPU path: single BGR appsink, cv2.imencode later (MJPEG input)
        src = (
            f"v4l2src device={device} io-mode=2 ! "
            f"image/jpeg,width={mjpeg_w},height={mjpeg_h},framerate={fps_val}/1 ! "
            f"nvv4l2decoder mjpeg=1 ! nvvidconv ! "
            f"video/x-raw,width={width},height={height},format=BGRx ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false"
        )
        return Gst.parse_launch(src)

    # Raw fallback: works with YUYV/YUY2 (most USB cameras, avoids not-negotiated)
    return _build_raw_pipeline(device, width, height, fps_val, needs_bgr)


def _build_raw_pipeline(device: str, width: int, height: int, fps_val: int, needs_bgr: bool = True) -> "Gst.Pipeline":
    """Raw (YUYV) pipeline - most compatible, no MJPEG required."""
    src = (
        f"v4l2src device={device} ! "
        f"video/x-raw,width={width},height={height},framerate={fps_val}/1 ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false"
    )
    return Gst.parse_launch(src)


class GstCameraReader:
    """One camera: pipeline + queue for frames. Supports GPU JPEG encoding (nvjpegenc)."""
    def __init__(self, camera_id: str, device: str, width: int = None, height: int = None,
                 use_gpu_jpeg: bool = True, needs_bgr: bool = True):
        self.camera_id = camera_id
        self.device = device
        self.width = width or CAMERA_WIDTH
        self.height = height or CAMERA_HEIGHT
        self.use_gpu_jpeg = use_gpu_jpeg
        self.needs_bgr = needs_bgr
        self.pipeline = None
        self.appsink = None
        self.jpeg_sink = None  # Only set when use_gpu_jpeg
        self.frame_queue = queue.Queue(maxsize=2)
        self.jpeg_queue = queue.Queue(maxsize=2)
        self.frame_count = 0
        self.last_frame = None
        self.last_jpeg = None
        self.lock = threading.Lock()

    def _on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        frame = gst_to_numpy(sample)
        if frame is not None:
            with self.lock:
                self.frame_count += 1
                self.last_frame = frame
            try:
                self.frame_queue.put_nowait(frame)
            except queue.Full:
                pass
        return Gst.FlowReturn.OK

    def _on_jpeg_sample(self, sink):
        """Called when GPU-encoded JPEG is ready."""
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        if buf is None:
            return Gst.FlowReturn.OK
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        try:
            jpeg_bytes = bytes(map_info.data)
            with self.lock:
                self.frame_count += 1
                self.last_jpeg = jpeg_bytes
            try:
                self.jpeg_queue.put_nowait(jpeg_bytes)
            except queue.Full:
                pass
        finally:
            buf.unmap(map_info)
        return Gst.FlowReturn.OK

    def build_and_connect(self):
        if not GST_AVAILABLE or Gst is None:
            return False
        self.pipeline = build_gst_pipeline(
            self.device, self.width, self.height, CAMERA_FPS or 15,
            use_gpu_jpeg=self.use_gpu_jpeg,
            needs_bgr=self.needs_bgr
        )
        self.appsink = self.pipeline.get_by_name("sink")
        if self.appsink is None and self.jpeg_sink is None:
            return False
        self.jpeg_sink = self.pipeline.get_by_name("jpegsink")
        if self.use_gpu_jpeg and self.jpeg_sink is None:
            self.use_gpu_jpeg = False  # Pipeline fell back to CPU-only
        # Attach bus watch for errors/info
        try:
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            def _on_bus(_, msg):
                t = msg.type
                if t == Gst.MessageType.ERROR:
                    err, dbg = msg.parse_error()
                    print(f"[DEEP] {self.camera_id} ERROR: {err} | {dbg}")
                elif t == Gst.MessageType.WARNING:
                    w, dbg = msg.parse_warning()
                    print(f"[DEEP] {self.camera_id} WARN: {w} | {dbg}")
            bus.connect("message", _on_bus)
        except Exception as e:
            print(f"[DEEP] {self.camera_id} bus watch add failed: {e}")
        for s in [s for s in [self.appsink, self.jpeg_sink] if s is not None]:
                try:
                    s.set_property("emit-signals", True)
                    s.set_property("max-buffers", 2)
                    s.set_property("drop", True)
                    s.set_property("sync", False)
                except Exception:
                    pass
        if self.appsink is not None:
            self.appsink.connect("new-sample", self._on_new_sample)
        if self.jpeg_sink is not None:
            self.jpeg_sink.connect("new-sample", self._on_jpeg_sample)
            print(f"[DEEP] {self.camera_id} using GPU JPEG encoding (nvjpegenc)")
        return True

    def set_playing(self):
        if self.pipeline:
            print(f"[DEEP] set PLAYING {self.camera_id} -> {self.device}")
            self.pipeline.set_state(Gst.State.PLAYING)

    def get_frame(self):
        with self.lock:
            if self.last_frame is not None:
                return self.last_frame.copy()
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None

    def get_jpeg(self):
        """Get GPU-encoded JPEG bytes if available. Returns None if not using GPU JPEG."""
        if not self.use_gpu_jpeg or self.jpeg_sink is None:
            return None
        with self.lock:
            if self.last_jpeg is not None:
                return self.last_jpeg
        try:
            return self.jpeg_queue.get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)


class YOLOTensorRT:
    """YOLOv8 TensorRT (.engine) detector. Prefer .engine on Jetson."""
    def __init__(self, model_path: str = None, confidence: float = YOLO_CONFIDENCE):
        self.model = None
        self.path = model_path or YOLO_MODEL_GENERAL
        self.confidence = confidence
        self.ready = False

    def load(self):
        if not YOLO_AVAILABLE:
            print("[DEEP] Ultralytics not available")
            return False
        try:
            if self.path and os.path.isfile(self.path):
                self.model = YOLO(self.path)
                print(f"[DEEP] YOLO TensorRT loaded: {self.path}")
            else:
                self.model = YOLO("yolov8n.pt")
                print("[DEEP] YOLO fallback: yolov8n.pt")
            self.ready = True
            return True
        except Exception as e:
            print(f"[DEEP] YOLO load error: {e}")
            return False

    def detect(self, frame) -> list:
        if not self.ready or self.model is None or frame is None:
            return []
        try:
            results = self.model(frame, conf=self.confidence, verbose=False)
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
                        "center": [(int(box.xyxy[0][0]) + int(box.xyxy[0][2])) // 2, (int(box.xyxy[0][1]) + int(box.xyxy[0][3])) // 2],
                    })
            return out
        except Exception as e:
            return []


def detect_qr(frame) -> list:
    if not PYZBAR_AVAILABLE or frame is None:
        return []
    try:
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        decoded = pyzbar.decode(gray)
        return [{
            "data": obj.data.decode("utf-8", errors="ignore"),
            "type": obj.type,
            "bbox": [obj.rect.left, obj.rect.top, obj.rect.left + obj.rect.width, obj.rect.top + obj.rect.height],
        } for obj in decoded]
    except Exception:
        return []


class DeepStreamYOLONode:
    """Minden kamera: DeepStream (GStreamer). YOLO+QR csak a yolo_camera_ids kamerákon. Runtime toggle for FPS."""
    def __init__(self, port: int = ZMQ_PORT_CAMERA, control_port: int = ZMQ_PORT_CAMERA_CONTROL,
                 model_path: str = None, yolo_camera_ids: tuple = None, qr_on_jetson: bool = False):
        self.port = port
        self.control_port = control_port
        self.yolo_camera_ids = set(yolo_camera_ids or DEFAULT_YOLO_CAMERAS)
        self.qr_on_jetson = qr_on_jetson  # False = QR on laptop (frees CPU for SLAM)
        self.yolo = YOLOTensorRT(model_path=model_path)
        self.cameras: dict = {}
        self.camera_res: dict = {}
        self.context = None
        self.socket = None
        self.control_socket = None
        self.running = False
        self.stream_thread = None
        self.control_thread = None
        self.main_loop = None
        self.target_fps = CAMERA_FPS  # Use full 30 FPS; --fps can raise to 60
        self.detect_every_n = 3  # Run YOLO every 3rd frame (was 2)
        self.qr_every_n = 5  # Run QR every 5th frame (QR is CPU-heavy, ~50-100ms)
        self.last_detections: dict = {}
        self.last_qr_codes: dict = {}
        # Runtime toggles: when False, skip inference for higher FPS (laptop can switch via ZMQ)
        self.enable_yolo = True
        self.enable_qr = True
        self._control_lock = threading.Lock()

    def start(self):
        if not GST_AVAILABLE:
            print("[DEEP] GStreamer not available. Install: sudo apt install gstreamer1.0-* python3-gi")
            return
        if not zmq:
            print("[DEEP] pyzmq not available")
            return
        Gst.init(None)
        OPEN_DELAY = 2.0
        EXTRA_BEFORE_THIRD = 1.5
        slot_order = list(CAMERAS.keys())
        for idx, cam_id in enumerate(slot_order):
            device = CAMERAS.get(cam_id)
            if not device or not os.path.exists(device):
                print(f"[DEEP] Skip {cam_id} ({device}) - missing device")
                continue
            if self.cameras:
                time.sleep(OPEN_DELAY)
                if len(self.cameras) == 2:
                    time.sleep(EXTRA_BEFORE_THIRD)
            # Use configured resolution (default 640x480 for lower latency)
            w, h = CAMERA_WIDTH, CAMERA_HEIGHT
            self.camera_res[cam_id] = (w, h)
            needs_bgr = cam_id in self.yolo_camera_ids
            reader = GstCameraReader(cam_id, device, width=w, height=h, needs_bgr=needs_bgr)
            if reader.build_and_connect():
                self.cameras[cam_id] = reader
                print(f"[DEEP] {cam_id} pipeline ready: {device} ({w}x{h})")
            else:
                print(f"[DEEP] Skip {cam_id} ({device})")
        if not self.cameras:
            print("[DEEP] No cameras started")
            return
        for r in self.cameras.values():
            r.set_playing()
        if self.yolo_camera_ids:
            self.yolo.load()
        else:
            print("[DEEP] No YOLO cameras set, streaming only")
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 60)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f"tcp://*:{self.port}")
        # Control channel: laptop sends {"enable_yolo": bool, "enable_qr": bool} for runtime toggle
        self.control_socket = self.context.socket(zmq.SUB)
        self.control_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.control_socket.setsockopt(zmq.RCVTIMEO, 500)
        self.control_socket.bind(f"tcp://*:{self.control_port}")
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        qr_info = "QR on Jetson" if self.qr_on_jetson else "QR on laptop (GPU-offload)"
        print(f"[DEEP] ZMQ port {self.port} | YOLO on: {sorted(self.yolo_camera_ids) or 'none'} | {qr_info}")
        self.running = True
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()
        print("[DEEP] DeepStream running: all cameras, YOLO/QR toggleable for FPS")
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
        """Receive enable_yolo / enable_qr from laptop (higher FPS when off)."""
        while self.running and self.control_socket:
            try:
                msg = self.control_socket.recv_string()
                data = json.loads(msg)
                with self._control_lock:
                    if "enable_yolo" in data:
                        self.enable_yolo = bool(data["enable_yolo"])
                    if "enable_qr" in data:
                        self.enable_qr = bool(data["enable_qr"])
            except (zmq.Again, json.JSONDecodeError, ValueError):
                pass
            except Exception:
                pass

    def _stream_loop(self):
        frame_interval = 1.0 / self.target_fps
        frame_counters = {cid: 0 for cid in self.cameras}
        while self.running:
            loop_start = time.time()
            with self._control_lock:
                enable_yolo = self.enable_yolo
                enable_qr = self.enable_qr
            for cam_id, reader in self.cameras.items():
                frame = reader.get_frame() if reader.appsink else None
                # For JPEG-only (non-YOLO) pipeline, frame is None; we use get_jpeg()
                jpeg_bytes = reader.get_jpeg() if hasattr(reader, 'get_jpeg') else None
                if frame is None and jpeg_bytes is None:
                    continue
                frame_counters[cam_id] += 1
                run_yolo = (
                    frame is not None
                    and cam_id in self.yolo_camera_ids
                    and (frame_counters[cam_id] % self.detect_every_n) == 0
                )
                run_qr = run_yolo and self.qr_on_jetson and (frame_counters[cam_id] % self.qr_every_n) == 0
                if run_yolo and (enable_yolo or (enable_qr and self.qr_on_jetson)):
                    detections = self.yolo.detect(frame) if enable_yolo else self.last_detections.get(cam_id, [])
                    if enable_yolo:
                        self.last_detections[cam_id] = detections
                    qr_codes = detect_qr(frame) if (enable_qr and run_qr) else (self.last_qr_codes.get(cam_id, []) if self.qr_on_jetson else [])
                    if enable_qr and self.qr_on_jetson:
                        self.last_qr_codes[cam_id] = qr_codes
                else:
                    detections = self.last_detections.get(cam_id, [])
                    qr_codes = self.last_qr_codes.get(cam_id, []) if self.qr_on_jetson else []
                # Prefer GPU-encoded JPEG (nvjpegenc) to reduce CPU load for SLAM
                if jpeg_bytes is not None:
                    jpeg = jpeg_bytes
                else:
                    # CPU encode (fallback when GPU JPEG unavailable)
                    if frame is not None:
                        _, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                        jpeg = enc.tobytes()
                    else:
                        continue  # No frame and no GPU JPEG
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
                    "detections": detections,
                    "qr_codes": qr_codes,
                }
                try:
                    self.socket.send_multipart([json.dumps(header).encode(), jpeg if isinstance(jpeg, bytes) else jpeg.tobytes()], zmq.NOBLOCK)
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
        print("[DEEP] Stopped")


def main():
    import argparse
    ap = argparse.ArgumentParser(description="DeepStream: all cameras, YOLO only on selected (default: front arm)")
    ap.add_argument("--port", type=int, default=ZMQ_PORT_CAMERA)
    ap.add_argument("--control-port", type=int, default=ZMQ_PORT_CAMERA_CONTROL, help="Port for laptop YOLO/QR toggle (FPS)")
    ap.add_argument("--model", type=str, default=None, help="YOLOv8 .engine or .pt path")
    ap.add_argument("--fps", type=int, default=CAMERA_FPS, help="Target FPS per camera (30-60)")
    ap.add_argument("--yolo-cameras", nargs="+", default=list(DEFAULT_YOLO_CAMERAS), help="Camera IDs for YOLO+QR (e.g. front arm)")
    ap.add_argument("--qr-on-jetson", action="store_true",
                    help="Run QR on Jetson (default: off; laptop decodes to free CPU for SLAM)")
    args = ap.parse_args()
    qr_on_jetson = args.qr_on_jetson
    node = DeepStreamYOLONode(port=args.port, control_port=args.control_port, model_path=args.model,
                              yolo_camera_ids=tuple(args.yolo_cameras), qr_on_jetson=qr_on_jetson)
    node.target_fps = min(max(args.fps, 10), 60)  # Clamp 10-60 FPS
    try:
        node.start()
    except KeyboardInterrupt:
        print("\n[DEEP] Interrupt")
    finally:
        node.stop()


if __name__ == "__main__":
    main()
