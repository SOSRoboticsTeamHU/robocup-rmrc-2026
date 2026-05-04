#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Vision Node
=====================================
- Front + Arm: Full GPU MJPEG (nvjpegdec → nvjpegenc) for YOLO; max 2 to avoid NVMM exhaustion
- Backward: Light MJPEG (320x240, no GPU) — saves NVMM for front/arm
- Fallback: OpenCV for non-MJPEG or when GPU pipelines fail
- YOLO11 on front + arm; Landolt, Snapshot, Control, ZMQ; 3 cameras on GUI

Env: VISION_FORCE_OPENCV=1 (skip GPU), VISION_MAX_GPU_CAMERAS=2 (default)
TensorRT: Re-export .engine on target Jetson if "engine across devices" warning appears.
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
from pyzbar import pyzbar
import cv2

# Vision modules
try:
    from vision.yolo11_processor import YOLO11Processor, YOLO11Config
    from vision.landolt_classifier import LandoltClassifier
    from vision.snapshot_mode import capture_hazmat_still, capture_landolt_still, build_snapshot_result
except ImportError:
    from jetson.vision.yolo11_processor import YOLO11Processor, YOLO11Config
    from jetson.vision.landolt_classifier import LandoltClassifier
    from jetson.vision.snapshot_mode import capture_hazmat_still, capture_landolt_still, build_snapshot_result

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
Gst.init(None)


def _bus_message_callback(bus, message, reader):
    """On pipeline error, try to restart up to MAX_PIPELINE_RESTARTS; then stop to avoid log spam."""
    if getattr(reader, "_ignore_bus_errors", False):
        return True
    if message.type != Gst.MessageType.ERROR and message.type != Gst.MessageType.WARNING:
        return True
    err, debug = message.parse_error() if message.type == Gst.MessageType.ERROR else message.parse_warning()
    cam_id = getattr(reader, "camera_id", "?")
    print(f"[VISION] {cam_id} GStreamer: {err} (debug: {debug[:80] if debug else '—'})")
    if message.type != Gst.MessageType.ERROR:
        return True
    pipeline = getattr(reader, "pipeline", None)
    if not pipeline:
        return True
    max_restarts = getattr(reader, "MAX_PIPELINE_RESTARTS", 3)
    restart_count = getattr(reader, "_restart_count", 0)
    if restart_count >= max_restarts:
        print(f"[VISION] {cam_id} pipeline error limit ({max_restarts}) reached; not restarting (use placeholder)")
        reader._ignore_bus_errors = True
        try:
            bus.remove_signal_watch()
        except Exception:
            pass
        try:
            pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        try:
            reader.pipeline = None
        except Exception:
            pass
        return True
    try:
        pipeline.set_state(Gst.State.NULL)
        time.sleep(0.3)
        ret = pipeline.set_state(Gst.State.PLAYING)
        reader._restart_count = restart_count + 1
        if ret == Gst.StateChangeReturn.FAILURE:
            print(f"[VISION] {cam_id} pipeline restart failed")
        else:
            print(f"[VISION] {cam_id} pipeline restarted after error ({reader._restart_count}/{max_restarts})")
    except Exception as e:
        print(f"[VISION] Pipeline restart error: {e}")
    return True


# ====================== CAMERA READERS ======================

class FullGPUMJPEGCameraReader:
    MAX_PIPELINE_RESTARTS = 3

    def __init__(self, camera_id: str, device: str, width: int = 640, height: int = 480, fps: int = 15):
        self.camera_id = camera_id
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.appsink = None
        self._last_jpeg = None
        self._lock = threading.Lock()
        self._restart_count = 0
        self._ignore_bus_errors = False
        self._last_update_ts = 0.0

    def build_and_connect(self, validate_first_frame: bool = True, timeout_sec: float = 3.0) -> bool:
        # Queue 2 + appsink 2 so pipeline does not block after one frame when running 3 cams
        pipeline_str = f"""
            v4l2src device={self.device} !
            queue max-size-buffers=2 leaky=downstream !
            image/jpeg,width={self.width},height={self.height},framerate={self.fps}/1 !
            nvjpegdec !
            nvvidconv !
            video/x-raw(memory:NVMM),format=NV12 !
            queue max-size-buffers=2 leaky=downstream !
            nvjpegenc quality={JPEG_QUALITY} !
            appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false
        """
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsink = self.pipeline.get_by_name("sink")
            self.appsink.set_property("emit-signals", True)
            self.appsink.connect("new-sample", self._on_new_sample)
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", _bus_message_callback, self)
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                return False
            if validate_first_frame:
                deadline = time.time() + timeout_sec
                ctx = GLib.MainContext.default()
                while time.time() < deadline:
                    while ctx.iteration(False):
                        pass
                    if self.get_jpeg():
                        break
                    time.sleep(0.02)
                if not self.get_jpeg():
                    self.stop()
                    return False
            print(f"[VISION] {self.camera_id} → FULL GPU OK: {self.device}")
            return True
        except Exception as e:
            print(f"[VISION] Full GPU failed for {self.camera_id}: {e}")
            return False

    def _on_new_sample(self, sink):
        try:
            sample = sink.emit("pull-sample")
            if not sample:
                return Gst.FlowReturn.OK
            buf = sample.get_buffer()
            if buf is None:
                return Gst.FlowReturn.OK
            if buf.get_size() == 0:
                return Gst.FlowReturn.OK
            success, mapinfo = buf.map(Gst.MapFlags.READ)
            if not success:
                return Gst.FlowReturn.OK
            try:
                with self._lock:
                    self._last_jpeg = bytes(mapinfo.data)
                    self._last_update_ts = time.time()
            finally:
                buf.unmap(mapinfo)
        except Exception:
            pass
        return Gst.FlowReturn.OK

    def get_jpeg(self):
        with self._lock:
            return self._last_jpeg

    def stop(self):
        if self.pipeline:
            self._ignore_bus_errors = True
            try:
                bus = self.pipeline.get_bus()
                bus.remove_signal_watch()
            except Exception:
                pass
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None


class LightMJPEGCameraReader:
    MAX_PIPELINE_RESTARTS = 3

    def __init__(self, camera_id: str, device: str, width: int = 640, height: int = 480, fps: int = 30,
                 max_pipeline_restarts: int = 3):
        self.camera_id = camera_id
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.MAX_PIPELINE_RESTARTS = max_pipeline_restarts
        self.pipeline = None
        self.appsink = None
        self._last_jpeg = None
        self._lock = threading.Lock()
        self._restart_count = 0
        self._ignore_bus_errors = False
        self._last_update_ts = 0.0

    def build_and_connect(self) -> bool:
        # Queue 2 + appsink 2 so source does not block (avoids one frame then stop)
        pipeline_str = f"""
            v4l2src device={self.device} !
            queue max-size-buffers=2 leaky=downstream !
            image/jpeg,width={self.width},height={self.height},framerate={self.fps}/1 !
            appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false
        """
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsink = self.pipeline.get_by_name("sink")
            self.appsink.set_property("emit-signals", True)
            self.appsink.connect("new-sample", self._on_new_sample)
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", _bus_message_callback, self)
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                return False
            print(f"[VISION] {self.camera_id} → Light MJPEG OK: {self.device}")
            return True
        except Exception as e:
            print(f"[VISION] Light pipeline failed for {self.camera_id}: {e}")
            return False

    def _on_new_sample(self, sink):
        try:
            sample = sink.emit("pull-sample")
            if not sample:
                return Gst.FlowReturn.OK
            buf = sample.get_buffer()
            if buf is None:
                return Gst.FlowReturn.OK
            if buf.get_size() == 0:
                return Gst.FlowReturn.OK
            success, mapinfo = buf.map(Gst.MapFlags.READ)
            if not success:
                return Gst.FlowReturn.OK
            try:
                with self._lock:
                    self._last_jpeg = bytes(mapinfo.data)
                    self._last_update_ts = time.time()
            finally:
                buf.unmap(mapinfo)
        except Exception:
            pass
        return Gst.FlowReturn.OK

    def get_jpeg(self):
        with self._lock:
            return self._last_jpeg

    def stop(self):
        if self.pipeline:
            self._ignore_bus_errors = True
            try:
                bus = self.pipeline.get_bus()
                bus.remove_signal_watch()
            except Exception:
                pass
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None


class OpenCVCameraReader:
    """
    Left/Right: CPU-only reader using OpenCV. No GStreamer, no NVMM/GPU.
    Low resolution (320x240) + low FPS (10) to save memory and avoid V4L2 allocation errors.
    """

    def __init__(self, camera_id: str, device: str, width: int = 320, height: int = 240, fps: int = 10):
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
        self._last_frame_ts = 0.0
        self._last_update_ts = 0.0

    def _open_capture(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap or not cap.isOpened():
            cap = cv2.VideoCapture(self.device)
        if not cap or not cap.isOpened():
            return None
        # Set MJPEG first (if supported) — much faster than YUYV on CPU
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except Exception:
            pass
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        return cap

    def build_and_connect(self) -> bool:
        self.cap = self._open_capture()
        if not self.cap:
            print(f"[VISION] {self.camera_id} OpenCV open failed: {self.device}")
            return False
        self._running = True
        self._last_frame_ts = time.time()
        self._last_update_ts = time.time()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        print(f"[VISION] {self.camera_id} → OpenCV CPU OK: {self.device} ({self.width}x{self.height}@{self.fps})")
        return True

    def _reader_loop(self):
        interval = 1.0 / max(self.fps, 1)
        frame_count = 0
        failed_reads = 0
        print(f"[VISION] {self.camera_id} reader loop starting...")
        while self._running:
            if not self.cap or not self.cap.isOpened():
                self.cap = self._open_capture()
                if not self.cap:
                    time.sleep(0.2)
                    continue
            ret, frame = self.cap.read()
            if not ret:
                failed_reads += 1
                if failed_reads >= 15 or (time.time() - self._last_frame_ts) > 1.5:
                    try:
                        if self.cap:
                            self.cap.release()
                    except Exception:
                        pass
                    self.cap = self._open_capture()
                    failed_reads = 0
                time.sleep(0.05)
                continue
            failed_reads = 0
            frame_count += 1
            if frame_count == 1:
                print(f"[VISION] {self.camera_id} captured first frame!")
            h, w = frame.shape[:2]
            if w != self.width or h != self.height:
                frame = cv2.resize(frame, (self.width, self.height))
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if ok:
                with self._lock:
                    self._last_jpeg = buf.tobytes()
                self._last_frame_ts = time.time()
                self._last_update_ts = self._last_frame_ts
            time.sleep(max(0.01, interval * 0.3))  # Minimal sleep — cap.read() is the bottleneck

    def force_reopen(self):
        try:
            if self.cap:
                self.cap.release()
        except Exception:
            pass
        self.cap = None

    def get_jpeg(self):
        with self._lock:
            return self._last_jpeg

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=0.5)
        if self.cap:
            self.cap.release()


class VisionNode:
    def __init__(self):
        self.cameras = {}
        self.running = False
        self.stream_thread = None
        self.control_thread = None
        self.inference_thread = None
        self.main_loop = None

        self.yolo = YOLO11Processor(
            models_dir=JETSON_DIR / "models" / "engines",
            config=YOLO11Config(confidence_threshold=YOLO_CONFIDENCE),
            default_task="detect"
        )

        self.landolt_classifier = LandoltClassifier(use_rule_based_fallback=True)

        self.context = None
        self.socket = None
        self.control_socket = None
        self.snapshot_request_socket = None
        self.snapshot_result_socket = None

        self.last_detections = {}
        self.last_qr_codes = {}
        self.last_landolt_readings = {}
        # QR output file (rulebook: write QR data to file for judge)
        self._qr_output_path = os.path.expanduser("~/qr_detections.txt")
        self._qr_seen_recently = {}  # data -> timestamp (deduplicate within 60s)
        # ArUco fiducial detection
        self._aruco_dict = None
        self._aruco_params = None
        try:
            self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self._aruco_params = cv2.aruco.DetectorParameters()
        except Exception:
            pass  # cv2.aruco not available
        self._latest_arm_frame = None
        self._latest_arm_detections = []
        self._latest_arm_landolt = []

        self._inference_lock = threading.Lock()
        self._inference_frames = {}   # cam_id -> frame for background inference
        self._placeholder_jpeg = None
        self._frame_nums = {}
        self._fallback_tried = set()   # cam_id where we already tried Light/OpenCV fallback
        self._last_real_frame_ts = {}
        self._opencv_recovery_tried = set()
        self._last_full_restart_ts = {}  # cam_id -> time of last full restart (cable/stream recovery)
        self._main_loop_thread = None

    def _supports_mjpeg(self, device: str) -> bool:
        """Check if a video device supports MJPEG capture."""
        try:
            import subprocess
            result = subprocess.run(
                ["v4l2-ctl", "--device", device, "--list-formats"],
                capture_output=True,
                text=True,
                timeout=2
            )
            return "MJPG" in result.stdout or "Motion-JPEG" in result.stdout
        except Exception:
            return False

    def _resolve_camera_devices(self):
        """Resolve camera_id -> /dev/videoX. Use CAMERAS + robot_params; auto-assign missing so we get 3 cams."""
        devices = dict(CAMERAS)
        source = "shared.constants"
        # Env override: CAMERA_DEVICES="0,1,2" for consecutive (front=0, arm=1, backward=2)
        env_ids = os.environ.get("CAMERA_DEVICES", "").strip()
        if env_ids:
            parts = [p.strip() for p in env_ids.split(",") if p.strip()]
            if len(parts) >= 3:
                want_names = ["front", "arm", "backward"]
                devices = {want_names[i]: f"/dev/video{parts[i]}" for i in range(3)}
                source = "CAMERA_DEVICES env"
                print(f"[VISION] Using CAMERA_DEVICES={env_ids} -> {devices}")
        else:
            rp = JETSON_DIR / "config" / "robot_params.yaml"
            if rp.exists():
                try:
                    import yaml
                    with open(rp) as f:
                        data = yaml.safe_load(f) or {}
                    for k, v in (data.get("cameras") or {}).items():
                        if k in ("front", "arm", "backward") and isinstance(v, str) and v.startswith("/dev/"):
                            devices[k] = v
                            source = "jetson/config/robot_params.yaml"
                except Exception:
                    pass
        # Valid = devices that exist (MJPEG or not; we fall back to OpenCV for non-MJPEG)
        all_existing = [f"/dev/video{i}" for i in range(12) if os.path.exists(f"/dev/video{i}")]
        mjpeg_devices = [d for d in all_existing if self._supports_mjpeg(d)]
        valid = {k: v for k, v in devices.items() if k in ("front", "arm", "backward") and v and os.path.exists(v)}
        if all_existing:
            print(f"[VISION] Existing devices: {all_existing}")
            print(f"[VISION] MJPEG-capable: {mjpeg_devices}")
        want = ["front", "arm", "backward"]
        missing = [c for c in want if c not in valid]
        # Auto-assign from ALL existing devices (not just MJPEG) when configured ones missing
        if missing and len(all_existing) >= len(missing):
            used = set(valid.values())
            for cam in missing:
                for dev in all_existing:
                    if dev not in used:
                        valid[cam] = dev
                        used.add(dev)
                        print(f"[VISION] Auto-assign {cam} -> {dev}")
                        break
                print(f"[VISION] Camera mapping source: {source}")
                print(f"[VISION] Resolved cameras: front={valid.get('front')}, arm={valid.get('arm')}, backward={valid.get('backward')}")
        return valid

    def _build_placeholder_jpeg(self):
        """Single 'No Signal' JPEG so we always send a frame per camera (GUI shows all 4 slots)."""
        buf = io.BytesIO()
        img = Image.new("RGB", (CAMERA_WIDTH, CAMERA_HEIGHT), (24, 24, 24))
        try:
            from PIL import ImageDraw, ImageFont
            draw = ImageDraw.Draw(img)
            text = "No Signal"
            try:
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 32)
            except Exception:
                font = ImageFont.load_default()
            bbox = draw.textbbox((0, 0), text, font=font)
            tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
            x = (CAMERA_WIDTH - tw) // 2
            y = (CAMERA_HEIGHT - th) // 2
            draw.text((x, y), text, fill=(80, 80, 80), font=font)
        except Exception:
            pass
        img.save(buf, format="JPEG", quality=70)
        return buf.getvalue()

    def _try_add_backward_camera(self, resolved: dict, force_opencv: bool = False) -> None:
        """Add backward camera.

        Default path: OpenCV reader. The Light MJPEG GStreamer pipeline has a
        bug where the backward USB camera only delivers a single frame before
        the queue stalls (caps negotiation mismatch / queue back-pressure).
        Override with BACKWARD_FORCE_OPENCV=0 to fall back to the legacy MJPEG
        path.
        """
        if "backward" in self.cameras:
            return
        configured = resolved.get("backward")
        used = set(getattr(r, "device", "") for r in self.cameras.values())
        all_existing = [f"/dev/video{i}" for i in range(12) if os.path.exists(f"/dev/video{i}")]
        candidates = [configured] if configured and configured not in used else []
        candidates += [d for d in all_existing if d not in used and d not in candidates]
        # BACKWARD_FORCE_OPENCV defaults to "1" (the reliable path). Set 0 to
        # try Light MJPEG again (e.g. for benchmarking).
        backward_opencv = os.environ.get("BACKWARD_FORCE_OPENCV", "1").strip().lower() in ("1", "true", "yes")
        if backward_opencv:
            for device in candidates:
                if not device or not os.path.exists(device):
                    continue
                # OpenCV CPU reader: own thread, auto-reopens, no GStreamer caps
                # negotiation. 320x240@15 keeps CPU and bandwidth low.
                reader = OpenCVCameraReader("backward", device, width=320, height=240, fps=15)
                if reader.build_and_connect():
                    self.cameras["backward"] = reader
                    if device != configured:
                        print(f"[VISION] backward: configured {configured} failed, using {device} instead")
                    print("[VISION] backward: using OpenCV reader (BACKWARD_FORCE_OPENCV=1)")
                    time.sleep(0.6)
                    return
            print(f"[VISION] backward: no signal via OpenCV (tried {candidates}). Check CAMERA_DEVICES.")
            return
        # Legacy path: Light MJPEG, then OpenCV fallback inside _try_add_camera.
        for device in candidates:
            if not device or not os.path.exists(device):
                continue
            if self._try_add_camera("backward", device, prefer_gpu=False, force_opencv=force_opencv):
                if device != configured:
                    print(f"[VISION] backward: configured {configured} failed, using {device} instead")
                return
        print(f"[VISION] backward: no signal (tried {candidates}). Check CAMERA_DEVICES or USB ports.")

    def _try_add_camera(self, cam_id: str, device: str, prefer_gpu: bool = True, force_opencv: bool = False) -> bool:
        """Try MJPEG reader first, fall back to OpenCV for non-MJPEG cameras."""
        if cam_id in self.cameras:
            return True
        if not device or not os.path.exists(device):
            return False
        w_cv, h_cv = (320, 240) if cam_id == "backward" else (640, 480)
        if force_opencv:
            reader = OpenCVCameraReader(cam_id, device, width=w_cv, height=h_cv, fps=15)
            if reader.build_and_connect():
                self.cameras[cam_id] = reader
                time.sleep(0.8)
                print(f"[VISION] {cam_id} OpenCV (VISION_FORCE_OPENCV)")
                return True
            return False
        # Full GPU MJPEG: only for front+arm (YOLO). Backward uses Light — 3x nvjpegdec exhausts NVMM.
        if prefer_gpu and self._supports_mjpeg(device):
            reader = FullGPUMJPEGCameraReader(cam_id, device, width=640, height=480, fps=15)
            if reader.build_and_connect():
                self.cameras[cam_id] = reader
                time.sleep(1.5)
                return True
        # Backward or non-MJPEG: try Light MJPEG (no GPU)
        if self._supports_mjpeg(device):
            light_restarts = 0 if cam_id == "backward" else 3
            w, h = (320, 240)  # backward: lower res saves bandwidth; front/arm fallback use 320x240
            f = 15 if cam_id == "backward" else 10
            reader = LightMJPEGCameraReader(
                cam_id,
                device,
                width=w,
                height=h,
                fps=f,
                max_pipeline_restarts=light_restarts,
            )
            if reader.build_and_connect():
                self.cameras[cam_id] = reader
                time.sleep(0.8)
                return True
        # Fallback: OpenCV (works with any camera format: YUYV, etc.)
        reader = OpenCVCameraReader(cam_id, device, width=w_cv, height=h_cv, fps=15)
        if reader.build_and_connect():
            self.cameras[cam_id] = reader
            time.sleep(0.8)
            print(f"[VISION] {cam_id} using OpenCV fallback (no MJPEG)")
            return True
        return False

    def _recover_camera_to_opencv(self, cam_id: str, reader) -> bool:
        """One-shot recovery path when a camera stops producing frames."""
        if cam_id in self._opencv_recovery_tried:
            return False
        device = getattr(reader, "device", None)
        if not device:
            return False
        w = 320 if cam_id == "backward" else 640
        h = 240 if cam_id == "backward" else 480
        old_reader = reader
        try:
            old_reader.stop()
        except Exception:
            pass
        new_reader = OpenCVCameraReader(cam_id, device, width=w, height=h, fps=15)
        if new_reader.build_and_connect():
            self.cameras[cam_id] = new_reader
            self._opencv_recovery_tried.add(cam_id)
            self._last_real_frame_ts[cam_id] = time.time()
            print(f"[VISION] {cam_id} recovered to OpenCV after stall")
            return True
        self.cameras[cam_id] = old_reader
        return False

    def _full_restart_camera(self, cam_id: str, reader) -> bool:
        """Full teardown + re-add when stream has been dead for a long time (e.g. cable wiggle)."""
        device = getattr(reader, "device", None)
        if not device:
            return False
        try:
            reader.stop()
        except Exception:
            pass
        del self.cameras[cam_id]
        self._opencv_recovery_tried.discard(cam_id)
        self._fallback_tried.discard(cam_id)
        time.sleep(2.0)  # USB settle before re-open
        force_opencv = getattr(self, "_vision_force_opencv", False)
        gpu_count = sum(1 for r in self.cameras.values() if type(r).__name__ == "FullGPUMJPEGCameraReader")
        max_gpu = getattr(self, "_vision_max_gpu", 2)
        prefer_gpu = not force_opencv and cam_id in ("front", "arm") and gpu_count < max_gpu
        if self._try_add_camera(cam_id, device, prefer_gpu=prefer_gpu, force_opencv=force_opencv):
            self._last_real_frame_ts[cam_id] = time.time()
            self._last_full_restart_ts[cam_id] = time.time()
            print(f"[VISION] {cam_id} full restart OK (stream was dead)")
            return True
        # Fallback: add OpenCV reader that keeps retrying — recovers when cable returns
        w, h = (320, 240) if cam_id == "backward" else (640, 480)
        retry_reader = OpenCVCameraReader(cam_id, device, width=w, height=h, fps=15)
        if retry_reader.build_and_connect():
            self.cameras[cam_id] = retry_reader
            self._last_real_frame_ts[cam_id] = time.time()
            self._last_full_restart_ts[cam_id] = time.time()
            print(f"[VISION] {cam_id} full restart → OpenCV retry (will recover when cable OK)")
            return True
        # Cable likely unplugged: start reader anyway so it keeps retrying when reconnected
        retry_reader.cap = None
        retry_reader._running = True
        retry_reader._thread = threading.Thread(target=retry_reader._reader_loop, daemon=True)
        retry_reader._thread.start()
        self.cameras[cam_id] = retry_reader
        self._last_full_restart_ts[cam_id] = time.time()
        print(f"[VISION] {cam_id} full restart → retry loop (will recover when cable reconnected)")
        return True

    def start(self):
        force_opencv = os.environ.get("VISION_FORCE_OPENCV", "").strip().lower() in ("1", "true", "yes")
        max_gpu = max(0, int(os.environ.get("VISION_MAX_GPU_CAMERAS", "2")))
        self._vision_force_opencv = force_opencv
        self._vision_max_gpu = max_gpu

        print("[VISION] Starting 3 cameras with hybrid GPU MJPEG...")
        if force_opencv:
            print("[VISION] VISION_FORCE_OPENCV=1 — using OpenCV for all cameras")
        elif max_gpu < 2:
            print(f"[VISION] VISION_MAX_GPU_CAMERAS={max_gpu} — limiting Full GPU pipelines")
        print("[VISION] Backward: Light MJPEG 320x240. Front+arm: Full GPU 640x480. Fallback: OpenCV.")

        resolved = self._resolve_camera_devices()

        # Run GLib main loop in a thread before adding cameras so GStreamer callbacks run for all 3
        self.main_loop = GLib.MainLoop()
        self._main_loop_thread = threading.Thread(target=self.main_loop.run, daemon=True)
        self._main_loop_thread.start()
        time.sleep(0.5)

        # Backward first (Light MJPEG, no GPU) — then front, arm with stagger
        self._try_add_backward_camera(resolved, force_opencv=force_opencv)
        time.sleep(1.0)
        # Front + Arm: Full GPU MJPEG (staggered), fallback to OpenCV
        for i, cam_id in enumerate(["front", "arm"]):
            if i > 0:
                time.sleep(1.5)  # Stagger Full GPU startup to reduce NVMM allocation spikes
            device = resolved.get(cam_id)
            gpu_count = sum(1 for r in self.cameras.values() if type(r).__name__ == "FullGPUMJPEGCameraReader")
            prefer_gpu = not force_opencv and gpu_count < max_gpu
            self._try_add_camera(cam_id, device, prefer_gpu=prefer_gpu, force_opencv=force_opencv)

        # Wait for all cameras to warm up and produce first frames  
        if self.cameras:
            print(f"[VISION] Waiting for {len(self.cameras)} cameras to warm up...")
            time.sleep(1.5)  # Give cameras time to produce first frames
            for cid, reader in self.cameras.items():
                jpeg = reader.get_jpeg()
                if jpeg:
                    print(f"[VISION] {cid} ready: {len(jpeg)} bytes")
                else:
                    print(f"[VISION] {cid} warming up (will use placeholder initially)...")

        if not self.cameras:
            print("[VISION] No cameras started!")
            if self.main_loop:
                self.main_loop.quit()
            return

        # ZMQ — low SNDHWM so we don't queue many frames when laptop is slow (reduces latency)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 3)
        self.socket.setsockopt(zmq.SNDTIMEO, 0)
        self.socket.bind(f"tcp://*:{ZMQ_PORT_CAMERA}")

        self.control_socket = self.context.socket(zmq.SUB)
        self.control_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.control_socket.bind(f"tcp://*:{ZMQ_PORT_CAMERA_CONTROL}")

        self.snapshot_request_socket = self.context.socket(zmq.SUB)
        self.snapshot_request_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.snapshot_request_socket.bind(f"tcp://*:{ZMQ_PORT_SNAPSHOT_REQUEST}")

        self.snapshot_result_socket = self.context.socket(zmq.PUB)
        self.snapshot_result_socket.bind(f"tcp://*:{ZMQ_PORT_SNAPSHOT_RESULT}")

        self._placeholder_jpeg = self._build_placeholder_jpeg()
        self._frame_nums = {cid: 0 for cid in self.cameras}
        now = time.time()
        self._last_real_frame_ts = {cid: now for cid in self.cameras}
        self.running = True

        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

        self.snapshot_thread = threading.Thread(target=self._snapshot_request_loop, daemon=True)
        self.snapshot_thread.start()

        self.inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self.inference_thread.start()

        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()

        print(f"[VISION] Vision node running - {len(self.cameras)} cameras")
        print(f"[VISION] YOLO active on: {YOLO_CAMERAS}")

        try:
            if self._main_loop_thread:
                self._main_loop_thread.join()
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
            for cam_id in ["front", "arm"]:
                if cam_id not in self.cameras:
                    continue
                jpeg = self.cameras[cam_id].get_jpeg()
                if not jpeg:
                    continue
                frame_rgb = self._jpeg_to_rgb(jpeg)
                if frame_rgb is None:
                    continue

                detections = self.yolo.detect_combined(frame_rgb) if self.yolo.combined_ready else self.yolo.detect(frame_rgb)

                landolt_readings = []
                landolt_dets = [d for d in detections if "landolt" in d.get("class_name", "").lower() or "ring" in d.get("class_name", "").lower()]
                if landolt_dets:
                    landolt_readings = self.landolt_classifier.predict_from_detections(frame_rgb, landolt_dets)

                self.last_detections[cam_id] = detections
                self.last_landolt_readings[cam_id] = landolt_readings

                if cam_id == "arm":
                    self._latest_arm_frame = frame_rgb
                    self._latest_arm_detections = detections
                    self._latest_arm_landolt = landolt_readings

                # QR detection + file output (rulebook requirement)
                qr_codes = self.last_qr_codes.get(cam_id, [])
                for qr in qr_codes:
                    data = qr.get("data", "")
                    if data:
                        now = time.time()
                        last_seen = self._qr_seen_recently.get(data, 0)
                        if now - last_seen > 60.0:  # deduplicate within 60s
                            self._qr_seen_recently[data] = now
                            try:
                                from datetime import datetime
                                line = f"{datetime.now().isoformat()} | cam={cam_id} | {data}\n"
                                with open(self._qr_output_path, "a") as f:
                                    f.write(line)
                            except Exception:
                                pass

                # ArUco fiducial detection on front camera
                if cam_id == "front" and self._aruco_dict is not None:
                    try:
                        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
                        corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict, parameters=self._aruco_params)
                        if ids is not None and len(ids) > 0:
                            for i, marker_id in enumerate(ids.flatten()):
                                # Approximate center in pixels → rough world position
                                c = corners[i][0].mean(axis=0)
                                # Publish fiducial on snapshot result port for SLAM integration
                                fid_msg = {
                                    "msg_type": "fiducial",
                                    "id": int(marker_id),
                                    "pixel_x": float(c[0]),
                                    "pixel_y": float(c[1]),
                                    "timestamp": time.time(),
                                }
                                if self.snapshot_result_socket:
                                    try:
                                        self.snapshot_result_socket.send_multipart(
                                            [json.dumps(fid_msg).encode()], zmq.NOBLOCK
                                        )
                                    except (zmq.Again, zmq.ZMQError):
                                        pass
                    except Exception:
                        pass

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
        """Send one frame per camera every loop so GUI always gets all 4. Use placeholder when reader has no frame yet."""
        placeholder = self._placeholder_jpeg
        frame_nums = self._frame_nums
        stall_timeout_sec = 2.0       # quick recovery: force_reopen or OpenCV fallback
        full_restart_sec = 15.0       # after 15s no frames: full teardown + re-add (cable/stream recovery)
        min_restart_interval = 45.0   # don't full-restart same camera more than once per 45s
        while self.running:
            try:
                for cam_id, reader in list(self.cameras.items()):
                    # If GPU pipeline gave up after N restarts, try Light MJPEG or OpenCV once
                    if (cam_id not in self._fallback_tried
                            and getattr(reader, "_restart_count", 0) >= getattr(reader, "MAX_PIPELINE_RESTARTS", 3)
                            and reader.get_jpeg() is None):
                        self._fallback_tried.add(cam_id)
                        device = getattr(reader, "device", None)
                        if device:
                            try:
                                reader.stop()
                            except Exception:
                                pass
                            del self.cameras[cam_id]
                            force_opencv = getattr(self, "_vision_force_opencv", False)
                            if self._try_add_camera(cam_id, device, prefer_gpu=False, force_opencv=force_opencv):
                                print(f"[VISION] {cam_id} fallback to Light MJPEG/OpenCV OK")
                            else:
                                self.cameras[cam_id] = reader  # put back so we keep using placeholder
                                self._fallback_tried.discard(cam_id)
                    reader = self.cameras.get(cam_id)
                    if reader is None:
                        continue
                    jpeg = reader.get_jpeg()
                    now_ts = time.time()
                    last_update_ts = getattr(reader, "_last_update_ts", 0.0)
                    if last_update_ts > 0:
                        self._last_real_frame_ts[cam_id] = last_update_ts
                    stale_for = now_ts - self._last_real_frame_ts.get(cam_id, 0.0)
                    last_restart = self._last_full_restart_ts.get(cam_id, 0.0)
                    # Full restart: stream dead for 15s+ and we haven't restarted this cam in 45s
                    if stale_for > full_restart_sec and (now_ts - last_restart) > min_restart_interval:
                        if self._full_restart_camera(cam_id, reader):
                            reader = self.cameras.get(cam_id)
                            if reader is not None:
                                jpeg = reader.get_jpeg()
                        continue
                    if stale_for > stall_timeout_sec:
                        if isinstance(reader, OpenCVCameraReader):
                            reader.force_reopen()
                            self._last_real_frame_ts[cam_id] = now_ts
                            print(f"[VISION] {cam_id} OpenCV stale frame recovery")
                        else:
                            if self._recover_camera_to_opencv(cam_id, reader):
                                reader = self.cameras.get(cam_id)
                                if reader is not None:
                                    jpeg = reader.get_jpeg()
                                self._last_real_frame_ts[cam_id] = now_ts
                    if not jpeg and placeholder:
                        jpeg = placeholder
                    if not jpeg:
                        continue
                    frame_nums[cam_id] = frame_nums.get(cam_id, 0) + 1
                    is_placeholder = jpeg is placeholder
                    w = CAMERA_WIDTH if is_placeholder else getattr(reader, "width", CAMERA_WIDTH)
                    h = CAMERA_HEIGHT if is_placeholder else getattr(reader, "height", CAMERA_HEIGHT)
                    fps_val = getattr(reader, "fps", CAMERA_FPS)
                    detections = self.last_detections.get(cam_id, [])
                    landolt_readings = self.last_landolt_readings.get(cam_id, [])

                    header = {
                        "msg_type": "camera_frame",
                        "camera_id": cam_id,
                        "timestamp": time.time(),
                        "width": w,
                        "height": h,
                        "fps": fps_val,
                        "frame_num": frame_nums[cam_id],
                        "task": self.yolo.get_current_task() if cam_id in YOLO_CAMERAS else "none",
                        "detections": detections,
                        "qr_codes": self.last_qr_codes.get(cam_id, []),
                        "landolt_readings": [
                            {"direction": getattr(r, "direction_name", str(r)), "confidence": getattr(r, "confidence", 0)}
                            for r in landolt_readings
                        ],
                    }

                    if not jpeg:
                        continue
                    try:
                        self.socket.send_multipart([json.dumps(header).encode(), jpeg], flags=zmq.NOBLOCK)
                    except zmq.Again:
                        pass
                    except Exception:
                        pass
            except Exception as e:
                print(f"[VISION] stream loop error: {e}")
                time.sleep(0.1)

            time.sleep(0.04)

    def _snapshot_request_loop(self):
        """Handle snapshot requests from autonomy executor (HAZMAT/Landolt/keypad_detect)."""
        from vision.snapshot_mode import capture_hazmat_still, capture_landolt_still
        while self.running:
            try:
                msg_raw = self.snapshot_request_socket.recv_json(zmq.NOBLOCK)
                req_type = msg_raw.get("result_type", msg_raw.get("request", "snapshot"))
                frame = self._latest_arm_frame
                if frame is None:
                    time.sleep(0.1)
                    continue

                if req_type == "keypad_detect":
                    # Run general YOLO on arm camera, look for button/keypad-like objects
                    detections = self.yolo.detect(frame) if self.yolo.ready else []
                    # Filter for button-like classes
                    button_classes = {"button", "keyboard", "remote", "cell phone", "keypad", "remote control"}
                    keypad_dets = [d for d in detections if d.get("class_name", "").lower() in button_classes]
                    result_meta = {
                        "msg_type": "snapshot_result",
                        "timestamp": time.time(),
                        "result_type": "keypad_detect",
                        "detections": keypad_dets,
                        "camera_id": "arm",
                    }
                    jpeg_bytes, _ = capture_hazmat_still(frame, keypad_dets)
                    try:
                        parts = [json.dumps(result_meta).encode()]
                        if jpeg_bytes:
                            parts.append(jpeg_bytes)
                        self.snapshot_result_socket.send_multipart(parts, zmq.NOBLOCK)
                    except (zmq.Again, zmq.ZMQError):
                        pass
                elif req_type == "pick_detect":
                    detections = self.yolo.detect(frame) if self.yolo.ready else []
                    ignored_classes = {"person", "face", "human"}
                    pick_dets = [d for d in detections if d.get("class_name", "").lower() not in ignored_classes]
                    result_meta = {
                        "msg_type": "snapshot_result",
                        "timestamp": time.time(),
                        "result_type": "pick_detect",
                        "detections": pick_dets,
                        "camera_id": "arm",
                    }
                    jpeg_bytes, _ = capture_hazmat_still(frame, pick_dets if pick_dets else detections)
                    try:
                        parts = [json.dumps(result_meta).encode()]
                        if jpeg_bytes:
                            parts.append(jpeg_bytes)
                        self.snapshot_result_socket.send_multipart(parts, zmq.NOBLOCK)
                    except (zmq.Again, zmq.ZMQError):
                        pass

                elif req_type in ("hazmat", "snapshot"):
                    detections = self._latest_arm_detections or []
                    hazmat_dets = [d for d in detections if d.get("source") == "hazmat" or "hazmat" in d.get("class_name", "").lower()]
                    jpeg_bytes, identity = capture_hazmat_still(frame, hazmat_dets if hazmat_dets else detections)
                    result_meta = {
                        "msg_type": "snapshot_result",
                        "timestamp": time.time(),
                        "result_type": "hazmat",
                        "identity": identity,
                        "camera_id": "arm",
                    }
                    try:
                        parts = [json.dumps(result_meta).encode()]
                        if jpeg_bytes:
                            parts.append(jpeg_bytes)
                        self.snapshot_result_socket.send_multipart(parts, zmq.NOBLOCK)
                    except (zmq.Again, zmq.ZMQError):
                        pass

                elif req_type == "landolt":
                    readings = self._latest_arm_landolt or []
                    direction = readings[0].direction_name if readings else "unknown"
                    jpeg_bytes = capture_landolt_still(frame, direction)
                    result_meta = {
                        "msg_type": "snapshot_result",
                        "timestamp": time.time(),
                        "result_type": "landolt",
                        "direction": direction,
                        "camera_id": "arm",
                    }
                    try:
                        parts = [json.dumps(result_meta).encode()]
                        if jpeg_bytes:
                            parts.append(jpeg_bytes)
                        self.snapshot_result_socket.send_multipart(parts, zmq.NOBLOCK)
                    except (zmq.Again, zmq.ZMQError):
                        pass

            except zmq.Again:
                pass
            except Exception as e:
                print(f"[VISION] Snapshot request error: {e}")
            time.sleep(0.05)

    def stop(self):
        self.running = False
        for r in self.cameras.values():
            r.stop()
        if self.main_loop:
            self.main_loop.quit()
        print("[VISION] Stopped")


if __name__ == "__main__":
    node = VisionNode()
    try:
        node.start()
    except KeyboardInterrupt:
        node.stop()