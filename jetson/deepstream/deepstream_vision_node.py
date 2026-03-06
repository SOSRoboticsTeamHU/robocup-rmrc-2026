#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Full DeepStream Vision Node
======================================================
Uses full DeepStream SDK: nvstreammux → nvinfer (TensorRT) → nvdsosd → nvstreamdemux
→ per-camera JPEG → ZMQ. Same protocol as vision_node (multipart [header_json, jpeg])
for maximum camera performance with TensorRT in-pipeline.

Requires: DeepStream 6.x, pyds, GStreamer 1.0, repo TensorRT engines under jetson/models/engines/
"""

import sys
import os
import json
import time
import threading
import tempfile
from pathlib import Path
from collections import deque

SCRIPT_DIR = Path(__file__).resolve().parent
JETSON_DIR = SCRIPT_DIR.parent
REPO_ROOT = JETSON_DIR.parent
for p in [str(REPO_ROOT), str(JETSON_DIR)]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from shared.constants import (
        ZMQ_PORT_CAMERA, ZMQ_PORT_CAMERA_CONTROL,
        CAMERAS, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
    )
except ImportError:
    ZMQ_PORT_CAMERA = 5557
    ZMQ_PORT_CAMERA_CONTROL = 5561
    CAMERAS = {"front": "/dev/video0", "arm": "/dev/video2", "backward": "/dev/video4"}
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    CAMERA_FPS = 30

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

Gst.init(None)

try:
    import pyds
except ImportError:
    pyds = None

try:
    import zmq
except ImportError:
    zmq = None


# Camera order for source_id → camera_id (must match pipeline source order)
CAMERA_IDS = ["front", "arm", "backward"]


def _repo_infer_config(
    template_path: Path,
    model_engine_path: Path,
    labelfile_path: Path,
    out_dir: Path,
    onnx_path: Path = None,
) -> Path:
    """Generate infer config with repo model/label paths; return path to temp config."""
    lines = template_path.read_text().splitlines()
    out_lines = []
    for line in lines:
        if line.strip().startswith("model-engine-file="):
            out_lines.append(f"model-engine-file={model_engine_path}")
        elif line.strip().startswith("labelfile-path="):
            out_lines.append(f"labelfile-path={labelfile_path}")
        elif line.strip().startswith("onnx-file=") and onnx_path is not None:
            out_lines.append(f"onnx-file={onnx_path}")
        else:
            out_lines.append(line)
    out_path = out_dir / template_path.name
    out_path.write_text("\n".join(out_lines))
    return out_path


def _load_labels(path: Path) -> list:
    """Load label file (one class per line)."""
    if not path.exists():
        return []
    return [line.strip() for line in path.read_text().splitlines() if line.strip()]


class DeepStreamVisionNode:
    """
    Full DeepStream pipeline: mux → nvinfer (HAZMAT + Landolt) → OSD → demux → JPEG per camera.
    Sends same ZMQ multipart [header_json, jpeg] as vision_node for laptop compatibility.
    """

    def __init__(
        self,
        num_cameras: int = 2,
        port: int = ZMQ_PORT_CAMERA,
        control_port: int = ZMQ_PORT_CAMERA_CONTROL,
        config_dir: Path = None,
        models_dir: Path = None,
        enable_inference: bool = True,
    ):
        self.num_cameras = min(num_cameras, 4)
        self.port = port
        self.control_port = control_port
        self.config_dir = config_dir or (SCRIPT_DIR / "configs")
        self.models_dir = models_dir or (JETSON_DIR / "models" / "engines")
        self.enable_inference = enable_inference
        self.pipeline = None
        self.loop = None
        self._temp_config_dir = Path(tempfile.mkdtemp(prefix="ds_vision_"))
        self._hazmat_labels = _load_labels(self.config_dir / "hazmat_labels.txt")
        self._landolt_labels = _load_labels(self.config_dir / "landolt_labels.txt")
        # Per-source detection queues (probe pushes, appsink pops)
        self._detection_queues = [deque(maxlen=4) for _ in range(4)]
        self._queue_lock = threading.Lock()
        self._frame_counters = [0] * 4
        self.running = False
        self._enable_yolo = True
        self._control_lock = threading.Lock()
        self.context = None
        self.socket = None
        self.control_socket = None
        self.control_thread = None

    def _get_infer_config_paths(self):
        """
        Return (hazmat_config_path or None, landolt_config_path or None).

        NOTE: Landolt (direction) is temporarily DISABLED in DeepStream because the existing
        landolt_yolo11n.engine on your Jetson was built with an incompatible TensorRT version
        (IRuntime::deserializeCudaEngine error). We run HAZMAT only in DeepStream and keep
        Landolt direction via the classic vision node / LandoltClassifier path.
        """
        # Prefer YOLO11 engine names (same as vision_node), fallback to hazmat_best
        hazmat_candidates = ["hazmat_yolo11n.engine", "hazmat_best.engine"]
        hazmat_engine = None
        for name in hazmat_candidates:
            p = self.models_dir / name
            if p.exists():
                hazmat_engine = p
                break

        hazmat_labels = self.config_dir / "hazmat_labels.txt"
        # Find ONNX source so DeepStream can rebuild the engine if the cached .engine is stale
        hazmat_onnx = None
        for name in ["hazmat_yolo11n.onnx", "best.onnx"]:
            p = self.models_dir / name
            if p.exists():
                hazmat_onnx = p
                break

        hazmat_config = None
        if hazmat_engine is not None:
            hazmat_config = str(_repo_infer_config(
                self.config_dir / "hazmat_infer_config.txt",
                hazmat_engine,
                hazmat_labels,
                self._temp_config_dir,
                onnx_path=hazmat_onnx,
            ))
        elif hazmat_onnx is not None:
            # No cached engine but ONNX available; DeepStream will build the engine
            hazmat_config = str(_repo_infer_config(
                self.config_dir / "hazmat_infer_config.txt",
                self.models_dir / "hazmat_yolo11n.engine",
                hazmat_labels,
                self._temp_config_dir,
                onnx_path=hazmat_onnx,
            ))
        else:
            print(f"[DEEPSTREAM] HAZMAT engine not found (tried {hazmat_candidates}); skipping HAZMAT inference.")

        # Landolt disabled for now to avoid engine deserialization failures; return None
        landolt_config = None
        if hazmat_config:
            print("[DEEPSTREAM] Landolt (direction) disabled in DeepStream; running HAZMAT only.")
        else:
            print(f"[DEEPSTREAM] No TensorRT engines in {self.models_dir}; running video only.")

        return hazmat_config, landolt_config

    def create_pipeline(self):
        """Build DeepStream pipeline: sources → mux → nvinfer x2 → osd → demux → per-cam JPEG → appsink."""
        pipeline = Gst.Pipeline.new("deepstream-vision")
        batch_size = self.num_cameras
        camera_ids = CAMERA_IDS[: self.num_cameras]
        sources = []

        for i, cam_id in enumerate(camera_ids):
            dev = CAMERAS.get(cam_id, f"/dev/video{i*2}")
            # MJPEG decode path (same as vision_node): USB cams often only do MJPEG at 640x480.
            # v4l2src ! image/jpeg ! jpegparse ! nvjpegdec ! nvvidconv ! queue -> mux
            caps_str = f"image/jpeg,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},framerate={CAMERA_FPS}/1"
            src = Gst.ElementFactory.make("v4l2src", f"src_{cam_id}")
            src.set_property("device", dev)
            capsf = Gst.ElementFactory.make("capsfilter", f"caps_{cam_id}")
            capsf.set_property("caps", Gst.Caps.from_string(caps_str))
            jpegparse = Gst.ElementFactory.make("jpegparse", f"jpegparse_{cam_id}")
            nvjpegdec = Gst.ElementFactory.make("nvjpegdec", f"nvjpegdec_{cam_id}")
            nvvidconv = Gst.ElementFactory.make("nvvidconv", f"conv_{cam_id}")
            queue = Gst.ElementFactory.make("queue", f"queue_{cam_id}")
            for el in (src, capsf, jpegparse, nvjpegdec, nvvidconv, queue):
                pipeline.add(el)
            src.link(capsf)
            capsf.link(jpegparse)
            jpegparse.link(nvjpegdec)
            nvjpegdec.link(nvvidconv)
            nvvidconv.link(queue)
            sources.append(queue)

        streammux = Gst.ElementFactory.make("nvstreammux", "mux")
        # Match mux resolution to per-camera caps (CAMERA_WIDTH x CAMERA_HEIGHT, typically 640x480)
        streammux.set_property("width", CAMERA_WIDTH)
        streammux.set_property("height", CAMERA_HEIGHT)
        streammux.set_property("batch-size", batch_size)
        streammux.set_property("batched-push-timeout", 40000)
        pipeline.add(streammux)
        for i, q in enumerate(sources):
            sinkpad = streammux.get_request_pad(f"sink_{i}")
            q.get_static_pad("src").link(sinkpad)

        last = streammux
        if self.enable_inference:
            try:
                hazmat_cfg, landolt_cfg = self._get_infer_config_paths()
            except Exception as e:
                print(f"[DEEPSTREAM] Infer config generation failed: {e}, running without inference")
                hazmat_cfg, landolt_cfg = None, None
            if hazmat_cfg:
                pgie = Gst.ElementFactory.make("nvinfer", "pgie_hazmat")
                pgie.set_property("config-file-path", hazmat_cfg)
                pipeline.add(pgie)
                streammux.link(pgie)
                last = pgie
                if landolt_cfg:
                    sgie = Gst.ElementFactory.make("nvinfer", "sgie_landolt")
                    sgie.set_property("config-file-path", landolt_cfg)
                    pipeline.add(sgie)
                    pgie.link(sgie)
                    last = sgie
            else:
                self.enable_inference = False

        nvosd = Gst.ElementFactory.make("nvdsosd", "osd")
        pipeline.add(nvosd)
        last.link(nvosd)

        # Probe on OSD sink: extract detections per source and push to queues
        if pyds and self.enable_inference:
            nvosd.get_static_pad("sink").add_probe(
                Gst.PadProbeType.BUFFER,
                self._osd_buffer_probe,
                0,
            )

        nvdemux = Gst.ElementFactory.make("nvstreamdemux", "demux")
        pipeline.add(nvdemux)
        nvosd.link(nvdemux)

        for i in range(batch_size):
            pad_name = f"src_{i}"
            demux_src = nvdemux.get_request_pad(pad_name)
            q = Gst.ElementFactory.make("queue", f"demux_q_{i}")
            nvconv = Gst.ElementFactory.make("nvvidconv", f"jpeg_conv_{i}")
            jpegenc = Gst.ElementFactory.make("nvjpegenc", f"jpeg_enc_{i}")
            appsink = Gst.ElementFactory.make("appsink", f"appsink_{i}")
            appsink.set_property("emit-signals", True)
            appsink.set_property("sync", False)
            appsink.set_property("max-buffers", 2)
            appsink.connect("new-sample", self._on_new_sample, i)
            for el in (q, nvconv, jpegenc, appsink):
                pipeline.add(el)
            demux_src.link(q.get_static_pad("sink"))
            q.link(nvconv)
            nvconv.link(jpegenc)
            jpegenc.link(appsink)

        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)
        self.pipeline = pipeline
        return pipeline

    def _osd_buffer_probe(self, pad, info, u_data):
        """Extract per-frame detections and push to per-source queues."""
        if not pyds:
            return Gst.PadProbeReturn.OK
        gst_buffer = info.get_buffer()
        if not gst_buffer:
            return Gst.PadProbeReturn.OK
        batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
        if not batch_meta:
            return Gst.PadProbeReturn.OK
        l_frame = batch_meta.frame_meta_list
        per_source = [[] for _ in range(4)]
        while l_frame is not None:
            try:
                frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
            except StopIteration:
                break
            src_id = frame_meta.source_id
            if src_id >= 4:
                try:
                    l_frame = l_frame.next
                except StopIteration:
                    break
                continue
            l_obj = frame_meta.obj_meta_list
            while l_obj is not None:
                try:
                    obj_meta = pyds.NvDsObjectMeta.cast(l_obj.data)
                except StopIteration:
                    break
                gie_id = getattr(obj_meta, "gie_unique_id", 1)
                labels = self._landolt_labels if gie_id == 2 else self._hazmat_labels
                class_id = obj_meta.class_id
                label = obj_meta.obj_label if obj_meta.obj_label else (labels[class_id] if class_id < len(labels) else str(class_id))
                det = {
                    "class_id": class_id,
                    "class_name": label,
                    "confidence": float(obj_meta.confidence),
                    "bbox": {
                        "left": obj_meta.rect_params.left,
                        "top": obj_meta.rect_params.top,
                        "width": obj_meta.rect_params.width,
                        "height": obj_meta.rect_params.height,
                    },
                    "source": "landolt" if gie_id == 2 else "hazmat",
                }
                per_source[src_id].append(det)
                try:
                    l_obj = l_obj.next
                except StopIteration:
                    break
            try:
                l_frame = l_frame.next
            except StopIteration:
                break
        with self._queue_lock:
            for i, dets in enumerate(per_source):
                if i < self.num_cameras:
                    self._detection_queues[i].append(dets)
        return Gst.PadProbeReturn.OK

    def _on_new_sample(self, appsink, source_index):
        """Appsink callback: get JPEG bytes, pop detections, send multipart ZMQ."""
        sample = appsink.emit("pull-sample")
        if not sample or not self.socket:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        if not buf:
            return Gst.FlowReturn.OK
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        jpeg_bytes = bytes(map_info.data)
        buf.unmap(map_info)
        with self._queue_lock:
            q = self._detection_queues[source_index]
            detections = q.popleft() if len(q) > 0 else []
        self._frame_counters[source_index] += 1
        camera_id = CAMERA_IDS[source_index] if source_index < len(CAMERA_IDS) else f"cam_{source_index}"
        with self._control_lock:
            enable_yolo = self._enable_yolo
        if not enable_yolo:
            detections = []
        landolt_readings = []
        for d in detections:
            if d.get("source") == "landolt" and d.get("class_name"):
                landolt_readings.append({"direction": d["class_name"], "confidence": d["confidence"]})
        header = {
            "msg_type": "camera_frame",
            "camera_id": camera_id,
            "timestamp": time.time(),
            "frame_num": self._frame_counters[source_index],
            "width": CAMERA_WIDTH,
            "height": CAMERA_HEIGHT,
            "fps": CAMERA_FPS,
            "task": "hazmat",
            "detections": detections,
            "qr_codes": [],
            "landolt_readings": landolt_readings,
        }
        try:
            self.socket.send_multipart([json.dumps(header).encode("utf-8"), jpeg_bytes], zmq.NOBLOCK)
        except zmq.Again:
            pass
        return Gst.FlowReturn.OK

    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            if self.loop:
                self.loop.quit()
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"[DEEPSTREAM] Error: {err}", debug)
            if self.loop:
                self.loop.quit()
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_s, new_s, _ = message.parse_state_changed()
                print(f"[DEEPSTREAM] Pipeline {old_s.value_nick} → {new_s.value_nick}")

    def _control_loop(self):
        while self.running and self.control_socket:
            try:
                msg = self.control_socket.recv_string()
                data = json.loads(msg)
                with self._control_lock:
                    if "enable_yolo" in data:
                        self._enable_yolo = bool(data["enable_yolo"])
            except (zmq.Again, json.JSONDecodeError):
                pass
            except Exception:
                pass

    def start(self):
        if not zmq:
            print("[DEEPSTREAM] zmq not available")
            return
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 60)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f"tcp://*:{self.port}")
        self.control_socket = self.context.socket(zmq.SUB)
        self.control_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.control_socket.setsockopt(zmq.RCVTIMEO, 500)
        self.control_socket.bind(f"tcp://*:{self.control_port}")
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        self.create_pipeline()
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("[DEEPSTREAM] Failed to set pipeline PLAYING")
            self.running = False
            return
        print(f"[DEEPSTREAM] ZMQ tcp://*:{self.port} control {self.control_port} | cameras {self.num_cameras} | TensorRT in-pipeline")
        self.loop = GLib.MainLoop()
        try:
            self.loop.run()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if self.control_socket:
            self.control_socket.close()
        if self.socket:
            self.socket.close()
        if self.context:
            self.context.term()
        try:
            import shutil
            shutil.rmtree(self._temp_config_dir, ignore_errors=True)
        except Exception:
            pass
        print("[DEEPSTREAM] Stopped")


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Full DeepStream vision node (TensorRT in-pipeline)")
    ap.add_argument("--port", type=int, default=ZMQ_PORT_CAMERA)
    ap.add_argument("--control-port", type=int, default=ZMQ_PORT_CAMERA_CONTROL)
    ap.add_argument("--num-cameras", type=int, default=2, help="2 or 4")
    ap.add_argument("--no-inference", action="store_true", help="Disable nvinfer (stream only)")
    ap.add_argument("--config-dir", type=str, default=None)
    ap.add_argument("--models-dir", type=str, default=None)
    args = ap.parse_args()
    config_dir = Path(args.config_dir) if args.config_dir else None
    models_dir = Path(args.models_dir) if args.models_dir else None
    node = DeepStreamVisionNode(
        num_cameras=args.num_cameras,
        port=args.port,
        control_port=args.control_port,
        config_dir=config_dir,
        models_dir=models_dir,
        enable_inference=not args.no_inference,
    )
    try:
        node.start()
    except KeyboardInterrupt:
        print("\n[DEEPSTREAM] Interrupt")
    finally:
        node.stop()


if __name__ == "__main__":
    main()
