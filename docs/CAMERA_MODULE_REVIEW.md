# RoboCup Rescue 2026 – Camera Module Review

## Overview

The robot's camera system provides multi-camera streaming with optional YOLO object detection and QR code reading. Streams are sent via ZMQ to the laptop GUI for display and to the SLAM bridge for mapping. This document summarizes the current architecture and recent GPU optimizations.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         JETSON (Primary Path)                            │
│  ┌─────────────┐    ┌──────────────────────────────────────────────┐    │
│  │ USB Cameras │───►│ GStreamer (v4l2src → nvv4l2decoder → nvvidconv)│   │
│  │ /dev/video* │    │ GPU decode + scale (1280×720 → 640×480)       │    │
│  └─────────────┘    └───────────────────┬──────────────────────────┘    │
│                                         │                                │
│         ┌───────────────────────────────┼───────────────────────────────┤
│         │ YOLO cameras (front, arm)     │ Non-YOLO cameras (left, right) │
│         ▼                               ▼                                │
│  ┌──────────────────────┐      ┌────────────────────┐                   │
│  │ tee → BGR appsink    │      │ nvjpegenc          │                   │
│  │      (YOLO + QR)     │      │ (JPEG only, no     │                   │
│  │ tee → nvjpegenc      │      │  GPU→CPU copy)     │                   │
│  └──────────────────────┘      └────────────────────┘                   │
│         │                               │                                │
│         └───────────────────────────────┼───────────────────────────────┤
│                                         ▼                                │
│                              ZMQ PUB (port 5557)                         │
└─────────────────────────────────────────┼───────────────────────────────┘
                                          │
┌─────────────────────────────────────────┼───────────────────────────────┐
│ LAPTOP                                  ▼                               │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │ camera_receiver.py (ZMQ SUB) → main_window.py (4-camera grid)    │   │
│  │ - cv2.imdecode (CPU)                                            │   │
│  │ - YOLO/QR overlays drawn on display                             │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘

Optional: PI path (pi/camera_stream.py) – OpenCV + CPU JPEG when cameras on Pi
```

---

## GPU Acceleration (2026 Update)

### Changes Made

1. **Hardware JPEG encoding (nvjpegenc)**  
   Replaced `cv2.imencode()` with GStreamer `nvjpegenc` for JPEG compression on the GPU. This reduces CPU load on the Jetson and frees CPU cycles for SLAM and other processes.

2. **Per-camera pipeline configuration**  
   - **YOLO cameras (front, arm):** `tee` → BGR appsink (for YOLO/QR) + `nvjpegenc` → JPEG. BGR is still brought to CPU for Ultralytics inference.  
   - **Non-YOLO cameras (left, right):** JPEG-only pipeline (no BGR branch). Frames stay on GPU until JPEG output, avoiding unnecessary GPU→CPU copies.

3. **Fallback behavior**  
   If `nvjpegenc` is unavailable (e.g., older Jetson or missing plugin), the pipeline falls back to the previous CPU `cv2.imencode()` path.

### Expected Benefits

- Lower CPU usage for camera processing  
- Less contention with SLAM (slam_bridge, Point-LIO, occupancy grid)  
- Lower latency for SLAM pose updates and map generation  

### Requirements

- Jetson with GStreamer `nvcodec` plugin and `nvjpegenc`  
- NVIDIA L4T (tested on Orin Nano; other Jetsons may need pipeline adjustments)  
- If `memory:NVMM` is unsupported, the pipeline falls back to the CPU path automatically  

---

## YOLO Nodes – Current State

### 0. `gpu_camera_node.py` (Jetson root) – **Primary / Default**

| Aspect | Status |
|--------|--------|
| **Role** | Default camera node used by `start_robot.sh`. GPU streaming always; YOLO when models present |
| **Without models** | Pure GPU streaming (JPEG-only pipeline). No YOLO dependency. |
| **With models** | Copy `.engine` to `models/engines/` → YOLO auto-enables on front+arm |
| **Pipeline** | GStreamer v4l2src → nvv4l2decoder → nvvidconv; nvjpegenc for JPEG |
| **YOLO** | ModelManager (HAZMAT, Landolt, General) – same as multi_model_yolo_node |
| **Notes** | Recommended for development and competition. No code changes when adding models. |

### 1. `deepstream_yolo_node.py` (Jetson root)

| Aspect | Status |
|--------|--------|
| **Role** | Main camera node used by `start_robot.sh` when no HAZMAT TensorRT engine is present |
| **Pipeline** | GStreamer v4l2src → nvv4l2decoder → nvvidconv; GPU JPEG (nvjpegenc); BGR only for YOLO cameras |
| **YOLO** | Single TensorRT model (yolov8n.engine or .pt fallback) |
| **QR** | pyzbar |
| **Control** | ZMQ port 5561: `enable_yolo`, `enable_qr` from laptop for FPS tuning |
| **Notes** | Primary path for streaming + detection. GPU JPEG added here. |

### 2. `multi_model_yolo_node.py` (jetson/deepstream/)

| Aspect | Status |
|--------|--------|
| **Role** | Task-based model switching: HAZMAT, Landolt-C, General |
| **Pipeline** | Uses `GstCameraReader` from `deepstream_yolo_node`; same GPU JPEG logic |
| **YOLO** | `ModelManager` with hazmat_yolo11n, landolt_yolo11n, yolo11n engines |
| **QR** | pyzbar |
| **Control** | `task`, `enable_yolo`, `enable_qr` via ZMQ |
| **Notes** | Recommended when HAZMAT/Landolt models are available; multi-model and GPU JPEG support. |

### 3. ~~`tensorrt_camera_node.py`~~ (removed; use vision_node or deepstream_yolo_node)

| Aspect | Status |
|--------|--------|
| **Role** | Used by `start_robot.sh` when `hazmat_yolov8n.engine` exists |
| **Pipeline** | GStreamer via `cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)` – uses `jpegdec` (CPU decode), not nvv4l2decoder |
| **YOLO** | HAZMAT + Landolt TensorRT engines loaded at once |
| **QR** | pyzbar |
| **Control** | No runtime YOLO/QR toggle |
| **Notes** | **Not GPU-optimized:** CPU JPEG decode and `cv2.imencode()`. Consider migrating to the DeepStream pipeline and nvjpegenc for consistency. |

### 4. `yolo_node.py` (ros2_ws rescue_robot)

| Aspect | Status |
|--------|--------|
| **Role** | Standalone YOLO node subscribing to camera ZMQ and publishing detections |
| **Pipeline** | Consumes already-encoded JPEG from camera stream; does not run cameras |
| **YOLO** | Ultralytics YOLO (TensorRT if .engine available) |
| **QR** | pyzbar |
| **Notes** | Redundant with the integrated pipeline; camera + YOLO are usually colocated in DeepStream/TensorRT nodes. |

### 5. `camera_stream_test.py` (jetson/deepstream/)

| Aspect | Status |
|--------|--------|
| **Role** | Camera hardware test without YOLO |
| **Pipeline** | Uses `GstCameraReader` – benefits from GPU decode; no JPEG branch changes (stream-only) |
| **Notes** | Simple streaming; no inference. |

---

## Pi Camera Path (`pi/camera_stream.py`)

| Aspect | Status |
|--------|--------|
| **Platform** | Raspberry Pi (no Jetson GPU) |
| **Pipeline** | OpenCV `cv2.VideoCapture()` + `cv2.imencode()` – fully CPU |
| **Usage** | Used when cameras are connected to Pi and streamed to laptop/Jetson |
| **GPU** | Pi VideoCore does not support nvjpegenc; hardware JPEG for USB cameras would require Pi-specific stacks (mmal/libcamera) and is not implemented |

---

## Summary and Recommendations

### Strengths

- GPU decode (nvv4l2decoder) and scale (nvvidconv) in place  
- GPU JPEG encoding (nvjpegenc) added for reduced CPU use and SLAM contention  
- Per-camera pipelines: BGR only where YOLO is needed  
- Runtime YOLO/QR toggle for FPS vs. detection trade-off  
- Multi-model support (HAZMAT, Landolt) in `multi_model_yolo_node`  

### Gaps and Suggestions

1. **`tensorrt_camera_node.py`**  
   Migrate to the DeepStream-style pipeline (nvv4l2decoder + nvjpegenc) and unify behavior with `deepstream_yolo_node`.

2. **`yolo_node.py` (ROS2)**  
   Evaluate whether it is still needed; if not, deprecate it to avoid duplicated logic.

3. **Laptop-side decode**  
   `camera_receiver.py` uses `cv2.imdecode()` on the laptop. Hardware decode is less critical there, but could be explored for very low-end machines.

4. **`nvjpegenc` availability**  
   Document supported Jetson models and L4T versions. If `memory:NVMM` fails on some devices, add a fallback pipeline using `video/x-raw,format=NV12` before nvjpegenc.

---

## Files Reference

| File | Description |
|------|-------------|
| `jetson/gpu_camera_node.py` | **Primary** – GPU stream; YOLO when models in models/engines/ |
| `jetson/deepstream_yolo_node.py` | DeepStream node; GPU JPEG, BGR for YOLO cameras |
| `jetson/deepstream/multi_model_yolo_node.py` | Multi-model YOLO (HAZMAT/Landolt/General) |
| `jetson/deepstream/camera_stream_test.py` | Camera test without YOLO |
| `jetson/vision/vision_node.py`   | HAZMAT + Landolt (primary)     |
| `jetson/ros2_ws/.../yolo_node.py` | Standalone YOLO subscriber |
| `pi/camera_stream.py` | Pi USB camera streamer (CPU) |
| `laptop/gui/camera_receiver.py` | ZMQ subscriber for laptop GUI |
| `laptop/gui/main_window.py` | 4-camera grid, YOLO/QR overlays |
