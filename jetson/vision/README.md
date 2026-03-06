# RoboCupRescue RMRC 2026 - Vision System

Vision pipeline optimized for Jetson Orin Nano Super with 2 AI cameras + 2 Pi cameras.

## Modules

### LandoltClassifier (`landolt_classifier.py`)
- **8-direction** Landolt-C gap classification: Up, Up-Right, Right, Down-Right, Down, Down-Left, Left, Up-Left
- Input: frame + bounding box from YOLO Landolt detection
- Fallback: rule-based orientation when no CNN model is loaded
- Optional: ONNX/PyTorch direction model at `models/engines/landolt_direction.onnx`

### YOLO11Processor (`yolo11_processor.py`)
- Multi-model: **general**, **hazmat**, **landolt**
- Auto-discovers `.engine` (TensorRT) and `.pt` models
- Prefers YOLO11, falls back to YOLOv8
- Memory-efficient: batch_size=1, FP16

### CameraManager (`camera_manager.py`)
- **2+2 camera** coordination: Jetson (AI) + Pi (streaming)
- Default: Jetson = arm, left (30 FPS, AI); Pi = front, right (15 FPS, no AI)
- Provides camera IDs, device mapping, AI flags

### VisionNode (`vision_node.py`)
- Unified pipeline: YOLO11 + Landolt direction + QR
- Uses `GstCameraReader` from `deepstream_yolo_node.py`
- ZMQ control: `task`, `enable_yolo`, `enable_qr`, `enable_landolt`
- **GPU-offload mode** (`--gpu-only`, default): QR on laptop, Landolt GPU model only (no rule-based), JPEG via nvjpegenc — frees Jetson CPU for SLAM/Autonomy

## Usage

### Standalone vision node (replaces deepstream_yolo_node for full vision)
```bash
cd jetson
python3 vision/vision_node.py --yolo-cameras arm left --task general
python3 vision/vision_node.py --task hazmat   # HAZMAT detection
python3 vision/vision_node.py --task landolt  # Landolt + direction
```

### As library
```python
from jetson.vision import LandoltClassifier, YOLO11Processor, CameraManager

# Camera layout
cm = CameraManager()
print(cm.get_ai_camera_ids())  # ['arm', 'left']

# YOLO
yolo = YOLO11Processor()
yolo.switch_task("hazmat")
dets = yolo.detect(frame)

# Landolt direction
lc = LandoltClassifier()
readings = lc.predict_from_detections(frame, detections)
```

## File structure
```
jetson/vision/
├── __init__.py
├── landolt_classifier.py
├── yolo11_processor.py
├── camera_manager.py
├── vision_node.py
└── README.md
```

## Model paths
Place models in `jetson/models/engines/`:
- `yolo11n.engine` or `yolov8n.engine` (general)
- `hazmat_yolo11n.engine` (HAZMAT)
- `landolt_yolo11n.engine` (Landolt rings)
- `landolt_direction.onnx` (optional, 8-direction classifier)
