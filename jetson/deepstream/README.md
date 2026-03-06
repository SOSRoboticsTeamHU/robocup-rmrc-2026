# DeepStream Camera Streaming Pipeline

## 🎯 Két Fázis

### Stage 1: Alapvető Streaming (MOST)
```
4× USB Camera → GStreamer → Encode → Stream
❌ Nincs AI inference
✅ Kamera validáció
✅ Streaming tesztelés
```

### Stage 2: TensorRT Inference (KÉSŐBB)
```
4× USB Camera → TensorRT (YOLO) → Bounding Box → Stream
✅ HAZMAT detekció
✅ Landolt-C detekció
✅ ZMQ streaming
```

---

## Full DeepStream vision node (same protocol as vision_node)

**deepstream_vision_node.py** uses the full DeepStream SDK (nvstreammux → nvinfer → nvdsosd → nvstreamdemux → per-camera JPEG) and publishes the **same ZMQ protocol** as `vision_node`: multipart `[header_json, jpeg]` on port 5557, so the laptop GUI works without changes. Use this for maximum camera performance with TensorRT in-pipeline.

- **Run standalone**: `python3 deepstream/deepstream_vision_node.py --num-cameras 2`
- **Run via start_robot**: `USE_DEEPSTREAM_SDK=1 ./start_robot.sh`
- Models and configs are taken from the repo: `jetson/models/engines/*.engine`, `jetson/deepstream/configs/*.txt` (temp configs are generated with correct paths).

---

## 📦 Struktúra

```
jetson/deepstream/
├── camera_stream_basic.py        # Stage 1: Streaming only
├── camera_stream_inference.py    # Stage 2: With TensorRT (ZMQ JSON only)
├── deepstream_vision_node.py     # Full SDK, same ZMQ as vision_node (video + detections)
├── configs/
│   ├── hazmat_infer_config.txt   # HAZMAT TensorRT config
│   ├── landolt_infer_config.txt  # Landolt-C TensorRT config
│   ├── hazmat_labels.txt         # 13 osztály
│   └── landolt_labels.txt        # 2 osztály
└── README.md
```

---

## 🚀 Stage 1: Alapvető Streaming

### Követelmények
```bash
# Jetson-on
sudo apt install python3-gst-1.0 python3-gi
pip3 install pyzmq
```

### Futtatás
```bash
cd /opt/robocup/deepstream
python3 camera_stream_basic.py
```

### Ellenőrzés
- 4 kamera látszik: `/dev/video0-3`
- Pipeline indul
- Nincs hiba

---

## 🧠 Stage 2: TensorRT Inference

### Előfeltételek

#### 1. DeepStream SDK Telepítés
```bash
# JetPack-kel együtt jön, vagy:
sudo apt install deepstream-6.2
```

#### 2. Trained Models (Colab-ról)
```bash
# Mac/Windows-ról Jetson-ra
scp hazmat_best.pt jetson@jetson-ip:/opt/robocup/models/
scp landolt_best.pt jetson@jetson-ip:/opt/robocup/models/
```

#### 3. TensorRT Export (Jetson-on)
```bash
# Jetson-on
cd /opt/robocup/models

# HAZMAT export
yolo export model=hazmat_best.pt format=engine device=0 imgsz=640

# Landolt-C export
yolo export model=landolt_best.pt format=engine device=0 imgsz=640
```

Ez létrehozza:
- `hazmat_best.engine` (~6MB)
- `landolt_best.engine` (~6MB)

#### 4. Config Files
Másold át a config fájlokat:
```bash
scp -r configs jetson@jetson-ip:/opt/robocup/
```

### Futtatás

#### Csak Streaming (nincs inference)
```bash
python3 camera_stream_inference.py
```

#### Inference-szel
```bash
python3 camera_stream_inference.py --inference
```

---

## 🔧 Kamera Konfiguráció

### USB Kamerák Ellenőrzése
```bash
# List cameras
v4l2-ctl --list-devices

# Check camera capabilities
v4l2-ctl -d /dev/video0 --list-formats-ext
```

### Kamera Kiosztás
```
/dev/video0 → Front camera (1920x1080)
/dev/video1 → Arm camera (1920x1080)
/dev/video2 → Left side (640x480)
/dev/video3 → Right side (640x480)
```

---

## 📡 ZMQ Streaming

### Server (Jetson)
```python
# camera_stream_inference.py
zmq_socket.bind("tcp://*:5557")
```

### Client (Laptop - Python)
```python
import zmq

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://JETSON_IP:5557")
socket.subscribe(b"")

while True:
    msg = socket.recv_json()
    print(f"Detections: {msg['detections']}")
```

### Client (Laptop - ROS)
```python
# laptop/src/vision_stream/vision_subscriber.py
import rospy
import zmq
from vision_msgs.msg import Detection2DArray

# ZMQ → ROS bridge
```

---

## 🎯 DeepStream Pipeline Diagram

```
┌─────────────────────────────────────────────────────────┐
│                    JETSON ORIN NANO                     │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  /dev/video0 (Front)  ─┐                               │
│  /dev/video1 (Arm)    ─┤                               │
│  /dev/video2 (Left)   ─┼→ nvstreammux (batch=4)        │
│  /dev/video3 (Right)  ─┘                               │
│                          │                              │
│                          ↓                              │
│                   ┌──────────────┐                      │
│                   │   nvinfer    │ (HAZMAT TensorRT)    │
│                   │  GPU Accel   │                      │
│                   └──────┬───────┘                      │
│                          │                              │
│                          ↓                              │
│                   ┌──────────────┐                      │
│                   │   nvinfer    │ (Landolt TensorRT)   │
│                   │  GPU Accel   │                      │
│                   └──────┬───────┘                      │
│                          │                              │
│                          ↓                              │
│                   ┌──────────────┐                      │
│                   │   nvdsosd    │ (Bounding Boxes)     │
│                   └──────┬───────┘                      │
│                          │                              │
│                   ┌──────┴────────┐                     │
│                   │               │                     │
│                   ↓               ↓                     │
│            H264 Encode      ZMQ Publisher               │
│                   │          (Metadata)                 │
│                   ↓               │                     │
│             RTSP Stream           │                     │
│                              tcp://*:5557               │
└───────────────────────────────────┼─────────────────────┘
                                    │
                                    ↓
                             ┌──────────────┐
                             │   LAPTOP     │
                             │ ZMQ Subscribe│
                             │  ROS Bridge  │
                             └──────────────┘
```

---

## 🐛 Troubleshooting

### Problem: "Failed to create nvinfer"
```bash
# Check TensorRT engine exists
ls -lh /opt/robocup/models/*.engine

# Re-export if needed
yolo export model=best.pt format=engine device=0
```

### Problem: "No cameras found"
```bash
# List USB devices
lsusb

# Check video devices
ls -l /dev/video*

# Permissions
sudo chmod 666 /dev/video*
```

### Problem: "Out of memory"
```bash
# Check GPU memory
tegrastats

# Reduce batch size in configs
# Edit configs/*_infer_config.txt
# batch-size=2  (instead of 4)
```

---

## 📊 Performance

### Expected FPS

| Config | Resolution | FPS | Latency |
|--------|------------|-----|---------|
| Streaming only | 1080p×2 + 480p×2 | 30 | ~30ms |
| + HAZMAT inference | Same | 20-25 | ~50ms |
| + Landolt inference | Same | 15-20 | ~70ms |

### Optimization
- Reduce resolution: 640×480 for all cameras
- Batch processing: batch-size=4
- TensorRT FP16: Enabled by default

---

## ✅ Checklist

### Stage 1 (Most)
- [ ] DeepStream SDK telepítve
- [ ] 4× USB kamera csatlakoztatva
- [ ] `camera_stream_basic.py` fut
- [ ] Stream látszik

### Stage 2 (Később)
- [ ] Trained models letöltve (Colab)
- [ ] TensorRT export (.engine fájlok)
- [ ] Config fájlok helyén
- [ ] `--inference` mód fut
- [ ] ZMQ detekciók érkeznek

---

## 🚀 Következő Lépések

1. **Stage 1**: Teszteld az alapvető streamet
2. **Modell tréning**: Várj a Colab-ra
3. **Model transfer**: Jetson-ra másolás
4. **TensorRT export**: `.pt` → `.engine`
5. **Stage 2**: Inference engedélyezés
6. **Laptop integráció**: ZMQ → ROS bridge
