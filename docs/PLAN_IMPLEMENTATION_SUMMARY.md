# Plan implementation summary

What was implemented from the consolidated RMRC plan (SLAM, autonomy, mission timer, stowed posture, Sensor Cabinet).

---

## 1. Mission timer (300 s preliminaries)

- **shared/constants.py**: `MISSION_DURATION = 300` (5 min), `MISSION_DURATION_FINALS = 600` (10 min).
- GUI uses `MISSION_DURATION` for the mission countdown; no code change needed in the GUI.

---

## 2. SLAM visualization on laptop (Jetson RViz off by default)

- **jetson/start_robot.sh**:
  - `USE_RVIZ_ON_JETSON` (default `0`): when `0`, the Jetson does **not** start the RViz 3D stream (saves CPU/GPU).
  - To enable RViz on the Jetson: `USE_RVIZ_ON_JETSON=1 ./start_robot.sh`.
  - Startup message states that SLAM is viewed on the laptop (2D map or 3D RViz stream from laptop).

---

## 3. 2D SLAM view more responsive

- **laptop/gui/slam_view.py**: ZMQ poll timeout reduced from 25 ms to 10 ms so SLAM updates are processed sooner.

---

## 4. Stowed posture

- **jetson/config/robot_params.yaml**: New `stowed_posture` section (description and optional `arm_joints` for a future automatic check).
- **shared/constants.py**: `STOWED_POSTURE_REQUIRED_FOR_AUTONOMY = True`.
- **laptop/gui/main_window.py**: "Stowed" checkbox in the control panel; operator confirms stowed before using autonomy (for 4x bonus).

---

## 5. Sensor Cabinet autonomy + GUI

- **jetson/autonomy_node.py**:
  - New state `SENSOR_CABINET` and mode `sensor_cabinet`.
  - `handle_command` accepts `"sensor_cabinet"` and calls `_start_sensor_cabinet()`.
  - Message: "Sensor Cabinet: position camera for HAZMAT/Landolt (arm preset or manual)."
- **laptop/gui/main_window.py**:
  - ZMQ PUSH socket to Jetson on port 5560 (autonomy commands).
  - Control panel: **Stowed** checkbox, **Mapping** button, **Sensor Cab** button.
  - **Mapping** sends `{"mode": "mapping"}`, **Sensor Cab** sends `{"mode": "sensor_cabinet"}`.
  - **Stop** mission also sends `{"mode": "disabled"}` to the autonomy node.

---

## Not implemented (from plan)

- **SLAM latency**: Not fixed (needs reproduction and profiling).
- **QR-on-map validation**: 30 cm accuracy vs fiducials not implemented.
- **Thermal sensor / magnetometer**: Out of scope.

---

## Full DeepStream SDK (TensorRT in-pipeline)

- **jetson/deepstream/deepstream_vision_node.py**: Full DeepStream pipeline (nvstreammux → nvinfer HAZMAT → nvinfer Landolt → nvdsosd → nvstreamdemux → per-camera JPEG → appsink). Sends the **same ZMQ protocol** as vision_node: multipart `[header_json, jpeg]` on port 5557, so the laptop GUI works unchanged. Control port 5561 (enable/disable YOLO). Uses repo model paths: `jetson/models/engines/*.engine` and `jetson/deepstream/configs/*.txt` (generates temp configs with correct paths).
- **jetson/start_robot.sh**:
  - `USE_DEEPSTREAM_SDK=1`: start **DeepStream vision node** instead of vision_node (max camera performance with TensorRT in-pipeline).
  - Default remains `USE_DEEPSTREAM_SDK=0` (vision_node). Run: `USE_DEEPSTREAM_SDK=1 ./start_robot.sh` to use full DeepStream.

The default camera path (vision_node + GstCameraReader) still uses **GStreamer + Python/Ultralytics YOLO** (TensorRT engines loaded in Python). For full SDK performance, set `USE_DEEPSTREAM_SDK=1` and ensure DeepStream 6.x and TensorRT engines are available under `jetson/models/engines/`.
