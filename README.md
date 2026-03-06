# RoboCup Rescue RMRC 2026

**Operator station and robot brain stack for the RoboCup Rescue Rapidly Manufactured Robot Challenge (RMRC).**

A split architecture: a **laptop** runs the operator GUI (joystick, arm leader, cameras, SLAM view, mission scoring), and a **Jetson** on the robot runs drive, vision (HAZMAT + Landolt-C), SLAM bridge, arm follower, and autonomy. They communicate over Ethernet via ZMQ.

---

## Pushing to GitHub

1. **Create a new repository** on GitHub (do not initialize with README).
2. **Add remote and push:**
   ```bash
   git remote add origin https://github.com/YOUR_USERNAME/robocup_rescue_2026.git
   git branch -M main
   git push -u origin main
   ```
3. **Replace** `YOUR_USERNAME` with your GitHub username or organization.

---

## Features

| Component | Description |
|-----------|-------------|
| **Operator GUI** | PyQt5: live camera feeds (front, arm, backward), 2D SLAM map, RViz 3D stream, joystick drive, arm leader teleop, mission report & QR log |
| **Vision** | 3 USB cameras on Jetson; 2 with YOLO (HAZMAT + Landolt-C); GPU MJPEG; optional DeepStream pipeline |
| **SLAM** | Point-LIO + Unitree L2 lidar; occupancy grid and pose over ZMQ; optional Nav2 + autonomy executor |
| **Arm** | Leader (laptop, LeRobot SO-ARM) → ZMQ → Follower (Jetson, Feetech bus); back-support/kickstand servo |
| **Scoring** | Test selector, QR codes to file, snapshot review, PDF report (rulebook-aligned) |

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  LAPTOP (Operator)                                                           │
│  main.py → GUI, joystick, arm leader, camera receiver, SLAM view, reports    │
└───────────────────────────────────┬─────────────────────────────────────────┘
                                    │ Ethernet (ZMQ)
                                    │ JETSON_IP / LAPTOP_IP
┌───────────────────────────────────▼─────────────────────────────────────────┐
│  JETSON (Robot)   start_robot.sh                                              │
│  drive_bridge, vision_node, slam_bridge, followerarm, status_publisher,      │
│  autonomy_node | autonomy_executor (if Nav2), Unitree L2 + Point-LIO          │
└─────────────────────────────────────────────────────────────────────────────┘
```

All application-level links use **ZMQ** (TCP). Ports are defined in `shared/constants.py`.

---

## Repository structure

| Path | Purpose |
|------|---------|
| **`laptop/`** | Operator station: GUI (`gui/`), joystick & arm leader (`control/`), `main.py` entrypoint |
| **`jetson/`** | Robot: `start_robot.sh`, `drive_bridge.py`, `vision/` (vision_node), `slam_bridge.py`, `followerarm.py`, `autonomy_node.py` / `autonomy_executor.py`, ROS2/SLAM/Nav2 launch |
| **`shared/`** | `constants.py` (IPs, ZMQ ports, serial ports, camera config, scoring), protocol helpers |
| **`jetson/models/`** | YOLO datasets, training scripts, **`engines/`** for TensorRT engine files (see [jetson/models/engines/README.md](jetson/models/engines/README.md)) |
| **`jetson/vision/`** | Vision node, YOLO11/Landolt classifier, camera manager ([jetson/vision/README.md](jetson/vision/README.md)) |
| **`docs/`** | Runbooks, checklists, integration notes (e.g. `OPERATOR_RUNBOOK.md`, `COMPETITION_CHECKLIST.md`) |
| **`reports/`** | Mission QR log, report generator output |
| **`pi/`** | Optional Pi camera streamer (if used) |

---

## Prerequisites

- **Python** 3.10+ (3.12 recommended for laptop).
- **Laptop:** PyQt5, joystick (e.g. Logitech), optional LeRobot-capable leader arm (SO-ARM).
- **Jetson:** ROS 2 Humble (or Iron), Unitree L2 lidar (via `unilidar_sdk2`), Point-LIO (`catkin_point_lio_unilidar`), 3× USB cameras, Pico (drive), Feetech bus (arm). Optional: DeepStream, Nav2.
- **Network:** Static Ethernet between laptop and Jetson (e.g. 192.168.2.x); no internet required for teleop.

---

## Installation

### 1. Clone

```bash
git clone https://github.com/YOUR_USERNAME/robocup_rescue_2026.git
cd robocup_rescue_2026
```

Replace `YOUR_USERNAME` with your GitHub username or organization.

### 2. Python dependencies

**Laptop (operator):**

```bash
pip install -r requirements.txt
```

**Jetson (robot):**  
Use the same root `requirements.txt`; on Jetson also ensure NumPy &lt; 2 (see `jetson/requirements.txt`):

```bash
pip install -r requirements.txt
pip install -r jetson/requirements.txt   # numpy<2 and Jetson notes
```

Optional: use a virtualenv on the laptop:

```bash
python3 -m venv .venv
source .venv/bin/activate   # or .venv\Scripts\activate on Windows
pip install -r requirements.txt
```

### 3. Configuration

Edit **`shared/constants.py`** (or override via environment where supported):

| Setting | Default | Description |
|---------|---------|-------------|
| `JETSON_IP` | `192.168.2.100` | Robot (Jetson) IP |
| `LAPTOP_IP` | `192.168.2.101` | Operator (laptop) IP |
| `PICO_SERIAL_PORT` | `/dev/ttyACM1` | Pico serial on Jetson (drive) |
| `ARM_SERIAL_PORT` | `/dev/ttyACM0` | Arm (Feetech) serial on Jetson |
| `ARM_LEADER_SERIAL` | macOS path | Leader arm serial on laptop (Linux: often `/dev/ttyUSB0` or `/dev/ttyACM*`) |
| `CAMERAS` | front/arm/backward devices | Camera device paths on Jetson |

**Leader arm (laptop):** If you use the LeRobot-based leader arm, set the env var:

```bash
export LEROBOT_SRC=/path/to/lerobot/src
```

**YOLO engines:** Place TensorRT engine files in **`jetson/models/engines/`** so the vision node can run HAZMAT and Landolt-C detection. Without them, the node still streams video. See [jetson/models/engines/README.md](jetson/models/engines/README.md) for names and export steps.

---

## Quick start

### Robot (Jetson)

From the repo root or from `jetson/`:

```bash
cd jetson
./start_robot.sh
```

This starts: drive bridge, vision node (3 cameras, 2 with AI), status publisher, follower arm, Unitree L2 + Point-LIO, SLAM bridge, autonomy node, and optionally RViz stream (TCP 5600). Logs go to `jetson/logs/`.

### Operator (laptop)

```bash
cd laptop
python main.py
```

Or with an explicit Jetson IP:

```bash
python main.py --jetson-ip 192.168.2.100
```

**Useful flags:**

| Flag | Description |
|------|-------------|
| `--no-gui` | Joystick only (no GUI) |
| `--no-joystick` | GUI only (e.g. camera + SLAM view) |
| `--no-arm` | Do not start arm leader |
| `--calibrate` | Joystick axis calibration |
| `--arm-serial PORT` | Override leader arm serial port |
| `--arm-no-lerobot` | Arm leader without hardware (neutral positions) |

---

## Jetson environment options

These can be set before running `./start_robot.sh`:

| Variable | Values | Effect |
|----------|--------|--------|
| `USE_DEEPSTREAM_SDK` | `0` (default), `1` | 1 = use DeepStream vision pipeline instead of vision_node |
| `USE_VISION_NODE` | `1` (default) | Use combined vision node (HAZMAT + Landolt + streams) when not using DeepStream |
| `USE_RVIZ_ON_JETSON` | `0`, `1` (default) | 1 = start RViz and stream (TCP 5600) on Jetson |
| `USE_SIM` | `0` (default), `1` | 1 = skip Pico/drive_bridge, optional Gazebo |
| `USE_NAV2` | `0` (default), `1` | 1 = run Nav2 stack + autonomy_executor + ROS2 drive_bridge |
| `USE_RERUN` | `0` (default), `1` | 1 = run Rerun bridge (port 9876) for 3D view from laptop |

---

## ZMQ ports (reference)

| Port | Purpose |
|------|---------|
| 5555 | Drive (joystick → Jetson) |
| 5556 | Arm commands (JSON) |
| 5557 | Camera streams (Jetson → laptop) |
| 5558 | Arm teleop (LeRobot msgpack) |
| 5559 | Robot status (Jetson → laptop) |
| 5560 | Autonomy commands (laptop → Jetson) |
| 5561 | Camera control (YOLO/QR enable) |
| 5562 | SLAM / Nav2 bridge (pose + map) |
| 5563 | Lidar scan |
| 5565 | Autonomy status |
| 5570 | Snapshot request (executor → vision) |
| 5571 | Snapshot result (vision → laptop) |
| 5572 | RViz window stream (ZMQ) |
| 5600 | RViz MJPEG stream (TCP, when `USE_RVIZ_ON_JETSON=1`) |

---

## Vision and YOLO

- **Cameras:** 3 on Jetson (front, arm, backward); 2 used for YOLO (front, arm). Config in `shared/constants.py` and `jetson/vision/`.
- **Models:** Put `.engine` (and optionally `.pt`) files in `jetson/models/engines/`. Preferred names: `hazmat_yolo11n.engine`, `landolt_yolo11n.engine`, `yolo11n.engine`. See [jetson/models/engines/README.md](jetson/models/engines/README.md).
- **Training:** `jetson/models/download_datasets.py` and `train_models.py` for HAZMAT and Landolt-C; export to TensorRT on the Jetson.

---

## Documentation

| Document | Description |
|----------|-------------|
| [docs/OPERATOR_RUNBOOK.md](docs/OPERATOR_RUNBOOK.md) | Day-of operations, startup order, troubleshooting |
| [docs/COMPETITION_CHECKLIST.md](docs/COMPETITION_CHECKLIST.md) | Pre-competition checks |
| [jetson/vision/README.md](jetson/vision/README.md) | Vision pipeline, Landolt/YOLO, camera layout |
| [jetson/models/engines/README.md](jetson/models/engines/README.md) | YOLO engine names and export |
| Other under `docs/` | QR troubleshooting, camera FPS, autonomy integration, Rerun, etc. |

---

## Tests

From repo root:

```bash
pytest tests/ -v
```

Relevant tests: `tests/test_drive.py`, `tests/test_arm.py`, `tests/test_cameras.py`, `tests/test_protocol.py`. Vision 4-camera test: `jetson/vision/test_4cameras.py` (run from Jetson with cameras attached).

---

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development setup, code style, and pull request guidelines.

## License

MIT License — see [LICENSE](LICENSE). Use according to your team and competition rules.
