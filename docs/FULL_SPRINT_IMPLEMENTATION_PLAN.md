# Full Competition Readiness — Implementation Plan (File-by-File)

Execute in day order. New files first, then modifications.

---

## Day 0 – Calibration & Gazebo (half day)

| Order | Action | File |
|-------|--------|------|
| 1 | Modify | `shared/constants.py` — Pre-fill `FIDUCIAL_POSITIONS` (placeholder); add `GUI_MODES`, `TEST_TYPES` stubs if not in Day 4 |
| 2 | Create | `jetson/gazebo/rescue_robot_gazebo.launch.py` — Launch URDF + Point-LIO + nav2_bridge + autonomy_executor in Gazebo (flat/incline/sand) |
| 3 | Modify | `jetson/start_robot.sh` — Add `USE_SIM=1`: skip Pico, use sim drive_bridge when set |

---

## Day 1: Nav2 Foundation + Autonomy Executor

| Order | Action | File |
|-------|--------|------|
| 1 | Create | `jetson/nav2_bridge.py` — TF map→odom→base_link; OccupancyGrid from `/cloud_registered`; dual ROS2 + ZMQ output |
| 2 | Create | `jetson/config/nav2_params.yaml` — Footprint 0.27×0.25 m, inflation 0.18 m, DWB, NavFn/SmacPlanner2D |
| 3 | Create | `jetson/launch_nav2.sh` — Source ROS2, launch robot_state_publisher, nav2_bridge, Nav2 bringup |
| 4 | Modify | `jetson/ros2_ws/.../drive_bridge.py` — cmd_vel scaling: m/s → motor % via `max_linear_vel_mps` |
| 5 | Create | `jetson/autonomy_executor.py` — ZMQ 5560 listener, Nav2 action client, modes (lap, sensor_cabinet, etc.), publish nav2_path, nav2_goal, step, laps_completed; **Task 1.7**: safety_watchdog 100 ms, Nav2 stuck >15 s → ABORT + TTS, battery <11.8 V, joystick log (30 cm exception) |
| 6 | Modify | `jetson/start_robot.sh` — `USE_NAV2=1`: launch robot_state_publisher, nav2_bridge, launch_nav2, autonomy_executor, drive_bridge --ros2 |

**Validation (Day 1):** `ros2 topic echo /map`, `ros2 run tf2_tools view_frames`, `ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}'`, send `{"mode":"lap","waypoint_a":[0,0],"waypoint_b":[2,0]}` → robot drives.

---

## Day 2: GUI Overhaul

| Order | Action | File |
|-------|--------|------|
| 1 | Modify | `laptop/gui/slam_view.py` — **Major**: ego-centric 4-line QPainter transform; cached grid QPixmap; path trail (fading indigo); nav2_path (cyan dashed); nav2_goal (orange pulse); distance rings 1/2/3 m; cardinal N/S/E/W; North-Up toggle; scale bar after resetTransform() |
| 2 | Modify | `laptop/gui/main_window.py` — **Major**: `self.gui_mode` (teleop/autonomy/snapshot_review); `QStackedWidget` control panels (0=teleop, 1=autonomy, 2=snapshot); `set_mode()`; camera layout (2×2 vs arm-expanded); joystick safety interlock 500 ms + 30 cm end-wall exception via SLAM pose; test selector dropdown (grouped); per-test scoring buttons; Linear Rail Pick QComboBox: "Full Autonomy", "Pause for Base Teleop", "Full Teleop (arm only)" |

**Validation (Day 2):** SLAM robot centered/heading-up; path trail; Nav2 overlay in autonomy; mode switches; joystick interlock.

---

## Day 3: Test-Specific Autonomy

| Order | Action | File |
|-------|--------|------|
| 1 | Create | `jetson/arm_presets.py` — stowed, inspect_high/mid/low, cabinet_view, keypad_push, pick_ready; `send_preset(name, zmq_socket)` |
| 2 | Modify | `jetson/config/robot_params.yaml` — Real joint values for arm presets (calibrate on robot) |
| 3 | Create | `jetson/vision/snapshot_mode.py` — HAZMAT YOLO still (bbox+label); Landolt still; send snapshot_result to GUI |
| 4 | Modify | `jetson/vision/landolt_classifier.py` — Option B Hough+radial fallback or simple heuristic |
| 5 | Modify | `jetson/autonomy_executor.py` — LapExecutor, LabyrinthExplorer, SensorCabinetExecutor, KeypadExecutor, InspectTubeExecutor; PickExecutor with 3 modes (Full Auto, Pause for Teleop, Full Teleop) |
| 6 | Modify | `shared/protocol.py` — snapshot_result, nav2_status; extend AutonomyStatus (nav2_path, nav2_goal, step, laps_completed) |

**Validation (Day 3):** Arm presets move arm; snapshot produces still image; each executor runs without crash.

---

## Day 4: Scoring, Docs, Testing

| Order | Action | File |
|-------|--------|------|
| 1 | Modify | `laptop/reports/mission_report.py` — Fix line 78: `total = teleop + auto` (auto already has 4×); per-test scoring dispatch using TEST_TYPES |
| 2 | Modify | `shared/constants.py` — `TEST_TYPES` list; `GUI_MODES`; optional FIDUCIAL_POSITIONS from photos |
| 3 | Modify | `laptop/gui/main_window.py` — Linear Rail Inspect: tube C1–C5, direction dropdown, "Record Reading", "Show Gap" |
| 4 | Create | `docs/OPERATOR_RUNBOOK.md` — Per-test procedures, Pick 3 modes, watchdog, judge phrases |
| 5 | Optional | `docs/robot_photos/` — Placeholder; COMPETITION_CHECKLIST fiducial/photo/container |

**Validation (Day 4):** Mission report no double-count; PDF for 3 test types; runbook readable; dry run.

---

## Validation Commands (Full)

```bash
# --- Day 0 / Day 1 ---
# TF / Map (after Point-LIO + nav2_bridge running)
ros2 topic echo /map --once
ros2 run tf2_tools view_frames

# Odom
ros2 topic echo /odom --once

# cmd_vel scaling (drive_bridge --ros2)
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}' --once

# Autonomy executor (ZMQ 5560) — from laptop or: python3 -c "import zmq,json; c=zmq.Context(); s=c.socket(zmq.PUSH); s.connect('tcp://JETSON_IP:5560'); s.send_json({'mode':'lap','waypoint_a':[0,0],'waypoint_b':[2,0]})"

# --- Day 2 ---
# GUI: Start OperatorStation; SLAM view = ego-centric (robot center, heading up); North-Up toggle; Nav2 path/goal when autonomy status has nav2_path/nav2_goal
# set_mode('teleop'|'autonomy'|'snapshot_review') for HUD accent

# --- Day 3 ---
# Arm presets: python3 jetson/arm_presets.py stowed
# Snapshot: use jetson/vision/snapshot_mode.capture_hazmat_still / build_snapshot_result from vision pipeline

# --- Day 4 ---
# Mission report: python3 laptop/reports/mission_report.py -o report.pdf --laps-auto 2 --qr-read 1
# Operator runbook: docs/OPERATOR_RUNBOOK.md
```
