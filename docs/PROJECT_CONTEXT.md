# RoboCup Rescue RMRC 2026 — Project Context
> Single source of truth for the SOSRoboticsTeamHU robot stack and the 2026 rulebook. Read this at the start of every coding/agent session before answering questions or making changes.

The competition rulebook PDF is committed at the repo root: [`RMRC2026_Rules.pdf`](../RMRC2026_Rules.pdf). When in doubt, the PDF wins.

---

## 1. Project overview
Split-architecture rescue robot for RoboCup Rescue RMRC 2026.
- **Repository**: <https://github.com/SOSRoboticsTeamHU/robocup-rmrc-2026>
- **Deadline**: end of May 2026
- **Languages**: Python 91 %, Shell 5 %, C++ 4 % (Pico firmware via CMake)
- **Python**: 3.10+ everywhere; 3.12 recommended on the laptop. **NumPy < 2 on Jetson** (compatibility hard requirement).

### 1.1 Architecture
- **Laptop** (operator station): PyQt5 GUI, joystick, arm leader (LeRobot SO-ARM), camera receiver, SLAM view, mission report PDF.
- **Jetson** (robot brain): `drive_bridge`, `vision_node` (YOLO11 HAZMAT + Landolt-C), `slam_bridge`, `followerarm`, `autonomy_node`/`autonomy_executor`.
- **Pico** (microcontroller): C++ firmware for drive control via serial.
- **Communication**: ZMQ over Ethernet (TCP). All ports defined in `shared/constants.py` — never hardcode.

### 1.2 Hardware
- Jetson Orin Nano Super (ROS 2 Humble), 3× USB cameras (front/arm/backward), Unitree L2 lidar.
- Robot arm: Leader on laptop (LeRobot SO-ARM) → ZMQ → Follower on Jetson (Feetech bus).
- SLAM: Point-LIO + Unitree L2 lidar; optional Nav2 + autonomy_executor.
- Optional: DeepStream pipeline, Rerun bridge.

### 1.3 ZMQ ports (`shared/constants.py`)
| Port | Purpose |
|---|---|
| 5555 | Drive commands (joystick → Jetson) |
| 5556 | Arm commands (JSON) |
| 5557 | Camera streams (Jetson → laptop) |
| 5558 | Arm teleop (LeRobot msgpack) |
| 5559 | Robot status |
| 5560 | Autonomy commands **and** mission-reset broadcasts |
| 5561 | Camera control (YOLO/QR enable) |
| 5562 | SLAM/Nav2 bridge (pose + map + fiducials + path) |
| 5563 | Lidar scan |
| 5564 | Point cloud |
| 5565 | Autonomy status |
| 5570/5571 | Snapshot request/result |
| 5572 / 5600 | RViz stream (ZMQ / TCP MJPEG) |

### 1.4 File structure
- `laptop/` — operator GUI, joystick, arm leader, `main.py`.
- `jetson/` — robot brain: `drive_bridge.py`, `vision/`, `slam_bridge.py`, `followerarm.py`, `autonomy_node.py`, `autonomy_executor.py`.
- `jetson/models/` — YOLO datasets, training scripts, `engines/` (TensorRT `.engine` files).
- `jetson/vision/` — vision node, YOLO11/Landolt classifier, camera manager.
- `shared/` — `constants.py`, `protocol.py`, `mission_reset.py`, `grid_codec.py`.
- `pico_cpp/` — C++ Pico firmware.
- `pi/` — optional Pi camera streamer.
- `docs/` — runbooks, checklists, this file.
- `tests/` — pytest suite.
- `tools/` — utility scripts.
- `reports/` — mission QR log, fiducials.json, PDF report output.

### 1.5 Feature flags (start_robot.sh)
`USE_NAV2`, `USE_DEEPSTREAM_SDK`, `USE_RVIZ_ON_JETSON`, `USE_SIM`, `USE_RERUN`, `USE_TETHER_MODE`.

---

## 2. RMRC 2026 rules — what matters for code
Comes from the rulebook in the repo root.

### 2.1 Mission timing
- 10-minute time slot = 3 min setup + 5 min mission + 2 min teardown, run on a **global clock** (no slips). Late teams only get the remaining mission time.
- Operator faces away from the robot/bay; only the judge may communicate during a run.
- Tether mode is strongly recommended because venue Wi-Fi is congested. A separate team member manages the tether, forbidden from talking to the operator. Motorised tether drums are allowed.

### 2.2 The 18 preliminary tests
Mobility (10 lap-based): Incline & Center — Horizontal / Inclined; Sand & Gravel; Ramps Continuous / Pinwheel; Elevated Ramps; Stairs (5-second level rule); Align (1 cm precision); K-Rails Horizontal / Crossover; Hurdles Single / Double.
Dexterity (3): Keypad Omni; Linear Rail Pick; Linear Rail Inspect.
Sensing (2): Sensor Cabinet (door + HAZMAT + concentric Landolt-C 3rd ring + thermal + magnets); Labyrinth Flat / K-Rails (QR + mapping bonus + Sensor Cabinet inside).
Other: Drop Test (finals only, optional).

### 2.3 Scoring priorities (highest impact first)
1. **Autonomy 4× multiplier** on laps and dexterity tasks. The single biggest score lever.
2. **Mobility laps**: 1 pt teleop, 4 pts autonomous.
3. **Dexterity**: Linear Rail Pick (1 pt removed, 2 pts in container); Keypad Omni (3 pts clean "5", 1 pt dirty); Linear Rail Inspect (25 pt prelims max, 10 pt finals).
4. **Sensing**: HAZMAT 1 pt, Landolt-C 1 pt per ring (Linear Rail) or 1 pt for the 3rd ring (Sensor Cabinet), thermal 1 pt, magnet pattern 1 pt, door 1 pt.
5. **Labyrinth**: QR 1 pt + 4 pts mapping bonus (if within 30 cm of two closest fiducials).
6. **TDM multiplier**: multiplies the **entire** preliminary score. Quality > polish.

### 2.4 Critical autonomy rules
- **Autonomous lap**: operator is hands-off the OCU except when any part of the robot is within 30 cm of an end wall.
- **Dexterity autonomy** (NEW 2026 — changelog 2026-04-12): robot must be **≥30 cm from the target object** before the autonomy trigger is accepted. Teleoperating closer than 30 cm voids the 4× claim.
- **Single command rule**: dexterity autonomy is started by exactly one event (button click, ZMQ message). No multi-step orchestration from the operator.
- **Stowed posture**: required before each autonomous attempt for dexterity/sensing.
- **Zero false positives** for autonomous HAZMAT/Landolt cabinet reads. Single still image, single annotation.
- **Linear Rail Inspect autonomy**: just position the camera; reading itself can be by the operator. Re-attempting the same tube autonomously requires moving ≥30 cm away first.

### 2.5 Mini-missions / resets
- Touching the robot or arena triggers a reset: robot returns to start, score zeroes, run continues with remaining time.
- Each reset is a fresh "mini-mission" with its own scoresheet.
- **Only the best mini-mission counts**. In-mission state must be cleared across vision/scoring/QR-log when an operator-initiated `mission_reset` is broadcast.

### 2.6 Robot configuration freeze
- Hardware/accessories must not change between prelim start and end of finals.
- Software updates are allowed.
- Required photographs (with 20 cm grid backdrop): Front, Side, Top, "publicity" angle, close-ups of unusual features, in **both** stowed and operating posture. See `docs/robot_photos/CHECKLIST.md`.

### 2.7 Best-in-Class
Need positive non-zero score in **≥80 % of available tests** (≥15 of 18). Best-in-Class Autonomy explicitly counts QR mapping bonus and sensor cabinet auto points.

### 2.8 Finals (day 3)
- Scoring is **not** normalised: 1 pt teleop / 4 pts autonomous, max 2 pts per terrain (each direction once).
- Linear Rail finals: only the 3rd and 4th Landolt-Cs count; tubes pre-loaded with manipulation objects.
- Keypad Omni finals: only "5" counts; 1 pt per keypad; pushing any other key on same keypad → 0 pts.
- Labyrinth finals: 5 sheets, only 2 mm code counts; mapping bonus replaces autonomy bonus; no Sensor Cabinet inside.
- Drop test (optional): 1 pt (15 cm), 2 pts (30 cm). Must score ≥1 pt in another test post-drop.
- Tie threshold: gaps <5 % count as ties.

### 2.9 Common pitfalls
- Touching controls during an autonomous lap (outside 30 cm of end walls) → instant teleop downgrade.
- Keypad Omni: pushing a neighbour key after a clean "5" *reduces* the keypad's score from 3 to 1. Drive off the dexterity beams to "lock in".
- QR codes: must be written to a file **and** verbally declared to the judge.
- Linear Rail Pick: grasping only — no suction, magnets, adhesives, skewering. Container is team-supplied, ≤30×30×10 cm.
- HAZMAT autonomy: zero false positives required.
- NUM LOCK on Keypad Omni scoring computer: protect from accidental presses.
- Configuration freeze: deciding "arm on/off" must happen **before** Day 1.

---

## 3. Code style & contracts
1. Always check `shared/constants.py` for existing constants before adding new ones. Never hardcode IPs/ports/device paths.
2. ZMQ socket choice: PUSH/PULL for unidirectional, PUB/SUB for broadcast (with `CONFLATE=1` for state), REQ/REP for request-response.
3. Camera streams: GPU MJPEG on Jetson (`nvjpegdec`/`nvjpegenc`). Backward camera is Light MJPEG only.
4. YOLO: prefer YOLO11n for speed; export to TensorRT `.engine` for Jetson inference.
5. Autonomy behaviours triggered by exactly one ZMQ event. No multi-step operator handshakes.
6. Always handle the no-hardware case (`USE_SIM=1`).
7. Robot logs go to `jetson/logs/`.
8. **30 cm dexterity guard**: every dexterity autonomy entrypoint validates pose ≥`DEXTERITY_AUTONOMY_MIN_DIST_M` from target before accepting trigger. Bench bypass: `--allow-near-target`.
9. **Idempotent mission reset**: every long-lived ZMQ component subscribes to autonomy port for `msg_type == "mission_reset"` and clears in-mission state.
10. **Zero false positives**: detections used to claim 4× points must pass temporal consistency (`SNAPSHOT_FRAME_CONSISTENCY_N` consecutive frames at ≥`SNAPSHOT_MIN_CONFIDENCE`).
11. **Single-still-image contract**: `snapshot_mode` returns one annotated JPEG per autonomy claim.
12. **Map output must include fiducials** so the QR mapping bonus is visually verifiable.

---

## 4. Testing
- `pytest tests/ -v` from repo root.
- Hardware tests: `jetson/vision/test_4cameras.py` (requires physical Jetson + cameras).
- Always test with `USE_SIM=1` first before hardware.

---

## 5. Monthly plan (May 2026)
### Week 1 (May 1–7) — Mobility + autonomy foundation
- Nav2 integration, first autonomous laps.
- Stowed posture defined + photographed.
- **`reset_mission()` IPC** wired across Jetson/laptop.
- **30 cm dexterity guard** in `autonomy_executor.py`.

### Week 2 (May 8–14) — Sensing & vision
- HAZMAT YOLO training + TensorRT export.
- Landolt-C classifier accuracy improvement.
- QR file output + mapping implementation.
- Thermal + magnet detection.
- **Fiducial layer** in `slam_bridge.py` + map overlay.
- **Single-still snapshot pipeline** for HAZMAT/Landolt cabinet reads.
- **Sensor Cabinet door opener**.

### Week 3 (May 15–21) — Dexterity
- Linear Rail Pick autonomous grasping.
- Keypad Omni precision (only "5", no neighbours).
- "Safe retreat" arm trajectory after a clean "5".
- Linear Rail Inspect "smallest readable" report flow.
- Harder mobility tests.

### Week 4 (May 22–31) — Integration + TDM
- Full system integration test (5-min mission simulation).
- TDM documentation writing.
- Robot photo documentation (stowed + operating, 20 cm grid).
- Tether mode drill (set `USE_TETHER_MODE=1`).
- 3-min/5-min/2-min drill.
- **Drop-test post-recovery** sanity check.
- Aim for **non-zero score in 15+ of 18 tests** (Best-in-Class breadth).

---

## 6. References
- Rulebook PDF: [`../RMRC2026_Rules.pdf`](../RMRC2026_Rules.pdf)
- Construction guide: [`../RMRC 2025 Construction Guide 2025-07-09.txt`](../RMRC%202025%20Construction%20Guide%202025-07-09.txt)
- Operator runbook: [OPERATOR_RUNBOOK.md](OPERATOR_RUNBOOK.md)
- Competition checklist: [COMPETITION_CHECKLIST.md](COMPETITION_CHECKLIST.md)
- Photo documentation: [robot_photos/CHECKLIST.md](robot_photos/CHECKLIST.md)
- Sprint plan: [FULL_SPRINT_IMPLEMENTATION_PLAN.md](FULL_SPRINT_IMPLEMENTATION_PLAN.md)
