# RMRC 2025 Auto Mission – Integration

## 1. How to run the auto mission node

**Prerequisites:** ROS2 Humble, Point-LIO running (`/Odometry`, `/cloud_registered`), drive_bridge subscribing to `/cmd_vel`, followerarm on ZMQ 5558.

- **Option A – After full robot start**  
  Run `jetson/start_robot.sh` (Point-LIO + drive_bridge + slam_bridge + followerarm). Then in a separate terminal on the Jetson:

  ```bash
  cd /path/to/robocup_rescue_2026
  export PYTHONPATH=$PWD
  python3 jetson/auto_mission_node.py
  ```

- **Option B – Launch script**  
  From repo root:

  ```bash
  PYTHONPATH=$PWD python3 jetson/launch/auto_launch.py
  ```

- **Option C – With config**  
  ```bash
  python3 jetson/auto_mission_node.py --bay-length 4.0 --wall-distance 0.15 --end-threshold 0.30 --timeout 300
  ```

Ensure **drive_bridge runs with ROS2** so it subscribes to `/cmd_vel` (e.g. in `start_robot.sh` set `USE_NAV2=1` or run drive_bridge with `--ros2`).

---

## 2. Calling from existing movement code

- **Auto mission** publishes `geometry_msgs/Twist` to `/cmd_vel`. Your existing movement stack (e.g. drive_bridge) already subscribes to `/cmd_vel` and converts to motor commands. No change needed there.
- When **autonomy is active**, the auto_mission_node is the only writer to `/cmd_vel`. Pause or stop the auto mission (GUI “ABORT” or ZMQ `{"cmd":"abort"}`) before sending manual Twist from joystick or Nav2.
- **Arm:** The node uses the same arm path as autonomy_executor: ZMQ PUSH to `127.0.0.1:5558` (followerarm). It calls `arm_presets.send_preset("stowed")` etc. No change to your existing arm code.

---

## 3. Announcing autonomy to the judge

- **Before an autonomous lap:** Operator announces (e.g. “Starting autonomous lap, hands off except in 30 cm of end walls”). Judge confirms. Then press **“Start Auto Lap”** on the GUI.
- **In code:** The node logs and prints:
  - On start lap:  
    `[AUTO] AUTONOMOUS LAP STARTED — Announce to judge. Hands off except when any part within 30 cm of bay end walls.`
  - Status is sent over ZMQ (port 5565) with `in_end_zone_30cm`, `lap_count`, `autonomy_active` for the GUI.
- **GUI:** Use the Auto Status widget labels: “Autonomy: ACTIVE”, “30cm zone: GREEN” (hands-on allowed) / “RED” (hands off). This gives judge-visible status.

---

## 4. ZMQ messages

- **Commands (laptop → Jetson, port 5560, PUSH):**
  - Start lap: `{"cmd": "start_auto_lap"}` or `{"mode": "lap"}`
  - Stowed: `{"cmd": "stowed_posture"}` or `{"cmd": "stowed"}`
  - Select test: `{"cmd": "select_test", "test_type": "krails_horiz"}`
  - Abort: `{"cmd": "abort"}` or `{"cmd": "stop"}`
- **Status (Jetson → laptop, port 5565, SUB):**  
  JSON with `msg_type: "auto_mission_status"`, `state`, `test_type`, `lap_count`, `laps_completed`, `in_end_zone_30cm`, `autonomy_active`, `stowed_done`, `message`, `step`, `step_label`, `progress`.  
  Same port as existing autonomy status; GUI can show laps and 30 cm zone from this.

---

## 5. Adding the Auto Status widget to the GUI

In `laptop/gui/main_window.py`:

1. Import:
   ```python
   from gui.auto_status_widget import AutoStatusWidget, create_and_connect
   ```
2. After creating `self.autonomy_panel`, create and connect the widget (use existing `autonomy_socket` so one PUSH is used):
   ```python
   def _send_autonomy_cmd(self, msg):
       if getattr(self, 'autonomy_socket', None):
           try:
               self.autonomy_socket.send_json(msg)
           except Exception:
               pass
   self.auto_status_widget = create_and_connect(
       self, self.jetson_ip, autonomy_socket_send=self._send_autonomy_cmd
   )
   if self.auto_status_widget:
       self.control_stacked.addWidget(self.auto_status_widget)  # or add to autonomy_panel layout
   ```
3. In the timer that polls `autonomy_status_socket`, when you receive a message, if it contains `msg_type == "auto_mission_status"` (or any autonomy status), call:
   ```python
   if hasattr(self, 'auto_status_widget') and self.auto_status_widget and msg.get('msg_type') == 'auto_mission_status':
       self.auto_status_widget.set_status(msg)
   ```
   Also keep updating `autonomy_panel` (laps, step) from the same message for compatibility.

---

## 6. Config snippet (bay, wall distance, speeds)

See `jetson/config/auto_mission_config.yaml`. Key parameters (override via CLI or load in node later):

| Parameter              | Default | Description                          |
|------------------------|---------|--------------------------------------|
| bay_length_m            | 4.0     | Bay length (m) for lap end detection |
| wall_distance_target_m | 0.15    | 10–20 cm from walls                 |
| end_wall_threshold_m    | 0.30    | Stop within 30 cm of end wall       |
| speed_normal_mps        | 0.15    | Normal terrain                       |
| speed_slow_mps          | 0.08    | Incline/sand/stairs                 |
| mission_timeout_s       | 300     | 5 min max mission                   |

CLI: `--bay-length`, `--wall-distance`, `--end-threshold`, `--timeout`, `--bay-axis`, `--control-hz`.

---

## 7. Robustness (already in node)

- **Timeout:** Mission aborts after 5 min (`mission_timeout_s`).
- **Stuck:** If the robot moves &lt; 8 cm over the last 5 s (50 samples at 10 Hz), it stops (safety stop).
- **Odometry:** EMA filter on pose for smoother control.
- **Logging:** In-memory event log for judge evidence (`_log_ev`); can be extended to file if needed.

---

## 8. Test types (lap-based vs single-shot)

- **Lap-based (wall-follow + end detect):** incline_horiz, incline_inclined, sand_gravel, ramps_*, krails_*, hurdles_*, stairs, labyrinth_flat, labyrinth_krails. Select via “Select Test” on the widget; same behavior, different speed (e.g. slow on incline/sand).
- **Align:** State `ALIGN`; centering + rotate to face end wall (separate flow; select test “align” and start from GUI when implemented).
- **Keypad / Linear rail / Sensor cabinet / Finals maze:** Placeholders in state machine; arm/camera integration can be wired the same way as in `autonomy_executor` (arm presets, snapshot request).

You can keep using the same movement and arm stack; only the autonomy “brain” is replaced or run in parallel (use one of AutoMissionNode or autonomy_executor at a time on the same ZMQ ports).
