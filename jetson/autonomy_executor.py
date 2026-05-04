#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Autonomy Executor (Jetson)
=====================================================
Replaces autonomy_node.py. ZMQ listener (5560), Nav2 action client, modes: lap, sensor_cabinet,
inspect_tube, keypad, pick, explore. Publishes autonomy_status (5565) with nav2_path, nav2_goal,
step, laps_completed. Task 1.7: safety_watchdog (100 ms), Nav2 stuck >15s → ABORT + TTS, battery
<11.8V warning, joystick log with 30cm end-wall exception.

Validation: send {"mode":"lap","waypoint_a":[0,0],"waypoint_b":[2,0]} → robot drives between points.
"""

import sys
import os
import time
import math
import json
import argparse
from typing import Optional, List, Tuple, Dict, Any
from enum import Enum
from dataclasses import dataclass, asdict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:
    import zmq
except ImportError:
    zmq = None

try:
    from shared.constants import (
        ZMQ_PORT_AUTONOMY,
        ZMQ_PORT_SLAM,
        ZMQ_PORT_DRIVE,
        ZMQ_PORT_AUTONOMY_STATUS,
        ZMQ_PORT_ARM_TELEOP,
        ZMQ_PORT_SNAPSHOT_REQUEST,
        ZMQ_PORT_SNAPSHOT_RESULT,
        ZMQ_PORT_STATUS,
        NAV2_STUCK_TIMEOUT_S,
        BATTERY_WARNING_V,
        END_WALL_EXCEPTION_M,
        FIDUCIAL_POSITIONS,
        DEXTERITY_AUTONOMY_MIN_DIST_M,
        SLAM_GRID_RESOLUTION,
    )
except ImportError:
    ZMQ_PORT_AUTONOMY = 5560
    ZMQ_PORT_SLAM = 5562
    ZMQ_PORT_DRIVE = 5555
    ZMQ_PORT_AUTONOMY_STATUS = 5565
    ZMQ_PORT_ARM_TELEOP = 5558
    ZMQ_PORT_SNAPSHOT_REQUEST = 5570
    ZMQ_PORT_SNAPSHOT_RESULT = 5571
    ZMQ_PORT_STATUS = 5559
    NAV2_STUCK_TIMEOUT_S = 15
    BATTERY_WARNING_V = 11.8
    END_WALL_EXCEPTION_M = 0.30
    FIDUCIAL_POSITIONS = []
    DEXTERITY_AUTONOMY_MIN_DIST_M = 0.30
    SLAM_GRID_RESOLUTION = 0.05

try:
    from shared.mission_reset import is_mission_reset
except ImportError:
    def is_mission_reset(msg):
        return isinstance(msg, dict) and msg.get("msg_type") == "mission_reset"

try:
    from auto_behaviors import recover_from_stuck, wall_follow, straight_line_fallback
except ImportError:
    recover_from_stuck = None
    wall_follow = None
    straight_line_fallback = None

MAX_RECOVERY_ATTEMPTS = 2

# Optional ROS2 / Nav2
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped
    from builtin_interfaces.msg import Duration
    ROS2_AVAILABLE = True
except ImportError:
    pass


class AutonomyState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    ERROR = "error"
    ABORTING = "aborting"


@dataclass
class AutonomyStatusMessage:
    """Published to GUI on port 5565."""
    msg_type: str
    state: str
    mode: str
    progress: float
    message: str
    timestamp: float
    # Nav2 overlay for SLAM view
    nav2_path: Optional[List[List[float]]] = None
    nav2_goal: Optional[List[float]] = None
    goal_distance_m: Optional[float] = None
    step: Optional[str] = None
    step_label: Optional[str] = None
    laps_completed: int = 0
    # Safety
    battery_voltage: Optional[float] = None
    nav2_stuck: bool = False


def _tts_speak(text: str) -> None:
    """Optional TTS for judge/operator (e.g. 'Nav2 stuck, switching to teleop')."""
    try:
        import subprocess
        subprocess.run(["espeak", "-s", "120", text], capture_output=True, timeout=2)
    except Exception:
        pass


class SafetyWatchdog:
    """Task 1.7: 100 ms timer checks; Nav2 stuck >15s → ABORT; battery <11.8V warning; joystick log (30cm exception)."""
    def __init__(self, stuck_timeout_s: float = NAV2_STUCK_TIMEOUT_S, battery_warning_v: float = BATTERY_WARNING_V):
        self.stuck_timeout = stuck_timeout_s
        self.battery_warning_v = battery_warning_v
        self.last_nav2_goal_time: Optional[float] = None
        self.last_pose: Optional[Tuple[float, float]] = None
        self.last_pose_time: Optional[float] = None
        self.battery_v: Optional[float] = None
        self.joystick_log_callback: Optional[Any] = None
        self.in_30cm_zone_callback: Optional[Any] = None  # () -> bool

    def set_battery(self, voltage: float) -> None:
        self.battery_v = voltage

    def set_joystick_log_callback(self, cb: Any) -> None:
        self.joystick_log_callback = cb

    def set_in_30cm_zone_callback(self, cb: Any) -> None:
        self.in_30cm_zone_callback = cb

    def on_nav2_goal_sent(self) -> None:
        self.last_nav2_goal_time = time.time()
        self.last_pose = None
        self.last_pose_time = None

    def update_pose(self, x: float, y: float) -> None:
        self.last_pose = (x, y)
        self.last_pose_time = time.time()

    def check(self) -> Dict[str, Any]:
        """Returns dict with keys: abort_reason, slow_down, nav2_stuck, battery_warning."""
        out = {"abort_reason": None, "slow_down": False, "nav2_stuck": False, "battery_warning": False}
        now = time.time()
        # Nav2 stuck: goal was sent but robot hasn't moved for >15s
        if self.last_nav2_goal_time is not None and self.last_pose_time is not None:
            if now - self.last_pose_time > self.stuck_timeout:
                out["nav2_stuck"] = True
                out["abort_reason"] = "Nav2 stuck, switching to teleop"
        if self.battery_v is not None and self.battery_v < self.battery_warning_v:
            out["battery_warning"] = True
            out["slow_down"] = True
        return out

    def log_joystick_if_outside_30cm(self, y: int, x: int, z: int) -> None:
        """Rulebook: log joystick movement when not in 30cm end-wall exception zone."""
        if self.in_30cm_zone_callback and self.in_30cm_zone_callback():
            return
        if self.joystick_log_callback and (abs(y) > 5 or abs(x) > 5 or abs(z) > 5):
            self.joystick_log_callback(y, x, z)


class AutonomyExecutor:
    """Main executor: ZMQ commands, Nav2 client, mode dispatch, status publish, safety watchdog."""

    def __init__(
        self,
        autonomy_port: int = ZMQ_PORT_AUTONOMY,
        slam_port: int = ZMQ_PORT_SLAM,
        status_port: int = ZMQ_PORT_AUTONOMY_STATUS,
        allow_near_target: bool = False,
    ):
        self.allow_near_target = bool(allow_near_target)
        self.state = AutonomyState.IDLE
        self.mode = "disabled"
        self.laps_completed = 0
        self.current_step = "0/1"
        self.step_label = "Idle"
        self.nav2_path: List[List[float]] = []
        self.nav2_goal: Optional[List[float]] = None
        self.goal_distance_m: Optional[float] = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.has_pose = False
        self.battery_voltage: Optional[float] = None

        self.watchdog = SafetyWatchdog()
        self._watchdog_timer_interval = 0.1  # 100 ms
        self._last_watchdog_time = 0.0

        # Stuck recovery state
        self._recovery_active = False
        self._recovery_phase = 0
        self._recovery_phase_start = 0.0
        self._recovery_attempt = 0
        self._recovery_saved_mode = ""
        self._recovery_saved_goal = None

        # SLAM grid for frontier exploration
        self._slam_grid = None
        self._slam_grid_resolution = SLAM_GRID_RESOLUTION
        self._slam_grid_origin_x = 0.0
        self._slam_grid_origin_y = 0.0

        # ZMQ
        self.ctx = zmq.Context()
        self.cmd_sock = self.ctx.socket(zmq.SUB)
        self.cmd_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.cmd_sock.setsockopt(zmq.RCVTIMEO, 100)
        self.cmd_sock.bind(f"tcp://*:{autonomy_port}")

        self.slam_sock = self.ctx.socket(zmq.SUB)
        self.slam_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.slam_sock.setsockopt(zmq.CONFLATE, 1)
        self.slam_sock.connect(f"tcp://127.0.0.1:{slam_port}")

        self.snapshot_result_sock = self.ctx.socket(zmq.SUB)
        self.snapshot_result_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.snapshot_result_sock.setsockopt(zmq.CONFLATE, 1)
        self.snapshot_result_sock.connect(f"tcp://127.0.0.1:{ZMQ_PORT_SNAPSHOT_RESULT}")

        self.robot_status_sock = self.ctx.socket(zmq.SUB)
        self.robot_status_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.robot_status_sock.setsockopt(zmq.CONFLATE, 1)
        self.robot_status_sock.connect(f"tcp://127.0.0.1:{ZMQ_PORT_STATUS}")

        self.status_sock = self.ctx.socket(zmq.PUB)
        self.status_sock.setsockopt(zmq.SNDHWM, 1)
        self.status_sock.bind(f"tcp://*:{status_port}")

        self.poller = zmq.Poller()
        self.poller.register(self.cmd_sock, zmq.POLLIN)
        self.poller.register(self.slam_sock, zmq.POLLIN)
        self.poller.register(self.snapshot_result_sock, zmq.POLLIN)
        self.poller.register(self.robot_status_sock, zmq.POLLIN)

        # ROS2 / Nav2 (optional)
        self.ros2_node = None
        self.nav2_client = None
        if ROS2_AVAILABLE:
            try:
                rclpy.init()
                self.ros2_node = Node("autonomy_executor")
                self.nav2_client = ActionClient(self.ros2_node, NavigateToPose, "navigate_to_pose")
            except Exception:
                self.ros2_node = None
                self.nav2_client = None

        self._abort_requested = False
        self._cancel_nav2_handle = None
        self._nav2_goal_handle = None
        # LapExecutor: A -> B -> A, count laps
        self._lap_waypoint_a: Optional[List[float]] = None
        self._lap_waypoint_b: Optional[List[float]] = None
        self._lap_phase = ""  # "to_b" | "to_a"
        self._pick_mode = "full_auto"  # full_auto | pause_teleop | full_teleop
        self._pick_container_goal: Optional[List[float]] = None

        # 30 cm zone: fiducials (from constants or file)
        self._fiducials: List[List[float]] = self._load_fiducials()

        # Drive PUB for stuck recovery (bypass Nav2, send directly to drive_bridge)
        self._drive_sock = self.ctx.socket(zmq.PUB)
        self._drive_sock.setsockopt(zmq.SNDHWM, 1)
        self._drive_sock.setsockopt(zmq.LINGER, 0)
        self._drive_sock.connect(f"tcp://127.0.0.1:{ZMQ_PORT_DRIVE}")

        # Arm presets (ZMQ PUSH to followerarm on 127.0.0.1)
        self._arm_sock = self.ctx.socket(zmq.PUSH)
        self._arm_sock.setsockopt(zmq.LINGER, 0)
        self._arm_sock.connect(f"tcp://127.0.0.1:{ZMQ_PORT_ARM_TELEOP}")
        # Snapshot request (vision pipeline can subscribe to trigger HAZMAT/Landolt capture)
        self._snapshot_sock = self.ctx.socket(zmq.PUSH)
        self._snapshot_sock.setsockopt(zmq.LINGER, 0)
        self._snapshot_sock.connect(f"tcp://127.0.0.1:{ZMQ_PORT_SNAPSHOT_REQUEST}")

        # Executor state machines (step index per mode)
        self._sensor_cabinet_step = 0
        self._keypad_step = 0
        self._inspect_tube_step = 0
        self._pick_step = 0
        self._explore_step = 0
        self._explore_waypoints: List[List[float]] = []
        self._explore_goal_sent = False
        self._pick_goal: Optional[List[float]] = None
        self._pick_goal_sent = False
        self._pick_arm_sent = False
        self._executor_delay_until = 0.0  # for step delays (arm movement, snapshot wait)
        self._latest_snapshot_results: Dict[str, Dict[str, Any]] = {}
        self._latest_gripper_status: Dict[str, Any] = {}
        self._keypad_pan_offset = 0.0
        self._keypad_lift_offset = 0.0
        self._keypad_adjust_count = 0
        self._pick_pan_offset = 0.0
        self._pick_lift_offset = 0.0
        self._pick_adjust_count = 0
        self._pick_close_retry_count = 0
        self._fallback_nav_goal: Optional[List[float]] = None
        self._fallback_follow_left = True

        print("[AUTONOMY] Executor started")
        print(f"  Commands: {autonomy_port}  SLAM: {slam_port}  Status: {status_port}")
        if self.nav2_client:
            print("  Nav2 action client: enabled")
        else:
            print("  Nav2: disabled (no ROS2/nav2_msgs)")
        if self._fiducials:
            print(f"  30cm zone: {len(self._fiducials)} fiducials (arena bounds)")

    def _load_fiducials(self) -> List[List[float]]:
        """Load fiducial positions from shared.constants or reports/fiducials.json."""
        try:
            from shared.constants import FIDUCIAL_POSITIONS as FP
            if FP and isinstance(FP, list):
                return [[float(p[0]), float(p[1])] for p in FP if len(p) >= 2]
        except Exception:
            pass
        path = os.path.join(os.path.dirname(__file__), "..", "reports", "fiducials.json")
        if os.path.isfile(path):
            try:
                with open(path, "r") as f:
                    data = json.load(f)
                if isinstance(data, list):
                    return [[float(p[0]), float(p[1])] for p in data if len(p) >= 2]
            except Exception:
                pass
        return []

    def _dexterity_guard_ok(self, target_xy, mode_name: str) -> bool:
        """Rulebook 2026-04-12: dexterity autonomy must start >=30 cm from target.

        Bypassed by --allow-near-target (bench testing). If no target waypoint
        was supplied or no SLAM pose is available yet, we allow the run
        because there is no object reference to enforce against.
        """
        if self.allow_near_target:
            return True
        if not target_xy or not self.has_pose:
            return True
        try:
            tx, ty = float(target_xy[0]), float(target_xy[1])
        except (TypeError, ValueError, IndexError):
            return True
        dist = math.sqrt((self.pose_x - tx) ** 2 + (self.pose_y - ty) ** 2)
        if dist < DEXTERITY_AUTONOMY_MIN_DIST_M:
            self.state = AutonomyState.ERROR
            self.step_label = (
                f"{mode_name}: too close to target ({dist*100:.1f} cm < "
                f"{DEXTERITY_AUTONOMY_MIN_DIST_M*100:.0f} cm). Drive back to claim 4x."
            )
            print(f"[AUTONOMY] {self.step_label}")
            return False
        return True

    def handle_command(self, msg: dict) -> None:
        # Mission-reset broadcast (rulebook 2.5): clear in-mission state and idle out.
        if is_mission_reset(msg):
            self._handle_mission_reset(msg)
            return
        cmd = msg.get("mode", msg.get("command", "disabled"))
        if cmd == "abort" or msg.get("cancel"):
            self._abort_requested = True
            self._cancel_nav2()
            self.state = AutonomyState.ABORTING
            self.mode = "disabled"
            # TTS only for Nav2-stuck case; manual abort is silent
            if msg.get("reason") == "nav2_stuck":
                _tts_speak("Nav2 stuck, switching to teleop")
            return
        if cmd == "resume" and self.state == AutonomyState.PAUSED and self.mode == "pick":
            self.state = AutonomyState.RUNNING
            self._pick_step = 2
            self._executor_delay_until = time.time() + 1.0
            self.step_label = "Linear Rail Pick (resumed)"
            return
        if self.state not in (AutonomyState.IDLE, AutonomyState.COMPLETED, AutonomyState.ERROR):
            return
        if cmd == "lap" or cmd == "mapping":
            # "mapping" = same as lap (e.g. joystick Mapping button); use current pose if no waypoints
            waypoint_a = msg.get("waypoint_a", [self.pose_x, self.pose_y])
            waypoint_b = msg.get("waypoint_b", [self.pose_x + 2.0, self.pose_y])
            self._start_lap(waypoint_a, waypoint_b)
        elif cmd == "sensor_cabinet":
            target = msg.get("target_xy")
            if not self._dexterity_guard_ok(target, "Sensor Cabinet"):
                return
            self.mode = "sensor_cabinet"
            self.state = AutonomyState.RUNNING
            self._sensor_cabinet_step = 0
            self.step_label = "Sensor Cabinet"
        elif cmd == "inspect_tube":
            target = msg.get("target_xy")
            if not self._dexterity_guard_ok(target, "Inspect tube"):
                return
            self.mode = "inspect_tube"
            self.state = AutonomyState.RUNNING
            self._inspect_tube_step = 0
            self.step_label = "Inspect tube"
        elif cmd == "keypad":
            target = msg.get("target_xy")
            if not self._dexterity_guard_ok(target, "Keypad"):
                return
            self.mode = "keypad"
            self.state = AutonomyState.RUNNING
            self._keypad_step = 0
            self.step_label = "Keypad"
        elif cmd == "pick":
            wp = msg.get("pick_goal")
            target = [float(wp[0]), float(wp[1])] if isinstance(wp, (list, tuple)) and len(wp) >= 2 else None
            if not self._dexterity_guard_ok(target, "Linear Rail Pick"):
                return
            self.mode = "pick"
            self._pick_mode = msg.get("pick_mode", "full_auto")
            self.state = AutonomyState.RUNNING
            self._pick_step = 0
            self._pick_goal_sent = False
            self._pick_goal = target
            container = msg.get("container_goal")
            if isinstance(container, (list, tuple)) and len(container) >= 2:
                self._pick_container_goal = [float(container[0]), float(container[1])]
            elif target:
                self._pick_container_goal = [target[0], target[1] + 0.6]
            else:
                self._pick_container_goal = None
            self.step_label = "Linear Rail Pick"
        elif cmd == "explore":
            self.mode = "explore"
            self.state = AutonomyState.RUNNING
            self.step_label = "Explore"
            self._explore_step = 0
            self._explore_goal_sent = False
            waypoints = msg.get("waypoints")
            if isinstance(waypoints, list) and len(waypoints) >= 2:
                self._explore_waypoints = [[float(w[0]), float(w[1])] for w in waypoints if len(w) >= 2]
            elif self._fiducials:
                self._explore_waypoints = list(self._fiducials)
            else:
                self._explore_waypoints = [[1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.0, 0.0]]
        elif cmd == "explore_labyrinth":
            self.mode = "explore_labyrinth"
            self.state = AutonomyState.RUNNING
            self.step_label = "Labyrinth — frontier exploration"
            self._frontier_goal_sent = False
            self._frontier_no_frontier_count = 0
            self._recovery_attempt = 0  # reset for new mission
        elif cmd == "disabled" or cmd == "reset":
            self._handle_mission_reset({"reason": "command"})

    def _handle_mission_reset(self, payload: dict) -> None:
        """Operator-initiated mini-mission reset (rulebook 2.5): clear all state."""
        self._abort_requested = True
        self._cancel_nav2()
        self.state = AutonomyState.IDLE
        self.mode = "disabled"
        self.laps_completed = 0
        self.current_step = "0/1"
        self.step_label = "Mission reset"
        self.nav2_path = []
        self.nav2_goal = None
        self.goal_distance_m = None
        self._lap_phase = ""
        self._lap_waypoint_a = None
        self._lap_waypoint_b = None
        self._sensor_cabinet_step = 0
        self._keypad_step = 0
        self._inspect_tube_step = 0
        self._pick_step = 0
        self._pick_goal = None
        self._pick_container_goal = None
        self._pick_goal_sent = False
        self._pick_arm_sent = False
        self._explore_step = 0
        self._explore_waypoints = []
        self._explore_goal_sent = False
        self._executor_delay_until = 0.0
        self._abort_requested = False
        reason = (payload or {}).get("reason", "")
        print(f"[AUTONOMY] Mission reset (reason={reason})")

    def _start_lap(self, waypoint_a: List[float], waypoint_b: List[float]) -> None:
        self.mode = "lap"
        self.state = AutonomyState.RUNNING
        self._lap_waypoint_a = list(waypoint_a)
        self._lap_waypoint_b = list(waypoint_b)
        self._lap_phase = "to_b"
        self.nav2_goal = waypoint_b
        self.nav2_path = [waypoint_b]
        self.step_label = "Navigate to B"
        self.current_step = "1/2"
        self.watchdog.on_nav2_goal_sent()
        self._send_nav2_goal(waypoint_b[0], waypoint_b[1])

    def _send_nav2_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        if not ROS2_AVAILABLE or not self.ros2_node or not self.nav2_client:
            self._fallback_nav_goal = [float(x), float(y)]
            return True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.ros2_node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        c = math.cos(yaw / 2)
        s = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = s
        goal_msg.pose.pose.orientation.w = c
        future = self.nav2_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_nav2_goal_response)
        self.watchdog.on_nav2_goal_sent()
        return True

    def _on_nav2_goal_response(self, future) -> None:
        """Callback when Nav2 accepts/rejects the goal."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                print("[AUTONOMY] Nav2 goal REJECTED")
                return
            self._nav2_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_nav2_result)
        except Exception as e:
            print(f"[AUTONOMY] Nav2 goal response error: {e}")

    def _on_nav2_result(self, future) -> None:
        """Callback when Nav2 goal reaches terminal state (succeeded/aborted/canceled)."""
        try:
            result = future.result()
            status = result.status
            # 4 = SUCCEEDED in action_msgs
            if status == 4:
                print("[AUTONOMY] Nav2 goal SUCCEEDED")
                # Force lap phase check immediately
                self._update_lap_phase()
            else:
                print(f"[AUTONOMY] Nav2 goal ended with status {status}")
        except Exception as e:
            print(f"[AUTONOMY] Nav2 result callback error: {e}")
        self._nav2_goal_handle = None

    def _cancel_nav2(self) -> None:
        if self._nav2_goal_handle:
            try:
                self._nav2_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._nav2_goal_handle = None
        self.nav2_goal = None
        self.nav2_path = []
        self._fallback_nav_goal = None
        self._send_drive_cmd(0.0, 0.0)

    def handle_slam(self, msg: dict) -> None:
        if "x" in msg:
            self.pose_x = float(msg.get("x", 0))
            self.pose_y = float(msg.get("y", 0))
            self.pose_yaw = float(msg.get("yaw", 0))
            self.has_pose = True
            self.watchdog.update_pose(self.pose_x, self.pose_y)
            self._update_lap_phase()
        if "path" in msg:
            self.nav2_path = msg.get("path", [])[-20:]
        # Store grid for frontier exploration
        if "grid" in msg:
            self._slam_grid = msg["grid"]
            self._slam_grid_resolution = float(msg.get("resolution", SLAM_GRID_RESOLUTION))
            self._slam_grid_origin_x = float(msg.get("origin_x", 0))
            self._slam_grid_origin_y = float(msg.get("origin_y", 0))

    def _update_lap_phase(self) -> None:
        """LapExecutor: when close to goal, switch to next (B->A or A->B) or increment lap."""
        if self.mode != "lap" or not self._lap_waypoint_a or not self._lap_waypoint_b:
            return
        arr_thresh = 0.25
        ax, ay = self._lap_waypoint_a[0], self._lap_waypoint_a[1]
        bx, by = self._lap_waypoint_b[0], self._lap_waypoint_b[1]
        dist_a = math.sqrt((self.pose_x - ax) ** 2 + (self.pose_y - ay) ** 2)
        dist_b = math.sqrt((self.pose_x - bx) ** 2 + (self.pose_y - by) ** 2)
        if self._lap_phase == "to_b" and dist_b < arr_thresh:
            self._lap_phase = "to_a"
            self.nav2_goal = self._lap_waypoint_a
            self.step_label = "Navigate to A"
            self.current_step = "2/2"
            self.watchdog.on_nav2_goal_sent()
            self._send_nav2_goal(ax, ay)
        elif self._lap_phase == "to_a" and dist_a < arr_thresh:
            self.laps_completed += 1
            self._lap_phase = "to_b"
            self.nav2_goal = self._lap_waypoint_b
            self.step_label = "Navigate to B"
            self.watchdog.on_nav2_goal_sent()
            self._send_nav2_goal(bx, by)

    def _send_arm_preset(self, name: str) -> bool:
        try:
            jetson_dir = os.path.dirname(os.path.abspath(__file__))
            if jetson_dir not in sys.path:
                sys.path.insert(0, jetson_dir)
            from arm_presets import send_preset
            return send_preset(name, self._arm_sock)
        except Exception:
            return False

    def _send_arm_override(
        self,
        base_preset: str,
        shoulder_pan_delta: float = 0.0,
        shoulder_lift_delta: float = 0.0,
        gripper: Optional[float] = None,
    ) -> bool:
        try:
            jetson_dir = os.path.dirname(os.path.abspath(__file__))
            if jetson_dir not in sys.path:
                sys.path.insert(0, jetson_dir)
            from arm_presets import send_preset, load_presets_from_yaml
            config_path = os.path.join(jetson_dir, "config", "robot_params.yaml")
            presets = load_presets_from_yaml(config_path)
            base = list(presets.get(base_preset, presets.get("pick_ready", [0.0, -0.2, 0.5, 0.3, 0.0])))
            while len(base) < 5:
                base.append(0.0)
            base[0] = float(base[0]) + float(shoulder_pan_delta)
            base[1] = float(base[1]) + float(shoulder_lift_delta)
            if gripper is not None:
                base[4] = float(gripper)
            presets["__override__"] = base
            return send_preset("__override__", self._arm_sock, presets=presets)
        except Exception:
            return False

    def _store_snapshot_result(self, meta: Dict[str, Any]) -> None:
        result_type = str(meta.get("result_type", "")).strip()
        if not result_type:
            return
        m = dict(meta)
        m["_rx_time"] = time.time()
        self._latest_snapshot_results[result_type] = m

    def _latest_snapshot(self, result_type: str, max_age_s: float = 1.5) -> Optional[Dict[str, Any]]:
        data = self._latest_snapshot_results.get(result_type)
        if not data:
            return None
        if time.time() - float(data.get("_rx_time", 0.0)) > max_age_s:
            return None
        return data

    def _best_detection_center(self, snapshot_meta: Optional[Dict[str, Any]]) -> Optional[Tuple[float, float]]:
        if not snapshot_meta:
            return None
        detections = snapshot_meta.get("detections", [])
        if not isinstance(detections, list) or not detections:
            return None
        best = max(detections, key=lambda d: float(d.get("confidence", 0.0)))
        bbox = best.get("bbox", [])
        if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
            return (0.5 * (float(bbox[0]) + float(bbox[2])), 0.5 * (float(bbox[1]) + float(bbox[3])))
        center = best.get("center", [])
        if isinstance(center, (list, tuple)) and len(center) >= 2:
            return (float(center[0]), float(center[1]))
        return None

    def _tick_executors(self) -> None:
        """Advance state machines for non-lap modes."""
        if self._recovery_active:
            self._tick_recovery()
            return
        if self._abort_requested or self.state in (AutonomyState.ABORTING, AutonomyState.IDLE):
            return
        if self.mode == "sensor_cabinet":
            self._tick_sensor_cabinet()
        elif self.mode == "keypad":
            self._tick_keypad()
        elif self.mode == "inspect_tube":
            self._tick_inspect_tube()
        elif self.mode == "pick":
            self._tick_pick()
        elif self.mode == "explore":
            self._tick_explore()
        elif self.mode == "explore_labyrinth":
            self._tick_explore_labyrinth()

    def _request_snapshot(self, result_type: str = "hazmat") -> None:
        """Ask vision pipeline to capture and send snapshot_result (vision subscribes on SNAPSHOT_REQUEST port)."""
        try:
            self._snapshot_sock.send_json({"request": "snapshot", "result_type": result_type}, zmq.NOBLOCK)
        except (zmq.Again, zmq.ZMQError):
            pass

    def _tick_sensor_cabinet(self) -> None:
        if self._sensor_cabinet_step == 0:
            self._send_arm_preset("cabinet_view")
            self.step_label = "Sensor Cabinet — viewing"
            self._executor_delay_until = time.time() + 1.0
            self._sensor_cabinet_step = 1
            self._request_snapshot("hazmat")
        elif self._sensor_cabinet_step == 1 and time.time() >= self._executor_delay_until:
            self.step_label = "Sensor Cabinet — complete"
            self.state = AutonomyState.COMPLETED
            self.mode = "disabled"

    def _tick_keypad(self) -> None:
        """Vision-closed-loop keypad: view → detect → adjust arm → push."""
        now = time.time()
        if self._keypad_step == 0:
            self._send_arm_preset("cabinet_view")
            self.step_label = "Keypad — viewing"
            self._executor_delay_until = now + 1.5
            self._keypad_step = 1
            self._keypad_adjust_count = 0
            self._keypad_pan_offset = 0.0
            self._keypad_lift_offset = 0.0
        elif self._keypad_step == 1 and now >= self._executor_delay_until:
            self._request_snapshot("keypad_detect")
            self.step_label = "Keypad — detecting"
            self._executor_delay_until = now + 0.4
            self._keypad_step = 2
        elif self._keypad_step == 2 and now >= self._executor_delay_until:
            snap = self._latest_snapshot("keypad_detect", max_age_s=2.0)
            center = self._best_detection_center(snap)
            if center is None:
                self._keypad_adjust_count += 1
                if self._keypad_adjust_count >= 12:
                    self.step_label = "Keypad — no detection, pushing"
                    self._keypad_step = 3
                else:
                    self._request_snapshot("keypad_detect")
                    self._executor_delay_until = now + 0.35
                return
            x_err = center[0] - 320.0
            y_err = center[1] - 240.0
            if abs(x_err) < 20.0 and abs(y_err) < 20.0:
                self.step_label = "Keypad — aligned"
                self._keypad_step = 3
                return
            self._keypad_pan_offset += -0.002 * x_err
            self._keypad_lift_offset += -0.002 * y_err
            self._send_arm_override(
                "cabinet_view",
                shoulder_pan_delta=self._keypad_pan_offset,
                shoulder_lift_delta=self._keypad_lift_offset,
            )
            self._keypad_adjust_count += 1
            if self._keypad_adjust_count >= 20:
                self.step_label = "Keypad — max adjust reached, pushing"
                self._keypad_step = 3
            else:
                self._request_snapshot("keypad_detect")
                self._executor_delay_until = now + 0.35
        elif self._keypad_step == 3:
            self._send_arm_preset("keypad_push")
            self.step_label = "Keypad — pushing"
            self._executor_delay_until = now + 2.0
            self._keypad_step = 4
        elif self._keypad_step == 4 and now >= self._executor_delay_until:
            # Rulebook safety: retract immediately, then back off so we do not
            # graze neighboring keys after a clean "5" press.
            self._send_arm_preset("stowed")
            self._send_drive_cmd(-0.10, 0.0)
            self.step_label = "Keypad — retreating"
            self._executor_delay_until = now + 0.8
            self._keypad_step = 5
        elif self._keypad_step == 5 and now >= self._executor_delay_until:
            self._send_drive_cmd(0.0, 0.0)
            self.step_label = "Keypad — complete"
            self.state = AutonomyState.COMPLETED
            self.mode = "disabled"

    def _tick_inspect_tube(self) -> None:
        """Move arm through inspect positions, take snapshot at each, stop early on high confidence."""
        presets = ["inspect_high", "inspect_mid", "inspect_low"]
        now = time.time()
        if not hasattr(self, '_inspect_best_conf'):
            self._inspect_best_conf = 0.0
        if self._inspect_tube_step < len(presets):
            if now >= self._executor_delay_until:
                self._send_arm_preset(presets[self._inspect_tube_step])
                self.step_label = f"Inspect tube — {presets[self._inspect_tube_step]}"
                self._executor_delay_until = now + 1.5
                self._inspect_tube_step += 1
                # Request snapshot at each position
                self._request_snapshot("hazmat")
        elif self._inspect_tube_step == len(presets) and now >= self._executor_delay_until:
            # Take final snapshot and complete
            self._request_snapshot("landolt")
            self._executor_delay_until = now + 1.0
            self._inspect_tube_step = len(presets) + 1
        elif self._inspect_tube_step > len(presets) and now >= self._executor_delay_until:
            self._send_arm_preset("stowed")
            self.step_label = "Inspect tube — complete"
            self.state = AutonomyState.COMPLETED
            self.mode = "disabled"
            self._inspect_best_conf = 0.0

    def _tick_pick(self) -> None:
        if self._pick_mode == "full_teleop":
            self.step_label = "Linear Rail Pick (full teleop)"
            self.state = AutonomyState.COMPLETED
            self.mode = "disabled"
            return
        now = time.time()
        if self._pick_step == 0:
            if not self._pick_goal:
                self._pick_step = 1
            elif not self._pick_goal_sent:
                self.nav2_goal = self._pick_goal
                self._send_nav2_goal(self._pick_goal[0], self._pick_goal[1])
                self._pick_goal_sent = True
                self.step_label = "Driving to pick area"
            else:
                dist = math.sqrt((self.pose_x - self._pick_goal[0]) ** 2 + (self.pose_y - self._pick_goal[1]) ** 2)
                if dist < 0.25:
                    self._pick_step = 1
        if self._pick_step == 1:
            if not self._pick_arm_sent:
                self._send_arm_preset("pick_ready")
                self._pick_arm_sent = True
                self._pick_pan_offset = 0.0
                self._pick_lift_offset = 0.0
                self._pick_adjust_count = 0
                self._pick_close_retry_count = 0
            self.step_label = "Linear Rail Pick — arm ready"
            if self._pick_mode == "pause_teleop":
                self.state = AutonomyState.PAUSED
                self.step_label = "Paused for base teleop — press Resume"
                self._pick_step = 2
            else:
                self._executor_delay_until = now + 0.8
                self._pick_step = 2
        if self._pick_step == 2 and now >= self._executor_delay_until:
            self._request_snapshot("pick_detect")
            self.step_label = "Linear Rail Pick — detecting object"
            self._executor_delay_until = now + 0.4
            self._pick_step = 3
        if self._pick_step == 3 and now >= self._executor_delay_until:
            snap = self._latest_snapshot("pick_detect", max_age_s=2.0)
            center = self._best_detection_center(snap)
            if center is None:
                self._pick_adjust_count += 1
                if self._pick_adjust_count >= 12:
                    self._pick_step = 4
                else:
                    self._request_snapshot("pick_detect")
                    self._executor_delay_until = now + 0.35
                return
            x_err = center[0] - 320.0
            y_err = center[1] - 240.0
            if abs(x_err) < 20.0 and abs(y_err) < 20.0:
                self._pick_step = 4
            else:
                self._pick_pan_offset += -0.002 * x_err
                self._pick_lift_offset += -0.002 * y_err
                self._send_arm_override(
                    "pick_ready",
                    shoulder_pan_delta=self._pick_pan_offset,
                    shoulder_lift_delta=self._pick_lift_offset,
                )
                self._pick_adjust_count += 1
                if self._pick_adjust_count >= 20:
                    self._pick_step = 4
                else:
                    self._request_snapshot("pick_detect")
                    self._executor_delay_until = now + 0.35
        if self._pick_step == 4:
            self._send_arm_override(
                "pick_ready",
                shoulder_pan_delta=self._pick_pan_offset,
                shoulder_lift_delta=self._pick_lift_offset,
                gripper=1.0,
            )
            self.step_label = "Linear Rail Pick — closing gripper"
            self._executor_delay_until = now + 1.0
            self._pick_step = 5
        if self._pick_step == 5 and now >= self._executor_delay_until:
            if bool(self._latest_gripper_status.get("gripper_closed", False)):
                self._pick_step = 6
            elif self._pick_close_retry_count < 2:
                self._pick_close_retry_count += 1
                self._pick_step = 4
            else:
                self._pick_step = 6
        if self._pick_step == 6:
            if self._pick_container_goal:
                self.nav2_goal = self._pick_container_goal
                self._send_nav2_goal(self._pick_container_goal[0], self._pick_container_goal[1])
                self.step_label = "Linear Rail Pick — driving to container"
                self._pick_step = 7
            else:
                self._pick_step = 8
        if self._pick_step == 7 and self._pick_container_goal:
            dist = math.sqrt(
                (self.pose_x - self._pick_container_goal[0]) ** 2 +
                (self.pose_y - self._pick_container_goal[1]) ** 2
            )
            if dist < 0.30:
                self._pick_step = 8
        if self._pick_step == 8:
            self._send_arm_override(
                "pick_ready",
                shoulder_pan_delta=self._pick_pan_offset,
                shoulder_lift_delta=self._pick_lift_offset,
                gripper=0.0,
            )
            self.step_label = "Linear Rail Pick — dropping object"
            self._executor_delay_until = now + 1.0
            self._pick_step = 9
        if self._pick_step == 9 and now >= self._executor_delay_until:
            self._send_arm_preset("stowed")
            self.state = AutonomyState.COMPLETED
            self.mode = "disabled"

    def _tick_nav2_fallback(self) -> None:
        if self.state != AutonomyState.RUNNING:
            return
        if self.nav2_client is not None or not self._fallback_nav_goal:
            return
        gx, gy = self._fallback_nav_goal[0], self._fallback_nav_goal[1]
        dist = math.sqrt((self.pose_x - gx) ** 2 + (self.pose_y - gy) ** 2)
        self.goal_distance_m = dist
        if dist < 0.25:
            self._send_drive_cmd(0.0, 0.0)
            self._fallback_nav_goal = None
            return
        linear = 0.10
        angular = 0.0
        if wall_follow is not None and self._slam_grid is not None:
            pts_robot = self._grid_to_points_robot()
            if pts_robot is not None and len(pts_robot) > 0:
                linear, angular, _ = wall_follow(
                    pts_robot,
                    follow_left=self._fallback_follow_left,
                    target_dist_m=0.18,
                    speed_mps=0.12,
                    angular_max_rps=0.4,
                )
            elif straight_line_fallback is not None:
                linear, angular, _ = straight_line_fallback(0.10)
        heading = math.atan2(gy - self.pose_y, gx - self.pose_x)
        yaw_err = math.atan2(math.sin(heading - self.pose_yaw), math.cos(heading - self.pose_yaw))
        angular += max(-0.35, min(0.35, 0.8 * yaw_err))
        if abs(yaw_err) > 1.0:
            linear *= 0.5
        self._send_drive_cmd(linear, angular)

    def _grid_to_points_robot(self):
        grid = self._slam_grid
        if grid is None:
            return None
        try:
            import numpy as np
            g = np.array(grid, dtype=np.int16)
        except Exception:
            return None
        occ = np.argwhere(g >= 80)
        if occ.size == 0:
            return None
        res = float(self._slam_grid_resolution)
        ox = float(self._slam_grid_origin_x)
        oy = float(self._slam_grid_origin_y)
        rows, cols = g.shape
        pts = []
        c = math.cos(-self.pose_yaw)
        s = math.sin(-self.pose_yaw)
        for r, cc in occ:
            wx = ox + (float(cc) + 0.5) * res
            wy = oy + (float(rows - 1 - r) + 0.5) * res
            dx = wx - self.pose_x
            dy = wy - self.pose_y
            rx = c * dx - s * dy
            ry = s * dx + c * dy
            if 0.05 <= rx <= 1.5 and -0.8 <= ry <= 0.8:
                pts.append([rx, ry, 0.0])
        if not pts:
            return None
        try:
            import numpy as np
            return np.array(pts, dtype=np.float64)
        except Exception:
            return None

    def _tick_explore(self) -> None:
        if not self._explore_waypoints:
            self.state = AutonomyState.COMPLETED
            self.mode = "disabled"
            return
        if self._explore_step >= len(self._explore_waypoints):
            self.state = AutonomyState.COMPLETED
            self.mode = "disabled"
            return
        goal = self._explore_waypoints[self._explore_step]
        if not self._explore_goal_sent:
            self.nav2_goal = goal
            self.nav2_path = [goal]
            self._send_nav2_goal(goal[0], goal[1])
            self._explore_goal_sent = True
            self.step_label = f"Explore waypoint {self._explore_step + 1}/{len(self._explore_waypoints)}"
        dist = math.sqrt((self.pose_x - goal[0]) ** 2 + (self.pose_y - goal[1]) ** 2)
        if dist < 0.25:
            self._explore_step += 1
            self._explore_goal_sent = False

    # -----------------------------------------------------------------
    # Frontier exploration for labyrinth
    # -----------------------------------------------------------------

    def _find_nearest_frontier(self) -> Optional[List[float]]:
        """Find the nearest frontier cell (free cell adjacent to unknown) in the SLAM grid.

        Returns [world_x, world_y] or None if no frontiers remain.
        """
        grid = self._slam_grid
        if grid is None:
            return None
        try:
            import numpy as np
            g = np.array(grid, dtype=np.int8)
        except Exception:
            return None
        rows, cols = g.shape
        # Free cells (0) adjacent to unknown (-1)
        free_mask = (g == 0)
        unknown_mask = (g == -1)
        # Dilate unknown by 1 cell to find adjacency
        frontier_mask = np.zeros_like(free_mask)
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(np.roll(unknown_mask, dr, axis=0), dc, axis=1)
            frontier_mask |= (free_mask & shifted)
        # Ignore edge cells (roll wraps around)
        frontier_mask[0, :] = False
        frontier_mask[-1, :] = False
        frontier_mask[:, 0] = False
        frontier_mask[:, -1] = False

        fy, fx = np.where(frontier_mask)
        if len(fx) == 0:
            return None

        # Convert to world coordinates
        res = self._slam_grid_resolution
        ox, oy = self._slam_grid_origin_x, self._slam_grid_origin_y
        world_x = fx * res + ox + res / 2
        world_y = fy * res + oy + res / 2

        # Find nearest to robot
        dists = (world_x - self.pose_x) ** 2 + (world_y - self.pose_y) ** 2
        # Skip frontiers too close (already visited)
        far_enough = dists > (0.3 ** 2)
        if not np.any(far_enough):
            return None
        dists[~far_enough] = float('inf')
        idx = int(np.argmin(dists))
        return [float(world_x[idx]), float(world_y[idx])]

    def _tick_explore_labyrinth(self) -> None:
        """Frontier-based labyrinth exploration: find unknown areas, navigate to them."""
        if not self._frontier_goal_sent:
            frontier = self._find_nearest_frontier()
            if frontier is None:
                self._frontier_no_frontier_count += 1
                if self._frontier_no_frontier_count >= 10:  # ~5s of no frontiers
                    self.step_label = "Labyrinth — fully explored"
                    self.state = AutonomyState.COMPLETED
                    self.mode = "disabled"
                    print("[AUTONOMY] Labyrinth fully explored")
                    return
                return  # wait for more SLAM data
            self._frontier_no_frontier_count = 0
            self.nav2_goal = frontier
            self.nav2_path = [frontier]
            self._send_nav2_goal(frontier[0], frontier[1])
            self._frontier_goal_sent = True
            self.step_label = f"Labyrinth — frontier ({frontier[0]:.1f}, {frontier[1]:.1f})"
            print(f"[AUTONOMY] Frontier goal: ({frontier[0]:.2f}, {frontier[1]:.2f})")
        else:
            # Check if arrived
            if self.nav2_goal:
                dist = math.sqrt(
                    (self.pose_x - self.nav2_goal[0]) ** 2 +
                    (self.pose_y - self.nav2_goal[1]) ** 2
                )
                if dist < 0.3:
                    self._frontier_goal_sent = False  # find next frontier

    def _in_30cm_zone(self) -> bool:
        """Rulebook: joystick allowed within 30cm of end walls. Uses fiducials as arena bounds."""
        if len(self._fiducials) < 2:
            return False
        xs = [p[0] for p in self._fiducials]
        ys = [p[1] for p in self._fiducials]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        m = END_WALL_EXCEPTION_M
        return (
            self.pose_x <= x_min + m or self.pose_x >= x_max - m
            or self.pose_y <= y_min + m or self.pose_y >= y_max - m
        )

    def _send_drive_cmd(self, linear_mps: float, angular_rps: float) -> None:
        """Send drive command directly (bypass Nav2) for stuck recovery."""
        try:
            from shared.constants import NAV2_MAX_LINEAR_VEL_MPS
        except ImportError:
            NAV2_MAX_LINEAR_VEL_MPS = 0.3
        # Scale m/s -> -100..100 motor range
        y = int(max(-100, min(100, (linear_mps / NAV2_MAX_LINEAR_VEL_MPS) * 100)))
        cmd = {"msg_type": "drive_cmd", "timestamp": time.time(), "y": y, "x": 0,
               "z": int(max(-100, min(100, (angular_rps / 1.0) * 50))), "emergency_stop": False}
        try:
            self._drive_sock.send_json(cmd, zmq.NOBLOCK)
        except (zmq.Again, zmq.ZMQError):
            pass

    def _start_recovery(self) -> None:
        """Begin stuck recovery sequence."""
        if recover_from_stuck is None:
            # No recovery available, just abort
            self._abort_requested = True
            self._cancel_nav2()
            self.state = AutonomyState.ABORTING
            self.mode = "disabled"
            return
        self._cancel_nav2()
        self._recovery_active = True
        self._recovery_phase = 0
        self._recovery_phase_start = time.time()
        self._recovery_saved_mode = self.mode
        self._recovery_saved_goal = list(self.nav2_goal) if self.nav2_goal else None
        self.step_label = f"STUCK — recovery attempt {self._recovery_attempt + 1}/{MAX_RECOVERY_ATTEMPTS}"
        print(f"[AUTONOMY] {self.step_label}")

    def _tick_recovery(self) -> None:
        """Advance stuck recovery state machine."""
        if not self._recovery_active or recover_from_stuck is None:
            return
        elapsed = time.time() - self._recovery_phase_start
        linear, angular, next_phase = recover_from_stuck(
            self._recovery_phase, elapsed, self._recovery_attempt
        )
        if next_phase != self._recovery_phase:
            self._recovery_phase = next_phase
            self._recovery_phase_start = time.time()
        if next_phase >= 3:
            # Recovery done — re-send goal or abort
            self._send_drive_cmd(0, 0)  # stop
            self._recovery_active = False
            self._recovery_attempt += 1
            self.watchdog.on_nav2_goal_sent()  # reset stuck timer
            if self._recovery_saved_goal:
                self.nav2_goal = self._recovery_saved_goal
                self._send_nav2_goal(self._recovery_saved_goal[0], self._recovery_saved_goal[1])
                self.step_label = f"Retrying goal after recovery"
                print(f"[AUTONOMY] Recovery done, retrying Nav2 goal")
            else:
                self.step_label = "Recovery done (no goal to retry)"
            return
        self._send_drive_cmd(linear, angular)

    def run_watchdog(self) -> None:
        now = time.time()
        if now - self._last_watchdog_time < self._watchdog_timer_interval:
            return
        self._last_watchdog_time = now
        # Don't check stuck while recovering
        if self._recovery_active:
            return
        self.watchdog.set_in_30cm_zone_callback(self._in_30cm_zone)
        res = self.watchdog.check()
        if res["abort_reason"]:
            if self._recovery_attempt < MAX_RECOVERY_ATTEMPTS and recover_from_stuck is not None:
                self._start_recovery()
            else:
                self._abort_requested = True
                self._cancel_nav2()
                self.state = AutonomyState.ABORTING
                self.mode = "disabled"
                _tts_speak(res["abort_reason"])
        if res["battery_warning"]:
            self.step_label = "LOW BATTERY - slow down"

    def get_status(self) -> dict:
        wd = self.watchdog.check()
        return {
            "msg_type": "autonomy_status",
            "state": self.state.value,
            "mode": self.mode,
            "progress": 0.0,
            "message": self.step_label,
            "timestamp": time.time(),
            "nav2_path": self.nav2_path,
            "nav2_goal": self.nav2_goal,
            "goal_distance_m": self.goal_distance_m,
            "step": self.current_step,
            "step_label": self.step_label,
            "laps_completed": self.laps_completed,
            "battery_voltage": self.battery_voltage,
            "nav2_stuck": wd["nav2_stuck"],
        }

    def publish_status(self) -> None:
        try:
            self.status_sock.send_json(self.get_status(), zmq.NOBLOCK)
        except zmq.Again:
            pass

    def run(self) -> None:
        last_status = 0.0
        try:
            while True:
                if self.ros2_node:
                    rclpy.spin_once(self.ros2_node, timeout_sec=0.01)
                events = dict(self.poller.poll(100))
                if self.cmd_sock in events:
                    try:
                        raw = self.cmd_sock.recv(zmq.NOBLOCK)
                        data = json.loads(raw.decode("utf-8"))
                        self.handle_command(data)
                    except (zmq.Again, json.JSONDecodeError):
                        pass
                if self.slam_sock in events:
                    try:
                        msg = self.slam_sock.recv_json(zmq.NOBLOCK)
                        self.handle_slam(msg)
                    except (zmq.Again, json.JSONDecodeError):
                        pass
                if self.snapshot_result_sock in events:
                    try:
                        parts = self.snapshot_result_sock.recv_multipart(zmq.NOBLOCK)
                        if parts:
                            meta = json.loads(parts[0].decode("utf-8"))
                            if isinstance(meta, dict) and meta.get("msg_type") == "snapshot_result":
                                self._store_snapshot_result(meta)
                    except (zmq.Again, json.JSONDecodeError):
                        pass
                if self.robot_status_sock in events:
                    try:
                        status_msg = self.robot_status_sock.recv_json(zmq.NOBLOCK)
                        if isinstance(status_msg, dict) and status_msg.get("msg_type") == "gripper_status":
                            self._latest_gripper_status = dict(status_msg)
                    except (zmq.Again, json.JSONDecodeError):
                        pass
                self.run_watchdog()
                self._tick_nav2_fallback()
                self._tick_executors()
                if time.time() - last_status > 0.5:
                    self.publish_status()
                    last_status = time.time()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self._cancel_nav2()
        self._send_drive_cmd(0, 0)  # stop motors
        try:
            self._drive_sock.close()
            self._arm_sock.close()
            self._snapshot_sock.close()
            self.snapshot_result_sock.close()
            self.robot_status_sock.close()
        except Exception:
            pass
        self.cmd_sock.close()
        self.slam_sock.close()
        self.status_sock.close()
        self.ctx.term()
        if self.ros2_node:
            self.ros2_node.destroy_node()
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description="Autonomy executor (replaces autonomy_node)")
    ap.add_argument("--autonomy-port", type=int, default=ZMQ_PORT_AUTONOMY)
    ap.add_argument("--slam-port", type=int, default=ZMQ_PORT_SLAM)
    ap.add_argument("--status-port", type=int, default=ZMQ_PORT_AUTONOMY_STATUS)
    ap.add_argument(
        "--allow-near-target",
        action="store_true",
        help="Bench bypass for the rulebook 30 cm dexterity guard. Do NOT use at competition.",
    )
    args = ap.parse_args()
    if zmq is None:
        print("[AUTONOMY] ERROR: pyzmq required")
        return
    ex = AutonomyExecutor(
        autonomy_port=args.autonomy_port,
        slam_port=args.slam_port,
        status_port=args.status_port,
        allow_near_target=args.allow_near_target,
    )
    ex.run()


if __name__ == "__main__":
    main()
