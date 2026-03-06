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
        ZMQ_PORT_AUTONOMY_STATUS,
        ZMQ_PORT_ARM_TELEOP,
        ZMQ_PORT_SNAPSHOT_REQUEST,
        NAV2_STUCK_TIMEOUT_S,
        BATTERY_WARNING_V,
        END_WALL_EXCEPTION_M,
        FIDUCIAL_POSITIONS,
    )
except ImportError:
    ZMQ_PORT_AUTONOMY = 5560
    ZMQ_PORT_SLAM = 5562
    ZMQ_PORT_AUTONOMY_STATUS = 5565
    ZMQ_PORT_ARM_TELEOP = 5558
    ZMQ_PORT_SNAPSHOT_REQUEST = 5570
    NAV2_STUCK_TIMEOUT_S = 15
    BATTERY_WARNING_V = 11.8
    END_WALL_EXCEPTION_M = 0.30
    FIDUCIAL_POSITIONS = []

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
    ):
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

        self.status_sock = self.ctx.socket(zmq.PUB)
        self.status_sock.setsockopt(zmq.SNDHWM, 1)
        self.status_sock.bind(f"tcp://*:{status_port}")

        self.poller = zmq.Poller()
        self.poller.register(self.cmd_sock, zmq.POLLIN)
        self.poller.register(self.slam_sock, zmq.POLLIN)

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

        # 30 cm zone: fiducials (from constants or file)
        self._fiducials: List[List[float]] = self._load_fiducials()

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

    def handle_command(self, msg: dict) -> None:
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
            self.mode = "sensor_cabinet"
            self.state = AutonomyState.RUNNING
            self._sensor_cabinet_step = 0
            self.step_label = "Sensor Cabinet"
        elif cmd == "inspect_tube":
            self.mode = "inspect_tube"
            self.state = AutonomyState.RUNNING
            self._inspect_tube_step = 0
            self.step_label = "Inspect tube"
        elif cmd == "keypad":
            self.mode = "keypad"
            self.state = AutonomyState.RUNNING
            self._keypad_step = 0
            self.step_label = "Keypad"
        elif cmd == "pick":
            self.mode = "pick"
            self._pick_mode = msg.get("pick_mode", "full_auto")
            self.state = AutonomyState.RUNNING
            self._pick_step = 0
            self._pick_goal_sent = False
            wp = msg.get("pick_goal")
            self._pick_goal = [float(wp[0]), float(wp[1])] if isinstance(wp, (list, tuple)) and len(wp) >= 2 else None
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
        elif cmd == "disabled" or cmd == "reset":
            self.state = AutonomyState.IDLE
            self.mode = "disabled"
            self.laps_completed = 0

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
            return False
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
        self.nav2_client.send_goal_async(goal_msg)
        self.watchdog.on_nav2_goal_sent()
        return True

    def _cancel_nav2(self) -> None:
        if self._nav2_goal_handle:
            try:
                self._nav2_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._nav2_goal_handle = None
        self.nav2_goal = None
        self.nav2_path = []

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

    def _tick_executors(self) -> None:
        """Advance state machines for non-lap modes."""
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
        if self._keypad_step == 0:
            self._send_arm_preset("keypad_push")
            self.step_label = "Keypad — pushing"
            self._executor_delay_until = time.time() + 2.0
            self._keypad_step = 1
        elif self._keypad_step == 1 and time.time() >= self._executor_delay_until:
            self.state = AutonomyState.COMPLETED
            self.mode = "disabled"

    def _tick_inspect_tube(self) -> None:
        presets = ["inspect_high", "inspect_mid", "inspect_low", "inspect_mid", "inspect_high"]
        now = time.time()
        if self._inspect_tube_step < len(presets):
            if now >= self._executor_delay_until:
                self._send_arm_preset(presets[self._inspect_tube_step])
                self.step_label = f"Inspect tube C{self._inspect_tube_step + 1}"
                self._executor_delay_until = now + 1.5
                self._inspect_tube_step += 1
        elif self._inspect_tube_step == len(presets) and now >= self._executor_delay_until:
            self.state = AutonomyState.COMPLETED
            self.mode = "disabled"

    def _tick_pick(self) -> None:
        if self._pick_mode == "full_teleop":
            self.step_label = "Linear Rail Pick (full teleop)"
            self.state = AutonomyState.COMPLETED
            self.mode = "disabled"
            return
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
            self.step_label = "Linear Rail Pick — arm ready"
            if self._pick_mode == "pause_teleop":
                self.state = AutonomyState.PAUSED
                self.step_label = "Paused for base teleop — press Resume"
                self._pick_step = 2
            else:
                self._executor_delay_until = time.time() + 2.0
                self._pick_step = 2
        if self._pick_step == 2 and time.time() >= self._executor_delay_until:
            self.state = AutonomyState.COMPLETED
            self.mode = "disabled"

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

    def run_watchdog(self) -> None:
        now = time.time()
        if now - self._last_watchdog_time < self._watchdog_timer_interval:
            return
        self._last_watchdog_time = now
        self.watchdog.set_in_30cm_zone_callback(self._in_30cm_zone)
        res = self.watchdog.check()
        if res["abort_reason"]:
            self._abort_requested = True
            self._cancel_nav2()
            self.state = AutonomyState.ABORTING
            self.mode = "disabled"
            _tts_speak(res["abort_reason"])  # e.g. "Nav2 stuck, switching to teleop"
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
                self.run_watchdog()
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
        try:
            self._arm_sock.close()
            self._snapshot_sock.close()
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
    args = ap.parse_args()
    if zmq is None:
        print("[AUTONOMY] ERROR: pyzmq required")
        return
    ex = AutonomyExecutor(
        autonomy_port=args.autonomy_port,
        slam_port=args.slam_port,
        status_port=args.status_port,
    )
    ex.run()


if __name__ == "__main__":
    main()
