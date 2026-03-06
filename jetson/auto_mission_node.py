#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2025 - Central Auto Mission Node (Jetson)
=============================================================
ROS2 Humble node: lap-based terrain tests, align, keypad, linear rail, sensor cabinet,
finals maze. Subscribes to /Odometry and /cloud_registered; publishes /cmd_vel;
arm control via ZMQ (followerarm); state machine; 30 cm zone detection; ZMQ status to GUI.
"""

import sys
import os
import time
import math
import json
import argparse
import logging
from enum import Enum
from typing import Optional, List, Tuple, Any
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:
    import zmq
except ImportError:
    zmq = None

try:
    from shared.constants import (
        ZMQ_PORT_AUTONOMY,
        ZMQ_PORT_AUTONOMY_STATUS,
        ZMQ_PORT_ARM_TELEOP,
        ROS2_TOPIC_ODOMETRY,
        ROS2_TOPIC_CLOUD_REGISTERED,
        END_WALL_EXCEPTION_M,
    )
except ImportError:
    ZMQ_PORT_AUTONOMY = 5560
    ZMQ_PORT_AUTONOMY_STATUS = 5565
    ZMQ_PORT_ARM_TELEOP = 5558
    ROS2_TOPIC_ODOMETRY = "/Odometry"
    ROS2_TOPIC_CLOUD_REGISTERED = "/cloud_registered"
    END_WALL_EXCEPTION_M = 0.30

# Auto behaviors (no ROS2)
from auto_behaviors import (
    world_to_robot_frame,
    min_distances_robot_frame,
    wall_follow,
    end_wall_detect_along_axis,
    straight_line_fallback,
    align_to_end_wall,
    stuck_detect,
    ema_pose,
    DEFAULT_BAY_LENGTH_M,
    DEFAULT_WALL_DISTANCE_TARGET_M,
    DEFAULT_END_WALL_THRESHOLD_M,
    DEFAULT_SPEED_NORMAL_MPS,
    DEFAULT_SPEED_SLOW_MPS,
    DEFAULT_ANGULAR_MAX_RPS,
    DEFAULT_STRAIGHT_SPEED_MPS,
    DEFAULT_ALIGN_ANGULAR_RPS,
    DEFAULT_ALIGN_DISTANCE_TOL_M,
    DEFAULT_ALIGN_YAW_TOL_RAD,
)

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Twist


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AutoState(str, Enum):
    IDLE = "idle"
    STOWED_WAIT = "stowed_wait"
    AUTONOMOUS_LAP = "autonomous_lap"
    ALIGN = "align"
    KEYPAD = "keypad"
    LINEAR_RAIL = "linear_rail"
    SENSOR_CABINET = "sensor_cabinet"
    FINALS_MAZE = "finals_maze"
    PAUSED = "paused"
    ABORT = "abort"
    COMPLETED = "completed"


# Lap-based test types (use wall-follow + end detect)
LAP_TEST_TYPES = {
    "incline_horiz", "incline_inclined", "sand_gravel", "ramps_continuous",
    "ramps_pinwheel", "elevated_ramps", "krails_horiz", "krails_crossover",
    "hurdles_single", "hurdles_double", "stairs", "labyrinth_flat", "labyrinth_krails",
}
# Terrain-specific speed (slower on incline/sand)
SLOW_TERRAINS = {"incline_horiz", "incline_inclined", "sand_gravel", "stairs"}


class AutoMissionNode(Node):
    """
    Central autonomy node: state machine, /Odometry + /cloud_registered -> /cmd_vel,
    arm via ZMQ, 30 cm zone -> ZMQ status for GUI.
    """

    def __init__(
        self,
        bay_length_m: float = DEFAULT_BAY_LENGTH_M,
        wall_distance_m: float = DEFAULT_WALL_DISTANCE_TARGET_M,
        end_threshold_m: float = None,
        mission_timeout_s: float = 300.0,
        control_hz: float = 20.0,
        bay_axis: str = "x",
        zmq_autonomy_port: int = ZMQ_PORT_AUTONOMY,
        zmq_status_port: int = ZMQ_PORT_AUTONOMY_STATUS,
    ):
        super().__init__("auto_mission_node")
        self.bay_length_m = bay_length_m
        self.wall_distance_m = wall_distance_m
        self.end_threshold_m = end_threshold_m or END_WALL_EXCEPTION_M
        self.mission_timeout_s = mission_timeout_s
        self.control_hz = control_hz
        self.bay_axis = bay_axis

        # State
        self.state = AutoState.IDLE
        self.test_type = "krails_horiz"
        self.lap_count = 0
        self.lap_direction_left = True   # follow left wall = forward, then switch
        self.in_end_zone_30cm = False
        self.autonomy_active = False
        self.stowed_done = False
        self.mission_start_time: Optional[float] = None
        self.last_lap_time: Optional[float] = None
        self._end_wall_cooldown_until = 0.0  # avoid double-counting laps

        # Pose (raw from ROS)
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        self._has_odom = False
        self._cloud_points: Optional[np.ndarray] = None
        self._cloud_stamp = 0.0
        self._ema_pose: Optional[Tuple[float, float, float]] = None

        # Pose history for stuck detection and lap logic
        self._pose_history: List[Tuple[float, float]] = []
        self._max_pose_history = 100

        # Align state
        self._align_target_yaw = 0.0
        self._align_done = False

        # Logging for judge evidence
        self._log = []
        self._log_max = 500

        # ROS2
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self._odom_sub = self.create_subscription(
            Odometry, ROS2_TOPIC_ODOMETRY, self._odom_cb, sensor_qos
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2, ROS2_TOPIC_CLOUD_REGISTERED, self._cloud_cb, sensor_qos
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._control_timer = self.create_timer(1.0 / control_hz, self._control_loop)

        # ZMQ: commands from GUI (PUSH from laptop -> we SUB)
        self._zmq_ctx = zmq.Context() if zmq else None
        self._cmd_sock = None
        self._status_sock = None
        self._arm_sock = None
        if self._zmq_ctx:
            self._cmd_sock = self._zmq_ctx.socket(zmq.PULL)
            self._cmd_sock.setsockopt(zmq.RCVTIMEO, 100)
            self._cmd_sock.bind(f"tcp://*:{zmq_autonomy_port}")
            self._status_sock = self._zmq_ctx.socket(zmq.PUB)
            self._status_sock.setsockopt(zmq.SNDHWM, 1)
            self._status_sock.bind(f"tcp://*:{zmq_status_port}")
            self._arm_sock = self._zmq_ctx.socket(zmq.PUSH)
            self._arm_sock.setsockopt(zmq.LINGER, 0)
            self._arm_sock.connect(f"tcp://127.0.0.1:{ZMQ_PORT_ARM_TELEOP}")

        self.get_logger().info(
            f"AutoMissionNode started: bay={bay_length_m}m end_thr={self.end_threshold_m}m "
            f"cmd_vel pub, ZMQ cmd={zmq_autonomy_port} status={zmq_status_port}"
        )

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._pose_x = p.x
        self._pose_y = p.y
        self._pose_yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)
        self._has_odom = True
        self._ema_pose = ema_pose(
            self._pose_x, self._pose_y, self._pose_yaw,
            self._ema_pose, alpha_xy=0.25, alpha_yaw=0.2,
        )
        self._pose_history.append((self._ema_pose[0], self._ema_pose[1]))
        if len(self._pose_history) > self._max_pose_history:
            self._pose_history.pop(0)

    def _cloud_cb(self, msg: PointCloud2):
        try:
            pts = [[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]
            if pts:
                self._cloud_points = np.array(pts, dtype=np.float64)
                self._cloud_stamp = time.time()
        except Exception as e:
            self.get_logger().warn(f"Cloud decode: {e}")

    def _in_30cm_zone(self) -> bool:
        if not self._has_odom:
            return False
        x, y = self._ema_pose[0], self._ema_pose[1]
        along = x if self.bay_axis == "x" else y
        dist_start = abs(along - 0.0)
        dist_end = abs(along - self.bay_length_m)
        self.in_end_zone_30cm = min(dist_start, dist_end) <= self.end_threshold_m
        return self.in_end_zone_30cm

    def _publish_cmd_vel(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular)
        self._cmd_vel_pub.publish(msg)

    def _stop(self):
        self._publish_cmd_vel(0.0, 0.0)

    def _send_arm_preset(self, name: str) -> bool:
        if not self._arm_sock:
            return False
        try:
            from arm_presets import send_preset
            return send_preset(name, self._arm_sock)
        except Exception as e:
            self.get_logger().warn(f"Arm preset {name}: {e}")
            return False

    def _publish_status(self):
        if not self._status_sock:
            return
        self._in_30cm_zone()
        msg = {
            "msg_type": "auto_mission_status",
            "timestamp": time.time(),
            "state": self.state.value,
            "test_type": self.test_type,
            "lap_count": self.lap_count,
            "laps_completed": self.lap_count,
            "in_end_zone_30cm": self.in_end_zone_30cm,
            "autonomy_active": self.autonomy_active,
            "stowed_done": self.stowed_done,
            "has_odom": self._has_odom,
            "message": self._status_message(),
            "step": "lap" if self.state == AutoState.AUTONOMOUS_LAP else self.state.value,
            "step_label": self._status_message(),
            "progress": 100 if self.state == AutoState.COMPLETED else (min(99, self.lap_count * 25)),
        }
        try:
            self._status_sock.send_json(msg, zmq.NOBLOCK)
        except zmq.Again:
            pass

    def _status_message(self) -> str:
        if self.state == AutoState.IDLE:
            return "Idle — Start Auto Lap or select test"
        if self.state == AutoState.STOWED_WAIT:
            return "Stowed posture requested — confirm then start lap"
        if self.state == AutoState.AUTONOMOUS_LAP:
            return f"Lap {self.lap_count} | 30cm zone: {'YES' if self.in_end_zone_30cm else 'NO'}"
        if self.state == AutoState.ALIGN:
            return "Aligning to end wall"
        return self.state.value

    def _process_zmq_commands(self):
        if not self._cmd_sock:
            return
        try:
            raw = self._cmd_sock.recv(zmq.NOBLOCK)
            data = json.loads(raw.decode("utf-8"))
            cmd = data.get("cmd") or data.get("mode")
            if cmd == "start_auto_lap" or cmd == "lap":
                self._log_ev("GUI: start_auto_lap")
                if self.state == AutoState.IDLE or self.state == AutoState.STOWED_WAIT:
                    self.autonomy_active = True
                    self.state = AutoState.AUTONOMOUS_LAP
                    self.mission_start_time = time.time()
                    self.lap_count = 0
                    self.last_lap_time = None
                    self.get_logger().info("AUTONOMOUS LAP STARTED — hands off (except in 30cm zone)")
                    print("[AUTO] AUTONOMOUS LAP STARTED — Announce to judge. Hands off except when any part within 30 cm of bay end walls.")
            elif cmd == "stowed_posture" or cmd == "stowed":
                self._log_ev("GUI: stowed_posture")
                ok = self._send_arm_preset("stowed")
                self.stowed_done = ok
                self.state = AutoState.STOWED_WAIT
                self.get_logger().info(f"Stowed posture sent: {ok}")
            elif cmd == "select_test" or "test_type" in data:
                tt = data.get("test_type") or data.get("test")
                if tt:
                    self.test_type = str(tt)
                    self._log_ev(f"GUI: select_test {self.test_type}")
                    self.get_logger().info(f"Test type: {self.test_type}")
            elif cmd == "abort" or cmd == "stop":
                self._log_ev("GUI: abort")
                self.state = AutoState.ABORT
                self.autonomy_active = False
                self._stop()
                self.get_logger().info("Abort — teleop allowed")
        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().warn(f"ZMQ cmd: {e}")

    def _log_ev(self, ev: str):
        self._log.append({"t": time.time(), "ev": ev})
        if len(self._log) > self._log_max:
            self._log.pop(0)

    def _control_loop(self):
        self._process_zmq_commands()

        # Timeout
        if self.mission_start_time and (time.time() - self.mission_start_time) > self.mission_timeout_s:
            self._log_ev("timeout_5min")
            self.state = AutoState.ABORT
            self.autonomy_active = False
            self._stop()
            self.get_logger().warn("Mission timeout 5 min — abort")
            self._publish_status()
            return

        if self.state == AutoState.ABORT or self.state == AutoState.IDLE:
            if self.state == AutoState.ABORT:
                self._stop()
            self._publish_status()
            return

        if self.state == AutoState.STOWED_WAIT:
            self._stop()
            self._publish_status()
            return

        if self.state == AutoState.AUTONOMOUS_LAP:
            self._run_lap_behavior()
        elif self.state == AutoState.ALIGN:
            self._run_align_behavior()
        else:
            self._stop()

        self._publish_status()

    def _run_lap_behavior(self):
        if not self._has_odom:
            self._stop()
            return
        x, y, yaw = self._ema_pose[0], self._ema_pose[1], self._ema_pose[2]
        at_end, dist_to_end, which_end = end_wall_detect_along_axis(
            x, y, self.bay_length_m, self.bay_axis, self.end_threshold_m
        )
        if at_end:
            self._stop()
            now = time.time()
            if now >= self._end_wall_cooldown_until:
                self.lap_count += 1
                self._log_ev(f"lap_end wall={which_end} lap_count={self.lap_count}")
                self.last_lap_time = now
                self.lap_direction_left = not self.lap_direction_left
                self._end_wall_cooldown_until = now + 1.5
                self.get_logger().info(f"Lap count: {self.lap_count} — reverse direction")
            return
        if stuck_detect(self._pose_history, min_dist_m=0.08, window_samples=50):
            self._log_ev("stuck_stop")
            self._stop()
            self.get_logger().warn("Safety: stuck detected — stop")
            return
        speed = DEFAULT_SPEED_SLOW_MPS if self.test_type in SLOW_TERRAINS else DEFAULT_SPEED_NORMAL_MPS
        points_robot = None
        if self._cloud_points is not None:
            points_robot = world_to_robot_frame(self._cloud_points, x, y, yaw)
        min_f, min_l, min_r = None, None, None
        if points_robot is not None and len(points_robot) > 0:
            min_f, min_l, min_r = min_distances_robot_frame(
                points_robot,
                forward_range=(0.1, 1.5),
                side_range=(-0.8, 0.8),
                height_range=(-0.3, 0.8),
            )
        has_wall_left = min_l is not None and min_l < 1.0
        has_wall_right = min_r is not None and min_r < 1.0
        if has_wall_left or has_wall_right:
            linear, angular, _ = wall_follow(
                points_robot,
                follow_left=self.lap_direction_left,
                target_dist_m=self.wall_distance_m,
                speed_mps=speed,
                angular_max_rps=DEFAULT_ANGULAR_MAX_RPS,
            )
        else:
            linear, angular, _ = straight_line_fallback(speed)
        self._publish_cmd_vel(linear, angular)

    def _run_align_behavior(self):
        if not self._has_odom:
            self._stop()
            return
        x, y, yaw = self._ema_pose[0], self._ema_pose[1], self._ema_pose[2]
        _, dist_to_end, _ = end_wall_detect_along_axis(x, y, self.bay_length_m, self.bay_axis, 1.0)
        linear, angular, _, aligned = align_to_end_wall(
            yaw, self._align_target_yaw, dist_to_end,
            DEFAULT_ALIGN_ANGULAR_RPS,
            DEFAULT_ALIGN_YAW_TOL_RAD,
            DEFAULT_ALIGN_DISTANCE_TOL_M,
        )
        if aligned:
            self._stop()
            self.state = AutoState.COMPLETED
            self._log_ev("align_done")
            return
        self._publish_cmd_vel(linear, angular)

    def shutdown(self):
        self._stop()
        if self._cmd_sock:
            self._cmd_sock.close()
        if self._status_sock:
            self._status_sock.close()
        if self._arm_sock:
            self._arm_sock.close()
        if self._zmq_ctx:
            self._zmq_ctx.term()


def main():
    parser = argparse.ArgumentParser(description="Auto Mission Node — RMRC 2025 autonomy")
    parser.add_argument("--bay-length", type=float, default=DEFAULT_BAY_LENGTH_M, help="Bay length (m)")
    parser.add_argument("--wall-distance", type=float, default=DEFAULT_WALL_DISTANCE_TARGET_M, help="Target distance from wall (m)")
    parser.add_argument("--end-threshold", type=float, default=END_WALL_EXCEPTION_M, help="End wall stop threshold (m)")
    parser.add_argument("--timeout", type=float, default=300.0, help="Mission timeout (s)")
    parser.add_argument("--control-hz", type=float, default=20.0, help="Control loop Hz")
    parser.add_argument("--bay-axis", choices=["x", "y"], default="x", help="Bay longitudinal axis")
    parser.add_argument("--zmq-cmd", type=int, default=ZMQ_PORT_AUTONOMY, help="ZMQ port for commands")
    parser.add_argument("--zmq-status", type=int, default=ZMQ_PORT_AUTONOMY_STATUS, help="ZMQ port for status")
    args = parser.parse_args()

    rclpy.init()
    node = AutoMissionNode(
        bay_length_m=args.bay_length,
        wall_distance_m=args.wall_distance,
        end_threshold_m=args.end_threshold,
        mission_timeout_s=args.timeout,
        control_hz=args.control_hz,
        bay_axis=args.bay_axis,
        zmq_autonomy_port=args.zmq_cmd,
        zmq_status_port=args.zmq_status,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
