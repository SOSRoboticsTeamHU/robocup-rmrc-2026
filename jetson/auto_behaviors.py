#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2025 - Autonomous mission behaviors
======================================================
Pure functions for wall-follow, end-wall detection, straight-line fallback,
align, and helpers. Used by AutoMissionNode. No ROS2 dependency.
"""

import math
from typing import List, Tuple, Optional
import numpy as np


# -----------------------------------------------------------------------------
# Config / constants (override from auto_mission_config.yaml)
# -----------------------------------------------------------------------------
DEFAULT_BAY_LENGTH_M = 4.0
DEFAULT_WALL_DISTANCE_TARGET_M = 0.15   # 10–20 cm from walls
DEFAULT_END_WALL_THRESHOLD_M = 0.30    # RMRC: stop when within 30 cm of end wall
DEFAULT_SPEED_NORMAL_MPS = 0.15
DEFAULT_SPEED_SLOW_MPS = 0.08         # incline/sand
DEFAULT_ANGULAR_MAX_RPS = 0.4
DEFAULT_STRAIGHT_SPEED_MPS = 0.12
DEFAULT_ALIGN_ANGULAR_RPS = 0.15
DEFAULT_ALIGN_DISTANCE_TOL_M = 0.05
DEFAULT_ALIGN_YAW_TOL_RAD = 0.03


def world_to_robot_frame(
    points: np.ndarray,
    robot_x: float, robot_y: float, robot_yaw: float
) -> np.ndarray:
    """Transform world (x,y,z) points to robot frame. points shape (N, 3)."""
    if points is None or len(points) == 0:
        return np.zeros((0, 3))
    c, s = math.cos(-robot_yaw), math.sin(-robot_yaw)
    R = np.array([[c, -s], [s, c]], dtype=np.float64)
    xy = points[:, :2] - np.array([robot_x, robot_y])
    xy_robot = (R @ xy.T).T
    return np.column_stack([xy_robot[:, 0], xy_robot[:, 1], points[:, 2]])


def min_distances_robot_frame(
    points_robot: np.ndarray,
    forward_range: Tuple[float, float] = (0.1, 2.0),
    side_range: Tuple[float, float] = (-1.0, 1.0),
    height_range: Tuple[float, float] = (-0.3, 1.0),
) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    """
    From points in robot frame, return (min_front, min_left, min_right).
    front = positive x; left = negative y; right = positive y.
    Only considers points within forward_range (x), side_range (y), height_range (z).
    """
    if points_robot is None or len(points_robot) == 0:
        return None, None, None
    x, y, z = points_robot[:, 0], points_robot[:, 1], points_robot[:, 2]
    mask = (
        (x >= forward_range[0]) & (x <= forward_range[1]) &
        (y >= side_range[0]) & (y <= side_range[1]) &
        (z >= height_range[0]) & (z <= height_range[1])
    )
    pts = points_robot[mask]
    if len(pts) == 0:
        return None, None, None
    min_front = float(np.min(pts[:, 0])) if len(pts) else None
    left_pts = pts[pts[:, 1] < 0]
    right_pts = pts[pts[:, 1] > 0]
    min_left = float(-np.min(left_pts[:, 1])) if len(left_pts) else None   # distance (positive)
    min_right = float(np.min(right_pts[:, 1])) if len(right_pts) else None
    return min_front, min_left, min_right


def wall_follow(
    points_robot: np.ndarray,
    follow_left: bool,
    target_dist_m: float,
    speed_mps: float,
    angular_max_rps: float,
    front_clear_min_m: float = 0.4,
) -> Tuple[float, float, str]:
    """
    Compute (linear_vel, angular_vel, debug_str) for wall-following.
    follow_left: True = keep wall on left (turn left when too far, right when too close).
    """
    min_front, min_left, min_right = min_distances_robot_frame(
        points_robot,
        forward_range=(0.1, 1.5),
        side_range=(-0.8, 0.8),
        height_range=(-0.3, 0.8),
    )
    linear = speed_mps
    angular = 0.0
    side_dist = min_left if follow_left else min_right
    other_side = min_right if follow_left else min_left

    if min_front is not None and min_front < front_clear_min_m:
        linear = max(0.0, speed_mps * 0.5)
        angular = angular_max_rps if follow_left else -angular_max_rps
        return linear, angular, "obstacle_ahead"

    if side_dist is None:
        # No wall on follow side: turn toward wall
        angular = angular_max_rps if follow_left else -angular_max_rps
        return linear, angular, "no_wall"

    err = side_dist - target_dist_m
    kp = 1.2
    angular = -kp * err if follow_left else kp * err
    angular = max(-angular_max_rps, min(angular_max_rps, angular))
    return linear, angular, "wall_follow"


def end_wall_detect_along_axis(
    robot_x: float, robot_y: float,
    bay_length_m: float,
    bay_axis: str,
    threshold_m: float,
) -> Tuple[bool, float, str]:
    """Same as end_wall_detect but uses robot_x or robot_y based on bay_axis."""
    along = robot_x if bay_axis == "x" else robot_y
    start_wall = 0.0
    end_wall = bay_length_m
    dist_start = abs(along - start_wall)
    dist_end = abs(along - end_wall)
    dist_to_nearest = min(dist_start, dist_end)
    at_end = dist_to_nearest <= threshold_m
    which = "start" if dist_start <= threshold_m else ("end" if dist_end <= threshold_m else "none")
    return at_end, dist_to_nearest, which


def straight_line_fallback(speed_mps: float) -> Tuple[float, float, str]:
    """Straight ahead when no walls detected."""
    return speed_mps, 0.0, "straight"


def align_to_end_wall(
    robot_yaw: float,
    target_yaw: float,
    distance_to_wall: float,
    align_angular_rps: float,
    yaw_tol_rad: float,
    dist_tol_m: float,
) -> Tuple[float, float, str, bool]:
    """
    Compute (linear_vel, angular_vel, debug_str, aligned).
    Target yaw = facing end wall (e.g. 0 or pi). When aligned and close enough, aligned=True.
    """
    yaw_err = math.atan2(
        math.sin(target_yaw - robot_yaw),
        math.cos(target_yaw - robot_yaw),
    )
    if abs(yaw_err) <= yaw_tol_rad and distance_to_wall <= dist_tol_m:
        return 0.0, 0.0, "aligned", True
    linear = 0.0
    angular = max(-align_angular_rps, min(align_angular_rps, yaw_err * 2.0))
    return linear, angular, "aligning", False


def junction_detect(
    points_robot: np.ndarray,
    left_open_m: float = 0.4,
    right_open_m: float = 0.4,
    front_range: float = 1.0,
) -> Tuple[bool, bool, bool]:
    """
    Detect maze junctions: (left_open, right_open, front_open).
    Open = no obstacle within given distance in that direction.
    """
    min_front, min_left, min_right = min_distances_robot_frame(
        points_robot,
        forward_range=(0.2, front_range),
        side_range=(-1.0, 1.0),
        height_range=(-0.3, 0.8),
    )
    front_open = min_front is None or min_front > 0.5
    left_open = min_left is None or min_left > left_open_m
    right_open = min_right is None or min_right > right_open_m
    return left_open, right_open, front_open


def stuck_detect(
    pose_history: List[Tuple[float, float]],
    min_dist_m: float = 0.08,
    window_samples: int = 50,
) -> bool:
    """
    True if robot moved less than min_dist_m over the last window_samples poses.
    pose_history: list of (x, y); assume last is newest. Typical: 10 Hz -> 50 = 5s.
    """
    if not pose_history or len(pose_history) < 5:
        return False
    recent = pose_history[-window_samples:] if len(pose_history) >= window_samples else pose_history
    x0, y0 = recent[0][0], recent[0][1]
    x1, y1 = recent[-1][0], recent[-1][1]
    dist = math.hypot(x1 - x0, y1 - y0)
    return dist < min_dist_m


def ema_filter(
    current: float,
    previous_ema: Optional[float],
    alpha: float = 0.3,
) -> float:
    """Exponential moving average. alpha in (0,1); smaller = smoother."""
    if previous_ema is None:
        return current
    return alpha * current + (1.0 - alpha) * previous_ema


def ema_pose(
    x: float, y: float, yaw: float,
    prev: Optional[Tuple[float, float, float]],
    alpha_xy: float = 0.25,
    alpha_yaw: float = 0.2,
) -> Tuple[float, float, float]:
    """EMA-smoothed pose (x, y, yaw)."""
    if prev is None:
        return x, y, yaw
    x_ema = ema_filter(x, prev[0], alpha_xy)
    y_ema = ema_filter(y, prev[1], alpha_xy)
    # yaw wrap
    yaw_prev = prev[2]
    yaw_diff = math.atan2(math.sin(yaw - yaw_prev), math.cos(yaw - yaw_prev))
    yaw_ema = yaw_prev + ema_filter(yaw_diff, 0.0, alpha_yaw)
    yaw_ema = math.atan2(math.sin(yaw_ema), math.cos(yaw_ema))
    return x_ema, y_ema, yaw_ema
