#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - SLAM Bridge (Jetson)
==============================================
Bridges Point-LIO ROS2 topics to ZMQ for GUI visualization.

Uses log-odds occupancy grid with decay for real-time updates.
Grid dynamically follows robot position.
"""

import sys
import os
import time
import math
import argparse
from typing import List, Tuple
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:
    import zmq
except ImportError:
    zmq = None

ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry, Path
    import sensor_msgs_py.point_cloud2 as pc2
    ROS2_AVAILABLE = True
except ImportError:
    pass

try:
    from shared.constants import (
        ZMQ_PORT_SLAM, ZMQ_PORT_CLOUD,
        ROS2_TOPIC_CLOUD_REGISTERED, ROS2_TOPIC_ODOMETRY,
        SLAM_GRID_RESOLUTION, SLAM_GRID_SIZE,
        SLAM_Z_MIN, SLAM_Z_MAX, SLAM_GRID_UPDATE_HZ
    )
except ImportError:
    ZMQ_PORT_SLAM = 5562
    ZMQ_PORT_CLOUD = 5564
    ROS2_TOPIC_CLOUD_REGISTERED = "/cloud_registered"
    ROS2_TOPIC_ODOMETRY = "/Odometry"
    SLAM_GRID_RESOLUTION = 0.05
    SLAM_GRID_SIZE = 200
    SLAM_Z_MIN = -0.5
    SLAM_Z_MAX = 2.0
    SLAM_GRID_UPDATE_HZ = 10


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class DynamicOccupancyGrid:
    """
    Log-odds occupancy grid with decay for real-time updates.
    Grid origin follows robot starting position.
    """
    
    def __init__(self, size: int = SLAM_GRID_SIZE, resolution: float = SLAM_GRID_RESOLUTION):
        self.size = size
        self.resolution = resolution
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_initialized = False
        
        # Log-odds grid
        self.log_odds = np.zeros((size, size), dtype=np.float32)
        
        # Parameters
        self.l_occ = 0.7      # Log-odds for occupied
        self.l_free = -0.4    # Log-odds for free
        self.l_max = 2.5
        self.l_min = -2.5
        self.decay = 0.995    # Decay per update
        
        self.grid = np.full((size, size), -1, dtype=np.int8)
        
    def set_origin_from_robot(self, robot_x: float, robot_y: float):
        if not self.origin_initialized:
            self.origin_x = robot_x - (self.size * self.resolution) / 2
            self.origin_y = robot_y - (self.size * self.resolution) / 2
            self.origin_initialized = True
            print(f"[SLAM] Grid origin: ({self.origin_x:.2f}, {self.origin_y:.2f})")
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy
    
    def is_valid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.size and 0 <= gy < self.size
    
    def update_from_points(self, points: np.ndarray, robot_x: float, robot_y: float,
                           z_min: float = SLAM_Z_MIN, z_max: float = SLAM_Z_MAX):
        if points is None or len(points) == 0:
            return
        
        self.set_origin_from_robot(robot_x, robot_y)
        
        # Decay existing observations
        self.log_odds *= self.decay
        
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        
        # Filter by height
        mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        filtered = points[mask]
        
        # Subsample for performance
        if len(filtered) > 1500:
            indices = np.random.choice(len(filtered), 1500, replace=False)
            filtered = filtered[indices]
        
        for point in filtered:
            gx, gy = self.world_to_grid(point[0], point[1])
            
            if not self.is_valid(gx, gy):
                continue
            
            # Mark occupied
            self.log_odds[gy, gx] = np.clip(
                self.log_odds[gy, gx] + self.l_occ, self.l_min, self.l_max)
            
            # Ray trace free
            self._ray_trace_free(robot_gx, robot_gy, gx, gy)
        
        self._update_grid_values()
    
    def _ray_trace_free(self, x0: int, y0: int, x1: int, y1: int):
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        
        while True:
            if (x, y) != (x1, y1) and self.is_valid(x, y):
                self.log_odds[y, x] = np.clip(
                    self.log_odds[y, x] + self.l_free, self.l_min, self.l_max)
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def _update_grid_values(self):
        unknown = np.abs(self.log_odds) < 0.25
        occupied = self.log_odds >= 0.25
        free = self.log_odds <= -0.25
        self.grid[unknown] = -1
        self.grid[occupied] = 100
        self.grid[free] = 0
    
    def get_grid_list(self) -> List[List[int]]:
        return self.grid.tolist()


class SLAMBridgeROS2(Node):
    def __init__(self, zmq_port: int = ZMQ_PORT_SLAM, cloud_port: int = ZMQ_PORT_CLOUD):
        super().__init__('slam_bridge')
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.pose_yaw = 0.0
        self.has_pose = False
        self.path_history = []
        self._latest_cloud = []  # downsampled XYZ for 3D viewer
        
        self.grid = DynamicOccupancyGrid()
        
        self.zmq_ctx = zmq.Context()
        self.slam_pub = self.zmq_ctx.socket(zmq.PUB)
        self.slam_pub.setsockopt(zmq.CONFLATE, 1)
        self.slam_pub.setsockopt(zmq.SNDHWM, 1)
        self.slam_pub.bind(f"tcp://*:{zmq_port}")
        
        # Separate PUB for point cloud data (3D viewer on laptop)
        self.cloud_pub = self.zmq_ctx.socket(zmq.PUB)
        self.cloud_pub.setsockopt(zmq.CONFLATE, 1)
        self.cloud_pub.setsockopt(zmq.SNDHWM, 1)
        self.cloud_pub.bind(f"tcp://*:{cloud_port}")
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.odom_sub = self.create_subscription(
            Odometry, ROS2_TOPIC_ODOMETRY, self.odom_callback, sensor_qos)
        self.cloud_sub = self.create_subscription(
            PointCloud2, ROS2_TOPIC_CLOUD_REGISTERED, self.cloud_callback, sensor_qos)
        
        # Publish at 25 Hz for lower SLAM latency on laptop (was SLAM_GRID_UPDATE_HZ)
        self.create_timer(0.04, self.publish_zmq)
        self.get_logger().info(f"SLAM Bridge started on port {zmq_port}")
    
    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.pose_x = pos.x
        self.pose_y = pos.y
        self.pose_z = pos.z
        self.pose_yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        self.has_pose = True
        
        self.path_history.append((self.pose_x, self.pose_y))
        if len(self.path_history) > 500:
            self.path_history = self.path_history[-300:]
    
    def cloud_callback(self, msg: PointCloud2):
        try:
            points = [[p[0], p[1], p[2]] for p in 
                      pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]
            if points:
                pts_arr = np.array(points, dtype=np.float32)
                self.grid.update_from_points(pts_arr, self.pose_x, self.pose_y)
                # Keep downsampled copy for 3D viewer (~3000 pts max)
                if len(pts_arr) > 3000:
                    idx = np.random.choice(len(pts_arr), 3000, replace=False)
                    pts_arr = pts_arr[idx]
                self._latest_cloud = pts_arr.tolist()
        except Exception as e:
            self.get_logger().warn(f"Cloud error: {e}")
    
    def publish_zmq(self):
        msg = {
            "msg_type": "slam_pose",
            "timestamp": time.time(),
            "x": self.pose_x,
            "y": self.pose_y,
            "z": self.pose_z,
            "yaw": self.pose_yaw,
            "robot_pose": {"x": self.pose_x, "y": self.pose_y, "z": self.pose_z, "yaw": self.pose_yaw},
            "arm_joints": [],  # Populated by arm node if available
            "grid": self.grid.get_grid_list(),
            "resolution": self.grid.resolution,
            "origin_x": self.grid.origin_x,
            "origin_y": self.grid.origin_y,
            "path": self.path_history[-100:]
        }
        try:
            self.slam_pub.send_json(msg, zmq.NOBLOCK)
        except zmq.Again:
            pass
        # Publish point cloud on separate port for 3D viewer
        if self._latest_cloud:
            cloud_msg = {
                "msg_type": "point_cloud",
                "timestamp": time.time(),
                "points": self._latest_cloud,
                "cloud_xyz": self._latest_cloud,  # Alias for GUI compatibility
            }
            try:
                self.cloud_pub.send_json(cloud_msg, zmq.NOBLOCK)
            except zmq.Again:
                pass
    
    def shutdown(self):
        self.slam_pub.close()
        self.cloud_pub.close()
        self.zmq_ctx.term()


def run_ros2_bridge(port: int):
    if not ROS2_AVAILABLE:
        print("[SLAM] ROS2 not available, using placeholder")
        run_placeholder(port)
        return
    
    rclpy.init()
    node = SLAMBridgeROS2(port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def run_placeholder(port: int, rate_hz: float = 20.0):
    if zmq is None:
        print("[SLAM] pyzmq required for placeholder mode")
        return
    
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.bind(f"tcp://*:{port}")
    
    size = 80
    grid = [[-1] * size for _ in range(size)]
    cx, cy = size // 2, size // 2
    for dy in range(-15, 16):
        for dx in range(-15, 16):
            if 0 <= cx+dx < size and 0 <= cy+dy < size:
                grid[cy+dy][cx+dx] = 0
    for y in (cy-18, cy+18):
        for x in range(cx-5, cx+6):
            if 0 <= x < size and 0 <= y < size:
                grid[y][x] = 100
    
    origin_x = -size * 0.05 / 2
    origin_y = -size * 0.05 / 2
    t_start = time.time()
    
    print(f"[SLAM] Placeholder on port {port}")
    
    try:
        while True:
            t = time.time() - t_start
            x = 0.5 * math.sin(t * 0.3)
            y = 0.5 * math.cos(t * 0.3)
            yaw = t * 0.2
            
            msg = {
                "msg_type": "slam_pose", "timestamp": time.time(),
                "x": x, "y": y, "z": 0.0, "yaw": yaw,
                "grid": grid, "resolution": 0.05,
                "origin_x": origin_x, "origin_y": origin_y,
                "path": [(x, y)]
            }
            sock.send_json(msg, zmq.NOBLOCK)
            time.sleep(1.0 / rate_hz)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        ctx.term()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=ZMQ_PORT_SLAM)
    ap.add_argument("--placeholder", action="store_true")
    ap.add_argument("--ros2", action="store_true")
    args = ap.parse_args()
    
    if args.ros2 or (ROS2_AVAILABLE and not args.placeholder):
        run_ros2_bridge(args.port)
    else:
        run_placeholder(args.port)


if __name__ == "__main__":
    main()
