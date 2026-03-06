#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Nav2 Bridge (Jetson)
===============================================
Bridges Point-LIO SLAM to Nav2: TF (map→odom→base_link), /map OccupancyGrid, /odom.
Dual output: ROS2 for Nav2 stack and ZMQ for laptop GUI (same grid/pose as slam_bridge).

Validation:
  ros2 topic echo /map --once
  ros2 run tf2_tools view_frames
  ros2 topic echo /odom --once
"""

import sys
import os
import time
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:
    import zmq
except ImportError:
    zmq = None

try:
    import numpy as np
except ImportError:
    np = None

# Reuse grid logic from slam_bridge (run from jetson/ or repo root)
try:
    from jetson.slam_bridge import DynamicOccupancyGrid, quaternion_to_yaw
except ImportError:
    try:
        from slam_bridge import DynamicOccupancyGrid, quaternion_to_yaw
    except ImportError:
        # Add jetson to path and retry
        _jet = os.path.join(os.path.dirname(__file__))
        if _jet not in sys.path:
            sys.path.insert(0, _jet)
        from slam_bridge import DynamicOccupancyGrid, quaternion_to_yaw

try:
    from shared.constants import (
        ZMQ_PORT_SLAM,
        ROS2_TOPIC_CLOUD_REGISTERED,
        ROS2_TOPIC_ODOMETRY,
        SLAM_GRID_RESOLUTION,
        SLAM_GRID_SIZE,
        SLAM_Z_MIN,
        SLAM_Z_MAX,
    )
except ImportError:
    ZMQ_PORT_SLAM = 5562
    ROS2_TOPIC_CLOUD_REGISTERED = "/cloud_registered"
    ROS2_TOPIC_ODOMETRY = "/Odometry"
    SLAM_GRID_RESOLUTION = 0.05
    SLAM_GRID_SIZE = 200
    SLAM_Z_MIN = -0.5
    SLAM_Z_MAX = 2.0

ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry, OccupancyGrid
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    import sensor_msgs_py.point_cloud2 as pc2
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    pass


def quat_to_yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class Nav2BridgeNode(Node):
    """
    Subscribes to Point-LIO /Odometry and /cloud_registered.
    - Publishes TF: map → odom (identity), odom → base_link (from Odometry).
    - Publishes /map as nav_msgs/OccupancyGrid (for Nav2).
    - Publishes /odom as nav_msgs/Odometry (for Nav2).
    - Optionally publishes same pose/grid on ZMQ for laptop (dual output).
    """

    def __init__(self, zmq_port: int = ZMQ_PORT_SLAM):
        if not ROS2_AVAILABLE:
            raise RuntimeError("rclpy/nav_msgs required for nav2_bridge")
        super().__init__("nav2_bridge")

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.pose_yaw = 0.0
        self.has_pose = False

        self.grid = DynamicOccupancyGrid(size=SLAM_GRID_SIZE, resolution=SLAM_GRID_RESOLUTION)

        # TF: map → odom → base_link (Point-LIO gives map-frame pose; we set odom = map, base_link from pose)
        self.tf_broadcaster = TransformBroadcaster(self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.odom_sub = self.create_subscription(
            Odometry, ROS2_TOPIC_ODOMETRY, self.odom_callback, sensor_qos
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, ROS2_TOPIC_CLOUD_REGISTERED, self.cloud_callback, sensor_qos
        )

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Publish map at 2 Hz, TF/odom at 20 Hz
        self.create_timer(0.05, self.publish_tf_and_odom)
        self.create_timer(0.5, self.publish_map_ros2)

        self.zmq_ctx = None
        self.slam_pub = None
        if zmq:
            self.zmq_ctx = zmq.Context()
            self.slam_pub = self.zmq_ctx.socket(zmq.PUB)
            self.slam_pub.setsockopt(zmq.CONFLATE, 1)
            self.slam_pub.setsockopt(zmq.SNDHWM, 1)
            self.slam_pub.bind(f"tcp://*:{zmq_port}")
        self.get_logger().info("Nav2 bridge started (TF + /map + /odom + ZMQ)")

    def odom_callback(self, msg: "Odometry"):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.pose_x = pos.x
        self.pose_y = pos.y
        self.pose_z = pos.z
        self.pose_yaw = quat_to_yaw(ori.x, ori.y, ori.z, ori.w)
        self.has_pose = True

    def cloud_callback(self, msg: "PointCloud2"):
        if not np or not self.has_pose:
            return
        try:
            points = np.array(
                [[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]
            )
            if len(points) > 0:
                self.grid.update_from_points(
                    points.astype(np.float32), self.pose_x, self.pose_y,
                    z_min=SLAM_Z_MIN, z_max=SLAM_Z_MAX
                )
        except Exception as e:
            self.get_logger().warn(f"Cloud error: {e}")

    def publish_tf_and_odom(self):
        if not self.has_pose:
            return
        now = self.get_clock().now().to_msg()
        # map → odom (identity; single frame from Point-LIO)
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = "map"
        t_map_odom.child_frame_id = "odom"
        t_map_odom.transform.translation.x = 0.0
        t_map_odom.transform.translation.y = 0.0
        t_map_odom.transform.translation.z = 0.0
        t_map_odom.transform.rotation.x = 0.0
        t_map_odom.transform.rotation.y = 0.0
        t_map_odom.transform.rotation.z = 0.0
        t_map_odom.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_map_odom)

        # odom → base_link (robot pose; Nav2 uses base_link)
        c, s = math.cos(self.pose_yaw), math.sin(self.pose_yaw)
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = now
        t_odom_base.header.frame_id = "odom"
        t_odom_base.child_frame_id = "base_link"
        t_odom_base.transform.translation.x = self.pose_x
        t_odom_base.transform.translation.y = self.pose_y
        t_odom_base.transform.translation.z = self.pose_z
        # Quaternion from yaw
        t_odom_base.transform.rotation.x = 0.0
        t_odom_base.transform.rotation.y = 0.0
        t_odom_base.transform.rotation.z = math.sin(self.pose_yaw / 2)
        t_odom_base.transform.rotation.w = math.cos(self.pose_yaw / 2)
        self.tf_broadcaster.sendTransform(t_odom_base)

        # /odom for Nav2
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.pose_x
        odom_msg.pose.pose.position.y = self.pose_y
        odom_msg.pose.pose.position.z = self.pose_z
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.pose_yaw / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.pose_yaw / 2)
        self.odom_pub.publish(odom_msg)

        # ZMQ (same as slam_bridge format for GUI)
        if self.slam_pub:
            path = [(self.pose_x, self.pose_y)]
            msg = {
                "msg_type": "slam_pose",
                "timestamp": time.time(),
                "x": self.pose_x, "y": self.pose_y, "z": self.pose_z, "yaw": self.pose_yaw,
                "robot_pose": {"x": self.pose_x, "y": self.pose_y, "z": self.pose_z, "yaw": self.pose_yaw},
                "arm_joints": [],
                "grid": self.grid.get_grid_list(),
                "resolution": self.grid.resolution,
                "origin_x": self.grid.origin_x, "origin_y": self.grid.origin_y,
                "path": path,
            }
            try:
                self.slam_pub.send_json(msg, zmq.NOBLOCK)
            except zmq.Again:
                pass

    def publish_map_ros2(self):
        if not self.has_pose:
            return
        grid_list = self.grid.get_grid_list()
        if not grid_list or not grid_list[0]:
            return
        rows, cols = len(grid_list), len(grid_list[0])
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.grid.resolution
        msg.info.width = cols
        msg.info.height = rows
        msg.info.origin.position.x = self.grid.origin_x
        msg.info.origin.position.y = self.grid.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        # Row-major: data[i*width + j] = grid[row][col]; row 0 = top (y_max)
        msg.data = []
        for i in range(rows - 1, -1, -1):
            msg.data.extend(grid_list[i])
        self.map_pub.publish(msg)

    def shutdown(self):
        if self.slam_pub:
            self.slam_pub.close()
        if self.zmq_ctx:
            self.zmq_ctx.term()


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Nav2 bridge: TF + /map + /odom + ZMQ")
    ap.add_argument("--zmq-port", type=int, default=ZMQ_PORT_SLAM)
    args = ap.parse_args()

    if not ROS2_AVAILABLE:
        print("[nav2_bridge] ROS2 (rclpy, nav_msgs, tf2_ros) required")
        return
    rclpy.init()
    node = Nav2BridgeNode(zmq_port=args.zmq_port)
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
