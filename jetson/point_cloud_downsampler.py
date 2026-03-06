#!/usr/bin/env python3
"""
Point-LIO Point Cloud Downsampling + Foxglove Bridge Compatibility
==================================================================
ROS2 node: subscribe /cloud_registered (sensor_msgs/PointCloud2), voxel/random
downsample to <10k points, publish /cloud_downsampled. Optional compression
via point_cloud_transport (draco/zstd) if available.

Config: voxel_leaf_size 0.05 m, rate throttle 5 Hz. Use /cloud_downsampled
for Foxglove 3D view to stay under ~20 MB message limit.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

# Optional: point_cloud_transport (draco/zstd) - if available, can publish to .../compressed
try:
    from point_cloud_interfaces.msg import CompressedPointCloud2
    _COMPRESSION_AVAILABLE = True
except ImportError:
    _COMPRESSION_AVAILABLE = False
    CompressedPointCloud2 = None


class PointCloudDownsamplerNode(Node):
    def __init__(self):
        super().__init__("point_cloud_downsampler")

        self.declare_parameter("voxel_leaf_size", 0.05)
        self.declare_parameter("max_points", 10000)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("input_topic", "/cloud_registered")
        self.declare_parameter("output_topic", "/cloud_downsampled")
        self.declare_parameter("use_random_fallback", True)

        self.voxel_leaf_size = self.get_parameter("voxel_leaf_size").value
        self.max_points = int(self.get_parameter("max_points").value)
        self.rate_hz = self.get_parameter("rate_hz").value
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.use_random_fallback = self.get_parameter("use_random_fallback").value

        self._min_interval = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.0
        self._last_pub_time = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._sub = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_callback, sensor_qos
        )
        self._pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.get_logger().info(
            f"Point cloud downsampler: {self.input_topic} -> {self.output_topic} "
            f"(voxel_leaf={self.voxel_leaf_size}m, max={self.max_points} pts, {self.rate_hz} Hz)"
        )

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """Voxel grid downsampling: one point per voxel (leaf_size)."""
        if len(points) == 0:
            return points
        leaf = float(self.voxel_leaf_size)
        indices = np.floor(points[:, :3] / leaf).astype(np.int32)
        keys = [tuple(idx) for idx in indices]
        seen = set()
        out = []
        for i, key in enumerate(keys):
            if key not in seen:
                seen.add(key)
                out.append(points[i])
        return np.array(out, dtype=np.float32) if out else np.empty((0, 3), dtype=np.float32)

    def _random_subsample(self, points: np.ndarray, n: int) -> np.ndarray:
        """Random subsample to at most n points."""
        if len(points) <= n:
            return points
        idx = np.random.choice(len(points), n, replace=False)
        return points[idx]

    def _downsample(self, points: np.ndarray) -> np.ndarray:
        """Voxel downsample then cap at max_points (random if over)."""
        out = self._voxel_downsample(points)
        if len(out) > self.max_points:
            out = self._random_subsample(out, self.max_points)
        return out

    def _create_pointcloud2(self, points: np.ndarray, header) -> PointCloud2:
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def _cloud_callback(self, msg: PointCloud2):
        now = self.get_clock().now().nanoseconds / 1e9
        if self._min_interval > 0 and (now - self._last_pub_time) < self._min_interval:
            return
        try:
            points = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([p[0], p[1], p[2]])
            if not points:
                return
            points = np.array(points, dtype=np.float32)
            downsampled = self._downsample(points)
            out_msg = self._create_pointcloud2(downsampled, msg.header)
            self._pub.publish(out_msg)
            self._last_pub_time = now
        except Exception as e:
            self.get_logger().warn(f"Downsample error: {e}")


def main():
    import sys
    rclpy.init(args=sys.argv[1:] if len(sys.argv) > 1 else None)
    node = PointCloudDownsamplerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
