#!/usr/bin/env python3
"""
Point Cloud Downsampler for Foxglove
====================================
Subscribes to /cloud_registered and publishes downsampled version
for Foxglove streaming (keeps data under bandwidth limits).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct


class CloudDownsampler(Node):
    def __init__(self):
        super().__init__('cloud_downsampler')
        
        # Parameters
        self.declare_parameter('voxel_size', 0.1)  # meters
        self.declare_parameter('max_points', 5000)
        self.declare_parameter('input_topic', '/cloud_registered')
        self.declare_parameter('output_topic', '/cloud_downsampled')
        
        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_points = self.get_parameter('max_points').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, sensor_qos)
        
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)
        
        self.get_logger().info(
            f"Downsampler: {input_topic} -> {output_topic} "
            f"(voxel={self.voxel_size}m, max={self.max_points} pts)")
    
    def voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """Simple voxel grid downsampling."""
        if len(points) == 0:
            return points
        
        # Quantize to voxel grid
        voxel_indices = np.floor(points[:, :3] / self.voxel_size).astype(np.int32)
        
        # Use dictionary to keep one point per voxel
        voxel_dict = {}
        for i, idx in enumerate(voxel_indices):
            key = tuple(idx)
            if key not in voxel_dict:
                voxel_dict[key] = points[i]
        
        result = np.array(list(voxel_dict.values()))
        
        # Further limit if still too many
        if len(result) > self.max_points:
            indices = np.random.choice(len(result), self.max_points, replace=False)
            result = result[indices]
        
        return result
    
    def cloud_callback(self, msg: PointCloud2):
        try:
            # Read points
            points = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([p[0], p[1], p[2]])
            
            if not points:
                return
            
            points = np.array(points, dtype=np.float32)
            
            # Downsample
            downsampled = self.voxel_downsample(points)
            
            # Create output message
            out_msg = self.create_pointcloud2(downsampled, msg.header)
            self.pub.publish(out_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Downsample error: {e}")
    
    def create_pointcloud2(self, points: np.ndarray, header) -> PointCloud2:
        """Create PointCloud2 message from numpy array."""
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Pack data
        msg.data = points.astype(np.float32).tobytes()
        
        return msg


def main():
    rclpy.init()
    node = CloudDownsampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
