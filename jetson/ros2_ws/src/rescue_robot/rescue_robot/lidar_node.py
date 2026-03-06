#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Lidar Node (ROS2 -> ZMQ Bridge)
=========================================================
Bridges Unitree L2 point cloud to ZMQ for 1st-person view in GUI.

Modes:
- ROS2 mode (default): Subscribe to /unilidar/cloud, convert to 2D scan
- Placeholder mode (--placeholder): Fixed 360-point scan for testing

Publishes (ZMQ):
- Port 5563: JSON with ranges/angles for 2D lidar visualization

The 1st-person view in the GUI uses this data to show lidar POV.
"""

import sys
import os
import time
import math
import argparse
import struct
from typing import List, Optional, Tuple
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))

# ZMQ
try:
    import zmq
except ImportError:
    zmq = None

# ROS2 imports (optional)
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2
    ROS2_AVAILABLE = True
except ImportError:
    pass

# Serial (fallback for direct lidar connection)
try:
    import serial
except ImportError:
    serial = None

# Constants
try:
    from shared.constants import (
        ZMQ_PORT_LIDAR, LIDAR_SERIAL_PORT, LIDAR_BAUD,
        ROS2_TOPIC_CLOUD
    )
except ImportError:
    ZMQ_PORT_LIDAR = 5563
    LIDAR_SERIAL_PORT = "/dev/ttyUSB1"
    LIDAR_BAUD = 230400
    ROS2_TOPIC_CLOUD = "/unilidar/cloud"


def pointcloud_to_2d_scan(points: np.ndarray, num_bins: int = 360, 
                           z_min: float = -0.3, z_max: float = 0.5,
                           max_range: float = 30.0) -> Tuple[List[float], List[float]]:
    """
    Convert 3D point cloud to 2D scan (ranges/angles).
    
    Points are filtered by height (z_min to z_max) and binned by angle.
    For each angle bin, the minimum range is used.
    
    Args:
        points: Nx3 array of (x, y, z) in lidar frame
        num_bins: Number of angle bins (360 = 1 degree resolution)
        z_min, z_max: Height filter (keep points in this range)
        max_range: Maximum range (points beyond are ignored)
    
    Returns:
        (ranges, angles): Lists of ranges and angles for 2D visualization
    """
    if points is None or len(points) == 0:
        # Return empty scan
        angles = [math.radians(i * 360.0 / num_bins) for i in range(num_bins)]
        ranges = [0.0] * num_bins
        return ranges, angles
    
    # Filter by height
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    filtered = points[mask]
    
    if len(filtered) == 0:
        angles = [math.radians(i * 360.0 / num_bins) for i in range(num_bins)]
        ranges = [0.0] * num_bins
        return ranges, angles
    
    # Calculate range and angle for each point
    x = filtered[:, 0]
    y = filtered[:, 1]
    
    point_ranges = np.sqrt(x**2 + y**2)
    point_angles = np.arctan2(y, x)  # -pi to pi
    
    # Normalize angles to 0 to 2*pi
    point_angles = np.mod(point_angles, 2 * np.pi)
    
    # Bin by angle
    bin_width = 2 * np.pi / num_bins
    bin_indices = (point_angles / bin_width).astype(int)
    bin_indices = np.clip(bin_indices, 0, num_bins - 1)
    
    # Initialize ranges with max_range (no obstacle)
    ranges = [max_range] * num_bins
    
    # For each bin, find minimum range
    for i in range(len(filtered)):
        bin_idx = bin_indices[i]
        r = point_ranges[i]
        if r < ranges[bin_idx] and r > 0.05:  # Ignore very close points
            ranges[bin_idx] = float(r)
    
    # Replace max_range with 0 (no detection) for cleaner visualization
    ranges = [r if r < max_range * 0.99 else 0.0 for r in ranges]
    
    # Generate angle list
    angles = [i * bin_width for i in range(num_bins)]
    
    return ranges, angles


class LidarNodeROS2(Node):
    """ROS2 node that subscribes to /unilidar/cloud and publishes 2D scan to ZMQ."""
    
    def __init__(self, zmq_port: int = ZMQ_PORT_LIDAR, rate_hz: float = 10.0):
        super().__init__('lidar_bridge')
        
        self.rate_hz = rate_hz
        self.last_ranges = []
        self.last_angles = []
        self.last_update = 0.0
        
        # ZMQ setup
        self.zmq_ctx = zmq.Context()
        self.zmq_pub = self.zmq_ctx.socket(zmq.PUB)
        self.zmq_pub.setsockopt(zmq.SNDHWM, 1)
        self.zmq_pub.setsockopt(zmq.LINGER, 0)
        self.zmq_pub.setsockopt(zmq.CONFLATE, 1)
        self.zmq_pub.bind(f"tcp://*:{zmq_port}")
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # ROS2 subscription
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            ROS2_TOPIC_CLOUD,
            self.cloud_callback,
            sensor_qos
        )
        
        # Timer for ZMQ publishing (rate limited)
        self.create_timer(1.0 / rate_hz, self.publish_zmq)
        
        self.get_logger().info(f"Lidar Bridge started (ROS2 mode)")
        self.get_logger().info(f"  Subscribing: {ROS2_TOPIC_CLOUD}")
        self.get_logger().info(f"  Publishing ZMQ: {zmq_port} at {rate_hz} Hz")
    
    def cloud_callback(self, msg: PointCloud2):
        """Handle point cloud from unilidar_sdk2."""
        try:
            # Convert PointCloud2 to numpy array
            points = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([p[0], p[1], p[2]])
            
            if len(points) > 0:
                points_np = np.array(points, dtype=np.float32)
                self.last_ranges, self.last_angles = pointcloud_to_2d_scan(points_np)
                self.last_update = time.time()
                
        except Exception as e:
            self.get_logger().warn(f"Error processing point cloud: {e}")
    
    def publish_zmq(self):
        """Publish 2D scan over ZMQ."""
        if not self.last_ranges:
            # No data yet, send empty scan
            self.last_ranges = [0.0] * 360
            self.last_angles = [math.radians(i) for i in range(360)]
        
        msg = {
            "msg_type": "lidar_scan",
            "timestamp": time.time(),
            "ranges": self.last_ranges,
            "angles": self.last_angles,
        }
        
        try:
            self.zmq_pub.send_json(msg, zmq.NOBLOCK)
        except zmq.Again:
            pass
    
    def shutdown(self):
        """Clean up resources."""
        self.zmq_pub.close()
        self.zmq_ctx.term()


def run_ros2_mode(port: int = ZMQ_PORT_LIDAR, rate_hz: float = 10.0):
    """Run the lidar node in ROS2 mode."""
    if not ROS2_AVAILABLE:
        print("[LIDAR] ERROR: ROS2 not available. Install rclpy and source ROS2.")
        print("[LIDAR] Falling back to placeholder mode...")
        run_placeholder_mode(port, rate_hz)
        return
    
    rclpy.init()
    node = LidarNodeROS2(port, rate_hz)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[LIDAR] Stopped")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def make_placeholder_scan(num_points: int = 360, range_m: float = 5.0) -> Tuple[List[float], List[float]]:
    """Return (ranges, angles) for a placeholder 360° scan."""
    angles = [math.radians(i * 360.0 / num_points) for i in range(num_points)]
    ranges = [range_m] * num_points
    return ranges, angles


def read_scan_from_serial(ser) -> Optional[Tuple[List[float], List[float]]]:
    """
    Read one scan from serial. Simple format: 360 x 2 bytes LE (distance mm).
    If read fails or protocol differs, returns None.
    """
    if ser is None or not ser.is_open:
        return None
    try:
        need = 360 * 2
        buf = b""
        ser.reset_input_buffer()
        deadline = time.time() + 0.5
        while len(buf) < need and time.time() < deadline:
            buf += ser.read(need - len(buf))
        if len(buf) < need:
            return None
        ranges_m = [struct.unpack_from("<H", buf, i * 2)[0] / 1000.0 for i in range(360)]
        angles = [math.radians(i) for i in range(360)]
        return ranges_m, angles
    except Exception:
        return None


def run_placeholder_mode(port: int = ZMQ_PORT_LIDAR, rate_hz: float = 10.0,
                         serial_port: Optional[str] = None):
    """Run in placeholder mode (no ROS2, optional serial)."""
    if not zmq:
        print("[LIDAR] pyzmq required")
        return

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.bind(f"tcp://*:{port}")

    ser = None
    if serial_port and serial:
        try:
            ser = serial.Serial(serial_port, LIDAR_BAUD, timeout=0.05)
            print(f"[LIDAR] Serial open: {serial_port} @ {LIDAR_BAUD}")
        except Exception as e:
            print(f"[LIDAR] Serial failed: {e}, using placeholder")

    fallback_ranges, fallback_angles = make_placeholder_scan(360, 5.0)
    period = 1.0 / rate_hz
    mode = "serial" if (ser and ser.is_open) else "placeholder"
    
    print(f"[LIDAR] Publishing on port {port} (mode={mode}) at {rate_hz} Hz")
    if mode == "placeholder":
        print(f"[LIDAR] Note: Run with --ros2 for real unilidar_sdk2 integration")

    try:
        while True:
            t0 = time.perf_counter()
            
            if ser and ser.is_open:
                scan = read_scan_from_serial(ser)
                ranges, angles = (scan if scan is not None else (fallback_ranges, fallback_angles))
            else:
                ranges, angles = fallback_ranges, fallback_angles
            
            msg = {
                "msg_type": "lidar_scan",
                "timestamp": time.time(),
                "ranges": ranges,
                "angles": angles,
            }
            
            try:
                sock.send_json(msg, zmq.NOBLOCK)
            except zmq.Again:
                pass
            
            elapsed = time.perf_counter() - t0
            if period - elapsed > 0:
                time.sleep(period - elapsed)
                
    except KeyboardInterrupt:
        print("\n[LIDAR] Stopped")
    finally:
        if ser and ser.is_open:
            ser.close()
        sock.close()
        ctx.term()


def main():
    ap = argparse.ArgumentParser(
        description="Lidar node: bridge /unilidar/cloud (ROS2) or serial to ZMQ"
    )
    ap.add_argument("--port", type=int, default=ZMQ_PORT_LIDAR, help="ZMQ port")
    ap.add_argument("--rate", type=float, default=10, help="Publish rate (Hz)")
    ap.add_argument("--placeholder", action="store_true", help="Force placeholder mode")
    ap.add_argument("--ros2", action="store_true", help="Force ROS2 mode")
    ap.add_argument("--serial-port", type=str, default=None,
                    help="Serial port for direct lidar (fallback)")
    args = ap.parse_args()
    
    # Default to ROS2 if available, otherwise placeholder
    use_ros2 = args.ros2 or (ROS2_AVAILABLE and not args.placeholder)
    
    if use_ros2 and ROS2_AVAILABLE:
        run_ros2_mode(args.port, args.rate)
    else:
        run_placeholder_mode(args.port, args.rate, args.serial_port)


if __name__ == "__main__":
    main()
