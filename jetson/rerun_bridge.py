#!/usr/bin/env python3
"""
Rerun.io Bridge — Point-LIO cloud + animated URDF (animated_urdf pattern)
=========================================================================
ROS2 node that streams data to Rerun for 3D visualization in the PyQt5 GUI.

Features:
  - /cloud_registered  → 12k-point rainbow cloud (orange floor z < 0.15)
  - /robot_description → full URDF with SO-ARM101 arm (log_file_from_path)
  - /joint_states      → animate every revolute joint via UrdfTree (0.29+ API)
  - /Odometry + /tf    → robot base pose
  - /plan              → Nav2 path LineStrips3D + goal marker

View behavior is tuned to match jetson/rviz/slam_3rd_person.rviz (fixed frame
camera_init, 30s cloud decay, single 3rd-person-style view).

Serve on gRPC port 9876 so laptop QWebEngineView can connect:
  rerun+http://JETSON_IP:9876/proxy

Usage:
  pip install rerun-sdk>=0.29.2
  python3 rerun_bridge.py --serve-port 9876
  # or via start_robot.sh: USE_RERUN=1 ./start_robot.sh
"""

from __future__ import annotations

import argparse
import math
import sys
import tempfile
import threading
from pathlib import Path

# ---------------------------------------------------------------------------
# Project root (for shared imports)
# ---------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# Default URDF path (on Jetson; robot_description topic is preferred)
DEFAULT_URDF_PATH = Path(__file__).resolve().parent / "robot_description" / "urdf" / "so101_new_calib.urdf"

# ---------------------------------------------------------------------------
# ROS2 imports
# ---------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2, JointState
    from nav_msgs.msg import Odometry, Path as NavPath
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String
    import sensor_msgs_py.point_cloud2 as pc2
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

# ---------------------------------------------------------------------------
# Rerun SDK
# ---------------------------------------------------------------------------
try:
    import rerun as rr
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False

# UrdfTree (available in rerun-sdk >= 0.29)
try:
    from rerun.urdf import UrdfTree
    URDF_TREE_AVAILABLE = True
except ImportError:
    UrdfTree = None
    URDF_TREE_AVAILABLE = False

# Blueprint API
try:
    import rerun.blueprint as rrb
    BLUEPRINT_AVAILABLE = True
except ImportError:
    rrb = None
    BLUEPRINT_AVAILABLE = False

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
TIMELINE_ROS = "ros_time"          # Rerun timeline for ROS timestamps
MAX_CLOUD_POINTS = 12_000          # Downsample target (matches RViz-style display)
CLOUD_DECAY_SEC = 30.0            # Keep points for 30s (match slam_3rd_person.rviz Decay Time)
CLOUD_LOG_INTERVAL_SEC = 0.2      # Throttle cloud log to ~5 Hz
FLOOR_Z_THRESHOLD = 0.15          # Points below this → orange
FLOOR_COLOR = np.array([255, 140, 0], dtype=np.uint8)  # Orange for floor
GRPC_PORT_DEFAULT = 9876           # gRPC serve port

# Arm joint names matching URDF and /joint_states topic from followerarm.py.
# Order: servo_1..servo_6 → shoulder_pan, shoulder_lift, elbow_flex,
#         wrist_flex, wrist_roll, gripper
ARM_JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
]


# ============================================================================
# Color utilities
# ============================================================================

def height_rainbow(z: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    """
    Map height values to rainbow RGB (blue→cyan→green→yellow→red).
    Returns (N, 3) uint8 array.  Floor points (z < FLOOR_Z_THRESHOLD)
    are colored orange separately by the caller.
    """
    span = max(z_max - z_min, 1e-6)
    t = np.clip((z - z_min) / span, 0.0, 1.0)

    r = np.zeros_like(t)
    g = np.zeros_like(t)
    b = np.zeros_like(t)

    # Blue → Cyan  (0.00 – 0.25)
    m = t < 0.25
    r[m], g[m], b[m] = 0.0, t[m] * 4.0, 1.0

    # Cyan → Green (0.25 – 0.50)
    m = (t >= 0.25) & (t < 0.50)
    r[m], g[m], b[m] = 0.0, 1.0, 1.0 - (t[m] - 0.25) * 4.0

    # Green → Yellow (0.50 – 0.75)
    m = (t >= 0.50) & (t < 0.75)
    r[m], g[m], b[m] = (t[m] - 0.50) * 4.0, 1.0, 0.0

    # Yellow → Red (0.75 – 1.00)
    m = t >= 0.75
    r[m], g[m], b[m] = 1.0, 1.0 - (t[m] - 0.75) * 4.0, 0.0

    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


def colorize_cloud(pts: np.ndarray) -> np.ndarray:
    """
    Assign colors: orange for floor (z < FLOOR_Z_THRESHOLD), height rainbow
    for everything else.  Returns (N, 3) uint8.
    """
    z = pts[:, 2]
    z_min, z_max = float(z.min()), float(z.max())
    colors = height_rainbow(z, z_min, z_max)

    # Override floor points with solid orange
    floor_mask = z < FLOOR_Z_THRESHOLD
    colors[floor_mask] = FLOOR_COLOR

    return colors


# ============================================================================
# Main
# ============================================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Rerun bridge: Point-LIO cloud + animated URDF (animated_urdf pattern)."
    )
    parser.add_argument(
        "--serve-port", type=int, default=GRPC_PORT_DEFAULT,
        help="gRPC port for viewer connection (default: %(default)s)"
    )
    parser.add_argument(
        "--no-serve", action="store_true",
        help="Do not start gRPC server (for local-only viewing)"
    )
    parser.add_argument(
        "--urdf", type=str, default=str(DEFAULT_URDF_PATH),
        help="Path to URDF file (fallback if /robot_description not published)"
    )
    parser.add_argument(
        "--test-connectivity", action="store_true",
        help="No ROS2: just serve gRPC and log a welcome point (to verify viewer connection)"
    )
    args = parser.parse_args()

    # --- Validate dependencies ---
    if not RERUN_AVAILABLE:
        print("[Rerun] rerun-sdk not installed. Run: pip install rerun-sdk>=0.29.2")
        sys.exit(1)
    if not args.test_connectivity and not ROS2_AVAILABLE:
        print("[Rerun] ROS2 (rclpy, sensor_msgs, nav_msgs) required.")
        sys.exit(1)
    if not args.test_connectivity and not NUMPY_AVAILABLE:
        print("[Rerun] numpy required.")
        sys.exit(1)
    if not args.test_connectivity and not URDF_TREE_AVAILABLE:
        print("[Rerun] WARNING: rerun.urdf.UrdfTree not available (need rerun-sdk >= 0.29).")
        print("[Rerun] Joint animation will be DISABLED. Install: pip install rerun-sdk>=0.29.2")

    # --- Init Rerun recording ---
    rr.init(application_id="RoboCupRescue RMRC 2026")

    # --- Start gRPC server so laptop / web viewer can connect ---
    if not args.no_serve:
        rr.serve_grpc(grpc_port=args.serve_port)
        print(
            f"[Rerun] gRPC server on 0.0.0.0:{args.serve_port}\n"
            f"[Rerun] Laptop: rerun --connect rerun+http://<jetson_ip>:{args.serve_port}/proxy\n"
            f"[Rerun] Web:    https://app.rerun.io/version/0.29.2/?url=rerun+http://<jetson_ip>:{args.serve_port}/proxy"
        )

    # --- Log welcome entity so viewer always has something to show ---
    try:
        rr.reset_time()
        rr.log("world/welcome", rr.Points3D([[0.0, 0.0, 0.0]], radii=0.1))
        rr.reset_time()
    except Exception as e:
        print(f"[Rerun] Welcome log warning (non-fatal): {e}")

    # --- Test connectivity mode: no ROS2, just keep server alive ---
    if args.test_connectivity:
        print("[Rerun] Test mode: connect from laptop. Press Ctrl+C to exit.")
        try:
            while True:
                import time
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        return

    # --- Send blueprint: single 3D view (URDF + world/cloud together), tuned to slam_3rd_person.rviz ---
    if BLUEPRINT_AVAILABLE and rrb is not None:
        try:
            blueprint = rrb.Grid(
                rrb.Spatial3DView(
                    origin="/",
                    contents="$origin/**",
                    name="Rescue Bot 3D",
                    overrides={
                        "rescue_robot/collision_geometries": rrb.EntityBehavior(visible=False),
                    },
                )
            )
            rr.send_blueprint(blueprint)
        except Exception as e:
            print(f"[Rerun] Blueprint warning (non-fatal): {e}")

    # --- Resolve fallback URDF: use rescue_robot.urdf if default path missing ---
    urdf_path = args.urdf
    if not Path(urdf_path).is_file():
        rescue_robot_urdf = Path(__file__).resolve().parent / "robot_description" / "urdf" / "rescue_robot.urdf"
        if rescue_robot_urdf.is_file():
            urdf_path = str(rescue_robot_urdf)
            print(f"[Rerun] Using fallback URDF: {urdf_path}")

    # --- Start ROS2 node ---
    rclpy.init()
    node = RerunBridgeNode(urdf_fallback_path=urdf_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[Rerun] Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ============================================================================
# ROS2 Node
# ============================================================================

class RerunBridgeNode(Node):
    """
    Logs Point-LIO cloud, full URDF (animated_urdf pattern), joint_states,
    odom/tf, and nav path to Rerun.
    """

    def __init__(self, urdf_fallback_path: str = str(DEFAULT_URDF_PATH)) -> None:
        super().__init__("rerun_bridge")

        # ----- Declare parameters -----
        self.declare_parameter("cloud_topic", "/cloud_registered")
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("max_points", MAX_CLOUD_POINTS)

        self._cloud_topic = self.get_parameter("cloud_topic").value
        self._robot_desc_topic = self.get_parameter("robot_description_topic").value
        self._joint_topic = self.get_parameter("joint_states_topic").value
        self._odom_topic = self.get_parameter("odom_topic").value
        self._tf_topic = self.get_parameter("tf_topic").value
        self._path_topic = self.get_parameter("path_topic").value
        self._max_points = int(self.get_parameter("max_points").value)

        # ----- URDF state -----
        self._urdf_path: Path | None = None
        self._urdf_tree: UrdfTree | None = None   # type: ignore[name-defined]
        self._urdf_logged = False
        self._urdf_fallback_path = urdf_fallback_path
        self._joint_positions: dict[str, float] = {}
        self._lock = threading.Lock()

        # ----- Debug counters -----
        self._cloud_count = 0
        self._cloud_error_count = 0

        # ----- Cloud accumulation (match RViz Decay Time: 30s from slam_3rd_person.rviz) -----
        self._cloud_buffer: list[tuple[np.ndarray, float]] = []  # (points Nx3, stamp_sec)
        self._cloud_last_log_stamp = 0.0

        # ----- QoS profiles -----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ----- Subscriptions -----
        self._cloud_sub = self.create_subscription(
            PointCloud2, self._cloud_topic, self._cloud_cb, sensor_qos
        )
        self._robot_desc_sub = self.create_subscription(
            String, self._robot_desc_topic, self._robot_description_cb, 10
        )
        self._joint_sub = self.create_subscription(
            JointState, self._joint_topic, self._joint_states_cb, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._odom_cb, 10
        )
        self._tf_sub = self.create_subscription(
            TFMessage, self._tf_topic, self._tf_cb, 10
        )
        self._path_sub = self.create_subscription(
            NavPath, self._path_topic, self._path_cb, 10
        )

        # ----- Log fallback URDF if file exists and topic not yet received -----
        self._try_log_fallback_urdf()

        self.get_logger().info(
            f"Rerun bridge started: cloud={self._cloud_topic}, "
            f"urdf={self._robot_desc_topic}, joints={self._joint_topic}, "
            f"odom={self._odom_topic}, path={self._path_topic}"
        )

    # ------------------------------------------------------------------
    # URDF: fallback from file if /robot_description never arrives
    # ------------------------------------------------------------------
    def _try_log_fallback_urdf(self) -> None:
        """Log URDF from disk as fallback (e.g. if robot_state_publisher not running)."""
        with self._lock:
            if self._urdf_logged:
                return
        path = Path(self._urdf_fallback_path)
        if not path.is_file():
            self.get_logger().warn(f"Fallback URDF not found: {path}")
            return
        self._log_urdf_from_path(path)
        self.get_logger().info(f"Logged fallback URDF from {path}")

    def _log_urdf_from_path(self, urdf_path: Path) -> None:
        """
        Log URDF to Rerun and build UrdfTree for joint animation.
        Follows the animated_urdf example pattern:
          1. rr.log_file_from_path(urdf_path)  — Rerun parses and visualizes the URDF
          2. UrdfTree.from_file_path(urdf_path) — for compute_transform() on joints
        """
        try:
            # IMPORTANT: Do NOT set a specific time for the URDF — use reset_time()
            # so it appears as "static" (timeless) data. Setting timestamp=0.0
            # would place it at epoch 1970, stretching the timeline and making
            # live cloud data invisible.
            rr.reset_time()

            # Step 1: Let Rerun's built-in URDF loader handle visualization
            rr.log_file_from_path(str(urdf_path))

            # Step 2: Build UrdfTree for joint animation (SDK >= 0.29)
            if URDF_TREE_AVAILABLE and UrdfTree is not None:
                self._urdf_tree = UrdfTree.from_file_path(str(urdf_path))
                self._urdf_path = urdf_path
            else:
                self._urdf_tree = None

            with self._lock:
                self._urdf_logged = True

            # Reset time again so subsequent logs aren't affected
            rr.reset_time()

        except Exception as e:
            self.get_logger().warn(f"URDF log error: {e}")
            import traceback
            traceback.print_exc()

    # ------------------------------------------------------------------
    # /robot_description → temp file + log
    # ------------------------------------------------------------------
    def _robot_description_cb(self, msg: String) -> None:
        """Receive URDF string from /robot_description, write to temp file and log."""
        with self._lock:
            if self._urdf_logged:
                return  # Already loaded (from fallback or previous msg)

        if not msg.data or len(msg.data) < 50:
            return

        try:
            with tempfile.NamedTemporaryFile(
                mode="w", suffix=".urdf", delete=False
            ) as f:
                f.write(msg.data)
                urdf_path = Path(f.name)

            self._log_urdf_from_path(urdf_path)
            self.get_logger().info("Logged /robot_description + UrdfTree (animated_urdf pattern)")

        except Exception as e:
            self.get_logger().warn(f"robot_description error: {e}")
            import traceback
            traceback.print_exc()

    # ------------------------------------------------------------------
    # /joint_states → animate every revolute/continuous joint
    # Pattern: exactly like animated_urdf.py rec.log("transforms", transform)
    # ------------------------------------------------------------------
    def _joint_states_cb(self, msg: JointState) -> None:
        """
        Update joint positions and log transforms via UrdfTree.
        Each joint's compute_transform() returns a Transform3D with the
        correct parent_frame/child_frame set, so we log ALL transforms
        to the same entity path "transforms" (Rerun 0.29+ frame-based dispatch).
        """
        # Store latest joint positions
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_positions[name] = float(msg.position[i])

        # Animate via UrdfTree (requires SDK >= 0.29)
        if self._urdf_tree is None:
            return

        try:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            rr.set_time(TIMELINE_ROS, timestamp=t)

            for joint in self._urdf_tree.joints():
                # Only animate movable joints
                if joint.joint_type not in ("revolute", "continuous"):
                    continue

                # Look up current angle from /joint_states
                angle = self._joint_positions.get(joint.name, 0.0)

                # Clamp to joint limits and compute full transform
                # (includes parent_frame + child_frame + rotation + translation)
                transform = joint.compute_transform(angle, clamp=True)

                # Log to "transforms" — Rerun dispatches by frame names
                rr.log("transforms", transform)

        except Exception as e:
            self.get_logger().warn(f"Joint animation error: {e}")
            import traceback
            traceback.print_exc()

    # ------------------------------------------------------------------
    # /cloud_registered → accumulate 30s decay + rainbow/orange (match RViz)
    # ------------------------------------------------------------------
    def _cloud_cb(self, msg: PointCloud2) -> None:
        """
        Accumulate points with 30s decay (match slam_3rd_person.rviz Decay Time),
        then downsample to MAX_CLOUD_POINTS and log. Builds a persistent map
        instead of replacing the cloud each frame.
        """
        try:
            pts = self._read_points_numpy(msg)
            if pts is None or len(pts) == 0:
                self._cloud_error_count += 1
                if self._cloud_error_count <= 5:
                    self.get_logger().warn(
                        f"Cloud: empty/None points (msg fields: "
                        f"{[f.name for f in msg.fields]}, "
                        f"w={msg.width}, h={msg.height}, "
                        f"point_step={msg.point_step})"
                    )
                return

            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # Per-scan cap to limit buffer growth (keep scan diversity)
            scan_cap = min(8000, self._max_points * 2)
            if len(pts) > scan_cap:
                idx = np.random.choice(len(pts), scan_cap, replace=False)
                pts = pts[idx]

            self._cloud_buffer.append((pts, t))

            # Drop points older than CLOUD_DECAY_SEC
            cutoff = t - CLOUD_DECAY_SEC
            self._cloud_buffer = [(p, s) for p, s in self._cloud_buffer if s > cutoff]

            # Throttle logging
            if (t - self._cloud_last_log_stamp) < CLOUD_LOG_INTERVAL_SEC:
                return
            self._cloud_last_log_stamp = t

            # Merge and downsample
            if not self._cloud_buffer:
                return
            merged = np.vstack([p for p, _ in self._cloud_buffer])
            if len(merged) > self._max_points:
                idx = np.random.choice(len(merged), self._max_points, replace=False)
                merged = merged[idx]

            colors = colorize_cloud(merged)
            rr.set_time(TIMELINE_ROS, timestamp=t)
            rr.log("world/cloud", rr.Points3D(merged, colors=colors, radii=0.01))

            self._cloud_count += 1
            if self._cloud_count <= 3 or self._cloud_count % 100 == 0:
                self.get_logger().info(
                    f"Cloud #{self._cloud_count}: {len(merged)} pts (buffer {len(self._cloud_buffer)} scans)"
                )

        except Exception as e:
            self.get_logger().warn(f"Cloud error: {e}")
            import traceback
            traceback.print_exc()

    @staticmethod
    def _read_points_numpy(msg: PointCloud2) -> np.ndarray | None:
        """
        Read PointCloud2 to Nx3 float32 numpy array.
        Uses structured array for speed; falls back to iterator.
        """
        try:
            # read_points returns a numpy structured array (or generator)
            cloud_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

            # In newer sensor_msgs_py, read_points returns a structured ndarray directly
            if isinstance(cloud_data, np.ndarray):
                if cloud_data.dtype.names:  # structured array
                    pts = np.column_stack([
                        cloud_data['x'].astype(np.float32),
                        cloud_data['y'].astype(np.float32),
                        cloud_data['z'].astype(np.float32),
                    ])
                else:
                    pts = cloud_data.astype(np.float32)
                return pts if len(pts) > 0 else None

            # Fallback: generator/list of tuples
            pts_list = list(cloud_data)
            if not pts_list:
                return None
            return np.array(pts_list, dtype=np.float32).reshape(-1, 3)
        except Exception:
            import traceback
            traceback.print_exc()
            return None

    # ------------------------------------------------------------------
    # /Odometry → robot base pose in world frame
    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        """Log robot base pose from odometry."""
        try:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            rr.set_time(TIMELINE_ROS, timestamp=t)

            p = msg.pose.pose.position
            o = msg.pose.pose.orientation

            rr.log(
                "world/odom/base_link",
                rr.Transform3D(
                    translation=(p.x, p.y, p.z),
                    rotation=rr.Quaternion(xyzw=(o.x, o.y, o.z, o.w)),
                ),
            )
        except Exception as e:
            self.get_logger().warn(f"Odom error: {e}")
            import traceback
            traceback.print_exc()

    # ------------------------------------------------------------------
    # /tf → key transforms (map→odom, odom→base_link, etc.)
    # ------------------------------------------------------------------
    def _tf_cb(self, msg: TFMessage) -> None:
        """Log TF transforms to Rerun."""
        try:
            for tf_stamped in msg.transforms:
                t_sec = (
                    tf_stamped.header.stamp.sec
                    + tf_stamped.header.stamp.nanosec * 1e-9
                )
                rr.set_time(TIMELINE_ROS, timestamp=t_sec)

                tr = tf_stamped.transform.translation
                rot = tf_stamped.transform.rotation

                rr.log(
                    f"world/tf/{tf_stamped.child_frame_id}",
                    rr.Transform3D(
                        translation=(tr.x, tr.y, tr.z),
                        rotation=rr.Quaternion(xyzw=(rot.x, rot.y, rot.z, rot.w)),
                    ),
                )
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")
            import traceback
            traceback.print_exc()

    # ------------------------------------------------------------------
    # /plan (Nav2 path) → LineStrips3D + goal marker
    # ------------------------------------------------------------------
    def _path_cb(self, msg: NavPath) -> None:
        """Log Nav2 planned path and goal marker."""
        try:
            if not msg.poses:
                return

            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            rr.set_time(TIMELINE_ROS, timestamp=t)

            # Path as line strip (cyan)
            positions = np.array(
                [
                    [p.pose.position.x, p.pose.position.y, p.pose.position.z]
                    for p in msg.poses
                ],
                dtype=np.float32,
            )
            rr.log(
                "world/nav_path",
                rr.LineStrips3D(
                    [positions],
                    colors=[(0, 200, 255)],  # Cyan path
                    radii=[0.02],
                ),
            )

            # Goal marker (last pose, red sphere)
            if len(positions) > 0:
                rr.log(
                    "world/nav_goal",
                    rr.Points3D(
                        positions[-1:],
                        colors=[(255, 50, 50)],   # Red goal
                        radii=[0.08],
                    ),
                )

        except Exception as e:
            self.get_logger().warn(f"Path error: {e}")
            import traceback
            traceback.print_exc()


# ============================================================================
# Entry point
# ============================================================================
if __name__ == "__main__":
    main()
