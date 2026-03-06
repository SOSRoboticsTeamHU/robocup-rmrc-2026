"""
RoboCupRescue RMRC 2026 - 3D Point Cloud Viewer
================================================
RViz-like 3D visualization of Point-LIO registered point cloud,
occupancy grid floor, robot pose, path trail, and Nav2 overlay.

Uses pyqtgraph.opengl for hardware-accelerated 3D rendering.
"""

import sys
import os
import math
import time
import struct
from typing import Optional, List, Dict, Any

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

try:
    from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QCheckBox
    from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
    PYQT_AVAILABLE = True
except ImportError:
    PYQT_AVAILABLE = False

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

try:
    import pyqtgraph.opengl as gl
    import pyqtgraph as pg
    OPENGL_AVAILABLE = True
except ImportError:
    OPENGL_AVAILABLE = False

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False

try:
    from shared.constants import ZMQ_PORT_SLAM, ZMQ_PORT_CLOUD
except ImportError:
    ZMQ_PORT_SLAM = 5562
    ZMQ_PORT_CLOUD = 5564

# Robot dimensions (meters)
ROBOT_LENGTH_M = 0.35
ROBOT_WIDTH_M = 0.25
ROBOT_HEIGHT_M = 0.20

# Height colormap: z-value -> RGBA
# Dark blue (ground) -> green (mid) -> yellow -> red (high)
def height_to_color(z, z_min=-0.3, z_max=1.5):
    """Map Z height to RGBA color (like RViz PointCloud2 with axis color)."""
    t = np.clip((z - z_min) / max(z_max - z_min, 0.01), 0, 1)
    # Blue -> Cyan -> Green -> Yellow -> Red
    r = np.where(t < 0.5, 0.0, np.clip((t - 0.5) * 4, 0, 1))
    g = np.where(t < 0.25, t * 4, np.where(t < 0.75, 1.0, 1.0 - (t - 0.75) * 4))
    b = np.where(t < 0.5, 1.0 - t * 2, 0.0)
    a = np.ones_like(t) * 0.85
    return np.column_stack([r, g, b, a]).astype(np.float32)


class CloudReceiverThread(QThread):
    """Background thread for receiving point cloud + SLAM data over ZMQ."""
    cloud_update = pyqtSignal(dict)
    slam_update = pyqtSignal(dict)

    def __init__(self, jetson_ip: str, cloud_port: int, slam_port: int):
        super().__init__()
        self.jetson_ip = jetson_ip
        self.cloud_port = cloud_port
        self.slam_port = slam_port
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        if not ZMQ_AVAILABLE:
            return
        try:
            ctx = zmq.Context()

            sock_cloud = ctx.socket(zmq.SUB)
            sock_cloud.setsockopt(zmq.SUBSCRIBE, b"")
            sock_cloud.setsockopt(zmq.RCVHWM, 2)
            sock_cloud.setsockopt(zmq.CONFLATE, 1)
            sock_cloud.connect(f"tcp://{self.jetson_ip}:{self.cloud_port}")

            sock_slam = ctx.socket(zmq.SUB)
            sock_slam.setsockopt(zmq.SUBSCRIBE, b"")
            sock_slam.setsockopt(zmq.RCVHWM, 2)
            sock_slam.setsockopt(zmq.CONFLATE, 1)
            sock_slam.connect(f"tcp://{self.jetson_ip}:{self.slam_port}")

            poller = zmq.Poller()
            poller.register(sock_cloud, zmq.POLLIN)
            poller.register(sock_slam, zmq.POLLIN)

            while self._running:
                socks = dict(poller.poll(timeout=10))
                if sock_cloud in socks:
                    try:
                        msg = sock_cloud.recv_json(zmq.NOBLOCK)
                        self.cloud_update.emit(msg)
                    except Exception:
                        pass
                if sock_slam in socks:
                    try:
                        msg = sock_slam.recv_json(zmq.NOBLOCK)
                        self.slam_update.emit(msg)
                    except Exception:
                        pass

            sock_cloud.close()
            sock_slam.close()
            ctx.term()
        except Exception:
            pass


class PointCloudView(QFrame):
    """3D Point Cloud viewer widget — RViz-like visualization."""

    def __init__(self, jetson_ip: str = "192.168.2.100",
                 cloud_port: int = None, slam_port: int = None, parent=None):
        super().__init__(parent)
        self.setObjectName("CameraFrame")
        self.setMinimumSize(400, 300)
        self.setStyleSheet("""
            QFrame#CameraFrame {
                background-color: #0a0a14;
                border: 1px solid #0f3460;
                border-radius: 4px;
            }
        """)

        self.jetson_ip = jetson_ip
        self.cloud_port = cloud_port if cloud_port is not None else ZMQ_PORT_CLOUD
        self.slam_port = slam_port if slam_port is not None else ZMQ_PORT_SLAM

        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_yaw = 0.0
        self.has_pose = False
        self.path_history: List[tuple] = []
        self.nav2_path: List[List[float]] = []
        self.nav2_goal = None
        self.grid_data = None
        self.grid_resolution = 0.05
        self.grid_origin_x = -5.0
        self.grid_origin_y = -5.0
        self._cloud_points = None  # Nx3 numpy array
        self._cloud_colors = None  # Nx4 numpy array
        self._follow_robot = True
        self._show_grid = True
        self._show_cloud = True

        # Cache keys for expensive updates
        self._grid_cache_key = None
        self._path_cache_key = None

        self._setup_ui()
        self._setup_3d_scene()
        self._start_receiver()

        # Render timer (30 fps)
        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._render_update)
        self._render_timer.start(33)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        # Header with toggles
        header = QHBoxLayout()
        title = QLabel("3D Point Cloud")
        title.setStyleSheet("color: #4ecca3; font-weight: bold; font-size: 11px;")
        header.addWidget(title)
        header.addStretch()

        self.follow_cb = QCheckBox("Follow")
        self.follow_cb.setChecked(True)
        self.follow_cb.setStyleSheet("color: #a0a0a0; font-size: 10px;")
        self.follow_cb.stateChanged.connect(lambda s: setattr(self, '_follow_robot', s == Qt.Checked))
        header.addWidget(self.follow_cb)

        self.grid_cb = QCheckBox("Grid")
        self.grid_cb.setChecked(True)
        self.grid_cb.setStyleSheet("color: #a0a0a0; font-size: 10px;")
        self.grid_cb.stateChanged.connect(self._toggle_grid)
        header.addWidget(self.grid_cb)

        self.cloud_cb = QCheckBox("Cloud")
        self.cloud_cb.setChecked(True)
        self.cloud_cb.setStyleSheet("color: #a0a0a0; font-size: 10px;")
        self.cloud_cb.stateChanged.connect(self._toggle_cloud)
        header.addWidget(self.cloud_cb)

        layout.addLayout(header)

        # 3D view
        if OPENGL_AVAILABLE and NUMPY_AVAILABLE:
            self.gl_widget = gl.GLViewWidget()
            self.gl_widget.setBackgroundColor(pg.mkColor(10, 10, 20))
            layout.addWidget(self.gl_widget, 1)
        else:
            lbl = QLabel("3D view requires:\npip install pyqtgraph PyOpenGL numpy")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("color: #666; font-size: 12px;")
            layout.addWidget(lbl, 1)
            self.gl_widget = None

        # Status bar
        self.status_label = QLabel("Waiting for point cloud...")
        self.status_label.setStyleSheet("color: #555; font-size: 9px;")
        layout.addWidget(self.status_label)

    def _setup_3d_scene(self):
        """Initialize the 3D scene with grid, axes, and placeholder items."""
        if not self.gl_widget:
            return

        # Ground grid (RViz-style gray grid lines)
        self._grid_item = gl.GLGridItem()
        self._grid_item.setSize(20, 20)  # 20m x 20m
        self._grid_item.setSpacing(1, 1)  # 1m grid lines
        self._grid_item.setColor(pg.mkColor(40, 45, 55, 100))
        self.gl_widget.addItem(self._grid_item)

        # Point cloud scatter plot
        self._cloud_item = gl.GLScatterPlotItem()
        self._cloud_item.setGLOptions('translucent')
        self.gl_widget.addItem(self._cloud_item)

        # Occupancy grid floor (scatter plot of occupied cells)
        self._occ_grid_item = gl.GLScatterPlotItem()
        self._occ_grid_item.setGLOptions('translucent')
        self.gl_widget.addItem(self._occ_grid_item)

        # Robot marker (box wireframe approximation — use scatter for now)
        self._robot_item = gl.GLScatterPlotItem()
        self.gl_widget.addItem(self._robot_item)

        # Robot direction arrow (line from robot center forward)
        self._robot_arrow = gl.GLLinePlotItem()
        self._robot_arrow.setGLOptions('translucent')
        self.gl_widget.addItem(self._robot_arrow)

        # Path trail
        self._path_item = gl.GLLinePlotItem()
        self._path_item.setGLOptions('translucent')
        self.gl_widget.addItem(self._path_item)

        # Nav2 planned path
        self._nav2_path_item = gl.GLLinePlotItem()
        self._nav2_path_item.setGLOptions('translucent')
        self.gl_widget.addItem(self._nav2_path_item)

        # Goal marker
        self._goal_item = gl.GLScatterPlotItem()
        self.gl_widget.addItem(self._goal_item)

        # Set initial camera position (RViz-like: looking down at 45°)
        self.gl_widget.setCameraPosition(distance=5, elevation=45, azimuth=-90)

    def _start_receiver(self):
        """Start ZMQ receiver thread."""
        self._receiver = CloudReceiverThread(
            self.jetson_ip, self.cloud_port, self.slam_port)
        self._receiver.cloud_update.connect(self._on_cloud_data)
        self._receiver.slam_update.connect(self._on_slam_data)
        self._receiver.start()

    def _on_cloud_data(self, msg: dict):
        """Handle incoming point cloud data."""
        if msg.get("msg_type") != "point_cloud":
            return
        try:
            points = msg.get("points", [])
            if not points:
                return
            pts = np.array(points, dtype=np.float32)
            if pts.ndim != 2 or pts.shape[1] < 3:
                return
            self._cloud_points = pts[:, :3]
            self._cloud_colors = height_to_color(pts[:, 2])
            n = len(pts)
            self.status_label.setText(f"Cloud: {n} pts | Pose: ({self.robot_x:.1f}, {self.robot_y:.1f})")
        except Exception:
            pass

    def _on_slam_data(self, msg: dict):
        """Handle incoming SLAM pose + grid data."""
        try:
            if "x" in msg and "y" in msg:
                self.robot_x = float(msg["x"])
                self.robot_y = float(msg["y"])
                self.robot_z = float(msg.get("z", 0.0))
                self.robot_yaw = float(msg.get("yaw", 0.0))
                self.has_pose = True

            if "path" in msg and msg["path"]:
                self.path_history = [(p[0], p[1]) for p in msg["path"] if len(p) >= 2]

            if "grid" in msg and msg["grid"]:
                self.grid_data = msg["grid"]
                self.grid_resolution = float(msg.get("resolution", 0.05))
                self.grid_origin_x = float(msg.get("origin_x", -5.0))
                self.grid_origin_y = float(msg.get("origin_y", -5.0))

            # Nav2 overlay
            if "nav2_path" in msg:
                self.nav2_path = msg["nav2_path"] or []
            if "nav2_goal" in msg:
                self.nav2_goal = msg["nav2_goal"]
            if "goal_distance_m" in msg:
                self.goal_distance_m = msg["goal_distance_m"]

            # Also check for embedded point cloud (if slam_bridge sends it inline)
            if "cloud" in msg and msg["cloud"]:
                pts = np.array(msg["cloud"], dtype=np.float32)
                if pts.ndim == 2 and pts.shape[1] >= 3:
                    self._cloud_points = pts[:, :3]
                    self._cloud_colors = height_to_color(pts[:, 2])
        except Exception:
            pass

    def _render_update(self):
        """Update the 3D scene (called at 30fps by timer)."""
        if not self.gl_widget:
            return

        # 1) Point cloud
        if self._show_cloud and self._cloud_points is not None and len(self._cloud_points) > 0:
            self._cloud_item.setData(
                pos=self._cloud_points,
                color=self._cloud_colors,
                size=2.0,
                pxMode=True
            )
        elif not self._show_cloud:
            self._cloud_item.setData(pos=np.zeros((0, 3)))

        # 2) Occupancy grid floor
        if self._show_grid and self.grid_data:
            grid_key = (id(self.grid_data), len(self.grid_data))
            if grid_key != self._grid_cache_key:
                self._grid_cache_key = grid_key
                self._update_occ_grid()

        # 3) Robot marker
        if self.has_pose:
            rx, ry, rz = self.robot_x, self.robot_y, self.robot_z
            # Robot as a bright teal point
            self._robot_item.setData(
                pos=np.array([[rx, ry, rz + 0.05]]),
                color=np.array([[0.3, 0.8, 0.6, 1.0]]),
                size=14.0,
                pxMode=True
            )
            # Direction arrow
            yaw = self.robot_yaw
            arrow_len = 0.4
            ax = rx + arrow_len * math.cos(yaw)
            ay = ry + arrow_len * math.sin(yaw)
            self._robot_arrow.setData(
                pos=np.array([[rx, ry, rz + 0.05], [ax, ay, rz + 0.05]]),
                color=pg.mkColor(255, 255, 255, 220),
                width=3.0,
                antialias=True
            )

        # 4) Path trail
        if self.path_history and len(self.path_history) > 1:
            path_key = len(self.path_history)
            if path_key != self._path_cache_key:
                self._path_cache_key = path_key
                pts = np.array([[p[0], p[1], 0.01] for p in self.path_history], dtype=np.float32)
                n = len(pts)
                # Fading indigo trail
                colors = np.zeros((n, 4), dtype=np.float32)
                colors[:, 0] = 0.4  # R
                colors[:, 1] = 0.4  # G
                colors[:, 2] = 0.95  # B
                colors[:, 3] = np.linspace(0.1, 0.9, n)  # Alpha: fade in
                self._path_item.setData(pos=pts, color=colors, width=2.0, antialias=True)

        # 5) Nav2 path
        if self.nav2_path and len(self.nav2_path) >= 2:
            nav_pts = np.array([[p[0], p[1], 0.02] for p in self.nav2_path
                               if len(p) >= 2], dtype=np.float32)
            if len(nav_pts) >= 2:
                self._nav2_path_item.setData(
                    pos=nav_pts,
                    color=pg.mkColor(0, 200, 255, 200),
                    width=2.5,
                    antialias=True
                )
        else:
            self._nav2_path_item.setData(pos=np.zeros((0, 3)))

        # 6) Goal marker
        goal = self.nav2_goal
        if goal and len(goal) >= 2:
            self._goal_item.setData(
                pos=np.array([[goal[0], goal[1], 0.05]]),
                color=np.array([[1.0, 0.65, 0.0, 1.0]]),
                size=16.0,
                pxMode=True
            )
        else:
            self._goal_item.setData(pos=np.zeros((0, 3)))

        # 7) Camera follow
        if self._follow_robot and self.has_pose:
            self.gl_widget.opts['center'] = pg.Vector(self.robot_x, self.robot_y, self.robot_z)

    def _update_occ_grid(self):
        """Build scatter plot for occupied cells in the occupancy grid."""
        if not self.grid_data:
            return
        try:
            grid = self.grid_data
            rows = len(grid)
            cols = len(grid[0]) if rows > 0 else 0
            res = self.grid_resolution
            ox, oy = self.grid_origin_x, self.grid_origin_y

            # Collect occupied and free cells
            occ_pts = []
            occ_colors = []
            for i in range(rows):
                row = grid[i]
                for j in range(cols):
                    v = row[j]
                    if v < 0:
                        continue  # skip unknown
                    wx = ox + j * res + res / 2
                    wy = oy + (rows - 1 - i) * res + res / 2
                    if v > 50:
                        # Occupied: red
                        occ_pts.append([wx, wy, 0.0])
                        occ_colors.append([0.9, 0.2, 0.15, 0.8])
                    elif v == 0:
                        # Free: dark green (subtle floor)
                        occ_pts.append([wx, wy, -0.01])
                        occ_colors.append([0.08, 0.18, 0.1, 0.35])

            if occ_pts:
                pts_arr = np.array(occ_pts, dtype=np.float32)
                col_arr = np.array(occ_colors, dtype=np.float32)
                # Size: each cell ~resolution meters, use pixel size
                self._occ_grid_item.setData(
                    pos=pts_arr,
                    color=col_arr,
                    size=max(3.0, res * 40),  # scale with zoom
                    pxMode=True
                )
            else:
                self._occ_grid_item.setData(pos=np.zeros((0, 3)))
        except Exception:
            pass

    def _toggle_grid(self, state):
        self._show_grid = (state == Qt.Checked)
        if not self._show_grid:
            self._occ_grid_item.setData(pos=np.zeros((0, 3)))

    def _toggle_cloud(self, state):
        self._show_cloud = (state == Qt.Checked)

    def set_jetson_ip(self, ip: str):
        self.jetson_ip = ip

    def closeEvent(self, event):
        if hasattr(self, '_receiver'):
            self._receiver.stop()
            self._receiver.wait(2000)
        if hasattr(self, '_render_timer'):
            self._render_timer.stop()
        super().closeEvent(event)

    def stop(self):
        """Stop receiver thread (called on app shutdown)."""
        if hasattr(self, '_receiver'):
            self._receiver.stop()
        if hasattr(self, '_render_timer'):
            self._render_timer.stop()
