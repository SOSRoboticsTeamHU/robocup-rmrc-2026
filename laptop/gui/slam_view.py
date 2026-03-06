"""
RoboCupRescue RMRC 2026 - SLAM View Widget (2D Map)
===================================================
Top-down 2D occupancy grid view with robot position and path.
For 3D visualization, use the RViz stream view.

Waymo-style colors: green = safe/free, red = danger/occupied, gray = unknown.
"""

import sys
import os
import math
from typing import Optional, List, Tuple, Dict, Any

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

try:
    from PyQt5.QtWidgets import (
        QWidget, QLabel, QVBoxLayout, QHBoxLayout, QFrame, QCheckBox, QPushButton,
        QSizePolicy,
    )
    from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF, QThread, pyqtSignal
    from PyQt5.QtGui import (
        QPainter, QColor, QPen, QBrush, QPolygonF, QFont, QImage, QWheelEvent,
        QMouseEvent,
    )
    PYQT_AVAILABLE = True
except ImportError:
    PYQT_AVAILABLE = False

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False

try:
    from shared.constants import ZMQ_PORT_LIDAR, ZMQ_PORT_SLAM, ZMQ_PORT_CLOUD, FIDUCIAL_POSITIONS
except ImportError:
    ZMQ_PORT_LIDAR = 5563
    ZMQ_PORT_SLAM = 5562
    ZMQ_PORT_CLOUD = 5564
    FIDUCIAL_POSITIONS = []


# Color palette
COLOR_BG = QColor(15, 15, 25)
COLOR_SAFE = QColor(34, 197, 94)       # green - free
COLOR_DANGER = QColor(239, 68, 68)     # red - occupied
COLOR_UNKNOWN = QColor(40, 50, 65)     # dark slate - unknown
COLOR_ROBOT = QColor(78, 204, 163)     # teal - robot
COLOR_ROBOT_EDGE = QColor(20, 60, 80)
COLOR_PATH = QColor(99, 102, 241)      # indigo - path

# Robot dimensions (meters)
ROBOT_LENGTH_M = 0.35
ROBOT_WIDTH_M = 0.25
ROBOT_HEIGHT_M = 0.20

# QR-on-map marker color
COLOR_QR_MAP = QColor(255, 165, 0)  # Orange
COLOR_FIDUCIAL = QColor(100, 149, 237)  # Cornflower blue


class SLAMReceiverThread(QThread):
    """Background thread for real-time SLAM/Lidar ZMQ receive."""
    slam_update = pyqtSignal(dict)
    lidar_update = pyqtSignal(dict)

    def __init__(self, jetson_ip: str, slam_port: int, lidar_port: int):
        super().__init__()
        self.jetson_ip = jetson_ip
        self.slam_port = slam_port
        self.lidar_port = lidar_port
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        if not ZMQ_AVAILABLE:
            return
        try:
            ctx = zmq.Context()
            sock_slam = ctx.socket(zmq.SUB)
            sock_slam.setsockopt(zmq.SUBSCRIBE, b"")
            sock_slam.setsockopt(zmq.RCVHWM, 1)
            sock_slam.connect(f"tcp://{self.jetson_ip}:{self.slam_port}")

            sock_lidar = sock_slam if self.slam_port == self.lidar_port else None
            if sock_lidar is None:
                sock_lidar = ctx.socket(zmq.SUB)
                sock_lidar.setsockopt(zmq.SUBSCRIBE, b"")
                sock_lidar.setsockopt(zmq.RCVHWM, 1)
                sock_lidar.connect(f"tcp://{self.jetson_ip}:{self.lidar_port}")

            poller = zmq.Poller()
            poller.register(sock_slam, zmq.POLLIN)
            if sock_lidar is not sock_slam:
                poller.register(sock_lidar, zmq.POLLIN)

            while self._running:
                socks = dict(poller.poll(timeout=5))  # 5 ms for lower SLAM latency
                if sock_slam in socks:
                    try:
                        msg = sock_slam.recv_json(zmq.NOBLOCK)
                        self.slam_update.emit(msg)
                    except Exception:
                        pass
                if sock_lidar is not sock_slam and sock_lidar in socks:
                    try:
                        msg = sock_lidar.recv_json(zmq.NOBLOCK)
                        self.lidar_update.emit(msg)
                    except Exception:
                        pass
            sock_slam.close()
            if sock_lidar is not sock_slam:
                sock_lidar.close()
            ctx.term()
        except Exception:
            pass


class SLAMView(QFrame):
    """2D SLAM map visualization widget."""

    def __init__(self, jetson_ip: str = "192.168.2.100", slam_port: int = None, 
                 lidar_port: int = None, parent=None):
        super().__init__(parent)
        self.setObjectName("CameraFrame")
        self.setMinimumSize(400, 300)
        self.setStyleSheet("""
            QFrame#CameraFrame {
                background-color: #0f0f19;
                border: 1px solid #0f3460;
                border-radius: 4px;
            }
        """)

        self.jetson_ip = jetson_ip
        self.slam_port = slam_port if slam_port is not None else ZMQ_PORT_SLAM
        self.lidar_port = lidar_port if lidar_port is not None else ZMQ_PORT_LIDAR
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_yaw = 0.0
        self.has_pose = False
        
        # Map data
        self.grid_data = None
        self.grid_resolution = 0.05
        self.grid_origin_x = -5.0
        self.grid_origin_y = -5.0
        
        # Path history
        self.path_history = []
        self.show_path = True
        
        # Lidar data
        self.lidar_ranges = []
        self.lidar_angles = []
        
        # View settings
        self.scale = 50.0  # pixels per meter
        self.zoom_factor = 1.0  # 1.0 = fit, >1 zoom in, <1 zoom out
        self.pan_x = 0.0
        self.pan_y = 0.0
        self._last_pan_pos = None

        # QR-on-map: list of {"data", "map_x", "map_y", ...} for drawing
        self.qr_on_map: List[Dict[str, Any]] = []

        # Nav2 overlay (autonomy mode) — set by main_window from autonomy_status
        self.nav2_path: List[List[float]] = []
        self.nav2_goal: Optional[Tuple[float, float]] = None
        self.goal_distance_m: Optional[float] = None
        # Ego-centric: True = heading-up (Waymo-style), False = North-up (classic)
        self.ego_mode = True
        # Cached grid pixmap (regenerate when grid_data changes)
        self._grid_pixmap: Optional[Any] = None
        self._grid_pixmap_key: Optional[tuple] = None

        # UI setup
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)
        
        # Header
        header = QHBoxLayout()
        self.title = QLabel("SLAM / Map")
        self.title.setStyleSheet("color: #4ecca3; font-weight: bold; font-size: 11px;")
        header.addWidget(self.title)
        header.addStretch()
        
        # Path toggle
        self.path_checkbox = QCheckBox("Path")
        self.path_checkbox.setChecked(True)
        self.path_checkbox.setStyleSheet("color: #a0a0a0; font-size: 10px;")
        self.path_checkbox.stateChanged.connect(self._on_path_toggle)
        header.addWidget(self.path_checkbox)
        # Heading-Up (ego) vs North-Up toggle
        self.north_up_btn = QPushButton("North-Up")
        self.north_up_btn.setCheckable(True)
        self.north_up_btn.setChecked(False)
        self.north_up_btn.setStyleSheet("color: #a0a0a0; font-size: 10px;")
        self.north_up_btn.clicked.connect(self._on_north_up_toggle)
        header.addWidget(self.north_up_btn)
        # Zoom controls
        zoom_btn_style = "color: #a0a0a0; font-size: 10px; padding: 2px 6px;"
        self.zoom_out_btn = QPushButton("−")
        self.zoom_out_btn.setStyleSheet(zoom_btn_style)
        self.zoom_out_btn.setToolTip("Zoom out")
        self.zoom_out_btn.clicked.connect(lambda: self._zoom_by(0.7))
        header.addWidget(self.zoom_out_btn)
        self.zoom_in_btn = QPushButton("+")
        self.zoom_in_btn.setStyleSheet(zoom_btn_style)
        self.zoom_in_btn.setToolTip("Zoom in")
        self.zoom_in_btn.clicked.connect(lambda: self._zoom_by(1.4))
        header.addWidget(self.zoom_in_btn)
        self.fit_btn = QPushButton("Fit")
        self.fit_btn.setStyleSheet(zoom_btn_style)
        self.fit_btn.setToolTip("Fit map to view")
        self.fit_btn.clicked.connect(self._zoom_fit)
        header.addWidget(self.fit_btn)
        layout.addLayout(header)
        
        # Canvas
        self.canvas = _SLAMCanvas(self)
        layout.addWidget(self.canvas, 1)
        
        # Status bar
        self.status_label = QLabel("Waiting for data...")
        self.status_label.setStyleSheet("color: #6b7280; font-size: 9px;")
        layout.addWidget(self.status_label)

        # Throttle canvas redraws to reduce SLAM latency (avoid queueing paint events)
        self._last_canvas_update = 0.0
        self._canvas_update_interval = 0.04  # 25 FPS max for map redraw

        # ZMQ: use background thread for real-time updates
        self._receiver_thread = None
        self._paused = False
        if ZMQ_AVAILABLE:
            self._setup_zmq()

    def pause(self):
        """Pause ZMQ receiver when view is inactive (performance)."""
        self._paused = True
        if self._receiver_thread:
            self._receiver_thread.stop()
            self._receiver_thread.wait(300)

    def resume(self):
        """Resume ZMQ receiver when view becomes active."""
        self._paused = False
        if ZMQ_AVAILABLE and (not self._receiver_thread or not self._receiver_thread.isRunning()):
            self._setup_zmq()

    def _on_path_toggle(self, state: int):
        self.show_path = state == Qt.Checked
        self.canvas.update()

    def _on_north_up_toggle(self):
        self.ego_mode = not self.north_up_btn.isChecked()
        self.canvas.update()

    def _zoom_by(self, factor: float):
        self.zoom_factor = max(0.25, min(4.0, self.zoom_factor * factor))
        self.canvas.update()

    def _zoom_fit(self):
        self.zoom_factor = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.canvas.update()

    def set_nav2_overlay(self, path: Optional[List[List[float]]] = None,
                        goal: Optional[Tuple[float, float]] = None,
                        distance_m: Optional[float] = None) -> None:
        """Set Nav2 path/goal for autonomy overlay (from autonomy_status)."""
        self.nav2_path = path or []
        self.nav2_goal = goal
        self.goal_distance_m = distance_m
        self.canvas.update()

    def _setup_zmq(self):
        try:
            self._receiver_thread = SLAMReceiverThread(
                self.jetson_ip, self.slam_port, self.lidar_port
            )
            self._receiver_thread.slam_update.connect(self._on_slam_data)
            self._receiver_thread.lidar_update.connect(self._on_lidar_data)
            self._receiver_thread.start()
        except Exception as e:
            self.title.setText(f"SLAM View (error)")

    def _on_slam_data(self, msg: dict):
        if getattr(self, "_paused", False):
            return
        import time as _time
        if "x" in msg:
            self.robot_x = float(msg.get("x", 0))
            self.robot_y = float(msg.get("y", 0))
            self.robot_z = float(msg.get("z", 0))
            self.robot_yaw = float(msg.get("yaw", 0))
            self.has_pose = True
        if "grid" in msg:
            self.grid_data = msg["grid"]
            self.grid_resolution = float(msg.get("resolution", 0.05))
            self.grid_origin_x = float(msg.get("origin_x", -5.0))
            self.grid_origin_y = float(msg.get("origin_y", -5.0))
        if "path" in msg:
            self.path_history = msg["path"]
        # Nav2 overlay (if embedded in SLAM message, e.g. from simulator or nav2_bridge)
        if "nav2_path" in msg:
            self.nav2_path = msg["nav2_path"] or []
        if "nav2_goal" in msg:
            g = msg["nav2_goal"]
            self.nav2_goal = tuple(g) if g and len(g) >= 2 else None
        if "goal_distance_m" in msg:
            self.goal_distance_m = msg["goal_distance_m"]
        if self.has_pose:
            self.status_label.setText(
                f"Pose: ({self.robot_x:.2f}, {self.robot_y:.2f}) yaw: {math.degrees(self.robot_yaw):.1f}°"
            )
        # Throttle redraws to reduce latency (fewer queued paint events)
        now = _time.time()
        if now - self._last_canvas_update >= self._canvas_update_interval:
            self._last_canvas_update = now
            self.canvas.update()

    def _on_lidar_data(self, msg: dict):
        if getattr(self, "_paused", False):
            return
        if "ranges" in msg:
            self.lidar_ranges = msg["ranges"]
            self.lidar_angles = msg.get("angles") or [
                math.radians(i * 360.0 / len(self.lidar_ranges))
                for i in range(len(self.lidar_ranges))
            ]
        self.canvas.update()

    def get_current_pose(self) -> Optional[Tuple[float, float]]:
        """Return current robot (x, y) in world frame for QR-on-map, or None if no pose."""
        if self.has_pose:
            return (self.robot_x, self.robot_y)
        return None

    def set_qr_on_map(self, entries: List[Dict[str, Any]]) -> None:
        """Set QR entries that have map positions for drawing (keys: data, map_x, map_y)."""
        self.qr_on_map = [e for e in entries if isinstance(e, dict) and "map_x" in e and "map_y" in e]
        self.canvas.update()

    def closeEvent(self, event):
        if self._receiver_thread:
            self._receiver_thread.stop()
            self._receiver_thread.wait(500)
        super().closeEvent(event)


class _SLAMCanvas(QWidget):
    """Canvas for drawing SLAM visualization (RViz-style occupancy grid rendering)."""

    # Grid color LUT: index 0-100 = occupancy value, 101 = unknown (-1)
    # RViz style: free=dark, occupied=bright, unknown=mid-gray
    _grid_lut = None

    @staticmethod
    def _build_grid_lut():
        """Build color lookup table for occupancy grid (like RViz map display)."""
        lut = [0] * (102 * 4)  # 102 entries × RGBA
        for v in range(101):
            if v == 0:
                # Free: dark background with subtle green tint
                lut[v * 4: v * 4 + 4] = [20, 28, 22, 255]
            elif v <= 30:
                # Low occupancy: blend dark→yellow-green
                t = v / 30.0
                lut[v * 4: v * 4 + 4] = [int(20 + 60 * t), int(28 + 50 * t), int(22 - 10 * t), 255]
            elif v <= 65:
                # Medium occupancy: yellow-orange
                t = (v - 30) / 35.0
                lut[v * 4: v * 4 + 4] = [int(80 + 120 * t), int(78 - 20 * t), int(12), 255]
            else:
                # High occupancy: red-white (like RViz occupied)
                t = (v - 65) / 35.0
                lut[v * 4: v * 4 + 4] = [int(200 + 55 * t), int(58 + 80 * t), int(12 + 50 * t), 255]
        # Unknown = -1 → index 101: dark slate
        lut[101 * 4: 102 * 4] = [25, 30, 40, 255]
        return lut

    def __init__(self, parent: SLAMView):
        super().__init__(parent)
        self.sv = parent
        self.setMinimumSize(400, 280)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setFocusPolicy(Qt.StrongFocus)
        if _SLAMCanvas._grid_lut is None:
            _SLAMCanvas._grid_lut = _SLAMCanvas._build_grid_lut()

    def wheelEvent(self, event: QWheelEvent):
        """Mouse wheel: zoom in/out."""
        delta = event.angleDelta().y()
        if delta > 0:
            self.sv._zoom_by(1.15)
        elif delta < 0:
            self.sv._zoom_by(1.0 / 1.15)
        event.accept()

    def mousePressEvent(self, event: QMouseEvent):
        """Start pan (North-Up mode) or track for context."""
        if event.button() == Qt.LeftButton and not self.sv.ego_mode:
            self.sv._last_pan_pos = event.pos()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent):
        """Pan when dragging in North-Up mode."""
        if self.sv._last_pan_pos is not None and not self.sv.ego_mode:
            ppm = self._current_ppm()
            dx = (event.pos().x() - self.sv._last_pan_pos.x()) / ppm
            dy = -(event.pos().y() - self.sv._last_pan_pos.y()) / ppm
            self.sv.pan_x += dx
            self.sv.pan_y += dy
            self.sv._last_pan_pos = event.pos()
            self.update()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() == Qt.LeftButton:
            self.sv._last_pan_pos = None
        super().mouseReleaseEvent(event)

    def _current_ppm(self) -> float:
        """Current pixels-per-meter for pan delta."""
        sv = self.sv
        grid = sv.grid_data
        rows = len(grid) if grid else 0
        cols = len(grid[0]) if grid and grid[0] else 0
        res = sv.grid_resolution
        max_dim = max(cols * res if cols else 10, rows * res if rows else 10, 1.0)
        base = min(self.width(), self.height()) * 0.85 / max_dim
        return base * getattr(sv, "zoom_factor", 1.0)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        w, h = self.width(), self.height()

        painter.fillRect(0, 0, w, h, COLOR_BG)

        if not self.sv.has_pose and not self.sv.grid_data and not self.sv.lidar_ranges:
            painter.setPen(COLOR_ROBOT)
            painter.drawText(0, 0, w, h, Qt.AlignCenter, "Waiting for SLAM / Lidar...")
            return

        self._paint_top_down(painter, w, h)

    def _paint_top_down(self, painter, w, h):
        """Waymo-style ego-centric top-down view: robot at center, heading up. Optional North-Up."""
        sv = self.sv
        cx, cy = w / 2.0, h / 2.0
        grid = sv.grid_data
        rows = len(grid) if grid else 0
        cols = len(grid[0]) if grid and grid[0] else 0
        res = sv.grid_resolution
        # Pixels per meter (scale to fit, then apply user zoom)
        grid_width_m = cols * res if cols else 10.0
        grid_height_m = rows * res if rows else 10.0
        max_dim = max(grid_width_m, grid_height_m, 1.0)
        base_ppm = min(w, h) * 0.85 / max_dim
        ppm = base_ppm * getattr(sv, "zoom_factor", 1.0)

        # --- Ego-centric transform: robot at center, heading up ---
        painter.save()
        painter.translate(cx, cy)
        if sv.ego_mode:
            painter.rotate(-math.degrees(sv.robot_yaw))
        painter.scale(ppm, -ppm)  # world meters → pixels, flip Y
        pan_x = getattr(sv, "pan_x", 0.0)
        pan_y = getattr(sv, "pan_y", 0.0)
        painter.translate(-sv.robot_x + pan_x, -sv.robot_y + pan_y)
        # Everything below is in world coordinates (meters)

        # 1) Occupancy grid as QImage (1 pixel per cell, like RViz)
        if grid and rows > 0 and cols > 0:
            key = (id(sv.grid_data), rows, cols)
            if getattr(sv, "_grid_img_key", None) != key:
                sv._grid_img_key = key
                sv._grid_qimage = self._render_grid_to_qimage(grid, rows, cols)
            if sv._grid_qimage is not None:
                # Grid origin in world coords; grid covers origin to origin + size
                gx = sv.grid_origin_x
                gy = sv.grid_origin_y
                gw = cols * res
                gh = rows * res
                # drawImage maps the image into the target QRectF in current (world) coords
                painter.drawImage(QRectF(gx, gy, gw, gh), sv._grid_qimage)

        # 2) Path trail (fading indigo) — use QPointF for float precision
        if sv.show_path and sv.path_history:
            n = len(sv.path_history)
            for i in range(1, n):
                try:
                    x0, y0 = sv.path_history[i - 1]
                    x1, y1 = sv.path_history[i]
                    alpha = 80 + int(170 * (i / max(n, 1)))
                    pen = QPen(QColor(99, 102, 241, min(255, alpha)))
                    pen.setWidthF(0.03)  # 3cm in world coords
                    pen.setCosmetic(False)
                    painter.setPen(pen)
                    painter.drawLine(QPointF(x0, y0), QPointF(x1, y1))
                except Exception:
                    pass

        # 3) Nav2 path (cyan dashed) — autonomy overlay
        nav2_path = getattr(sv, "nav2_path", []) or []
        if nav2_path and len(nav2_path) >= 2:
            pen = QPen(QColor(0, 200, 255), 0.04)
            pen.setStyle(Qt.DashLine)
            pen.setCosmetic(False)
            painter.setPen(pen)
            for i in range(1, len(nav2_path)):
                a, b = nav2_path[i - 1], nav2_path[i]
                if len(a) >= 2 and len(b) >= 2:
                    painter.drawLine(QPointF(a[0], a[1]), QPointF(b[0], b[1]))

        # 4) QR-on-map and fiducials (world coords)
        for qr in getattr(sv, "qr_on_map", []) or []:
            mx, my = qr.get("map_x"), qr.get("map_y")
            if mx is None or my is None:
                continue
            pen = QPen(COLOR_QR_MAP); pen.setWidthF(0.02); pen.setCosmetic(False)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QRectF(mx - 0.15, my - 0.15, 0.3, 0.3))
        try:
            from shared.constants import FIDUCIAL_POSITIONS
            fiducials = FIDUCIAL_POSITIONS
        except ImportError:
            fiducials = []
        for (fx, fy) in (fiducials or []):
            pen = QPen(COLOR_FIDUCIAL); pen.setWidthF(0.02); pen.setCosmetic(False)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QRectF(fx - 0.12, fy - 0.12, 0.24, 0.24))

        # 5) Goal marker (pulsing orange) — autonomy overlay
        goal = getattr(sv, "nav2_goal", None)
        if goal and len(goal) >= 2:
            gx, gy = goal[0], goal[1]
            from PyQt5.QtCore import QTime
            t = (QTime.currentTime().msecsSinceStartOfDay() % 1000) / 1000.0
            r = 0.2 + 0.05 * (0.5 + 0.5 * math.sin(t * 6.28))
            pen = QPen(QColor(255, 165, 0)); pen.setWidthF(0.03); pen.setCosmetic(False)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QRectF(gx - r, gy - r, 2 * r, 2 * r))

        # 6) Robot icon at (robot_x, robot_y) — always at screen center due to transform
        self._draw_robot_topdown(painter, sv.robot_x, sv.robot_y, 0.0 if sv.ego_mode else -sv.robot_yaw)

        painter.restore()

        # --- Screen-space overlays (after restoring world transform) ---
        painter.save()
        painter.resetTransform()
        # Distance rings (1m, 2m, 3m)
        for radius_m in (1.0, 2.0, 3.0):
            r_px = radius_m * ppm
            painter.setPen(QPen(QColor(60, 60, 80, 120), 1, Qt.DotLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QRectF(cx - r_px, cy - r_px, 2 * r_px, 2 * r_px))
            painter.setPen(QColor(80, 80, 100))
            painter.setFont(QFont("Sans", 7))
            painter.drawText(int(cx + r_px + 2), int(cy - 2), f"{radius_m:.0f}m")
        # Cardinal labels
        cardinals = [("N", 0, -1), ("S", 0, 1), ("E", 1, 0), ("W", -1, 0)]
        painter.setFont(QFont("Sans", 9, QFont.Bold))
        for label, dx, dy in cardinals:
            lx = cx + dx * (min(w, h) * 0.42)
            ly = cy + dy * (min(w, h) * 0.42)
            painter.setPen(QColor(100, 100, 120))
            painter.drawText(int(lx - 5), int(ly + 4), label)
        # Scale bar
        self._draw_scale_bar(painter, w, h, ppm)
        # Goal distance label (screen space, bottom-right)
        goal = getattr(sv, "nav2_goal", None)
        dist = getattr(sv, "goal_distance_m", None)
        if goal and dist is not None:
            painter.setPen(QColor(255, 165, 0))
            painter.setFont(QFont("Sans", 10, QFont.Bold))
            painter.drawText(w - 100, h - 10, f"Goal: {dist:.1f}m")
        painter.restore()

        if not grid and not sv.path_history:
            painter.setPen(COLOR_ROBOT)
            painter.drawText(0, 0, w, h, Qt.AlignCenter, "No map data")

    def _render_grid_to_qimage(self, grid, rows, cols):
        """Render occupancy grid to QImage — 1 pixel per cell (like RViz OccupancyGrid display).
        Row 0 in grid data = top of grid (highest y). QImage row 0 = top pixel.
        Since painter has y-flipped transform, we flip rows so grid[0] maps to highest y.
        """
        try:
            lut = _SLAMCanvas._grid_lut
            img = QImage(cols, rows, QImage.Format_RGBA8888)
            for i in range(rows):
                for j in range(cols):
                    v = grid[i][j]
                    idx = max(0, min(100, v)) if v >= 0 else 101
                    # Flip vertically: grid row 0 = highest y = bottom of QImage
                    img_row = rows - 1 - i
                    r = lut[idx * 4]
                    g = lut[idx * 4 + 1]
                    b = lut[idx * 4 + 2]
                    img.setPixel(j, img_row, QColor(r, g, b).rgb())
            return img
        except Exception:
            return None

    def _draw_robot_topdown(self, painter, x, y, yaw):
        """Draw robot footprint in world coords. Uses QPointF for float precision."""
        L = ROBOT_LENGTH_M
        W = ROBOT_WIDTH_M
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        corners = [
            (x + (-L/2) * cos_y - (-W/2) * sin_y, y + (-L/2) * sin_y + (-W/2) * cos_y),
            (x + ( L/2) * cos_y - (-W/2) * sin_y, y + ( L/2) * sin_y + (-W/2) * cos_y),
            (x + ( L/2) * cos_y - ( W/2) * sin_y, y + ( L/2) * sin_y + ( W/2) * cos_y),
            (x + (-L/2) * cos_y - ( W/2) * sin_y, y + (-L/2) * sin_y + ( W/2) * cos_y),
        ]
        poly = QPolygonF([QPointF(c[0], c[1]) for c in corners])
        pen = QPen(COLOR_ROBOT_EDGE); pen.setWidthF(0.02); pen.setCosmetic(False)
        painter.setPen(pen)
        painter.setBrush(COLOR_ROBOT)
        painter.drawPolygon(poly)
        # Nose direction indicator
        nose_x = x + (L/2 + 0.06) * cos_y
        nose_y = y + (L/2 + 0.06) * sin_y
        pen2 = QPen(QColor(255, 255, 255)); pen2.setWidthF(0.025); pen2.setCosmetic(False)
        painter.setPen(pen2)
        painter.drawLine(QPointF(x, y), QPointF(nose_x, nose_y))

    def _draw_scale_bar(self, painter, w, h, ppm):
        """Draw scale bar in screen coords (call after resetTransform)."""
        bar_m = 1.0
        bar_px = bar_m * ppm
        if bar_px > w * 0.4:
            bar_m = 0.5
            bar_px = bar_m * ppm
        x0, y0 = 15, h - 25
        painter.setPen(QPen(QColor(150, 150, 150), 2))
        painter.drawLine(x0, y0, int(x0 + bar_px), y0)
        painter.drawLine(x0, y0 - 4, x0, y0 + 4)
        painter.drawLine(int(x0 + bar_px), y0 - 4, int(x0 + bar_px), y0 + 4)
        painter.setFont(QFont("Arial", 9))
        painter.drawText(x0, y0 - 8, f"{bar_m}m")
