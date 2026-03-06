"""
RoboCupRescue RMRC 2026 - 3D SLAM View (RViz-style)
====================================================
QOpenGLWidget-based 3D visualization matching Point-LIO RViz:
- Fixed 3rd-person camera (world space, not ego-centric)
- Rainbow height-colored point cloud from /cloud_registered
- Robot model from URDF (simple primitives)
- Dark RViz-like background

Uses PyOpenGL + PyOpenGL_accelerate. Data via ZMQ (cloud + slam ports).
"""

import os
import math
import sys
import threading
import time
from typing import Optional, List, Tuple

# Max points for smooth FPS (optimized for 10k)
MAX_POINT_CLOUD_POINTS = 10000
POINT_SIZE = 2.75  # 2.5-3.0 range

sys_path_insert = os.path.join(os.path.dirname(__file__), "..", "..")
if sys_path_insert not in sys.path:
    sys.path.insert(0, sys_path_insert)

try:
    from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
    from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
    from PyQt5.QtGui import QSurfaceFormat
    PYQT_AVAILABLE = True
except ImportError:
    QWidget = object  # type: ignore
    QThread = None  # type: ignore
    QTimer = None  # type: ignore
    pyqtSignal = None  # type: ignore
    PYQT_AVAILABLE = False

try:
    from PyQt5.QtOpenGL import QOpenGLWidget
    OPENGL_WIDGET_AVAILABLE = True
except ImportError:
    QOpenGLWidget = None  # type: ignore
    OPENGL_WIDGET_AVAILABLE = False

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
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

# Require PyQt5 - main_window will catch ImportError and use fallback
if not PYQT_AVAILABLE:
    raise ImportError("PyQt5 required for three_d_slam_view")

# Robot dimensions from rescue_robot.urdf (meters)
ROBOT_BASE_L = 0.27
ROBOT_BASE_W = 0.25
ROBOT_BASE_H = 0.10
ROBOT_WHEEL_R = 0.04
ROBOT_WHEEL_L = 0.02
ROBOT_LIDAR_R = 0.035
ROBOT_LIDAR_L = 0.04
# Wheel positions relative to base center (URDF: x=±0.09, y=±0.135)
WHEEL_FL = (0.09, 0.135, 0)
WHEEL_FR = (0.09, -0.135, 0)
WHEEL_RL = (-0.09, 0.135, 0)
WHEEL_RR = (-0.09, -0.135, 0)
LIDAR_POS = (0.08, 0, 0.10)  # relative to base center
BASE_ORIGIN_Z = 0.05  # box origin at z=0.05 (half height)


# Floor plane: z < 0.1 → orange (simple ground segmentation)
FLOOR_Z_THRESHOLD = 0.1
# Orange for floor (video ground segmentation)
FLOOR_COLOR = (0.95, 0.55, 0.15)


def _point_color(z: float, z_min: float, z_max: float, is_floor: bool) -> Tuple[float, float, float]:
    """Floor plane = orange. Other points = RViz rainbow (blue->green->yellow->red)."""
    if is_floor:
        return FLOOR_COLOR
    t = max(0, min(1, (z - z_min) / max(z_max - z_min, 0.01)))
    if t < 0.25:
        r, g, b = 0, t * 4, 1
    elif t < 0.5:
        r, g, b = 0, 1, 1 - (t - 0.25) * 4
    elif t < 0.75:
        r, g, b = (t - 0.5) * 4, 1, 0
    else:
        r, g, b = 1, 1 - (t - 0.75) * 4, 0
    return (r, g, b)


if PYQT_AVAILABLE and ZMQ_AVAILABLE:
    class _3DSLAMReceiverThread(QThread):
        """Background thread receiving cloud + slam over ZMQ."""
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
            try:
                ctx = zmq.Context()
                sock_cloud = ctx.socket(zmq.SUB)
                sock_cloud.setsockopt(zmq.SUBSCRIBE, b"")
                sock_cloud.setsockopt(zmq.RCVHWM, 2)
                sock_cloud.connect(f"tcp://{self.jetson_ip}:{self.cloud_port}")
                sock_slam = ctx.socket(zmq.SUB)
                sock_slam.setsockopt(zmq.SUBSCRIBE, b"")
                sock_slam.setsockopt(zmq.RCVHWM, 2)
                sock_slam.connect(f"tcp://{self.jetson_ip}:{self.slam_port}")
                poller = zmq.Poller()
                poller.register(sock_cloud, zmq.POLLIN)
                poller.register(sock_slam, zmq.POLLIN)
                while self._running:
                    socks = dict(poller.poll(timeout=10))
                    if sock_cloud in socks:
                        try:
                            self.cloud_update.emit(sock_cloud.recv_json(zmq.NOBLOCK))
                        except Exception:
                            pass
                    if sock_slam in socks:
                        try:
                            self.slam_update.emit(sock_slam.recv_json(zmq.NOBLOCK))
                        except Exception:
                            pass
            except Exception:
                pass
else:
    class _3DSLAMReceiverThread:
        """Stub when PyQt5 or zmq not available."""
        cloud_update = None
        slam_update = None

        def __init__(self, jetson_ip: str, cloud_port: int, slam_port: int):
            pass

        def stop(self):
            pass

        def run(self):
            pass

        def start(self):
            pass

        def wait(self, _ms: int = 0):
            pass


def _create_3d_slam_view_base():
    """Return the appropriate base class for _3DSLAMView."""
    if OPENGL_WIDGET_AVAILABLE and QOpenGLWidget:
        return QOpenGLWidget
    if PYQT_AVAILABLE:
        return QWidget
    return object


class _3DSLAMView(_create_3d_slam_view_base()):
    """
    RViz-style 3D SLAM view: fixed camera, rainbow point cloud, robot model.
    Camera: ~8m distance, ~6m height, ~35° from horizontal, looking at map origin.
    """

    def __init__(self, jetson_ip: str = "192.168.2.100", parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._cloud_xyz: List[List[float]] = []
        self._path_history: List[Tuple[float, float]] = []
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._arm_joints: Optional[List[float]] = None
        self._lock = threading.Lock()
        self._z_min = -0.3
        self._z_max = 1.5
        self._receiver_thread: Optional[_3DSLAMReceiverThread] = None
        self.jetson_ip = jetson_ip
        self._paint_timer = None
        self._gl_ready = OPENGL_WIDGET_AVAILABLE and OPENGL_AVAILABLE and NUMPY_AVAILABLE

        if not self._gl_ready:
            layout = QVBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            missing = []
            if not OPENGL_WIDGET_AVAILABLE:
                missing.append("PyQt5.QtOpenGL")
            if not OPENGL_AVAILABLE:
                missing.append("PyOpenGL")
            if not NUMPY_AVAILABLE:
                missing.append("numpy")
            msg = "3D SLAM requires: pip install PyOpenGL PyOpenGL-accelerate numpy"
            if missing:
                msg += f"\n\nMissing: {', '.join(missing)}"
            lbl = QLabel(msg)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("color: #888; background: #0a0a14; font-size: 11px;")
            lbl.setWordWrap(True)
            layout.addWidget(lbl)
            return

        # Request compatibility profile for gluLookAt/gluPerspective
        fmt = QSurfaceFormat()
        fmt.setProfile(QSurfaceFormat.CompatibilityProfile)
        fmt.setVersion(2, 1)
        self.setFormat(fmt)

        self._receiver_thread = _3DSLAMReceiverThread(
            jetson_ip, ZMQ_PORT_CLOUD, ZMQ_PORT_SLAM
        )
        self._receiver_thread.cloud_update.connect(self._on_cloud_msg)
        self._receiver_thread.slam_update.connect(self._on_slam_msg)
        self._receiver_thread.start()

        # Throttle repaints for 25+ FPS
        self._paint_timer = QTimer(self)
        self._paint_timer.timeout.connect(self.update)
        self._paint_timer.start(40)  # 25 Hz

        # Camera params (distance m, height m, azimuth deg) - user-adjustable
        self._cam_distance = 8.0
        self._cam_height = 6.0
        self._cam_angle = 0.0  # azimuth: 0=from -Y, 90=from +X
        self._cam_target_dist = 8.0
        self._cam_target_height = 6.0
        self._cam_target_angle = 0.0
        self._paused = False
        self._has_pose = False
        self._fps_frames = 0
        self._fps_last_time = time.time()
        self._fps_value = 0

    def set_camera_params(self, distance: float, height: float, angle_deg: float):
        """Set camera position: distance (m), height (m), azimuth angle (deg)."""
        self._cam_target_dist = max(1.0, min(25.0, distance))
        self._cam_target_height = max(0.5, min(15.0, height))
        self._cam_target_angle = angle_deg % 360.0

    def pause(self):
        """Pause repaints for performance when view is inactive. ZMQ receiver keeps data fresh."""
        self._paused = True
        if self._paint_timer:
            self._paint_timer.stop()
        if self._receiver_thread:
            self._receiver_thread.stop()
            self._receiver_thread.wait(300)

    def resume(self):
        """Resume when view becomes active. Restarts ZMQ receiver if it was stopped."""
        self._paused = False
        if self._receiver_thread and not self._receiver_thread.isRunning():
            self._receiver_thread = _3DSLAMReceiverThread(
                self.jetson_ip, ZMQ_PORT_CLOUD, ZMQ_PORT_SLAM
            )
            self._receiver_thread.cloud_update.connect(self._on_cloud_msg)
            self._receiver_thread.slam_update.connect(self._on_slam_msg)
            self._receiver_thread.start()
        if self._paint_timer:
            self._paint_timer.start(40)

        # FPS label overlay (top-left corner, green text)
        self._fps_label = QLabel("0 FPS", self)
        self._fps_label.setStyleSheet("color: #4ecca3; font-size: 10px; background: transparent;")
        self._fps_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        self._fps_timer = QTimer(self)
        self._fps_timer.timeout.connect(self._update_fps_label)
        self._fps_timer.start(500)

    def _update_fps_label(self):
        fps = getattr(self, "_fps_value", 0)
        if hasattr(self, "_fps_label"):
            self._fps_label.setText(f"{fps} FPS")
            self._fps_label.adjustSize()
            self._fps_label.move(8, 6)  # Top-left corner

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "_fps_label") and self._fps_label:
            self._fps_label.move(8, 6)  # Top-left corner

    def _on_cloud_msg(self, msg: dict):
        pts = msg.get("points") or msg.get("cloud_xyz") or []
        if pts:
            if len(pts) > MAX_POINT_CLOUD_POINTS and NUMPY_AVAILABLE:
                arr = np.array(pts, dtype=np.float32)
                idx = np.random.choice(len(arr), MAX_POINT_CLOUD_POINTS, replace=False)
                pts = arr[idx].tolist()
            with self._lock:
                self._cloud_xyz = list(pts)

    def _on_slam_msg(self, msg: dict):
        with self._lock:
            if "x" in msg:
                self._robot_x = float(msg.get("x", 0))
                self._robot_y = float(msg.get("y", 0))
                self._robot_yaw = float(msg.get("yaw", 0))
                self._has_pose = True
            if "path" in msg:
                self._path_history = [(float(p[0]), float(p[1])) for p in msg["path"] if len(p) >= 2][-100:]

    @staticmethod
    def is_gl_ready() -> bool:
        """Return True if OpenGL 3D view can render (for main_window fallback)."""
        return bool(OPENGL_WIDGET_AVAILABLE and OPENGL_AVAILABLE and NUMPY_AVAILABLE)

    def get_current_pose(self):
        """Return (x, y) for waypoint/fiducial compatibility, or None if no pose yet."""
        with self._lock:
            if not getattr(self, "_has_pose", False):
                return None
            return (self._robot_x, self._robot_y)

    def update_3d_data(
        self,
        cloud_xyz: Optional[List] = None,
        robot_x: Optional[float] = None,
        robot_y: Optional[float] = None,
        robot_yaw: Optional[float] = None,
        arm_joints: Optional[List[float]] = None,
        path: Optional[List] = None,
    ):
        """Public API: update data from external source (e.g. ZMQ callback)."""
        with self._lock:
            if path is not None:
                self._path_history = [(float(p[0]), float(p[1])) for p in path if len(p) >= 2][-100:]
            if cloud_xyz is not None:
                pts = list(cloud_xyz)
                if len(pts) > MAX_POINT_CLOUD_POINTS and NUMPY_AVAILABLE:
                    arr = np.array(pts, dtype=np.float32)
                    idx = np.random.choice(len(arr), MAX_POINT_CLOUD_POINTS, replace=False)
                    pts = arr[idx].tolist()
                self._cloud_xyz = pts
            if robot_x is not None:
                self._robot_x = float(robot_x)
                self._has_pose = True
            if robot_y is not None:
                self._robot_y = float(robot_y)
                self._has_pose = True
            if robot_yaw is not None:
                self._robot_yaw = float(robot_yaw)
            if arm_joints is not None:
                self._arm_joints = list(arm_joints)

    def initializeGL(self):
        if not (OPENGL_WIDGET_AVAILABLE and OPENGL_AVAILABLE):
            return
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_POINT_SMOOTH)
        glHint(GL_POINT_SMOOTH_HINT, GL_NICEST)
        glPointSize(POINT_SIZE)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, (5, 5, 10, 1))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.3, 0.3, 0.3, 1))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.7, 0.7, 0.7, 1))

    def resizeGL(self, w: int, h: int):
        if not (OPENGL_WIDGET_AVAILABLE and OPENGL_AVAILABLE):
            return
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = w / max(h, 1)
        gluPerspective(45, aspect, 0.5, 50)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        if not (OPENGL_WIDGET_AVAILABLE and OPENGL_AVAILABLE):
            return
        # FPS counter
        self._fps_frames += 1
        now = time.time()
        if now - self._fps_last_time >= 1.0:
            self._fps_value = self._fps_frames
            self._fps_frames = 0
            self._fps_last_time = now

        glClearColor(0.067, 0.067, 0.067, 1)  # #111111 dark RViz background
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # Smooth camera orbit: lerp toward target for smoother motion
        tdist = getattr(self, "_cam_target_dist", 8.0)
        tht = getattr(self, "_cam_target_height", 6.0)
        tang = getattr(self, "_cam_target_angle", 0.0)
        self._cam_distance += (tdist - self._cam_distance) * 0.12
        self._cam_height += (tht - self._cam_height) * 0.12
        self._cam_angle += (tang - self._cam_angle) * 0.12
        dist = self._cam_distance
        ht = self._cam_height
        ang = math.radians(self._cam_angle)
        cx = dist * math.sin(ang)
        cy = -dist * math.cos(ang)
        gluLookAt(cx, cy, ht, 0, 0, 0, 0, 0, 1)

        with self._lock:
            cloud = list(self._cloud_xyz)
            path = list(self._path_history)
            rx, ry, ryaw = self._robot_x, self._robot_y, self._robot_yaw

        # Ground grid (subtle, like RViz)
        self._draw_ground_grid()

        # Coordinate axes at origin (small)
        self._draw_axes()

        # Thin blue path trail (faint, like video)
        self._draw_path_trail(path)

        # Point cloud: floor plane orange, others rainbow (blue-green-yellow-red)
        if cloud and NUMPY_AVAILABLE:
            arr = np.array(cloud, dtype=np.float32)
            if arr.size >= 3:
                z_vals = arr[:, 2]
                z_min = float(np.min(z_vals)) if len(z_vals) else -0.3
                z_max = float(np.max(z_vals)) if len(z_vals) else 1.5
                self._z_min = min(self._z_min, z_min)
                self._z_max = max(self._z_max, z_max)
                glDisable(GL_LIGHTING)
                glBegin(GL_POINTS)
                for i in range(len(arr)):
                    z = arr[i, 2]
                    is_floor = z < FLOOR_Z_THRESHOLD
                    r, g, b = _point_color(z, self._z_min, self._z_max, is_floor)
                    glColor3f(r, g, b)
                    glVertex3f(arr[i, 0], arr[i, 1], arr[i, 2])
                glEnd()
                glEnable(GL_LIGHTING)

        # Robot model: red base, black wheels, blue arm (match user's RViz)
        self._draw_robot(rx, ry, ryaw)

    def _draw_robot(self, x: float, y: float, yaw: float):
        glPushMatrix()
        glTranslatef(x, y, 0)
        glRotatef(math.degrees(yaw), 0, 0, 1)

        # Base box: red (match user's RViz)
        glPushMatrix()
        glTranslatef(0, 0, BASE_ORIGIN_Z)
        glColor3f(0.85, 0.2, 0.15)
        self._draw_box(ROBOT_BASE_L, ROBOT_BASE_W, ROBOT_BASE_H)
        glPopMatrix()

        # Arm/lidar: blue (match user's RViz)
        glPushMatrix()
        glTranslatef(LIDAR_POS[0], LIDAR_POS[1], LIDAR_POS[2])
        glRotatef(90, 1, 0, 0)
        glColor3f(0.2, 0.4, 0.9)
        self._draw_cylinder(ROBOT_LIDAR_R, ROBOT_LIDAR_L)
        glPopMatrix()

        # Wheels: black (match URDF)
        glColor3f(0.08, 0.08, 0.08)
        for wx, wy, wz in [WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR]:
            glPushMatrix()
            glTranslatef(wx, wy, wz)
            glRotatef(90, 1, 0, 0)
            self._draw_cylinder(ROBOT_WHEEL_R, ROBOT_WHEEL_L)
            glPopMatrix()

        glPopMatrix()

        # TODO: Add full URDF mesh loading (assimp) for arm and detailed geometry

    def _draw_ground_grid(self):
        """Faint ground grid on z=0 (like RViz)."""
        glDisable(GL_LIGHTING)
        glColor4f(0.12, 0.12, 0.16, 0.35)
        glLineWidth(1.0)
        size = 10.0
        step = 0.5
        glBegin(GL_LINES)
        for i in range(int(-size / step), int(size / step) + 1):
            x = i * step
            glVertex3f(x, -size, 0)
            glVertex3f(x, size, 0)
            glVertex3f(-size, x, 0)
            glVertex3f(size, x, 0)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_path_trail(self, path: List[Tuple[float, float]]):
        """Thin blue path trail: opacity 0.7, width 0.05m in world units (ribbon)."""
        if len(path) < 2:
            return
        glDisable(GL_LIGHTING)
        glColor4f(0.25, 0.5, 0.9, 0.7)  # Blue, opacity 0.7
        half_w = 0.025  # 0.05m total width in world units
        z = 0.01
        glBegin(GL_QUAD_STRIP)
        for i in range(len(path)):
            x, y = path[i][0], path[i][1]
            if i < len(path) - 1:
                dx = path[i + 1][0] - x
                dy = path[i + 1][1] - y
            else:
                dx = path[i][0] - path[i - 1][0]
                dy = path[i][1] - path[i - 1][1]
            L = math.sqrt(dx * dx + dy * dy) or 1e-6
            nx, ny = -dy / L, dx / L
            glVertex3f(x + nx * half_w, y + ny * half_w, z)
            glVertex3f(x - nx * half_w, y - ny * half_w, z)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_axes(self):
        """Small coordinate axes at origin: red X, green Y, blue Z."""
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        s = 0.25
        glBegin(GL_LINES)
        glColor3f(0.9, 0.2, 0.2)
        glVertex3f(0, 0, 0)
        glVertex3f(s, 0, 0)
        glColor3f(0.2, 0.8, 0.2)
        glVertex3f(0, 0, 0)
        glVertex3f(0, s, 0)
        glColor3f(0.2, 0.4, 0.95)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, s)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_box(self, lx: float, ly: float, lz: float):
        """Draw box centered at origin, half-extents."""
        hx, hy, hz = lx / 2, ly / 2, lz / 2
        glBegin(GL_QUADS)
        # +X
        glNormal3f(1, 0, 0)
        glVertex3f(hx, -hy, -hz); glVertex3f(hx, hy, -hz); glVertex3f(hx, hy, hz); glVertex3f(hx, -hy, hz)
        # -X
        glNormal3f(-1, 0, 0)
        glVertex3f(-hx, -hy, hz); glVertex3f(-hx, hy, hz); glVertex3f(-hx, hy, -hz); glVertex3f(-hx, -hy, -hz)
        # +Y
        glNormal3f(0, 1, 0)
        glVertex3f(-hx, hy, -hz); glVertex3f(hx, hy, -hz); glVertex3f(hx, hy, hz); glVertex3f(-hx, hy, hz)
        # -Y
        glNormal3f(0, -1, 0)
        glVertex3f(-hx, -hy, hz); glVertex3f(hx, -hy, hz); glVertex3f(hx, -hy, -hz); glVertex3f(-hx, -hy, -hz)
        # +Z
        glNormal3f(0, 0, 1)
        glVertex3f(-hx, -hy, hz); glVertex3f(-hx, hy, hz); glVertex3f(hx, hy, hz); glVertex3f(hx, -hy, hz)
        # -Z
        glNormal3f(0, 0, -1)
        glVertex3f(-hx, -hy, -hz); glVertex3f(hx, -hy, -hz); glVertex3f(hx, hy, -hz); glVertex3f(-hx, hy, -hz)
        glEnd()

    def _draw_cylinder(self, radius: float, length: float, slices: int = 16):
        """Draw cylinder along Z, centered at origin."""
        glBegin(GL_QUAD_STRIP)
        for i in range(slices + 1):
            a = 2 * math.pi * i / slices
            nx, ny = math.cos(a), math.sin(a)
            glNormal3f(nx, ny, 0)
            glVertex3f(radius * nx, radius * ny, -length / 2)
            glVertex3f(radius * nx, radius * ny, length / 2)
        glEnd()
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0, 0, -1)
        glVertex3f(0, 0, -length / 2)
        for i in range(slices + 1):
            a = 2 * math.pi * i / slices
            glVertex3f(radius * math.cos(a), radius * math.sin(a), -length / 2)
        glEnd()
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0, 0, 1)
        glVertex3f(0, 0, length / 2)
        for i in range(slices + 1):
            a = 2 * math.pi * (slices - i) / slices
            glVertex3f(radius * math.cos(a), radius * math.sin(a), length / 2)
        glEnd()

    def stop(self):
        if getattr(self, "_receiver_thread", None):
            self._receiver_thread.stop()
            self._receiver_thread.wait(500)
        if getattr(self, "_paint_timer", None):
            self._paint_timer.stop()
        if getattr(self, "_fps_timer", None):
            self._fps_timer.stop()
