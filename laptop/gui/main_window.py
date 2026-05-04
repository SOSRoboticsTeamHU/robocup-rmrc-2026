#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Operator Station GUI
==============================================
Waymo-inspired cockpit interface for robot control.

Features:
- Dark theme with gradient blue-gray
- Top status HUD (mission state, alerts)
- Central SLAM view (2D/3D toggle)
- 3-camera grid (front, arm, backward) with YOLO/QR overlays
- Control buttons (Start Mission, Stop, Report)
- Joystick status display
"""

import sys
import time
import json
import threading
import subprocess
from typing import Optional, Dict, Any
from dataclasses import dataclass

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QFrame, QSplitter,
    QStatusBar, QGroupBox, QProgressBar, QStackedWidget, QCheckBox,
    QComboBox, QScrollArea, QSlider, QShortcut, QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QPalette, QColor, QImage, QPixmap, QPainter, QKeySequence

import numpy as np

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# Import local modules
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'shared'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'control'))

try:
    from shared.constants import (
        JETSON_IP, ZMQ_PORT_CAMERA, ZMQ_PORT_CAMERA_CONTROL, ZMQ_PORT_STATUS,
        ZMQ_PORT_DRIVE, ZMQ_PORT_ARM, ZMQ_PORT_AUTONOMY, ZMQ_PORT_AUTONOMY_STATUS,
        ZMQ_PORT_SNAPSHOT_RESULT, ZMQ_PORT_SLAM, MISSION_DURATION
    )
except ImportError:
    JETSON_IP = "192.168.2.100"
    ZMQ_PORT_CAMERA = 5557
    ZMQ_PORT_CAMERA_CONTROL = 5561
    ZMQ_PORT_STATUS = 5559
    ZMQ_PORT_DRIVE = 5555
    ZMQ_PORT_ARM = 5556
    ZMQ_PORT_AUTONOMY = 5560
    ZMQ_PORT_AUTONOMY_STATUS = 5565
    ZMQ_PORT_SNAPSHOT_RESULT = 5571
    ZMQ_PORT_SLAM = 5562
    MISSION_DURATION = 300

ZMQ_RECONNECT_TIMEOUT_S = 5.0  # Reconnect all sockets after this many seconds of silence

# Import camera receiver - try multiple import paths
CAMERA_AVAILABLE = False
CameraReceiver = None
frame_to_pixmap = None

# Try relative import first (when run from laptop/)
try:
    from gui.camera_receiver import CameraReceiver, frame_to_pixmap
    CAMERA_AVAILABLE = True
except ImportError:
    pass

# Try direct import (when run from gui/)
if not CAMERA_AVAILABLE:
    try:
        from camera_receiver import CameraReceiver, frame_to_pixmap
        CAMERA_AVAILABLE = True
    except ImportError:
        pass

# Try absolute path import
if not CAMERA_AVAILABLE:
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "camera_receiver",
            os.path.join(os.path.dirname(__file__), "camera_receiver.py")
        )
        camera_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(camera_module)
        CameraReceiver = camera_module.CameraReceiver
        frame_to_pixmap = camera_module.frame_to_pixmap
        CAMERA_AVAILABLE = True
    except Exception as e:
        print(f"[GUI] Camera import error: {e}")

# Auto mission status widget (RMRC 2025)
AUTO_STATUS_WIDGET_AVAILABLE = False
AutoStatusWidget = None
create_and_connect_auto_status = None
try:
    from gui.auto_status_widget import AutoStatusWidget, create_and_connect
    create_and_connect_auto_status = create_and_connect
    AUTO_STATUS_WIDGET_AVAILABLE = True
except ImportError:
    pass

# SLAM view
SLAM_AVAILABLE = False
SLAMView = None
try:
    from gui.slam_view import SLAMView
    SLAM_AVAILABLE = True
except ImportError:
    try:
        from slam_view import SLAMView
        SLAM_AVAILABLE = True
    except ImportError:
        try:
            import importlib.util
            _slam_path = os.path.join(os.path.dirname(__file__), "slam_view.py")
            if os.path.exists(_slam_path):
                spec = importlib.util.spec_from_file_location("slam_view", _slam_path)
                _slam = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(_slam)
                SLAMView = _slam.SLAMView
                SLAM_AVAILABLE = True
        except Exception:
            pass

# 3D SLAM view (RViz-style: QOpenGLWidget, fixed camera, rainbow point cloud)
THREE_D_SLAM_AVAILABLE = False
_3DSLAMView = None
try:
    from gui.three_d_slam_view import _3DSLAMView
    THREE_D_SLAM_AVAILABLE = True
except ImportError:
    try:
        from three_d_slam_view import _3DSLAMView
        THREE_D_SLAM_AVAILABLE = True
    except ImportError:
        try:
            import importlib.util
            _td_path = os.path.join(os.path.dirname(__file__), "three_d_slam_view.py")
            if os.path.exists(_td_path):
                spec = importlib.util.spec_from_file_location("three_d_slam_view", _td_path)
                _td = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(_td)
                _3DSLAMView = _td._3DSLAMView
                THREE_D_SLAM_AVAILABLE = True
        except Exception:
            pass

# 3D Point Cloud view (pyqtgraph fallback)
POINTCLOUD_VIEW_AVAILABLE = False
PointCloudView = None
try:
    from gui.pointcloud_view import PointCloudView
    POINTCLOUD_VIEW_AVAILABLE = True
except ImportError:
    try:
        from pointcloud_view import PointCloudView
        POINTCLOUD_VIEW_AVAILABLE = True
    except ImportError:
        try:
            import importlib.util
            _pc_path = os.path.join(os.path.dirname(__file__), "pointcloud_view.py")
            if os.path.exists(_pc_path):
                spec = importlib.util.spec_from_file_location("pointcloud_view", _pc_path)
                _pc = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(_pc)
                PointCloudView = _pc.PointCloudView
                POINTCLOUD_VIEW_AVAILABLE = True
        except Exception:
            pass

# RViz stream view (MJPEG fallback)
RVIZ_STREAM_AVAILABLE = False
RVizStreamView = None
try:
    from gui.rviz_stream_view import RVizStreamView
    RVIZ_STREAM_AVAILABLE = True
except ImportError:
    try:
        from rviz_stream_view import RVizStreamView
        RVIZ_STREAM_AVAILABLE = True
    except ImportError:
        try:
            import importlib.util
            _rviz_path = os.path.join(os.path.dirname(__file__), "rviz_stream_view.py")
            if os.path.exists(_rviz_path):
                spec = importlib.util.spec_from_file_location("rviz_stream_view", _rviz_path)
                _rviz = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(_rviz)
                RVizStreamView = _rviz.RVizStreamView
                RVIZ_STREAM_AVAILABLE = True
        except Exception:
            pass


# =============================================================================
# STYLE CONSTANTS
# =============================================================================

STYLE_SHEET = """
QMainWindow {
    background-color: #1a1a2e;
}

QWidget {
    background-color: transparent;
    color: #e0e0e0;
    font-family: 'Segoe UI', 'Helvetica Neue', Arial, sans-serif;
}

QFrame#StatusBar {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #16213e, stop:1 #1a1a2e);
    border-bottom: 1px solid #0f3460;
    padding: 6px 10px;
}

QFrame#CameraFrame {
    background-color: #0f0f1a;
    border: 1px solid #0f3460;
    border-radius: 8px;
}

QFrame#ControlPanel {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #1a1a2e, stop:1 #16213e);
    border-top: 1px solid #0f3460;
    padding: 8px 10px;
}

QLabel#StatusLabel {
    color: #4ecca3;
    font-size: 14px;
    font-weight: bold;
}

QLabel#AlertLabel {
    color: #ff6b6b;
    font-size: 14px;
}

QLabel#CameraLabel {
    color: #a0a0a0;
    font-size: 13px;
    background-color: rgba(0, 0, 0, 0.5);
    padding: 4px 10px;
    border-radius: 2px;
}

QPushButton {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #0f3460, stop:1 #16213e);
    border: 1px solid #0f3460;
    border-radius: 5px;
    color: #e0e0e0;
    padding: 8px 14px;
    font-size: 13px;
    min-height: 20px;
    font-weight: bold;
}

QPushButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #1a4f7a, stop:1 #0f3460);
    border: 1px solid #4ecca3;
}

QPushButton:pressed {
    background-color: #0f3460;
}

QPushButton#StartButton {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #2d6a4f, stop:1 #1b4332);
    border: 1px solid #4ecca3;
}

QPushButton#StartButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #40916c, stop:1 #2d6a4f);
}

QPushButton#StopButton {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #9d0208, stop:1 #6a040f);
    border: 1px solid #ff6b6b;
}

QPushButton#StopButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #d00000, stop:1 #9d0208);
}

QCheckBox {
    color: #e0e0e0;
    font-size: 13px;
    spacing: 6px;
    padding: 2px 0;
}

QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border-radius: 3px;
    border: 1px solid #0f3460;
    background: #16213e;
}

QCheckBox::indicator:checked {
    background: #4ecca3;
    border-color: #4ecca3;
}

QComboBox {
    min-height: 24px;
    padding: 4px 10px;
    font-size: 13px;
    border: 1px solid #0f3460;
    border-radius: 5px;
    background: #16213e;
    color: #e0e0e0;
}

QComboBox:hover {
    border-color: #4ecca3;
}

QProgressBar {
    background-color: #16213e;
    border: 1px solid #0f3460;
    border-radius: 4px;
    height: 20px;
    text-align: center;
}

QProgressBar::chunk {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #4ecca3, stop:1 #0f3460);
    border-radius: 3px;
}

QGroupBox {
    border: 1px solid #0f3460;
    border-radius: 6px;
    margin-top: 10px;
    padding: 10px 12px 12px 12px;
    padding-top: 14px;
    font-weight: bold;
    font-size: 14px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 8px;
    color: #4ecca3;
    font-size: 14px;
}

QScrollArea {
    border: none;
    background: transparent;
}
"""


# =============================================================================
# STATUS HUD WIDGET
# =============================================================================

class StatusHUD(QFrame):
    """Top status bar: mission state, timer, robot info, connection indicators (compact)."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("StatusBar")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 3, 8, 3)
        layout.setSpacing(2)
        
        # Row 1: Mission status, timer, connection indicators
        row1 = QHBoxLayout()
        row1.setSpacing(10)
        
        self.status_label = QLabel("IDLE - Ready")
        self.status_label.setObjectName("StatusLabel")
        self.status_label.setFont(QFont("Segoe UI", 12, QFont.Bold))
        row1.addWidget(self.status_label)
        
        row1.addStretch()
        
        self.timer_label = QLabel("05:00")
        self.timer_label.setFont(QFont("Consolas", 18, QFont.Bold))
        self.timer_label.setStyleSheet("color: #4ecca3;")
        row1.addWidget(self.timer_label)
        
        row1.addStretch()
        
        indicators_layout = QHBoxLayout()
        indicators_layout.setSpacing(10)
        self.jetson_indicator = self._create_indicator("Jetson")
        self.pico_indicator = self._create_indicator("Pico")
        self.arm_indicator = self._create_indicator("Arm")
        self.lidar_indicator = self._create_indicator("Lidar")
        indicators_layout.addWidget(self.jetson_indicator)
        indicators_layout.addWidget(self.pico_indicator)
        indicators_layout.addWidget(self.arm_indicator)
        indicators_layout.addWidget(self.lidar_indicator)
        row1.addLayout(indicators_layout)
        
        layout.addLayout(row1)
        
        # Row 2: Robot info (CPU, uptime, cameras)
        self.robot_info_label = QLabel("CPU: --°C  |  Uptime: --s  |  Cameras: 0")
        self.robot_info_label.setStyleSheet("color: #a0a0a0; font-size: 12px;")
        layout.addWidget(self.robot_info_label)
    
    def set_robot_info(self, cpu_temp: float = 0, uptime: float = 0, cameras_active: int = 0):
        """Update robot info line in header."""
        self.robot_info_label.setText(
            f"CPU: {cpu_temp:.1f}°C  |  Uptime: {int(uptime)}s  |  Cameras: {cameras_active}"
        )
    
    def _create_indicator(self, name: str) -> QLabel:
        """Create a connection indicator."""
        label = QLabel(f"● {name}")
        label.setStyleSheet("color: #666666;")  # Gray = disconnected
        return label
    
    def set_status(self, text: str):
        """Update status text."""
        self.status_label.setText(text)
    
    def set_timer(self, seconds: int):
        """Update timer display."""
        mins = seconds // 60
        secs = seconds % 60
        self.timer_label.setText(f"{mins:02d}:{secs:02d}")
        
        # Color warning when low
        if seconds <= 30:
            self.timer_label.setStyleSheet("color: #ff6b6b;")
        elif seconds <= 60:
            self.timer_label.setStyleSheet("color: #ffc107;")
        else:
            self.timer_label.setStyleSheet("color: #4ecca3;")
    
    def set_connection(self, name: str, connected: bool):
        """Update connection indicator."""
        indicator_map = {
            "jetson": self.jetson_indicator,
            "pico": self.pico_indicator,
            "arm": self.arm_indicator,
            "lidar": self.lidar_indicator,
        }
        
        indicator = indicator_map.get(name.lower())
        if indicator:
            color = "#4ecca3" if connected else "#666666"
            indicator.setStyleSheet(f"color: {color};")


# =============================================================================
# CAMERA VIEW WIDGET
# =============================================================================

class CameraView(QFrame):
    """Single camera view with overlay support."""

    def __init__(self, camera_id: str, parent=None, compact: bool = False):
        super().__init__(parent)
        self.setObjectName("CameraFrame")
        self.camera_id = camera_id
        self._compact = compact

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # Camera label
        self.label = QLabel(camera_id.upper())
        self.label.setObjectName("CameraLabel")
        self.label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.label.setStyleSheet("font-weight: bold; color: #4ecca3; font-size: 11px;")

        # Image display
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        min_w, min_h = (280, 210) if not compact else (120, 90)
        self.image_label.setMinimumSize(min_w, min_h)
        self.image_label.setStyleSheet("background-color: #0a0a14; border-radius: 4px;")

        self.image_label.setText("No Signal")
        fs = "11px" if compact else "13px"
        self.image_label.setStyleSheet(f"""
            background-color: #0a0a14;
            color: #505050;
            font-size: {fs};
            border-radius: 4px;
        """)
        
        layout.addWidget(self.label)
        layout.addWidget(self.image_label, 1)
        
        # Detection overlays
        self.detections = []
        self.qr_codes = []
        self.landolt_readings = []
    
    def update_frame(self, frame: np.ndarray, detections: list = None, qr_codes: list = None,
                     landolt_readings: list = None):
        """Update camera frame with optional overlays (detections, QR, Landolt-C directions for judge)."""
        if frame is None:
            return
        
        self.detections = detections or []
        self.qr_codes = qr_codes or []
        self.landolt_readings = landolt_readings or []
        
        # Draw overlays
        display_frame = frame.copy()
        
        # Draw YOLO detections
        for det in self.detections:
            bbox = det.get("bbox", [])
            if len(bbox) == 4:
                x1, y1, x2, y2 = map(int, bbox)
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                label = det.get("class_name", "")
                conf = det.get("confidence", 0)
                text = f"{label} {conf:.2f}"
                cv2.putText(display_frame, text, (x1, y1-5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw QR codes
        for qr in self.qr_codes:
            bbox = qr.get("bbox", [])
            if len(bbox) == 4:
                x1, y1, x2, y2 = map(int, bbox)
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
                
                data = qr.get("data", "")[:20]
                cv2.putText(display_frame, f"QR: {data}", (x1, y1-5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        # Landolt-C direction overlay (rulebook: screen overlay or status line for judge)
        if self.landolt_readings:
            parts = []
            for r in self.landolt_readings:
                d = r.get("direction", "")
                c = r.get("confidence", 0)
                if d:
                    parts.append(f"{d} {c:.0%}")
            if parts:
                line = "Landolt: " + ", ".join(parts)
                h, w = display_frame.shape[:2]
                # Larger, readable text: scale 0.9, thickness 2; dark bar behind for contrast
                font = cv2.FONT_HERSHEY_SIMPLEX
                (tw, th), _ = cv2.getTextSize(line, font, 0.9, 2)
                x, y = 8, h - 12
                cv2.rectangle(display_frame, (x, y - th - 4), (x + tw + 8, y + 4), (0, 0, 0), -1)
                cv2.putText(display_frame, line, (x + 4, y), font, 0.9, (0, 255, 255), 2)
        
        # Convert to QPixmap
        self._display_image(display_frame)
    
    def _display_image(self, frame: np.ndarray):
        """Convert numpy array to QPixmap and display."""
        if frame is None:
            return
        
        # Convert BGR to RGB
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        h, w = frame.shape[:2]
        bytes_per_line = 3 * w
        
        q_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        
        # Scale to fit
        scaled = pixmap.scaled(
            self.image_label.size(),
            Qt.KeepAspectRatio,
            Qt.FastTransformation
        )
        
        self.image_label.setPixmap(scaled)
    
    def set_no_signal(self):
        """Display no signal message."""
        self.image_label.clear()
        self.image_label.setText("No Signal")


# =============================================================================
# CAMERA GRID WIDGET
# =============================================================================

class CameraGrid(QWidget):
    """Front = main (left, full height). Arm = large (top-right). Backward = small (bottom-right)."""

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QGridLayout(self)
        layout.setSpacing(8)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setColumnStretch(0, 2)  # Front: 2/3 width
        layout.setColumnStretch(1, 1)  # Right: 1/3 width
        layout.setRowStretch(0, 2)     # Arm: 2/3 height
        layout.setRowStretch(1, 1)     # Backward: 1/3 height

        self.cameras = {
            "front": CameraView("Front", self, compact=False),
            "arm": CameraView("Arm", self, compact=False),
            "backward": CameraView("Backward", self, compact=True),
        }

        # Front: left column, full height (main driving view)
        layout.addWidget(self.cameras["front"], 0, 0, 2, 1)
        # Arm: top-right (inspection)
        layout.addWidget(self.cameras["arm"], 0, 1)
        # Backward: bottom-right (rear view, smallest)
        layout.addWidget(self.cameras["backward"], 1, 1)
        self._arm_expanded = False
        self._full_width_layout = False

    def set_full_width_layout(self, full_width: bool):
        """When True: one row of 3 equal cameras (use when SLAM panel is closed). When False: normal grid."""
        if getattr(self, "_full_width_layout", False) == full_width:
            return
        self._full_width_layout = full_width
        layout = self.layout()
        if not layout:
            return

        def _do_layout():
            try:
                for i in range(layout.count() - 1, -1, -1):
                    item = layout.takeAt(i)
                    if item.widget():
                        item.widget().setParent(None)
                if self._full_width_layout:
                    layout.setColumnStretch(0, 1)
                    layout.setColumnStretch(1, 1)
                    layout.setColumnStretch(2, 1)
                    layout.setRowStretch(0, 1)
                    layout.setRowStretch(1, 0)
                    layout.addWidget(self.cameras["front"], 0, 0)
                    layout.addWidget(self.cameras["arm"], 0, 1)
                    layout.addWidget(self.cameras["backward"], 0, 2)
                else:
                    layout.setColumnStretch(0, 2)
                    layout.setColumnStretch(1, 1)
                    layout.setColumnStretch(2, 0)  # no third column in normal layout
                    layout.setRowStretch(0, 2)
                    layout.setRowStretch(1, 1)
                    layout.addWidget(self.cameras["front"], 0, 0, 2, 1)
                    layout.addWidget(self.cameras["arm"], 0, 1)
                    layout.addWidget(self.cameras["backward"], 1, 1)
            except Exception as e:
                print(f"[GUI] set_full_width_layout error: {e}")
        QTimer.singleShot(0, _do_layout)

    def set_arm_expanded(self, expanded: bool):
        """Inspection mode: arm full height, backward below front."""
        if getattr(self, "_full_width_layout", False):
            return  # keep full-width layout when SLAM is closed
        if getattr(self, "_arm_expanded", False) == expanded:
            return
        self._arm_expanded = expanded
        layout = self.layout()
        if not layout:
            return

        def _do_layout():
            try:
                for i in range(layout.count() - 1, -1, -1):
                    layout.takeAt(i)
                if self._arm_expanded:
                    layout.addWidget(self.cameras["front"], 0, 0)
                    layout.addWidget(self.cameras["arm"], 0, 1, 2, 1)
                    layout.addWidget(self.cameras["backward"], 1, 0)
                else:
                    layout.addWidget(self.cameras["front"], 0, 0, 2, 1)
                    layout.addWidget(self.cameras["arm"], 0, 1)
                    layout.addWidget(self.cameras["backward"], 1, 1)
            except Exception as e:
                print(f"[GUI] set_arm_expanded error: {e}")
        QTimer.singleShot(0, _do_layout)

    def update_camera(self, camera_id: str, frame: np.ndarray,
                      detections: list = None, qr_codes: list = None, landolt_readings: list = None):
        """Update a specific camera view by id (front/arm/backward)."""
        if camera_id in self.cameras:
            self.cameras[camera_id].update_frame(frame, detections, qr_codes, landolt_readings)


# =============================================================================
# JOYSTICK INDICATOR WIDGET
# =============================================================================

class JoystickIndicator(QWidget):
    """Visual joystick position indicator."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(80, 80)
        self.setMaximumSize(100, 100)
        
        self.x_pos = 0.0  # -1 to 1
        self.y_pos = 0.0  # -1 to 1
        self.z_pos = 0.0  # -1 to 1 (shown as rotation indicator)
    
    def set_position(self, x: float, y: float, z: float = 0.0):
        """Set joystick position (-1 to 1 for each axis)."""
        self.x_pos = max(-1, min(1, x))
        self.y_pos = max(-1, min(1, y))
        self.z_pos = max(-1, min(1, z))
        self.update()
    
    def paintEvent(self, event):
        """Draw the joystick indicator."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        radius = min(w, h) // 2 - 5
        
        # Draw background circle
        painter.setPen(QColor(15, 52, 96))
        painter.setBrush(QColor(10, 10, 20))
        painter.drawEllipse(cx - radius, cy - radius, radius * 2, radius * 2)
        
        # Draw crosshairs
        painter.setPen(QColor(40, 40, 60))
        painter.drawLine(cx - radius, cy, cx + radius, cy)
        painter.drawLine(cx, cy - radius, cx, cy + radius)
        
        # Draw Z indicator (rotation) as arc
        if abs(self.z_pos) > 0.05:
            painter.setPen(QColor(255, 165, 0, 150))  # Orange
            arc_angle = int(self.z_pos * 90 * 16)  # Qt uses 1/16th degrees
            painter.drawArc(cx - radius + 5, cy - radius + 5, 
                           (radius - 5) * 2, (radius - 5) * 2,
                           90 * 16, -arc_angle)
        
        # Draw joystick position
        jx = cx + int(self.x_pos * (radius - 10))
        jy = cy - int(self.y_pos * (radius - 10))  # Y is inverted
        
        # Glow effect
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(78, 204, 163, 100))
        painter.drawEllipse(jx - 12, jy - 12, 24, 24)
        
        # Main dot
        painter.setBrush(QColor(78, 204, 163))
        painter.drawEllipse(jx - 8, jy - 8, 16, 16)


# =============================================================================
# DETECTION LOG WIDGET
# =============================================================================

class DetectionLog(QFrame):
    """Scrollable log of detected objects and QR codes (for footer)."""
    qr_added_for_judge = pyqtSignal()  # Emitted when a new QR is read (remind operator to point to judge)
    
    def __init__(self, parent=None, max_height: int = 100):
        super().__init__(parent)
        self.setObjectName("CameraFrame")
        self.setMaximumHeight(max_height)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(2)
        
        # Title + stats in one line
        from PyQt5.QtWidgets import QListWidget, QListWidgetItem
        header = QHBoxLayout()
        title = QLabel("Detections")
        title.setStyleSheet("font-weight: bold; color: #4ecca3; font-size: 13px;")
        self.stats_label = QLabel("Objects: 0 | QR: 0")
        self.stats_label.setStyleSheet("color: #e0e0e0; font-size: 15px; font-weight: bold;")
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.stats_label)
        layout.addLayout(header)
        
        self.list_widget = QListWidget()
        self.list_widget.setMaximumHeight(max_height - 28)
        self.list_widget.setStyleSheet("""
            QListWidget {
                background-color: #0a0a14;
                border: none;
                color: #e0e0e0;
                font-size: 13px;
            }
            QListWidget::item {
                padding: 4px;
                border-bottom: 1px solid #1a1a2e;
            }
            QListWidget::item:selected {
                background-color: #0f3460;
            }
        """)
        layout.addWidget(self.list_widget)
        
        self.detection_count = 0
        self.qr_count = 0
        self.hazmat_count = 0
        self.landolt_count = 0
        # Full QR contents for PDF report (data, timestamp, camera_id)
        self.qr_entries = []
        self._qr_entries_max = 500
        self._seen_qr_data = set()  # Rulebook: each QR scanned once (no duplicate points)
        # HAZMAT keywords (from constants / vision labels) for report aggregation
        try:
            from shared.constants import HAZMAT_LABELS
            self._hazmat_keywords = {s.lower().strip() for s in HAZMAT_LABELS}
        except ImportError:
            self._hazmat_keywords = {"explosive", "flammable", "poison", "oxygen", "corrosive", "radioactive", "oxidizer", "organic peroxide", "non-flammable", "dangerous", "miscellaneous"}
    
    def add_detection(self, camera_id: str, class_name: str, confidence: float, is_victim: bool = False):
        """Add a detection to the log; aggregate HAZMAT and Landolt for mission report."""
        from PyQt5.QtWidgets import QListWidgetItem
        
        timestamp = time.strftime("%H:%M:%S")
        icon = "🚨" if is_victim else "📦"
        text = f"{icon} [{timestamp}] {camera_id}: {class_name} ({confidence:.0%})"
        
        item = QListWidgetItem(text)
        if is_victim:
            item.setForeground(QColor(255, 107, 107))  # Red for victims
        
        self.list_widget.insertItem(0, item)
        self.detection_count += 1
        
        cn = (class_name or "").lower().strip()
        if cn in self._hazmat_keywords or any(kw in cn for kw in ("flammable", "explosive", "poison", "oxygen", "corrosive", "radioactive", "hazmat")):
            self.hazmat_count += 1
        if "landolt" in cn or "ring" in cn or cn in ("up", "up-right", "right", "down-right", "down", "down-left", "left", "up-left"):
            self.landolt_count += 1
        
        # Keep max 50 items
        while self.list_widget.count() > 50:
            self.list_widget.takeItem(self.list_widget.count() - 1)
        
        self._update_stats()
    
    def add_qr_code(self, camera_id: str, data: str, pose_xy: tuple = None):
        """Add a QR code detection (once per unique code per rulebook). pose_xy=(x,y) for QR-on-map."""
        from PyQt5.QtWidgets import QListWidgetItem

        if not data or data in self._seen_qr_data:
            return
        self._seen_qr_data.add(data)

        timestamp = time.strftime("%H:%M:%S")
        date_str = time.strftime("%Y-%m-%d")
        text = f"📱 [{timestamp}] {camera_id}: QR={data[:30]}"

        item = QListWidgetItem(text)
        item.setForeground(QColor(255, 0, 255))  # Magenta for QR

        self.list_widget.insertItem(0, item)
        self.qr_count += 1
        entry = {"data": data, "timestamp": timestamp, "camera_id": camera_id, "date": date_str}
        if pose_xy is not None and len(pose_xy) >= 2:
            entry["map_x"] = float(pose_xy[0])
            entry["map_y"] = float(pose_xy[1])
        self.qr_entries.insert(0, entry)
        while len(self.qr_entries) > self._qr_entries_max:
            self.qr_entries.pop()

        while self.list_widget.count() > 50:
            self.list_widget.takeItem(self.list_widget.count() - 1)

        self._write_qr_to_file(entry)
        self.qr_added_for_judge.emit()

        self._update_stats()

    def _write_qr_to_file(self, entry: dict):
        """Append QR data to file (rulebook: information must be written to a file for judge)."""
        try:
            from shared.constants import QR_CODES_OUTPUT_FILE
        except ImportError:
            QR_CODES_OUTPUT_FILE = "mission_qr_codes.txt"
        out_dir = os.path.join(os.path.dirname(__file__), "..", "..", "reports")
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, QR_CODES_OUTPUT_FILE)
        write_header = not os.path.exists(path) or os.path.getsize(path) == 0
        with open(path, "a", encoding="utf-8") as f:
            if write_header:
                f.write("Date\tTime\tCamera\tContent\n")
                f.write("-" * 60 + "\n")
            f.write(f"{entry.get('date', '')}\t{entry.get('timestamp', '')}\t{entry.get('camera_id', '')}\t{entry.get('data', '')}\n")
    
    def _update_stats(self):
        self.stats_label.setText(f"Objects: {self.detection_count} | QR: {self.qr_count} | HAZMAT: {self.hazmat_count} | Landolt: {self.landolt_count}")


# =============================================================================
# CONTROL PANEL WIDGET
# =============================================================================

class ControlPanel(QFrame):
    """Bottom control panel: joystick, drive, optional middle widget (e.g. detection log), mission buttons."""

    # Signals
    start_mission = pyqtSignal()
    stop_mission = pyqtSignal()
    generate_report = pyqtSignal()
    lap_teleop_clicked = pyqtSignal()
    autonomy_mapping = pyqtSignal()
    autonomy_sensor_cabinet = pyqtSignal()

    def __init__(self, parent=None, middle_widget: Optional[QWidget] = None):
        super().__init__(parent)
        self.setObjectName("ControlPanel")
        self.setFixedHeight(100 if middle_widget else 88)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(12)

        # Joystick visual indicator
        self.joystick_indicator = JoystickIndicator()
        layout.addWidget(self.joystick_indicator)

        # Joystick status text
        joystick_group = QGroupBox("Joystick")
        joystick_group.setStyleSheet("QGroupBox { font-size: 13px; font-weight: bold; }")
        joystick_layout = QVBoxLayout(joystick_group)

        self.joystick_status = QLabel("Disconnected")
        self.joystick_status.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.joystick_values = QLabel("Y: 0  X: 0  Z: 0")
        self.joystick_values.setFont(QFont("Consolas", 12))
        self.joystick_values.setStyleSheet("font-size: 13px; color: #e0e0e0;")

        joystick_layout.addWidget(self.joystick_status)
        joystick_layout.addWidget(self.joystick_values)
        layout.addWidget(joystick_group)

        # Drive status
        drive_group = QGroupBox("Drive")
        drive_group.setStyleSheet("QGroupBox { font-size: 13px; font-weight: bold; }")
        drive_layout = QVBoxLayout(drive_group)

        self.drive_status = QLabel("Stopped")
        self.drive_status.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.drive_values = QLabel("L: 0  R: 0")
        self.drive_values.setFont(QFont("Consolas", 12))
        self.drive_values.setStyleSheet("font-size: 13px; color: #e0e0e0;")
        
        drive_layout.addWidget(self.drive_status)
        drive_layout.addWidget(self.drive_values)
        layout.addWidget(drive_group)
        
        # Detection log (between joystick/drive and Start Mission)
        if middle_widget is not None:
            middle_widget.setMinimumWidth(260)
            middle_widget.setMaximumWidth(420)
            layout.addWidget(middle_widget, 1)

        layout.addStretch()

        # Control buttons
        self.start_button = QPushButton("▶ Start Mission")
        self.start_button.setObjectName("StartButton")
        self.start_button.setMinimumHeight(40)
        self.start_button.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.start_button.clicked.connect(self.start_mission.emit)
        
        self.stop_button = QPushButton("⬛ STOP")
        self.stop_button.setObjectName("StopButton")
        self.stop_button.setMinimumHeight(40)
        self.stop_button.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.stop_button.clicked.connect(self.stop_mission.emit)
        
        self.lap_button = QPushButton("Lap (T)")
        self.lap_button.setMinimumHeight(36)
        self.lap_button.setToolTip("Increment teleop lap count for report")
        self.lap_button.clicked.connect(self.lap_teleop_clicked.emit)
        
        self.reset_button = QPushButton("↻ Reset")
        self.reset_button.setMinimumHeight(36)
        self.reset_button.setToolTip("Mini-mission reset (rulebook 2.5): clear scores, keep timer running")
        
        self.report_button = QPushButton("📄 Report")
        self.report_button.setMinimumHeight(36)
        self.report_button.setToolTip("Generate mission report PDF + QR list for judge. Saved to ~/mission_report.pdf and ~/mission_report_qr_codes.txt. See docs/COMPETITION_CHECKLIST.md.")
        self.report_button.clicked.connect(self.generate_report.emit)

        self.stowed_checkbox = QCheckBox("Stowed")
        self.stowed_checkbox.setToolTip("Confirm robot is in stowed posture (required for autonomy 4x bonus)")
        self.mapping_btn = QPushButton("Mapping")
        self.mapping_btn.setMinimumHeight(36)
        self.mapping_btn.setToolTip("Start mapping autonomy (lap detection)")
        self.mapping_btn.clicked.connect(self.autonomy_mapping.emit)
        self.sensor_cabinet_btn = QPushButton("Sensor Cab")
        self.sensor_cabinet_btn.setMinimumHeight(36)
        self.sensor_cabinet_btn.setToolTip("Start Sensor Cabinet autonomy (HAZMAT/Landolt)")
        self.sensor_cabinet_btn.clicked.connect(self.autonomy_sensor_cabinet.emit)
        
        layout.setSpacing(6)
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        layout.addWidget(self.lap_button)
        layout.addWidget(self.stowed_checkbox)
        layout.addWidget(self.mapping_btn)
        layout.addWidget(self.sensor_cabinet_btn)
        layout.addWidget(self.reset_button)
        layout.addWidget(self.report_button)
    
    def update_joystick(self, connected: bool, y: int = 0, x: int = 0, z: int = 0):
        """Update joystick display."""
        if connected:
            self.joystick_status.setText("Connected")
            self.joystick_status.setStyleSheet("color: #4ecca3;")
        else:
            self.joystick_status.setText("Disconnected")
            self.joystick_status.setStyleSheet("color: #ff6b6b;")
        
        self.joystick_values.setText(f"Y:{y:+4d}  X:{x:+4d}  Z:{z:+4d}")
        
        # Update visual indicator (-100 to 100 -> -1 to 1)
        self.joystick_indicator.set_position(x / 100.0, y / 100.0, z / 100.0)
    
    def update_drive(self, left: int, right: int, moving: bool):
        """Update drive display."""
        if moving:
            self.drive_status.setText("Moving")
            self.drive_status.setStyleSheet("color: #4ecca3;")
        else:
            self.drive_status.setText("Stopped")
            self.drive_status.setStyleSheet("color: #a0a0a0;")
        
        self.drive_values.setText(f"L:{left:+4d}  R:{right:+4d}")


# =============================================================================
# TELEOP PANEL (test selector + scoring stack + Pick combo + control panel)
# =============================================================================

class TeleopPanel(QWidget):
    """Teleop mode: test selector, per-test scoring buttons, Linear Rail Pick combo, control panel."""
    start_mission = pyqtSignal()
    stop_mission = pyqtSignal()
    generate_report = pyqtSignal()
    lap_teleop_clicked = pyqtSignal()
    start_autonomy = pyqtSignal(str)  # test_type or mode
    set_waypoint_a = pyqtSignal()
    set_waypoint_b = pyqtSignal()
    add_fiducial_clicked = pyqtSignal()

    def __init__(self, control_panel: ControlPanel, parent=None):
        super().__init__(parent)
        self.control_panel = control_panel
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        # Row 1: Test selector + Linear Rail Pick mode (visible when test is linear_rail_pick)
        row1 = QHBoxLayout()
        row1.setSpacing(10)
        test_label = QLabel("Test:")
        test_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #e0e0e0;")
        row1.addWidget(test_label)
        self.test_combo = QComboBox()
        self.test_combo.setMinimumWidth(160)
        try:
            from shared.constants import TEST_GROUPS
            for group_name, items in TEST_GROUPS:
                for label, test_id in items:
                    self.test_combo.addItem(label, test_id)
        except ImportError:
            self.test_combo.addItem("Sand & Gravel", "sand_gravel")
        self.test_combo.currentIndexChanged.connect(self._on_test_changed)
        row1.addWidget(self.test_combo)
        pick_label = QLabel("Pick mode:")
        pick_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #e0e0e0;")
        row1.addWidget(pick_label)
        self.pick_mode_combo = QComboBox()
        try:
            from shared.constants import LINEAR_RAIL_PICK_MODES
            for label, mode_id in LINEAR_RAIL_PICK_MODES:
                self.pick_mode_combo.addItem(label, mode_id)
        except ImportError:
            self.pick_mode_combo.addItem("Full Autonomy", "full_auto")
        self.pick_mode_combo.setMinimumWidth(150)
        self.pick_mode_combo.setVisible(False)
        row1.addWidget(self.pick_mode_combo)
        row1.addStretch()
        self.btn_set_fiducial = QPushButton("Set Fiducial")
        self.btn_set_fiducial.setToolTip("Add current SLAM pose to fiducials (saved to reports/fiducials.json)")
        self.btn_set_fiducial.clicked.connect(self.add_fiducial_clicked.emit)
        row1.addWidget(self.btn_set_fiducial)
        layout.addLayout(row1)
        # Row 2: Per-test scoring stack
        self.scoring_stacked = QStackedWidget()
        self._build_scoring_pages()
        self.scoring_stacked.setMaximumHeight(48)
        layout.addWidget(self.scoring_stacked)
        layout.addWidget(control_panel)
        # Forward signals from control_panel
        control_panel.start_mission.connect(self.start_mission.emit)
        control_panel.stop_mission.connect(self.stop_mission.emit)
        control_panel.generate_report.connect(self.generate_report.emit)
        control_panel.lap_teleop_clicked.connect(self.lap_teleop_clicked.emit)
        control_panel.autonomy_mapping.connect(lambda: self.start_autonomy.emit("lap"))
        control_panel.autonomy_sensor_cabinet.connect(lambda: self.start_autonomy.emit("sensor_cabinet"))

    def _on_test_changed(self, idx):
        test_id = self.test_combo.currentData()
        self.pick_mode_combo.setVisible(test_id == "linear_rail_pick")
        mobility = ("incline_horiz", "incline_inclined", "sand_gravel", "ramps_continuous", "ramps_pinwheel",
                    "elevated_ramps", "krails_horiz", "krails_crossover", "hurdles_single", "hurdles_double")
        page_map = {"keypad_omni": 1, "sensor_cabinet": 2, "linear_rail_inspect": 3, "linear_rail_pick": 4,
                    "stairs": 5, "align": 6, "drop_test": 7}
        page = page_map.get(test_id, 0) if test_id not in mobility else 0
        self.scoring_stacked.setCurrentIndex(min(page, self.scoring_stacked.count() - 1))

    def _build_scoring_pages(self):
        # Page 0: Mobility — Lap (T), Lap (A), Set A, Set B
        p0 = QWidget()
        l0 = QHBoxLayout(p0)
        l0.setSpacing(6)
        score_label = QLabel("Scoring:")
        score_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #e0e0e0;")
        l0.addWidget(score_label)
        self.btn_lap_t = QPushButton("Lap (T)")
        self.btn_lap_a = QPushButton("Lap (A)")
        self.btn_waypoint_a = QPushButton("Set Waypoint A")
        self.btn_waypoint_b = QPushButton("Set Waypoint B")
        for b in (self.btn_lap_t, self.btn_lap_a, self.btn_waypoint_a, self.btn_waypoint_b):
            b.setMinimumHeight(36)
            l0.addWidget(b)
        self.btn_lap_t.clicked.connect(lambda: self._scoring_clicked("lap_teleop"))
        self.btn_lap_a.clicked.connect(lambda: self._scoring_clicked("lap_auto"))
        self.btn_waypoint_a.clicked.connect(self.set_waypoint_a.emit)
        self.btn_waypoint_b.clicked.connect(self.set_waypoint_b.emit)
        l0.addStretch()
        self.scoring_stacked.addWidget(p0)
        # Page 1: Keypad — +3 clean, +1 dirty, Left area
        p1 = QWidget()
        l1 = QHBoxLayout(p1)
        l1.addWidget(QLabel("Scoring:"))
        for lbl, key in [("+3 clean", "keypad_clean"), ("+1 dirty", "keypad_dirty"), ("Left area", "keypad_left")]:
            btn = QPushButton(lbl)
            btn.clicked.connect(lambda checked, k=key: self._scoring_clicked(k))
            l1.addWidget(btn)
        l1.addStretch()
        self.scoring_stacked.addWidget(p1)
        # Page 2: Sensor Cabinet — door, hazmat, landolt checkboxes
        p2 = QWidget()
        l2 = QHBoxLayout(p2)
        l2.addWidget(QLabel("Scoring:"))
        self.cb_door = QCheckBox("Door")
        self.cb_hazmat = QCheckBox("HAZMAT")
        self.cb_landolt = QCheckBox("Landolt")
        for w in (self.cb_door, self.cb_hazmat, self.cb_landolt):
            l2.addWidget(w)
        l2.addStretch()
        self.scoring_stacked.addWidget(p2)
        # Page 3: Linear Rail Inspect — C1-C5, direction, Record Reading, Show Gap
        p3 = QWidget()
        l3 = QHBoxLayout(p3)
        l3.addWidget(QLabel("Tube:"))
        self.inspect_tube_combo = QComboBox()
        for i in range(1, 6):
            self.inspect_tube_combo.addItem(f"C{i}", f"C{i}")
        l3.addWidget(self.inspect_tube_combo)
        l3.addWidget(QLabel("Direction:"))
        self.inspect_direction_combo = QComboBox()
        try:
            from shared.constants import LANDOLT_DIRECTIONS
            for d in LANDOLT_DIRECTIONS:
                self.inspect_direction_combo.addItem(d)
        except ImportError:
            for d in ["Up", "Up-Right", "Right", "Down-Right", "Down", "Down-Left", "Left", "Up-Left"]:
                self.inspect_direction_combo.addItem(d)
        l3.addWidget(self.inspect_direction_combo)
        self.btn_record_reading = QPushButton("Record Reading")
        self.btn_show_gap = QPushButton("Show Gap")
        l3.addWidget(self.btn_record_reading)
        l3.addWidget(self.btn_show_gap)
        self.btn_record_reading.clicked.connect(lambda: self._scoring_clicked("inspect_record"))
        self.btn_show_gap.clicked.connect(lambda: self._scoring_clicked("inspect_show_gap"))
        l3.addStretch()
        self.scoring_stacked.addWidget(p3)
        # Page 4: Linear Rail Pick — per-tube Removed, In Container
        p4 = QWidget()
        l4 = QHBoxLayout(p4)
        l4.addWidget(QLabel("Scoring:"))
        self.pick_tube_combo = QComboBox()
        for i in range(1, 6):
            self.pick_tube_combo.addItem(f"Tube {i}", i)
        l4.addWidget(self.pick_tube_combo)
        self.btn_removed = QPushButton("Removed")
        self.btn_in_container = QPushButton("In Container")
        self.btn_removed.clicked.connect(lambda: self._scoring_clicked("pick_removed"))
        self.btn_in_container.clicked.connect(lambda: self._scoring_clicked("pick_in_container"))
        l4.addWidget(self.btn_removed)
        l4.addWidget(self.btn_in_container)
        l4.addStretch()
        self.scoring_stacked.addWidget(p4)
        # Page 5: Stairs — +1 level (T), +1 level (A)
        p5 = QWidget()
        l5 = QHBoxLayout(p5)
        for lbl, key in [("+1 level (T)", "stairs_teleop"), ("+1 level (A)", "stairs_auto")]:
            btn = QPushButton(lbl)
            btn.clicked.connect(lambda checked, k=key: self._scoring_clicked(k))
            l5.addWidget(btn)
        l5.addStretch()
        self.scoring_stacked.addWidget(p5)
        # Page 6: Align — +1 crossing (T), +1 crossing (A)
        p6 = QWidget()
        l6 = QHBoxLayout(p6)
        for lbl, key in [("+1 crossing (T)", "align_teleop"), ("+1 crossing (A)", "align_auto")]:
            btn = QPushButton(lbl)
            btn.clicked.connect(lambda checked, k=key: self._scoring_clicked(k))
            l6.addWidget(btn)
        l6.addStretch()
        self.scoring_stacked.addWidget(p6)
        # Page 7: Drop Test
        p7 = QWidget()
        l7 = QHBoxLayout(p7)
        for lbl, key in [("Survived 15cm", "drop_15"), ("Survived 30cm", "drop_30")]:
            btn = QPushButton(lbl)
            btn.clicked.connect(lambda checked, k=key: self._scoring_clicked(k))
            l7.addWidget(btn)
        l7.addStretch()
        self.scoring_stacked.addWidget(p7)

    scoring_clicked = pyqtSignal(str)

    def _scoring_clicked(self, key: str):
        """Emit generic scoring signal; parent can collect for mission_data."""
        self.scoring_clicked.emit(key)

    def get_current_test_id(self):
        return self.test_combo.currentData() or "sand_gravel"

    def get_pick_mode_id(self):
        return self.pick_mode_combo.currentData() or "full_auto"

    def get_inspect_reading(self):
        return {"tube": self.inspect_tube_combo.currentData(), "direction": self.inspect_direction_combo.currentText()}


# =============================================================================
# AUTONOMY PANEL (ABORT, HANDS OFF, Nav2 progress, step, laps, Report)
# =============================================================================

class AutonomyPanel(QWidget):
    abort_requested = pyqtSignal()
    resume_requested = pyqtSignal()
    generate_report = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(15, 8, 15, 8)
        self.abort_btn = QPushButton("⬛ ABORT → Teleop")
        self.abort_btn.setObjectName("StopButton")
        self.abort_btn.setMinimumSize(180, 50)
        self.abort_btn.clicked.connect(self.abort_requested.emit)
        layout.addWidget(self.abort_btn)
        self.resume_btn = QPushButton("▶ Resume")
        self.resume_btn.setMinimumSize(100, 50)
        self.resume_btn.setVisible(False)
        self.resume_btn.clicked.connect(self.resume_requested.emit)
        layout.addWidget(self.resume_btn)
        self.hands_off_label = QLabel("🚫 HANDS OFF")
        self.hands_off_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #f59e0b;")
        layout.addWidget(self.hands_off_label)
        self.nav2_label = QLabel("Nav2: —")
        self.nav2_label.setStyleSheet("color: #a0a0a0;")
        layout.addWidget(self.nav2_label)
        self.nav2_progress = QProgressBar()
        self.nav2_progress.setMaximumWidth(120)
        self.nav2_progress.setValue(0)
        layout.addWidget(self.nav2_progress)
        self.step_label = QLabel("Step —")
        layout.addWidget(self.step_label)
        self.laps_label = QLabel("Laps: 0")
        layout.addWidget(self.laps_label)
        layout.addStretch()
        self.report_btn = QPushButton("📄 Report")
        self.report_btn.clicked.connect(self.generate_report.emit)
        layout.addWidget(self.report_btn)

    def set_nav2_status(self, goal_name: str, distance_m: float, progress: int):
        self.nav2_label.setText(f"Nav2: →{goal_name} {distance_m:.1f}m")
        self.nav2_progress.setValue(min(100, max(0, progress)))

    def set_step(self, step: str, label: str):
        self.step_label.setText(f"Step {step}: {label}")

    def set_laps(self, n: int):
        self.laps_label.setText(f"Laps: {n}")

    def set_paused(self, paused: bool):
        """Show Resume button when autonomy is paused (e.g. Pick pause_teleop)."""
        self.resume_btn.setVisible(bool(paused))


# =============================================================================
# SNAPSHOT REVIEW PANEL (ABORT, result label, Accept, Retry, direction, Record)
# =============================================================================

class SnapshotReviewPanel(QWidget):
    abort_requested = pyqtSignal()
    accept_declare = pyqtSignal()
    retry = pyqtSignal()
    record_reading = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(15, 8, 15, 8)
        self.abort_btn = QPushButton("⬛ ABORT")
        self.abort_btn.setObjectName("StopButton")
        self.abort_btn.clicked.connect(self.abort_requested.emit)
        layout.addWidget(self.abort_btn)
        self.result_label = QLabel("—")
        self.result_label.setStyleSheet("color: #3b82f6; font-weight: bold;")
        layout.addWidget(self.result_label)
        self.accept_btn = QPushButton("✓ Accept & Declare to Judge")
        self.accept_btn.setStyleSheet("background-color: #22c55e; color: black;")
        self.accept_btn.clicked.connect(self.accept_declare.emit)
        layout.addWidget(self.accept_btn)
        self.retry_btn = QPushButton("↻ Retry")
        self.retry_btn.clicked.connect(self.retry.emit)
        layout.addWidget(self.retry_btn)
        layout.addWidget(QLabel("Direction:"))
        self.direction_combo = QComboBox()
        try:
            from shared.constants import LANDOLT_DIRECTIONS
            for d in LANDOLT_DIRECTIONS:
                self.direction_combo.addItem(d)
        except ImportError:
            for d in ["Up", "Up-Right", "Right", "Down-Right", "Down", "Down-Left", "Left", "Up-Left"]:
                self.direction_combo.addItem(d)
        layout.addWidget(self.direction_combo)
        self.record_btn = QPushButton("Record Reading")
        self.record_btn.clicked.connect(self.record_reading.emit)
        layout.addWidget(self.record_btn)
        layout.addStretch()

    def set_result(self, text: str):
        self.result_label.setText(text)


# =============================================================================
# MAIN WINDOW
# =============================================================================

# Joystick button ID -> GUI action. Run: python -m control.joystick_button_debug to find your button IDs.
# Button 7 = back support (1 DOF kickstand) UP, 8 = DOWN. Servo ID 7 on arm bus.
JOY_BUTTON_MAP = {
    0: "start",            # Start Mission
    1: "stop",             # STOP
    2: "lap",              # Lap (T)
    3: "report",           # Report
    4: "yolo",             # YOLO toggle
    5: "qr",               # QR toggle
    6: "stowed",           # Stowed checkbox
    7: "back_support_up",  # 1 DOF back support servo (ID 7) — up / deploy
    8: "back_support_down",# 1 DOF back support — down / stow
    9: "mapping",          # Mapping
    10: "sensor_cab",      # Sensor Cab
}


class OperatorStation(QMainWindow):
    """Main operator station window."""
    
    def __init__(self, jetson_ip: str = JETSON_IP, joy_buttons_debug: bool = False):
        super().__init__()
        
        self.jetson_ip = jetson_ip
        self._joy_buttons_debug = joy_buttons_debug
        self.setWindowTitle("RoboCupRescue RMRC 2026 - Operator Station")
        self.setMinimumSize(1280, 800)
        self.setStyleSheet(STYLE_SHEET)
        
        # Mission state
        self.mission_active = False
        self.mission_start_time = 0
        self.remaining_time = MISSION_DURATION
        self.mission_laps_teleop = 0
        self.mission_laps_auto = 0
        # GUI mode: teleop | autonomy | snapshot_review (rulebook; controls HUD accent and panels)
        self.gui_mode = "teleop"
        self._joystick_activity_start = None  # for 500ms interlock in autonomy
        self._snapshot_frozen_image = None  # for snapshot review
        # Lap waypoints (Set Waypoint A/B from SLAM pose)
        self._waypoint_a = None  # [x, y] or None
        self._waypoint_b = None

        # Setup UI
        self._setup_ui()

        # ZMQ first (camera receiver needs shared context to avoid libzmq crashes)
        self.zmq_context = None
        self.status_socket = None
        self.camera_control_socket = None
        if ZMQ_AVAILABLE:
            self._setup_zmq()
            self._setup_camera_control()

        # Setup camera receiver (uses shared zmq_context; socket created in receive thread)
        self.camera_receiver = None
        print(f"[GUI] CAMERA_AVAILABLE = {CAMERA_AVAILABLE}")
        if CAMERA_AVAILABLE:
            self._setup_cameras()
        else:
            print("[GUI] Camera module not available!")

        # Setup timers
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update)
        self.update_timer.start(40)  # 25 Hz UI update (lower latency)

        # Camera update timer (separate, slower)
        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self._update_cameras)
        self.camera_timer.start(33)  # ~30Hz camera update
        self.yolo_checkbox.stateChanged.connect(self._on_stream_toggle)
        self.qr_checkbox.stateChanged.connect(self._on_stream_toggle)
        # Joystick button -> GUI action (edge detection)
        self._last_joystick_buttons = []
        # Back support (servo 7): raise/lower by step on button 7/8, not jump to 0/1023
        try:
            from shared.constants import BACK_SUPPORT_DOWN
            self._back_support_position = BACK_SUPPORT_DOWN
        except ImportError:
            self._back_support_position = 0
        if self._joy_buttons_debug:
            self.setWindowTitle(self.windowTitle() + " [Joystick button debug: press buttons to see ID]")
    
    def _setup_ui(self):
        """Setup the user interface."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # Status HUD (top)
        self.status_hud = StatusHUD()
        main_layout.addWidget(self.status_hud)
        self._last_cpu_temp = 0.0
        self._last_uptime = 0.0
        
        # Main content area (fills space)
        content_widget = QWidget()
        content_widget.setMinimumHeight(380)
        content_widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        content_layout = QHBoxLayout(content_widget)
        content_layout.setContentsMargins(12, 12, 12, 12)
        content_layout.setSpacing(12)

        # Left: Cameras
        camera_left = QFrame()
        camera_left.setObjectName("CameraPanel")
        camera_left.setStyleSheet("""
            QFrame#CameraPanel {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #0f0f1a, stop:1 #0a0a14);
                border: 1px solid #0f3460;
                border-radius: 10px;
                padding: 8px;
            }
        """)
        camera_left_layout = QVBoxLayout(camera_left)
        camera_left_layout.setContentsMargins(12, 12, 12, 12)
        camera_left_layout.setSpacing(8)
        cam_header = QHBoxLayout()
        cam_title = QLabel("Cameras")
        cam_title.setStyleSheet("font-size: 14px; font-weight: bold; color: #4ecca3;")
        cam_header.addWidget(cam_title)
        cam_header.addStretch()
        stream_label = QLabel("Stream:")
        stream_label.setStyleSheet("font-size: 12px; color: #a0a0a0;")
        cam_header.addWidget(stream_label)
        self.yolo_checkbox = QCheckBox("YOLO")
        self.yolo_checkbox.setChecked(True)
        self.yolo_checkbox.setToolTip("Disable to increase camera FPS (no object detection)")
        self.qr_checkbox = QCheckBox("QR")
        self.qr_checkbox.setChecked(True)
        self.qr_checkbox.setToolTip("Disable to increase camera FPS (no QR scan)")
        cam_header.addWidget(self.yolo_checkbox)
        cam_header.addWidget(self.qr_checkbox)
        camera_left_layout.addLayout(cam_header)
        self.camera_grid = CameraGrid()
        self.btn_show_slam = QPushButton("Show SLAM")
        self.btn_show_slam.setToolTip("Show SLAM / RViz panel")
        self.btn_show_slam.clicked.connect(self._on_show_slam_clicked)
        self.btn_show_slam.setVisible(False)  # shown when SLAM panel is closed
        cam_header.addWidget(self.btn_show_slam)
        camera_left_layout.addWidget(self.camera_grid, 1)
        # Landolt-C readout (large text for judge / Linear Rail Inspect)
        self.landolt_label = QLabel("Landolt-C: —")
        self.landolt_label.setStyleSheet(
            "color: #00cccc; font-size: 16px; font-weight: bold; padding: 6px 10px; "
            "background: #0a0a14; border-radius: 6px; border: 1px solid #0f3460;"
        )
        self.landolt_label.setMinimumHeight(36)
        camera_left_layout.addWidget(self.landolt_label)
        content_layout.addWidget(camera_left, 3)

        # Right panel: RViz SLAM stream (closeable so cameras can use full width)
        self.slam_panel_wrapper = QWidget()
        slam_panel_layout = QVBoxLayout(self.slam_panel_wrapper)
        slam_panel_layout.setContentsMargins(0, 0, 0, 0)
        slam_panel_layout.setSpacing(6)
        slam_header = QHBoxLayout()
        slam_title = QLabel("SLAM / RViz")
        slam_title.setStyleSheet("font-size: 14px; font-weight: bold; color: #4ecca3;")
        slam_header.addWidget(slam_title)
        slam_header.addStretch()
        self.btn_close_slam = QPushButton("Close")
        self.btn_close_slam.setToolTip("Hide SLAM panel — cameras expand to full width")
        self.btn_close_slam.clicked.connect(self._on_close_slam_clicked)
        self.btn_close_slam.setStyleSheet(
            "QPushButton { padding: 4px 12px; background: #0f3460; color: #e0e0e0; border-radius: 4px; }"
            "QPushButton:hover { background: #16213e; }"
        )
        slam_header.addWidget(self.btn_close_slam)
        slam_panel_layout.addLayout(slam_header)
        if RVizStreamView is not None:
            self.rviz_stream = RVizStreamView(self, jetson_ip=self.jetson_ip)
            self.rviz_stream.setMinimumSize(480, 360)
            self.rviz_stream.setStyleSheet("""
                QFrame {
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                        stop:0 #0f0f1a, stop:1 #0a0a14);
                    border: 1px solid #0f3460;
                    border-radius: 10px;
                }
            """)
            slam_panel_layout.addWidget(self.rviz_stream, 1)
        else:
            self.rviz_stream = None
            placeholder = QLabel("RViz stream not available (rviz_stream_view)")
            placeholder.setMinimumSize(500, 380)
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setStyleSheet(
                "background: #0a0a14; color: #888; padding: 24px; font-size: 15px; border-radius: 8px;"
            )
            slam_panel_layout.addWidget(placeholder, 1)
        self.slam_panel_wrapper.setStyleSheet("")
        content_layout.addWidget(self.slam_panel_wrapper, 3)
        
        # Footer: control panel with detection log
        self.detection_log = DetectionLog(parent=self, max_height=64)
        self.detection_log.qr_added_for_judge.connect(self._on_qr_for_judge)
        self.control_panel = ControlPanel(middle_widget=self.detection_log)
        self.control_panel.setToolTip(
            "Joystick: 0=Start, 1=Stop, 2=Lap, 3=Report, 4=YOLO, 5=QR, 6=Stowed, 7=Back support UP, 8=Back support DOWN, 9=Mapping, 10=Sensor Cab. Use --joy-buttons-debug to see ID on press."
        )
        self.control_panel.reset_button.clicked.connect(self._on_mission_reset_clicked)
        self.teleop_panel = TeleopPanel(self.control_panel)
        self.teleop_panel.start_mission.connect(self._start_mission)
        self.teleop_panel.stop_mission.connect(self._stop_mission)
        self.teleop_panel.generate_report.connect(self._generate_report)
        self.teleop_panel.lap_teleop_clicked.connect(self._on_lap_teleop)
        self.teleop_panel.start_autonomy.connect(self._on_start_autonomy)
        self.teleop_panel.scoring_clicked.connect(self._on_scoring_clicked)
        self.teleop_panel.set_waypoint_a.connect(self._on_set_waypoint_a)
        self.teleop_panel.set_waypoint_b.connect(self._on_set_waypoint_b)
        self.teleop_panel.add_fiducial_clicked.connect(self._on_add_fiducial)
        self.autonomy_panel = AutonomyPanel()
        self.autonomy_panel.abort_requested.connect(self._on_abort_autonomy)
        self.autonomy_panel.resume_requested.connect(self._on_resume_autonomy)
        self.autonomy_panel.generate_report.connect(self._generate_report)
        # RMRC 2025 auto mission status widget (labels + Start Auto Lap, Stowed, Select Test)
        self.auto_status_widget = None
        autonomy_container = QWidget()
        autonomy_layout = QVBoxLayout(autonomy_container)
        autonomy_layout.setContentsMargins(0, 0, 0, 0)
        autonomy_layout.addWidget(self.autonomy_panel)
        if AUTO_STATUS_WIDGET_AVAILABLE and create_and_connect_auto_status:
            try:
                def _send_autonomy_cmd(msg):
                    if getattr(self, "autonomy_socket", None):
                        try:
                            self.autonomy_socket.send_json(msg)
                        except Exception:
                            pass
                self.auto_status_widget = create_and_connect_auto_status(
                    self, self.jetson_ip, autonomy_socket_send=_send_autonomy_cmd
                )
                if self.auto_status_widget:
                    autonomy_layout.addWidget(self.auto_status_widget)
            except Exception as e:
                print(f"[GUI] Auto status widget: {e}")
        self.snapshot_panel = SnapshotReviewPanel()
        self.snapshot_panel.abort_requested.connect(self._on_abort_autonomy)
        self.snapshot_panel.accept_declare.connect(self._on_snapshot_accept)
        self.snapshot_panel.retry.connect(self._on_snapshot_retry)
        self.snapshot_panel.record_reading.connect(self._on_snapshot_record_reading)
        self.control_stacked = QStackedWidget()
        self.control_stacked.addWidget(self.teleop_panel)
        self.control_stacked.addWidget(autonomy_container)
        self.control_stacked.addWidget(self.snapshot_panel)
        
        main_layout.addWidget(content_widget, 1)  # Body fills available space
        main_layout.addWidget(self.control_stacked, 0)  # Footer fixed size
        
        self._scoring_state = {"keypad_clean": 0, "keypad_dirty": 0, "stairs_teleop": 0, "stairs_auto": 0,
                               "align_teleop": 0, "align_auto": 0, "drop_15": 0, "drop_30": 0,
                               "objects_removed": 0, "objects_in_container": 0, "inspect_readings": []}
        self._load_fiducials_from_file()
        self.set_mode("teleop")
        self.statusBar().showMessage("Ready")
    

    def _setup_cameras(self):
        """Setup camera receiver. Uses shared zmq_context to avoid libzmq crashes."""
        try:
            print(f"[GUI] Setting up camera receiver for {self.jetson_ip}:{ZMQ_PORT_CAMERA}")
            self.camera_receiver = CameraReceiver(
                jetson_ip=self.jetson_ip,
                port=ZMQ_PORT_CAMERA,
                zmq_context=self.zmq_context,
            )
            self.camera_receiver.start()
            print(f"[GUI] Camera receiver started successfully")
        except Exception as e:
            print(f"[GUI] Camera setup error: {e}")
            import traceback
            traceback.print_exc()
            self.camera_receiver = None
    
    def _setup_zmq(self):
        """Setup ZMQ subscriber for robot status, SLAM pose, autonomy status, snapshot_result; PUSH for autonomy and arm."""
        self.autonomy_socket = None
        self.autonomy_status_socket = None
        self.snapshot_result_socket = None
        self.arm_socket = None
        self.slam_socket = None
        self._slam_pose = None  # (x, y, yaw) from SLAM bridge
        try:
            self.zmq_context = zmq.Context()
            self.status_socket = self.zmq_context.socket(zmq.SUB)
            self.status_socket.setsockopt(zmq.SUBSCRIBE, b"")
            self.status_socket.setsockopt(zmq.RCVTIMEO, 10)
            self.status_socket.setsockopt(zmq.CONFLATE, 1)  # Only latest
            self.status_socket.connect(f"tcp://{self.jetson_ip}:{ZMQ_PORT_STATUS}")
            # SLAM pose subscriber (port 5562) — for waypoints, fiducials, dexterity guard
            self.slam_socket = self.zmq_context.socket(zmq.SUB)
            self.slam_socket.setsockopt(zmq.SUBSCRIBE, b"")
            self.slam_socket.setsockopt(zmq.RCVTIMEO, 5)
            self.slam_socket.setsockopt(zmq.CONFLATE, 1)
            self.slam_socket.connect(f"tcp://{self.jetson_ip}:{ZMQ_PORT_SLAM}")
            self.autonomy_socket = self.zmq_context.socket(zmq.PUSH)
            self.autonomy_socket.setsockopt(zmq.LINGER, 0)
            self.autonomy_socket.connect(f"tcp://{self.jetson_ip}:{ZMQ_PORT_AUTONOMY}")
            self.arm_socket = self.zmq_context.socket(zmq.PUSH)
            self.arm_socket.setsockopt(zmq.LINGER, 0)
            self.arm_socket.connect(f"tcp://{self.jetson_ip}:{ZMQ_PORT_ARM}")
            self.autonomy_status_socket = self.zmq_context.socket(zmq.SUB)
            self.autonomy_status_socket.setsockopt(zmq.SUBSCRIBE, b"")
            self.autonomy_status_socket.setsockopt(zmq.RCVTIMEO, 10)
            self.autonomy_status_socket.setsockopt(zmq.CONFLATE, 1)
            self.autonomy_status_socket.connect(f"tcp://{self.jetson_ip}:{ZMQ_PORT_AUTONOMY_STATUS}")
            self.snapshot_result_socket = self.zmq_context.socket(zmq.SUB)
            self.snapshot_result_socket.setsockopt(zmq.SUBSCRIBE, b"")
            self.snapshot_result_socket.setsockopt(zmq.RCVTIMEO, 10)
            self.snapshot_result_socket.connect(f"tcp://{self.jetson_ip}:{ZMQ_PORT_SNAPSHOT_RESULT}")
        except Exception as e:
            print(f"ZMQ setup error: {e}")
            self.autonomy_socket = None
            self.autonomy_status_socket = None
            self.snapshot_result_socket = None
            self.arm_socket = None
            self.slam_socket = None

    def _setup_camera_control(self):
        """Setup ZMQ PUB to Jetson to toggle YOLO/QR (higher FPS when off)."""
        try:
            self.camera_control_socket = self.zmq_context.socket(zmq.PUB)
            self.camera_control_socket.setsockopt(zmq.LINGER, 0)
            self.camera_control_socket.connect(f"tcp://{self.jetson_ip}:{ZMQ_PORT_CAMERA_CONTROL}")
            # Send initial state so Jetson knows defaults
            self._send_stream_control()
        except Exception as e:
            print(f"[GUI] Camera control socket error: {e}")
            self.camera_control_socket = None

    def _send_stream_control(self):
        """Send enable_yolo / enable_qr / enable_landolt to Jetson (vision node runs HAZMAT+Landolt combined)."""
        if self.camera_control_socket is None:
            return
        try:
            msg = json.dumps({
                "enable_yolo": self.yolo_checkbox.isChecked(),
                "enable_qr": self.qr_checkbox.isChecked(),
                "enable_landolt": True,
            })
            self.camera_control_socket.send_string(msg)
        except Exception:
            pass

    def _on_stream_toggle(self, _state):
        """Called when YOLO or QR checkbox changes; send new state to Jetson."""
        self._send_stream_control()

    def set_mode(self, mode: str):
        """Set GUI mode: teleop (green), autonomy (amber), snapshot_review (blue). Swaps control panel and HUD accent."""
        if mode not in ("teleop", "autonomy", "snapshot_review"):
            return
        # Defer to next event loop to avoid Qt re-entrancy crash when called from button click
        QTimer.singleShot(0, lambda m=mode: self._do_set_mode(m))

    def _do_set_mode(self, mode: str):
        """Internal: perform mode switch (called deferred)."""
        self.gui_mode = mode
        colors = {"teleop": "#4ecca3", "autonomy": "#f59e0b", "snapshot_review": "#3b82f6"}
        self.status_hud.status_label.setStyleSheet(f"color: {colors.get(mode, colors['teleop'])}; font-size: 14px; font-weight: bold;")
        idx = {"teleop": 0, "autonomy": 1, "snapshot_review": 2}[mode]
        self.control_stacked.setCurrentIndex(idx)
        if hasattr(self, "camera_grid") and hasattr(self.camera_grid, "set_arm_expanded"):
            if mode == "teleop" or mode == "snapshot_review":
                self.camera_grid.set_arm_expanded(False)
            elif mode == "autonomy":
                test_id = getattr(self.teleop_panel, "get_current_test_id", lambda: "")()
                self.camera_grid.set_arm_expanded(test_id in ("sensor_cabinet", "linear_rail_inspect"))
        if mode == "autonomy":
            self.status_hud.set_status("AUTONOMY — HANDS OFF")
        elif mode == "snapshot_review":
            self.status_hud.set_status("REVIEW — Snapshot")
        else:
            self.status_hud.set_status("TELEOP — Ready")

    def _explore_waypoints_from_gui(self):
        """Return list of [x,y] for explore: fiducials, or waypoint A+B, or default box."""
        try:
            from shared.constants import FIDUCIAL_POSITIONS
            if FIDUCIAL_POSITIONS and len(FIDUCIAL_POSITIONS) >= 2:
                return [[float(p[0]), float(p[1])] for p in FIDUCIAL_POSITIONS if len(p) >= 2]
        except Exception:
            pass
        way_a = getattr(self, "_waypoint_a", None)
        way_b = getattr(self, "_waypoint_b", None)
        if way_a and way_b:
            return [way_a, way_b]
        return [[1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.0, 0.0]]

    def _send_autonomy_cmd(self, mode: str, **kwargs):
        """Send autonomy command to Jetson (mapping, sensor_cabinet, lap, pick, etc.)."""
        if getattr(self, "autonomy_socket", None) is None:
            self.statusBar().showMessage("Autonomy socket not connected")
            return
        try:
            msg = {"mode": mode, **kwargs}
            self.autonomy_socket.send_json(msg)
            self.statusBar().showMessage(f"Autonomy: {mode}")
        except Exception as e:
            self.statusBar().showMessage(f"Autonomy send error: {e}")

    def _back_support_raise(self):
        """Raise back support by one step (button 7)."""
        try:
            from shared.constants import BACK_SUPPORT_UP, BACK_SUPPORT_STEP
            step = BACK_SUPPORT_STEP
            max_pos = BACK_SUPPORT_UP
        except ImportError:
            step, max_pos = 60, 1023
        self._back_support_position = min(max_pos, self._back_support_position + step)
        self._send_back_support(self._back_support_position)

    def _back_support_lower(self):
        """Lower back support by one step (button 8)."""
        try:
            from shared.constants import BACK_SUPPORT_DOWN, BACK_SUPPORT_STEP
            step = BACK_SUPPORT_STEP
            min_pos = BACK_SUPPORT_DOWN
        except ImportError:
            step, min_pos = 60, 0
        self._back_support_position = max(min_pos, self._back_support_position - step)
        self._send_back_support(self._back_support_position)

    def _send_back_support(self, position: int):
        """Send back support (1 DOF kickstand) to Jetson. Servo ID 7: 0=down/stow, 1023=up/deploy."""
        if getattr(self, "arm_socket", None) is None:
            return
        try:
            self.arm_socket.send_json({
                "msg_type": "arm_cmd",
                "positions": {},  # Don't move arm joints
                "gripper": position,  # Servo 7 on arm bus = back support
            })
            self.statusBar().showMessage(f"Back support: {position}", 1500)
        except Exception as e:
            self.statusBar().showMessage(f"Back support send error: {e}", 2000)

    def _dexterity_target_too_close(self, target_xy) -> bool:
        """GUI pre-check: rulebook 2026-04-12 dexterity 30 cm guard.

        Returns True if a target_xy is provided and the current SLAM pose is
        closer than DEXTERITY_AUTONOMY_MIN_DIST_M to it. The Jetson re-checks
        as authoritative; this is just a fast operator nudge.
        """
        if not target_xy:
            return False
        try:
            from shared.constants import DEXTERITY_AUTONOMY_MIN_DIST_M
        except ImportError:
            DEXTERITY_AUTONOMY_MIN_DIST_M = 0.30
        pose = self._get_slam_pose()
        if not pose:
            return False
        try:
            dx = float(pose[0]) - float(target_xy[0])
            dy = float(pose[1]) - float(target_xy[1])
        except (TypeError, ValueError, IndexError):
            return False
        dist = (dx * dx + dy * dy) ** 0.5
        if dist < DEXTERITY_AUTONOMY_MIN_DIST_M:
            self.statusBar().showMessage(
                f"Too close to target ({dist*100:.0f} cm < "
                f"{DEXTERITY_AUTONOMY_MIN_DIST_M*100:.0f} cm) — drive back to claim 4×.",
                5000,
            )
            QApplication.beep()
            return True
        return False

    def _on_start_autonomy(self, mode_or_test: str):
        """Start autonomy: require Stowed, send mode from test, switch to autonomy panel."""
        if not self.control_panel.stowed_checkbox.isChecked():
            self.statusBar().showMessage("Check Stowed before starting autonomy (4× bonus)")
            return
        test_id = getattr(self.teleop_panel, "get_current_test_id", lambda: "sand_gravel")()
        mobility = ("incline_horiz", "incline_inclined", "sand_gravel", "ramps_continuous", "ramps_pinwheel",
                    "elevated_ramps", "krails_horiz", "krails_crossover", "hurdles_single", "hurdles_double")
        if mode_or_test == "lap" or test_id in mobility:
            way_a = getattr(self, "_waypoint_a", None) or [0.0, 0.0]
            way_b = getattr(self, "_waypoint_b", None) or [2.0, 0.0]
            self._send_autonomy_cmd("lap", waypoint_a=way_a, waypoint_b=way_b)
        elif test_id == "sensor_cabinet":
            target = getattr(self, "_waypoint_b", None) or getattr(self, "_waypoint_a", None)
            if self._dexterity_target_too_close(target):
                return
            self._send_autonomy_cmd("sensor_cabinet", target_xy=target)
        elif test_id == "keypad_omni":
            target = getattr(self, "_waypoint_b", None) or getattr(self, "_waypoint_a", None)
            if self._dexterity_target_too_close(target):
                return
            self._send_autonomy_cmd("keypad", target_xy=target)
        elif test_id == "linear_rail_pick":
            pick_mode = self.teleop_panel.get_pick_mode_id()
            pick_goal = getattr(self, "_waypoint_b", None) or getattr(self, "_waypoint_a", None)
            if pick_goal is None and hasattr(self, "slam_view"):
                pose = self._get_slam_pose()
                pick_goal = [pose[0], pose[1]] if pose else None
            if self._dexterity_target_too_close(pick_goal):
                return
            self._send_autonomy_cmd("pick", pick_mode=pick_mode, pick_goal=pick_goal)
        elif test_id == "linear_rail_inspect":
            target = getattr(self, "_waypoint_b", None) or getattr(self, "_waypoint_a", None)
            if self._dexterity_target_too_close(target):
                return
            self._send_autonomy_cmd("inspect_tube", target_xy=target)
        elif test_id in ("labyrinth_flat", "labyrinth_krails"):
            # Prefer frontier-based exploration for real autonomy points
            self._send_autonomy_cmd("explore_labyrinth")
        else:
            self._send_autonomy_cmd(mode_or_test)
        self.set_mode("autonomy")

    def _on_scoring_clicked(self, key: str):
        """Update scoring state for mission report."""
        s = getattr(self, "_scoring_state", {})
        if key == "lap_teleop":
            self.mission_laps_teleop += 1
            self.statusBar().showMessage(f"Lap (T): {self.mission_laps_teleop}")
        elif key == "lap_auto":
            self.mission_laps_auto += 1
            self.statusBar().showMessage(f"Lap (A): {self.mission_laps_auto}")
        elif key in s:
            s[key] = s.get(key, 0) + 1
            self.statusBar().showMessage(f"Scoring: {key}")
        elif key == "pick_removed":
            s["objects_removed"] = s.get("objects_removed", 0) + 1
        elif key == "pick_in_container":
            s["objects_in_container"] = s.get("objects_in_container", 0) + 1
        elif key == "inspect_record" and hasattr(self, "teleop_panel"):
            r = self.teleop_panel.get_inspect_reading()
            s.setdefault("inspect_readings", []).append(r)

    def _on_set_waypoint_a(self):
        pose = self._get_slam_pose()
        if pose:
            self._waypoint_a = [round(pose[0], 3), round(pose[1], 3)]
            self.statusBar().showMessage(f"Waypoint A set: ({pose[0]:.2f}, {pose[1]:.2f})")
        else:
            self._waypoint_a = None
            self.statusBar().showMessage("No SLAM pose — Waypoint A not set")
    def _on_set_waypoint_b(self):
        pose = self._get_slam_pose()
        if pose:
            self._waypoint_b = [round(pose[0], 3), round(pose[1], 3)]
            self.statusBar().showMessage(f"Waypoint B set: ({pose[0]:.2f}, {pose[1]:.2f})")
        else:
            self._waypoint_b = None
            self.statusBar().showMessage("No SLAM pose — Waypoint B not set")

    def _load_fiducials_from_file(self):
        """Load reports/fiducials.json into shared.constants.FIDUCIAL_POSITIONS at startup."""
        import os
        import json
        try:
            reports_dir = os.path.join(os.path.dirname(__file__), "..", "..", "reports")
            path = os.path.join(reports_dir, "fiducials.json")
            if os.path.isfile(path):
                with open(path, "r") as f:
                    data = json.load(f)
                if isinstance(data, list) and len(data) > 0:
                    from shared import constants as sc
                    sc.FIDUCIAL_POSITIONS = [[float(p[0]), float(p[1])] for p in data if len(p) >= 2]
                    print(f"[GUI] Loaded {len(sc.FIDUCIAL_POSITIONS)} fiducials from {path}")
        except Exception as e:
            print(f"[GUI] Could not load fiducials: {e}")

    def _on_add_fiducial(self):
        """Append current SLAM pose to reports/fiducials.json."""
        pose = self._get_slam_pose()
        if not pose:
            self.statusBar().showMessage("No SLAM pose available")
            return
        import json
        import os
        reports_dir = os.path.join(os.path.dirname(__file__), "..", "..", "reports")
        path = os.path.join(reports_dir, "fiducials.json")
        try:
            os.makedirs(reports_dir, exist_ok=True)
            data = []
            if os.path.exists(path):
                with open(path, "r") as f:
                    data = json.load(f)
            if not isinstance(data, list):
                data = []
            data.append([round(pose[0], 3), round(pose[1], 3)])
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            self._load_fiducials_from_file()
            self.statusBar().showMessage(f"Fiducial added: ({pose[0]:.2f}, {pose[1]:.2f}) — {len(data)} total in {path}")
        except Exception as e:
            self.statusBar().showMessage(f"Failed to save fiducial: {e}")

    def _on_abort_autonomy(self):
        self._send_autonomy_cmd("abort")
        self.set_mode("teleop")
        self._stop_mission()

    def _on_resume_autonomy(self):
        """Resume after Pause for Base Teleop (Linear Rail Pick)."""
        self._send_autonomy_cmd("resume")

    def _on_snapshot_accept(self):
        self.set_mode("autonomy")
    def _on_snapshot_retry(self):
        self._send_autonomy_cmd("snapshot_retry")
        self.set_mode("autonomy")
    def _on_snapshot_record_reading(self):
        if hasattr(self, "snapshot_panel"):
            d = self.snapshot_panel.direction_combo.currentText()
            getattr(self, "_scoring_state", {}).setdefault("inspect_readings", []).append({"direction": d})

    def enter_snapshot_review(self, identity: str = "", direction: str = "", result_type: str = "hazmat"):
        """Switch to snapshot review panel (e.g. when snapshot_result received)."""
        self.set_mode("snapshot_review")
        if result_type == "landolt":
            self.snapshot_panel.set_result(f"Landolt: {direction}" if direction else "Landolt")
        else:
            self.snapshot_panel.set_result(f"HAZMAT: {identity}" if identity else "HAZMAT")
        self.snapshot_panel.direction_combo.setCurrentText(direction) if direction else None

    def _in_30cm_endwall_zone(self):
        """Rulebook: joystick allowed within 30cm of end walls. True = in zone (don't abort)."""
        try:
            from shared.constants import FIDUCIAL_POSITIONS, END_WALL_EXCEPTION_M
        except ImportError:
            return False
        xy = self._get_slam_pose()
        if not xy or len(xy) < 2:
            return False
        x, y = xy[0], xy[1]
        for (fx, fy) in (FIDUCIAL_POSITIONS or []):
            d = ((x - fx) ** 2 + (y - fy) ** 2) ** 0.5
            if d <= END_WALL_EXCEPTION_M:
                return True
        return False
    
    def _update(self):
        """Periodic update callback."""
        # Update mission timer
        if self.mission_active:
            elapsed = time.time() - self.mission_start_time
            self.remaining_time = max(0, MISSION_DURATION - int(elapsed))
            self.status_hud.set_timer(self.remaining_time)

            # Audio/visual alerts at key thresholds
            if not hasattr(self, '_alert_60_fired'):
                self._alert_60_fired = False
                self._alert_30_fired = False
            if self.remaining_time <= 60 and not self._alert_60_fired:
                self._alert_60_fired = True
                self.statusBar().showMessage("⚠ 1 MINUTE REMAINING!")
                QApplication.beep()
            if self.remaining_time <= 30 and not self._alert_30_fired:
                self._alert_30_fired = True
                self.statusBar().showMessage("⚠ 30 SECONDS REMAINING!")
                QApplication.beep()

            if self.remaining_time <= 0:
                self._stop_mission()
        
        # Poll SLAM pose (port 5562)
        if getattr(self, 'slam_socket', None):
            try:
                slam_msg = self.slam_socket.recv_json(zmq.NOBLOCK)
                if 'x' in slam_msg and 'y' in slam_msg:
                    self._slam_pose = (float(slam_msg['x']), float(slam_msg['y']))
            except (zmq.Again, Exception):
                pass
        
        # Poll ZMQ for status updates
        if self.status_socket:
            try:
                msg = self.status_socket.recv_json(zmq.NOBLOCK)
                self._handle_status(msg)
                self._last_status_time = time.time()
            except zmq.Again:
                # Check for connection loss
                if hasattr(self, '_last_status_time') and time.time() - self._last_status_time > 3.0:
                    self.status_hud.set_connection("jetson", False)
                # Auto-reconnect after timeout
                if hasattr(self, '_last_status_time') and time.time() - self._last_status_time > ZMQ_RECONNECT_TIMEOUT_S:
                    self._reconnect_zmq()
        # Poll autonomy status (laps, Nav2 overlay for SLAM)
        if getattr(self, "autonomy_status_socket", None):
            try:
                msg = self.autonomy_status_socket.recv_json(zmq.NOBLOCK)
                if self.mission_active:
                    laps = msg.get("laps_completed", 0)
                    if isinstance(laps, (int, float)):
                        self.mission_laps_auto = int(laps)
                # Nav2 overlay for ego-centric SLAM (autonomy mode)
                if hasattr(self, "slam_view") and hasattr(self.slam_view, "set_nav2_overlay"):
                    path = msg.get("nav2_path")
                    goal = msg.get("nav2_goal")
                    dist = msg.get("goal_distance_m")
                    if isinstance(goal, list) and len(goal) >= 2:
                        self.slam_view.set_nav2_overlay(path, (goal[0], goal[1]), dist)
                    else:
                        self.slam_view.set_nav2_overlay(path, None, dist)
                # Update autonomy panel (goal, step, laps)
                if hasattr(self, "autonomy_panel"):
                    step = msg.get("step") or "—"
                    step_label = msg.get("step_label") or "—"
                    self.autonomy_panel.set_step(str(step), str(step_label))
                    self.autonomy_panel.set_laps(int(msg.get("laps_completed", 0)))
                    self.autonomy_panel.set_paused(msg.get("state") == "paused")
                    goal_name = "B" if msg.get("nav2_goal") else "—"
                    dist_m = msg.get("goal_distance_m") or 0.0
                    self.autonomy_panel.set_nav2_status(goal_name, dist_m, int(msg.get("progress", 0)))
                # RMRC 2025 auto mission status widget (30cm zone, test type, laps)
                if getattr(self, "auto_status_widget", None) and msg.get("msg_type") == "auto_mission_status":
                    self.auto_status_widget.set_status(msg)
            except zmq.Again:
                pass
            except Exception:
                pass
        # Snapshot result (vision -> GUI): switch to snapshot review
        if getattr(self, "snapshot_result_socket", None):
            try:
                parts = self.snapshot_result_socket.recv_multipart(zmq.NOBLOCK)
                if len(parts) >= 1:
                    meta = json.loads(parts[0].decode("utf-8"))
                    if meta.get("msg_type") == "snapshot_result":
                        identity = meta.get("identity", "")
                        direction = meta.get("direction", "")
                        result_type = meta.get("result_type", "hazmat")
                        image_bytes = parts[1] if len(parts) > 1 and parts[1] else None
                        if image_bytes and hasattr(self, "_snapshot_frozen_image"):
                            self._snapshot_frozen_image = image_bytes
                        if hasattr(self, "enter_snapshot_review"):
                            self.enter_snapshot_review(identity=identity, direction=direction, result_type=result_type)
            except zmq.Again:
                pass
            except Exception:
                pass

    def _apply_camera_frame(self, display_name: str, cam_id: str, frame_data) -> None:
        """Apply one camera frame to a grid slot and log detections/QR."""
        self.camera_grid.update_camera(
            display_name,
            frame_data.frame,
            frame_data.detections,
            frame_data.qr_codes,
            getattr(frame_data, "landolt_readings", None) or []
        )
        if frame_data.detections:
            for det in frame_data.detections:
                if det.get("confidence", 0) > 0.6:
                    self.detection_log.add_detection(
                        cam_id,
                        det.get("class_name", "unknown"),
                        det.get("confidence", 0),
                        det.get("is_victim", False)
                    )
        if frame_data.qr_codes:
            pose_xy = None
            pose_xy = self._get_slam_pose()
            for qr in frame_data.qr_codes:
                self.detection_log.add_qr_code(cam_id, qr.get("data", ""), pose_xy=pose_xy)
        # Push QR-on-map entries to SLAM view for drawing
        if hasattr(self, "slam_view") and hasattr(self.slam_view, "set_qr_on_map"):
            self.slam_view.set_qr_on_map(getattr(self.detection_log, "qr_entries", []))
    
    def _update_cameras(self):
        """Update camera displays from receiver. Fills front/arm/backward by camera_id (Jetson or Pi)."""
        if not self.camera_receiver:
            return
        # Poll ZMQ on main thread only (avoids libzmq fq.cpp crash)
        self.camera_receiver.poll()

        # Map logical camera IDs to display slots.
        # Swap front/arm so the physical cameras appear in the desired places on the GUI.
        slot_mappings = [
            ("front", "arm"),       # show arm camera feed in the Front slot
            ("arm", "front"),       # show front camera feed in the Arm slot
            ("backward", "backward"),
        ]
        active_cameras = 0
        latest_landolt = []  # aggregate from front + arm for GUI readout
        for display_name, cam_id in slot_mappings:
            frame_data = self.camera_receiver.get_frame_data(cam_id)
            # Backward: if stale (no new frame in 1.5s), show No Signal so user knows stream stopped
            if display_name == "backward" and hasattr(self, "camera_grid") and "backward" in self.camera_grid.cameras:
                if not self.camera_receiver.is_camera_active("backward", timeout=1.5):
                    self.camera_grid.cameras["backward"].set_no_signal()
                    continue
            if frame_data is not None:
                self._apply_camera_frame(display_name, cam_id, frame_data)
                active_cameras += 1
                if cam_id in ("front", "arm"):
                    readings = getattr(frame_data, "landolt_readings", None) or []
                    if readings:
                        latest_landolt = readings

        if hasattr(self, "landolt_label"):
            if latest_landolt:
                parts = []
                for r in latest_landolt:
                    d = r.get("direction", "")
                    c = r.get("confidence", 0)
                    if d:
                        parts.append(f"{d} {c:.0%}")
                self.landolt_label.setText("Landolt-C: " + ("  |  ".join(parts) if parts else "—"))
            else:
                self.landolt_label.setText("Landolt-C: —")

        self.status_hud.set_robot_info(
            self._last_cpu_temp, self._last_uptime, active_cameras
        )
    
    def _reconnect_zmq(self):
        """Tear down and recreate all ZMQ sockets (auto-reconnect after Jetson reboot)."""
        print("[GUI] ZMQ reconnecting...")
        for attr in ('status_socket', 'slam_socket', 'autonomy_socket', 'arm_socket',
                     'autonomy_status_socket', 'snapshot_result_socket', 'camera_control_socket'):
            sock = getattr(self, attr, None)
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
                setattr(self, attr, None)
        # Recreate (reuse existing context)
        if self.zmq_context:
            self._setup_zmq()
            self._setup_camera_control()
        self._last_status_time = time.time()  # prevent immediate re-trigger
        self.statusBar().showMessage("ZMQ reconnected", 3000)

    def _handle_status(self, status: dict):
        """Handle status update from robot."""
        self.status_hud.set_connection("pico", status.get("pico_connected", False))
        self.status_hud.set_connection("jetson", True)
        
        left = status.get("left_speed", 0)
        right = status.get("right_speed", 0)
        moving = abs(left) > 0 or abs(right) > 0
        self.control_panel.update_drive(left, right, moving)
        
        cpu_temp = status.get("cpu_temp", 0)
        uptime = status.get("uptime", 0)
        cameras = status.get("cameras_active", 0)
        self._last_cpu_temp = cpu_temp
        self._last_uptime = uptime
        self.status_hud.set_robot_info(cpu_temp, uptime, cameras)
    
    def _on_mission_reset_clicked(self):
        """Operator-initiated mini-mission reset (rulebook 2.5).

        Broadcasts mission_reset on the autonomy port, clears in-mission counters,
        and re-arms the mission timer so the run continues with the remaining time.
        Persistent best scores in ~/.rmrc/best_scores.json are preserved.
        """
        try:
            from shared.mission_reset import broadcast_mission_reset
        except ImportError:
            broadcast_mission_reset = None
        # Increment mini-mission index so the report can list each one.
        self._mini_mission_index = getattr(self, "_mini_mission_index", 0) + 1
        if broadcast_mission_reset and getattr(self, "autonomy_socket", None):
            try:
                broadcast_mission_reset(
                    self.autonomy_socket,
                    mission_id=getattr(self, "_mission_id", ""),
                    mini_mission_index=self._mini_mission_index,
                    reason="operator",
                )
            except Exception:
                pass
        # Local clear (preserves persistent best score)
        self.mission_laps_teleop = 0
        self.mission_laps_auto = 0
        self._scoring_state = {
            "keypad_clean": 0, "keypad_dirty": 0, "stairs_teleop": 0, "stairs_auto": 0,
            "align_teleop": 0, "align_auto": 0, "drop_15": 0, "drop_30": 0,
            "objects_removed": 0, "objects_in_container": 0, "inspect_readings": [],
        }
        if hasattr(self, "detection_log"):
            for attr in ("_seen_qr_data", "qr_entries"):
                obj = getattr(self.detection_log, attr, None)
                if obj is not None:
                    try:
                        obj.clear()
                    except Exception:
                        pass
            for c in ("qr_count", "detection_count", "hazmat_count", "landolt_count"):
                if hasattr(self.detection_log, c):
                    setattr(self.detection_log, c, 0)
            if hasattr(self.detection_log, "list_widget"):
                self.detection_log.list_widget.clear()
            if hasattr(self.detection_log, "_update_stats"):
                self.detection_log._update_stats()
        self.set_mode("teleop")
        self.statusBar().showMessage(
            f"Mission reset — mini-mission #{self._mini_mission_index}. Best mini-mission counts.",
            5000,
        )

    def _start_mission(self):
        """Start a new mission."""
        self.mission_active = True
        self.mission_start_time = time.time()
        # Reset detection log for fresh mission (rulebook: each QR scanned once per mission)
        if hasattr(self.detection_log, "_seen_qr_data"):
            self.detection_log._seen_qr_data.clear()
        if hasattr(self.detection_log, "qr_entries"):
            self.detection_log.qr_entries.clear()
        if hasattr(self.detection_log, "qr_count"):
            self.detection_log.qr_count = 0
        if hasattr(self.detection_log, "detection_count"):
            self.detection_log.detection_count = 0
        if hasattr(self.detection_log, "hazmat_count"):
            self.detection_log.hazmat_count = 0
        if hasattr(self.detection_log, "landolt_count"):
            self.detection_log.landolt_count = 0
        if hasattr(self.detection_log, "list_widget"):
            self.detection_log.list_widget.clear()
        self.detection_log._update_stats()
        # Clear live QR file for new mission (report will write fresh _qr_codes.txt)
        try:
            from shared.constants import QR_CODES_OUTPUT_FILE
            out_dir = os.path.join(os.path.dirname(__file__), "..", "..", "reports")
            path = os.path.join(out_dir, QR_CODES_OUTPUT_FILE)
            if os.path.exists(path):
                open(path, "w").close()
        except Exception:
            pass
        self.remaining_time = MISSION_DURATION
        self._alert_60_fired = False
        self._alert_30_fired = False
        
        self.status_hud.set_status("ACTIVE - Mission Running")
        self.status_hud.set_timer(MISSION_DURATION)
        self.statusBar().showMessage("Mission started")

    def _on_close_slam_clicked(self):
        """Hide SLAM / RViz panel and switch to full-width camera layout (one row of 3 equal cameras)."""
        if hasattr(self, "slam_panel_wrapper"):
            self.slam_panel_wrapper.setVisible(False)
        if hasattr(self, "btn_show_slam"):
            self.btn_show_slam.setVisible(True)
        if hasattr(self, "camera_grid") and hasattr(self.camera_grid, "set_full_width_layout"):
            self.camera_grid.set_full_width_layout(True)
        self.statusBar().showMessage("SLAM panel hidden — cameras full width; click 'Show SLAM' to restore")

    def _on_show_slam_clicked(self):
        """Show the SLAM / RViz panel and restore normal camera grid (front large left, arm/backward right)."""
        if hasattr(self, "slam_panel_wrapper"):
            self.slam_panel_wrapper.setVisible(True)
        if hasattr(self, "btn_show_slam"):
            self.btn_show_slam.setVisible(False)
        if hasattr(self, "camera_grid") and hasattr(self.camera_grid, "set_full_width_layout"):
            self.camera_grid.set_full_width_layout(False)
        self.statusBar().showMessage("SLAM panel visible")
    
    def _get_slam_pose(self):
        """Return (x, y) from SLAM ZMQ subscriber, or None if no pose yet."""
        return self._slam_pose if getattr(self, '_slam_pose', None) else None
    
    def _stop_mission(self):
        """Stop the current mission."""
        self.mission_active = False
        self._send_autonomy_cmd("disabled")
        self.status_hud.set_status("STOPPED - Mission Ended")
        self.statusBar().showMessage("Mission stopped")
    
    def _on_lap_teleop(self):
        """Increment teleop lap count (for report)."""
        self.mission_laps_teleop += 1
        self.statusBar().showMessage(f"Teleop laps: {self.mission_laps_teleop}")

    def _on_qr_for_judge(self):
        """Remind operator to declare QR and point to judge (rulebook)."""
        self.statusBar().showMessage("New QR read – point to judge and declare", 5000)
    
    def _generate_report(self):
        """Generate mission report PDF from current mission data and detection log."""
        self.statusBar().showMessage("Generating report...")
        generate_mission_report = None
        # Ensure laptop dir is on path so "reports" package can be found
        _gui_dir = os.path.dirname(os.path.abspath(__file__))
        _laptop_dir = os.path.dirname(_gui_dir)
        if _laptop_dir not in sys.path:
            sys.path.insert(0, _laptop_dir)
        try:
            from reports.mission_report import generate_mission_report
        except ImportError:
            try:
                from laptop.reports.mission_report import generate_mission_report
            except ImportError:
                try:
                    import importlib.util
                    _rp = os.path.join(_laptop_dir, "reports", "mission_report.py")
                    if os.path.exists(_rp):
                        spec = importlib.util.spec_from_file_location("mission_report", _rp)
                        mod = importlib.util.module_from_spec(spec)
                        spec.loader.exec_module(mod)
                        generate_mission_report = mod.generate_mission_report
                except Exception:
                    pass
        if not generate_mission_report:
            self.statusBar().showMessage("Report module not found. Check laptop/reports/mission_report.py")
            return
        duration_sec = int(time.time() - self.mission_start_time) if self.mission_active else 0
        if not self.mission_active and self.mission_start_time > 0:
            duration_sec = min(MISSION_DURATION, int(time.time() - self.mission_start_time))
        qr_entries = getattr(self.detection_log, "qr_entries", [])
        qr_mapped_count = sum(1 for e in qr_entries if isinstance(e, dict) and "map_x" in e and "map_y" in e)
        test_type = getattr(self.teleop_panel, "get_current_test_id", lambda: "sand_gravel")()
        scoring = getattr(self, "_scoring_state", {})
        mission_data = {
            "duration_sec": duration_sec,
            "test_type": test_type,
            "laps_teleop": self.mission_laps_teleop,
            "laps_auto": self.mission_laps_auto,
            "qr_read_count": getattr(self.detection_log, "qr_count", 0),
            "qr_mapped_count": qr_mapped_count,
            "hazmat_count": getattr(self.detection_log, "hazmat_count", 0),
            "landolt_count": getattr(self.detection_log, "landolt_count", 0),
            "objects_removed": scoring.get("objects_removed", 0),
            "objects_in_container": scoring.get("objects_in_container", 0),
            "keypad_clean": scoring.get("keypad_clean", 0),
            "keypad_dirty": scoring.get("keypad_dirty", 0),
            "stairs_teleop": scoring.get("stairs_teleop", 0),
            "stairs_auto": scoring.get("stairs_auto", 0),
            "align_teleop": scoring.get("align_teleop", 0),
            "align_auto": scoring.get("align_auto", 0),
            "drop_15": scoring.get("drop_15", 0),
            "drop_30": scoring.get("drop_30", 0),
            "inspect_readings": scoring.get("inspect_readings", []),
            "detection_count": getattr(self.detection_log, "detection_count", 0),
            "qr_entries": qr_entries,
        }
        out_path = os.path.join(os.path.expanduser("~"), "mission_report.pdf")
        try:
            if generate_mission_report(out_path, mission_data=mission_data):
                self.statusBar().showMessage(f"Report saved to {out_path}")
            else:
                self.statusBar().showMessage("Report failed. Install reportlab? pip install reportlab")
        except Exception as e:
            self.statusBar().showMessage(f"Report error: {e}")
    
    def update_joystick_state(self, state: dict):
        """Update joystick display and handle button presses via JOY_BUTTON_MAP."""
        connected = state.get("connected", False)
        axes = state.get("axes", {}) if "drive" not in state else state.get("drive", {})

        # Support both direct axes and drive command format
        if "drive" in state:
            drive = state["drive"]
            y = drive.get("y", 0)
            x = drive.get("x", 0)
            z = drive.get("z", 0)
            if isinstance(y, (int, float)) and abs(y) <= 1:
                y, x, z = int(y * 100), int(x * 100), int(z * 100)
        else:
            y = int(axes.get("y", 0) * 100)
            x = int(axes.get("x", 0) * 100)
            z = int(axes.get("z", 0) * 100)

        self.control_panel.update_joystick(connected, y, x, z)

        # Joystick safety interlock (rulebook): in autonomy, if joystick beyond deadzone >500ms -> teleop
        try:
            from shared.constants import JOYSTICK_DEADZONE
            deadzone = int(JOYSTICK_DEADZONE * 100)
        except ImportError:
            deadzone = 5
        if self.gui_mode == "autonomy":
            if abs(y) > deadzone or abs(x) > deadzone or abs(z) > deadzone:
                if self._in_30cm_endwall_zone():
                    self._joystick_activity_start = None
                else:
                    now = time.time()
                    if self._joystick_activity_start is None:
                        self._joystick_activity_start = now
                    elif now - self._joystick_activity_start > 0.5:
                        self.statusBar().showMessage("Joystick detected — lap becomes teleop!", 5000)
                        self._send_autonomy_cmd("abort")
                        self.set_mode("teleop")
                        self._joystick_activity_start = None
            else:
                self._joystick_activity_start = None
        else:
            self._joystick_activity_start = None

        # Button edge detection: trigger GUI actions on rising edge (0 -> 1)
        buttons = axes.get("buttons", [])
        if not isinstance(buttons, (list, tuple)) or len(buttons) == 0:
            return
        last = self._last_joystick_buttons
        for i in range(len(buttons)):
            pressed = buttons[i] if i < len(buttons) else 0
            prev = last[i] if i < len(last) else 0
            if not (pressed and not prev):
                continue
            # Debug mode: show button ID only, no action
            if self._joy_buttons_debug:
                action = JOY_BUTTON_MAP.get(i, "unmapped")
                self.statusBar().showMessage(f"Joystick button {i} pressed (action: {action})", 3000)
                break
            action = JOY_BUTTON_MAP.get(i)
            if action == "start":
                self._start_mission()
            elif action == "stop":
                self._stop_mission()
            elif action == "lap":
                self._on_lap_teleop()
            elif action == "report":
                self._generate_report()
            elif action == "yolo":
                self.yolo_checkbox.setChecked(not self.yolo_checkbox.isChecked())
            elif action == "qr":
                self.qr_checkbox.setChecked(not self.qr_checkbox.isChecked())
            elif action == "stowed":
                self.control_panel.stowed_checkbox.setChecked(
                    not self.control_panel.stowed_checkbox.isChecked())
            elif action == "back_support_up":
                self._back_support_raise()
            elif action == "back_support_down":
                self._back_support_lower()
            elif action == "mapping":
                self._send_autonomy_cmd("mapping")
            elif action == "sensor_cab":
                self._send_autonomy_cmd("sensor_cabinet")
        self._last_joystick_buttons = list(buttons)
    
    def closeEvent(self, event):
        """Handle window close."""
        # Stop 3D point cloud viewer
        if hasattr(self, 'pointcloud_view') and hasattr(self.pointcloud_view, 'stop'):
            self.pointcloud_view.stop()
        # Stop SLAM view
        if hasattr(self, 'slam_view') and hasattr(self.slam_view, 'stop'):
            self.slam_view.stop()
        # Stop camera receiver
        if self.camera_receiver:
            self.camera_receiver.stop()
            self.camera_receiver.disconnect()
        
        if self.zmq_context:
            for attr in ('camera_control_socket', 'arm_socket', 'autonomy_socket',
                         'autonomy_status_socket', 'snapshot_result_socket',
                         'slam_socket', 'status_socket'):
                sock = getattr(self, attr, None)
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass
            self.zmq_context.term()
        
        event.accept()


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Main entry point."""
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    # Set dark palette
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(26, 26, 46))
    palette.setColor(QPalette.WindowText, QColor(224, 224, 224))
    palette.setColor(QPalette.Base, QColor(15, 15, 26))
    palette.setColor(QPalette.AlternateBase, QColor(26, 26, 46))
    palette.setColor(QPalette.ToolTipBase, QColor(26, 26, 46))
    palette.setColor(QPalette.ToolTipText, QColor(224, 224, 224))
    palette.setColor(QPalette.Text, QColor(224, 224, 224))
    palette.setColor(QPalette.Button, QColor(15, 52, 96))
    palette.setColor(QPalette.ButtonText, QColor(224, 224, 224))
    palette.setColor(QPalette.BrightText, QColor(78, 204, 163))
    palette.setColor(QPalette.Highlight, QColor(78, 204, 163))
    palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)
    
    window = OperatorStation()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
