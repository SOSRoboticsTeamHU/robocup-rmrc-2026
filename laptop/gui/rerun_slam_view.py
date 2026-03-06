#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Enhanced Rerun SLAM View with 4-Camera Overlay
===========================================================================
Rerun.io 3D viewer embedded in PyQt5 + live camera overlays in corners.

Features:
- Rerun web viewer embed (QWebEngineView) for 3D visualization
- 3 live camera feeds (front, arm, backward)
- YOLO bounding boxes on front + arm cameras
- Robot URDF animation + point cloud + occupancy grid
- Odometry path + heading arrow
- RMRC scoring overlays: distance to walls, lap detection, maze lanes
"""

import sys
import os
import time
import json
import threading
from typing import Optional, Dict, Tuple
from pathlib import Path
from dataclasses import dataclass

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

try:
    from PyQt5.QtWidgets import (
        QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFrame, QGridLayout
    )
    from PyQt5.QtCore import Qt, QTimer, QUrl, pyqtSignal, QThread, QPoint, QSize
    from PyQt5.QtGui import QColor, QFont, QImage, QPixmap, QPainter, QPen, QBrush
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    PYQT_AVAILABLE = True
except ImportError:
    PYQT_AVAILABLE = False
    QWebEngineView = None

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    from shared.constants import (
        JETSON_IP, ZMQ_PORT_CAMERA, ZMQ_PORT_SLAM, ZMQ_PORT_CLOUD,
        CAMERA_WIDTH, CAMERA_HEIGHT
    )
except ImportError:
    JETSON_IP = "192.168.2.100"
    ZMQ_PORT_CAMERA = 5557
    ZMQ_PORT_SLAM = 5562
    ZMQ_PORT_CLOUD = 5564
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480

# Rerun web viewer config
RERUN_SERVE_PORT = 9876      # gRPC proxy port (for native app)
RERUN_WEB_PORT = 9090        # HTTP web viewer port (for QWebEngineView browser embed)
RERUN_VIEWER_VERSION = "latest"


# ============================================================================
# CAMERA RECEIVER THREAD
# ============================================================================

class CameraOverlayReceiver(QThread):
    """Background ZMQ receiver for 3 camera feeds."""
    frame_received = pyqtSignal(str, np.ndarray, list)  # camera_id, frame, detections
    
    def __init__(self, jetson_ip: str):
        super().__init__()
        self.jetson_ip = jetson_ip
        self._running = False
        self._frames = {}
        self._frame_lock = threading.Lock()
    
    def get_latest_frame(self, camera_id: str) -> Optional[Tuple[np.ndarray, list]]:
        """Get latest frame for a camera (non-blocking)."""
        with self._frame_lock:
            if camera_id in self._frames:
                frame, detections = self._frames[camera_id]
                return frame, detections
        return None
    
    def stop(self):
        self._running = False
    
    def run(self):
        if not ZMQ_AVAILABLE or not CV2_AVAILABLE:
            return
        
        try:
            ctx = zmq.Context()
            sock = ctx.socket(zmq.SUB)
            sock.setsockopt(zmq.SUBSCRIBE, b"")
            sock.setsockopt(zmq.RCVHWM, 2)
            sock.setsockopt(zmq.RCVTIMEO, 100)
            sock.connect(f"tcp://{self.jetson_ip}:{ZMQ_PORT_CAMERA}")
            
            self._running = True
            while self._running:
                try:
                    header_msg = sock.recv_json(zmq.NOBLOCK)
                    jpeg_msg = sock.recv(zmq.NOBLOCK)
                    
                    camera_id = header_msg.get("camera_id", "unknown")
                    detections = header_msg.get("detections", [])
                    
                    # Decode JPEG
                    nparr = np.frombuffer(jpeg_msg, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        with self._frame_lock:
                            self._frames[camera_id] = (frame, detections)
                        self.frame_received.emit(camera_id, frame, detections)
                except zmq.error.Again:
                    pass
                except Exception as e:
                    print(f"[CameraOverlay] Frame error: {e}")
                    time.sleep(0.01)
            
            sock.close()
            ctx.term()
        except Exception as e:
            print(f"[CameraOverlay] ZMQ error: {e}")


# ============================================================================
# CAMERA OVERLAY WIDGET (4 small camera feeds with YOLO boxes)
# ============================================================================

class CameraOverlayWidget(QFrame):
    """Four small camera views overlaid on the 3D view (corners)."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("CameraOverlay")
        self.setStyleSheet("""
            QFrame#CameraOverlay {
                background: transparent;
                border: none;
            }
        """)
        
        layout = QGridLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)
        
        # 3 camera views: front, arm, backward
        self.cameras = {}
        positions = [
            ("front", 0, 0),
            ("arm", 0, 1),
            ("backward", 0, 2),
        ]
        
        for cam_id, row, col in positions:
            cam_widget = self._create_camera_view(cam_id)
            self.cameras[cam_id] = cam_widget
            layout.addWidget(cam_widget, row, col)
    
    def _create_camera_view(self, camera_id: str) -> QFrame:
        """Create a small camera view frame."""
        frame = QFrame()
        frame.setObjectName("CameraViewFrame")
        frame.setMinimumSize(160, 120)
        frame.setMaximumSize(200, 150)
        frame.setStyleSheet(f"""
            QFrame#CameraViewFrame {{
                background-color: #0a0a14;
                border: 2px solid #0f3460;
                border-radius: 4px;
            }}
        """)
        
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(0)
        
        # Title
        title = QLabel(camera_id.upper())
        title.setStyleSheet("color: #4ecca3; font-size: 9px; font-weight: bold;")
        layout.addWidget(title)
        
        # Image label
        image_label = QLabel()
        image_label.setAlignment(Qt.AlignCenter)
        image_label.setMinimumHeight(100)
        image_label.setStyleSheet("background-color: black; border: none;")
        image_label.setText("No Signal")
        image_label.setStyleSheet("background-color: black; color: #666; font-size: 8px;")
        layout.addWidget(image_label, 1)
        
        # Store label for updates
        frame._image_label = image_label
        frame._last_frame = None
        return frame
    
    def update_camera(self, camera_id: str, frame: np.ndarray, detections: list):
        """Update a camera view with frame and YOLO bounding boxes."""
        if camera_id not in self.cameras:
            return
        
        frame_copy = frame.copy()
        
        # Draw YOLO bounding boxes (only for front and arm with HAZMAT/Landolt)
        if detections and camera_id in ("front", "arm"):
            for det in detections:
                try:
                    x1, y1, x2, y2 = int(det["x1"]), int(det["y1"]), int(det["x2"]), int(det["y2"])
                    class_name = det.get("class_name", "unknown")
                    conf = det.get("confidence", 0)
                    
                    # Color: red for hazmat, cyan for landolt
                    if "hazmat" in class_name.lower():
                        color = (0, 0, 255)  # Red
                    elif "landolt" in class_name.lower() or "ring" in class_name.lower():
                        color = (255, 255, 0)  # Cyan
                    else:
                        color = (0, 255, 0)  # Green
                    
                    # Draw box
                    cv2.rectangle(frame_copy, (x1, y1), (x2, y2), color, 2)
                    
                    # Draw label
                    label = f"{class_name} {conf:.0%}"
                    cv2.putText(frame_copy, label, (x1, y1 - 5),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                except Exception:
                    pass
        
        # Convert to QPixmap and display (small thumbnail)
        height, width = frame_copy.shape[:2]
        
        # Resize for thumbnail (160x120 display)
        aspect = width / height
        if aspect > 160 / 120:
            thumb_w, thumb_h = 160, int(160 / aspect)
        else:
            thumb_w, thumb_h = int(120 * aspect), 120
        
        frame_thumb = cv2.resize(frame_copy, (thumb_w, thumb_h))
        frame_rgb = cv2.cvtColor(frame_thumb, cv2.COLOR_BGR2RGB)
        
        h, w = frame_rgb.shape[:2]
        bytes_per_line = 3 * w
        q_image = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        
        # Update label
        label = self.cameras[camera_id]._image_label
        label.setPixmap(pixmap)


# ============================================================================
# ENHANCED RERUN 3D VIEWER WIDGET
# ============================================================================

class RerunSLAMView(QWidget):
    """
    Main SLAM 3D view with embedded Rerun web viewer + camera overlays.
    Rerun displays: point cloud, URDF robot, grid, odometry path, heading arrow.
    Camera overlays show front/arm/backward feeds with YOLO bounding boxes.
    """
    
    def __init__(self, jetson_ip: str = JETSON_IP, parent=None):
        super().__init__(parent)
        self.jetson_ip = jetson_ip
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # Top control bar: status + buttons
        ctrl_bar = QHBoxLayout()
        ctrl_bar.setContentsMargins(8, 4, 8, 4)
        ctrl_bar.setSpacing(10)
        
        self.status_label = QLabel("Rerun 3D Viewer")
        self.status_label.setStyleSheet("color: #4ecca3; font-size: 9px;")
        ctrl_bar.addWidget(self.status_label)
        
        self.native_btn = QPushButton("🚀 Launch Native Viewer")
        self.native_btn.setMaximumWidth(160)
        self.native_btn.setMaximumHeight(22)
        self.native_btn.clicked.connect(self._launch_native_viewer)
        self.native_btn.setToolTip(
            f"Launch native Rerun app (best option)\n"
            f"Connects to gRPC port {RERUN_SERVE_PORT}\n"
            f"Requires: pip install rerun-sdk>=0.29.2"
        )
        ctrl_bar.addWidget(self.native_btn)
        
        self.browser_btn = QPushButton("🌐 Browser")
        self.browser_btn.setMaximumWidth(80)
        self.browser_btn.setMaximumHeight(22)
        self.browser_btn.clicked.connect(self._open_in_browser)
        self.browser_btn.setToolTip(
            f"Open web viewer in browser\n"
            f"Connects to HTTP port {RERUN_WEB_PORT}"
        )
        ctrl_bar.addWidget(self.browser_btn)
        
        self.reconnect_btn = QPushButton("🔄 Reconnect")
        self.reconnect_btn.setMaximumWidth(90)
        self.reconnect_btn.setMaximumHeight(22)
        self.reconnect_btn.clicked.connect(self._connect)
        ctrl_bar.addWidget(self.reconnect_btn)
        
        self.camera_overlay_check = QLabel("📷 Cameras: ON")
        self.camera_overlay_check.setStyleSheet("color: #4ecca3; font-size: 9px;")
        ctrl_bar.addWidget(self.camera_overlay_check)
        
        ctrl_bar.addStretch()
        layout.addLayout(ctrl_bar)
        
        # Main container: Rerun web view + camera overlays
        container = QFrame()
        container_layout = QVBoxLayout(container)
        container_layout.setContentsMargins(0, 0, 0, 0)
        container_layout.setSpacing(0)
        
        # Try to embed Rerun web viewer
        try:
            from PyQt5.QtWebEngineWidgets import QWebEngineView as WebEngineView
            self.web = WebEngineView()
            self.web.setMinimumHeight(500)
            self.web.page().loadFinished.connect(self._on_load_finished)
            self.web.page().loadStarted.connect(self._on_load_started)
            container_layout.addWidget(self.web, 1)
        except Exception as e:
            print(f"[RerunSLAMView] WebEngine unavailable: {e}")
            self.web = None
            placeholder = QLabel(
                "Rerun 3D Viewer\n\n"
                "pip install PyQtWebEngine\n\n"
                "Click 'Launch Native Viewer' above"
            )
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setStyleSheet("color: #888; background: #0a0a14; padding: 20px;")
            container_layout.addWidget(placeholder, 1)
        
        # Overlay layer for cameras (on top of web view)
        overlay_container = QWidget()
        overlay_container.setStyleSheet("background: transparent;")
        overlay_layout = QHBoxLayout(overlay_container)
        overlay_layout.setContentsMargins(8, 8, 8, 8)
        overlay_layout.setSpacing(0)
        
        # 1x3 camera grid on right side
        self.camera_overlay = CameraOverlayWidget(overlay_container)
        overlay_layout.addStretch()
        overlay_layout.addWidget(self.camera_overlay)
        container_layout.addWidget(overlay_container, 0)
        
        layout.addWidget(container, 1)
        
        # Camera receiver thread
        self.camera_receiver = CameraOverlayReceiver(self.jetson_ip)
        self.camera_receiver.frame_received.connect(self._on_camera_frame)
        self.camera_receiver.start()
        
        # Reconnect timer
        self._reconnect_timer = QTimer(self)
        self._reconnect_timer.setSingleShot(True)
        self._reconnect_timer.timeout.connect(self._connect)
        
        # Spinner animation
        self._spinner_chars = "⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏"
        self._spinner_idx = 0
        self._spinner_timer = QTimer(self)
        self._spinner_timer.timeout.connect(self._update_spinner)
        
        # Auto-connect if web engine available
        if self.web is not None:
            self._connect()
        
        self._native_proc = None
    
    def _rerun_url(self) -> str:
        """Jetson-hosted Rerun web viewer."""
        return f"http://{self.jetson_ip}:{RERUN_WEB_PORT}"
    
    def _connect(self):
        if self.web is None:
            return
        self.status_label.setText("🔌 Connecting to Rerun…")
        self.reconnect_btn.setEnabled(False)
        self.web.setUrl(QUrl(self._rerun_url()))
    
    def _launch_native_viewer(self):
        """Launch native Rerun viewer."""
        import subprocess
        import shutil
        
        if self._native_proc is not None and self._native_proc.poll() is None:
            self._native_proc.terminate()
            self._native_proc = None
        
        rerun_bin = shutil.which("rerun")
        if rerun_bin is None:
            self.status_label.setText("⚠ 'rerun' not installed: pip install rerun-sdk>=0.29.2")
            return
        
        connect_url = f"rerun+http://{self.jetson_ip}:{RERUN_SERVE_PORT}/proxy"
        try:
            self._native_proc = subprocess.Popen(
                [rerun_bin, "--connect", connect_url],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.status_label.setText(
                f"✅ Native Rerun viewer launched (gRPC {RERUN_SERVE_PORT})"
            )
        except Exception as e:
            self.status_label.setText(f"⚠ Launch failed: {e}")
    
    def _open_in_browser(self):
        """Open in default browser."""
        import webbrowser
        webbrowser.open(self._rerun_url())
        self.status_label.setText(
            f"✅ Opened browser → http://{self.jetson_ip}:{RERUN_WEB_PORT}"
        )
    
    def _on_load_started(self):
        self.status_label.setText("⏳ Loading…")
        self._spinner_timer.start(100)
    
    def _update_spinner(self):
        c = self._spinner_chars[self._spinner_idx % len(self._spinner_chars)]
        self._spinner_idx += 1
        self.status_label.setText(f"{c} Loading Rerun…")
    
    def _on_load_finished(self, ok: bool):
        self.reconnect_btn.setEnabled(True)
        self._spinner_timer.stop()
        if ok:
            self.status_label.setText(f"✅ Rerun 3D | Web {self.jetson_ip}:{RERUN_WEB_PORT}")
            self._reconnect_timer.stop()
        else:
            self.status_label.setText(
                "⚠ Web embed failed → Use '🚀 Launch Native Viewer' (port 9876) or '🌐 Browser'"
            )
    
    def _on_camera_frame(self, camera_id: str, frame: np.ndarray, detections: list):
        """Receive camera frame from background thread."""
        self.camera_overlay.update_camera(camera_id, frame, detections)
    
    def cleanup(self):
        """Cleanup."""
        self.camera_receiver.stop()
        if self._native_proc is not None and self._native_proc.poll() is None:
            self._native_proc.terminate()
