"""
RViz Stream View - TCP MJPEG receiver (no ZMQ).
Connects to Jetson GStreamer stream on port 5600.
Jetson: run stream_rviz.sh with GStreamer.
"""

import socket
import time

from PyQt5.QtWidgets import QFrame, QVBoxLayout, QLabel, QPushButton, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    from shared.constants import JETSON_IP, RVIZ_STREAM_TCP_PORT
except ImportError:
    JETSON_IP = "192.168.2.100"
    RVIZ_STREAM_TCP_PORT = 5600

# JPEG markers - format-agnostic, works with multipart MJPEG or raw stream
JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"


class RVizStreamView(QFrame):
    """Receives GStreamer MJPEG stream over TCP (port 5600). No ZMQ."""

    def __init__(self, parent=None, jetson_ip: str | None = None):
        super().__init__(parent)
        self.jetson_ip = jetson_ip or JETSON_IP
        self._port = RVIZ_STREAM_TCP_PORT
        self._sock = None
        self._buffer = b""
        self._poll_timer = None
        self._fps_times = []
        self.setup_ui()

    def setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        rviz_header = QHBoxLayout()
        rviz_title = QLabel("RViz SLAM")
        rviz_title.setStyleSheet("font-size: 14px; font-weight: bold; color: #4ecca3;")
        rviz_header.addWidget(rviz_title)
        rviz_header.addStretch()
        layout.addLayout(rviz_header)

        self.video_label = QLabel()
        self.video_label.setMinimumSize(320, 240)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setScaledContents(False)
        self.video_label.setStyleSheet(
            "background-color: #0a0a14; color: #606060; font-size: 12px; padding: 24px; border-radius: 6px;"
        )
        self.video_label.setText(
            f"Connect to {self.jetson_ip}:{self._port}\n\n"
            "Start stream_rviz.sh on Jetson first.\n"
            "Then click \u201cConnect\u201d below."
        )
        layout.addWidget(self.video_label, 1)

        ctrl_layout = QHBoxLayout()
        ctrl_layout.setSpacing(10)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setMinimumWidth(120)
        self.connect_btn.setMinimumHeight(34)
        self.connect_btn.setStyleSheet("font-size: 13px; font-weight: bold; padding: 8px 16px;")
        self.connect_btn.clicked.connect(self.toggle_stream)
        ctrl_layout.addWidget(self.connect_btn)
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #808080; font-size: 12px;")
        ctrl_layout.addWidget(self.status_label, 1)
        layout.addLayout(ctrl_layout)

    def toggle_stream(self) -> None:
        if self._sock is not None:
            self.stop_stream()
        else:
            self.start_stream()

    def start_stream(self) -> None:
        self.stop_stream()
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(5.0)
            self._sock.connect((self.jetson_ip, self._port))
            # TCP_NODELAY: disable Nagle for lower latency
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # Receive buffer: 128k for higher throughput at 20fps
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 128 * 1024)
            self._sock.settimeout(0.02)
            self._buffer = b""
            self.connect_btn.setText("Disconnect")
            self.status_label.setText(f"Connected to {self.jetson_ip}:{self._port}")
            self._poll_timer = QTimer(self)
            self._poll_timer.timeout.connect(self._poll)
            self._poll_timer.start(15)  # ~66 Hz poll to keep up with 20fps stream
        except Exception as e:
            self.status_label.setText(f"Connect failed: {str(e)[:50]}")
            self._close_socket()

    def _close_socket(self) -> None:
        if self._poll_timer:
            self._poll_timer.stop()
            self._poll_timer = None
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        self._buffer = b""

    def stop_stream(self) -> None:
        self._close_socket()
        self.connect_btn.setText("Connect")
        self.status_label.setText("Disconnected")

    def _poll(self) -> None:
        if self._sock is None:
            return
        try:
            # Drain socket completely to prevent kernel buffer buildup
            while True:
                data = self._sock.recv(65536)
                if not data:
                    self.status_label.setText("Connection closed")
                    self.stop_stream()
                    return
                self._buffer += data
                if len(data) < 65536:
                    break
            jpg = self._extract_jpeg()
            if jpg:
                self._on_frame(jpg)
        except socket.timeout:
            pass
        except OSError as e:
            self.status_label.setText(f"Error: {str(e)[:40]}")
            self.stop_stream()

    def _extract_jpeg(self) -> bytes | None:
        """Extract latest complete JPEG only. Aggressively discard older frames to minimize latency."""
        buf = self._buffer
        # Fast path: no SOI marker yet
        start = buf.find(JPEG_SOI)
        if start < 0:
            # Keep only tail (possible partial SOI marker)
            if len(buf) > 64 * 1024:
                self._buffer = b""
            return None
        # Jump to last SOI - skip all older frames entirely
        last_soi = buf.rfind(JPEG_SOI)
        if last_soi > start:
            buf = buf[last_soi:]
        elif start > 0:
            buf = buf[start:]
        # Find EOI after the SOI
        end = buf.find(JPEG_EOI, 2)
        if end < 0:
            # Incomplete frame - keep buffer, discard if too large
            self._buffer = buf if len(buf) < 64 * 1024 else buf[-32 * 1024:]
            return None
        # Got a complete frame
        jpeg = buf[:end + 2]
        self._buffer = buf[end + 2:]
        return jpeg

    def _on_frame(self, jpg_data: bytes) -> None:
        image = QImage()
        if not (image.loadFromData(jpg_data, "JPEG") and not image.isNull()):
            # Fallback: cv2 decode (slower due to color conversion)
            if CV2_AVAILABLE:
                arr = np.frombuffer(jpg_data, dtype=np.uint8)
                cv_img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if cv_img is None:
                    return
                h, w, ch = cv_img.shape
                image = QImage(cv_img.data, w, h, ch * w, QImage.Format_RGB888).copy().rgbSwapped()
            else:
                return
        if image.isNull():
            return
        pixmap = QPixmap.fromImage(image)
        if pixmap.isNull():
            return
        sz = self.video_label.size()
        if sz.width() > 0 and sz.height() > 0:
            scaled = pixmap.scaled(sz, Qt.KeepAspectRatio, Qt.FastTransformation)
        else:
            scaled = pixmap.scaled(640, 480, Qt.KeepAspectRatio, Qt.FastTransformation)
        self.video_label.setPixmap(scaled)
        self.video_label.setText("")
        now = time.time()
        self._fps_times.append(now)
        cutoff = now - 1.0
        while self._fps_times and self._fps_times[0] < cutoff:
            self._fps_times.pop(0)
        if len(self._fps_times) >= 2:
            self.status_label.setText(f"Connected | {len(self._fps_times)} FPS")

    def set_jetson_ip(self, ip: str) -> None:
        self.jetson_ip = ip
        if self._sock is not None:
            self.stop_stream()
            self.start_stream()

    def closeEvent(self, event) -> None:
        self.stop_stream()
        super().closeEvent(event)
