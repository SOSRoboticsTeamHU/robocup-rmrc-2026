#!/usr/bin/env python3
"""
Standalone RViz Stream Viewer - runs in separate process to avoid ZMQ context
conflicts with main GUI. Receives screenshot stream from Jetson via ZMQ.

Usage: python rviz_stream_standalone.py [--jetson-ip IP] [--port PORT]
"""

import argparse
import sys
import os

# Ensure shared module path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

try:
    from shared.constants import JETSON_IP, ZMQ_PORT_RVIZ_STREAM
except ImportError:
    JETSON_IP = "192.168.2.100"
    ZMQ_PORT_RVIZ_STREAM = 5572

try:
    import zmq
except ImportError:
    print("ERROR: pip install pyzmq")
    sys.exit(1)

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
import time


class StandaloneRVizViewer(QMainWindow):
    """Minimal window that receives RViz stream via ZMQ. Own process, own context."""

    def __init__(self, jetson_ip: str, port: int):
        super().__init__()
        self.jetson_ip = jetson_ip
        self.port = port
        self._zmq_ctx = zmq.Context()
        self._zmq_socket = None
        self._fps_times = []
        self.setWindowTitle(f"RViz Stream — {jetson_ip}:{port}")
        self.setMinimumSize(640, 480)
        self.resize(1280, 720)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: #0a0a14; color: #888;")
        self.video_label.setText(f"Connecting to {jetson_ip}:{port}...")
        layout.addWidget(self.video_label, 1)

        self._connect_socket()
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll)
        self._poll_timer.start(50)

    def _connect_socket(self):
        try:
            self._zmq_socket = self._zmq_ctx.socket(zmq.SUB)
            self._zmq_socket.setsockopt(zmq.SUBSCRIBE, b"")
            self._zmq_socket.setsockopt(zmq.RCVTIMEO, 100)
            self._zmq_socket.setsockopt(zmq.CONFLATE, 1)
            self._zmq_socket.setsockopt(zmq.LINGER, 0)
            self._zmq_socket.setsockopt(zmq.CONNECT_TIMEOUT, 5000)
            self._zmq_socket.connect(f"tcp://{self.jetson_ip}:{self.port}")
        except Exception as e:
            self.video_label.setText(f"Connect failed: {e}")

    def _poll(self):
        if self._zmq_socket is None:
            return
        try:
            parts = self._zmq_socket.recv_multipart(zmq.NOBLOCK)
            if len(parts) >= 2:
                jpg_data = parts[1]
                image = QImage()
                if image.loadFromData(jpg_data, "JPEG"):
                    pixmap = QPixmap.fromImage(image)
                    scaled = pixmap.scaled(
                        self.video_label.size(),
                        Qt.KeepAspectRatio,
                        Qt.SmoothTransformation,
                    )
                    self.video_label.setPixmap(scaled)
                now = time.time()
                self._fps_times.append(now)
                cutoff = now - 1.0
                while self._fps_times and self._fps_times[0] < cutoff:
                    self._fps_times.pop(0)
                fps = len(self._fps_times)
                self.setWindowTitle(f"RViz Stream — {self.jetson_ip}:{self.port} | {fps} FPS")
        except zmq.Again:
            pass
        except zmq.ZMQError:
            self._poll_timer.stop()
            self.video_label.setText("Connection lost")

    def closeEvent(self, event):
        self._poll_timer.stop()
        if self._zmq_socket:
            self._zmq_socket.close()
        self._zmq_ctx.term()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--jetson-ip", default=JETSON_IP)
    parser.add_argument("--port", type=int, default=ZMQ_PORT_RVIZ_STREAM)
    args = parser.parse_args()

    app = QApplication(sys.argv)
    win = StandaloneRVizViewer(args.jetson_ip, args.port)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
