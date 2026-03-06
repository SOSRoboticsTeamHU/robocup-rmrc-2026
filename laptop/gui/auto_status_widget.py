#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2025 - Auto Mission Status Widget (PyQt5)
=============================================================
Simple labels and buttons for autonomy status. Add to main GUI.
Labels: Autonomy ACTIVE/IDLE | Lap: N | 30cm zone: GREEN/RED | Test: <name>
Buttons: Start Auto Lap, Stowed Posture, Select Test (combo).
Parent supplies status updates via set_status() and connects signals to send ZMQ commands.
"""

import os
import sys
from typing import Optional, Dict, Any

# Add shared for TEST_GROUPS
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "shared"))

try:
    from PyQt5.QtWidgets import (
        QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
        QGroupBox, QFrame,
    )
    from PyQt5.QtCore import Qt, pyqtSignal
    from PyQt5.QtGui import QFont
    PYQT_AVAILABLE = True
except ImportError:
    PYQT_AVAILABLE = False

try:
    from shared.constants import TEST_GROUPS
except ImportError:
    TEST_GROUPS = [
        ("Mobility", [
            ("K-Rails Horizontal", "krails_horiz"),
            ("K-Rails Crossover", "krails_crossover"),
            ("Incline Horizontal", "incline_horiz"),
            ("Sand & Gravel", "sand_gravel"),
            ("Stairs", "stairs"),
        ]),
        ("Other", [("Align", "align")]),
    ]


class AutoStatusWidget(QWidget if PYQT_AVAILABLE else object):
    """
    Widget for RMRC 2025 auto mission: status labels + Start Auto Lap, Stowed, Select Test.
    Parent must call set_status(msg) when ZMQ autonomy status is received,
    and connect signals to send commands (e.g. autonomy_socket.send_json).
    """
    start_auto_lap_clicked = pyqtSignal() if PYQT_AVAILABLE else None
    stowed_posture_clicked = pyqtSignal() if PYQT_AVAILABLE else None
    test_selected = pyqtSignal(str) if PYQT_AVAILABLE else None
    abort_clicked = pyqtSignal() if PYQT_AVAILABLE else None

    def __init__(self, parent=None):
        if not PYQT_AVAILABLE:
            raise RuntimeError("PyQt5 required for AutoStatusWidget")
        super().__init__(parent)
        self._status: Dict[str, Any] = {}
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)

        # Status line: Autonomy: ACTIVE | Lap: 3 | 30cm zone: GREEN | Test: K-Rails
        self.status_frame = QFrame()
        self.status_frame.setStyleSheet("QFrame { background-color: #1e293b; border-radius: 6px; padding: 6px; }")
        status_layout = QHBoxLayout(self.status_frame)
        self.autonomy_label = QLabel("Autonomy: —")
        self.autonomy_label.setStyleSheet("color: #94a3b8; font-weight: bold;")
        status_layout.addWidget(self.autonomy_label)
        status_layout.addWidget(QLabel("|"))
        self.lap_label = QLabel("Lap: 0")
        self.lap_label.setStyleSheet("color: #94a3b8;")
        status_layout.addWidget(self.lap_label)
        status_layout.addWidget(QLabel("|"))
        self.zone_label = QLabel("30cm zone: —")
        self.zone_label.setStyleSheet("color: #94a3b8;")
        status_layout.addWidget(self.zone_label)
        status_layout.addWidget(QLabel("|"))
        self.test_label = QLabel("Test: —")
        self.test_label.setStyleSheet("color: #94a3b8;")
        status_layout.addWidget(self.test_label)
        status_layout.addStretch()
        layout.addWidget(self.status_frame)

        # Buttons row
        btn_layout = QHBoxLayout()
        self.start_lap_btn = QPushButton("Start Auto Lap")
        self.start_lap_btn.setMinimumHeight(44)
        self.start_lap_btn.setStyleSheet("background-color: #22c55e; color: black; font-weight: bold;")
        self.start_lap_btn.clicked.connect(self._on_start_auto_lap)
        btn_layout.addWidget(self.start_lap_btn)

        self.stowed_btn = QPushButton("Stowed Posture")
        self.stowed_btn.setMinimumHeight(44)
        self.stowed_btn.setToolTip("Retract arms/flippers before single-command autonomy (4x multiplier)")
        self.stowed_btn.clicked.connect(self._on_stowed)
        btn_layout.addWidget(self.stowed_btn)

        btn_layout.addWidget(QLabel("Select Test:"))
        self.test_combo = QComboBox()
        self.test_combo.setMinimumWidth(180)
        for group_name, items in TEST_GROUPS:
            for display_name, value in items:
                self.test_combo.addItem(display_name, value)
        self.test_combo.currentIndexChanged.connect(self._on_test_selected)
        btn_layout.addWidget(self.test_combo)

        self.abort_btn = QPushButton("⬛ ABORT")
        self.abort_btn.setObjectName("StopButton")
        self.abort_btn.setMinimumHeight(44)
        self.abort_btn.setStyleSheet("background-color: #dc2626; color: white; font-weight: bold;")
        self.abort_btn.clicked.connect(self.abort_clicked.emit)
        btn_layout.addWidget(self.abort_btn)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

    def _on_start_auto_lap(self):
        self.start_auto_lap_clicked.emit()

    def _on_stowed(self):
        self.stowed_posture_clicked.emit()

    def _on_test_selected(self):
        idx = self.test_combo.currentIndex()
        if idx >= 0:
            value = self.test_combo.currentData()
            if value:
                self.test_selected.emit(str(value))

    def set_status(self, msg: Dict[str, Any]):
        """Update labels from autonomy status message (from ZMQ 5565)."""
        self._status = msg
        state = msg.get("state", "idle")
        lap_count = msg.get("lap_count", msg.get("laps_completed", 0))
        in_zone = msg.get("in_end_zone_30cm", False)
        test_type = msg.get("test_type", "—")
        autonomy_active = msg.get("autonomy_active", False)

        if autonomy_active or state == "autonomous_lap":
            self.autonomy_label.setText("Autonomy: ACTIVE")
            self.autonomy_label.setStyleSheet("color: #22c55e; font-weight: bold;")
        else:
            self.autonomy_label.setText("Autonomy: IDLE")
            self.autonomy_label.setStyleSheet("color: #94a3b8; font-weight: bold;")

        self.lap_label.setText(f"Lap: {lap_count}")

        if in_zone:
            self.zone_label.setText("30cm zone: GREEN (hands-on allowed)")
            self.zone_label.setStyleSheet("color: #22c55e;")
        else:
            self.zone_label.setText("30cm zone: RED (hands off)")
            self.zone_label.setStyleSheet("color: #f59e0b;")

        self.test_label.setText(f"Test: {test_type}")

    def get_selected_test(self) -> str:
        """Return current test type value from combo."""
        v = self.test_combo.currentData()
        return str(v) if v else "krails_horiz"


def create_and_connect(
    parent_qwidget,
    jetson_ip: str,
    autonomy_socket_send=None,
) -> Optional["AutoStatusWidget"]:
    """
    Create AutoStatusWidget and connect to ZMQ if autonomy_socket_send is provided.
    autonomy_socket_send: callable(msg_dict) to send JSON to Jetson (e.g. lambda msg: autonomy_socket.send_json(msg)).
    Returns widget; parent must call widget.set_status(msg) when status is received.
    """
    if not PYQT_AVAILABLE:
        return None
    widget = AutoStatusWidget(parent_qwidget)
    if autonomy_socket_send is not None:
        def send(cmd_dict):
            try:
                autonomy_socket_send(cmd_dict)
            except Exception:
                pass
        def on_start():
            send({"cmd": "start_auto_lap"})
        def on_stowed():
            send({"cmd": "stowed_posture"})
        def on_test(t):
            send({"cmd": "select_test", "test_type": t})
        def on_abort():
            send({"cmd": "abort"})
        widget.start_auto_lap_clicked.connect(on_start)
        widget.stowed_posture_clicked.connect(on_stowed)
        widget.test_selected.connect(on_test)
        widget.abort_clicked.connect(on_abort)
    return widget
