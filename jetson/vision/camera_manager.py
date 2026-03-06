#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Camera Manager
=========================================
3 Jetson cameras (no Pi): front, arm, backward.
2 with AI (front, arm): YOLO HAZMAT/Landolt inference.
1 streaming only (backward): 640x480 @ 30 FPS.
"""

import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum

SCRIPT_DIR = Path(__file__).resolve().parent
JETSON_DIR = SCRIPT_DIR.parent
REPO_ROOT = JETSON_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    from shared.constants import (
        CAMERAS, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
        ZMQ_PORT_CAMERA, VISION_CAMERA_CONFIG, JETSON_CAMERAS,
    )
except ImportError:
    CAMERAS = {"front": "/dev/video0", "arm": "/dev/video2", "backward": "/dev/video4"}
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    CAMERA_FPS = 30
    ZMQ_PORT_CAMERA = 5557
    VISION_CAMERA_CONFIG = {}
    JETSON_CAMERAS = ("front", "arm", "backward")


class CameraSource(str, Enum):
    """Camera hardware source."""
    JETSON = "jetson"
    PI = "pi"


@dataclass
class CameraConfig:
    """Per-camera configuration."""
    camera_id: str
    source: CameraSource
    device: Optional[str] = None  # /dev/videoX for Jetson
    device_index: Optional[int] = None  # For Pi OpenCV
    width: int = 640
    height: int = 480
    fps: int = 30
    ai_enabled: bool = True
    resolution: str = "640x480"

    def __post_init__(self):
        self.resolution = f"{self.width}x{self.height}"


# Default: from VISION_CAMERA_CONFIG, or fallback
def _default_jetson_cameras():
    jc = VISION_CAMERA_CONFIG.get("jetson_cameras", {})
    if jc:
        return list(jc.keys())
    try:
        return list(JETSON_CAMERAS)
    except NameError:
        return ["front", "arm", "backward"]

def _default_pi_cameras():
    return list(VISION_CAMERA_CONFIG.get("pi_cameras", {}).keys())

DEFAULT_JETSON_CAMERAS = _default_jetson_cameras()
DEFAULT_PI_CAMERAS = _default_pi_cameras()


class CameraManager:
    """
    Manages 2+2 camera system: assigns cameras to Jetson (AI) vs Pi (streaming).
    Provides unified camera ID list and config for vision pipeline.
    """

    def __init__(
        self,
        jetson_camera_ids: Optional[List[str]] = None,
        pi_camera_ids: Optional[List[str]] = None,
        jetson_devices: Optional[Dict[str, str]] = None,
        pi_ip: Optional[str] = None,
        pi_port: int = ZMQ_PORT_CAMERA,
    ):
        self.jetson_ids = jetson_camera_ids or DEFAULT_JETSON_CAMERAS
        self.pi_ids = pi_camera_ids or DEFAULT_PI_CAMERAS
        self.jetson_devices = jetson_devices or dict(CAMERAS)
        self.pi_ip = pi_ip  # Unused — no Pi cameras
        self.pi_port = pi_port

        self.configs: Dict[str, CameraConfig] = {}
        self._build_configs()

    def _build_configs(self) -> None:
        """Build per-camera config from Jetson + Pi assignments."""
        for cam_id in self.jetson_ids:
            device = self.jetson_devices.get(cam_id)
            self.configs[cam_id] = CameraConfig(
                camera_id=cam_id,
                source=CameraSource.JETSON,
                device=device,
                width=CAMERA_WIDTH,
                height=CAMERA_HEIGHT,
                fps=CAMERA_FPS,
                ai_enabled=True,
            )
        for cam_id in self.pi_ids:
            self.configs[cam_id] = CameraConfig(
                camera_id=cam_id,
                source=CameraSource.PI,
                device=None,
                device_index=self.pi_ids.index(cam_id) if cam_id in self.pi_ids else 0,
                width=CAMERA_WIDTH,
                height=CAMERA_HEIGHT,
                fps=15,
                ai_enabled=False,
            )

    def get_all_camera_ids(self) -> List[str]:
        """Return ordered list of all camera IDs (Jetson first, then Pi)."""
        return list(self.jetson_ids) + list(self.pi_ids)

    def get_ai_camera_ids(self) -> List[str]:
        """Return camera IDs that run AI inference (YOLO, Landolt, QR)."""
        return [cid for cid, cfg in self.configs.items() if cfg.ai_enabled]

    def get_jetson_camera_ids(self) -> List[str]:
        """Return Jetson-attached camera IDs."""
        return list(self.jetson_ids)

    def get_pi_camera_ids(self) -> List[str]:
        """Return Pi-attached camera IDs."""
        return list(self.pi_ids)

    def get_config(self, camera_id: str) -> Optional[CameraConfig]:
        """Get config for a camera."""
        return self.configs.get(camera_id)

    def get_jetson_devices(self) -> Dict[str, str]:
        """Return Jetson camera_id -> /dev/videoX mapping."""
        return {
            cid: cfg.device
            for cid, cfg in self.configs.items()
            if cfg.source == CameraSource.JETSON and cfg.device
        }

    def get_pi_connect_str(self) -> str:
        """Return ZMQ connect string for Pi camera stream."""
        return f"tcp://{self.pi_ip}:{self.pi_port}"

    def is_ai_camera(self, camera_id: str) -> bool:
        """Check if camera runs AI inference."""
        cfg = self.configs.get(camera_id)
        return cfg.ai_enabled if cfg else False
