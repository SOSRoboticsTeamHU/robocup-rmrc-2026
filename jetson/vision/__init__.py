"""
RoboCupRescue RMRC 2026 - Vision System
=======================================
Optimized for Jetson Orin Nano Super with 2 AI cameras + 2 Pi cameras.

Modules:
- landolt_classifier: 8-direction Landolt-C gap orientation classification
- yolo11_processor: YOLO11 multi-model inference (HAZMAT, Landolt, General)
- camera_manager: 2+2 camera coordination (Jetson AI + Pi streams)
"""

from .landolt_classifier import (
    LandoltClassifier,
    LANDOLT_DIRECTIONS,
    LANDOLT_DIRECTIONS_SHORT,
    LandoltReading,
)
from .yolo11_processor import YOLO11Processor, YOLO11Config
from .camera_manager import CameraManager, CameraConfig, CameraSource

__all__ = [
    "LandoltClassifier",
    "LandoltReading",
    "LANDOLT_DIRECTIONS",
    "LANDOLT_DIRECTIONS_SHORT",
    "YOLO11Processor",
    "YOLO11Config",
    "CameraManager",
    "CameraConfig",
    "CameraSource",
]
