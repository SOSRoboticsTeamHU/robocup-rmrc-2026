#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Snapshot Mode (HAZMAT / Landolt)
===========================================================
Rulebook: "provide a single still image with indication of where the hazmat label is and the identity."
Captures frame → YOLO HAZMAT detect → annotate still (bbox + label) → send to GUI as snapshot_result.
Landolt: still image with direction text + gap marker for Sensor Cabinet / Linear Rail Inspect.
"""

import sys
import os
import time
from typing import Optional, List, Tuple, Dict, Any

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
JETSON_DIR = os.path.dirname(SCRIPT_DIR)
if JETSON_DIR not in sys.path:
    sys.path.insert(0, JETSON_DIR)

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


def capture_hazmat_still(
    frame,
    detections: List[Dict],
    label_key: str = "class_name",
    bbox_key: str = "bbox",
) -> Tuple[Optional[bytes], str]:
    """
    Annotate frame with HAZMAT bbox + label; return JPEG bytes and identity string.
    detections: list of {class_name, confidence, bbox: [x1,y1,x2,y2]}.
    """
    if cv2 is None or frame is None:
        return None, ""
    img = frame.copy() if len(frame.shape) >= 2 else frame
    identity = ""
    for det in detections or []:
        cls = det.get(label_key, det.get("class", ""))
        conf = det.get("confidence", 0)
        bbox = det.get(bbox_key, [])
        if len(bbox) >= 4:
            x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        if cls:
            identity = cls if not identity else identity + "; " + cls
            cv2.putText(
                img, f"{cls} {conf:.2f}", (bbox[0] if bbox else 10, (bbox[1] if bbox else 30) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )
    _, jpeg = cv2.imencode(".jpg", img)
    return jpeg.tobytes() if jpeg is not None else None, identity


def capture_landolt_still(
    frame,
    direction: str,
    gap_xy: Optional[Tuple[int, int]] = None,
) -> Optional[bytes]:
    """Annotate still with Landolt direction text and optional gap marker."""
    if cv2 is None or frame is None:
        return None
    img = frame.copy() if len(frame.shape) >= 2 else frame
    cv2.putText(img, f"Direction: {direction}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    if gap_xy:
        cx, cy = gap_xy
        cv2.circle(img, (cx, cy), 8, (255, 255, 0), 2)
    _, jpeg = cv2.imencode(".jpg", img)
    return jpeg.tobytes() if jpeg is not None else None


def build_snapshot_result(
    image_bytes: Optional[bytes],
    result_type: str,
    identity: str = "",
    direction: str = "",
    camera_id: str = "arm",
) -> Dict[str, Any]:
    """Build snapshot_result message for GUI (ZMQ or in-process)."""
    return {
        "msg_type": "snapshot_result",
        "timestamp": time.time(),
        "image_bytes": image_bytes,
        "result_type": result_type,  # "hazmat" | "landolt"
        "identity": identity,
        "direction": direction,
        "camera_id": camera_id,
    }
