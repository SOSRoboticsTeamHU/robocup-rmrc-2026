#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - YOLO11 Processor
===========================================
Memory-efficient YOLO11 integration for Jetson Orin Nano Super.

Features:
- YOLO11n (nano) model for speed
- Multi-model support: HAZMAT, Landolt, General
- TensorRT (.engine) for Jetson
- FP16 precision for memory/speed balance
- Task-based runtime switching
"""

import os
import sys
from pathlib import Path
from typing import List, Dict, Optional, Any
from dataclasses import dataclass, field

SCRIPT_DIR = Path(__file__).resolve().parent
JETSON_DIR = SCRIPT_DIR.parent
REPO_ROOT = JETSON_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    YOLO = None


@dataclass
class YOLO11Config:
    """Jetson Orin Nano Super optimized YOLO configuration."""
    model_size: str = "n"
    precision: str = "FP16"
    batch_size: int = 1
    max_detections: int = 20
    confidence_threshold: float = 0.5
    iou_threshold: float = 0.45
    imgsz: int = 640


# Model discovery patterns: prefer YOLO11/TensorRT, fallback to YOLOv8
MODEL_PATTERNS = {
    "general": ["yolo11n.engine", "yolov11n.engine", "yolo11n.pt", "yolov11n.pt", "yolov8n.engine", "yolov8n.pt"],
    "hazmat": ["hazmat_yolo11n.engine", "hazmat_yolo11n.pt", "hazmat_yolov8n.engine", "hazmat_best.engine", "hazmat_yolov8n.pt"],
    "landolt": ["landolt_yolo11n.engine", "landolt_yolo11n.pt", "landolt_yolov8n.engine", "landolt_best.engine", "landolt_yolov8n.pt"],
}


def _run_model(model, frame, conf, max_det, imgsz, iou, source_tag: str, exclude_classes: set = None) -> list:
    """Run a YOLO model and return detections with source tag."""
    if model is None or frame is None:
        return []
    try:
        results = model(frame, conf=conf, iou=iou, verbose=False, max_det=max_det, imgsz=imgsz)
        out = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0])
                class_name = r.names.get(cls_id, f"class_{cls_id}")
                if exclude_classes and class_name.lower() in exclude_classes:
                    continue
                out.append({
                    "class_id": cls_id,
                    "class_name": class_name,
                    "confidence": round(float(box.conf[0]), 3),
                    "bbox": list(map(int, box.xyxy[0].tolist())),
                    "center": [
                        (int(box.xyxy[0][0]) + int(box.xyxy[0][2])) // 2,
                        (int(box.xyxy[0][1]) + int(box.xyxy[0][3])) // 2,
                    ],
                    "source": source_tag,
                })
        return out
    except Exception:
        return []


class YOLO11Processor:
    """
    YOLO11 multi-model processor for RoboCup Rescue tasks.
    Supports combined mode: runs HAZMAT + Landolt on every frame (no task switching).
    """

    def __init__(
        self,
        models_dir: Optional[Path] = None,
        config: Optional[YOLO11Config] = None,
        default_task: str = "general",
    ):
        self.config = config or YOLO11Config()
        self.default_task = default_task
        self.models_dir = Path(models_dir) if models_dir else JETSON_DIR / "models" / "engines"
        self.models: Dict[str, Optional[str]] = {"general": None, "hazmat": None, "landolt": None}
        self.current_task = None
        self.current_model = None
        self._hazmat_model = None
        self._landolt_model = None
        self.ready = False
        self.combined_ready = False
        self._discover_models()
        self._load_combined_models()
        if not self.combined_ready:
            self._load_default()

    def _discover_models(self) -> None:
        """Discover available .engine and .pt models in models_dir."""
        if not self.models_dir.exists():
            return
        for task, patterns in MODEL_PATTERNS.items():
            for name in patterns:
                path = self.models_dir / name
                if path.exists():
                    self.models[task] = str(path)
                    break
            # Also check repo-level models/
            if self.models[task] is None:
                repo_models = REPO_ROOT / "models"
                if repo_models.exists():
                    for name in patterns:
                        path = repo_models / name
                        if path.exists():
                            self.models[task] = str(path)
                            break

    def _load_combined_models(self) -> None:
        """Load HAZMAT and Landolt models for combined detection (both run every frame)."""
        if not YOLO_AVAILABLE:
            return
        for task in ("hazmat", "landolt"):
            path = self.models.get(task)
            if not path or not os.path.exists(path):
                continue
            try:
                m = YOLO(path, task="detect")
                if task == "hazmat":
                    self._hazmat_model = m
                else:
                    self._landolt_model = m
            except Exception as e:
                print(f"[YOLO11] Load {task} error: {e}")
        self.combined_ready = self._hazmat_model is not None or self._landolt_model is not None
        if self.combined_ready:
            print(f"[YOLO11] Combined mode: HAZMAT={'✓' if self._hazmat_model else '-'} Landolt={'✓' if self._landolt_model else '-'}")

    def _load_default(self) -> bool:
        """Load default task model (fallback when combined not available)."""
        return self.switch_task(self.default_task)

    def detect_combined(
        self,
        frame: Any,
        conf: Optional[float] = None,
        max_det: Optional[int] = None,
    ) -> List[Dict[str, Any]]:
        """
        Run both HAZMAT and Landolt models on frame and merge detections.
        Returns combined list with each detection tagged as source='hazmat' or 'landolt'.
        """
        if frame is None:
            return []
        conf = conf if conf is not None else self.config.confidence_threshold
        max_det = max_det if max_det is not None else self.config.max_detections
        imgsz, iou = self.config.imgsz, self.config.iou_threshold

        all_detections = []
        if self._hazmat_model:
            all_detections.extend(_run_model(self._hazmat_model, frame, conf, max_det, imgsz, iou, "hazmat"))
        if self._landolt_model:
            all_detections.extend(_run_model(self._landolt_model, frame, conf, max_det, imgsz, iou, "landolt", exclude_classes={"person"}))

        return all_detections

    def switch_task(self, task: str) -> bool:
        """Switch to model for given task (general, hazmat, landolt)."""
        if task not in self.models:
            return False
        path = self.models.get(task)
        if not path or not os.path.exists(path):
            return False
        if self.current_task == task and self.ready:
            return True
        if not YOLO_AVAILABLE:
            print("[YOLO11] Ultralytics not available")
            return False
        try:
            self.current_model = YOLO(path, task="detect")
            self.current_task = task
            self.ready = True
            return True
        except Exception as e:
            print(f"[YOLO11] Load error for {task}: {e}")
            self.ready = False
            return False

    def detect(
        self,
        frame: Any,
        conf: Optional[float] = None,
        max_det: Optional[int] = None,
    ) -> List[Dict[str, Any]]:
        """
        Run YOLO inference on frame.

        Args:
            frame: BGR numpy array (H, W, 3)
            conf: Override confidence threshold
            max_det: Override max detections

        Returns:
            List of detections: {class_id, class_name, confidence, bbox, center}
        """
        if not self.ready or self.current_model is None or frame is None:
            return []

        conf = conf if conf is not None else self.config.confidence_threshold
        max_det = max_det if max_det is not None else self.config.max_detections

        try:
            results = self.current_model(
                frame,
                conf=conf,
                iou=self.config.iou_threshold,
                verbose=False,
                max_det=max_det,
                imgsz=self.config.imgsz,
            )
            out = []
            for r in results:
                if r.boxes is None:
                    continue
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    out.append({
                        "class_id": cls_id,
                        "class_name": r.names.get(cls_id, f"class_{cls_id}"),
                        "confidence": round(float(box.conf[0]), 3),
                        "bbox": list(map(int, box.xyxy[0].tolist())),
                        "center": [
                            (int(box.xyxy[0][0]) + int(box.xyxy[0][2])) // 2,
                            (int(box.xyxy[0][1]) + int(box.xyxy[0][3])) // 2,
                        ],
                    })
            return out
        except Exception as e:
            print(f"[YOLO11] Detection error: {e}")
            return []

    def get_available_tasks(self) -> List[str]:
        """Return list of tasks with loaded models."""
        return [t for t, p in self.models.items() if p]

    def get_current_task(self) -> Optional[str]:
        """Return current active task. 'combined' when HAZMAT+Landolt run together."""
        if self.combined_ready:
            return "combined"
        return self.current_task
