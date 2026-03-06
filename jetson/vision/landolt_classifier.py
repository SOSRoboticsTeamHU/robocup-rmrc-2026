#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Landolt-C Direction Classifier
=========================================================
8-direction classification of Landolt-C gap orientation for Linear Rail Inspect test.

Directions (relative to identifier/up): N, NE, E, SE, S, SW, W, NW
Optimized for Jetson Orin Nano Super: lightweight CNN (~50MB) or rule-based fallback.
"""

import os
import sys
from pathlib import Path
from typing import List, Tuple, Optional
from dataclasses import dataclass

# Add repo root for imports
SCRIPT_DIR = Path(__file__).resolve().parent
JETSON_DIR = SCRIPT_DIR.parent
REPO_ROOT = JETSON_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    import numpy as np
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    np = None
    cv2 = None

# 8 directions: 0=Up(N), 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW
LANDOLT_DIRECTIONS = ["Up", "Up-Right", "Right", "Down-Right", "Down", "Down-Left", "Left", "Up-Left"]
LANDOLT_DIRECTIONS_SHORT = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]


@dataclass
class LandoltReading:
    """Single Landolt-C reading with direction and confidence."""
    direction_idx: int
    direction_name: str
    confidence: float
    bbox: Optional[Tuple[int, int, int, int]] = None
    center: Optional[Tuple[int, int]] = None


class LandoltClassifier:
    """
    Landolt-C 8-direction classifier.
    
    Uses a two-stage approach:
    1. YOLO detects Landolt ring bounding boxes (external)
    2. This classifier predicts gap direction from cropped region
    
    Fallback: rule-based orientation from edge/circle detection when no CNN loaded.
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        input_size: Tuple[int, int] = (64, 64),
        confidence_threshold: float = 0.5,
        use_rule_based_fallback: bool = False,
    ):
        self.model_path = model_path
        self.input_size = input_size
        self.confidence_threshold = confidence_threshold
        self.use_rule_based_fallback = use_rule_based_fallback
        self.model = None
        self.ready = False
        self._try_load_model()

    def _try_load_model(self) -> bool:
        """Load lightweight CNN for direction classification if available."""
        if not self.model_path:
            models_dir = JETSON_DIR / "models" / "engines"
            candidates = [
                models_dir / "landolt_direction.onnx",
                models_dir / "landolt_direction.pt",
                REPO_ROOT / "models" / "landolt_direction.onnx",
            ]
            for p in candidates:
                if p and Path(p).exists():
                    self.model_path = str(p)
                    break

        if not self.model_path or not os.path.exists(self.model_path):
            print("[LANDOLT] No direction model found; using Option B rule-based fallback (Hough/radial sampling)")
            self.use_rule_based_fallback = True
            return False

        try:
            ext = Path(self.model_path).suffix.lower()
            if ext == ".onnx":
                import onnxruntime as ort
                self.model = ort.InferenceSession(
                    self.model_path,
                    providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
                )
                self.ready = True
                print(f"[LANDOLT] ONNX model loaded: {self.model_path}")
                return True
            elif ext == ".pt":
                try:
                    import torch
                    self.model = torch.jit.load(self.model_path, map_location="cuda" if torch.cuda.is_available() else "cpu")
                    self.model.eval()
                    self.ready = True
                    print(f"[LANDOLT] PyTorch model loaded: {self.model_path}")
                    return True
                except Exception:
                    pass
        except Exception as e:
            print(f"[LANDOLT] Model load error: {e}")

        return False

    def _preprocess_crop(self, frame: "np.ndarray", bbox: Tuple[int, int, int, int]) -> Optional["np.ndarray"]:
        """Extract and preprocess Landolt ring crop for inference."""
        if not CV2_AVAILABLE or frame is None:
            return None
        x1, y1, x2, y2 = bbox
        h, w = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return None
        resized = cv2.resize(crop, self.input_size, interpolation=cv2.INTER_LINEAR)
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY) if len(resized.shape) == 3 else resized
        gray = gray.astype(np.float32) / 255.0
        gray = (gray - 0.5) / 0.5
        if self.model and hasattr(self.model, "run"):
            return np.expand_dims(np.expand_dims(gray, 0), 0).astype(np.float32)
        if self.model and hasattr(self.model, "__call__"):
            import torch
            t = torch.from_numpy(np.expand_dims(np.expand_dims(gray, 0), 0)).float()
            if hasattr(self.model, "cuda") and torch.cuda.is_available():
                t = t.cuda()
            return t
        return gray

    def _predict_cnn(self, inp) -> Tuple[int, float]:
        """Run CNN inference. Returns (direction_idx, confidence)."""
        try:
            if hasattr(self.model, "run"):
                outputs = self.model.run(None, {"input": inp})
                logits = outputs[0][0]
            else:
                import torch
                with torch.no_grad():
                    out = self.model(inp)
                    logits = out.cpu().numpy()[0] if isinstance(out, torch.Tensor) else out[0]
            probs = self._softmax(logits)
            idx = int(np.argmax(probs))
            conf = float(probs[idx])
            return idx, conf
        except Exception as e:
            print(f"[LANDOLT] CNN predict error: {e}")
            return 0, 0.0

    def _softmax(self, x: "np.ndarray") -> "np.ndarray":
        e = np.exp(x - np.max(x))
        return e / e.sum()

    def _predict_rule_based(self, frame: "np.ndarray", bbox: Tuple[int, int, int, int]) -> Tuple[int, float]:
        """
        Rule-based direction estimation from edge/circle detection.
        Estimates gap direction from gradient magnitude orientation in the ring region.
        """
        if not CV2_AVAILABLE:
            return 0, 0.3
        x1, y1, x2, y2 = bbox
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return 0, 0.2
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) if len(crop.shape) == 3 else crop
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 50, 150)
        h, w = edges.shape
        cx, cy = w // 2, h // 2
        if h < 10 or w < 10:
            return 0, 0.2
        # Sample radial directions and find strongest edge response (gap)
        angles = np.linspace(0, 2 * np.pi, 9)[:-1]  # 8 directions
        scores = []
        r_min, r_max = min(w, h) // 4, min(w, h) // 2
        for a in angles:
            sx = cx + int(r_max * np.cos(a))
            sy = cy + int(r_max * np.sin(a))
            sx = np.clip(sx, 0, w - 1)
            sy = np.clip(sy, 0, h - 1)
            if 0 <= sx < w and 0 <= sy < h:
                val = np.mean(edges[max(0, sy - 2):min(h, sy + 3), max(0, sx - 2):min(w, sx + 3)])
                scores.append(val)
            else:
                scores.append(0)
        if not scores:
            return 0, 0.2
        scores = np.array(scores)
        idx = int(np.argmax(scores))
        mx = np.max(scores)
        conf = min(0.9, 0.3 + mx / 80.0) if mx > 0 else 0.3
        return idx, conf

    def predict_direction(
        self,
        frame: "np.ndarray",
        bbox: Tuple[int, int, int, int],
    ) -> LandoltReading:
        """
        Predict Landolt-C gap direction from frame and bounding box.

        Args:
            frame: BGR image
            bbox: (x1, y1, x2, y2) from YOLO Landolt detection

        Returns:
            LandoltReading with direction_idx (0-7), name, confidence
        """
        direction_idx = 0
        confidence = 0.0

        if self.ready and self.model:
            inp = self._preprocess_crop(frame, bbox)
            if inp is not None:
                direction_idx, confidence = self._predict_cnn(inp)

        if confidence < self.confidence_threshold and self.use_rule_based_fallback:
            direction_idx, confidence = self._predict_rule_based(frame, bbox)

        x1, y1, x2, y2 = bbox
        center = ((x1 + x2) // 2, (y1 + y2) // 2)

        return LandoltReading(
            direction_idx=direction_idx,
            direction_name=LANDOLT_DIRECTIONS[direction_idx],
            confidence=confidence,
            bbox=bbox,
            center=center,
        )

    def predict_from_detections(
        self,
        frame: "np.ndarray",
        detections: List[dict],
        landolt_class_names: Optional[set] = None,
    ) -> List[LandoltReading]:
        """
        Predict directions for all Landolt ring detections in frame.

        Args:
            frame: BGR image
            detections: List of YOLO detections with 'bbox' and optionally 'class_name'
            landolt_class_names: Set of class names that are Landolt rings (default: rings, landolt, etc.)

        Returns:
            List of LandoltReading for each detected Landolt ring
        """
        if landolt_class_names is None:
            landolt_class_names = {"rings", "landolt", "landolt_c", "ring", "landolt_rings"}

        readings = []
        for d in detections:
            cls_name = (d.get("class_name") or "").lower()
            if landolt_class_names and cls_name not in landolt_class_names:
                continue
            bbox = d.get("bbox")
            if not bbox or len(bbox) < 4:
                continue
            bbox = tuple(int(x) for x in bbox[:4])
            reading = self.predict_direction(frame, bbox)
            readings.append(reading)
        return readings
