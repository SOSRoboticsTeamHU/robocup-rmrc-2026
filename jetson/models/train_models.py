#!/usr/bin/env python3
"""
Train YOLOv8 models for HAZMAT and Landolt-C detection
Then export to TensorRT for Jetson deployment
"""
import os
import sys
from pathlib import Path

MODELS_DIR = Path(__file__).resolve().parent
DATASETS_DIR = MODELS_DIR / "datasets"
WEIGHTS_DIR = MODELS_DIR / "weights"
ENGINES_DIR = MODELS_DIR / "engines"

# Training config
EPOCHS = 100
IMGSZ = 640
BATCH = 16  # Adjust based on GPU memory

def train_hazmat():
    """Train YOLOv8 on HAZMAT dataset"""
    from ultralytics import YOLO
    
    dataset_path = DATASETS_DIR / "hazmat_yolo" / "data.yaml"
    if not dataset_path.exists():
        print(f"ERROR: HAZMAT dataset not found at {dataset_path}")
        print("Run download_datasets.py first or download manually from Roboflow")
        return None
    
    print("="*50)
    print("Training HAZMAT detection model")
    print("="*50)
    
    model = YOLO("yolov8n.pt")  # Start with pretrained nano model
    
    results = model.train(
        data=str(dataset_path),
        epochs=EPOCHS,
        imgsz=IMGSZ,
        batch=BATCH,
        name="hazmat_yolov8n",
        project=str(WEIGHTS_DIR),
        exist_ok=True,
        device=0,  # GPU
        workers=4,
        patience=20,  # Early stopping
        save=True,
        plots=True,
    )
    
    # Save best model
    best_model = WEIGHTS_DIR / "hazmat_yolov8n" / "weights" / "best.pt"
    if best_model.exists():
        final_path = WEIGHTS_DIR / "hazmat_yolov8n.pt"
        import shutil
        shutil.copy(best_model, final_path)
        print(f"Best model saved to: {final_path}")
        return final_path
    return None

def train_landolt():
    """Train YOLOv8 on Landolt-C dataset"""
    from ultralytics import YOLO
    
    dataset_path = DATASETS_DIR / "landolt_yolo" / "data.yaml"
    if not dataset_path.exists():
        print(f"ERROR: Landolt dataset not found at {dataset_path}")
        print("Run download_datasets.py first or download manually from Roboflow")
        return None
    
    print("="*50)
    print("Training Landolt-C detection model")
    print("="*50)
    
    model = YOLO("yolov8n.pt")
    
    results = model.train(
        data=str(dataset_path),
        epochs=EPOCHS,
        imgsz=IMGSZ,
        batch=BATCH,
        name="landolt_yolov8n",
        project=str(WEIGHTS_DIR),
        exist_ok=True,
        device=0,
        workers=4,
        patience=20,
        save=True,
        plots=True,
    )
    
    best_model = WEIGHTS_DIR / "landolt_yolov8n" / "weights" / "best.pt"
    if best_model.exists():
        final_path = WEIGHTS_DIR / "landolt_yolov8n.pt"
        import shutil
        shutil.copy(best_model, final_path)
        print(f"Best model saved to: {final_path}")
        return final_path
    return None

def export_tensorrt(model_path: Path, output_name: str):
    """Export PyTorch model to TensorRT engine"""
    from ultralytics import YOLO
    
    print(f"Exporting {model_path} to TensorRT...")
    
    model = YOLO(str(model_path))
    
    # Export to TensorRT (FP16 for Jetson)
    engine_path = model.export(
        format="engine",
        device=0,
        half=True,  # FP16 for faster inference
        imgsz=IMGSZ,
        workspace=4,  # GB
        verbose=True,
    )
    
    # Move to engines directory
    if engine_path and Path(engine_path).exists():
        final_path = ENGINES_DIR / f"{output_name}.engine"
        import shutil
        shutil.move(engine_path, final_path)
        print(f"TensorRT engine saved to: {final_path}")
        return final_path
    return None

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--hazmat", action="store_true", help="Train HAZMAT model")
    parser.add_argument("--landolt", action="store_true", help="Train Landolt-C model")
    parser.add_argument("--export", action="store_true", help="Export to TensorRT")
    parser.add_argument("--all", action="store_true", help="Train all and export")
    args = parser.parse_args()
    
    if args.all or (not args.hazmat and not args.landolt and not args.export):
        args.hazmat = args.landolt = args.export = True
    
    hazmat_model = None
    landolt_model = None
    
    if args.hazmat:
        hazmat_model = train_hazmat()
    
    if args.landolt:
        landolt_model = train_landolt()
    
    if args.export:
        if hazmat_model or (WEIGHTS_DIR / "hazmat_yolov8n.pt").exists():
            export_tensorrt(
                hazmat_model or WEIGHTS_DIR / "hazmat_yolov8n.pt",
                "hazmat_yolov8n"
            )
        
        if landolt_model or (WEIGHTS_DIR / "landolt_yolov8n.pt").exists():
            export_tensorrt(
                landolt_model or WEIGHTS_DIR / "landolt_yolov8n.pt",
                "landolt_yolov8n"
            )
    
    print()
    print("="*50)
    print("Training complete!")
    print("="*50)
    print(f"Weights: {WEIGHTS_DIR}")
    print(f"Engines: {ENGINES_DIR}")
