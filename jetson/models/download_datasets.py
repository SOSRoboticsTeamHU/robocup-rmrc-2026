#!/usr/bin/env python3
"""
Download HAZMAT and Landolt-C datasets from Roboflow
Run once to download, then train with train_models.py
"""
import os

MODELS_DIR = os.path.dirname(os.path.abspath(__file__))
DATASETS_DIR = os.path.join(MODELS_DIR, "datasets")

def download_hazmat():
    """Download DOT-HAZMAT dataset from Roboflow"""
    from roboflow import Roboflow
    
    print("Downloading HAZMAT dataset...")
    # Public dataset - no API key needed for download
    rf = Roboflow()
    project = rf.workspace("divyadharshini-k").project("dot-hazmat")
    dataset = project.version(2).download("yolov8", location=os.path.join(DATASETS_DIR, "hazmat_yolo"))
    print(f"HAZMAT dataset saved to: {dataset.location}")
    return dataset.location

def download_landolt():
    """Download Landolt-C dataset from Roboflow"""
    from roboflow import Roboflow
    
    print("Downloading Landolt-C dataset...")
    rf = Roboflow()
    project = rf.workspace("landoltring").project("landoltring")
    dataset = project.version(1).download("yolov8", location=os.path.join(DATASETS_DIR, "landolt_yolo"))
    print(f"Landolt-C dataset saved to: {dataset.location}")
    return dataset.location

if __name__ == "__main__":
    print("=== RoboCup Rescue Dataset Downloader ===")
    print(f"Target directory: {DATASETS_DIR}")
    print()
    
    try:
        hazmat_path = download_hazmat()
    except Exception as e:
        print(f"HAZMAT download failed: {e}")
        print("Try manually: https://universe.roboflow.com/divyadharshini-k/dot-hazmat")
        hazmat_path = None
    
    print()
    
    try:
        landolt_path = download_landolt()
    except Exception as e:
        print(f"Landolt download failed: {e}")
        print("Try manually: https://universe.roboflow.com/landoltring/landoltring")
        landolt_path = None
    
    print()
    print("=== Download Complete ===")
    if hazmat_path:
        print(f"HAZMAT: {hazmat_path}")
    if landolt_path:
        print(f"Landolt: {landolt_path}")
