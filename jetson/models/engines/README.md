# YOLO Model Engines

Copy your trained TensorRT engine files here to enable YOLO detection on the robot.

## Supported Models

The GPU camera node auto-discovers models and enables inference when any of these exist:

| Task | Preferred | Alternatives |
|------|-----------|--------------|
| **HAZMAT** | hazmat_yolo11n.engine | hazmat_yolo11n.pt, hazmat_yolov8n.engine |
| **Landolt-C** | landolt_yolo11n.engine | landolt_yolo11n.pt, landolt_yolov8n.engine |
| **General** | yolo11n.engine | yolo11n.pt, yolov8n.engine, yolov8n.pt |

## Enabling YOLO

1. Train models (see `../train_models.py` or Windows/Colab docs).
2. Export to TensorRT on the Jetson:
   ```bash
   yolo export model=runs/detect/hazmat_yolo11n/weights/best.pt format=engine device=0 half=True imgsz=640
   ```
3. Copy engine files here:
   ```bash
   cp hazmat_yolo11n.engine ~/robocup_rescue_2026/jetson/models/engines/
   cp landolt_yolo11n.engine ~/robocup_rescue_2026/jetson/models/engines/
   ```
4. Restart the robot (`./start_robot.sh`). YOLO will auto-enable on front+arm cameras.

No code changes required. Without models, the node streams GPU-accelerated video only.

## Repository / GitHub

Large files (`.pt`, `.whl`, `.tar.xz`, `cudnn-*`, `torchvision/`) are in `.gitignore` so the repo stays uploadable to GitHub. Keep only `.engine` files (and this README) in version control, or add engines via release/assets and document the install path.
