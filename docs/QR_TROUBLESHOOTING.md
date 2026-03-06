# QR Code Detection Troubleshooting

## "I showed a QR code but nothing happened"

### 1. Check where QR runs (laptop vs Jetson)

QR decoding can run on **Jetson** (vision node) or **laptop** (GUI receiver). By default it runs on the laptop to free Jetson CPU for SLAM.

**If QR runs on laptop** (default):
- Requires `pyzbar` on the laptop
- Install: `pip install pyzbar`
- On macOS: `brew install zbar` (pyzbar needs libzbar)
- On Ubuntu: `sudo apt install libzbar0`

**If pyzbar is not installed on laptop** – use Jetson instead:
- In `shared/constants.py`: set `QR_DECODE_ON_LAPTOP = False`
- On Jetson, run with QR enabled:
  ```bash
  python3 deepstream_yolo_node.py --qr-on-jetson
  # or
  python3 vision/vision_node.py --no-gpu-only
  ```

### 2. Ensure the QR checkbox is ON (when using Jetson QR)

In the GUI, the "QR" checkbox must be checked so Jetson runs QR detection.  
(When QR runs on the laptop, the checkbox does not control it.)

### 3. QR code quality

- Size: QR code should be at least ~50×50 pixels in frame
- Distance: Move camera closer if the code looks small
- Lighting: Avoid glare and strong shadows
- Focus: Camera must be in focus
- Angle: Face the QR code roughly toward the camera

### 4. Which camera?

QR detection runs on the cameras that receive AI streams (e.g. arm, left for vision_node). Point the QR at one of those cameras, not a passive Pi-only camera.

### 5. Decode rate

QR is decoded every Nth frame (default: 2) to save CPU. To change it, set `QR_DECODE_INTERVAL = 1` in `shared/constants.py` for every-frame decoding.
