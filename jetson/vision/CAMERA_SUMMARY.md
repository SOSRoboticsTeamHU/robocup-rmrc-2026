# Camera Configuration Summary

## Current Status

### What's Working ✓
- **Front + Arm cameras**: Using Full GPU pipeline with TensorRT
  - Pipeline: v4l2src → nvjpegdec → nvvidconv → nvjpegenc
  - Resolution: 640x480@15fps
  - Hardware accelerated JPEG decode/encode on GPU
  - YOLO11 TensorRT engines for inference (.engine files)
  - Perfect for your SLAM/TensorRT workflow

### What's NOT Working ❌
- **Only 2 cameras work at once** due to USB bandwidth limitation
- All 4 cameras share the same USB 2.0 hub (Bus 01, 480 Mbps)
- Cannot access Bus 02 (USB 3.0) even when physically moving cameras

## Are You Using DeepStream/TensorRT?

**DeepStream**: ❌ NO
- Your code does NOT use DeepStream SDK
- DeepStream would use `nvinfer` GStreamer plugin
- You're using custom Python inference with Ultralytics YOLO

**TensorRT**: ✓ YES  
- YOLO11Processor loads `.engine` files (TensorRT format)
- Models: hazmat_yolo11n.engine, landolt_yolo11n.engine
- TensorRT inference runs on GPU efficiently
- Output shows: "Loading .../hazmat_yolo11n.engine for TensorRT inference"

**GPU Acceleration for Cameras**: ✓ YES (for front/arm)
- nvjpegdec: Hardware JPEG decode on GPU
- nvvidconv: GPU video conversion  
- nvjpegenc: Hardware JPEG encode on GPU
- This offloads camera processing from CPU

## Camera Pipeline Details

### Front/Arm (FullGPUMJPEGCameraReader):
```
USB Camera (MJPEG) 
  ↓
v4l2src (capture)
  ↓  
nvjpegdec (GPU decode to NV12)
  ↓
nvvidconv (GPU conversion)
  ↓
nvjpegenc (GPU encode back to JPEG)
  ↓
Python (YOLO inference with TensorRT)
```

### Left/Right (LightMJPEGCameraReader):
```
USB Camera (MJPEG)
  ↓
v4l2src (capture)
  ↓
appsink (CPU)
  ↓
Python (no inference)
```

## Recommendations

### For Competition:
1. **Use 2 cameras** (front + arm) with Full GPU pipeline
2. GPU handles camera decoding → frees CPU for SLAM
3. TensorRT handles YOLO inference efficiently
4. This is optimal for your setup

### If You Need 3-4 Cameras:
1. Buy powered USB 3.0 hub with independent bandwidth
2. Or use CSI/MIPI cameras instead of USB
3. Or lower all resolutions to 320x240@10fps (poor quality)

## Files Modified
- vision_node.py: Added MJPEG device validation, GPU/Light pipelines
- All changes focused on USB bandwidth optimization

## Next Steps
Test with other cameras to see if they have better USB compatibility.
The software is optimized; the limitation is purely hardware (USB bandwidth).
