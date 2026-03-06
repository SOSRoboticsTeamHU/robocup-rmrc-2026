# Camera Stream FPS Optimization

## If you're getting ~1 FPS (laggy stream)

### Changes made (2026)

1. **Removed 15 FPS cap** – Now uses 30 FPS (configurable to 60) instead of `min(CAMERA_FPS, 15)`.
2. **QR detection throttled** – QR (pyzbar) runs every 5th frame instead of every 2nd; it is CPU-heavy (~50–100 ms per frame).
3. **YOLO every 3rd frame** – Inference runs every 3rd frame on YOLO cameras instead of every 2nd.
4. **Increased appsink buffers** – `max-buffers=4` for JPEG, `max-buffers=2` for BGR to reduce pipeline blocking.
5. **Removed verbose logging** – Dropped per-frame debug prints that add overhead.

### For maximum FPS (60+)

- **Disable YOLO and QR** – Use the laptop GUI checkboxes or ZMQ control (`enable_yolo: false`, `enable_qr: false`). Streaming-only can reach 30–60 FPS.
- **Run with `--fps 60`:**
  ```bash
  python3 gpu_camera_node.py --fps 60
  ```
- **Use fewer YOLO cameras** – YOLO on front only: `--yolo-cameras front` (deepstream_yolo_node).

### Architecture note

This setup uses **GStreamer + Ultralytics YOLO**, not the full DeepStream SDK with `nvinfer`. For a fully GPU pipeline (decode → inference → encode without Python), you’d need a custom GStreamer pipeline using `nvstreammux`, `nvinfer`, and `nvvideoconvert`. The current design keeps flexibility while using `nvv4l2decoder` and `nvjpegenc` for GPU decode/encode.
