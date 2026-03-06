#!/usr/bin/env python3
"""
Multi-Camera ZMQ Streamer with GPU Acceleration
Streams all cameras to laptop GUI via ZMQ (like deepstream_yolo_node but simpler)
"""

import sys
import os
import time
import threading
import json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

try:
    from shared.constants import ZMQ_PORT_CAMERA, JPEG_QUALITY
except:
    ZMQ_PORT_CAMERA = 5557
    JPEG_QUALITY = 65

try:
    import zmq
except:
    print("ERROR: pyzmq not installed")
    sys.exit(1)

try:
    import numpy as np
    import cv2
except:
    print("ERROR: opencv-python not installed")
    sys.exit(1)

try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst, GLib
    Gst.init(None)
except:
    print("ERROR: GStreamer not available")
    sys.exit(1)

def gst_to_numpy(sample):
    """Convert GstSample to BGR numpy"""
    if sample is None:
        return None
    buf = sample.get_buffer()
    caps = sample.get_caps()
    if buf is None or caps is None:
        return None
    fmt = caps.get_structure(0)
    w = fmt.get_value("width")
    h = fmt.get_value("height")
    success, map_info = buf.map(Gst.MapFlags.READ)
    if not success:
        return None
    try:
        arr = np.ndarray((h, w, 3), dtype=np.uint8, buffer=map_info.data)
        return arr.copy()
    finally:
        buf.unmap(map_info)

class GstCameraReader:
    """GPU-accelerated camera reader using GStreamer"""
    def __init__(self, cam_id, device, width=640, height=480, fps=10):
        self.cam_id = cam_id
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.appsink = None
        
    def build_pipeline(self):
        # GPU pipeline: v4l2src -> nvv4l2decoder -> nvvidconv -> BGR -> appsink
        src = (
            f"v4l2src device={self.device} io-mode=2 ! "
            f"image/jpeg,width={self.width},height={self.height},framerate={self.fps}/1 ! "
            f"nvv4l2decoder mjpeg=1 ! "
            f"nvvidconv ! "
            f"video/x-raw,format=BGRx ! "
            f"videoconvert ! "
            f"video/x-raw,format=BGR ! "
            f"appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
        )
        try:
            self.pipeline = Gst.parse_launch(src)
            self.appsink = self.pipeline.get_by_name("sink")
            if not self.appsink:
                return False
            self.appsink.set_property("emit-signals", True)
            self.appsink.set_property("max-buffers", 1)
            self.appsink.set_property("drop", True)
            self.appsink.set_property("sync", False)
            return True
        except Exception as e:
            print(f"[{self.cam_id}] Pipeline build failed: {e}")
            return False
    
    def start(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.PLAYING)
    
    def pull_frame(self):
        """Pull latest frame (non-blocking)"""
        if not self.appsink:
            return None
        sample = self.appsink.emit("pull-sample")
        if sample:
            return gst_to_numpy(sample)
        return None
    
    def stop(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)

def test_camera(device):
    """Test if camera works"""
    try:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            return False
        ret, _ = cap.read()
        cap.release()
        return ret
    except:
        return False

class MultiCameraStreamer:
    def __init__(self, port=ZMQ_PORT_CAMERA, max_cameras=2):
        self.port = port
        self.max_cameras = max_cameras
        self.cameras = {}
        self.context = None
        self.socket = None
        self.running = False
        self.stream_thread = None
        self.frame_counters = {}
        
    def start(self):
        print("🔍 Detecting cameras...")
        available = []
        cam_names = {0:'front', 2:'arm', 4:'backward'}
        
        for i in range(11):
            dev = f"/dev/video{i}"
            if os.path.exists(dev):
                print(f"   Testing {dev}...", end="", flush=True)
                if test_camera(i):
                    available.append(i)
                    print(f" ✅ {cam_names.get(i, f'cam{i}')}")
                else:
                    print(" ❌")
        
        if not available:
            print("❌ No cameras found")
            return
        
        # Limit cameras for USB bandwidth
        if len(available) > self.max_cameras:
            print(f"⚠️  Limiting to {self.max_cameras} cameras (USB bandwidth)")
            available = available[:self.max_cameras]
        
        print(f"\n📹 Starting {len(available)} cameras...")
        
        # Build pipelines (low res for bandwidth)
        WIDTH, HEIGHT, FPS = 640, 480, 10
        
        for idx in available:
            cam_id = cam_names.get(idx, f'cam{idx}')
            dev = f"/dev/video{idx}"
            
            reader = GstCameraReader(cam_id, dev, WIDTH, HEIGHT, FPS)
            if reader.build_pipeline():
                self.cameras[cam_id] = reader
                self.frame_counters[cam_id] = 0
                print(f"✅ {cam_id}: {dev} ({WIDTH}x{HEIGHT}@{FPS}fps)")
            else:
                print(f"❌ {cam_id}: pipeline failed")
        
        if not self.cameras:
            print("❌ No cameras started")
            return
        
        # Start pipelines
        for reader in self.cameras.values():
            reader.start()
        
        time.sleep(1)  # Let pipelines stabilize
        
        # ZMQ socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 60)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f"tcp://*:{self.port}")
        
        print(f"\n🚀 Streaming on tcp://*:{self.port}")
        print(f"   Cameras: {list(self.cameras.keys())}")
        print("   Press Ctrl+C to stop\n")
        
        self.running = True
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()
        
        # Main loop
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n⚠️  Stopping...")
            self.stop()
    
    def _stream_loop(self):
        """Stream frames from all cameras via ZMQ"""
        frame_interval = 1.0 / 10  # 10 FPS target
        
        while self.running:
            loop_start = time.time()
            
            for cam_id, reader in self.cameras.items():
                frame = reader.pull_frame()
                if frame is None:
                    continue
                
                self.frame_counters[cam_id] += 1
                
                # Encode JPEG
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
                success, jpeg = cv2.imencode('.jpg', frame, encode_params)
                if not success:
                    continue
                
                jpeg_bytes = jpeg.tobytes()
                h, w = frame.shape[:2]
                
                # Header
                header = {
                    "msg_type": "camera_frame",
                    "camera_id": cam_id,
                    "timestamp": time.time(),
                    "frame_num": self.frame_counters[cam_id],
                    "width": w,
                    "height": h,
                    "fps": 10,
                    "detections": [],  # No inference in this simple version
                    "qr_codes": []
                }
                
                header_json = json.dumps(header).encode('utf-8')
                
                # Send: header + JPEG
                try:
                    self.socket.send_multipart([header_json, jpeg_bytes], flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass  # Socket full, skip frame
            
            # Sleep to maintain FPS
            elapsed = time.time() - loop_start
            sleep_time = frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def stop(self):
        self.running = False
        if self.stream_thread:
            self.stream_thread.join(timeout=2)
        for reader in self.cameras.values():
            reader.stop()
        if self.socket:
            self.socket.close()
        if self.context:
            self.context.term()
        print("🛑 Stopped")

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=ZMQ_PORT_CAMERA, help='ZMQ port')
    parser.add_argument('--max-cameras', type=int, default=2, help='Max cameras (USB bandwidth)')
    args = parser.parse_args()
    
    print(f"""
╔══════════════════════════════════════════════════════════╗
║   RoboCup Rescue - Camera ZMQ Streamer                  ║
║   GPU Accelerated (max {args.max_cameras} cameras)                   ║
╚══════════════════════════════════════════════════════════╝
    """)
    
    streamer = MultiCameraStreamer(port=args.port, max_cameras=args.max_cameras)
    streamer.start()

if __name__ == '__main__':
    main()
