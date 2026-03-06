#!/usr/bin/env python3
"""
DeepStream Multi-Camera Display with GPU Acceleration
Ultra-low bandwidth mode for USB cameras
"""

import sys
import os
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

def test_camera(device):
    """Quick test if camera is valid capture device"""
    try:
        import cv2
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            return False
        ret, _ = cap.read()
        cap.release()
        return ret
    except:
        return False

class MultiCameraDisplay:
    def __init__(self, max_cameras=2):
        self.pipeline = None
        self.loop = None
        self.max_cameras = max_cameras
        
    def create_pipeline(self):
        print("🎬 Creating GPU-accelerated multi-camera pipeline...")
        
        # Detect cameras
        print("🔍 Detecting cameras...")
        available_cameras = []
        
        for i in range(11):
            if os.path.exists(f"/dev/video{i}"):
                print(f"   Testing /dev/video{i}...", end="", flush=True)
                if test_camera(i):
                    available_cameras.append(i)
                    print(" ✅")
                else:
                    print(" ❌")
        
        if not available_cameras:
            print("❌ No cameras found!")
            return None
        
        # Limit to max_cameras to avoid USB bandwidth issues
        if len(available_cameras) > self.max_cameras:
            print(f"\n⚠️  Found {len(available_cameras)} cameras, limiting to {self.max_cameras} for USB bandwidth")
            available_cameras = available_cameras[:self.max_cameras]
        
        print(f"📹 Active cameras: {available_cameras} ({len(available_cameras)} total)")
        num_cams = len(available_cameras)
        
        # Layout (front | arm | backward)
        if num_cams == 1:
            layout = [(0, 0, 1280, 720)]
        elif num_cams == 2:
            layout = [(0, 0, 640, 720), (640, 0, 640, 720)]
        else:
            layout = [(0, 0, 426, 720), (426, 0, 426, 720), (852, 0, 426, 720)]
        
        # ULTRA-LOW bandwidth settings
        CAPTURE_WIDTH = 640
        CAPTURE_HEIGHT = 480
        CAPTURE_FPS = 10  # Very low FPS
        
        print(f"\n⚡ USB Bandwidth settings:")
        print(f"   Capture: {CAPTURE_WIDTH}x{CAPTURE_HEIGHT}@{CAPTURE_FPS}fps")
        print(f"   Total: ~{num_cams * CAPTURE_WIDTH * CAPTURE_HEIGHT * CAPTURE_FPS * 1.5 / 1024 / 1024:.1f} MB/s")
        
        # Build pipeline
        cam_pipelines = []
        for idx, cam_idx in enumerate(available_cameras):
            cam_name = {0:'front', 2:'arm', 4:'backward'}.get(cam_idx, f'cam{cam_idx}')
            x, y, w, h = layout[min(idx, len(layout)-1)]
            
            # GPU pipeline
            cam_pipe = (
                f"v4l2src device=/dev/video{cam_idx} io-mode=2 ! "
                f"image/jpeg,width={CAPTURE_WIDTH},height={CAPTURE_HEIGHT},framerate={CAPTURE_FPS}/1 ! "
                f"nvv4l2decoder mjpeg=1 ! "
                f"nvvidconv ! "
                f"video/x-raw(memory:NVMM),width={w},height={h},format=RGBA ! "
                f"queue max-size-buffers=2 leaky=2 ! "
                f"comp.sink_{idx}"
            )
            cam_pipelines.append(cam_pipe)
            print(f"✅ Camera {cam_idx} ({cam_name}) @ ({x},{y}) {w}x{h}")
        
        # Compositor
        comp_pipeline = "nvcompositor name=comp "
        for idx in range(num_cams):
            x, y, w, h = layout[min(idx, len(layout)-1)]
            comp_pipeline += f"sink_{idx}::xpos={x} sink_{idx}::ypos={y} sink_{idx}::width={w} sink_{idx}::height={h} "
        
        full_pipeline = " ".join(cam_pipelines) + " " + comp_pipeline + " ! nvvidconv ! xvimagesink sync=false"
        
        print(f"\n🔧 Pipeline: {len(full_pipeline)} chars")
        
        try:
            pipeline = Gst.parse_launch(full_pipeline)
        except Exception as e:
            print(f"❌ GPU pipeline failed: {e}")
            print(f"\nTrying CPU fallback...")
            return self.create_pipeline_fallback(available_cameras, layout, CAPTURE_WIDTH, CAPTURE_HEIGHT, CAPTURE_FPS)
        
        print(f"✅ GPU pipeline ready")
        
        # Bus
        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_bus_message)
        
        self.pipeline = pipeline
        return pipeline
    
    def create_pipeline_fallback(self, cameras, layout, cap_w, cap_h, cap_fps):
        """CPU compositor fallback"""
        print("   Using CPU pipeline...")
        
        num_cams = len(cameras)
        cam_pipelines = []
        
        for idx, cam_idx in enumerate(cameras):
            cam_name = {0:'front', 2:'arm', 4:'backward'}.get(cam_idx, f'cam{cam_idx}')
            x, y, w, h = layout[min(idx, len(layout)-1)]
            
            cam_pipe = (
                f"v4l2src device=/dev/video{cam_idx} ! "
                f"image/jpeg,width={cap_w},height={cap_h},framerate={cap_fps}/1 ! "
                f"jpegdec ! "
                f"videoconvert ! "
                f"videoscale ! "
                f"video/x-raw,width={w},height={h} ! "
                f"queue max-size-buffers=2 leaky=2 ! "
                f"comp.sink_{idx}"
            )
            cam_pipelines.append(cam_pipe)
            print(f"✅ Camera {cam_idx} ({cam_name}) @ ({x},{y}) {w}x{h}")
        
        comp_pipeline = "compositor name=comp "
        for idx in range(num_cams):
            x, y = layout[min(idx, len(layout)-1)][:2]
            comp_pipeline += f"sink_{idx}::xpos={x} sink_{idx}::ypos={y} "
        
        full_pipeline = " ".join(cam_pipelines) + " " + comp_pipeline + " ! videoconvert ! xvimagesink sync=false"
        
        try:
            pipeline = Gst.parse_launch(full_pipeline)
            print(f"✅ CPU pipeline ready")
        except Exception as e:
            print(f"❌ Failed: {e}")
            return None
        
        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_bus_message)
        
        self.pipeline = pipeline
        return pipeline
    
    def on_bus_message(self, bus, message):
        t = message.type
        
        if t == Gst.MessageType.EOS:
            print("End-of-stream")
            self.loop.quit()
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"❌ Error: {err}")
            if debug:
                print(f"   Debug: {debug}")
            self.loop.quit()
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending = message.parse_state_changed()
                if new_state == Gst.State.PLAYING:
                    print("✅ Display ACTIVE")
        elif t == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            if "buffers are being dropped" not in str(warn):
                print(f"⚠️  {warn}")
    
    def start(self):
        print("🚀 Starting multi-camera display...")
        print("=" * 60)
        
        if not self.create_pipeline():
            print("❌ Failed to create pipeline")
            sys.exit(1)
        
        print("⏳ Starting...")
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        
        if ret == Gst.StateChangeReturn.FAILURE:
            print("❌ Start failed")
            sys.exit(1)
        
        if ret == Gst.StateChangeReturn.ASYNC:
            ret, state, pending = self.pipeline.get_state(15 * Gst.SECOND)
            if ret == Gst.StateChangeReturn.FAILURE:
                print("❌ Failed to reach PLAYING")
                sys.exit(1)
        
        print("\n📺 Multi-camera display active!")
        print("   Ultra-low bandwidth mode")
        print("Press Ctrl+C to stop\n")
        
        try:
            self.loop = GLib.MainLoop()
            self.loop.run()
        except KeyboardInterrupt:
            print("\n⚠️  Stopping...")
        finally:
            self.stop()
    
    def stop(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        print("🛑 Stopped")


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--max-cameras', type=int, default=2, help='Max cameras (default: 2 for USB bandwidth)')
    args = parser.parse_args()
    
    print(f"""
╔══════════════════════════════════════════════════════════╗
║   RoboCup Rescue - Multi-Camera Display                 ║
║   Ultra-Low Bandwidth (max {args.max_cameras} cameras)                  ║
╚══════════════════════════════════════════════════════════╝
    """)
    
    display = MultiCameraDisplay(max_cameras=args.max_cameras)
    display.start()


if __name__ == '__main__':
    main()
