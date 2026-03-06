#!/usr/bin/env python3
"""
DeepStream Basic Camera Streaming (No Inference)
USB Cameras → H264 Encode → Display
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

class MultiCameraStreamer:
    def __init__(self):
        self.pipeline = None
        self.loop = None
        
    def create_pipeline(self):
        print("🎬 Creating DeepStream pipeline...")
        
        pipeline = Gst.Pipeline.new("multi-camera-pipeline")
        
        # Detect and validate cameras
        print("🔍 Detecting cameras...")
        available_cameras = []
        for i in [1, 3, 5]:  # Known working cameras from lerobot
            device_path = f"/dev/video{i}"
            if os.path.exists(device_path):
                print(f"   Testing {device_path}...", end="")
                if test_camera(i):
                    available_cameras.append(i)
                    print(" ✅")
                else:
                    print(" ❌")
        
        if not available_cameras:
            print("❌ No working cameras found!")
            return None
        
        print(f"📹 Using cameras: {available_cameras}")
        
        # Use only first camera for now (simplest)
        cam_idx = available_cameras[0]
        cam_name = {0:'front', 2:'arm', 4:'backward'}.get(cam_idx, f'cam{cam_idx}')
        
        # Source
        source = Gst.ElementFactory.make("v4l2src", "source")
        source.set_property("device", f"/dev/video{cam_idx}")
        
        # Caps - MJPEG format
        caps_str = "image/jpeg,width=1280,height=720,framerate=30/1"
        caps = Gst.ElementFactory.make("capsfilter", "caps")
        caps.set_property("caps", Gst.Caps.from_string(caps_str))
        
        # JPEG decoder
        jpegdec = Gst.ElementFactory.make("jpegdec", "jpegdec")
        
        # Video convert
        vidconv = Gst.ElementFactory.make("videoconvert", "vidconv")
        
        # Encoder
        encoder = Gst.ElementFactory.make("x264enc", "encoder")
        encoder.set_property("bitrate", 2000)
        encoder.set_property("speed-preset", "ultrafast")
        encoder.set_property("tune", "zerolatency")
        
        # Parser
        parser = Gst.ElementFactory.make("h264parse", "parser")
        
        # Sink
        sink = Gst.ElementFactory.make("fakesink", "sink")
        sink.set_property("sync", False)
        
        # Add all elements
        for elem in [source, caps, jpegdec, vidconv, encoder, parser, sink]:
            pipeline.add(elem)
        
        # Link
        if not source.link(caps):
            print("❌ Failed to link source")
            return None
        if not caps.link(jpegdec):
            print("❌ Failed to link jpegdec")
            return None
        if not jpegdec.link(vidconv):
            print("❌ Failed to link vidconv")
            return None
        if not vidconv.link(encoder):
            print("❌ Failed to link encoder")
            return None
        if not encoder.link(parser):
            print("❌ Failed to link parser")
            return None
        if not parser.link(sink):
            print("❌ Failed to link sink")
            return None
        
        print(f"✅ Pipeline created (camera {cam_idx} - {cam_name})")
        
        # Bus watch
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
                    print(f"✅ Pipeline PLAYING")
        elif t == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            print(f"⚠️  {warn}")
    
    def start(self):
        print("🚀 Starting camera streaming...")
        print("=" * 60)
        
        if not self.create_pipeline():
            print("❌ Failed to create pipeline")
            sys.exit(1)
        
        # Start
        print("⏳ Starting pipeline...")
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        
        if ret == Gst.StateChangeReturn.FAILURE:
            print("❌ State change returned FAILURE")
            sys.exit(1)
        
        if ret == Gst.StateChangeReturn.ASYNC:
            print("   (ASYNC state change, waiting...)")
            ret, state, pending = self.pipeline.get_state(5 * Gst.SECOND)
            if ret == Gst.StateChangeReturn.FAILURE:
                print("❌ Failed to reach PLAYING state")
                sys.exit(1)
        
        print("\n🎥 Streaming active!")
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
    print("""
╔══════════════════════════════════════════════════════════╗
║   RoboCup Rescue - Camera Streaming                     ║
║   Stage 1: Basic Streaming                              ║
╚══════════════════════════════════════════════════════════╝
    """)
    
    streamer = MultiCameraStreamer()
    streamer.start()


if __name__ == '__main__':
    main()
