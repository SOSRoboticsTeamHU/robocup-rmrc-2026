#!/usr/bin/env python3
"""
DeepStream Basic Camera Streaming (No Inference)
4x USB Cameras → H264 Encode → ZMQ Stream

Stage 1: Camera validation and basic streaming
Stage 2: Add TensorRT inference later
"""

import sys
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import zmq
import configparser

# Initialize GStreamer
Gst.init(None)

class MultiCameraStreamer:
    def __init__(self, config_file='camera_config.ini'):
        """Initialize multi-camera streaming pipeline"""
        self.config = self.load_config(config_file)
        self.pipeline = None
        self.loop = None
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        
    def load_config(self, config_file):
        """Load camera configuration"""
        config = configparser.ConfigParser()
        config.read(config_file)
        return config
    
    def create_pipeline(self):
        """Create GStreamer pipeline for 3 cameras"""
        
        # Pipeline string
        # 3 cameras: front, arm, backward
        pipeline_str = """
        # Front Camera (USB /dev/video0)
        v4l2src device=/dev/video0 name=cam_front ! 
            video/x-raw,width=1920,height=1080,framerate=30/1 !
            nvvidconv ! 
            video/x-raw(memory:NVMM),format=NV12 !
            queue ! mux.sink_0
        
        # Arm Camera (USB /dev/video2)  
        v4l2src device=/dev/video2 name=cam_arm !
            video/x-raw,width=1920,height=1080,framerate=30/1 !
            nvvidconv !
            video/x-raw(memory:NVMM),format=NV12 !
            queue ! mux.sink_1
        
        # Backward Camera (USB /dev/video4)
        v4l2src device=/dev/video4 name=cam_backward !
            video/x-raw,width=640,height=480,framerate=30/1 !
            nvvidconv !
            video/x-raw(memory:NVMM),format=NV12 !
            queue ! mux.sink_2
        
        # Muxer (3 streams)
        nvstreammux name=mux batch-size=3 width=1920 height=1080 !
        
        # H264 Encode (hardware accelerated)
        nvv4l2h264enc bitrate=4000000 !
        h264parse !
        
        # RTSP or File Sink (for now, fakesink for testing)
        fakesink sync=false
        """
        
        print("🎬 Creating DeepStream pipeline...")
        self.pipeline = Gst.parse_launch(pipeline_str)
        
        # Add bus watch
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_bus_message)
        
        return self.pipeline
    
    def on_bus_message(self, bus, message):
        """Handle GStreamer bus messages"""
        t = message.type
        
        if t == Gst.MessageType.EOS:
            print("End-of-stream")
            self.loop.quit()
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}, {debug}")
            self.loop.quit()
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending = message.parse_state_changed()
                print(f"Pipeline state: {old_state.value_nick} → {new_state.value_nick}")
    
    def start(self):
        """Start streaming"""
        print("🚀 Starting camera streaming...")
        print("📹 Cameras: front, arm, backward")
        print("🔧 Mode: Basic streaming (no inference)")
        print("=" * 60)
        
        # Create pipeline
        self.create_pipeline()
        
        # Start pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("❌ Failed to start pipeline")
            sys.exit(1)
        
        # Run main loop
        try:
            self.loop = GLib.MainLoop()
            self.loop.run()
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted by user")
        finally:
            self.stop()
    
    def stop(self):
        """Stop streaming"""
        print("🛑 Stopping pipeline...")
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if self.zmq_socket:
            self.zmq_socket.close()
        if self.zmq_context:
            self.zmq_context.term()


def main():
    print("""
    ╔══════════════════════════════════════════════════════════╗
    ║   RoboCup Rescue - DeepStream Camera Streaming          ║
    ║   Stage 1: Basic Streaming (No Inference)               ║
    ╚══════════════════════════════════════════════════════════╝
    """)
    
    streamer = MultiCameraStreamer()
    streamer.start()


if __name__ == '__main__':
    main()
