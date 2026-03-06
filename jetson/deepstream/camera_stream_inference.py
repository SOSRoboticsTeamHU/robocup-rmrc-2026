#!/usr/bin/env python3
"""
DeepStream Camera Streaming with TensorRT Inference
4x USB Cameras → TensorRT (YOLO) → OSD → ZMQ Stream

Stage 2: Add YOLO inference with TensorRT engines
Models: hazmat_best.engine, landolt_best.engine
"""

import sys
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import pyds  # DeepStream Python bindings
import zmq
import json
from pathlib import Path

Gst.init(None)

class InferenceStreamer:
    def __init__(self, enable_inference=True):
        """
        Initialize multi-camera streaming with optional inference
        
        Args:
            enable_inference: Enable/disable TensorRT inference
        """
        self.enable_inference = enable_inference
        self.pipeline = None
        self.loop = None
        
        # ZMQ setup
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.bind("tcp://*:5557")
        
        # Model paths (TensorRT engines)
        self.models = {
            'hazmat': '/opt/robocup/models/hazmat_best.engine',
            'landolt': '/opt/robocup/models/landolt_best.engine'
        }
        
    def create_pipeline(self):
        """Create DeepStream pipeline with nvinfer"""
        
        print("🎬 Creating DeepStream inference pipeline...")
        
        # Create pipeline
        pipeline = Gst.Pipeline()
        
        # ============================================================
        # SOURCE: 3x USB Cameras (front=0, arm=2, backward=4)
        # ============================================================
        
        sources = []
        cam_config = [('front', 0), ('arm', 2), ('backward', 4)]
        for i, (cam_name, dev_idx) in enumerate(cam_config):
            # V4L2 Source
            source = Gst.ElementFactory.make("v4l2src", f"cam_{cam_name}")
            source.set_property('device', f'/dev/video{dev_idx}')
            
            # Caps filter
            caps_str = "video/x-raw,width=1920,height=1080,framerate=30/1" if i < 2 else \
                      "video/x-raw,width=640,height=480,framerate=30/1"  # backward
            caps = Gst.ElementFactory.make("capsfilter", f"caps_{cam_name}")
            caps.set_property('caps', Gst.Caps.from_string(caps_str))
            
            # NV Video Converter
            nvvidconv = Gst.ElementFactory.make("nvvidconv", f"nvvidconv_{cam_name}")
            
            # Queue
            queue = Gst.ElementFactory.make("queue", f"queue_{cam_name}")
            
            # Add to pipeline
            pipeline.add(source)
            pipeline.add(caps)
            pipeline.add(nvvidconv)
            pipeline.add(queue)
            
            # Link
            source.link(caps)
            caps.link(nvvidconv)
            nvvidconv.link(queue)
            
            sources.append(queue)
        
        # ============================================================
        # STREAM MUX
        # ============================================================
        
        streammux = Gst.ElementFactory.make("nvstreammux", "mux")
        streammux.set_property('width', 1920)
        streammux.set_property('height', 1080)
        streammux.set_property('batch-size', 3)
        streammux.set_property('batched-push-timeout', 40000)
        pipeline.add(streammux)
        
        # Link sources to mux
        for i, source_queue in enumerate(sources):
            sinkpad = streammux.get_request_pad(f"sink_{i}")
            srcpad = source_queue.get_static_pad("src")
            srcpad.link(sinkpad)
        
        # ============================================================
        # INFERENCE (TensorRT) - CONDITIONAL
        # ============================================================
        
        if self.enable_inference:
            # Primary Inference: HAZMAT Detection
            pgie_hazmat = Gst.ElementFactory.make("nvinfer", "pgie_hazmat")
            pgie_hazmat.set_property('config-file-path', 'configs/hazmat_infer_config.txt')
            pipeline.add(pgie_hazmat)
            
            # Secondary Inference: Landolt-C Detection  
            sgie_landolt = Gst.ElementFactory.make("nvinfer", "sgie_landolt")
            sgie_landolt.set_property('config-file-path', 'configs/landolt_infer_config.txt')
            pipeline.add(sgie_landolt)
            
            # Link: mux → hazmat → landolt
            streammux.link(pgie_hazmat)
            pgie_hazmat.link(sgie_landolt)
            
            last_element = sgie_landolt
        else:
            last_element = streammux
        
        # ============================================================
        # ON-SCREEN DISPLAY (Bounding Boxes)
        # ============================================================
        
        nvosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")
        pipeline.add(nvosd)
        last_element.link(nvosd)
        
        # ============================================================
        # ENCODING & OUTPUT
        # ============================================================
        
        # Video Converter
        nvvidconv_out = Gst.ElementFactory.make("nvvidconv", "nvvidconv_out")
        pipeline.add(nvvidconv_out)
        nvosd.link(nvvidconv_out)
        
        # H264 Encoder
        encoder = Gst.ElementFactory.make("nvv4l2h264enc", "encoder")
        encoder.set_property('bitrate', 4000000)
        pipeline.add(encoder)
        nvvidconv_out.link(encoder)
        
        # H264 Parser
        h264parser = Gst.ElementFactory.make("h264parse", "h264parser")
        pipeline.add(h264parser)
        encoder.link(h264parser)
        
        # RTSP/File Sink (for now, fakesink)
        sink = Gst.ElementFactory.make("fakesink", "sink")
        sink.set_property('sync', False)
        pipeline.add(sink)
        h264parser.link(sink)
        
        # ============================================================
        # PROBE for metadata extraction
        # ============================================================
        
        if self.enable_inference:
            osdsinkpad = nvosd.get_static_pad("sink")
            osdsinkpad.add_probe(Gst.PadProbeType.BUFFER, self.osd_sink_pad_buffer_probe, 0)
        
        # Bus watch
        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_bus_message)
        
        self.pipeline = pipeline
        return pipeline
    
    def osd_sink_pad_buffer_probe(self, pad, info, u_data):
        """
        Extract metadata (detections) from buffer
        Send via ZMQ
        """
        gst_buffer = info.get_buffer()
        if not gst_buffer:
            return Gst.PadProbeReturn.OK
        
        # Get batch metadata
        batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
        l_frame = batch_meta.frame_meta_list
        
        detections = []
        
        while l_frame is not None:
            try:
                frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
            except StopIteration:
                break
            
            l_obj = frame_meta.obj_meta_list
            while l_obj is not None:
                try:
                    obj_meta = pyds.NvDsObjectMeta.cast(l_obj.data)
                except StopIteration:
                    break
                
                # Extract detection info
                detection = {
                    'camera_id': frame_meta.source_id,
                    'class_id': obj_meta.class_id,
                    'confidence': obj_meta.confidence,
                    'bbox': {
                        'left': obj_meta.rect_params.left,
                        'top': obj_meta.rect_params.top,
                        'width': obj_meta.rect_params.width,
                        'height': obj_meta.rect_params.height
                    },
                    'label': obj_meta.obj_label
                }
                detections.append(detection)
                
                try:
                    l_obj = l_obj.next
                except StopIteration:
                    break
            
            try:
                l_frame = l_frame.next
            except StopIteration:
                break
        
        # Send via ZMQ
        if detections:
            self.zmq_socket.send_json({
                'timestamp': GLib.get_real_time(),
                'detections': detections
            })
        
        return Gst.PadProbeReturn.OK
    
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
                print(f"Pipeline: {old_state.value_nick} → {new_state.value_nick}")
    
    def start(self):
        """Start pipeline"""
        mode = "INFERENCE ENABLED" if self.enable_inference else "STREAMING ONLY"
        print(f"🚀 Starting pipeline: {mode}")
        print("📹 Cameras: front, arm, backward")
        print("🌐 ZMQ: tcp://*:5557")
        print("=" * 60)
        
        self.create_pipeline()
        
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("❌ Failed to start pipeline")
            sys.exit(1)
        
        try:
            self.loop = GLib.MainLoop()
            self.loop.run()
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted")
        finally:
            self.stop()
    
    def stop(self):
        """Stop pipeline"""
        print("🛑 Stopping...")
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if self.zmq_socket:
            self.zmq_socket.close()
        if self.zmq_context:
            self.zmq_context.term()


def main():
    import argparse
    parser = argparse.ArgumentParser(description='DeepStream Inference Streamer')
    parser.add_argument('--inference', action='store_true', 
                       help='Enable TensorRT inference (requires models)')
    args = parser.parse_args()
    
    print("""
    ╔══════════════════════════════════════════════════════════╗
    ║   RoboCup Rescue - DeepStream Inference Pipeline        ║
    ║   Stage 2: TensorRT Inference                           ║
    ╚══════════════════════════════════════════════════════════╝
    """)
    
    streamer = InferenceStreamer(enable_inference=args.inference)
    streamer.start()


if __name__ == '__main__':
    main()
