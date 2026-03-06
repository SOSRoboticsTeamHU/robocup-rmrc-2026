#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Pi Camera Viewer
==========================================
Simple viewer to display camera stream from Raspberry Pi.

Usage:
    python pi_camera_viewer.py                    # Connect to Pi at default IP
    python pi_camera_viewer.py --ip 192.168.2.102 # Specify Pi IP
    
Press 'q' to quit.
"""

import cv2
import zmq
import json
import time
import argparse
import numpy as np


# Pi IP address
PI_IP = "192.168.2.102"
PI_CAMERA_PORT = 5557


def main():
    parser = argparse.ArgumentParser(description='Pi Camera Viewer')
    parser.add_argument('--ip', type=str, default=PI_IP,
                        help=f'Pi IP address (default: {PI_IP})')
    parser.add_argument('--port', type=int, default=PI_CAMERA_PORT,
                        help=f'ZMQ port (default: {PI_CAMERA_PORT})')
    args = parser.parse_args()
    
    # Setup ZMQ subscriber
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    socket.setsockopt(zmq.RCVTIMEO, 2000)  # 2 second timeout
    socket.setsockopt(zmq.RCVHWM, 2)       # Only buffer 2 frames
    
    connect_str = f"tcp://{args.ip}:{args.port}"
    print(f"[VIEWER] Connecting to {connect_str}...")
    socket.connect(connect_str)
    
    print("[VIEWER] Waiting for frames... Press 'q' to quit")
    
    frame_count = 0
    fps_start = time.time()
    display_fps = 0.0
    
    try:
        while True:
            try:
                # Receive multipart message [header_json, jpeg_bytes]
                msg_parts = socket.recv_multipart()
                
                if len(msg_parts) != 2:
                    continue
                
                header = json.loads(msg_parts[0].decode('utf-8'))
                jpeg_data = msg_parts[1]
                
                # Decode JPEG
                frame = cv2.imdecode(
                    np.frombuffer(jpeg_data, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )
                
                if frame is None:
                    continue
                
                frame_count += 1
                
                # Calculate display FPS
                elapsed = time.time() - fps_start
                if elapsed >= 1.0:
                    display_fps = frame_count / elapsed
                    frame_count = 0
                    fps_start = time.time()
                
                # Get info from header
                camera_id = header.get("camera_id", "unknown")
                source_fps = header.get("fps", 0)
                
                # Draw info overlay
                info_text = f"{camera_id} | Source: {source_fps:.1f} FPS | Display: {display_fps:.1f} FPS"
                cv2.putText(frame, info_text, (10, 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Show frame
                cv2.imshow(f"Pi Camera - {args.ip}", frame)
                
                # Check for quit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                    
            except zmq.Again:
                # Timeout - no data
                print(f"[VIEWER] No data from {args.ip}:{args.port}, waiting...")
                
    except KeyboardInterrupt:
        print("\n[VIEWER] Interrupted")
    finally:
        cv2.destroyAllWindows()
        socket.close()
        context.term()
        print("[VIEWER] Stopped")


if __name__ == "__main__":
    main()
