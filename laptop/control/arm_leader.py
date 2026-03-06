#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Arm Leader
====================================
Sends arm commands (ArmCommand / ArmTeleopData) to Jetson over ZMQ port 5556.
Can be driven by LeRobot leader hardware or by keyboard/test commands.
"""

import sys
import os
import time
import json
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

try:
    import zmq
except ImportError:
    print("pip install pyzmq")
    sys.exit(1)

try:
    from shared.constants import JETSON_IP, ZMQ_PORT_ARM
    from shared.protocol import ArmCommand, ArmTeleopData, asdict
except ImportError:
    JETSON_IP = "192.168.2.100"
    ZMQ_PORT_ARM = 5556
    ArmCommand = None
    ArmTeleopData = None


def run_sender(jetson_ip: str, port: int, rate_hz: float = 30):
    """Run ZMQ PUB sender; sends ArmTeleopData at rate_hz (default positions 0)."""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f"tcp://{jetson_ip}:{port}")

    # SO-ARM101 typical joint names (for reference)
    joint_names = [
        "shoulder_pan", "shoulder_lift", "elbow", "wrist_angle",
        "wrist_rotate", "gripper"
    ]
    positions = {j: 0.0 for j in joint_names}
    gripper = 0.0

    period = 1.0 / rate_hz
    print(f"[ARM_LEADER] Sending to {jetson_ip}:{port} at {rate_hz} Hz (Ctrl+C to stop)")

    try:
        while True:
            t0 = time.perf_counter()
            msg = {
                "msg_type": "arm_teleop",
                "timestamp": time.time(),
                "joint_positions": positions.copy(),
            }
            if ArmTeleopData:
                cmd = ArmTeleopData(joint_positions=positions)
                sock.send_json(asdict(cmd), zmq.NOBLOCK)
            else:
                sock.send_json(msg, zmq.NOBLOCK)
            elapsed = time.perf_counter() - t0
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\n[ARM_LEADER] Stopped")
    finally:
        sock.close()
        ctx.term()


def send_command_once(jetson_ip: str, port: int, positions: dict = None, gripper: float = 0):
    """Send a single ArmCommand (e.g. from script or LeRobot wrapper)."""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f"tcp://{jetson_ip}:{port}")
    positions = positions or {}
    if ArmCommand:
        cmd = ArmCommand(positions=positions, gripper=gripper)
        sock.send_json(asdict(cmd))
    else:
        sock.send_json({"msg_type": "arm_cmd", "positions": positions, "gripper": gripper})
    sock.close()
    ctx.term()


def main():
    ap = argparse.ArgumentParser(description="Arm leader - send arm commands to Jetson")
    ap.add_argument("--jetson", default=JETSON_IP, help="Jetson IP")
    ap.add_argument("--port", type=int, default=ZMQ_PORT_ARM, help="ZMQ port")
    ap.add_argument("--rate", type=float, default=30, help="Send rate Hz")
    ap.add_argument("--once", action="store_true", help="Send one command and exit (for testing)")
    args = ap.parse_args()

    if args.once:
        send_command_once(args.jetson, args.port)
        return
    run_sender(args.jetson, args.port, args.rate)


if __name__ == "__main__":
    main()
