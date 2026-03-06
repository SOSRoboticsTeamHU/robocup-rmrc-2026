#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Arm Preset Positions
===============================================
Named presets for SO-ARM101: stowed, inspect_high/mid/low, cabinet_view, keypad_push, pick_ready.
Sends msgpack to followerarm.py (ZMQ 5558). Actual servo values must be calibrated on real robot;
see jetson/config/robot_params.yaml.
"""

import os
import sys
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:
    import msgpack
except ImportError:
    msgpack = None

try:
    from shared.constants import ZMQ_PORT_ARM_TELEOP
except ImportError:
    ZMQ_PORT_ARM_TELEOP = 5558

# Default joint values (radians or degrees per robot_params); override from YAML
PRESETS = {
    "stowed": [0.0, -0.5, 0.0, 0.0, 0.0],
    "inspect_high": [0.0, -0.3, 0.4, 0.0, 0.0],
    "inspect_mid": [0.0, -0.2, 0.6, 0.0, 0.0],
    "inspect_low": [0.0, 0.0, 0.8, 0.0, 0.0],
    "cabinet_view": [0.0, -0.25, 0.5, 0.2, 0.0],
    "keypad_push": [0.0, -0.1, 0.7, 0.1, 0.0],
    "pick_ready": [0.0, -0.2, 0.5, 0.3, 0.0],
}


def load_presets_from_yaml(path: str) -> dict:
    """Load arm presets from robot_params.yaml (key: arm.presets.<name>)."""
    if not os.path.isfile(path):
        return PRESETS
    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        arm = data.get("arm", {})
        p = arm.get("presets", {})
        for name, values in p.items():
            if isinstance(values, (list, tuple)) and len(values) >= 5:
                PRESETS[name] = list(values)[:5]
    except Exception:
        pass
    return PRESETS


def send_preset(preset_name: str, zmq_socket, presets: dict = None) -> bool:
    """
    Send a preset position to the arm via ZMQ (msgpack to followerarm).
    zmq_socket: ZMQ PUSH or PUB socket connected to Jetson ARM_TELEOP port.
    """
    if msgpack is None:
        return False
    config_path = os.path.join(os.path.dirname(__file__), "config", "robot_params.yaml")
    presets = presets or load_presets_from_yaml(config_path)
    if preset_name not in presets:
        return False
    joint_positions = presets[preset_name]
    # followerarm expects msgpack: joint names -> values (e.g. shoulder_pan, shoulder_lift, ...)
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist", "gripper"]
    msg = {name: joint_positions[i] for i, name in enumerate(joint_names) if i < len(joint_positions)}
    try:
        zmq_socket.send(msgpack.packb(msg))
        return True
    except Exception:
        return False


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("preset", choices=list(PRESETS.keys()), help="Preset name")
    ap.add_argument("--config", default=None, help="robot_params.yaml path")
    args = ap.parse_args()
    if args.config:
        load_presets_from_yaml(args.config)
    try:
        import zmq
        ctx = zmq.Context()
        sock = ctx.socket(zmq.PUSH)
        sock.connect(f"tcp://127.0.0.1:{ZMQ_PORT_ARM_TELEOP}")
        ok = send_preset(args.preset, sock)
        sock.close()
        ctx.term()
        print("OK" if ok else "FAIL")
    except Exception as e:
        print(f"Error: {e}")
