#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Arm Preset Calibration Tool
=====================================================
Interactive tool: move the arm manually via the leader arm, then save
the current joint positions as a named preset.

Usage:
  python3 laptop/control/arm_calibrate.py
  python3 laptop/control/arm_calibrate.py --jetson-ip 192.168.2.100

Workflow:
  1. Move the arm to the desired position using the leader arm.
  2. Press Enter to read current positions from the follower arm bus.
  3. Type a preset name (e.g. "inspect_high") and press Enter to save.
  4. Presets are saved to jetson/config/arm_presets_custom.yaml.
  5. arm_presets.py loads these automatically as overrides.
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

try:
    import zmq
except ImportError:
    print("ERROR: pyzmq required. Run: pip install pyzmq")
    sys.exit(1)

try:
    import yaml
except ImportError:
    yaml = None
    print("WARNING: pyyaml not installed. Will use fallback YAML writer.")

try:
    from shared.constants import JETSON_IP, ZMQ_PORT_ARM_TELEOP
except ImportError:
    JETSON_IP = "192.168.2.100"
    ZMQ_PORT_ARM_TELEOP = 5558

# Default presets file path (relative to project root -> jetson/config/)
PRESETS_FILE = os.path.join(
    os.path.dirname(__file__), "..", "..", "jetson", "config", "arm_presets_custom.yaml"
)

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow", "wrist", "gripper"]


def read_current_positions_from_leader(jetson_ip: str, port: int, timeout_ms: int = 2000):
    """
    Read the last leader arm positions echoed back on the teleop channel.
    The leader_arm.py PUBs msgpack with {'pos': [v1..v6]} on port 5558.
    We subscribe briefly to capture the latest frame.
    """
    try:
        import msgpack
    except ImportError:
        print("ERROR: msgpack required. Run: pip install msgpack")
        return None

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.connect(f"tcp://{jetson_ip}:{port}")

    # Wait a moment for subscription to propagate
    time.sleep(0.3)

    positions = None
    try:
        raw = sock.recv()
        data = msgpack.unpackb(raw)
        if isinstance(data, dict) and "pos" in data:
            positions = list(data["pos"])[:5]
        elif isinstance(data, dict):
            # Might be {joint_name: value} format
            positions = [data.get(name, 0.0) for name in JOINT_NAMES]
    except zmq.Again:
        print("No arm data received (is the leader arm running and connected?)")
    except Exception as e:
        print(f"Error reading arm positions: {e}")
    finally:
        sock.close()
        ctx.term()

    return positions


def load_existing_presets(path: str) -> dict:
    """Load existing custom presets from YAML."""
    if not os.path.isfile(path):
        return {}
    try:
        if yaml:
            with open(path, "r") as f:
                data = yaml.safe_load(f) or {}
            return data.get("arm", {}).get("presets", {})
        else:
            # Minimal YAML parser for simple key-value
            return {}
    except Exception:
        return {}


def save_presets(path: str, presets: dict):
    """Save presets to YAML file."""
    os.makedirs(os.path.dirname(path), exist_ok=True)

    data = {"arm": {"presets": presets}}

    if yaml:
        with open(path, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    else:
        # Fallback: write simple YAML manually
        with open(path, "w") as f:
            f.write("arm:\n  presets:\n")
            for name, values in presets.items():
                vals_str = ", ".join(f"{v:.4f}" for v in values)
                f.write(f"    {name}: [{vals_str}]\n")

    print(f"Saved to {os.path.abspath(path)}")


def main():
    ap = argparse.ArgumentParser(description="Arm Preset Calibration Tool")
    ap.add_argument("--jetson-ip", type=str, default=JETSON_IP, help="Jetson IP address")
    ap.add_argument("--port", type=int, default=ZMQ_PORT_ARM_TELEOP, help="ZMQ arm teleop port")
    ap.add_argument("--output", type=str, default=PRESETS_FILE, help="Output YAML file")
    ap.add_argument("--list", action="store_true", help="List existing custom presets and exit")
    args = ap.parse_args()

    presets = load_existing_presets(args.output)

    if args.list:
        if not presets:
            print("No custom presets found.")
        else:
            print("Custom presets:")
            for name, values in presets.items():
                vals = ", ".join(f"{v:.3f}" for v in values)
                print(f"  {name}: [{vals}]")
        return

    print("=" * 50)
    print("  Arm Preset Calibration Tool")
    print("=" * 50)
    print(f"Jetson: {args.jetson_ip}:{args.port}")
    print(f"Output: {args.output}")
    print()
    print("Instructions:")
    print("  1. Move the arm to desired position using the leader arm")
    print("  2. Press Enter to capture current joint positions")
    print("  3. Type a preset name to save (or 'skip' to discard)")
    print("  4. Type 'quit' or Ctrl+C to exit")
    print()

    if presets:
        print(f"Existing custom presets: {', '.join(presets.keys())}")
        print()

    try:
        while True:
            input("Press Enter when arm is in desired position...")

            print("Reading arm positions...")
            positions = read_current_positions_from_leader(args.jetson_ip, args.port)

            if positions is None:
                print("Failed to read positions. Make sure leader_arm.py is running.")
                continue

            print(f"Current positions: {[f'{v:.3f}' for v in positions]}")
            for i, name in enumerate(JOINT_NAMES):
                if i < len(positions):
                    print(f"  {name}: {positions[i]:.4f}")

            name = input("\nPreset name (or 'skip'/'quit'): ").strip()

            if name.lower() == "quit":
                break
            if name.lower() == "skip" or not name:
                print("Skipped.\n")
                continue

            # Confirm overwrite if exists
            if name in presets:
                old = [f"{v:.3f}" for v in presets[name]]
                confirm = input(f"  '{name}' exists ({old}). Overwrite? [y/N]: ").strip()
                if confirm.lower() != "y":
                    print("Skipped.\n")
                    continue

            presets[name] = positions
            save_presets(args.output, presets)
            print(f"Saved preset '{name}'\n")

    except KeyboardInterrupt:
        print("\n")

    if presets:
        save_presets(args.output, presets)
        print(f"\nAll presets saved to {os.path.abspath(args.output)}")
    print("Done.")


if __name__ == "__main__":
    main()
