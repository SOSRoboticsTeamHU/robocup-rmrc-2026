#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Leader Arm (LeRobot)
==============================================
Reads joint positions from the physical leader arm and sends them to the Jetson
using the same protocol as the working reference (Rescue_Client/leaderarm.py).

Protocol (must match jetson/followerarm.py):
- ZMQ: laptop PUB -> connect to jetson:5558; Jetson SUB -> bind 5558.
- Payload: msgpack {'pos': [v1, v2, v3, v4, v5, v6]} with values -100..100 (LeRobot normalized).
- Order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.
"""

import sys
import os
import time
import json
import argparse
from pathlib import Path
from typing import Optional, Dict, List

# Project paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "shared"))

try:
    import zmq
except ImportError:
    zmq = None

try:
    import msgpack
except ImportError:
    msgpack = None

try:
    from shared.constants import (
        JETSON_IP,
        ZMQ_PORT_ARM_TELEOP,
        ARM_LEADER_SERIAL,
        ARM_LEADER_RATE_HZ,
    )
except ImportError:
    JETSON_IP = "192.168.2.100"
    ZMQ_PORT_ARM_TELEOP = 5558
    ARM_LEADER_SERIAL = "/dev/tty.usbmodem5AE60833341"
    ARM_LEADER_RATE_HZ = 100

# Same order as reference leaderarm.py (calibration file)
MOTOR_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

DEFAULT_CALIBRATION_FILE = Path.home() / ".cache/huggingface/lerobot/calibration/teleoperators/so_leader/leaderarm.json"


def try_import_lerobot():
    try:
        lerobot_src = os.environ.get("LEROBOT_SRC", "")
        if lerobot_src not in sys.path:
            sys.path.insert(0, lerobot_src)
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors import Motor, MotorNormMode, MotorCalibration
        return FeetechMotorsBus, Motor, MotorNormMode, MotorCalibration
    except Exception:
        return None, None, None, None


def build_lerobot_bus(port: str, calibration_path: Path):
    """Build FeetechMotorsBus from calibration file (same as reference)."""
    FeetechMotorsBus, Motor, MotorNormMode, MotorCalibration = try_import_lerobot()
    if FeetechMotorsBus is None or Motor is None:
        return None, MOTOR_NAMES
    if not calibration_path.exists():
        return None, MOTOR_NAMES
    try:
        with open(calibration_path) as f:
            calib_data = json.load(f)
    except Exception:
        return None, MOTOR_NAMES

    motor_model = "sts3215"
    motors = {}
    calibration = {}
    for name in MOTOR_NAMES:
        if name not in calib_data:
            return None, MOTOR_NAMES
        cal = calib_data[name]
        motors[name] = Motor(id=cal["id"], model=motor_model, norm_mode=MotorNormMode.RANGE_M100_100)
        calibration[name] = MotorCalibration(
            id=cal["id"],
            drive_mode=cal.get("drive_mode", 0),
            homing_offset=cal.get("homing_offset", 0),
            range_min=cal.get("range_min", -100),
            range_max=cal.get("range_max", 100),
        )

    try:
        bus = FeetechMotorsBus(port=port, motors=motors, calibration=calibration)
        bus.connect()
        return bus, MOTOR_NAMES
    except Exception:
        return None, MOTOR_NAMES


def run_leader_arm(
    jetson_ip: str = JETSON_IP,
    port: int = ZMQ_PORT_ARM_TELEOP,
    serial_port: str = ARM_LEADER_SERIAL,
    calibration_file: Optional[Path] = None,
    rate_hz: float = ARM_LEADER_RATE_HZ,
    no_lerobot: bool = False,
    debug: bool = False,
) -> None:
    """
    Same protocol as reference leaderarm.py:
    - Read positions via LeRobot sync_read("Present_Position") -> -100..100 per joint.
    - Send msgpack {'pos': [v1..v6]} to jetson_ip:port (5558).
    """
    if zmq is None:
        print("[LEADER_ARM] pyzmq required")
        return
    if msgpack is None:
        print("[LEADER_ARM] msgpack required (pip install msgpack)")
        return

    calibration_path = calibration_file or DEFAULT_CALIBRATION_FILE
    bus = None
    motor_names = MOTOR_NAMES

    if not no_lerobot:
        bus, motor_names = build_lerobot_bus(serial_port, calibration_path)
        if bus is not None:
            print(f"[LEADER_ARM] LeRobot bus connected on {serial_port}")
        else:
            print("[LEADER_ARM] LeRobot not available or calibration missing; exiting (use --no-lerobot to send zeros)")
            if not no_lerobot:
                return

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.setsockopt(zmq.LINGER, 0)
    addr = f"tcp://{jetson_ip}:{port}"
    sock.connect(addr)
    period = 1.0 / rate_hz
    print(f"[LEADER_ARM] Sending msgpack {{'pos': [6 values]}} to {addr} at {rate_hz} Hz")

    # When no hardware: send neutral (0) so follower doesn't jump
    neutral_list = [0] * 6

    try:
        while True:
            t0 = time.perf_counter()
            pos_list = neutral_list

            if bus is not None:
                try:
                    positions = bus.sync_read("Present_Position")
                    if positions is not None:
                        pos_list = [int(positions[name]) for name in motor_names]
                except Exception as e:
                    if debug:
                        print(f"[LEADER_ARM] read error: {e}")

            payload = {"pos": pos_list}
            try:
                sock.send(msgpack.packb(payload), zmq.NOBLOCK)
            except zmq.Again:
                pass

            if debug and bus is not None:
                print(f"[LEADER_ARM] {pos_list}")

            elapsed = time.perf_counter() - t0
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[LEADER_ARM] Stopped")
    finally:
        if bus is not None:
            try:
                bus.disconnect()
            except Exception:
                pass
        sock.close()
        ctx.term()


def main():
    ap = argparse.ArgumentParser(
        description="Leader arm: LeRobot SO-ARM leader -> msgpack to Jetson (same as reference leaderarm.py)"
    )
    ap.add_argument("--jetson", default=JETSON_IP, help="Jetson IP")
    ap.add_argument("--port", type=int, default=ZMQ_PORT_ARM_TELEOP, help="ZMQ port (follower binds this)")
    ap.add_argument("--serial", default=ARM_LEADER_SERIAL, help="Leader arm serial port")
    ap.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION_FILE, help="leaderarm.json path")
    ap.add_argument("--rate", type=float, default=ARM_LEADER_RATE_HZ, help="Send rate Hz")
    ap.add_argument("--no-lerobot", action="store_true", help="Send neutral positions (no hardware)")
    ap.add_argument("--debug", action="store_true", help="Print positions")
    args = ap.parse_args()

    run_leader_arm(
        jetson_ip=args.jetson,
        port=args.port,
        serial_port=args.serial,
        calibration_file=args.calibration,
        rate_hz=args.rate,
        no_lerobot=args.no_lerobot,
        debug=args.debug,
    )


if __name__ == "__main__":
    main()
