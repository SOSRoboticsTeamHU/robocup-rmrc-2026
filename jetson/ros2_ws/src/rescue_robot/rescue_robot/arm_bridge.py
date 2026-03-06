#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Arm Bridge
====================================
Receives arm commands from laptop (ZMQ port 5556), forwards to SO-ARM101 via serial.
Uses Feetech SCS serial protocol: WRITE instruction, GOAL_POSITION at 0x2E (0-1023).
Servo IDs: 1-6 = joints, 7 = gripper or 1 DOF back support (kickstand). When arm_cmd
has empty positions and only gripper set, only servo 7 is written (e.g. joystick buttons 7/8).
"""

import sys
import os
import time
import json
import argparse
from typing import Optional, Dict, List

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))

try:
    import zmq
except ImportError:
    zmq = None

try:
    import serial
except ImportError:
    serial = None

try:
    from shared.constants import ZMQ_PORT_ARM, ARM_SERIAL_PORT
except ImportError:
    ZMQ_PORT_ARM = 5556
    ARM_SERIAL_PORT = "/dev/ttyUSB0"

# Feetech SCS protocol constants
SCS_HEADER = bytes([0xFF, 0xFF])
SCS_WRITE = 0x03
SCS_GOAL_POSITION_ADDR = 0x2E  # 2-byte register, 0-1023


def scs_checksum(packet: bytes) -> int:
    """Checksum = ~(sum of ID through last param) & 0xFF."""
    return (~sum(packet[2:]) & 0xFF)


def scs_write_goal_position(servo_id: int, position: int) -> bytes:
    """Build SCS WRITE packet for GOAL_POSITION. Position clamped to 0-1023."""
    pos = max(0, min(1023, int(position)))
    # SCS: address 0x2E, 2 bytes (low, high)
    addr_lo, addr_hi = 0x2E, 0x00
    val_lo, val_hi = pos & 0xFF, (pos >> 8) & 0xFF
    length = 4  # 2 addr + 2 data
    body = bytes([servo_id & 0xFF, length + 2, SCS_WRITE, addr_lo, addr_hi, val_lo, val_hi])
    checksum = scs_checksum(body)
    return SCS_HEADER + body + bytes([checksum])


def positions_to_scs_packets(
    positions: Dict,
    gripper: int,
    servo_ids: List[int],
) -> List[bytes]:
    """
    Map positions dict and gripper to SCS packets.
    positions: keys "j1".."j6" or 1..6, values 0-1023 (or 0.0-1.0 normalized).
    servo_ids: [id_j1, id_j2, ..., id_j6, id_gripper] (length 7).
    """
    if len(servo_ids) < 7:
        servo_ids = (servo_ids + [1, 2, 3, 4, 5, 6, 7])[:7]
    packets = []
    for i in range(6):
        val = positions.get(f"j{i + 1}", positions.get(i + 1, positions.get(str(i + 1))))
        if val is None:
            continue
        pos = int(val * 1023) if isinstance(val, float) and 0 <= val <= 1 else int(val)
        packets.append(scs_write_goal_position(servo_ids[i], pos))
    gripper_pos = int(gripper * 1023) if isinstance(gripper, float) and 0 <= gripper <= 1 else int(gripper)
    packets.append(scs_write_goal_position(servo_ids[6], gripper_pos))
    return packets


def main():
    ap = argparse.ArgumentParser(description="Arm bridge: ZMQ 5556 -> SO-ARM101 serial (Feetech SCS)")
    ap.add_argument("--port", type=int, default=ZMQ_PORT_ARM, help="ZMQ port to bind")
    ap.add_argument("--serial-port", type=str, default=ARM_SERIAL_PORT, help="Serial port for arm")
    ap.add_argument("--no-serial", action="store_true", help="Only log, do not open serial")
    ap.add_argument("--servo-ids", type=str, default="1,2,3,4,5,6,7",
                    help="Comma-separated servo IDs: j1,j2,...,j6,gripper (default: 1,2,3,4,5,6,7)")
    ap.add_argument("--debug", action="store_true", help="Print every command")
    args = ap.parse_args()

    servo_ids = [int(x.strip()) for x in args.servo_ids.split(",") if x.strip()]
    if len(servo_ids) < 7:
        servo_ids = (servo_ids + [1, 2, 3, 4, 5, 6, 7])[:7]

    if not zmq:
        print("[ARM_BRIDGE] pyzmq required")
        return

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.RCVHWM, 1)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.RCVTIMEO, 100)
    sock.bind(f"tcp://*:{args.port}")

    ser = None
    if serial and not args.no_serial:
        try:
            ser = serial.Serial(args.serial_port, 1000000, timeout=0.01)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print(f"[ARM_BRIDGE] Serial open: {args.serial_port} (Feetech SCS)")
        except Exception as e:
            print(f"[ARM_BRIDGE] Serial failed: {e} (running without serial)")

    print(f"[ARM_BRIDGE] Listening on port {args.port}, servo IDs: {servo_ids}")

    try:
        while True:
            try:
                msg = sock.recv_json()
                msg_type = msg.get("msg_type", "")
                if args.debug:
                    print(f"[ARM_BRIDGE] {msg_type}: {msg}")

                if msg_type == "arm_cmd":
                    positions = msg.get("positions") or {}
                    gripper = msg.get("gripper", 0)
                    packets = positions_to_scs_packets(positions, gripper, servo_ids)
                    if ser and ser.is_open:
                        for pkt in packets:
                            ser.write(pkt)
                        ser.flush()
                elif msg_type == "arm_teleop":
                    joint_positions = msg.get("joint_positions") or {}
                    # Map j1..j6; gripper from joint_positions or default 0
                    positions = {k: v for k, v in joint_positions.items() if str(k).startswith(("j", "1", "2", "3", "4", "5", "6"))}
                    gripper = joint_positions.get("gripper", joint_positions.get(7, 0))
                    packets = positions_to_scs_packets(positions, gripper, servo_ids)
                    if ser and ser.is_open:
                        for pkt in packets:
                            ser.write(pkt)
                        ser.flush()
            except zmq.Again:
                pass
            except Exception as e:
                print(f"[ARM_BRIDGE] Error: {e}")
    except KeyboardInterrupt:
        print("\n[ARM_BRIDGE] Stopped")
    finally:
        if ser and ser.is_open:
            ser.close()
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
