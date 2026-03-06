#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Follower Arm (LeRobot)
================================================
Receives arm positions from the laptop leader via ZMQ (msgpack) and writes
them to the follower arm using LeRobot FeetechMotorsBus.

Protocol (must match laptop/control/leader_arm.py and reference leaderarm.py):
- ZMQ: Jetson SUB bind *:5558; laptop PUB connects to Jetson:5558.
- Payload: msgpack {'pos': [v1, v2, v3, v4, v5, v6]} with values -100..100.
- Order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
  -> mapped to servo_1..servo_6.

Also listens on port 5556 for arm_cmd (JSON) from GUI — back support (servo 7)
 via joystick buttons 7/8. Servo 7 = 1 DOF kickstand on same Feetech bus.
"""

import sys
import os
import argparse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, PROJECT_DIR)
sys.path.insert(0, os.path.join(PROJECT_DIR, "..", "shared"))

try:
    import zmq
except ImportError:
    zmq = None

try:
    import msgpack
except ImportError:
    msgpack = None

try:
    from shared.constants import ZMQ_PORT_ARM_TELEOP, ZMQ_PORT_ARM, ARM_SERIAL_PORT
except ImportError:
    ZMQ_PORT_ARM_TELEOP = 5558
    ZMQ_PORT_ARM = 5556
    ARM_SERIAL_PORT = "/dev/ttyACM1"

DEFAULT_FOLLOWER_PORT = ARM_SERIAL_PORT
SERVO_IDS = [1, 2, 3, 4, 5, 6]
BACK_SUPPORT_SERVO_ID = 7  # 1 DOF kickstand on same bus

# Safe mechanical range for back support (servo 7), in raw units
BACK_SUPPORT_MIN_RAW = 150
BACK_SUPPORT_MAX_RAW = 1800


def raw_to_lerobot_norm(pos_raw: float) -> float:
    """
    Convert back-support raw range [150..1820] to LeRobot -100..100 for RANGE_M100_100.
    Values outside the range are clamped for safety.
    """
    # Clamp to safe range
    if pos_raw < BACK_SUPPORT_MIN_RAW:
        pos = BACK_SUPPORT_MIN_RAW
    elif pos_raw > BACK_SUPPORT_MAX_RAW:
        pos = BACK_SUPPORT_MAX_RAW
    else:
        pos = float(pos_raw)
    span = BACK_SUPPORT_MAX_RAW - BACK_SUPPORT_MIN_RAW
    if span <= 0:
        return 0.0
    # Map [min,max] -> [0,1] -> [-100,100]
    norm01 = (pos - BACK_SUPPORT_MIN_RAW) / span
    return (norm01 - 0.5) * 200.0


def try_import_lerobot():
    try:
        lerobot_src = os.environ.get("LEROBOT_SRC", "/usr/local/lerobot/src")
        if lerobot_src not in sys.path:
            sys.path.insert(0, lerobot_src)
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors import Motor, MotorNormMode
        return FeetechMotorsBus, Motor, MotorNormMode
    except Exception:
        return None, None, None


def main():
    ap = argparse.ArgumentParser(
        description="Follower arm: leader teleop (5558) + back support (5556) -> LeRobot bus"
    )
    ap.add_argument("--port", type=int, default=ZMQ_PORT_ARM_TELEOP, help="ZMQ port for leader teleop")
    ap.add_argument("--arm-port", type=int, default=ZMQ_PORT_ARM, help="ZMQ port for arm_cmd (back support)")
    ap.add_argument("--serial", type=str, default=DEFAULT_FOLLOWER_PORT, help="Follower arm serial port")
    args = ap.parse_args()

    if not zmq:
        print("[FOLLOWER_ARM] pyzmq required")
        return
    if not msgpack:
        print("[FOLLOWER_ARM] msgpack required (pip install msgpack)")
        return

    FeetechMotorsBus, Motor, MotorNormMode = try_import_lerobot()
    if FeetechMotorsBus is None:
        print("[FOLLOWER_ARM] LeRobot not available (set LEROBOT_SRC or install lerobot)")
        return

    # servo_1..servo_6 = arm joints, servo_7 = back support (kickstand)
    all_ids = list(SERVO_IDS) + [BACK_SUPPORT_SERVO_ID]
    motors = {
        f"servo_{id_}": Motor(id_, "sts3215", MotorNormMode.RANGE_M100_100)
        for id_ in all_ids
    }
    bus = FeetechMotorsBus(port=args.serial, motors=motors)
    bus.connect()
    bus.calibration = bus.read_calibration()

    ctx = zmq.Context()
    # 5558: leader PUB -> Jetson SUB (msgpack teleop)
    sock_teleop = ctx.socket(zmq.SUB)
    sock_teleop.setsockopt(zmq.CONFLATE, 1)
    sock_teleop.bind(f"tcp://*:{args.port}")
    sock_teleop.setsockopt_string(zmq.SUBSCRIBE, "")
    sock_teleop.setsockopt(zmq.RCVTIMEO, 50)

    # 5556: GUI PUSH -> Jetson PULL (arm_cmd, back support from joystick buttons 7/8)
    sock_arm = ctx.socket(zmq.PULL)
    sock_arm.setsockopt(zmq.RCVHWM, 1)
    sock_arm.bind(f"tcp://*:{args.arm_port}")
    sock_arm.setsockopt(zmq.RCVTIMEO, 50)

    poller = zmq.Poller()
    poller.register(sock_teleop, zmq.POLLIN)
    poller.register(sock_arm, zmq.POLLIN)

    # Start stowed near mechanical minimum
    last_back_support_norm = raw_to_lerobot_norm(BACK_SUPPORT_MIN_RAW)

    print(f"[FOLLOWER_ARM] Listening on tcp://*:{args.port} (teleop) + tcp://*:{args.arm_port} (back support), serial {args.serial}")

    try:
        while True:
            for sock, _ in poller.poll(timeout=100):
                if sock == sock_teleop:
                    try:
                        msg = sock_teleop.recv()
                    except zmq.Again:
                        continue
                    try:
                        data = msgpack.unpackb(msg)
                    except Exception:
                        continue
                    pos_list = data.get("pos") if isinstance(data, dict) else None
                    if not pos_list or len(pos_list) < len(SERVO_IDS):
                        continue
                    # pos list order = servo_1..servo_6
                    positions = {f"servo_{id_}": pos for id_, pos in zip(SERVO_IDS, pos_list[: len(SERVO_IDS)])}
                    # Include servo_7 (back support) so we don't overwrite it with stale value
                    positions["servo_7"] = last_back_support_norm
                    bus.sync_write("Goal_Position", positions)

                elif sock == sock_arm:
                    try:
                        msg = sock_arm.recv_json()
                    except zmq.Again:
                        continue
                    if msg.get("msg_type") == "arm_cmd":
                        gripper = msg.get("gripper", 0)
                        last_back_support_norm = raw_to_lerobot_norm(gripper)
                        bus.sync_write("Goal_Position", {"servo_7": last_back_support_norm})
    except KeyboardInterrupt:
        print("\n[FOLLOWER_ARM] Stopped")
    finally:
        try:
            bus.disconnect()
        except Exception:
            pass
        sock_teleop.close()
        sock_arm.close()
        ctx.term()


if __name__ == "__main__":
    main()
