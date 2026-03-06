#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Pre-Flight Check
===========================================
Verifies all subsystems before a mission run.
Run on Jetson before starting the robot.

Usage:
    python3 preflight_check.py
"""

import os
import sys
import time
import subprocess
import json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, REPO_ROOT)

try:
    from shared.constants import (
        CAMERAS, PICO_SERIAL_PORT, ARM_SERIAL_PORT,
        ZMQ_PORT_DRIVE, ZMQ_PORT_CAMERA, ZMQ_PORT_STATUS,
        ZMQ_PORT_SLAM, ZMQ_PORT_AUTONOMY,
        YOLO_MODEL_HAZMAT, YOLO_MODEL_LANDOLT,
    )
except ImportError:
    CAMERAS = {"front": "/dev/video2", "arm": "/dev/video4", "backward": "/dev/video0"}
    PICO_SERIAL_PORT = "/dev/ttyACM0"
    ARM_SERIAL_PORT = "/dev/ttyACM1"
    ZMQ_PORT_DRIVE = 5555
    ZMQ_PORT_CAMERA = 5557
    ZMQ_PORT_STATUS = 5559
    ZMQ_PORT_SLAM = 5562
    ZMQ_PORT_AUTONOMY = 5560
    YOLO_MODEL_HAZMAT = "models/hazmat_yolov8n.engine"
    YOLO_MODEL_LANDOLT = "models/landolt_yolov8n.engine"


PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"
WARN = "\033[93m[WARN]\033[0m"
INFO = "\033[94m[INFO]\033[0m"


def check_device(path: str, name: str) -> bool:
    exists = os.path.exists(path)
    status = PASS if exists else FAIL
    print(f"  {status} {name}: {path}")
    return exists


def check_cameras() -> int:
    print("\n--- Cameras ---")
    ok = 0
    for cam_id, dev_path in CAMERAS.items():
        if check_device(dev_path, f"Camera ({cam_id})"):
            ok += 1
    return ok


def check_serial() -> dict:
    print("\n--- Serial Devices ---")
    results = {}
    results["pico"] = check_device(PICO_SERIAL_PORT, "Pico (drive controller)")
    results["arm"] = check_device(ARM_SERIAL_PORT, "SO-ARM101 (servo bus)")
    return results


def check_lidar() -> bool:
    print("\n--- LiDAR ---")
    # Check if unilidar ROS2 topic is publishing
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True, text=True, timeout=5,
            env={**os.environ, "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0")}
        )
        topics = result.stdout.strip().split("\n") if result.returncode == 0 else []
        has_cloud = any("/unilidar/cloud" in t for t in topics)
        has_imu = any("/unilidar/imu" in t for t in topics)
        if has_cloud:
            print(f"  {PASS} LiDAR cloud topic active")
        else:
            print(f"  {WARN} LiDAR cloud topic not found (is unilidar_sdk2 running?)")
        if has_imu:
            print(f"  {PASS} LiDAR IMU topic active")
        return has_cloud
    except (subprocess.TimeoutExpired, FileNotFoundError):
        print(f"  {WARN} ROS2 not available or timed out")
        return False


def check_models() -> int:
    print("\n--- YOLO Models ---")
    ok = 0
    models_dir = os.path.join(SCRIPT_DIR, "vision")
    for model_rel in [YOLO_MODEL_HAZMAT, YOLO_MODEL_LANDOLT]:
        model_path = os.path.join(models_dir, model_rel)
        if not os.path.exists(model_path):
            model_path = os.path.join(SCRIPT_DIR, model_rel)
        exists = os.path.exists(model_path)
        size_mb = os.path.getsize(model_path) / (1024 * 1024) if exists else 0
        status = PASS if exists else WARN
        name = os.path.basename(model_rel)
        extra = f" ({size_mb:.1f} MB)" if exists else " (not found - will need re-export)"
        print(f"  {status} {name}{extra}")
        if exists:
            ok += 1
    return ok


def check_jetson_stats() -> None:
    print("\n--- Jetson System ---")
    # CPU temp
    try:
        with open("/sys/devices/virtual/thermal/thermal_zone0/temp", "r") as f:
            cpu_temp = int(f.read().strip()) / 1000.0
        status = PASS if cpu_temp < 75 else (WARN if cpu_temp < 85 else FAIL)
        print(f"  {status} CPU temp: {cpu_temp:.1f}°C")
    except Exception:
        print(f"  {WARN} CPU temp: unable to read")

    # GPU temp
    try:
        with open("/sys/devices/virtual/thermal/thermal_zone1/temp", "r") as f:
            gpu_temp = int(f.read().strip()) / 1000.0
        status = PASS if gpu_temp < 75 else (WARN if gpu_temp < 85 else FAIL)
        print(f"  {status} GPU temp: {gpu_temp:.1f}°C")
    except Exception:
        pass

    # Disk space
    try:
        stat = os.statvfs("/")
        free_gb = (stat.f_bavail * stat.f_frsize) / (1024 ** 3)
        status = PASS if free_gb > 2 else (WARN if free_gb > 0.5 else FAIL)
        print(f"  {status} Disk free: {free_gb:.1f} GB")
    except Exception:
        pass

    # Memory
    try:
        with open("/proc/meminfo", "r") as f:
            lines = f.readlines()
        mem_total = mem_avail = 0
        for line in lines:
            if line.startswith("MemTotal:"):
                mem_total = int(line.split()[1]) / 1024  # MB
            elif line.startswith("MemAvailable:"):
                mem_avail = int(line.split()[1]) / 1024  # MB
        if mem_total > 0:
            pct_used = 100 * (1 - mem_avail / mem_total)
            status = PASS if pct_used < 80 else (WARN if pct_used < 90 else FAIL)
            print(f"  {status} Memory: {mem_avail:.0f}/{mem_total:.0f} MB free ({pct_used:.0f}% used)")
    except Exception:
        pass


def check_network() -> bool:
    print("\n--- Network ---")
    # Check if Ethernet interface is up
    try:
        result = subprocess.run(
            ["ip", "addr", "show"],
            capture_output=True, text=True, timeout=3
        )
        has_192_168_2 = "192.168.2." in result.stdout
        status = PASS if has_192_168_2 else FAIL
        print(f"  {status} Ethernet tether (192.168.2.x)")
        return has_192_168_2
    except Exception:
        print(f"  {FAIL} Could not check network")
        return False


def check_ports() -> int:
    print("\n--- ZMQ Ports (conflict check) ---")
    conflicts = 0
    ports = {
        "Drive": ZMQ_PORT_DRIVE,
        "Camera": ZMQ_PORT_CAMERA,
        "Status": ZMQ_PORT_STATUS,
        "SLAM": ZMQ_PORT_SLAM,
        "Autonomy": ZMQ_PORT_AUTONOMY,
    }
    try:
        result = subprocess.run(
            ["ss", "-tlnp"],
            capture_output=True, text=True, timeout=3
        )
        for name, port in ports.items():
            in_use = f":{port}" in result.stdout
            if in_use:
                print(f"  {WARN} Port {port} ({name}) already in use")
                conflicts += 1
            else:
                print(f"  {PASS} Port {port} ({name}) available")
    except Exception:
        print(f"  {INFO} Could not check ports (ss not available)")
    return conflicts


def main():
    print("=" * 55)
    print("  RoboCupRescue RMRC 2026 - Pre-Flight Check")
    print("=" * 55)

    issues = 0

    cam_ok = check_cameras()
    if cam_ok < len(CAMERAS):
        issues += len(CAMERAS) - cam_ok

    serial = check_serial()
    if not serial["pico"]:
        issues += 1
    if not serial["arm"]:
        issues += 1

    check_lidar()
    models_ok = check_models()
    check_jetson_stats()
    net_ok = check_network()
    if not net_ok:
        issues += 1
    port_conflicts = check_ports()
    issues += port_conflicts

    print("\n" + "=" * 55)
    if issues == 0:
        print(f"  {PASS} ALL CHECKS PASSED - Ready for mission!")
    else:
        print(f"  {WARN} {issues} issue(s) found - review above")
    print("=" * 55)
    return 0 if issues == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
