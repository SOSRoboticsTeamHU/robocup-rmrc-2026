#!/usr/bin/env python3
"""
SLAM Simulator — Test 2D Map + 3D Point Cloud views without a robot.

Publishes fake SLAM data (pose + occupancy grid + path + Nav2 overlay) on ZMQ
port 5562, and fake point cloud data on port 5564 — identical to what
slam_bridge.py sends on the real robot.

The simulated robot drives in a figure-8 pattern through a generated arena.

Usage:
    python tools/slam_simulator.py                   # defaults (localhost)
    python tools/slam_simulator.py --nav2            # also publish fake Nav2 overlay
    python tools/slam_simulator.py --no-cloud        # skip point cloud generation

Then run the GUI (in another terminal):
    cd laptop && python main.py --jetson-ip 127.0.0.1 --no-joystick --no-arm

The 2D Map tab shows occupancy grid + robot + path.
The 3D RViz tab shows the point cloud + grid floor + path.
"""

import argparse
import json
import math
import random
import sys
import time

try:
    import zmq
except ImportError:
    print("ERROR: pip install pyzmq")
    sys.exit(1)

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False


def generate_arena(size=200, resolution=0.05):
    """Generate a realistic RMRC-style arena occupancy grid.

    Returns grid[row][col]: -1=unknown, 0=free, 100=occupied.
    Origin is at (-size*resolution/2, -size*resolution/2).
    """
    grid = [[-1 for _ in range(size)] for _ in range(size)]
    half = size // 2

    # Clear a large area in the center (free space)
    for i in range(20, size - 20):
        for j in range(20, size - 20):
            grid[i][j] = 0  # free

    # Outer walls
    for i in range(size):
        for j in range(size):
            if i < 22 or i > size - 23 or j < 22 or j > size - 23:
                if 20 <= i < size - 20 and 20 <= j < size - 20:
                    grid[i][j] = 100  # wall

    # Inner obstacles (boxes, L-shapes)
    # Box 1
    for i in range(60, 80):
        for j in range(60, 75):
            grid[i][j] = 100
    # Box 2
    for i in range(120, 140):
        for j in range(130, 145):
            grid[i][j] = 100
    # L-shape
    for i in range(90, 110):
        for j in range(40, 50):
            grid[i][j] = 100
    for i in range(100, 110):
        for j in range(50, 70):
            grid[i][j] = 100
    # Corridor walls
    for i in range(50, 150):
        if 95 <= i <= 105:
            continue  # gap
        grid[i][100] = 100
        grid[i][101] = 100

    return grid


def figure_8_pose(t, scale=2.0):
    """Generate a figure-8 trajectory. Returns (x, y, yaw)."""
    # Lemniscate of Bernoulli parametrized by t
    a = scale
    denom = 1 + math.sin(t) ** 2
    x = a * math.cos(t) / denom
    y = a * math.sin(t) * math.cos(t) / denom

    # Compute heading from velocity
    dt = 0.001
    denom2 = 1 + math.sin(t + dt) ** 2
    x2 = a * math.cos(t + dt) / denom2
    y2 = a * math.sin(t + dt) * math.cos(t + dt) / denom2
    yaw = math.atan2(y2 - y, x2 - x)

    return x, y, yaw


def generate_point_cloud(grid, size, resolution, robot_x, robot_y, robot_z=0.0, max_points=3000):
    """Generate a fake 3D point cloud from the occupancy grid (walls become tall points).

    Adds wall points at various heights, floor points, and random obstacle scatter.
    Returns list of [x, y, z] points.
    """
    if not NUMPY_AVAILABLE:
        return []
    origin = -size * resolution / 2.0
    pts = []

    # Wall points: occupied cells get vertical columns
    for i in range(0, size, 2):  # skip every other cell for performance
        for j in range(0, size, 2):
            v = grid[i][j]
            if v >= 100:
                wx = origin + j * resolution + resolution / 2
                wy = origin + (size - 1 - i) * resolution + resolution / 2
                # Only include walls within 5m of robot
                dx, dy = wx - robot_x, wy - robot_y
                if dx * dx + dy * dy > 25.0:
                    continue
                # Add points at multiple heights
                for z in [0.0, 0.15, 0.3, 0.5, 0.7]:
                    pts.append([wx + random.uniform(-0.02, 0.02),
                                wy + random.uniform(-0.02, 0.02),
                                z + random.uniform(-0.02, 0.02)])

    # Floor scatter (free cells near robot)
    for _ in range(400):
        angle = random.uniform(0, 2 * math.pi)
        dist = random.uniform(0.3, 3.0)
        fx = robot_x + dist * math.cos(angle)
        fy = robot_y + dist * math.sin(angle)
        pts.append([fx, fy, random.uniform(-0.05, 0.02)])

    # Random environment scatter (simulates noisy LiDAR returns)
    for _ in range(200):
        angle = random.uniform(0, 2 * math.pi)
        dist = random.uniform(0.5, 4.0)
        pts.append([robot_x + dist * math.cos(angle),
                    robot_y + dist * math.sin(angle),
                    random.uniform(0.0, 1.2)])

    # Subsample if too many
    if len(pts) > max_points:
        random.shuffle(pts)
        pts = pts[:max_points]

    return pts


def main():
    parser = argparse.ArgumentParser(description="SLAM + Point Cloud simulator")
    parser.add_argument("--port", type=int, default=5562, help="ZMQ PUB port for SLAM data (default: 5562)")
    parser.add_argument("--cloud-port", type=int, default=5564, help="ZMQ PUB port for point cloud (default: 5564)")
    parser.add_argument("--rate", type=float, default=10.0, help="Publish rate Hz (default: 10)")
    parser.add_argument("--speed", type=float, default=0.3, help="Robot speed multiplier (default: 0.3)")
    parser.add_argument("--nav2", action="store_true", help="Also publish fake Nav2 path/goal overlay")
    parser.add_argument("--qr", action="store_true", help="Place fake QR markers on the map")
    parser.add_argument("--no-cloud", action="store_true", help="Skip point cloud generation")
    parser.add_argument("--grid-size", type=int, default=200, help="Grid size in cells (default: 200)")
    parser.add_argument("--resolution", type=float, default=0.05, help="Grid resolution m/cell (default: 0.05)")
    args = parser.parse_args()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.bind(f"tcp://*:{args.port}")

    # Point cloud PUB socket (separate port, like real slam_bridge)
    cloud_sock = None
    if not args.no_cloud:
        cloud_sock = ctx.socket(zmq.PUB)
        cloud_sock.setsockopt(zmq.CONFLATE, 1)
        cloud_sock.setsockopt(zmq.SNDHWM, 1)
        cloud_sock.bind(f"tcp://*:{args.cloud_port}")

    print(f"SLAM Simulator publishing on tcp://*:{args.port}")
    if cloud_sock:
        print(f"Point Cloud publishing on tcp://*:{args.cloud_port}")
    print(f"  Rate: {args.rate} Hz | Speed: {args.speed} | Nav2: {args.nav2}")
    print(f"  Grid: {args.grid_size}×{args.grid_size} @ {args.resolution} m/cell")
    print()
    print("Run the GUI with:  cd laptop && python main.py --jetson-ip 127.0.0.1 --no-joystick --no-arm")
    print("Press Ctrl+C to stop.\n")

    grid = generate_arena(args.grid_size, args.resolution)
    origin = -args.grid_size * args.resolution / 2.0

    path_history = []
    t = 0.0
    dt = args.speed / args.rate

    # Fake Nav2 goal waypoints (cycle through)
    nav2_goals = [[2.0, 0.0], [-1.0, 1.5], [0.0, -2.0], [1.5, 1.0]]
    nav2_goal_idx = 0
    nav2_goal_timer = 0.0

    try:
        while True:
            x, y, yaw = figure_8_pose(t, scale=2.5)
            path_history.append([x, y])
            if len(path_history) > 500:
                path_history = path_history[-500:]

            msg = {
                "msg_type": "slam_pose",
                "timestamp": time.time(),
                "x": x,
                "y": y,
                "z": 0.0,
                "yaw": yaw,
                "grid": grid,
                "resolution": args.resolution,
                "origin_x": origin,
                "origin_y": origin,
                "path": path_history,
            }

            # Nav2 overlay (autonomy simulation)
            if args.nav2:
                goal = nav2_goals[nav2_goal_idx % len(nav2_goals)]
                dist = math.hypot(goal[0] - x, goal[1] - y)
                # Simple straight-line "planned path" from robot to goal
                steps = max(2, int(dist / 0.2))
                nav2_path = []
                for i in range(steps + 1):
                    frac = i / steps
                    px = x + (goal[0] - x) * frac
                    py = y + (goal[1] - y) * frac
                    nav2_path.append([px, py])
                msg["nav2_path"] = nav2_path
                msg["nav2_goal"] = goal
                msg["goal_distance_m"] = dist

                # Cycle to next goal every 8 seconds
                nav2_goal_timer += dt
                if nav2_goal_timer > 8.0:
                    nav2_goal_timer = 0.0
                    nav2_goal_idx += 1

            sock.send_json(msg, zmq.NOBLOCK)

            # Publish point cloud on separate port
            if cloud_sock and not args.no_cloud:
                cloud_pts = generate_point_cloud(
                    grid, args.grid_size, args.resolution, x, y)
                cloud_msg = {
                    "msg_type": "point_cloud",
                    "timestamp": time.time(),
                    "points": cloud_pts
                }
                cloud_sock.send_json(cloud_msg, zmq.NOBLOCK)

            # Print pose
            deg = math.degrees(yaw)
            cloud_n = len(cloud_pts) if cloud_sock else 0
            sys.stdout.write(f"\r  Pose: ({x:+6.2f}, {y:+6.2f}) yaw: {deg:+6.1f}°  cloud: {cloud_n} pts  t={t:.1f}")
            sys.stdout.flush()

            t += dt
            time.sleep(1.0 / args.rate)

    except KeyboardInterrupt:
        print("\n\nStopping simulator.")
    finally:
        sock.close()
        if cloud_sock:
            cloud_sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
