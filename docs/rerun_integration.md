# Rerun.io Integration — Point-LIO + Animated Arm

Embedded Rerun 3D viewer with point cloud, full robot URDF, joint animation, odometry, and Nav2 path. Same pattern as the [official animated_urdf example](https://rerun.io/viewer?url=https://app.rerun.io/version/0.29.2/examples/animated_urdf.rrd).

## Installation

```bash
# Jetson (ROS2 + bridge)
pip install rerun-sdk numpy

# Laptop (GUI with embedded viewer)
pip install rerun-sdk PyQtWebEngine
```

PyQt5 is required for the laptop GUI; PyQtWebEngine is needed for the embedded Rerun web viewer tab. Without PyQtWebEngine, the Rerun area shows a placeholder; use the “Launch Rerun Viewer” or “Browser” button, or run: `rerun --connect "rerun+http://JETSON_IP:9876/proxy"`.

## Quick run

1. **Jetson:** Start robot with Rerun enabled:
   ```bash
   USE_RERUN=1 ./start_robot.sh
   ```
   Or run the bridge alone:
   ```bash
   python3 jetson/rerun_bridge.py --serve-port 9876
   ```

2. **Laptop:** Start operator station; open the **"Rerun 3D + Animated Arm"** tab (default). The embedded viewer connects to `rerun+http://JETSON_IP:9876/proxy`. Use the **🔄 Rerun 3D View** button to switch to this tab from the Control Panel.

## 2-day plan

### Day 1 — Bridge and connectivity

| Step | Task | Check |
|------|------|--------|
| 1.1 | Install on Jetson: `pip install rerun-sdk numpy` | `python3 -c "import rerun as rr; print('ok')"` |
| 1.2 | Run bridge: `USE_RERUN=1 ./start_robot.sh` or `python3 jetson/rerun_bridge.py` | Process runs; no Python tracebacks |
| 1.3 | On laptop, run native viewer: `rerun --connect "rerun+http://JETSON_IP:9876/proxy"` | Viewer connects; see welcome point and/or point cloud and URDF |
| 1.4 | Confirm topics: `/cloud_registered`, `/robot_description`, `/joint_states`, `/Odometry` or `/tf`, `/plan` | Bridge logs without errors |
| 1.5 | If `UrdfTree` or `log_file_from_path` missing (older rerun-sdk), bridge falls back to cloud/tf/path only | No crash; point cloud and path still work |

### Day 2 — GUI embed and polish

| Step | Task | Check |
|------|------|--------|
| 2.1 | Install on laptop: `pip install PyQtWebEngine` | Rerun tab shows WebEngineView, not placeholder |
| 2.2 | Confirm main view shows cameras left, Rerun right | No separate tab; Rerun is the main 3D view |
| 2.3 | Verify status label and Reconnect button in Rerun area | "Connecting…" → "Rerun 3D \| robot IP:9876" or load failed message |
| 2.4 | Reconnect: stop bridge on Jetson, click Reconnect after restarting | Viewer recovers after reconnect |
| 2.5 | Competition run: USE_RERUN=1 on Jetson; operator uses Rerun as primary 3D view | Stable for full run |

## Architecture

- **Jetson** (`jetson/rerun_bridge.py`): Subscribes to ROS2 topics, downsamples point cloud to 12k points (height rainbow), logs URDF + joint transforms (animated_urdf style), odometry/tf, and Nav2 path; serves gRPC on port 9876.
- **Laptop** (`laptop/gui/main_window.py`): Main content = cameras (left) + Rerun 3D (right). Rerun is a single `RerunEmbedWidget` (QWebEngineView loading the Rerun web viewer connected to `rerun+http://JETSON_IP:9876/proxy`). Footer = test selector, scoring, control panel.

## Troubleshooting

Use this checklist to find where the pipeline fails (A → E). Replace `JETSON_IP` with your robot’s IP (e.g. `192.168.1.10`).

| Step | Check | How |
|------|--------|-----|
| **A. Bridge starts** | Jetson runs the bridge without Python errors | `USE_RERUN=1 ./start_robot.sh` or `python3 jetson/rerun_bridge.py --serve-port 9876`. Check `jetson/logs/rerun_bridge.log` or terminal for tracebacks. |
| **B. Port reachable** | Laptop can reach Jetson:9876 | From laptop: `curl -v http://JETSON_IP:9876` or `nc -zv JETSON_IP 9876`. If connection refused or timeout, open port 9876 on the Jetson (e.g. `sudo ufw allow 9876`) and ensure the bridge binds to `0.0.0.0`. |
| **C. Native viewer** | Native Rerun app connects and shows data | On laptop: `pip install rerun-sdk` then run exactly: `rerun --connect "rerun+http://JETSON_IP:9876/proxy"`. If you see the welcome point and/or robot/cloud, the bridge and network are fine. |
| **D. Web viewer URL** | Embedded or browser viewer loads and connects | In a browser open: `https://app.rerun.io/version/0.29.2/?url=rerun%2Bhttp%3A%2F%2FJETSON_IP%3A9876%2Fproxy` (replace `JETSON_IP`). If this works, the embed in the GUI should use the same URL. |
| **E. Data in viewer** | Something appears (cloud, robot, or at least one entity) | The bridge logs a small “welcome” point at startup. If the viewer connects but stays empty, ROS2 topics may not be published; check bridge logs and ensure Point-LIO / robot_state_publisher are running. |

### Exact connection URLs

- **Native viewer (laptop terminal):**
  ```bash
  rerun --connect "rerun+http://JETSON_IP:9876/proxy"
  ```
  Some setups also support: `rerun --connect tcp://JETSON_IP:9876` (try if the above fails).

- **Web viewer (browser or embed):**
  ```
  https://app.rerun.io/version/0.29.2/?url=rerun%2Bhttp%3A%2F%2FJETSON_IP%3A9876%2Fproxy
  ```

### Test connectivity (no ROS2)

To verify only “viewer → Jetson” without any ROS2 topics, on the Jetson run:

```bash
python3 jetson/rerun_bridge.py --test-connectivity
```

Then from the laptop run `rerun --connect "rerun+http://JETSON_IP:9876/proxy"`. You should see a single point at the origin. Press Ctrl+C on the Jetson to stop.

### Other issues

- **Viewer does not connect:** Ensure `rerun_bridge` is running on the Jetson and firewall allows port 9876. Use Reconnect in the Rerun area, or use “Launch Rerun Viewer” / “Browser” from the GUI.
- **No URDF / no arm motion:** Ensure `/robot_description` is published or that the fallback URDF file exists (`jetson/robot_description/urdf/rescue_robot.urdf` or `so101_new_calib.urdf`). Check bridge logs for URDF load errors.
- **Placeholder instead of viewer:** Install PyQtWebEngine on the laptop; restart the GUI. If the embed stays blank, use “Launch Rerun Viewer” or “Browser” for a reliable view.
