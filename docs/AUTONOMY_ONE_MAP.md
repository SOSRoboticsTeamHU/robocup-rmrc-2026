# Step-by-Step: Autonomous Incline Center & Labyrinth

This guide walks you through setting up **Incline Center** (one lane, laps) and **Labyrinth** (explore waypoints) so autonomy runs end-to-end and the report shows autonomous laps or exploration.

---

## Part A — Incline Center (autonomous laps)

Do this first: one straight lane, robot drives A↔B and laps are counted.

### Step A1. Start the robot with Nav2 (Jetson)

```bash
cd jetson
USE_NAV2=1 ./start_robot.sh
```

Confirm in the output:

- `✓ Nav2 bridge`
- `✓ Nav2 bringup`
- `✓ Autonomy executor`

### Step A2. Start the operator GUI (laptop)

- Start the operator station and connect to the Jetson (set the Jetson IP if needed).
- Confirm you see: **SLAM view**, **status bar**, and the **test dropdown** (e.g. "Incline Horizontal", "Sand & Gravel").

### Step A3. Let SLAM settle

- Wait or drive a short distance so Point-LIO has a stable map and pose.
- The SLAM view should show the map and robot pose; the autonomy executor uses this same pose for waypoints.

### Step A4. Set Waypoint A (start of Incline Center lane)

- **Drive the robot** to the **start** of the Incline Center lane (where you want each lap to begin).
- In the GUI, open the **Scoring** area and click **Set Waypoint A**.
- You should see: *"Waypoint A set: (x.xx, y.yy)"* in the status bar.

### Step A5. Set Waypoint B (end of lane)

- **Drive the robot** to the **end** of the Incline Center lane (turnaround point).
- Click **Set Waypoint B**.
- You should see: *"Waypoint B set: (x.xx, y.yy)"*.

### Step A6. Return to start and stow

- Drive the robot back to the **start** (Waypoint A).
- Put the arm in **stowed** (preset or manual).
- Leave the robot powered and ready; do not move it after starting autonomy for the first lap.

### Step A7. Start mission and run autonomy (Incline Center)

1. In the test dropdown, select **Incline Horizontal** or **Incline Inclined** (or **Sand & Gravel** if using that lane).
2. Check the **Stowed** checkbox (required for 4× autonomy bonus).
3. Click **▶ Start Mission** (or your “Start Mission” button).
4. Click **Mapping**.

The GUI sends a **lap** command with Waypoint A and B. The autonomy executor will:

- Send a Nav2 goal to **B**; when the robot reaches B (within ~0.25 m), it sends a goal to **A** and increments the lap count.
- Keep alternating A↔B so **Laps** in the autonomy panel increase.

### Step A8. Monitor and stop

- Watch **Laps** in the autonomy panel.
- To stop: click **ABORT** or move the joystick for 500 ms (except in the 30 cm end-wall zone).
- If Nav2 doesn’t move the robot for >15 s, the watchdog aborts (and can say “Nav2 stuck, switching to teleop”).

### Step A9. Report (Incline Center)

- After the run, click **Report** (or Generate Report).
- The PDF should include **Laps (autonomy)** so the judge can see the autonomous laps for Incline Center.

---

## Part B — Labyrinth (autonomous explore)

After Incline Center works, set up Labyrinth so the robot drives through the maze autonomously via **Explore** mode (sequence of waypoints). You define the waypoints with **Set Fiducial** (or Waypoint A/B as fallback).

### Step B1. Same startup as Incline Center

- Jetson: `USE_NAV2=1 ./start_robot.sh`
- Laptop: operator GUI connected, SLAM view and status visible.

### Step B2. Drive the labyrinth once (teleop) to learn the path

- Select **Labyrinth Flat** or **Labyrinth K-Rails** in the test dropdown.
- Drive through the maze with the joystick and decide **2–4 key points** (e.g. corners or gates) that form a path the robot should follow autonomously. You will mark these as fiducials in the next step.

### Step B3. Set fiducials along the path (in order)

- Drive to the **first** key point (e.g. first corner after start).
- Click **Set Fiducial**. The current SLAM pose is appended to `reports/fiducials.json`.
- Drive to the **second** key point → **Set Fiducial** again.
- Repeat for the **third** (and optionally fourth) point, in the order you want the robot to visit.

The GUI uses **fiducials** as the explore waypoints when you start Labyrinth autonomy. If you have no fiducials, it falls back to Waypoint A and B only (two points), or a default box.

### Step B4. Return to start and stow

- Drive the robot back to the **start** of the labyrinth (e.g. at or near the first fiducial).
- Put the arm in **stowed**.

### Step B5. Start mission and run autonomy (Labyrinth)

1. In the test dropdown, select **Labyrinth Flat** or **Labyrinth K-Rails**.
2. Check **Stowed**.
3. Click **▶ Start Mission**.
4. Start autonomy (the GUI sends **explore** with the list of waypoints from fiducials).

The autonomy executor will drive to each waypoint in order (1 → 2 → 3 → …). When it reaches the last waypoint, the explore run completes.

### Step B6. Monitor and stop

- Watch the autonomy panel for step/progress (e.g. “Explore waypoint 2/4”).
- To stop: **ABORT** or joystick 500 ms (with 30 cm exception).

### Step B7. Report (Labyrinth)

- Generate the **Report** after the run. You can note in the report or verbally that Labyrinth was run in Explore mode (autonomous waypoint sequence).

---

## Summary checklist

| Step | Incline Center | Labyrinth |
|------|----------------|-----------|
| Start | `USE_NAV2=1 ./start_robot.sh` + GUI | Same |
| Waypoints | **Set Waypoint A** at start, **Set Waypoint B** at end | **Set Fiducial** at 2–4 points along path (in order) |
| Test | Incline Horizontal / Incline Inclined (or Sand & Gravel) | Labyrinth Flat / Labyrinth K-Rails |
| Stowed | ✓ Check **Stowed** | ✓ Check **Stowed** |
| Start | **Start Mission** → **Mapping** (lap mode) | **Start Mission** → start autonomy (explore mode) |
| Result | Laps counted A↔B; **Laps (autonomy)** in report | Robot visits waypoints in order; report documents run |

---

## Fallback: Labyrinth with only A and B

If you don’t use **Set Fiducial**, the GUI uses **Waypoint A** and **Waypoint B** as the explore sequence (two points). So you can:

- Set **Waypoint A** at the labyrinth start.
- Set **Waypoint B** at the first key point (e.g. first corner).
- Select Labyrinth and start autonomy → robot goes A → B and stops.

For a longer path, add more points with **Set Fiducial** (they are saved in `reports/fiducials.json` and reused until you change them).

---

## Troubleshooting

| Issue | Check |
|-------|--------|
| Robot doesn’t move (Incline or Labyrinth) | `USE_NAV2=1`; “Nav2 bringup” and “Autonomy executor” in `start_robot.sh` output. |
| Wrong or no waypoints | Set A/B or fiducials **after** SLAM has a good pose (status bar confirms). |
| Laps not in report (Incline) | Autonomy status (port 5565) reaching GUI; **laps_auto** updated; then generate Report. |
| Labyrinth goes to wrong order | Fiducials are used in the order they were added; set them in the desired visit order. |
| Nav2 stuck / 15 s abort | Clearer path, shorter segment, or check map/obstacles. |

---

See also: **OPERATOR_RUNBOOK.md**, **COMPETITION_CHECKLIST.md**.
