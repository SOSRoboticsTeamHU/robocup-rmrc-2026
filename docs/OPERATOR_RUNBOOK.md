# RoboCupRescue RMRC 2026 – Operator Runbook

Per-test procedures, Pick modes, watchdog behavior, and judge announcement phrases.

---

## Startup

- **Teleop only:** `./start_robot.sh`
- **With Nav2/autonomy:** `USE_NAV2=1 ./start_robot.sh`
- **Sim (no Pico):** `USE_SIM=1 ./start_robot.sh`
- GUI starts in **TELEOP** mode. Select test type from dropdown. Set waypoints (lap tests) or approach (dexterity) via teleop. Check **Stowed** → click **▶ Start Autonomy** → GUI switches to **AUTONOMY**. Judge sees **HANDS OFF** and grayed joystick. **ABORT**: big red button or joystick movement >500 ms → teleop.

---

## Linear Rail Pick – 3 Modes (rulebook allowed)

| Mode | Behavior |
|------|----------|
| **Full Autonomy** | Nav2 → grasp → lift → drive to container → drop → stow |
| **Pause for Base Teleop** | Grasp → lift → **PAUSE** (GUI: "BASE TELEOP ALLOWED – arm locked") → operator drives to container → **Resume Auto** → drop → stow |
| **Full Teleop (arm only)** | Arm presets only; base stays teleop |

Select mode in GUI combo before starting Pick run.

---

## Watchdog & Safety

- **Nav2 stuck >15 s** → auto ABORT + TTS: "Nav2 stuck, switching to teleop".
- **Battery <11.8 V** → warning + slow down.
- **Joystick** in autonomy: beyond deadzone for **500 ms** → lap becomes teleop (except within **30 cm of end walls** – rulebook exception). Joystick movement is logged when not in 30 cm zone.

---

## Judge / Announcements

- Before autonomy: *"Robot is stowed; starting autonomous run."*
- On snapshot (HAZMAT/Landolt): *"Providing still image with label location and identity."*
- On abort: *"Aborting autonomy; taking teleop control."*
- After run: hand over **mission report PDF** and **QR codes file**.

---

## Per-Test (summary)

- **Mobility (laps):** Set Waypoint A/B, start autonomy, laps counted automatically; Lap (T) / Lap (A) scoring.
- **Sensor Cabinet:** Stowed → door → cabinet_view → HAZMAT snapshot → Landolt snapshot → stow; door/hazmat/landolt checkboxes.
- **Keypad:** Nav2 to keypad → arm extend → drive forward → retract → out; +3 clean / +1 dirty.
- **Linear Rail Inspect:** Tube C1–C5, direction dropdown, Record Reading, Show Gap.
- **Linear Rail Pick:** Per-tube Removed / In Container; use designated container ≤30×30×10 cm.
- **Stairs / Align / Drop Test:** Teleop only; per-test scoring buttons as in GUI.

---

## Foxglove / 3D view

- **Use downsampled cloud for Foxglove 3D view to avoid ~20 MB limit.** Subscribe to `/cloud_downsampled` (not `/cloud_registered`) when connecting Foxglove Bridge or any remote 3D viewer. The point_cloud_downsampler node keeps output under 10k points at 5 Hz (voxel_leaf_size 0.05 m).

---

## Reset / Between runs

- ABORT or task complete → return to TELEOP. Clear scoring if new mission. Generate Report before judge handover.
