# No-GPS Drone Project — Plan

## Goal

Build a drone system that can localize itself and detect objects without GPS, using visual place recognition (AnyLoc), object detection (YOLO), and ArduPilot for flight control. The full pipeline is validated in Isaac Sim before deploying to real hardware.

---

## Project Structure

```
no_GPS_drone_project/
├── instructions/         # This folder — plans, notes, references
├── simulator/            # Isaac Sim environment (Chiayi, Taiwan)
│   ├── cesium_scene.py   # Main scene: Cesium terrain + OSM buildings + NLSC imagery
│   └── run_chiayi.sh     # Launch script
├── localization/         # AnyLoc — GPS-denied place recognition (TODO)
├── detection/            # YOLO — object detection (TODO)
├── control/              # ArduPilot MAVLink interface (TODO)
└── main.py               # Top-level orchestrator (TODO)
```

---

## Modules

### 1. Simulator (`simulator/`)

**Status:** Working

Isaac Sim 6.0.0 scene centred on Chiayi, Taiwan (23.4509°N, 120.2861°E).

- **Terrain:** Cesium World Terrain (asset 1) — quantized-mesh-1.0, 9 tiles at level 13
- **Imagery:** Taiwan NLSC aerial orthophoto WMTS (PHOTO2, zoom 18, resized to 4096×4096)
- **Buildings:** Cesium OSM Buildings (asset 96188) — 83 buildings from 4 B3DM tiles
- **Environment:** conda env `isaac_sim_test`, Python 3.12, RTX 2080 Ti

Run:
```bash
cd simulator
./run_chiayi.sh
```

Next steps:
- Attach a virtual camera prim to the drone agent
- Publish rendered frames to a ROS2 topic or shared memory for localization and detection

---

### 2. Localization (`localization/`)

**Status:** TODO

Use **AnyLoc** (universal visual place recognition) to estimate the drone's position from camera images without GPS.

Plan:
1. Build a map database from Isaac Sim rendered views (offline, at known positions)
2. At runtime, query AnyLoc with the live camera frame to retrieve the closest database entry
3. Refine the estimate using visual odometry between consecutive frames
4. Output: estimated (lat, lon, altitude) or ENU (x, y, z) position

Key references:
- AnyLoc paper: "AnyLoc: Towards Universal Visual Place Recognition" (IRAL 2024)
- AnyLoc repo: https://github.com/AnyLoc/AnyLoc

---

### 3. Object Detection (`detection/`)

**Status:** TODO

Use **YOLOv8** (or YOLOv11) to detect objects of interest from the drone's camera.

Plan:
1. Train or fine-tune YOLO on classes relevant to the mission (people, vehicles, targets)
2. Run inference on rendered Isaac Sim frames during simulation
3. Output: bounding boxes + class labels + confidence scores
4. Pass detections to the control module to trigger flight manoeuvres

---

### 4. Flight Control (`control/`)

**Status:** TODO

Interface with **ArduPilot** via MAVLink to command the drone.

Plan:
1. **Simulation:** ArduPilot SITL (Software In The Loop) — no real hardware needed
   - Connect MAVLink to SITL over UDP
   - Use `pymavlink` or `dronekit` to send commands
2. **Real hardware:** swap SITL connection for serial/UDP to a real flight controller
3. Behaviours to implement:
   - Takeoff / land
   - Waypoint navigation using AnyLoc position estimates
   - Hover and track a detected object
   - Return to launch on localization failure

---

## Integration Flow

```
Isaac Sim (or real camera)
        │
        ▼
   Camera frame (RGB)
        │
   ┌────┴─────────────────┐
   │                      │
   ▼                      ▼
AnyLoc               YOLO
(position estimate)  (detections)
   │                      │
   └────────┬─────────────┘
            ▼
       main.py (orchestrator)
            │
            ▼
     ArduPilot (SITL or real)
```

---

## Milestones

| # | Milestone | Status |
|---|-----------|--------|
| 1 | Isaac Sim scene running with Cesium terrain + NLSC imagery | Done |
| 2 | Virtual drone camera publishing frames in simulation | TODO |
| 3 | AnyLoc database built from simulated views | TODO |
| 4 | AnyLoc localization working on simulated frames | TODO |
| 5 | YOLO detection working on simulated frames | TODO |
| 6 | ArduPilot SITL connected and responding to MAVLink commands | TODO |
| 7 | Full pipeline integrated in simulation (localize → detect → control) | TODO |
| 8 | Deploy to real drone hardware | TODO |
