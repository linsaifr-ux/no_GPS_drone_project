# No-GPS Drone Project

Autonomous drone system that localises itself and detects objects without GPS, validated in Isaac Sim before deployment to real hardware.

**Location:** Chiayi, Taiwan — 23.4509°N, 120.2861°E  
**Stack:** Isaac Sim 6.0.0 · AnyLoc · YOLOv8 · ArduPilot MAVLink

---

## Pipeline

```
Isaac Sim (or real camera)
        │
        ▼
  drone_frames/latest.jpg   (640×480 nadir RGB, ~6 Hz)
        │
   ┌────┴──────────────────┐
   ▼                       ▼
AnyLoc                  YOLO
(position estimate)     (bounding boxes)
   │                       │
   └───────────┬───────────┘
               ▼
          main.py (orchestrator)
               │
               ▼
        ArduPilot (SITL or real)
```

---

## Repository Layout

```
no_GPS_drone_project/
├── instructions/
│   ├── project_plan.md    # module status, design decisions, milestones
│   └── history.md         # session-by-session change log
├── simulator/             # Isaac Sim scene — WORKING
│   ├── cesium_scene.py    # main scene: terrain + buildings + drone + camera
│   ├── drone_frames/      # live output: latest.jpg + latest_meta.json
│   └── run_chiayi.sh      # launch script
├── localization/          # AnyLoc — TODO
├── detection/             # YOLO — TODO
├── control/               # ArduPilot MAVLink — TODO
└── main.py                # top-level orchestrator — TODO
```

---

## Milestones

| # | Milestone | Status |
|---|-----------|--------|
| 1 | Isaac Sim scene: Cesium terrain + NLSC imagery + OSM buildings | Done |
| 2 | Virtual drone + nadir camera publishing frames | Done |
| 3 | AnyLoc map database built from simulated views | TODO |
| 4 | AnyLoc localisation working on simulated frames | TODO |
| 5 | YOLO detection working on simulated frames | TODO |
| 6 | ArduPilot SITL responding to MAVLink commands | TODO |
| 7 | Full pipeline integrated in simulation | TODO |
| 8 | Deploy to real hardware | TODO |

---

## Quickstart

### Requirements

- Isaac Sim 6.0.0 (Kit 106, Python 3.12)
- conda env `isaac_sim_test`
- Cesium ion account (token already embedded in `cesium_scene.py`)
- Display (X11 or virtual framebuffer, e.g. `DISPLAY=:2`)

### Run the simulator

```bash
cd simulator
./run_chiayi.sh
```

On first run, tiles are downloaded from Cesium ion and Taiwan NLSC and cached locally. Subsequent runs start immediately from cache.

### Drone keyboard controls (simulator window must be focused)

| Key | Action |
|-----|--------|
| W / S | Fly north / south (5 m/step) |
| A / D | Fly west / east |
| Q / E | Descend / ascend |
| Z / X | Yaw left / right (1°/step) |

### Frame output

Each simulation step (every 5 updates) the drone camera writes:

- `simulator/drone_frames/latest.jpg` — 640×480 RGB nadir view
- `simulator/drone_frames/latest_meta.json` — `{step, lat, lon, alt_m, yaw_deg, frame_w, frame_h}`

Localization and detection modules consume these files.

---

## Data Sources

| Layer | Source | License |
|-------|--------|---------|
| Terrain | Cesium World Terrain (asset 1) | © Cesium ion |
| Buildings | Cesium OSM Buildings (asset 96188) | © OpenStreetMap contributors (ODbL) |
| Imagery | Taiwan NLSC PHOTO2 orthophoto WMTS | © 內政部國土測繪中心 |
