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
├── anyloc/                # AnyLoc visual localization — WORKING
│   ├── build_database.py  # build VLAD database from satellite orthophoto (run once)
│   ├── localizer.py       # AnyLocLocalizer (DINOv2 + VLAD + FAISS)
│   ├── run_localizer.py   # live dual postview
│   └── database/          # 2821-entry VLAD database (49152-dim, 50 m grid)
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
| 3 | AnyLoc map database built from simulated views | Done |
| 4 | AnyLoc localisation + dual postview on simulated frames | Done |
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

### HUD

A semi-transparent overlay in the top-left corner of the viewport shows the drone's live position:

```
  LAT  23.45087°N    LON  120.28614°E
  ALT  96.3 m MSL    AGL  50.0 m
  CAM  Overview
```

### Drone keyboard controls (simulator window must be focused)

| Key | Action |
|-----|--------|
| Tab | Toggle viewport: overview ↔ drone nadir (90°×73.7° FOV) |
| W / S | Fly north / south (5 m/step) |
| A / D | Fly west / east |
| Q / E | Descend / ascend |
| Z / X | Yaw left / right (1°/step) |

### Drone model

Quadcopter (~0.8 m span): central body, 4 arms at 45° intervals, motor pods and propeller discs at arm tips. An orange beacon light (`SphereLight`, 5000 cd) makes the drone findable from the overview camera.

### Frame output

Every 5 sim steps the drone camera writes to `simulator/drone_frames/`:

- `latest.jpg` — 640×480 RGB nadir view (ML input for AnyLoc and YOLO)
- `latest_meta.json` — `{step, lat, lon, alt_m, agl_m, centre_elev, yaw_deg, frame_w, frame_h}`

The Tab viewport renders the same camera at 1920×1080 for visual inspection — intentionally a different resolution from the ML output.

### Run the AnyLoc localizer (separate terminal)

```bash
DISPLAY=:2 conda run -n isaac_sim_test python anyloc/run_localizer.py
```

Two side-by-side matplotlib windows appear:
- **Drone Camera** — live `latest.jpg` with ground-truth LAT / LON / ALT / YAW overlay
- **AnyLoc+VO** — satellite crop at the matched position with estimated LAT / LON / ERR overlay; text turns green when error < 200 m; mode tag shows `ANYLOC` on anchor frames and `VO +Nf` between them

AnyLoc runs every 10 frames; Visual Odometry (LK optical flow) fills in between, accumulating a Δlat/Δlon from the last anchor. After the first anchor is set, each AnyLoc retrieval is geo-constrained to the 200 m window around the VO estimate, preventing jumps to wrong tiles.

Typical localisation performance (RTX 2080 Ti): ~183 ms per AnyLoc frame, ~15–20 m anchor error, ~5–10 m between anchors (50 m grid, 2,821 database entries).

Rebuild the database if the scene or camera FOV changes:

```bash
conda run -n isaac_sim_test python anyloc/build_database.py --rebuild
```

---

## Data Sources

| Layer | Source | License |
|-------|--------|---------|
| Terrain | Cesium World Terrain (asset 1) | © Cesium ion |
| Buildings | Cesium OSM Buildings (asset 96188) | © OpenStreetMap contributors (ODbL) |
| Imagery | Taiwan NLSC PHOTO2 orthophoto WMTS | © 內政部國土測繪中心 |
