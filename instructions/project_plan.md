# No-GPS Drone Project — Plan

## Goal

Build a drone system that can localize itself and detect objects without GPS, using visual place recognition (AnyLoc), object detection (YOLO), and ArduPilot for flight control. The full pipeline is validated in Isaac Sim before deploying to real hardware.

---

## Project Structure

```
no_GPS_drone_project/
├── instructions/         # This folder — plans, notes, references
├── simulator/            # Isaac Sim environment (Chiayi, Taiwan)
│   ├── cesium_scene.py   # Main scene: terrain + buildings + drone + nadir camera
│   ├── drone_frames/     # Live output: latest.jpg + latest_meta.json (per step)
│   └── run_chiayi.sh     # Launch script
├── anyloc/               # AnyLoc visual localization — WORKING
│   ├── build_database.py # Build geo-tagged VLAD database from satellite orthophoto (run once)
│   ├── localizer.py      # AnyLocLocalizer class (DINOv2 + VLAD + FAISS)
│   ├── vo_refiner.py     # VORefiner class (LK optical flow, frame-to-frame delta)
│   ├── run_localizer.py  # Live dual postview with AnyLoc+VO combined estimate
│   ├── requirements.txt  # Dependency notes
│   └── database/         # Built database (2821 entries, VLAD dim=49152, 50 m grid)
├── detection/            # YOLO — object detection (TODO)
├── control/              # ArduPilot MAVLink interface (TODO)
└── main.py               # Top-level orchestrator (TODO)
```

---

## Modules

### 1. Simulator (`simulator/`)

**Status:** Working — drone + camera added

Isaac Sim 6.0.0 scene centred on Chiayi, Taiwan (23.4509°N, 120.2861°E).

- **Terrain:** Cesium World Terrain (asset 1) — quantized-mesh-1.0, 9 tiles at level 13
- **Imagery:** Taiwan NLSC aerial orthophoto WMTS (PHOTO2, zoom 18, resized to 4096×4096)
- **Buildings:** Cesium OSM Buildings (asset 96188) — 83 buildings from 4 B3DM tiles
- **Drone:** `/World/Drone` Xform — starts at scene centre, 50 m AGL; keyboard-controlled; quadcopter model (body + 4 arms + motor pods + propeller discs, ~0.8 m span); orange beacon light for visibility from overview
- **Camera:** `/World/Drone/Camera` — nadir, 18 mm / 36×27 mm aperture, **90°×73.7° FOV**, 640×480 render product; viewport (Tab) renders at 1920×1080 from same camera — intentionally separate from ML output
- **HUD:** `omni.ui` overlay (top-left) showing live LAT / LON / ALT MSL / AGL / active camera
- **Frame output:** `drone_frames/latest.jpg` + `latest_meta.json` written every 5 sim steps via `omni.replicator.core`; meta fields: `step`, `lat`, `lon`, `alt_m`, `agl_m`, `centre_elev`, `yaw_deg`, `frame_w`, `frame_h`
- **Environment:** conda env `isaac_sim_test`, Python 3.12, RTX 2080 Ti

Keyboard controls (window must be focused):

| Key | Action |
|-----|--------|
| Tab | Toggle viewport: overview ↔ drone nadir view |
| W / S | Fly north / south (5 m/step) |
| A / D | Fly west / east |
| Q / E | Descend / ascend |
| Z / X | Yaw left / right (1°/step) |

Run:
```bash
cd simulator
./run_chiayi.sh
```

Next steps:
- Wire YOLO detection into the frame loop
- Upgrade frame transport to shared memory when latency matters

---

### 2. Localization (`anyloc/`)

**Status:** Working — AnyLoc + VO; 2,821-entry database (50 m grid); ~15–20 m anchor error; ~5–10 m between anchors

Use **AnyLoc** (universal visual place recognition) to estimate the drone's position from camera images without GPS.

Implementation:
1. **Database** (`build_database.py`): 50 m grid, ±1500 m from scene centre → 2,821 positions; each position crops the NLSC satellite orthophoto at 50 m AGL → DINOv2 ViT-B/14 patch features → intra-normalised VLAD (k=64, dim=49,152); saved with `torch.save()`
2. **Inference** (`localizer.py`): `AnyLocLocalizer.localize(img, agl_m)` — extracts VLAD, queries FAISS IndexFlatIP (cosine sim), returns `(est_lat, est_lon, est_alt, match_img, score, db_idx)`. Match image re-cropped from satellite at drone's actual AGL.
3. **VO refinement** (`vo_refiner.py`): `VORefiner` tracks Shi-Tomasi features with LK optical flow every frame; median pixel displacement → Δlat/Δlon via AGL + FOV + yaw rotation. `reset()` clears state after each AnyLoc re-anchor.
4. **Postview** (`run_localizer.py`): two matplotlib TkAgg windows — `[Drone Camera]` with ground-truth overlay, `[AnyLoc+VO]` with combined estimate; mode tag shows `ANYLOC` on anchor frames and `VO +Nf` between them; error text green < 200 m, blue otherwise.

Accuracy vs grid step:

| Grid step | ~Positions | Expected error |
|-----------|-----------|----------------|
| 200 m | 172 | ~65 m |
| 100 m | ~688 | ~30–40 m |
| **50 m (current)** | **2,821** | **~15–20 m** |
| 25 m | ~11,000 | ~8–12 m |

Hard floor at ~50 m AGL: camera footprint is ~100 m × 75 m, so grid steps below ~50 m produce overlapping images that are hard to distinguish — retrieval accuracy stops improving.

Key design choices:
- All intermediate ops in **torch tensors** (no `np.array` calls) due to dual-numpy conflict in `isaac_sim_test` env
- Numpy reductions (`.sum()`, `.mean()`) replaced with `arr.tolist()` + Python builtins — numpy's `_core/_methods.py` (2.x stub) is broken
- `faiss.Kmeans` replaces sklearn KMeans (sklearn broken by conda-forge faiss-cpu install)
- matplotlib TkAgg replaces cv2 GUI (cv2 built headless in this env)
- PIL ImageDraw for text overlays (avoids numpy ops)
- `cv2.goodFeaturesToTrack` + `cv2.calcOpticalFlowPyrLK` work fine (C-level, not affected by broken numpy)

Run:
```bash
DISPLAY=:2 conda run -n isaac_sim_test python anyloc/run_localizer.py
```

Rebuild database (needed only once, or after scene changes):
```bash
conda run -n isaac_sim_test python anyloc/build_database.py --rebuild
```

VO + AnyLoc combined pipeline (`ANYLOC_INTERVAL = 10`):

```
Frame 1:    AnyLoc retrieval → anchor fix (±15–20 m); vo.reset()
Frame 2–9:  VO only → accum_dlat += dlat, accum_dlon += dlon
            final_pos = anchor + (accum_dlat, accum_dlon)
Frame 10:   AnyLoc retrieval → new anchor; reset accum; vo.reset()
```

Coordinate convention (verify empirically — derived analytically):
- `raw_east = -dx_px × m_per_px_x`  (feature moved right → drone moved west)
- `raw_north = +dy_px × m_per_px_y`  (feature moved down → drone moved north)
- World ENU with yaw: `east = raw_east·cos(yaw) + raw_north·sin(yaw)`

Note: requires textured ground. Homogeneous fields or water produce sparse/noisy matches. The Chiayi urban scene has sufficient texture.

Key references:
- AnyLoc paper: "AnyLoc: Towards Universal Visual Place Recognition" (IRAL 2024)
- AnyLoc repo: https://github.com/AnyLoc/AnyLoc

---

### 3. Object Detection (`detection/`)

**Status:** TODO (frame source ready — reads `simulator/drone_frames/latest.jpg`)

Use **YOLOv8** (or YOLOv11) to detect objects of interest from the drone's camera.

Plan:
1. Train or fine-tune YOLO on classes relevant to the mission (people, vehicles, targets)
2. Run inference on rendered Isaac Sim frames during simulation
3. Output: bounding boxes + class labels + confidence scores
4. Pass detections to the control module to trigger flight manoeuvres

Frame interface: same as localization — poll `simulator/drone_frames/latest.jpg`.

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
| 2 | Quadcopter drone + nadir camera + HUD publishing frames | Done |
| 3 | AnyLoc database built from simulated views | Done |
| 4 | AnyLoc localization working on simulated frames + dual postview | Done |
| 5 | YOLO detection working on simulated frames | TODO |
| 6 | ArduPilot SITL connected and responding to MAVLink commands | TODO |
| 7 | Full pipeline integrated in simulation (localize → detect → control) | TODO |
| 8 | Deploy to real drone hardware | TODO |
