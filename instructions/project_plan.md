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
2. **Inference** (`localizer.py`): `AnyLocLocalizer.localize(img, agl_m, center_lat, center_lon, radius_m)` — extracts VLAD, then either (a) geo-filters the database to entries within `radius_m` of `(center_lat, center_lon)` and does a torch inner-product search on the subset, or (b) falls back to full FAISS IndexFlatIP search when no center is given. Returns `(est_lat, est_lon, est_alt, match_img, score, db_idx)`. Match image re-cropped from satellite at drone's actual AGL.
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
Frame 1:    AnyLoc full search (2821 entries) → anchor fix (±15–20 m); vo.reset()
Frame 2–9:  VO only → accum_dlat += dlat, accum_dlon += dlon
            final_pos = anchor + (accum_dlat, accum_dlon)
Frame 10:   AnyLoc constrained search (≤~50 entries within 200 m of VO estimate)
            → new anchor; reset accum; vo.reset()
```

#### Geo-constrained retrieval technique

**Problem:** unconstrained top-1 VLAD retrieval can jump to a visually similar but geographically distant tile (e.g. two similar-looking road intersections 800 m apart). Once the anchor is wrong, the accumulated VO offset compounds the error until the next AnyLoc run — which is also unconstrained and can jump again.

**Technique:** after the first anchor is established, each AnyLoc retrieval is constrained to a geographic window centred on the current VO-refined position estimate. Only the database entries inside that window are considered candidates.

**Steps:**
1. Compute the VO-refined estimate: `center = anchor + (accum_dlat, accum_dlon)`
2. Compute flat-Earth distance from every DB entry to `center`:
   - `d_north = (lat_i − center_lat) × 111,320 m`
   - `d_east  = (lon_i − center_lon) × 111,320 m × cos(lat)`
   - `dist_m  = √(d_north² + d_east²)`
3. Select the subset `in_range` where `dist_m ≤ radius_m` (typically ~50 entries at 200 m)
4. Compute cosine similarity on the subset only: `sims = vlads[in_range] @ desc`
   - Both the query `desc` and stored `vlads` are L2-normalised, so inner product = cosine similarity
5. Pick `argmax(sims)` → the best matching entry within the window

**Why this works:** VO tracks features between frames and accumulates small Δlat/Δlon increments. Even with ~10 % VO drift, the accumulated error over 10 frames is well under 20 m at typical drone speeds — so the true position is always inside the 200 m window. The wrong-tile failure mode requires the true position to be inside the window but a wrong tile to score higher than the correct one. This is much less likely when the candidate pool is 50 geographically local tiles rather than 2,821 scene-wide tiles, because distant visually-similar tiles are excluded by geometry before any feature comparison.

**Radius choice — 200 m:**
- DB grid spacing: 50 m → 200 m = 4 grid steps → ~50 candidate entries
- Max drone displacement in 10 frames (~2 s at 5 fps, 20 m/s): ~40 m
- Typical VO residual error on 40 m: < 10 m
- Safety margin: 200 m / 50 m ≈ 4× — robust against fast flight and VO drift
- Fallback: if `in_range` is empty (VO diverged severely), reverts to full FAISS search

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

**Status:** Working — YOLOv8n COCO pretrained; vehicle classes (car / motorcycle / bus / truck); live annotated postview

Use **YOLOv8** to detect vehicles from the drone's nadir camera.

Implementation:
1. **Detector** (`detector.py`): `YOLODetector` wraps `ultralytics.YOLO`; `detect(pil_img)` filters inference output to COCO vehicle class IDs `{2: car, 3: motorcycle, 5: bus, 7: truck}` and returns a list of `{label, conf, x1, y1, x2, y2}` dicts; `draw(pil_img, detections)` overlays coloured bounding boxes + label chips using PIL `ImageDraw` (numpy-safe).
2. **Postview** (`run_detector.py`): same mtime-polling pattern as `run_localizer.py`; single matplotlib TkAgg window; title shows vehicle count + inference time + drone geo; each detection printed to terminal.

COCO vehicle class IDs and display colours:

| Class | ID | Colour |
|-------|----|--------|
| car | 2 | red `#ff4444` |
| motorcycle | 3 | orange `#ff8800` |
| bus | 5 | purple `#cc44ff` |
| truck | 7 | yellow `#ffee00` |

Model: `yolov8n.pt` (nano, ~6 MB, COCO pretrained, downloaded on first run). Confidence threshold: 0.35.

Run:
```bash
DISPLAY=:2 conda run -n isaac_sim_test python detection/run_detector.py
```

Key design choices:
- PIL `ImageDraw` for bounding box / label rendering — avoids numpy ops (same env constraint as localizer)
- `box.xyxy[0].tolist()` to extract coordinates — stays in torch, avoids broken numpy dispatch
- PIL image passed directly to `model()` — ultralytics accepts PIL natively, no `np.array()` needed

Known limitation: YOLOv8n was trained on horizontal (eye-level) COCO photos. Nadir/aerial vehicle views differ significantly in aspect ratio and appearance — detection confidence will be lower than in horizontal scenes. Fine-tuning on aerial vehicle imagery is needed for production accuracy.

Next steps:
- Fine-tune on an aerial vehicle dataset (e.g. DOTA, VisDrone) for nadir detection
- Feed `{label, conf, bbox, drone_lat, drone_lon}` into `main.py` orchestrator alongside AnyLoc estimate

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
| 5 | YOLO detection working on simulated frames | Done |
| 6 | ArduPilot SITL connected and responding to MAVLink commands | TODO |
| 7 | Full pipeline integrated in simulation (localize → detect → control) | TODO |
| 8 | Deploy to real drone hardware | TODO |
