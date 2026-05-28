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
├── detection/            # YOLO — object detection (WORKING)
│   ├── detector.py       # YOLODetector — auto-detects COCO / VisDrone class maps
│   ├── run_detector.py   # live mtime-polling postview
│   ├── label_writer.py   # pure-Python nadir projection for synthetic label export
│   ├── collect_training_data.py  # Isaac Sim headless synthetic data collector
│   ├── prepare_dataset.py        # download VisDrone + remap + merge synth
│   └── finetune.py               # YOLOv8 top-down fine-tuning script
├── yolov8l_visdrone.pt   # YOLOv8l pre-trained on VisDrone (active model)
├── yolov8n.pt            # YOLOv8n COCO pretrained (baseline)
├── control/              # ArduPilot MAVLink interface (in progress)
│   ├── sitl_bridge.py    #   UDP :9002 server — binary servo in, JSON physics out
│   ├── stub_bridge.py    #   minimal bridge for testing MAVLink without Isaac Sim
│   ├── mavlink_ctrl.py   #   MAVLinkCtrl class + EKF flag constants
│   └── run_mavlink.py    #   live terminal monitor
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

**Status:** Working — `yolov8l_visdrone.pt` (VisDrone-trained); auto class-map; fine-tuning pipeline ready to run

Use **YOLOv8** to detect vehicles from the drone's nadir camera.

#### Active model

`yolov8l_visdrone.pt` — YOLOv8-large pre-trained on VisDrone 2019 DET (10 aerial vehicle classes). Confidence threshold: 0.30.

#### Implementation

1. **Detector** (`detector.py`): `YOLODetector` wraps `ultralytics.YOLO`. Class mapping is built automatically at load time from `model.names` via a canonical name dict (`_NAME_TO_LABEL`) — supports both COCO and VisDrone models without code changes. `detect(pil_img)` returns `{label, conf, x1, y1, x2, y2}` dicts; `draw()` overlays coloured boxes via PIL `ImageDraw` (numpy-safe).

2. **Postview** (`run_detector.py`): same mtime-polling pattern as `run_localizer.py`; single matplotlib TkAgg window; title: vehicle count + inference time + drone geo.

VisDrone → canonical class map (active):

| VisDrone class | ID | Canonical label | Colour |
|----------------|----|-----------------|--------|
| car | 3 | car | red `#ff4444` |
| van | 4 | car | red `#ff4444` |
| truck | 5 | truck | yellow `#ffee00` |
| tricycle | 6 | motorcycle | orange `#ff8800` |
| awning-tricycle | 7 | motorcycle | orange `#ff8800` |
| bus | 8 | bus | purple `#cc44ff` |
| motor | 9 | motorcycle | orange `#ff8800` |

Run:
```bash
DISPLAY=:2 conda run -n isaac_sim_test python detection/run_detector.py
```

#### Fine-tuning pipeline (top-down specific)

Four scripts implement the full fine-tuning workflow:

| Script | Env | Purpose |
|--------|-----|---------|
| `collect_training_data.py` | `isaac_sim_test` (headless) | Fly grid at 30/60/100 m AGL; export JPEG + YOLO labels for 43 vehicles |
| `prepare_dataset.py` | any ultralytics env | Download VisDrone (auto via ultralytics); remap to 4 classes; merge synth |
| `finetune.py` | any ultralytics env (GPU) | 100 epochs, `degrees=45`, `scale=0.5`, `mosaic=1.0`, `flipud=0.5` |
| `label_writer.py` | shared lib | Pure-Python nadir projection; no numpy; safe inside `isaac_sim_test` |

Dataset after `prepare_dataset.py`:
- Source: VisDrone 2019 DET train (~7k images) + synthetic frames
- Classes: `[car, motorcycle, bus, truck]` (nc=4)
- Layout: `detection/dataset/{images,labels}/{train,val}/`

Best weights after training: `detection/runs/topdown_v1/weights/best.pt`

Key design choices (all scripts):
- PIL `ImageDraw` for bounding box rendering — avoids numpy ops
- `box.xyxy[0].tolist()` to extract coordinates — stays in torch
- PIL image passed directly to `model()` — no `np.array()` needed
- `label_writer.py` uses pure Python math — safe inside `isaac_sim_test`
- `collect_training_data.py` uses `Image.frombytes()` instead of `.astype()` on the replicator buffer

Next step:
- Feed `{label, conf, bbox, drone_lat, drone_lon}` into `main.py` orchestrator alongside AnyLoc estimate

---

### 4. Flight Control (`control/`)

**Status:** In progress — ArduPilot SITL + MAVLink next; IMU fusion after

#### Architecture decision (2026-05-27)

Build ArduPilot SITL + MAVLink first, then IMU fusion. On real hardware IMU data arrives via MAVLink `HIGHRES_IMU` messages — building the reader against MAVLink now means zero interface changes at deployment. ArduPilot SITL's own sensor models (bias drift, noise, temperature effects) are also more realistic than analytical derivatives from position data.

#### Step 1 — ArduPilot SITL + Isaac Sim JSON bridge (`control/sitl_bridge.py`) — DONE

**Protocol:** ArduPilot is the JSON **client**; the bridge is the **server**.

```
ArduPilot SITL ──binary servo_packet_16──► bridge :9002  (UDP server, listens)
ArduPilot SITL ◄──physics JSON + \n─────── bridge        (reply to sender's ephemeral port)
ArduPilot SITL ──MAVLink TCP:5762──► mavlink_ctrl.py
```

`sitl_bridge.py` is a UDP server embedded in the Isaac Sim simulation loop:
- Binds to port 9002; waits for ArduPilot to send a **binary** `servo_packet_16` struct
  (40 bytes, little-endian: `uint16 magic=18458`, `uint16 frame_rate`, `uint32 frame_count`, `uint16 pwm[16]`)
- Learns ArduPilot's reply address from the source port of the first valid servo packet
- Replies with physics state JSON **terminated by `\n`** (required by ArduPilot's `recv_fdm` parser)
- Computes velocity + acceleration by finite-differencing successive ENU positions
- Clamps velocity to ±30 m/s and low-pass filters acceleration (EMA α=0.3)

Physics state sent each step (key names must match ArduPilot's `SIM_JSON` keytable exactly):

```json
{
  "timestamp": 1234.5,
  "imu": {
    "gyro":       [0.0, 0.0, yaw_rate],
    "accel_body": [sf_bx, sf_by, sf_bz]
  },
  "attitude":  [0.0, 0.0, yaw_rad],
  "velocity":  [vn, ve, vd],
  "position":  [north, east, down],
  "rng_1":     agl_m,
  "battery":   {"voltage": 12.6, "current": 5.0}
}
```

`position` and `velocity` currently act as a GPS substitute in ArduPilot's EKF.
They are **removed in milestone 6b-ii** once `VISION_POSITION_ESTIMATE` is working.

Build SITL once before first run:
```bash
cd third_party/ardupilot
git submodule update --init --depth=1 --recursive
python3 waf configure --board sitl && python3 waf copter
cd ../..
```

Run SITL:
```bash
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild --console --map \
    -l 23.450868,120.286135,46,0 \
    --add-param-file=control/no_gps.parm
```

#### Step 2 — No-GPS MAVLink integration (`control/mavlink_ctrl.py`)

This is the core no-GPS milestone. Four sub-steps must happen in order:

**6b-i  pymavlink connection** — DONE
`pymavlink` connects to SITL directly on `tcp:localhost:5762` (no mavproxy needed). Subscribe to:
`HEARTBEAT`, `HIGHRES_IMU`, `ATTITUDE`, `LOCAL_POSITION_NED`, `EKF_STATUS_REPORT`.

**6b-ii  Disable GPS; strip position from JSON bridge**
- Set SITL param `GPS_TYPE=0` (disables GPS sensor in ArduPilot)
- Remove `"position"` and `"velocity"` from the JSON bridge state packet (currently they act as GPS substitute)
- ArduPilot EKF3 now runs on IMU + baro only → position drifts until vision arrives

**6b-iii  Feed AnyLoc → ArduPilot EKF3 via `VISION_POSITION_ESTIMATE`**
- `mavlink_ctrl.py` reads AnyLoc position estimate from `latest_meta.json` (or shared state)
- Sends `VISION_POSITION_ESTIMATE` MAVLink message each AnyLoc anchor frame:
  ```
  time_usec, x (NED north m), y (NED east m), z (NED down m),
  roll=0, pitch=0, yaw (rad), covariance matrix
  ```
- ArduPilot EKF3 fuses this as the external vision position source — same mechanism
  as Intel RealSense T265 or OptiTrack on a real drone
- Verify EKF accepts it: `EKF_STATUS_REPORT.flags` bit 9 (`PRED_POS_HORIZ_ABS`) goes high

**6b-iv  Flight commands replace keyboard**
Send:
- `SET_POSITION_TARGET_LOCAL_NED` — fly to NED waypoints from planned path
- `COMMAND_LONG (MAV_CMD_NAV_TAKEOFF)` — arming + takeoff
- `COMMAND_LONG (MAV_CMD_NAV_RETURN_TO_LAUNCH)` — RTL on localization failure

**Why this order matters:**
ArduPilot will not accept position commands until EKF3 has a valid position estimate.
`VISION_POSITION_ESTIMATE` must arrive and be accepted before `SET_POSITION_TARGET` works.

#### Step 3 — IMU data via MAVLink (`control/imu_reader.py`)

Reads `HIGHRES_IMU` from the MAVLink stream and publishes alongside `latest_meta.json`:

```python
imu_fields = {
    "accel_xyz": [xacc, yacc, zacc],   # m/s² — includes gravity
    "gyro_xyz":  [xgyro, ygyro, zgyro], # rad/s
    "timestamp_us": time_usec
}
```

This is the same message format a real ArduPilot flight controller sends — no code changes needed at hardware deployment.

#### Step 4 — IMU fusion into localization (`control/imu_fusion.py`)

IMU data is used as a sanity check on AnyLoc anchor updates, not as a primary position estimate:

1. **Anchor validator** — if a new AnyLoc anchor deviates more than `jump_threshold` from the IMU dead-reckoned position, reject the anchor and hold the current VO estimate
2. **VO quality gate** — if IMU detects high angular velocity or linear acceleration spikes, mark that VO frame as unreliable and skip the accumulation step
3. **Dead-reckoning fallback** — if both AnyLoc and VO fail (low feature count + bad scores), use IMU double-integration for short bridging intervals

Files status:

| File | Status | Purpose |
|------|--------|---------|
| `control/sitl_bridge.py` | Done | Binary servo in → JSON physics out, UDP :9002; no GPS |
| `control/no_gps.parm` | Done | SITL param file: GPS_TYPE=0 |
| `control/stub_bridge.py` | Done | Static hover for testing MAVLink without Isaac Sim |
| `control/mavlink_ctrl.py` | Done (6b-i) | pymavlink subscriber + vision + command stubs |
| `control/run_mavlink.py` | Done | Live terminal monitor at 10 Hz |
| `control/imu_reader.py` | TODO (6c) | HIGHRES_IMU reader, writes to shared state |
| `control/imu_fusion.py` | TODO (6d) | IMU-based anchor validation + VO quality gate |

Real hardware path: swap SITL UDP address for serial/UDP to the real flight controller — no other changes needed.

---

## Integration Flow

```
Isaac Sim (cesium_scene.py)
    │
    ▼
control/sitl_bridge.py  ◄──binary servo_packet_16── ArduPilot SITL
  (UDP server :9002)    ──physics JSON+\n──────►  (JSON client)
                                                   │ MAVLink TCP:5762
                                              ▼
                                   control/mavlink_ctrl.py
                                   ┌──────────┴────────────┐
                                   ▼                       ▼
                             imu_reader.py          command sender
                             (HIGHRES_IMU)          (SET_POSITION_TARGET)
                                   │
                                   ▼
drone_frames/latest.jpg    imu_fusion.py
        │                  (anchor validator + VO gate)
   ┌────┴──────────────┐          ▲
   ▼                   ▼          │ IMU data
AnyLoc + VO          YOLO         │
(position estimate)  (detections) │
   │                              │
   ├──VISION_POSITION_ESTIMATE──► ArduPilot EKF3
   │  (MAVLink, via mavlink_ctrl) (no-GPS position fusion)
   │
   └──────────────────────────────┐
                                  ▼
                           main.py (orchestrator)
                                  │
                                  ▼
                           MAVLink commands
                           → ArduPilot SITL / real FC
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
| 5a | Switch to VisDrone-trained YOLOv8l; auto class-map in detector | Done |
| 5b | Top-down fine-tuning pipeline (VisDrone + synthetic data) | Ready to run |
| 6a | ArduPilot SITL + Isaac Sim JSON bridge (IMU + baro → SITL each step) | Done |
| 6b-i | pymavlink connection to ArduPilot SITL (tcp:localhost:5762) | Done |
| 6b-ii | Disable GPS in SITL; strip position/velocity from JSON bridge (IMU+baro only) | Done |
| 6b-iii | Feed AnyLoc estimates to ArduPilot EKF3 via VISION_POSITION_ESTIMATE | TODO |
| 6b-iv | Send flight commands via SET_POSITION_TARGET_LOCAL_NED (replaces keyboard) | TODO |
| 6c | Read HIGHRES_IMU back from ArduPilot MAVLink → feed localization pipeline | TODO |
| 6d | IMU fusion: AnyLoc anchor validator + VO quality gate using IMU data | TODO |
| 7 | Full pipeline integrated: AnyLoc + VO + IMU → ArduPilot commands | TODO |
| 8 | Deploy to real drone hardware | TODO |
