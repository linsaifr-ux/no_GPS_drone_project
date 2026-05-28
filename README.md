# No-GPS Drone Project

Autonomous drone system that localises itself and detects objects without GPS, validated in Isaac Sim before deployment to real hardware.

**Location:** Chiayi, Taiwan — 23.4509°N, 120.2861°E  
**Stack:** Isaac Sim 6.0.0 · AnyLoc · YOLOv8 · ArduPilot MAVLink

---

## Pipeline

```
Isaac Sim (cesium_scene.py)
    │ IMU + baro JSON  ◄──servo PWM──┐
    ▼                                │
control/sitl_bridge.py          ArduPilot SITL
  (UDP server :9002)  ──physics──►  (JSON client)
                                     │ MAVLink UDP:14550
                              ┌──────┴──────────────────┐
                              ▼                         ▼
                      HIGHRES_IMU                EKF_STATUS_REPORT
                      → imu_fusion.py            (position valid?)

drone_frames/latest.jpg + latest_meta.json
    │
    ├──► AnyLoc + VO  ──VISION_POSITION_ESTIMATE──► ArduPilot EKF3
    │    (position estimate)                        (no-GPS fusion)
    │
    └──► YOLO (bounding boxes)

imu_fusion.py validates AnyLoc anchors using HIGHRES_IMU
    │
    ▼
main.py (orchestrator)
    │ SET_POSITION_TARGET_LOCAL_NED
    ▼
ArduPilot SITL / real FC
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
├── detection/             # YOLO — WORKING
│   ├── detector.py        # YOLODetector (auto-detects COCO / VisDrone class maps)
│   ├── run_detector.py    # live annotated postview
│   ├── label_writer.py    # nadir projection math for synthetic label generation
│   ├── collect_training_data.py  # Isaac Sim headless synthetic data collector
│   ├── prepare_dataset.py # download VisDrone + remap classes + merge synth data
│   └── finetune.py        # fine-tune YOLOv8 on the top-down dataset
├── yolov8l_visdrone.pt    # YOLOv8l pre-trained on VisDrone (10 aerial classes)
├── yolov8n.pt             # YOLOv8n COCO pretrained (baseline)
├── control/               # ArduPilot MAVLink + IMU fusion
│   ├── sitl_bridge.py     #   UDP server :9002 — receives servo PWM, replies physics (DONE)
│   ├── mavlink_ctrl.py    #   pymavlink: VISION_POSITION_ESTIMATE + flight commands (TODO)
│   ├── imu_reader.py      #   HIGHRES_IMU reader from MAVLink (TODO)
│   └── imu_fusion.py      #   AnyLoc anchor validator + VO quality gate (TODO)
├── third_party/
│   └── ardupilot/         #   ArduPilot source — built SITL binary at build/sitl/bin/arducopter
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
| 5 | YOLO detection working on simulated frames | Done |
| 5a | Switch to VisDrone-trained YOLOv8l; auto class-map in detector | Done |
| 5b | Top-down fine-tuning pipeline (VisDrone dataset + synthetic data) | Ready to run |
| 6a | ArduPilot SITL + Isaac Sim JSON bridge (IMU + baro) | Done |
| 6b-i | pymavlink connection to ArduPilot MAVLink output | TODO |
| 6b-ii | Disable GPS; strip position from JSON bridge (IMU+baro only) | TODO |
| 6b-iii | AnyLoc → ArduPilot EKF3 via VISION_POSITION_ESTIMATE | TODO |
| 6b-iv | Flight commands via SET_POSITION_TARGET (replaces keyboard) | TODO |
| 6c | HIGHRES_IMU from ArduPilot → localization pipeline | TODO |
| 6d | IMU fusion: AnyLoc anchor validator + VO quality gate | TODO |
| 7 | Full pipeline integrated in simulation | TODO |
| 8 | Deploy to real hardware | TODO |

---

## Quickstart

### Requirements

- Isaac Sim 6.0.0 (Kit 106, Python 3.12)
- conda env `isaac_sim_test`
- Cesium ion account (token already embedded in `cesium_scene.py`)
- Display (X11 or virtual framebuffer, e.g. `DISPLAY=:2`)
- Python 3 system packages: `pexpect`, `mavproxy`, `pymavlink`, `future`
  ```bash
  pip3 install --user --break-system-packages pexpect mavproxy pymavlink future
  ```

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

### Run the YOLO vehicle detector (separate terminal)

```bash
DISPLAY=:2 conda run -n isaac_sim_test python detection/run_detector.py
```

A matplotlib window shows the live drone frame with bounding boxes overlaid for detected vehicles (car / motorcycle / bus / truck). Title shows vehicle count, inference time, and current drone geo. Each detection is also printed to the terminal.

Currently uses **`yolov8l_visdrone.pt`** — a YOLOv8-large model pre-trained on VisDrone 2019 DET (10 aerial vehicle classes). The detector auto-maps VisDrone class names to the four canonical labels (car, motorcycle, bus, truck) at load time, so it also works with COCO-trained models without code changes.

#### Fine-tune for better top-down accuracy

A full fine-tuning pipeline is included. Run in order:

```bash
# 1. Generate synthetic labeled frames from Isaac Sim (headless, ~10–20 min)
OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test \
    python detection/collect_training_data.py

# 2. Download VisDrone + convert to 4-class YOLO format (~30 min download)
python detection/prepare_dataset.py

# 3. Fine-tune YOLOv8n (GPU recommended, ~2 h for 100 epochs)
python detection/finetune.py
```

The best weights are saved to `detection/runs/topdown_v1/weights/best.pt`. Update `run_detector.py` to use them.

---

### Run ArduPilot SITL (separate terminal)

ArduPilot must be built once before first use:

```bash
# 1. Initialize submodules (one-time, ~5 min)
cd third_party/ardupilot
git submodule update --init --depth=1 --recursive

# 2. Build ArduCopter SITL binary (~2 min)
python3 waf configure --board sitl
python3 waf copter
cd ../..
```

Then start SITL before Isaac Sim:

```bash
# Terminal 1 — SITL (listens for bridge on port 9002, MAVLink on 14550)
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild --console --map \
    --home=23.450868,120.286135,46,0

# Terminal 2 — Isaac Sim (bridge auto-connects on first step)
cd simulator && ./run_chiayi.sh
```

The bridge (`control/sitl_bridge.py`) is a UDP server embedded in the Isaac Sim loop.
ArduPilot sends servo PWM to port 9002; the bridge replies with IMU + baro + attitude each step.
"No JSON sensor message received, resending servos" is normal until Isaac Sim finishes loading.

---

## Data Sources

| Layer | Source | License |
|-------|--------|---------|
| Terrain | Cesium World Terrain (asset 1) | © Cesium ion |
| Buildings | Cesium OSM Buildings (asset 96188) | © OpenStreetMap contributors (ODbL) |
| Imagery | Taiwan NLSC PHOTO2 orthophoto WMTS | © 內政部國土測繪中心 |
