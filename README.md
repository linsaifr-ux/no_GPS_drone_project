# No-GPS Drone Project

Autonomous drone system that localises itself and detects objects without GPS, validated in Isaac Sim before deployment to real hardware.

**Location:** Chiayi, Taiwan — 23.4509°N, 120.2861°E  
**Stack:** Isaac Sim 6.0.0 · AnyLoc · YOLOv8 · ArduPilot MAVLink

---

## Pipeline

```
Isaac Sim (cesium_scene.py)
    │ physics JSON  ◄──binary servo PWM──┐
    ▼                                    │
control/sitl_bridge.py              ArduPilot SITL
  (UDP server :9002)  ──JSON+\n──►  (JSON client)
                                         │ MAVLink TCP:5762
                              ┌──────────┴──────────────────┐
                              ▼                             ▼
                      HIGHRES_IMU                  EKF_STATUS_REPORT
                      → imu_fusion.py              (position valid?)

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
│   ├── sitl_bridge.py     #   UDP server :9002 — receives binary servo PWM, replies physics JSON (DONE)
│   ├── stub_bridge.py     #   kinematic drone stub for testing without Isaac Sim
│   ├── mavlink_ctrl.py    #   MAVLinkCtrl: recv loop + vision + mode + arm + waypoint helpers
│   ├── run_mavlink.py     #   live terminal monitor: attitude, NED pos, IMU, EKF flags
│   ├── run_vision.py      #   standalone vision bridge (use run_flight.py for combined operation)
│   ├── run_flight.py      #   arm → takeoff → waypoints → RTL + vision thread (6b-iv)
│   ├── no_gps.parm        #   SITL param file: GPS_TYPE=0, EK3_SRC1_POSXY=6, VISO_TYPE=1
│   ├── imu_reader.py      #   HIGHRES_IMU reader from MAVLink (TODO 6c)
│   └── imu_fusion.py      #   AnyLoc anchor validator + VO quality gate (TODO 6d)
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
| 6b-i | pymavlink connection to ArduPilot MAVLink output | Done |
| 6b-ii | Disable GPS; strip position from JSON bridge (IMU+baro only) | Done |
| 6b-iii | AnyLoc → ArduPilot EKF3 via VISION_POSITION_ESTIMATE | Done |
| 6b-iv | Flight commands via SET_POSITION_TARGET (replaces keyboard) | In progress |
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
# Terminal 1 — FIRST RUN (or after changing no_gps.parm): flush EEPROM with --wipe
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild --console --map \
    -l 23.450868,120.286135,<centre_elev>,0 \
    --add-param-file=control/no_gps.parm --wipe
# Wait for "Saved N params" in MAVProxy console, then type: reboot
# (VISO_TYPE and SCHED_LOOP_RATE require a second boot to take effect)

# Terminal 1 — SUBSEQUENT RUNS (params already in EEPROM):
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild --console --map \
    -l 23.450868,120.286135,<centre_elev>,0

# Terminal 2 — Isaac Sim (bridge auto-connects on first step)
cd simulator && ./run_chiayi.sh
```

The bridge (`control/sitl_bridge.py`) is a UDP server embedded in the Isaac Sim loop.
ArduPilot sends a **binary** `servo_packet_16` (40 bytes, little-endian, magic=18458) to port 9002;
the bridge parses it, learns ArduPilot's reply address from the source port, and sends back a
JSON physics state terminated by `\n` each step.
"No JSON sensor message received, resending servos" is normal until Isaac Sim finishes loading (~2–5 min).

Key `no_gps.parm` parameters: `GPS_TYPE=0` (disable GPS), `EK3_SRC1_POSXY=6` (ExtNav position from VPE),
`VISO_TYPE=1` (enable vision odometry driver), `FS_GPS_ENABLE=0` (prevent GPS failsafe GUIDED→LAND after arming),
`FENCE_ENABLE=0` (prevent geofence blocking flight near origin).

To test MAVLink without Isaac Sim, use the kinematic stub bridge (prints physics state at 1 Hz for cross-checking):

```bash
# Terminal 2 — stub (kinematic altitude model, responds to ArduPilot thrust)
python3 control/stub_bridge.py
```

### Run the flight sequence (separate terminal)

```bash
python3 control/run_flight.py
```

Handles vision position and flight commands in one process on a single MAVLink connection
(`tcp:localhost:5762`). No second TCP port or `run_vision.py` needed.

Sequence: waits for EKF POS_ABS → GUIDED mode → arm → takeoff → waypoints → RTL.

Vision sending (`VISION_POSITION_ESTIMATE` at 5 Hz) runs in a background thread, feeding
EKF3 from `anyloc/latest_estimate.json`. If the file doesn't exist a stub estimate at home
is created automatically so the pipeline works without `run_localizer.py`.

Requires SITL launched with `--add-param-file=control/no_gps.parm` so that
`EK3_SRC1_POSXY=6` (ExtNav) and `VISO_TYPE=1` are set.

`run_vision.py` is kept as a standalone alternative when testing vision fusion without flying.

---

### Monitor MAVLink state (separate terminal)

```bash
python3 control/run_mavlink.py
```

Connects to SITL on `tcp:localhost:5762` (direct, no mavproxy needed).
Prints a live rolling line at 10 Hz showing attitude, NED position, IMU accelerations,
and EKF status flags. Start after SITL + bridge are both running; waits up to 60 s for HEARTBEAT.

```
    TIME    ROLL°    PCH°    YAW°          N m          E m          D m       Ax      Ay      Az  EKF flags
-------- ------- ------- -------  --------- --------- ---------  ------- ------- -------  ---------
  1234.5    0.00    0.00    0.00       0.00       0.00      -5.00     0.00    0.00   -9.81  0x0400 UNINIT
  1235.0    0.01   -0.02    0.00       0.00       0.00      -5.00     0.01   -0.01   -9.81  0x0001 ATT
  1240.0    0.01   -0.02   90.00       0.12       0.05      -9.87     0.01   -0.01   -9.81  0x003f ATT,VEL,POS_ABS
```

Expected EKF progression after bridge connects:
- `UNINIT` (0x0400) — normal at startup; EKF hasn't initialised yet
- `ATT` — IMU tilt alignment complete (~5–10 s)
- `ATT,VEL_H,VEL_V,ALT` + bit 7 (`CONST_POS_MODE`) — bridge running but no VPE yet; N/E/D show `nan`
- `ATT,VEL,ALT,POS_ABS` — VPE fused; N/E/D populate; flight commands accepted

---

## Data Sources

| Layer | Source | License |
|-------|--------|---------|
| Terrain | Cesium World Terrain (asset 1) | © Cesium ion |
| Buildings | Cesium OSM Buildings (asset 96188) | © OpenStreetMap contributors (ODbL) |
| Imagery | Taiwan NLSC PHOTO2 orthophoto WMTS | © 內政部國土測繪中心 |
