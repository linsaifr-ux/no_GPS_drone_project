# No-GPS Drone Project

Autonomous drone system that localises itself and detects objects without GPS, validated in Isaac Sim before deployment to real hardware.

**Location:** Chiayi, Taiwan — 23.4509°N, 120.2861°E  
**Stack:** Isaac Sim 6.0.0 · AnyLoc · YOLOv8 · ArduPilot MAVLink

---

## Pipeline

```
Isaac Sim (cesium_scene.py)
    │ /drone/camera/image_raw  (sensor_msgs/Image, ROS2)
    │ /drone/pose              (geometry_msgs/PoseStamped, ROS2)
    │ /drone/agl               (std_msgs/Float64, ROS2)
    │ physics JSON  ◄──binary servo PWM──┐
    ▼                                    │
control/sitl_bridge.py              ArduPilot SITL
  (UDP server :9002)  ──JSON+\n──►  (JSON client)
                                         │ MAVLink TCP:5762
                                         ▼
                                      MAVROS2
                              ┌──────────┴──────────────────┐
                              ▼                             ▼
                    /mavros/imu/data_raw         /mavros/state
                    /mavros/local_position/pose  /mavros/ekf_status

/drone/camera/image_raw
    │
    ├──► anyloc/ros2_node.py ──/mavros/vision_pose/pose──► MAVROS2 ──► ArduPilot EKF3
    │    (AnyLoc + VO)            (VISION_POSITION_ESTIMATE)            (no-GPS fusion)
    │
    └──► detection/ros2_node.py ──/yolo/detections──► (mission planner)
         (YOLOv8)

control/flight_commander.py
    │ /mavros/setpoint_position/local
    ▼
MAVROS2 ──SET_POSITION_TARGET_LOCAL_NED──► ArduPilot SITL / real FC
```

---

## Repository Layout

```
no_GPS_drone_project/
├── instructions/
│   ├── project_plan.md    # module status, design decisions, milestones
│   └── history.md         # session-by-session change log
├── simulator/             # Isaac Sim scene — WORKING
│   ├── cesium_scene.py    # main scene: publishes /drone/camera/image_raw + /drone/pose via ROS2
│   ├── drone_frames/      # fallback file output (used only when ROS2 not available)
│   └── run_chiayi.sh      # launch script (sources ROS2 Jazzy before starting Isaac Sim)
├── anyloc/                # AnyLoc visual localization — WORKING
│   ├── build_database.py  # build VLAD database from satellite orthophoto (run once)
│   ├── localizer.py       # AnyLocLocalizer (DINOv2 + VLAD + FAISS)
│   ├── vo_refiner.py      # VORefiner (LK optical flow)
│   ├── ros2_node.py       # ROS2 node: dual postview + pub /anyloc/pose_estimate + /mavros/vision_pose/pose
│   ├── run_ros2_localizer.sh  # launch script for ros2_node.py (sources ROS2, uses conda env)
│   ├── run_localizer.py   # legacy file-based dual postview (non-ROS2 fallback)
│   └── database/          # 2821-entry VLAD database (49152-dim, 50 m grid)
├── detection/             # YOLO — WORKING
│   ├── detector.py        # YOLODetector (auto-detects COCO / VisDrone class maps)
│   ├── ros2_node.py       # ROS2 node: sub /drone/camera → pub /yolo/detections
│   ├── run_detector.py    # legacy file-based postview (non-ROS2 fallback)
│   ├── label_writer.py    # nadir projection math for synthetic label generation
│   ├── collect_training_data.py  # Isaac Sim headless synthetic data collector
│   ├── prepare_dataset.py # download VisDrone + remap classes + merge synth data
│   └── finetune.py        # fine-tune YOLOv8 on the top-down dataset
├── yolov8l_visdrone.pt    # YOLOv8l pre-trained on VisDrone (10 aerial classes)
├── yolov8n.pt             # YOLOv8n COCO pretrained (baseline)
├── control/               # ArduPilot + ROS2/MAVROS2 flight control
│   ├── sitl_bridge.py     #   UDP server :9002 — receives binary servo PWM, replies physics JSON
│   ├── stub_bridge.py     #   kinematic drone stub for testing without Isaac Sim
│   ├── launch_mavros.sh   #   start MAVROS2 connected to SITL TCP:5762
│   ├── flight_commander.py #  ROS2 node: GUIDED → arm → takeoff → waypoints → RTL (via MAVROS2)
│   ├── mavlink_ctrl.py    #   legacy pymavlink controller (non-ROS2 fallback)
│   ├── run_flight.py      #   legacy pymavlink flight script (non-ROS2 fallback)
│   ├── run_mavlink.py     #   legacy terminal monitor (replaced by ros2 topic echo)
│   ├── run_vision.py      #   legacy standalone vision bridge (replaced by anyloc/ros2_node.py)
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
| 6b-iv | Flight commands via SET_POSITION_TARGET (replaces keyboard) | Done |
| 6c | HIGHRES_IMU from ArduPilot → localization pipeline | TODO |
| 6d | IMU fusion: AnyLoc anchor validator + VO quality gate | TODO |
| 6e | ROS2 migration: all IPC via topics/MAVROS2 | Done |
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
- ROS2 Jazzy + MAVROS2 (already installed on this machine)
  ```bash
  # one-time geographiclib datasets (needed by MAVROS2)
  sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
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

**ROS2 mode (recommended):**
```bash
./anyloc/run_ros2_localizer.sh
```
Same dual-window postview as the legacy localizer. Subscribes to `/drone/camera/image_raw`,
`/drone/pose`, `/drone/agl`; publishes `/anyloc/pose_estimate` and `/mavros/vision_pose/pose`;
also writes `anyloc/latest_estimate.json` for legacy `run_flight.py` compatibility.

**Legacy file-based mode (fallback, no ROS2):**
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

**ROS2 mode (recommended):**
```bash
source /opt/ros/jazzy/setup.bash
python3 detection/ros2_node.py
```
Publishes detections to `/yolo/detections` (vision_msgs/Detection2DArray).

**Legacy file-based mode (fallback, no ROS2):**
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
    -l 23.450868,120.286135,28.17,0 \
    --add-param-file=control/no_gps.parm --wipe
# Wait for "Saved N params" in MAVProxy console, then type: reboot
# (VISO_TYPE and SCHED_LOOP_RATE require a second boot to take effect)

# Terminal 1 — SUBSEQUENT RUNS (params already in EEPROM):
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild --console --map \
    -l 23.450868,120.286135,28.17,0

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

### Run MAVROS2 (separate terminal)

```bash
bash control/launch_mavros.sh
```

Bridges MAVLink ↔ ROS2. Connects to SITL on `tcp:localhost:5762`.
Key topics provided:
- `/mavros/state` — armed status, flight mode, EKF health
- `/mavros/local_position/pose` — NED position from EKF
- `/mavros/vision_pose/pose` ← feed from `anyloc/ros2_node.py` → `VISION_POSITION_ESTIMATE`
- `/mavros/setpoint_position/local` ← feed from `flight_commander.py` → position commands

### Run the flight sequence (separate terminal)

**ROS2 mode (recommended) — requires MAVROS2 + AnyLoc ROS2 node running:**
```bash
source /opt/ros/jazzy/setup.bash
python3 control/flight_commander.py
```

Sequence:
1. `SET_GPS_GLOBAL_ORIGIN` + `SET_HOME_POSITION` via pymavlink (before VPE arrives)
2. Wait for MAVROS2 connection
3. Wait for EKF position fix (driven by `/mavros/vision_pose/pose` from AnyLoc node)
4. GUIDED → arm → takeoff → waypoints → RTL

**Legacy pymavlink mode (fallback, no MAVROS2):**
```bash
python3 control/run_flight.py
```

Requires SITL launched with `--add-param-file=control/no_gps.parm` so that
`EK3_SRC1_POSXY=6` (ExtNav) and `VISO_TYPE=1` are set.

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

## ROS2 full-pipeline run order

```bash
# Terminal 1 — SITL
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild --console --map \
    -l 23.450868,120.286135,28.17,0 --add-param-file=control/no_gps.parm

# Terminal 2 — physics bridge (or Isaac Sim for full scene)
python3 control/stub_bridge.py
# cd simulator && ./run_chiayi.sh   ← use this instead for full Isaac Sim scene

# Terminal 3 — MAVROS2
bash control/launch_mavros.sh

# Terminal 4 — AnyLoc ROS2 node (opens dual postview window)
./anyloc/run_ros2_localizer.sh

# Terminal 5 — YOLO detection (optional)
source /opt/ros/jazzy/setup.bash && conda run -n isaac_sim_test python3 detection/ros2_node.py

# Terminal 6 — flight commander
source /opt/ros/jazzy/setup.bash && python3 control/flight_commander.py
```

Monitor topics:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /drone/camera/image_raw     # expect ~6 Hz
ros2 topic echo /mavros/vision_pose/pose  # AnyLoc VPE flowing to EKF3
ros2 topic echo /mavros/state             # armed, mode, EKF status
ros2 topic echo /yolo/detections          # vehicle detections
```

---

## Data Sources

| Layer | Source | License |
|-------|--------|---------|
| Terrain | Cesium World Terrain (asset 1) | © Cesium ion |
| Buildings | Cesium OSM Buildings (asset 96188) | © OpenStreetMap contributors (ODbL) |
| Imagery | Taiwan NLSC PHOTO2 orthophoto WMTS | © 內政部國土測繪中心 |
