# No-GPS Drone Project — Plan

## Goal

Build a drone system that can localize itself and detect objects without GPS, using visual place recognition (AnyLoc), object detection (YOLO), and ArduPilot for flight control. The full pipeline is validated in Isaac Sim before deploying to real hardware.

---

## Project Structure

```
no_GPS_drone_project/
├── instructions/               # Plans, notes, contest references
├── simulator/
│   ├── cesium_scene.py         # Pure Isaac Sim visualiser — subscribes /drone/state
│   │                           #   publishes /drone/camera/image_raw + /drone/pose + /drone/agl
│   └── run_chiayi.sh           # Launch: sources ROS2 Jazzy, runs in conda env
├── anyloc/
│   ├── build_database.py       # Build VLAD database (run once)
│   ├── localizer.py            # AnyLocLocalizer (DINOv2 + VLAD + FAISS)
│   ├── vo_refiner.py           # VORefiner (LK optical flow)
│   ├── ros2_node.py            # ROS2: sub camera/pose → pub VPE + detections
│   └── run_ros2_localizer.sh   # Launch script
├── detection/
│   ├── detector.py             # YOLODetector (auto class-map)
│   └── ros2_node.py            # ROS2: sub /drone/camera → pub /yolo/detections
├── control/
│   ├── drone_sim.py            # ★ 6-DOF kinematic physics + SITL bridge (UDP 9002)
│   │                           #   publishes /drone/state (PoseStamped, ENU, 100 Hz)
│   ├── sitl_bridge.py          # UDP :9002 server — binary servo in → JSON physics out
│   ├── stub_bridge.py          # DEPRECATED — use drone_sim.py
│   ├── flight_commander.py     # ROS2: arm → NAV_TAKEOFF → waypoints → RTL
│   ├── launch_mavros.sh        # MAVROS2 on UDP 14550 (fcu_url=udp://:14550@)
│   ├── no_gps.parm             # SITL: GPS_TYPE=0, EK3_SRC1_POSXY=6, VISO_TYPE=1
│   ├── mavlink_ctrl.py         # Legacy pymavlink controller
│   ├── run_flight.py           # Legacy pymavlink flight script
│   └── run_vision.py           # Legacy standalone vision bridge
└── main.py                     # Top-level orchestrator (TODO)
```

---

## System Architecture

```
ArduPilot SITL  ◄─UDP 9002─►  control/drone_sim.py
                               (6-DOF kinematic physics)
                               publishes /drone/state
                                         │
                               simulator/cesium_scene.py
                               (pure visualiser — optional)
                               publishes /drone/camera/image_raw
                                         │
                               anyloc/ros2_node.py
                               → /mavros/vision_pose/pose_cov (VPE)

ArduPilot SITL
  ─UDP 14550──► MAVROS2  ◄──/mavros/vision_pose/pose_cov (EKF3 fusion)
                │          └── /uas1/mavlink_source (BEST_EFFORT) → flight_commander.py
                │              (EKF origin confirm, EKF status flags, motor PWM)
                ├─ /mavros/global_position/set_gp_origin → MAVROS2 → SET_GPS_GLOBAL_ORIGIN
                ├─ /mavros/setpoint_position/local  → MAVROS2 → ArduPilot
                └─ MAV_CMD_NAV_TAKEOFF              → MAVROS2 → ArduPilot
```

### Port map

| Port | Protocol | Owner |
|------|----------|-------|
| TCP 5760 | MAVLink | MAVProxy ↔ ArduPilot SITL (internal; single client only) |
| UDP 9002 | JSON SITL | drone_sim.py ↔ ArduPilot physics |
| UDP 14550 | MAVLink | MAVROS2 listens (MAVProxy → MAVROS2) |

---

## Modules

### 1. Simulator (`simulator/cesium_scene.py`)

**Status:** Working — pure visualiser; drone position driven by `/drone/state`

Isaac Sim 6.0.0 scene centred on Chiayi, Taiwan (23.4509°N, 120.2861°E).

- **Terrain:** Cesium World Terrain (asset 1) — quantized-mesh-1.0, 9 tiles at level 13
- **Imagery:** Taiwan NLSC PHOTO2 orthophoto WMTS (zoom 18, resized to 4096×4096)
- **Buildings:** Cesium OSM Buildings (asset 96188) — B3DM, ~83 buildings
- **Drone mesh:** `/World/Drone` Xform — position driven by `/drone/state` from `drone_sim.py`
- **Camera:** `/World/Drone/Camera` — nadir, 18 mm / 36×27 mm, 90°×73.7° FOV, 640×480
- **HUD:** `omni.ui` overlay showing live LAT / LON / ALT MSL / AGL
- **Frame output:** `drone_frames/latest.jpg` + `latest_meta.json` every 5 sim steps
- **ROS2 publishers:** `/drone/camera/image_raw`, `/drone/pose` (WGS84), `/drone/agl`
- **ROS2 subscriber:** `/drone/state` (ENU PoseStamped from `drone_sim.py`)

Keyboard controls: **removed** (drone is ArduPilot-commanded via `drone_sim.py`).

### 2. Drone Simulator (`control/drone_sim.py`)

**Status:** Done — replaces `stub_bridge.py` with full 6-DOF + ROS2

Standalone ROS2 node running 6-DOF kinematic physics and the ArduPilot SITL bridge.

- Reads `control/home_elevation.json` (written by `cesium_scene.py`); falls back to 28.17 m
- Physics loop at 100 Hz: integrates PWM → thrust → roll/pitch → NED acceleration → ENU position
- `SITLBridge` on UDP 9002: sends JSON physics state to ArduPilot, receives servo PWM
- Publishes `/drone/state` (`PoseStamped`, `frame_id="local_enu"`, 100 Hz)
- **Isaac Sim is NOT required** — `drone_sim.py` + `flight_commander.py` is a valid headless config

Run:
```bash
source /opt/ros/jazzy/setup.bash
python3 control/drone_sim.py
```

### 3. Localisation (`anyloc/`)

**Status:** Working — AnyLoc + VO; 2,821-entry database; ~15–20 m anchor error

(Architecture unchanged — see previous plan for full details.)

Run:
```bash
./anyloc/run_ros2_localizer.sh
```

### 4. Object Detection (`detection/`)

**Status:** Working — `yolov8l_visdrone.pt` (VisDrone-trained), 10 aerial classes

(Architecture unchanged — see previous plan for full details.)

### 5. Flight Control (`control/flight_commander.py`)

**Status:** Arming + EKF POS_ABS working; takeoff under investigation

ROS2 node. No pymavlink — uses MAVROS2 raw MAVLink exclusively. Full arming and flight sequence:

1. Start VPE thread — Phase 1: home position (0,0) at 0.1 m² cov; Phase 2: AnyLoc above 50 m AGL
2. Wait for MAVROS2 connection (`/mavros/state.connected`)
3. Publish EKF origin to `/mavros/global_position/set_gp_origin`; confirm via GPS_GLOBAL_ORIGIN (msg 49) on `/uas1/mavlink_source`
4. Arm in STABILIZE (bypasses GPS/VisOdom pre-arm; only needs IMU attitude)
5. Switch to GUIDED
6. Wait for `EKF_POS_HORIZ_ABS` (bit 4) from EKF_STATUS_REPORT (msg 193) on `/uas1/mavlink_source` — flags at byte offset 20
7. Send `MAV_CMD_NAV_TAKEOFF` (breaks "landed" state — position setpoints alone are insufficient)
8. Rate-limited position setpoint ramp to 90 m AGL at 1 m/s (prints motor PWM alongside AGL for diagnostics)
9. Square waypoint pattern → RTL

**VPE covariance design:**
- `frame_id = "map"` (ENU): x = East, y = North, z = Up
- Phase 1 x/y covariance = **0.1 m²** (EKF sets POS_HORIZ_ABS immediately at ground)
- Phase 2 x/y covariance = **max(1, error_m²)** (scales with AnyLoc confidence)
- z covariance = **1e6 m²** (EKF ignores VPE altitude, uses barometer)

**MAVROS2 (UDP):**
- `launch_mavros.sh` uses `fcu_url:="udp://:14550@"` (binds local port 14550)
- MAVProxy sends to 14550 via `--out udp:127.0.0.1:14550`
- `/mavros/estimator_status` is advertised but publishes no messages in MAVROS2 Jazzy 2.14; use `/uas1/mavlink_source` instead

**no_gps.parm highlights:**

| Param | Value | Reason |
|-------|-------|--------|
| `GPS_TYPE` | 0 | disable GPS |
| `EK3_SRC1_POSXY` | 6 | ExternalNav horizontal position |
| `EK3_SRC1_POSZ` | 1 | barometer altitude |
| `EK3_SRC1_YAW` | 6 | ExternalNav yaw |
| `VISO_TYPE` | 1 | MAVLink vision odometry |
| `FS_GPS_ENABLE` | 0 | no GPS failsafe |
| `ARMING_CHECK` | 0 | skip pre-arm (SITL only) |
| `MOT_THST_HOVER` | 0.5 | kinematic hover PWM = 1500 |
| `SCHED_LOOP_RATE` | 50 | matches Isaac Sim frame rate |

---

## Run order

### Headless (no GPU, no Isaac Sim)

```bash
# T1 — ArduPilot SITL
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild \
    -l 23.450868,120.286135,28.17,0 \
    --add-param-file=control/no_gps.parm --wipe \
    --out udp:127.0.0.1:14550

# T2 — Drone physics (start within ~10 s of SITL)
source /opt/ros/jazzy/setup.bash && python3 control/drone_sim.py

# T3 — MAVROS2
bash control/launch_mavros.sh

# T4 — Flight commander
source /opt/ros/jazzy/setup.bash && python3 control/flight_commander.py
```

> **Note:** Restart all three (SITL + drone_sim + MAVROS2) after any failed run.

### Full (with Isaac Sim)

```bash
# T1 — Isaac Sim (writes home_elevation.json; start first)
cd simulator && ./run_chiayi.sh

# T2 — ArduPilot SITL
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild \
    -l 23.450868,120.286135,28.17,0 \
    --add-param-file=control/no_gps.parm --wipe \
    --out udp:127.0.0.1:14550

# T3 — Drone physics
source /opt/ros/jazzy/setup.bash && python3 control/drone_sim.py

# T4 — MAVROS2
bash control/launch_mavros.sh

# T5 — AnyLoc
./anyloc/run_ros2_localizer.sh

# T6 — Flight commander
source /opt/ros/jazzy/setup.bash && python3 control/flight_commander.py
```

---

## Milestones

| # | Milestone | Status |
|---|-----------|--------|
| 1 | Isaac Sim scene: Cesium terrain + NLSC imagery + OSM buildings | Done |
| 2 | Virtual drone + nadir camera + HUD publishing frames | Done |
| 3 | AnyLoc database built from simulated views | Done |
| 4 | AnyLoc localisation working + dual postview | Done |
| 5 | YOLO detection working on simulated frames | Done |
| 5a | VisDrone-trained YOLOv8l + auto class-map | Done |
| 5b | Top-down fine-tuning pipeline | Ready to run |
| 6a | ArduPilot SITL + JSON bridge (IMU + baro) | Done |
| 6b-i | pymavlink connection to ArduPilot | Done |
| 6b-ii | Disable GPS; strip position from bridge | Done |
| 6b-iii | AnyLoc → ArduPilot EKF3 via VPE | Done |
| 6b-iv | Flight commands via SET_POSITION_TARGET | Done |
| 6e | ROS2 migration: all IPC via topics + MAVROS2 | Done |
| 6f | Separate drone physics (drone_sim.py) from Isaac Sim | Done |
| 6g | Fix VPE: ENU coordinate order + 1 m² covariance for EKF POS_ABS | Done |
| 6h | Remove pymavlink; EKF origin + status via MAVROS2 raw MAVLink; two-phase VPE | Done (takeoff pending) |
| 6c | HIGHRES_IMU from ArduPilot → localization pipeline | TODO |
| 6d | IMU fusion: AnyLoc anchor validator + VO quality gate | TODO |
| 7 | Full pipeline: AnyLoc + VO + IMU → ArduPilot commands | TODO |
| 8 | Deploy to real drone hardware | TODO |
