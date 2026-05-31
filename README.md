# No-GPS Drone Project

Autonomous drone system that localises itself and detects objects without GPS, validated in Isaac Sim before deployment to real hardware.

**Location:** Chiayi, Taiwan — 23.4509°N, 120.2861°E  
**Stack:** Isaac Sim 6.0.0 · AnyLoc · YOLOv8 · ArduPilot · ROS2 Jazzy · MAVROS2

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  control/drone_sim.py          (NEW — replaces stub_bridge.py)  │
│  6-DOF kinematic physics + SITL bridge                          │
│  UDP 9002 ◄──binary servo PWM──► ArduPilot SITL                │
│  Publishes /drone/state (PoseStamped, ENU, 100 Hz)             │
└────────────────────────┬────────────────────────────────────────┘
                         │ /drone/state
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│  simulator/cesium_scene.py     (pure Isaac Sim visualiser)      │
│  Subscribes /drone/state → moves USD drone mesh                 │
│  Publishes /drone/camera/image_raw  /drone/pose  /drone/agl    │
└────────────────────────┬────────────────────────────────────────┘
                         │
             ┌───────────┴──────────────┐
             ▼                          ▼
   anyloc/ros2_node.py        detection/ros2_node.py
   AnyLoc + VO localisation   YOLOv8 vehicle detection
   → /mavros/vision_pose/     → /yolo/detections
     pose_cov (VPE)

                    ArduPilot SITL
                    MAVLink ──UDP 14550──► MAVROS2
                            ──UDP 14551──► flight_commander.py (pymavlink)
                         ▲
                         │ /mavros/vision_pose/pose_cov
                         │   (VISION_POSITION_ESTIMATE → EKF3)
                    MAVROS2
                         ▲
                         │ /mavros/setpoint_position/local
                    flight_commander.py
                    (arm → NAV_TAKEOFF → waypoints → RTL)
```

**Isaac Sim is optional.** `drone_sim.py` + `flight_commander.py` alone is a valid headless configuration.

---

## Repository Layout

```
no_GPS_drone_project/
├── instructions/
│   ├── project_plan.md       # module status, design decisions, milestones
│   └── history.md            # session-by-session change log
├── simulator/                # Isaac Sim — pure 3D visualiser
│   ├── cesium_scene.py       # Cesium terrain + buildings + drone mesh
│   │                         #   subscribes /drone/state, publishes camera + pose
│   └── run_chiayi.sh         # launch script (sources ROS2 Jazzy before conda)
├── anyloc/                   # AnyLoc visual localisation — WORKING
│   ├── build_database.py     # build VLAD database from satellite orthophoto (run once)
│   ├── localizer.py          # AnyLocLocalizer (DINOv2 + VLAD + FAISS)
│   ├── vo_refiner.py         # VORefiner (LK optical flow)
│   ├── ros2_node.py          # ROS2: pub /anyloc/pose_estimate + /mavros/vision_pose/pose_cov
│   ├── run_ros2_localizer.sh # launch script (sources ROS2, runs in conda env)
│   └── database/             # 2821-entry VLAD database (49152-dim, 50 m grid)
├── detection/                # YOLO — WORKING
│   ├── detector.py           # YOLODetector (auto-detects COCO / VisDrone class maps)
│   ├── ros2_node.py          # ROS2: sub /drone/camera → pub /yolo/detections
│   └── run_ros2_detector.sh  # launch script
├── control/                  # Flight control
│   ├── drone_sim.py          # ★ 6-DOF kinematic physics + SITL bridge (NEW)
│   │                         #   publishes /drone/state; replaces stub_bridge.py
│   ├── sitl_bridge.py        # UDP :9002 server — binary servo in, JSON physics out
│   ├── stub_bridge.py        # DEPRECATED — use drone_sim.py
│   ├── flight_commander.py   # ROS2: GUIDED arm → NAV_TAKEOFF → waypoints → RTL
│   ├── launch_mavros.sh      # MAVROS2 on UDP 14550
│   ├── no_gps.parm           # SITL params: GPS_TYPE=0, EK3_SRC1_POSXY=6, VISO_TYPE=1
│   ├── mavlink_ctrl.py       # legacy pymavlink controller (non-ROS2 fallback)
│   ├── run_flight.py         # legacy pymavlink flight script
│   └── run_vision.py         # legacy standalone vision bridge
├── yolov8l_visdrone.pt       # YOLOv8l pre-trained on VisDrone (active model)
├── third_party/
│   └── ardupilot/            # ArduPilot source — SITL binary at build/sitl/bin/arducopter
└── main.py                   # top-level orchestrator — TODO
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
| 6b-ii | Disable GPS; strip position from JSON bridge | Done |
| 6b-iii | AnyLoc → ArduPilot EKF3 via VISION_POSITION_ESTIMATE | Done |
| 6b-iv | Flight commands via SET_POSITION_TARGET | Done |
| 6e | ROS2 migration: all IPC via topics + MAVROS2 | Done |
| 6f | Separate drone physics process from Isaac Sim (drone_sim.py) | Done |
| 6g | Fix VPE: correct ENU x/y order + covariance for EKF POS_ABS | Done |
| 6c | HIGHRES_IMU from ArduPilot → localization pipeline | TODO |
| 6d | IMU fusion: AnyLoc anchor validator + VO quality gate | TODO |
| 7 | Full pipeline integrated in simulation | TODO |
| 8 | Deploy to real hardware | TODO |

---

## Port Map

| Port | Protocol | Owner | Direction |
|------|----------|-------|-----------|
| TCP 5760 | MAVLink | MAVProxy ↔ ArduPilot SITL | internal |
| UDP 9002 | JSON SITL | drone_sim.py ↔ ArduPilot | physics bridge |
| UDP 14550 | MAVLink | MAVROS2 listens | MAVProxy → MAVROS2 |
| UDP 14551 | MAVLink | flight_commander.py listens | MAVProxy → pymavlink |

---

## Quickstart

### Requirements

- Isaac Sim 6.0.0 (Kit 106, Python 3.12) — optional for headless flight
- conda env `isaac_sim_test`
- Display (X11 or virtual framebuffer, e.g. `DISPLAY=:2`) — only for Isaac Sim
- ROS2 Jazzy + MAVROS2
  ```bash
  sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs
  sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
  ```
- Python packages: `pymavlink`, `mavproxy`
  ```bash
  pip3 install --user --break-system-packages pymavlink mavproxy
  ```

### Build ArduPilot SITL (one-time)

```bash
cd third_party/ardupilot
git submodule update --init --depth=1 --recursive
python3 waf configure --board sitl && python3 waf copter
cd ../..
```

---

## Run order — headless (no Isaac Sim)

```bash
# T1 — ArduPilot SITL
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild \
    -l 23.450868,120.286135,28.17,0 \
    --add-param-file=control/no_gps.parm --wipe \
    --out udp:127.0.0.1:14550 \
    --out udp:127.0.0.1:14551

# T2 — Drone physics + SITL bridge (start within ~10 s of SITL)
source /opt/ros/jazzy/setup.bash
python3 control/drone_sim.py

# T3 — MAVROS2
bash control/launch_mavros.sh

# T4 — Flight commander
source /opt/ros/jazzy/setup.bash
python3 control/flight_commander.py
```

## Run order — full (with Isaac Sim)

```bash
# T1 — Isaac Sim visualiser (start first — writes home_elevation.json)
cd simulator && ./run_chiayi.sh

# T2 — ArduPilot SITL
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild \
    -l 23.450868,120.286135,28.17,0 \
    --add-param-file=control/no_gps.parm --wipe \
    --out udp:127.0.0.1:14550 \
    --out udp:127.0.0.1:14551

# T3 — Drone physics + SITL bridge
source /opt/ros/jazzy/setup.bash
python3 control/drone_sim.py

# T4 — MAVROS2
bash control/launch_mavros.sh

# T5 — AnyLoc localiser
./anyloc/run_ros2_localizer.sh

# T6 — Flight commander
source /opt/ros/jazzy/setup.bash
python3 control/flight_commander.py
```

### First-run SITL note

After loading `no_gps.parm` with `--wipe`, type `reboot` in the MAVProxy console once and wait for "Saved N params". `VISO_TYPE` and `SCHED_LOOP_RATE` require a second boot to activate. Drop `--wipe` on subsequent runs (params persist in `eeprom.bin`).

---

## Key design decisions

### SITL bridge (UDP 9002)
`drone_sim.py` owns the physics bridge. ArduPilot sends binary `servo_packet_16` (40 bytes, magic=18458); bridge replies with JSON physics state terminated by `\n`. `"position"` is intentionally absent (GPS substitute); `"velocity"` is present (required by ArduPilot JSON parser).

### VPE (vision position estimate)
`flight_commander.py` publishes `PoseWithCovarianceStamped` to `/mavros/vision_pose/pose_cov`:
- **frame**: `"map"` (ENU) — x = East, y = North, z = Up
- **x/y covariance**: 1 m² — must be ≤ ~5 m² for EKF to set `EKF_POS_HORIZ_ABS`
- **z covariance**: 1e6 m² — EKF ignores VPE altitude, uses barometer
- MAVROS2 converts to `VISION_POSITION_ESTIMATE` → ArduPilot EKF3

### Takeoff sequence
1. Arm in STABILIZE (bypasses GPS/VisOdom pre-arm checks)
2. Switch to GUIDED
3. Wait for `EKF_POS_HORIZ_ABS` flag (VPE must reach EKF3)
4. Send `MAV_CMD_NAV_TAKEOFF` — **required** to break ArduPilot out of "landed" state; position setpoints alone are insufficient
5. Rate-limited position setpoint ramp to 90 m AGL

### no_gps.parm highlights
| Param | Value | Reason |
|-------|-------|--------|
| `GPS_TYPE` | 0 | disable GPS driver |
| `EK3_SRC1_POSXY` | 6 | ExternalNav horizontal position |
| `EK3_SRC1_POSZ` | 1 | barometer altitude |
| `VISO_TYPE` | 1 | MAVLink vision odometry |
| `FS_GPS_ENABLE` | 0 | no GPS failsafe GUIDED→LAND |
| `ARMING_CHECK` | 0 | skip pre-arm (SITL only) |
| `MOT_THST_HOVER` | 0.5 | kinematic hover PWM = 1500 |

---

## Monitor topics

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /drone/state              # 100 Hz from drone_sim.py
ros2 topic hz /drone/camera/image_raw  # ~6 Hz from cesium_scene.py
ros2 topic echo /mavros/state          # armed, mode, connected
ros2 topic echo /mavros/vision_pose/pose_cov  # VPE flowing to EKF3
```

---

## Data Sources

| Layer | Source | License |
|-------|--------|---------|
| Terrain | Cesium World Terrain (asset 1) | © Cesium ion |
| Buildings | Cesium OSM Buildings (asset 96188) | © OpenStreetMap contributors (ODbL) |
| Imagery | Taiwan NLSC PHOTO2 orthophoto WMTS | © 內政部國土測繪中心 |
