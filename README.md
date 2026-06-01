# No-GPS Drone Project

Autonomous drone system that localises itself and detects objects without GPS, validated in Isaac Sim before deployment to real hardware.

**Location:** Chiayi, Taiwan — 23.4509°N, 120.2861°E  
**Stack:** Isaac Sim 6.0.0 · AnyLoc · YOLOv8 · ArduPilot · ROS2 Jazzy · MAVROS2

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  simulator/cesium_scene.py          (physics + visualiser)          │
│  100 Hz background thread: 6-DOF kinematic model + SITLBridge      │
│  UDP 9002 ◄──binary servo PWM──► ArduPilot SITL                   │
│  Publishes /drone/state (PoseStamped, ENU, 100 Hz)                 │
│  Publishes /drone/camera/image_raw  /drone/pose  /drone/agl        │
└────────────────────────┬────────────────────────────────────────────┘
                         │ /drone/state
             ┌───────────┴──────────────┐
             ▼                          ▼
   anyloc/ros2_node.py        detection/ros2_node.py          flight_commander.py
   AnyLoc + VO localisation   YOLOv8 vehicle detection        (reads /drone/state
   → /mavros/vision_pose/     → /yolo/detections               for AGL truth)
     pose_cov (VPE)

                    ArduPilot SITL
                    MAVLink ──UDP 14550──► MAVROS2
                         ▲                    │ /uas1/mavlink_source (BEST_EFFORT)
                         │ /mavros/vision_pose/pose_cov  │ EKF origin + status
                         │   (VISION_POSITION_ESTIMATE → EKF3)
                    MAVROS2
                         ▲
                         │ /mavros/setpoint_position/local  (NED coords — MAVROS2 passes through)
                    flight_commander.py
                    (arm → NAV_TAKEOFF → waypoints → RTL)
```

**Headless fallback:** `control/drone_sim.py` provides a kinematic physics bridge for runs without Isaac Sim. `drone_sim.py` is not needed when Isaac Sim is running.

---

## Repository Layout

```
no_GPS_drone_project/
├── instructions/
│   ├── project_plan.md       # module status, design decisions, milestones
│   └── history.md            # session-by-session change log
├── simulator/                # Isaac Sim — physics engine + visualiser
│   ├── cesium_scene.py       # Cesium terrain + buildings + 100 Hz kinematic physics
│   │                         #   background thread: kinematic model + SITLBridge (UDP 9002)
│   │                         #   publishes /drone/state (100 Hz) + camera + pose + agl
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
│   ├── drone_sim.py          # Headless-only fallback: kinematic physics + SITL bridge
│   │                         #   publishes /drone/state; NOT used when Isaac Sim runs
│   ├── sitl_bridge.py        # UDP :9002 server — binary servo in, JSON physics out
│   │                         #   used by both cesium_scene.py and drone_sim.py
│   ├── stub_bridge.py        # DEPRECATED
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
| 6h | Remove pymavlink; MAVROS2 raw MAVLink; DISARM_DELAY=0 | Done ✓ |
| 6i | NAV_TAKEOFF replaces P-controller; EKF origin blocking; reduced ATC gains | Done ✓ |
| 6j | Fix altitude runaway (EK3_SRC1_POSZ→6) + fix 90° course error (motor layout) | Done ✓ |
| 6k | Fix waypoint direction inversion (MAVROS2 NED passthrough) + altitude drop | Done ✓ |
| 6l | Integrate kinematic physics into cesium_scene.py (100 Hz thread); eliminate drone_sim.py | Done ✓ |
| 6k-wp | Waypoints clean run — verify with Isaac Sim physics | WIP |
| 6c | HIGHRES_IMU from ArduPilot → localization pipeline | TODO |
| 6d | IMU fusion: AnyLoc anchor validator + VO quality gate | TODO |
| 7 | Full pipeline integrated in simulation | TODO |
| 8 | Deploy to real hardware | TODO |

---

## Port Map

| Port | Protocol | Owner | Direction |
|------|----------|-------|-----------|
| TCP 5760 | MAVLink | MAVProxy ↔ ArduPilot SITL | internal (single client only) |
| UDP 9002 | JSON SITL | cesium_scene.py (or drone_sim.py) ↔ ArduPilot | physics bridge |
| UDP 14550 | MAVLink | MAVROS2 listens | MAVProxy → MAVROS2 |

---

## Quickstart

### Requirements

- Isaac Sim 6.0.0 (Kit 106, Python 3.12) — required for normal operation; optional for headless
- conda env `isaac_sim_test`
- Display (X11 or virtual framebuffer, e.g. `DISPLAY=:2`) — only for Isaac Sim
- ROS2 Jazzy + MAVROS2
  ```bash
  sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs
  sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
  ```
- Python packages: `mavproxy` (pymavlink is no longer used by flight_commander.py)
  ```bash
  pip3 install --user --break-system-packages mavproxy
  ```

### Build ArduPilot SITL (one-time)

```bash
cd third_party/ardupilot
git submodule update --init --depth=1 --recursive
python3 waf configure --board sitl && python3 waf copter
cd ../..
```

---

## Run order — with Isaac Sim (standard)

```bash
# T1 — Isaac Sim (physics + visualiser; writes home_elevation.json; start first)
cd simulator && ./run_chiayi.sh

# T2 — ArduPilot SITL (start after Isaac Sim is showing the scene)
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild \
    -l 23.450868,120.286135,28.17,0 \
    --add-param-file=control/no_gps.parm --wipe \
    --out udp:127.0.0.1:14550

# T3 — MAVROS2
bash control/launch_mavros.sh

# T4 — AnyLoc localiser
./anyloc/run_ros2_localizer.sh

# T5 — Flight commander
source /opt/ros/jazzy/setup.bash
python3 control/flight_commander.py
```

> **Note:** Isaac Sim now owns the physics bridge (UDP 9002). Do **not** run `drone_sim.py` when Isaac Sim is running — both would try to bind UDP 9002.

## Run order — headless (no Isaac Sim)

```bash
# T1 — ArduPilot SITL
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild \
    -l 23.450868,120.286135,28.17,0 \
    --add-param-file=control/no_gps.parm --wipe \
    --out udp:127.0.0.1:14550

# T2 — Drone physics + SITL bridge (start within ~10 s of SITL)
source /opt/ros/jazzy/setup.bash
python3 control/drone_sim.py

# T3 — MAVROS2
bash control/launch_mavros.sh

# T4 — Flight commander
source /opt/ros/jazzy/setup.bash
python3 control/flight_commander.py
```

> **Note:** Restart all processes (SITL + physics + MAVROS2) after every run, always using `--wipe` on SITL. Without `--wipe`, new parameters don't load and old EKF state persists.

### First-run SITL note

After loading `no_gps.parm` with `--wipe`, type `reboot` in the MAVProxy console once and wait for "Saved N params". `VISO_TYPE` and `SCHED_LOOP_RATE` require a second boot to activate. Drop `--wipe` on subsequent runs (params persist in `eeprom.bin`).

---

## Key design decisions

### SITL bridge (UDP 9002)

`control/sitl_bridge.py` owns the bridge protocol. When Isaac Sim runs, `cesium_scene.py` instantiates `SITLBridge` and applies motor thrust forces to the rigid body each frame. When running headless, `drone_sim.py` does the same with a kinematic model.

ArduPilot sends binary `servo_packet_16` (40 bytes, magic=18458); the bridge replies with JSON physics state terminated by `\n`. `"position"` is intentionally absent (GPS substitute); `"velocity"` is present (required by ArduPilot JSON parser).

### Isaac Sim physics (cesium_scene.py)

`cesium_scene.py` runs a **100 Hz background thread** (`_run_physics`) containing the same 6-DOF kinematic model previously in `drone_sim.py`, plus the `SITLBridge`. The render loop (~13 Hz) only reads the current state and updates the drone mesh.

Physics thread each step (100 Hz):
1. Call `bridge.step()` → send kinematic state to ArduPilot SITL, drain latest PWM
2. Integrate kinematic model: PWM → mean thrust + motor differential → roll/pitch target → NED velocity → ENU position
3. Ground constraint: zero horizontal velocity when z ≤ terrain (friction — prevents spool-up sliding)
4. Publish `/drone/state` at 100 Hz

Motor layout (ArduCopter X-frame FRAME_TYPE=1): ch1=FR(NE), ch2=RL(SW), ch3=RR(SE), ch4=FL(NW). Hover at PWM 1500 (p_norm=0.5) matches `MOT_THST_HOVER=0.5`.

**Why 100 Hz matters:** Isaac Sim renders at ~13 fps. If the physics + bridge ran in the render loop, ArduPilot would see 13 Hz physics replies. At 13 Hz, the altitude PID I-term accumulates too aggressively and the drone oscillates between ground and ~4 m AGL indefinitely. At 100 Hz (background thread) the control loop is stable.

### MAVROS2 setpoint coordinate convention

MAVROS2 Jazzy's `setpoint_position/local` plugin passes `PoseStamped` x,y,z directly into `SET_POSITION_TARGET_LOCAL_NED` **without** ENU→NED axis conversion (the `vision_pose` plugin does convert correctly). Position setpoints in `flight_commander.py` therefore send **NED coordinates** directly:
- `x = north`, `y = east`, `z = down` (negative value = altitude)

### VPE (vision position estimate)

`flight_commander.py` publishes `PoseWithCovarianceStamped` to `/mavros/vision_pose/pose_cov`. Two-phase strategy:
- **Phase 1 (below 50 m AGL):** position = kinematic ENU from `/drone/state`, cov_xy = 0.1 m²
- **Phase 2 (above 50 m AGL):** position = AnyLoc estimate, cov_xy = max(1, error_m²)
- **frame**: `"map"` (ENU) — the vision_pose plugin converts ENU→NED correctly
- **z**: kinematic AGL from `/drone/state`; cov_z = 0.25 m² (0.5 m std dev)
- `EK3_SRC1_POSZ = 6` routes VPE z to EKF altitude

### EKF origin and status (no pymavlink)

`flight_commander.py` uses MAVROS2 raw MAVLink exclusively:
- **EKF origin**: published to `/mavros/global_position/set_gp_origin`; confirmed when GPS_GLOBAL_ORIGIN (msg 49) arrives on `/uas1/mavlink_source`
- **EKF status**: EKF_STATUS_REPORT (msg 193) decoded from `/uas1/mavlink_source` (BEST_EFFORT QoS); flags at byte offset 20

### Takeoff sequence

1. Start VPE thread (Phase 1: kinematic position stub at 20 Hz)
2. Set EKF global origin — block up to 60 s; abort if unconfirmed
3. Arm in STABILIZE (bypasses GPS/VisOdom pre-arm checks)
4. Switch to GUIDED
5. Wait for `EKF_POS_HORIZ_ABS` flag (VPE accepted by EKF3)
6. Send `MAV_CMD_NAV_TAKEOFF` — monitor `/drone/state` AGL; abort if still on ground after 30 s
7. Hold 5 s at takeoff altitude (send current-position setpoints in NED to maintain altitude)
8. Send waypoint position setpoints in NED

### no_gps.parm highlights

| Param | Value | Reason |
|-------|-------|--------|
| `GPS_TYPE` | 0 | disable GPS driver |
| `EK3_SRC1_POSXY` | 6 | ExternalNav horizontal position |
| `EK3_SRC1_POSZ` | 6 | ExternalNav altitude (VPE z = kinematic AGL; barometer unreliable in SIM_JSON) |
| `VISO_TYPE` | 1 | MAVLink vision odometry |
| `FS_GPS_ENABLE` | 0 | no GPS failsafe GUIDED→LAND |
| `ARMING_CHECK` | 0 | skip pre-arm (SITL only) |
| `MOT_THST_HOVER` | 0.5 | hover at PWM 1500 (kinematic model matches this) |
| `DISARM_DELAY` | 0 | enables NAV_TAKEOFF path (removes land-detector deadlock) |
| `WPNAV_SPEED` | 100 | 1 m/s horizontal max |
| `ATC_ANG_RLL/PIT_P` | 1.5 | gentler angle correction (default 4.5 → I-term windup) |

---

## Monitor topics

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /drone/state              # physics rate from cesium_scene.py (or drone_sim.py)
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
