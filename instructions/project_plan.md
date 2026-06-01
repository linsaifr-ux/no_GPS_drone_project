# No-GPS Drone Project — Plan

## Goal

Build a drone system that can localize itself and detect objects without GPS, using visual place recognition (AnyLoc), object detection (YOLO), and ArduPilot for flight control. The full pipeline is validated in Isaac Sim before deploying to real hardware.

---

## Project Structure

```
no_GPS_drone_project/
├── instructions/               # Plans, notes, contest references
├── simulator/
│   ├── cesium_scene.py         # Physics engine + visualiser
│   │                           #   100 Hz background thread: 6-DOF kinematic model + SITLBridge (UDP 9002)
│   │                           #   render loop (~13 Hz): reads state, updates mesh, captures camera
│   │                           #   publishes /drone/state (100 Hz) + /drone/camera/image_raw + /drone/pose + /drone/agl
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
│   ├── drone_sim.py            # Headless-only fallback: kinematic physics + SITL bridge
│   │                           #   publishes /drone/state; NOT used when Isaac Sim runs
│   ├── sitl_bridge.py          # UDP :9002 server — binary servo in → JSON physics out
│   │                           #   shared by cesium_scene.py and drone_sim.py
│   ├── stub_bridge.py          # DEPRECATED
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
simulator/cesium_scene.py
  100 Hz background thread: kinematic model + SITLBridge
  UDP 9002 ◄──PWM──► ArduPilot SITL
  publishes /drone/state (ENU PoseStamped, 100 Hz)
            │
            ├── flight_commander.py  (reads AGL truth from /drone/state)
            │
            └── anyloc/ros2_node.py
                → /mavros/vision_pose/pose_cov (VPE, ENU via vision_pose plugin)

ArduPilot SITL
  ─UDP 14550──► MAVROS2  ◄── /mavros/vision_pose/pose_cov (EKF3 VPE fusion)
                │           └── /uas1/mavlink_source (BEST_EFFORT)
                │               EKF origin confirm (msg 49) + EKF status (msg 193) + motor PWM (msg 36)
                ├─ /mavros/global_position/set_gp_origin → SET_GPS_GLOBAL_ORIGIN
                ├─ /mavros/setpoint_position/local  → SET_POSITION_TARGET_LOCAL_NED (NED coords)
                └─ MAV_CMD_NAV_TAKEOFF              → ArduPilot altitude controller
```

### Coordinate conventions

| Frame | x | y | z | Used by |
|-------|---|---|---|---------|
| Isaac Sim / ENU | East | North | Up | cesium_scene.py, /drone/state, vision_pose VPE |
| ArduPilot NED | North | East | Down (−=alt) | SET_POSITION_TARGET_LOCAL_NED, VISION_POSITION_ESTIMATE |
| MAVROS2 vision_pose | ENU in, NED out | | | converts correctly |
| MAVROS2 setpoint_position/local | **NED passthrough** | | | does NOT convert; send NED directly |

### Port map

| Port | Protocol | Owner |
|------|----------|-------|
| TCP 5760 | MAVLink | MAVProxy ↔ ArduPilot SITL (internal; single client only) |
| UDP 9002 | JSON SITL | cesium_scene.py (or drone_sim.py headless) ↔ ArduPilot |
| UDP 14550 | MAVLink | MAVROS2 listens (MAVProxy → MAVROS2) |

---

## Modules

### 1. Simulator / Physics (`simulator/cesium_scene.py`)

**Status:** Working — 100 Hz kinematic physics thread; owns SITL bridge; publishes /drone/state

Isaac Sim 6.0.0 scene centred on Chiayi, Taiwan (23.4509°N, 120.2861°E).

- **Terrain:** Cesium World Terrain (asset 1) — quantized-mesh-1.0, 9 tiles at level 13
- **Buildings:** Cesium OSM Buildings (asset 96188) — B3DM, ~83 buildings
- **Imagery:** Taiwan NLSC PHOTO2 orthophoto WMTS (zoom 18, resized to 4096×4096)
- **Physics thread (100 Hz):** `_run_physics()` — kinematic 6-DOF model + `SITLBridge`; publishes `/drone/state`; rate decoupled from render loop
- **Render loop (~13 Hz):** reads shared kinematic state under lock; updates `/World/Drone` mesh position/orientation; captures nadir camera frames
- **Motor layout:** ArduCopter X-frame ch1=FR(NE), ch2=RL(SW), ch3=RR(SE), ch4=FL(NW)
- **SITLBridge:** `control/sitl_bridge.py` on UDP 9002 — binary servo in, JSON physics out
- **Camera:** `/World/Drone/Camera` — nadir, 18 mm / 36×27 mm, 90°×73.7° FOV, 640×480
- **HUD:** `omni.ui` overlay showing live LAT / LON / ALT MSL / AGL
- **Frame output:** `drone_frames/latest.jpg` + `latest_meta.json` every 5 sim steps
- **ROS2 publishers:** `/drone/state`, `/drone/camera/image_raw`, `/drone/pose` (WGS84), `/drone/agl`

### 2. Drone Simulator Fallback (`control/drone_sim.py`)

**Status:** Headless fallback — not used when Isaac Sim is running

Standalone ROS2 node for headless SITL testing without Isaac Sim. Provides the same `/drone/state` topic and `SITLBridge` connection as `cesium_scene.py`.

- 6-DOF kinematic physics: PWM → thrust → roll/pitch → NED acceleration → ENU position
- Ground friction: horizontal velocity zeroed on ground contact (no sliding during motor spool-up)
- `SITLBridge` on UDP 9002: same protocol as used by `cesium_scene.py`
- Publishes `/drone/state` (`PoseStamped`, `frame_id="local_enu"`)

**Do not run alongside `cesium_scene.py`** — both bind UDP 9002.

### 3. Localisation (`anyloc/`)

**Status:** Working — AnyLoc + VO; 2,821-entry database; ~15–20 m anchor error

### 4. Object Detection (`detection/`)

**Status:** Working — `yolov8l_visdrone.pt` (VisDrone-trained), 10 aerial classes

### 5. Flight Control (`control/flight_commander.py`)

**Status:** Takeoff working ✓; waypoint direction and altitude bugs fixed; clean run with Isaac Sim physics pending

ROS2 node. No pymavlink — uses MAVROS2 raw MAVLink exclusively.

**Full arming and flight sequence:**

1. Start VPE thread — Phase 1: kinematic position (from `/drone/state`) at 0.1 m² cov; Phase 2: AnyLoc above 50 m AGL
2. Wait for MAVROS2 connection (`/mavros/state.connected`)
3. Publish EKF origin to `/mavros/global_position/set_gp_origin`; block up to 60 s waiting for GPS_GLOBAL_ORIGIN (msg 49) echo; abort if unconfirmed
4. Arm in STABILIZE (bypasses GPS/VisOdom pre-arm; only needs IMU attitude)
5. Switch to GUIDED
6. Wait for `EKF_POS_HORIZ_ABS` (bit 4) from EKF_STATUS_REPORT (msg 193) on `/uas1/mavlink_source`
7. Send `MAV_CMD_NAV_TAKEOFF` — ArduPilot's own altitude controller climbs to 90 m AGL. Monitor `/drone/state` AGL; abort if still on ground after 30 s.
8. Hold 5 s at takeoff altitude (send current-position setpoints in NED to maintain altitude through the transition)
9. Send waypoint position setpoints in NED; wait until within 5 m radius
10. RTL on Ctrl-C

**MAVROS2 setpoint coordinate bug:**  
`setpoint_position/local` plugin in MAVROS2 Jazzy passes coordinates **directly** into `SET_POSITION_TARGET_LOCAL_NED` without ENU→NED axis swap. Position setpoints must be sent in NED: `x=north, y=east, z=down` (negative = altitude above origin).

**VPE covariance design:**
- `frame_id = "map"` (ENU): x = East, y = North, z = Up — the `vision_pose` plugin converts correctly to NED
- Phase 1 x/y covariance = **0.1 m²** (EKF sets POS_HORIZ_ABS immediately at ground)
- Phase 2 x/y covariance = **max(1, error_m²)** (scales with AnyLoc confidence)
- z = kinematic AGL from `/drone/state`; z covariance = **0.25 m²**; `EK3_SRC1_POSZ=6` routes to EKF altitude

**no_gps.parm highlights:**

| Param | Value | Reason |
|-------|-------|--------|
| `GPS_TYPE` | 0 | disable GPS |
| `EK3_SRC1_POSXY` | 6 | ExternalNav horizontal position |
| `EK3_SRC1_POSZ` | 6 | ExternalNav altitude (VPE z = kinematic AGL; barometer unreliable in SIM_JSON without `position` field) |
| `EK3_SRC1_YAW` | 6 | ExternalNav yaw |
| `VISO_TYPE` | 1 | MAVLink vision odometry |
| `FS_GPS_ENABLE` | 0 | no GPS failsafe |
| `ARMING_CHECK` | 0 | skip pre-arm (SITL only) |
| `MOT_THST_HOVER` | 0.5 | hover PWM = 1500 (matches Isaac Sim k_T and kinematic model) |
| `SCHED_LOOP_RATE` | 50 | matches Isaac Sim frame rate |
| `DISARM_DELAY` | 0 | enables NAV_TAKEOFF path (removes land-detector deadlock) |
| `WPNAV_SPEED` | 100 | 1 m/s horizontal max |
| `PSC_POSXY_P` | 0.3 | horizontal position P (default 1.0) |
| `PSC_VELXY_P` | 0.5 | horizontal velocity P (default 2.0) |
| `PSC_VELXY_I` | 0.0 | horizontal velocity I — zeroed to prevent windup during 90 s climb |
| `ATC_ANG_RLL_P` | 1.5 | roll angle P (default 4.5 → I-term windup → drift) |
| `ATC_ANG_PIT_P` | 1.5 | pitch angle P |
| `ATC_RAT_RLL_P/I` | 0.10 / 0.02 | roll rate gains |
| `ATC_RAT_PIT_P/I` | 0.10 / 0.02 | pitch rate gains |

---

## Run order

### With Isaac Sim (standard)

```bash
# T1 — Isaac Sim (physics + visualiser; writes home_elevation.json; start first)
cd simulator && ./run_chiayi.sh

# T2 — ArduPilot SITL
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild \
    -l 23.450868,120.286135,28.17,0 \
    --add-param-file=control/no_gps.parm --wipe \
    --out udp:127.0.0.1:14550

# T3 — MAVROS2
bash control/launch_mavros.sh

# T4 — AnyLoc
./anyloc/run_ros2_localizer.sh

# T5 — Flight commander
source /opt/ros/jazzy/setup.bash && python3 control/flight_commander.py
```

> Do **not** run `drone_sim.py` alongside `cesium_scene.py` — both bind UDP 9002.

### Headless (no Isaac Sim)

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

> Restart all processes (SITL + physics + MAVROS2) after any failed run.

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
| 6h | Remove pymavlink; MAVROS2 raw MAVLink; DISARM_DELAY=0 | Done ✓ |
| 6i | NAV_TAKEOFF replaces P-controller; EKF origin blocking; reduced ATC gains | Done ✓ |
| 6j | Fix altitude runaway (EK3_SRC1_POSZ→ExternalNav) + fix 90° course error | Done ✓ |
| 6k | Fix waypoint direction inversion (MAVROS2 NED passthrough) + altitude drop on hold | Done ✓ |
| 6l | Integrate kinematic physics into cesium_scene.py (100 Hz thread); eliminate drone_sim.py | Done ✓ |
| 6k-wp | Waypoints clean run with Isaac Sim physics | WIP |
| 6c | HIGHRES_IMU from ArduPilot → localization pipeline | TODO |
| 6d | IMU fusion: AnyLoc anchor validator + VO quality gate | TODO |
| 7 | Full pipeline: AnyLoc + VO + IMU → ArduPilot commands | TODO |
| 8 | Deploy to real drone hardware | TODO |
