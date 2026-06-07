# No-GPS Drone Project

Autonomous drone that localises itself and detects objects without GPS, validated in Isaac Sim before deployment to real hardware.

**Location:** Chiayi, Taiwan — 23.4509°N, 120.2861°E  
**Stack:** Isaac Sim 6.0.0 · AnyLoc (DINOv2+VLAD) · YOLOv8 · **PX4 SITL** · ROS2 Jazzy · MAVROS2

> **Autopilot:** migrated from ArduPilot → **PX4** (2026-06). ArduPilot's horizontal position
> controller (`AC_PosControl`) inverted its output with verified-correct EKF inputs. PX4
> position-hold gate is validated (<0.3 m drift, 40 s); full waypoint nav (65 m AGL, 699 m leg) is
> implemented. Toggle with `PX4_SIM=1`; physics, Cesium, and AnyLoc are unchanged.

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│  simulator/cesium_scene.py  (Isaac Sim — physics + visualiser)      │
│  100 Hz background thread: 6-DOF kinematic model                     │
│  ArduPilot: SITLBridge  UDP 9002  (binary servo in / JSON out)      │
│  PX4:       PX4SimBridge TCP 4560  (HIL_ACTUATOR_CONTROLS in /       │
│                                     HIL_SENSOR out)                  │
│  Publishes: /drone/state (ENU PoseStamped, 100 Hz)                  │
│             /drone/camera/image_raw (2-axis gimbal nadir, 1024×768)  │
│             /drone/pose  /drone/agl                                  │
└───────┬──────────────────────────────────────────────────────────────┘
        │ /drone/state
  ┌─────┴──────────────────────┐
  ▼                            ▼
anyloc/ros2_node.py       detection/ros2_node.py
DINOv2+VLAD localisation  YOLOv8 detection
→ /mavros/vision_pose/    → /yolo/detections
  pose_cov  (VPE)

 ── ArduPilot path ────────────────────────────────────────────────────
 MAVProxy (TCP 5760) → UDP 14550 → MAVROS2
 /mavros/vision_pose/pose_cov → EKF3 (ExternalNav)
 flight_commander.py: STABILIZE→arm→GUIDED→NAV_TAKEOFF→WP→RTL

 ── PX4 path (active) ─────────────────────────────────────────────────
 PX4 SITL (TCP 4560 HIL) → UDP 14540/14580 → MAVROS2
 /mavros/vision_pose/pose_cov → EKF2 (EV_CTRL=15)
 px4_commander.py: stream setpoints→OFFBOARD→arm→climb 65m→WP→RTL
```

**Headless fallback:** `control/drone_sim.py` provides the same kinematic bridge without Isaac Sim — used for fast control-loop testing. Not used when Isaac Sim runs.

---

## Repository Layout

```
no_GPS_drone_project/
├── run.sh                        # top-level launcher (ArduPilot + PX4 tmux modes)
├── simulator/                    # Isaac Sim — physics + visualiser
│   ├── cesium_scene.py           # Cesium terrain + 100 Hz kinematic thread + bridge
│   └── run_chiayi.sh             # launch: ./run_chiayi.sh [--px4]
├── control/                      # autopilot integration + mission
│   ├── drone_sim.py              # headless physics rig (PX4_SIM=0/1)
│   ├── px4_sim_bridge.py         # PX4 HIL bridge (TCP 4560, pymavlink)
│   ├── sitl_bridge.py            # ArduPilot SIM_JSON bridge (UDP 9002)
│   ├── px4_commander.py          # PX4 mission: OFFBOARD→65m→WP(699m)→RTL
│   ├── flight_commander.py       # ArduPilot mission (reference; WP nav unsolved)
│   ├── px4_no_gps.params         # PX4: EKF2_EV_CTRL=15, GPS off, no RC
│   ├── no_gps.parm               # ArduPilot: EK3 ExternalNav, GPS off
│   ├── launch_px4_sitl.sh        # start PX4 SITL (waits for TCP 4560 then UDP 14580)
│   ├── launch_mavros_px4.sh      # MAVROS2 → PX4 (UDP 14540)
│   ├── launch_commander_px4.sh   # run px4_commander.py
│   ├── apply_px4_params.sh       # set + save PX4 params, auto-reboot
│   ├── launch_sitl.sh            # ArduPilot SITL via MAVProxy
│   ├── launch_mavros.sh          # MAVROS2 → ArduPilot (UDP 14550)
│   └── launch_commander.sh       # run flight_commander.py
├── anyloc/                       # visual localisation
│   ├── build_database.py         # build ~2 820-entry VLAD database (AGL 65 m only; run once)
│   ├── localizer.py              # AnyLocLocalizer (DINOv2+VLAD+FAISS)
│   ├── ros2_node.py              # ROS2: pub /mavros/vision_pose/pose_cov
│   └── run_ros2_localizer.sh     # launch script
├── detection/                    # object detection
│   ├── detector.py               # YOLODetector (auto class-map COCO/VisDrone)
│   ├── ros2_node.py              # ROS2: sub /drone/camera → pub /yolo/detections
│   └── run_ros2_detector.sh      # launch script
├── tools/                        # Post-flight and live analysis tools
│   ├── live_trace.py             # Real-time flight-trace viewer (FuncAnimation, 200 ms)
│   └── plot_trace.py             # Post-flight two-panel plot (top view + altitude)
├── yolov8l_visdrone.pt           # YOLOv8l fine-tuned on VisDrone (active)
└── third_party/ardupilot/        # ArduPilot source (SITL binary inside)
```

---

## Milestones

| # | Milestone | Status |
|---|-----------|--------|
| 1 | Isaac Sim: Cesium terrain + NLSC imagery + OSM buildings | Done |
| 2 | Virtual drone + nadir camera | Done |
| 3 | AnyLoc database from simulated satellite views | Done |
| 4 | AnyLoc + VO localisation on simulated frames | Done |
| 5 | YOLOv8 detection on simulated frames (VisDrone model) | Done |
| 6a | ArduPilot SITL + Isaac Sim physics bridge (JSON FDM) | Done |
| 6b | GPS-denied EKF (EK3 ExternalNav) + VPE from AnyLoc | Done |
| 6c–n | ROS2/MAVROS2 migration, VPE tuning, takeoff, 90 m altitude | Done |
| 6m-wp | **ArduPilot WP nav** — takeoff+90 m OK; horizontal nav inverts (AC_PosControl) | Blocked |
| PX4-1 | PX4 SITL ↔ HIL bridge validated (27k+ frames, EKF2 level) | Done |
| PX4-2 | Vision + MAVROS↔PX4 link established | Done |
| PX4-3 | **Position-hold gate passed** (<0.3 m drift, 40 s) | Done |
| PX4-4 | Waypoint nav ported to px4_commander.py (65 m, 699 m leg) | Done |
| PX4-5 | Isaac Sim pipeline wired (`run.sh --tmux --px4`) | Done |
| PX4-6 | End-to-end Isaac Sim waypoint flight (65 m AGL, 699 m leg, horiz_err < 60 m) | Done ✓ |
| PX4-7 | AnyLoc + detection integration in PX4 pipeline | In progress |
| PX4-8 | Survey mission plan: lawnmower + car detection response | Done ✓ (impl pending) |
| PX4-9 | Implement survey commander: 12 m/s, 6 strips, YOLO divert+log | TODO |
| PX4-10 | Jetson distributed sim (Jetson = commander+AnyLoc+YOLO; PC = Isaac+PX4) | TODO |
| 8 | Deploy to real hardware | TODO |

---

## Port Map

| Port | Protocol | Owner | Direction |
|------|----------|-------|-----------|
| TCP 4560 | MAVLink HIL | PX4SimBridge (server) ↔ PX4 SITL (client) | physics bridge |
| UDP 9002 | JSON FDM | SITLBridge ↔ ArduPilot SITL | physics bridge |
| TCP 5760 | MAVLink | MAVProxy ↔ ArduPilot SITL | internal |
| UDP 14550 | MAVLink | MAVROS2 ← MAVProxy → ArduPilot | ArduPilot offboard |
| UDP 14540 | MAVLink | MAVROS2 receives from PX4 | PX4 offboard |
| UDP 14580 | MAVLink | PX4 SITL listens (onboard link) | PX4 offboard |
| UDP 18570 | MAVLink | PX4 SITL → GCS (QGC) | PX4 GCS |

---

## Quick Start — PX4 (recommended)

### Prerequisites

```bash
# ROS2 Jazzy + MAVROS2
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# PX4 SITL (one-time build, ~20 min)
cd ~/PX4-Autopilot
PATH=~/.local/bin:$PATH make px4_sitl_nolockstep
```

### Run — Isaac Sim (full pipeline)

```bash
# First run — apply params and wipe saved state:
bash run.sh --tmux --px4 --params --wipe

# Subsequent runs (control loop only):
bash run.sh --tmux --px4

# With AnyLoc GPS-denied localisation (Phase 2 VPE from visual matching):
bash run.sh --tmux --px4 --anyloc

# With AnyLoc + YOLO vehicle detection:
bash run.sh --tmux --px4 --anyloc --detection

# Headless Isaac Sim (no display window — full camera/AnyLoc/YOLO still run):
bash run.sh --tmux --px4 --no-window
bash run.sh --tmux --px4 --no-window --anyloc --detection
```

tmux windows: **0 Isaac** · **1 PX4** · **2 MAVROS** · **3 Commander** · **4 AnyLoc** · **5 Detection**  
Switch with `Ctrl-B 0–5`. The commander prints `[PX4Cmd]` progress to window 3.

> **AnyLoc startup:** the database is now ~2,820 entries (AGL 65 m only). Load time is much shorter than the old 36,673-entry database.

### Run — distributed (PC = sim only, Jetson = everything that runs on real drone)

```bash
# PC — Isaac Sim + PX4 SITL + MAVProxy bridge only
export ROS_DOMAIN_ID=0
bash run.sh --tmux --px4 --jetson-sim

# Jetson Orin NX — MAVROS + Commander + AnyLoc + YOLO (same as real hardware)
export ROS_DOMAIN_ID=0
bash run_jetson.sh
```

tmux on PC: **0 Isaac · 1 PX4 · 2 MAVProxy**  
tmux on Jetson: **0 MAVROS · 1 Commander · 2 AnyLoc · 3 Detection**

See `instructions/jetson_distributed_plan.md` for network setup, MAVProxy bridge details, code changes required, and real hardware transition notes.

### Run — headless (no Isaac Sim, for control-loop testing)

```bash
# First run (apply params):
bash run.sh --tmux --px4 --headless --params

# Subsequent runs:
bash run.sh --tmux --px4 --headless
```

tmux windows: **0 Bridge** · **1 PX4** · **2 MAVROS** · **3 Commander**  
Switch with `Ctrl-B 0/1/2/3`. The commander prints `[PX4Cmd]` progress to window 3.

<details>
<summary>Manual steps (without run.sh)</summary>

```bash
source /opt/ros/jazzy/setup.bash

# T1 — physics bridge (must own TCP 4560 before PX4 starts)
PX4_SIM=1 python3 control/drone_sim.py

# T2 — PX4 SITL
bash control/launch_px4_sitl.sh
bash control/apply_px4_params.sh     # first run only

# T3 — MAVROS
bash control/launch_mavros_px4.sh

# T4 — commander
source /opt/ros/jazzy/setup.bash
python3 control/px4_commander.py
```

</details>

### Hold-gate test only (Phase 3 regression check)

```bash
HOLDTEST=1 python3 control/px4_commander.py
```

---

## Quick Start — ArduPilot (reference)

```bash
# tmux launcher
bash run.sh --tmux          # normal run
bash run.sh --tmux --wipe   # first run (wipe EEPROM)

# or manual:
bash control/launch_sitl.sh   # T1
bash control/launch_mavros.sh # T2
bash control/launch_commander.sh  # T3
```

> First run: type `reboot` in MAVProxy after params load. Drop `--wipe` subsequently.

---

## Key Design Decisions

### Autopilot bridge

`cesium_scene.py` and `drone_sim.py` both honour `PX4_SIM`:
- `PX4_SIM=0` (default): `SITLBridge` on UDP 9002 — binary servo packet in, JSON FDM out
- `PX4_SIM=1`: `PX4SimBridge` on TCP 4560 — `HIL_ACTUATOR_CONTROLS` in, `HIL_SENSOR` out

The bridge must own its port **before** the autopilot starts; otherwise SITL/PX4 exits immediately.

### VPE (vision position estimate) — two phases

Both commanders publish `PoseWithCovarianceStamped` to `/mavros/vision_pose/pose_cov`:

| Phase | Trigger | Position source | cov_xy |
|-------|---------|-----------------|--------|
| 1 | AGL < 50 m | `/drone/state` kinematic truth (ENU) | 0.1 m² |
| 2 | AGL ≥ 50 m | AnyLoc `latest_estimate.json` | max(1, err_m²) |

Altitude always comes from `/drone/state` (kinematic AGL); cov_z = 0.25 m².  
Frame: `"map"` (ENU) — MAVROS converts to NED for PX4/ArduPilot.

**Heading:** ENU yaw = π/2 (North) in **both** phases. `/drone/pose` encodes orientation as `qz = sin(−_kyaw_rad / 2)`, so a North-facing drone (`_kyaw_rad = 0`) produces `yaw_deg = 0` (East) in `latest_estimate.json` — a 90° error. Since the drone never yaws, the commander hardcodes π/2 for consistent, correct heading across both phases.

### MAVROS2 setpoint convention

`/mavros/setpoint_raw/local` with `FRAME_LOCAL_NED`:  
**MAVROS2 always applies ENU→NED** regardless of the frame flag. Send:
- `position.x = East`, `position.y = North`, `position.z = Up (AGL)` — MAVROS negates z to NED Down.

### PX4 OFFBOARD mode

PX4 requires setpoints streaming ≥ 2 Hz **before** switching to OFFBOARD. `px4_commander.py` pre-streams 40 setpoints at 20 Hz, then switches mode and arms. OFFBOARD is maintained by continuous setpoint publishing in `takeoff()` and `go_to_ned()`.

### Position carrot navigation (PX4)

`go_to_ned()` publishes a position target 25 m ahead of the drone toward the waypoint. This prevents PX4 from commanding max speed toward a 700 m jump and gives smooth, bounded velocity. Within 25 m the carrot snaps to the exact target.

### PX4 parameters (px4_no_gps.params)

| Param | Value | Reason |
|-------|-------|--------|
| `EKF2_GPS_CTRL` | 0 | disable GPS |
| `SYS_HAS_GPS` | 0 | GPS not present |
| `COM_ARM_WO_GPS` | 1 | allow arming without GPS |
| `EKF2_EV_CTRL` | 15 | fuse EV pos + height + vel + yaw |
| `EKF2_HGT_REF` | 3 | vision altitude reference |
| `EKF2_BARO_CTRL` | 0 | disable baro (EV handles altitude) |
| `COM_RC_IN_MODE` | 4 | no RC required |
| `NAV_RCL_ACT` | 0 | no RC loss failsafe |
| `NAV_DLL_ACT` | 0 | no datalink loss failsafe |

### Why 100 Hz physics matters

Isaac Sim renders at ~13 fps. If the physics + bridge ran in the render loop, the autopilot would see 13 Hz physics replies — too slow for stable PID control (altitude oscillates). The background thread at 100 Hz gives the autopilot a stable high-rate loop.

---

## Monitor Topics

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /drone/state                      # physics rate (should be ~100 Hz)
ros2 topic hz /drone/camera/image_raw          # ~6 Hz from Isaac Sim
ros2 topic echo /mavros/state                  # connected, armed, mode
ros2 topic echo /mavros/local_position/pose    # EKF2 position estimate
ros2 topic echo /mavros/vision_pose/pose_cov   # VPE from commander
```

---

## Flight Trace Tools

Both `drone_sim.py` and `cesium_scene.py` write a CSV trace at 5 Hz to `simulator/flight_traces/trace_<timestamp>.csv`:

```
t_s, east_m, north_m, agl_m, vn_ms, ve_ms
```

**Live viewer** (open before or during flight):
```bash
python3 tools/live_trace.py              # auto-attach to latest trace
DISPLAY=:2 python3 tools/live_trace.py  # headless display
```

**Post-flight plot** (saves `simulator/flight_traces/trace_plot.png`):
```bash
python3 tools/plot_trace.py             # latest trace
python3 tools/plot_trace.py --all       # overlay all traces
```

---

## Data Sources

| Layer | Source | License |
|-------|--------|---------|
| Terrain | Cesium World Terrain (asset 1) | © Cesium ion |
| Buildings | Cesium OSM Buildings (asset 96188) | © OpenStreetMap (ODbL) |
| Satellite imagery (database build) | Taiwan NLSC PHOTO2 WMTS | © 內政部國土測繪中心 |
| Validation imagery | Esri World Imagery | © Esri / contributors |
