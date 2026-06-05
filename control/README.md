# control/ — Flight control, SITL bridges, and autopilot integration

This module connects the simulator physics to an autopilot (SITL) and runs the
autonomous mission. **It supports two autopilots**, selected by the `PX4_SIM` env var:

| Autopilot | Bridge | Transport | Commander | Status |
|-----------|--------|-----------|-----------|--------|
| **ArduPilot** (default) | `sitl_bridge.py` | JSON FDM, UDP 9002 | `flight_commander.py` | takeoff+90 m alt OK; **horizontal WP nav has an unsolved `AC_PosControl` inversion** (see memory `horizontal-flyaway-diagnosis`) |
| **PX4** (`PX4_SIM=1`) | `px4_sim_bridge.py` | MAVLink HIL, TCP 4560 | `px4_commander.py` | **migration in progress** — Phase 1 done (bridge↔PX4 validated), Phase 2/3 wiring underway |

The PX4 migration was started because the ArduPilot position-control inversion
defied an exhaustive investigation (EKF/velocity/yaw all verified correct, yet
position setpoints + AUTO fly the mirror direction; only direct velocity commands track).

## Files

### Bridges (simulator ↔ autopilot)
- **`sitl_bridge.py`** — ArduPilot SIM_JSON bridge. UDP 9002 server; binary servo PWM in,
  JSON physics (IMU/attitude/velocity, NED) out. Owns the clock-reset-on-reconnect fix
  (re-zeros the FDM clock per SITL session so SITL is restartable; see memory).
- **`px4_sim_bridge.py`** — PX4 Simulator-MAVLink (HIL) bridge. **TCP 4560 server** (PX4 is
  the client). In: `HIL_ACTUATOR_CONTROLS` (16 normalised motor outputs). Out: `HIL_SENSOR`
  (accel/gyro body-FRD, synthetic mag rotated by attitude, baro). Reuses the IMU/specific-force
  math from `sitl_bridge`. Notes:
  - **time_usec**: PX4 `px4_clock_settime`s its CLOCK_MONOTONIC to it (even nolockstep) →
    send `time.monotonic()*1e6`, not 0-based (a backward jump makes baro/mag STALE).
  - Motor decode (PX4 none_iris CA_ROTOR geometry) is done in the sim: `control[0]=FR(+,+)
    [1]=RL(-,-) [2]=FL(+,-) [3]=RR(-,+)`; roll=`(m1+m2)-(m0+m3)`, pitch=`(m0+m2)-(m1+m3)`.

### Physics rigs (publish `/drone/state`, no Isaac Sim)
- **`drone_sim.py`** — headless kinematic 6-DOF rig + bridge. `PX4_SIM=1` → PX4 bridge (4560),
  else ArduPilot (9002). The fast control-loop iteration tool (no Isaac render overhead).
  The full visual sim is `simulator/cesium_scene.py` (also honours `PX4_SIM`).

### Commanders (the mission)
- **`flight_commander.py`** — ArduPilot/MAVROS: VPE injection (`/mavros/vision_pose`),
  vision_speed, STABILIZE→GUIDED arm, NAV_TAKEOFF, waypoint nav (velocity-carrot `go_to_ned`).
  Diagnostic harnesses: `CALIBRATE=1` (yaw/velocity probes), `HOLDTEST=1` (position-hold).
- **`px4_commander.py`** — PX4/MAVROS: vision injection (`/mavros/vision_pose` + `/mavros/vision_speed`),
  OFFBOARD-mode arm + takeoff-and-hold (the position-hold **gate** test).

### Parameters
- **`no_gps.parm`** — ArduPilot SITL params (EK3_SRC*=ExternalNav, GPS off, tuned PSC/ATC gains).
- **`px4_no_gps.params`** — PX4 params (`EKF2_GPS_CTRL=0`, `EKF2_EV_CTRL=15`, `EKF2_HGT_REF=3`,
  no-RC/failsafe disables). Apply with `apply_px4_params.sh` then reboot PX4.

### Launch scripts
- ArduPilot: `launch_sitl.sh` (+`--wipe`), `launch_mavros.sh`, `launch_commander.sh`.
- PX4: `apply_px4_params.sh`, `launch_mavros_px4.sh`, `px4_bridge_test.py` (standalone HIL link test).

## PX4 launch (hard-won — see memory `px4-migration-plan`)
1. Build once: `cd ~/PX4-Autopilot && PATH=~/.local/bin:$PATH make px4_sitl_nolockstep`
   (nolockstep = PX4 free-runs, right for a render-paced sim). Python deps + `ninja` via
   `pip install --user --break-system-packages …`.
2. Bridge first (must own TCP 4560): `PX4_SIM=1 python3 control/drone_sim.py`.
3. PX4: from `build/px4_sitl_nolockstep/rootfs`, run `PX4_SYS_AUTOSTART=10016 ../bin/px4 -d`
   with **no other args** (extra args break px4's data_path auto-detect). Use `setsid` so the
   daemon survives the launching shell.
4. MAVROS: `bash control/launch_mavros_px4.sh` (fcu_url udp 14540).
5. Params: `bash control/apply_px4_params.sh`, then reboot PX4.
6. Commander: `python3 control/px4_commander.py` (vision + OFFBOARD hold gate).

## Phased migration status (2026-06-05)
- ✅ **Phase 1** — PX4 SITL ↔ `px4_sim_bridge` validated: PX4 connects on 4560, 27k+ HIL_SENSOR
  frames, **EKF2 → level attitude** (roll/pitch/yaw 0.0), sensors fresh. The bridge/IMU math is correct.
- 🚧 **Phase 2/3** — `drone_sim PX4_SIM=1` runs and PX4 connects; `px4_commander`, MAVROS launch,
  and params are built. **Open blockers:** (a) MAVROS↔PX4 link — PX4 SITL's onboard MAVLink
  (14540/14580) isn't streaming bidirectionally to MAVROS (VER timeouts, `connected:false`);
  (b) PX4 daemon stability/IPC under `-d`+`setsid` (px4 client occasionally can't reach the daemon).
  Next: get the onboard MAVLink link up, then run the **position-hold gate** (make-or-break vs the
  ArduPilot inversion), then waypoint, then AnyLoc/detection.
