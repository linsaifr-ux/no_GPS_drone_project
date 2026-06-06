#!/usr/bin/env python3
"""
PX4 flight commander (MAVROS2) — external-vision no-GPS.

Full mission equivalent of flight_commander.py ported to PX4 OFFBOARD mode.

Key differences from ArduPilot flight_commander.py:
  - No set_gp_origin: PX4 EKF2 auto-sets its local frame origin from first EV pose.
  - No STABILIZE→GUIDED arm dance: stream setpoints ≥2 Hz → OFFBOARD → arm.
  - No NAV_TAKEOFF: climb via OFFBOARD position setpoints (monitored from drone_state).
  - Vision via /mavros/vision_pose/pose_cov (PoseWithCovarianceStamped) so EKF2
    can weight Phase-1 vs Phase-2 VPE correctly.
  - go_to_ned() uses a position carrot (25 m ahead) — PX4's position loop is stable,
    unlike ArduPilot AC_PosControl which inverted direction.

Environment variables:
  HOLDTEST=1         run Phase-3 hold gate (HOLD_AGL m) instead of full mission
  TAKEOFF_ALT=<m>    override mission cruise altitude (default 90.0 m)

Run:
  source /opt/ros/jazzy/setup.bash
  python3 control/px4_commander.py              # full mission
  HOLDTEST=1 python3 control/px4_commander.py   # hold-gate only
"""
import json
import math
import os
import sys
import threading
import time

_ROS2_SITE = "/opt/ros/jazzy/lib/python3.12/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode

_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

# ── Home position ──────────────────────────────────────────────────────────────
HOME_LAT  = 23.450868
HOME_LON  = 120.286135
_HOME_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "home_elevation.json")
try:
    with open(_HOME_CFG) as _f:
        HOME_ALT_MSL = float(json.load(_f)["centre_elev_m"])
    print(f"[PX4Cmd] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m  (from {_HOME_CFG})")
except (FileNotFoundError, KeyError):
    HOME_ALT_MSL = 28.17
    print(f"[PX4Cmd] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m  (default)")

# ── Mission parameters ─────────────────────────────────────────────────────────
TAKEOFF_ALT          = float(os.environ.get("TAKEOFF_ALT", "90.0"))
HOLD_AGL             = 3.0    # m — Phase-3 gate altitude (HOLDTEST mode)
WAYPOINT_RADIUS      = 60.0   # m — arrival threshold
WAYPOINT_TIMEOUT     = 900.0  # s per waypoint
MIN_LOCALISATION_AGL = 50.0   # m — below this use truth VPE; above, use AnyLoc

COS_LAT   = math.cos(math.radians(HOME_LAT))
M_PER_DEG = 111_320.0

# Target: 23.45564°N, 120.28169°E  (N=+531.2 m, E=−453.9 m, dist≈699 m)
WAYPOINTS = [
    (531.2, -453.9, TAKEOFF_ALT),   # (north_m, east_m, agl_m)
]

ESTIMATE_JSON = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "anyloc", "latest_estimate.json"
)


class PX4Commander(rclpy.node.Node):
    def __init__(self):
        super().__init__("px4_commander")
        self._state     = State()
        self._local_pos = None   # /mavros/local_position/pose  (ENU, from EKF2)
        self._local_vel = None   # /mavros/local_position/velocity_local (ENU)
        self._drone     = None   # /drone/state  (ENU, kinematic truth)

        # Subscribers
        from geometry_msgs.msg import PoseStamped
        self.create_subscription(State, "/mavros/state", self._cb_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_local, _SENSOR_QOS)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local",
                                 self._cb_vel, _SENSOR_QOS)
        self.create_subscription(PoseStamped, "/drone/state",
                                 self._cb_drone, _SENSOR_QOS)

        # Publishers
        self._vpe_pub  = self.create_publisher(
            PoseWithCovarianceStamped, "/mavros/vision_pose/pose_cov", 1)
        self._vspd_pub = self.create_publisher(
            TwistStamped, "/mavros/vision_speed/speed_twist", 1)
        self._sp_pub   = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 1)

        # Service clients
        self._arm_cli  = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_cli = self.create_client(SetMode,     "/mavros/set_mode")

        self.get_logger().info("PX4 commander ready")

    # ── Callbacks ──────────────────────────────────────────────────────────────
    def _cb_state(self, m):  self._state     = m
    def _cb_local(self, m):  self._local_pos = m
    def _cb_vel(self, m):    self._local_vel = m
    def _cb_drone(self, m):  self._drone     = m

    # ── Vision injection thread ────────────────────────────────────────────────
    def start_vision(self, stop):
        """
        20 Hz background thread: publish VPE + velocity to MAVROS → PX4 EKF2.

        Two-phase strategy (mirrors flight_commander.py):
          Phase 1 (AGL < MIN_LOCALISATION_AGL):
            position = drone_state kinematic truth, cov_xy = 0.1 m²
          Phase 2 (AGL ≥ MIN_LOCALISATION_AGL):
            position = AnyLoc estimate from latest_estimate.json, cov_xy = err_m²

        Heading quaternion: ENU yaw = π/2 (North) in both phases.
        /drone/pose encodes -_kyaw_rad (not π/2-_kyaw_rad), so the kinematic
        yaw_deg=0 maps to East, not North. Since the drone never yaws, π/2 is
        always correct and keeps Phase 1→2 VPE yaw continuous.
        MAVROS converts ENU yaw=π/2 → NED yaw=0 (North).
        """
        def loop():
            last_ds      = None
            anyloc_est   = None
            last_mtime   = 0.0
            n_sent       = 0
            phase_logged = False

            while not stop.is_set():
                t0 = time.time()

                # AGL from EKF2 local position (ENU z = Up = AGL above origin)
                agl = 0.0
                if self._local_pos is not None:
                    agl = max(0.0, self._local_pos.pose.position.z)

                # True AGL from kinematic model (authoritative altitude source)
                drone_agl = 0.0
                if self._drone is not None:
                    drone_agl = max(0.0, self._drone.pose.position.z - HOME_ALT_MSL)

                # Phase 2: read AnyLoc estimate when high enough.
                # Use kinematic drone_agl (truth), NOT EKF agl — stale EKF state
                # from a previous flight would otherwise trigger Phase 2 on the
                # ground, sending altitude=0 VPE while EKF thinks we're at 60+ m,
                # which confuses the position controller.
                if drone_agl >= MIN_LOCALISATION_AGL:
                    if not phase_logged:
                        print(f"[PX4Cmd] AGL {drone_agl:.0f} m ≥ {MIN_LOCALISATION_AGL:.0f} m"
                              " — VPE → AnyLoc")
                        phase_logged = True
                    try:
                        mtime = os.path.getmtime(ESTIMATE_JSON)
                        if mtime != last_mtime:
                            with open(ESTIMATE_JSON) as fh:
                                est = json.load(fh)
                            err_m = est.get("error_m", 999.0)
                            if (est.get("agl_m", 0.0) >= MIN_LOCALISATION_AGL
                                    and err_m < 100.0):
                                lat  = est["est_lat"]; lon = est["est_lon"]
                                # Always π/2 (ENU North): the kinematic drone never
                                # yaws and /drone/pose encodes -_kyaw_rad (not π/2-_kyaw_rad),
                                # giving yaw_deg=0 (East) when the drone actually faces
                                # North. Hardcoding π/2 matches Phase 1 and avoids a
                                # 90° VPE yaw jump at the Phase 1→2 transition.
                                yaw  = math.pi / 2.0
                                n_v  = (lat - HOME_LAT) * M_PER_DEG
                                e_v  = (lon - HOME_LON) * M_PER_DEG * COS_LAT
                                cov  = max(1.0, err_m ** 2)
                                anyloc_est = (e_v, n_v, yaw, cov)
                                last_mtime = mtime
                                if n_sent < 2:
                                    print(f"[PX4Cmd] AnyLoc VPE: N={n_v:+.1f}"
                                          f" E={e_v:+.1f} m  err={err_m:.1f} m")
                    except (FileNotFoundError, KeyError, json.JSONDecodeError):
                        pass

                # Select position and covariance
                if agl >= MIN_LOCALISATION_AGL and anyloc_est is not None:
                    east_v, north_v, yaw_v, cov_xy = anyloc_est
                else:
                    if self._drone is not None:
                        east_v  = self._drone.pose.position.x
                        north_v = self._drone.pose.position.y
                    else:
                        east_v, north_v = 0.0, 0.0
                    yaw_v  = math.pi / 2.0   # ENU yaw 90° = facing North
                    cov_xy = 0.1

                # Publish VPE (heading-only quaternion, ENU frame)
                hy  = yaw_v / 2.0
                msg = PoseWithCovarianceStamped()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"   # ENU
                msg.pose.pose.position.x    = east_v
                msg.pose.pose.position.y    = north_v
                msg.pose.pose.position.z    = drone_agl
                msg.pose.pose.orientation.z = math.sin(hy)
                msg.pose.pose.orientation.w = math.cos(hy)
                cov = [0.0] * 36
                cov[0]  = cov_xy  # East variance
                cov[7]  = cov_xy  # North variance
                cov[14] = 0.25    # altitude: 0.5 m std
                cov[21] = 0.09    # roll
                cov[28] = 0.09    # pitch
                cov[35] = 0.09    # yaw
                msg.pose.covariance = cov
                self._vpe_pub.publish(msg)
                n_sent += 1
                if n_sent == 1:
                    print("[PX4Cmd] vision thread started (Phase 1 — truth)")

                # Velocity aiding: differentiate drone_state for ENU velocity
                if self._drone is not None:
                    ds = self._drone.pose.position
                    now_t = time.time()
                    if last_ds is not None:
                        dt_v = now_t - last_ds[3]
                        if dt_v > 1e-3:
                            tw = TwistStamped()
                            tw.header.stamp    = msg.header.stamp
                            tw.header.frame_id = "map"
                            tw.twist.linear.x  = (ds.x - last_ds[0]) / dt_v  # ENU East
                            tw.twist.linear.y  = (ds.y - last_ds[1]) / dt_v  # ENU North
                            tw.twist.linear.z  = (ds.z - last_ds[2]) / dt_v  # ENU Up
                            self._vspd_pub.publish(tw)
                    last_ds = (ds.x, ds.y, ds.z, now_t)

                elapsed = time.time() - t0
                time.sleep(max(0.0, 0.05 - elapsed))   # 20 Hz

        t = threading.Thread(target=loop, daemon=True)
        t.start()
        return t

    # ── Helpers ────────────────────────────────────────────────────────────────
    def _spin_until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    def set_mode(self, mode, timeout=8.0):
        req = SetMode.Request(); req.custom_mode = mode
        fut = self._mode_cli.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        ok = fut.done() and fut.result().mode_sent
        self.get_logger().info(f"set_mode {mode}: {'✓' if ok else 'FAIL'}")
        return ok

    def arm(self, value=True, timeout=8.0):
        req = CommandBool.Request(); req.value = value
        fut = self._arm_cli.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        ok = fut.done() and fut.result().success
        self.get_logger().info(f"{'arm' if value else 'disarm'}: {'✓' if ok else 'FAIL'}")
        return ok

    def _agl(self):
        """Current AGL from drone_state (truth), falling back to EKF local pos z."""
        if self._drone is not None:
            return self._drone.pose.position.z - HOME_ALT_MSL
        if self._local_pos is not None:
            return self._local_pos.pose.position.z
        return 0.0

    def make_sp(self, east, north, up):
        """
        Build a position PositionTarget in ENU.

        MAVROS2 applies ENU→NED for setpoint_raw/local regardless of FRAME_LOCAL_NED.
        Send x=East, y=North, z=Up(AGL); MAVROS converts to NED: x=North, y=East, z=-Up.
        """
        sp = PositionTarget()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                        PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                        PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                        PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
        sp.position.x = float(east)
        sp.position.y = float(north)
        sp.position.z = float(up)
        return sp

    # ── Pre-stream + switch to OFFBOARD + arm ──────────────────────────────────
    def engage_offboard(self, east, north, up, n_pre=40):
        """
        Stream n_pre setpoints at 20 Hz, then switch OFFBOARD and arm.
        PX4 requires the setpoint stream to be active before accepting OFFBOARD.
        Returns True if OFFBOARD + armed within timeout.
        """
        sp = self.make_sp(east, north, up)
        for _ in range(n_pre):
            sp.header.stamp = self.get_clock().now().to_msg()
            self._sp_pub.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.set_mode("OFFBOARD"):
            return False

        # Keep streaming setpoints while waiting for sensor health to settle.
        # BARO can transiently go stale right after mode change; give it 2 s.
        settle_end = time.time() + 2.0
        while time.time() < settle_end:
            sp.header.stamp = self.get_clock().now().to_msg()
            self._sp_pub.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Retry arm up to 10 times with 1-second intervals.
        for attempt in range(10):
            if self.arm():
                return True
            print(f"[PX4Cmd] arm attempt {attempt+1}/10 failed — retrying in 1 s …")
            retry_end = time.time() + 1.0
            while time.time() < retry_end:
                sp.header.stamp = self.get_clock().now().to_msg()
                self._sp_pub.publish(sp)
                rclpy.spin_once(self, timeout_sec=0.05)

        return False

    # ── Takeoff ────────────────────────────────────────────────────────────────
    def takeoff(self, alt_agl, timeout=180.0):
        """
        Climb to alt_agl via OFFBOARD position setpoints; keep streaming throughout.
        Monitors drone_state (kinematic truth) for actual AGL.
        Returns True when within 2 m of target.
        """
        self.get_logger().info(f"Climbing to {alt_agl:.0f} m AGL …")
        e0 = self._drone.pose.position.x if self._drone else 0.0
        n0 = self._drone.pose.position.y if self._drone else 0.0
        sp = self.make_sp(e0, n0, alt_agl)

        deadline   = time.time() + timeout
        last_print = time.time()

        while time.time() < deadline:
            sp.header.stamp = self.get_clock().now().to_msg()
            self._sp_pub.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.05)

            agl = self._agl()
            now = time.time()
            if now - last_print > 3.0:
                print(f"[PX4Cmd] AGL={agl:.1f} m  target={alt_agl:.0f} m"
                      f"  mode={self._state.mode}  armed={self._state.armed}")
                last_print = now

            if agl >= alt_agl - 2.0:
                self.get_logger().info(f"Reached {alt_agl:.0f} m AGL ✓")
                return True

        self.get_logger().warn("Takeoff timeout")
        return False

    # ── Waypoint navigation ────────────────────────────────────────────────────
    def go_to_ned(self, north, east, agl, timeout=WAYPOINT_TIMEOUT):
        """
        Fly to (north, east, agl) via OFFBOARD velocity setpoints.
        MAVROS converts ENU velocity → NED; PX4 velocity controller closes the loop.
        Returns True when within WAYPOINT_RADIUS of the target.
        """
        NAV_SPEED_H = 5.0   # m/s horizontal cruise
        NAV_SPEED_V = 2.0   # m/s vertical limit
        ALT_KP      = 0.4   # altitude error → desired vz

        # Velocity-only: ignore position and acceleration
        _VMASK = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                  PositionTarget.IGNORE_PZ |
                  PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                  PositionTarget.IGNORE_AFZ |
                  PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)

        deadline   = time.time() + timeout
        last_print = time.time()

        while time.time() < deadline:
            if self._drone is not None:
                ds = self._drone.pose.position
                cur_e, cur_n = ds.x, ds.y
                drone_agl    = ds.z - HOME_ALT_MSL
            elif self._local_pos is not None:
                p = self._local_pos.pose.position
                cur_e, cur_n = p.x, p.y
                drone_agl    = p.z
            else:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            dx = cur_e - east    # East error  (positive = drone E of target)
            dy = cur_n - north   # North error (positive = drone N of target)
            hdist = math.hypot(dx, dy)

            speed = min(NAV_SPEED_H, hdist)
            if hdist > 0.5:
                v_e = -dx / hdist * speed   # ENU East toward target
                v_n = -dy / hdist * speed   # ENU North toward target
            else:
                v_e = v_n = 0.0

            v_up = max(-NAV_SPEED_V, min(NAV_SPEED_V, ALT_KP * (agl - drone_agl)))

            sp = PositionTarget()
            sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            sp.type_mask = _VMASK
            sp.velocity.x = float(v_e)    # ENU East  (MAVROS converts to NED)
            sp.velocity.y = float(v_n)    # ENU North
            sp.velocity.z = float(v_up)   # ENU Up
            sp.header.stamp = self.get_clock().now().to_msg()
            self._sp_pub.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.05)

            now = time.time()
            if now - last_print > 5.0:
                ekf = ""
                if self._local_pos:
                    lp = self._local_pos.pose.position
                    ekf = f"  EKF=({lp.x:+.0f},{lp.y:+.0f},{lp.z:+.0f})"
                vel_s = ""
                if self._local_vel:
                    lv = self._local_vel.twist.linear
                    vel_s = f"  vm=({lv.x:+.1f},{lv.y:+.1f})"
                print(f"[PX4Cmd] WP  errN={dy:+.1f} errE={dx:+.1f}"
                      f"  AGL={drone_agl:.1f} m  dist={hdist:.1f} m{ekf}{vel_s}")
                last_print = now

            if hdist <= WAYPOINT_RADIUS:
                return True

        return False


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    cmd = PX4Commander()
    stop = threading.Event()

    # Overwrite stale AnyLoc estimate so Phase 2 always starts from kinematic truth.
    try:
        os.makedirs(os.path.dirname(ESTIMATE_JSON), exist_ok=True)
        with open(ESTIMATE_JSON, "w") as _ef:
            json.dump({"agl_m": -1.0, "error_m": 999.0}, _ef)
    except OSError:
        pass

    cmd.start_vision(stop)

    # Wait for MAVROS to connect to PX4
    print("[PX4Cmd] waiting for MAVROS connection …")
    if not cmd._spin_until(lambda: cmd._state.connected, 60.0):
        print("[PX4Cmd] MAVROS not connected — start PX4 + MAVROS first")
        stop.set(); cmd.destroy_node(); rclpy.shutdown(); return
    print("[PX4Cmd] MAVROS connected ✓")

    # Wait for drone_state and local_position to arrive.
    # EKF2 needs time to initialize from VPE on a fresh start (can take 60-90 s).
    print("[PX4Cmd] waiting for EKF local position (up to 120 s) …")
    _last_diag = [time.time()]
    def _wait_cond():
        now = time.time()
        if now - _last_diag[0] > 10.0:
            _last_diag[0] = now
            print(f"[PX4Cmd] diag: drone={'OK' if cmd._drone is not None else 'None'}"
                  f"  local_pos={'OK' if cmd._local_pos is not None else 'None'}"
                  f"  t={now-start_t:.0f}s")
        return cmd._local_pos is not None and cmd._drone is not None
    start_t = time.time()
    if not cmd._spin_until(_wait_cond, 120.0):
        print(f"[PX4Cmd] no /drone/state or /mavros/local_position after 120 s"
              f"  drone={'OK' if cmd._drone is not None else 'None'}"
              f"  local_pos={'OK' if cmd._local_pos is not None else 'None'}")
        stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

    # Settle: let EKF2 initialise its local frame from the first VPE batch.
    # After a previous flight the EKF retains its last altitude (e.g. 60+ m).
    # VPE (Phase 1, kinematic ground truth at z=0) will override this, but EKF
    # needs time to converge. Wait until EKF z is within 5 m of ground before
    # arming, or proceed after 15 s with a warning.
    print("[PX4Cmd] waiting for EKF z to converge to ground …")
    def _ekf_near_ground():
        return (cmd._local_pos is not None and
                abs(cmd._local_pos.pose.position.z) < 5.0)
    if not cmd._spin_until(_ekf_near_ground, timeout=15.0):
        ekf_z = cmd._local_pos.pose.position.z if cmd._local_pos else 999.0
        print(f"[PX4Cmd] WARNING: EKF z={ekf_z:.1f} m after 15 s — proceeding anyway")
    else:
        ekf_z = cmd._local_pos.pose.position.z if cmd._local_pos else 0.0
        print(f"[PX4Cmd] EKF z={ekf_z:.2f} m — converged ✓")

    # In-air restart detection: skip takeoff sequence if already airborne
    start_agl = cmd._agl()
    in_air = start_agl > 5.0
    if in_air:
        print(f"[PX4Cmd] in-air restart at {start_agl:.0f} m AGL — skipping takeoff")
        if cmd._state.mode != "OFFBOARD":
            print(f"[PX4Cmd] mode={cmd._state.mode}, switching to OFFBOARD …")
            e0 = cmd._drone.pose.position.x if cmd._drone else 0.0
            n0 = cmd._drone.pose.position.y if cmd._drone else 0.0
            sp = cmd.make_sp(e0, n0, start_agl)
            for _ in range(40):
                sp.header.stamp = cmd.get_clock().now().to_msg()
                cmd._sp_pub.publish(sp)
                rclpy.spin_once(cmd, timeout_sec=0.05)
            cmd.set_mode("OFFBOARD")

    # HOLDTEST mode: position-hold gate only (Phase 3 test)
    if os.environ.get("HOLDTEST"):
        e0 = cmd._drone.pose.position.x if cmd._drone else 0.0
        n0 = cmd._drone.pose.position.y if cmd._drone else 0.0

        if not in_air:
            if not cmd.engage_offboard(e0, n0, HOLD_AGL):
                print("[PX4Cmd] engage_offboard failed"); stop.set()
                cmd.destroy_node(); rclpy.shutdown(); return

        sp  = cmd.make_sp(e0, n0, HOLD_AGL)
        print(f"[PX4Cmd] === HOLD GATE: {HOLD_AGL:.0f} m AGL for 40 s ===")
        t_end = time.time() + 40.0; t_log = 0.0
        while time.time() < t_end:
            sp.header.stamp = cmd.get_clock().now().to_msg()
            cmd._sp_pub.publish(sp)
            rclpy.spin_once(cmd, timeout_sec=0.02)
            if time.time() - t_log > 3.0 and cmd._drone is not None:
                t_log = time.time()
                ds = cmd._drone.pose.position
                agl = ds.z - HOME_ALT_MSL
                print(f"[PX4Cmd] drift E={ds.x-e0:+6.1f} N={ds.y-n0:+6.1f}"
                      f"  AGL={agl:4.1f}  dist={math.hypot(ds.x-e0,ds.y-n0):5.1f} m"
                      f"  mode={cmd._state.mode} armed={cmd._state.armed}")
        print("[PX4Cmd] === gate done ===")
        stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

    # ── Full mission ──────────────────────────────────────────────────────────
    if not in_air:
        # Pre-stream at ground-level hold setpoint, then engage OFFBOARD
        e0 = cmd._drone.pose.position.x if cmd._drone else 0.0
        n0 = cmd._drone.pose.position.y if cmd._drone else 0.0
        print(f"[PX4Cmd] engaging OFFBOARD at ground  E={e0:+.1f} N={n0:+.1f} …")
        if not cmd.engage_offboard(e0, n0, 0.5):
            print("[PX4Cmd] ABORT: engage_offboard failed")
            stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

        # Takeoff to cruise altitude
        if not cmd.takeoff(TAKEOFF_ALT):
            print("[PX4Cmd] ABORT: takeoff failed")
            stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

    # Hold briefly at cruise altitude before starting waypoints
    if cmd._drone is not None:
        hold_e = cmd._drone.pose.position.x
        hold_n = cmd._drone.pose.position.y
    else:
        hold_e, hold_n = 0.0, 0.0
    print(f"[PX4Cmd] holding 5 s at {TAKEOFF_ALT:.0f} m AGL …")
    t_hold = time.time() + 5.0
    sp = cmd.make_sp(hold_e, hold_n, TAKEOFF_ALT)
    while time.time() < t_hold:
        sp.header.stamp = cmd.get_clock().now().to_msg()
        cmd._sp_pub.publish(sp)
        rclpy.spin_once(cmd, timeout_sec=0.05)

    # Waypoint navigation
    try:
        for i, (wn, we, wagl) in enumerate(WAYPOINTS):
            print(f"[PX4Cmd] WP {i+1}/{len(WAYPOINTS)}  N={wn:+.0f} E={we:+.0f}"
                  f"  AGL={wagl:.0f} m")
            reached = cmd.go_to_ned(wn, we, wagl, timeout=WAYPOINT_TIMEOUT)
            if cmd._drone is not None:
                ds  = cmd._drone.pose.position
                agl = ds.z - HOME_ALT_MSL
                dx  = ds.x - we; dy = ds.y - wn
                print(f"[PX4Cmd] WP {i+1} {'ARRIVED ✓' if reached else 'TIMEOUT'}  "
                      f"pos E={ds.x:+.1f} N={ds.y:+.1f} AGL={agl:.1f} m  "
                      f"horiz_err={math.hypot(dx, dy):.1f} m")
            time.sleep(1.0)

        # Hold at final waypoint — Ctrl-C triggers RTL
        print("[PX4Cmd] Holding at target — Ctrl-C to RTL")
        e_tgt = WAYPOINTS[-1][1]; n_tgt = WAYPOINTS[-1][0]
        sp_hold = cmd.make_sp(e_tgt, n_tgt, WAYPOINTS[-1][2])
        try:
            while True:
                sp_hold.header.stamp = cmd.get_clock().now().to_msg()
                cmd._sp_pub.publish(sp_hold)
                rclpy.spin_once(cmd, timeout_sec=0.1)
        except KeyboardInterrupt:
            print("[PX4Cmd] Ctrl-C — RTL")
            cmd.set_mode("RTL")
            cmd._spin_until(lambda: not cmd._state.armed, timeout=150.0)
            print("[PX4Cmd] Disarmed — landed ✓")

    except Exception as exc:
        print(f"[PX4Cmd] mission aborted: {exc}")
    finally:
        stop.set()
        try:
            cmd.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
