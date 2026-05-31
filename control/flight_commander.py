#!/usr/bin/env python3
"""
Autonomous flight commander — ROS2 / MAVROS2 version.

Replaces run_flight.py. Uses MAVROS2 topics and services for all flight
commands. Vision position injection is handled by anyloc/ros2_node.py
(publishes /mavros/vision_pose/pose → MAVROS2 → VISION_POSITION_ESTIMATE).

Architecture:
  anyloc/ros2_node.py  →  /mavros/vision_pose/pose  →  MAVROS2  →  ArduPilot EKF3
  this node            →  /mavros/setpoint_position/local  →  MAVROS2  →  ArduPilot

EKF origin and status are handled via MAVROS2 raw MAVLink topics
(/uas1/mavlink_source BEST_EFFORT) — no pymavlink or extra UDP port needed.

Run:
  source /opt/ros/jazzy/setup.bash
  python3 control/flight_commander.py

Prerequisites (running concurrently):
  - ArduPilot SITL  (sim_vehicle.py ... --add-param-file=control/no_gps.parm)
  - stub_bridge.py or Isaac Sim  (physics bridge on UDP 9002)
  - MAVROS2  (control/launch_mavros.sh)
  - AnyLoc ROS2 node  (anyloc/ros2_node.py)
"""

import json
import math
import os
import struct
import sys
import threading
import time

_ROS2_SITE = "/opt/ros/jazzy/lib/python3.12/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import Mavlink, State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)

# ── Home position ──────────────────────────────────────────────────────────────
HOME_LAT     = 23.450868
HOME_LON     = 120.286135
_HOME_CFG    = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "home_elevation.json")
try:
    with open(_HOME_CFG) as _f:
        HOME_ALT_MSL = float(json.load(_f)["centre_elev_m"])
    print(f"[Commander] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m (from {_HOME_CFG})")
except (FileNotFoundError, KeyError):
    HOME_ALT_MSL = 28.17
    print(f"[Commander] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m (default)")

# ── Mission parameters ─────────────────────────────────────────────────────────
TAKEOFF_ALT          = 90.0   # metres AGL
WAYPOINT_RADIUS      =  3.0   # metres — how close counts as reached
WAYPOINT_TIMEOUT     = 60.0   # seconds per waypoint
MIN_LOCALISATION_AGL = 50.0   # metres AGL — below this, VPE is locked to home position;
                               # above this, AnyLoc estimates are used

# NED waypoints (north m, east m, down m) — down = -alt_agl
WAYPOINTS = [
    ( 20.0,   0.0, -90.0),
    ( 20.0,  20.0, -90.0),
    (  0.0,  20.0, -90.0),
    (  0.0,   0.0, -90.0),
]

COS_LAT   = math.cos(math.radians(HOME_LAT))
M_PER_DEG = 111_320.0

ESTIMATE_JSON = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "anyloc", "latest_estimate.json"
)


# ── Stub estimate (written at startup so VPE flows before AnyLoc starts) ──────

def _write_stub_estimate() -> None:
    est = {
        "timestamp": time.time(),
        "est_lat":   HOME_LAT, "est_lon":  HOME_LON,
        "alt_msl_m": HOME_ALT_MSL, "agl_m": 0.0,
        "yaw_deg": 0.0, "score": 1.0, "error_m": 0.0,
    }
    tmp = ESTIMATE_JSON + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(est, fh)
    os.replace(tmp, ESTIMATE_JSON)


# ── Flight commander ROS2 node ─────────────────────────────────────────────────

class FlightCommander(rclpy.node.Node):
    def __init__(self):
        super().__init__("flight_commander")

        self._state               = State()
        self._local_pos           = None
        self._ekf_flags           = 0     # from EKF_STATUS_REPORT (msg 193)
        self._gps_origin_received = False # set when GPS_GLOBAL_ORIGIN (msg 49) arrives
        self._last_motor_pwm      = None  # from SERVO_OUTPUT_RAW (msg 36)

        # Subscribers
        self.create_subscription(State, "/mavros/state",
                                 self._cb_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_local_pos, _SENSOR_QOS)
        # Raw MAVLink from FCU — BEST_EFFORT matches mavros_router's QoS
        self.create_subscription(Mavlink, "/uas1/mavlink_source",
                                 self._cb_mavlink, _SENSOR_QOS)

        # Publishers
        self._pos_pub    = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 1)
        self._vpe_pub    = self.create_publisher(
            PoseWithCovarianceStamped, "/mavros/vision_pose/pose_cov", 1)
        self._origin_pub = self.create_publisher(
            GeoPointStamped, "/mavros/global_position/set_gp_origin", 1)

        # Service clients
        self._arm_cli  = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._cmd_cli  = self.create_client(CommandLong, "/mavros/cmd/command")
        self._mode_cli = self.create_client(SetMode,     "/mavros/set_mode")
        self._tof_cli  = self.create_client(CommandTOL,  "/mavros/cmd/takeoff")

        self.get_logger().info("Flight commander ready")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def start_vision_thread(self, stop_event: threading.Event) -> threading.Thread:
        """
        Background thread: publishes VPE to /mavros/vision_pose/pose_cov at 5 Hz.

        Two-phase strategy:
          Phase 1 — below MIN_LOCALISATION_AGL:
            VPE = home position (0, 0) with 0.1 m² covariance.
            The drone is on the ground at the known home position so this is accurate.
            EKF sets POS_HORIZ_ABS and holds XY at home while baro handles altitude.

          Phase 2 — above MIN_LOCALISATION_AGL:
            VPE = AnyLoc estimate from latest_estimate.json.
            Camera can see landmarks at altitude → real visual localisation.
            Covariance scales with AnyLoc's reported error_m (min 1 m²).
        """
        def _loop():
            last_mtime   = 0.0
            anyloc_est   = None   # (east, north, yaw_rad, cov_xy) — set once above altitude
            n_sent       = 0
            n_anyloc     = 0
            phase_logged = False

            _write_stub_estimate()

            while not stop_event.is_set():
                t0 = time.time()

                # Current AGL from EKF local position (ENU z = up from home = AGL)
                agl = 0.0
                if self._local_pos is not None:
                    agl = max(0.0, self._local_pos.pose.position.z)

                # ── Phase 2: above localisation altitude — read AnyLoc estimate ──
                if agl >= MIN_LOCALISATION_AGL:
                    if not phase_logged:
                        print(f"[Commander] AGL {agl:.0f} m ≥ {MIN_LOCALISATION_AGL:.0f} m "
                              "— switching VPE to AnyLoc")
                        phase_logged = True
                    try:
                        mtime = os.path.getmtime(ESTIMATE_JSON)
                        if mtime != last_mtime:
                            with open(ESTIMATE_JSON) as fh:
                                est = json.load(fh)
                            # Only accept estimates taken at altitude (not the ground stub)
                            if est.get("agl_m", 0.0) >= MIN_LOCALISATION_AGL:
                                lat  = est["est_lat"]; lon = est["est_lon"]
                                yaw  = math.radians(est.get("yaw_deg", 0.0))
                                north = (lat - HOME_LAT) * M_PER_DEG
                                east  = (lon - HOME_LON) * M_PER_DEG * COS_LAT
                                cov_xy = max(1.0, est.get("error_m", 10.0) ** 2)
                                anyloc_est = (east, north, yaw, cov_xy)
                                last_mtime = mtime
                                n_anyloc += 1
                                if n_anyloc == 1:
                                    print(f"[Commander] First AnyLoc VPE: "
                                          f"N={north:+.1f} E={east:+.1f} m  "
                                          f"err={est.get('error_m', 0):.1f} m")
                    except (FileNotFoundError, KeyError, json.JSONDecodeError):
                        pass

                # ── Choose position and covariance ────────────────────────────────
                if agl >= MIN_LOCALISATION_AGL and anyloc_est is not None:
                    east_v, north_v, yaw_v, cov_xy = anyloc_est
                else:
                    # Phase 1 — home position.  The drone is at the known home point;
                    # use 0.1 m² so EKF marks POS_HORIZ_ABS "good" immediately.
                    east_v, north_v, yaw_v, cov_xy = 0.0, 0.0, 0.0, 0.1

                # ── Publish VPE ───────────────────────────────────────────────────
                hy  = yaw_v / 2.0
                msg = PoseWithCovarianceStamped()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"   # ENU: x=East, y=North, z=Up
                msg.pose.pose.position.x    = east_v
                msg.pose.pose.position.y    = north_v
                msg.pose.pose.position.z    = 0.0   # z irrelevant — huge cov below
                msg.pose.pose.orientation.z = math.sin(hy)
                msg.pose.pose.orientation.w = math.cos(hy)
                cov = [0.0] * 36
                cov[0]  = cov_xy  # x variance
                cov[7]  = cov_xy  # y variance
                cov[14] = 1e6     # z → EKF ignores VPE altitude, uses barometer
                cov[21] = 0.09    # roll  (0.3 rad std)
                cov[28] = 0.09    # pitch
                cov[35] = 0.09    # yaw
                msg.pose.covariance = cov
                self._vpe_pub.publish(msg)
                n_sent += 1
                if n_sent == 1:
                    print("[Commander] VPE thread started — Phase 1 (home position)")

                elapsed = time.time() - t0
                time.sleep(max(0.0, 0.2 - elapsed))   # 5 Hz

        t = threading.Thread(target=_loop, daemon=True)
        t.start()
        return t

    def _cb_state(self, msg: State):
        self._state = msg

    def _cb_local_pos(self, msg: PoseStamped):
        self._local_pos = msg

    def _cb_mavlink(self, msg: Mavlink) -> None:
        raw = b"".join(x.to_bytes(8, "little") for x in msg.payload64)[:msg.len]
        if msg.msgid == 193 and len(raw) >= 22:  # EKF_STATUS_REPORT: flags at byte 20
            self._ekf_flags = struct.unpack_from("<H", raw, 20)[0]
        elif msg.msgid == 49:  # GPS_GLOBAL_ORIGIN
            self._gps_origin_received = True
        elif msg.msgid == 36 and len(raw) >= 12:  # SERVO_OUTPUT_RAW: 4 motors after uint32
            ch = struct.unpack_from("<4H", raw, 4)
            self._last_motor_pwm = ch

    def set_ekf_origin(self, lat: float, lon: float, alt_msl_m: float,
                       timeout: float = 30.0) -> bool:
        """
        Publish GPS global origin via /mavros/global_position/set_gp_origin and
        confirm receipt by waiting for GPS_GLOBAL_ORIGIN (msg 49) from ArduPilot.
        MAVROS2's global_position plugin converts GeoPointStamped → SET_GPS_GLOBAL_ORIGIN.
        """
        self.get_logger().info(
            f"Setting EKF origin: {lat:.6f}°N {lon:.6f}°E {alt_msl_m:.1f} m MSL")
        self._gps_origin_received = False

        origin_msg = GeoPointStamped()
        origin_msg.position.latitude  = lat
        origin_msg.position.longitude = lon
        origin_msg.position.altitude  = alt_msl_m

        deadline = time.time() + timeout
        attempt  = 0
        while time.time() < deadline:
            attempt += 1
            origin_msg.header.stamp = self.get_clock().now().to_msg()
            self._origin_pub.publish(origin_msg)
            print(f"[Commander] SET_GPS_GLOBAL_ORIGIN attempt {attempt} …")

            t_end = time.time() + 2.0
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._gps_origin_received:
                    self.get_logger().info("EKF origin confirmed via GPS_GLOBAL_ORIGIN ✓")
                    return True

        self.get_logger().warn(
            f"EKF origin not confirmed after {attempt} attempts — continuing anyway")
        return False

    # ── Blocking helpers ──────────────────────────────────────────────────────

    def _spin_until(self, condition, timeout=30.0, interval=0.05):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=interval)
            if condition():
                return True
        return False

    def wait_connected(self, timeout=60.0) -> bool:
        self.get_logger().info("Waiting for MAVROS2 connection …")
        ok = self._spin_until(lambda: self._state.connected, timeout)
        if ok:
            self.get_logger().info("MAVROS2 connected ✓")
        return ok

    def wait_ekf_pos(self, timeout=90.0) -> bool:
        """
        Block until EKF_POS_HORIZ_ABS is set (bit 4 of EKF_STATUS_REPORT flags).
        Uses _ekf_flags updated by _cb_mavlink via /uas1/mavlink_source.
        /mavros/estimator_status is NOT published in MAVROS2 Jazzy 2.14 at useful rate.
        """
        EKF_POS_HORIZ_ABS = 0x010
        _FLAG_NAMES = {
            0x001: "ATT", 0x002: "VEL_H", 0x004: "VEL_V",
            0x008: "POS_H_REL", 0x010: "POS_H_ABS", 0x020: "POS_V_ABS",
            0x040: "POS_V_AGL", 0x080: "CONST_POS",
        }
        self.get_logger().info(
            "Waiting for EKF POS_ABS — VPE must reach ArduPilot EKF3 …")

        deadline   = time.time() + timeout
        last_print = 0.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._ekf_flags & EKF_POS_HORIZ_ABS:
                self.get_logger().info("EKF POS_ABS ✓")
                return True
            now = time.time()
            if now - last_print > 5.0:
                active = " | ".join(n for v, n in _FLAG_NAMES.items()
                                   if self._ekf_flags & v)
                self.get_logger().warn(
                    f"EKF flags 0x{self._ekf_flags:03x}: [{active or 'none'}]"
                    " — waiting for POS_H_ABS")
                last_print = now

        self.get_logger().warn("EKF POS_ABS timeout — check VPE flow and EKF origin")
        return False

    def set_mode(self, mode: str, timeout=10.0) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._mode_cli.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if future.done() and future.result().mode_sent:
            self.get_logger().info(f"Mode {mode} set ✓")
            return True
        self.get_logger().warn(f"Mode {mode} failed")
        return False

    def arm(self, timeout=10.0) -> bool:
        # Regular arm first
        req = CommandBool.Request()
        req.value = True
        future = self._arm_cli.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if future.done() and future.result().success:
            self.get_logger().info("Armed ✓")
            return True

        # Regular arm failed (GPS bad fix, VisOdom, etc.) — force arm
        self.get_logger().warn("Regular arm failed — retrying with force arm …")
        req2 = CommandLong.Request()
        req2.command  = 400      # MAV_CMD_COMPONENT_ARM_DISARM
        req2.param1   = 1.0      # arm
        req2.param2   = 21196.0  # force arm magic (bypasses all pre-arm checks)
        future2 = self._cmd_cli.call_async(req2)
        deadline = time.time() + timeout
        while not future2.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if future2.done() and future2.result().success:
            self.get_logger().info("Force armed ✓")
            return True

        self.get_logger().warn("Arm failed (regular + force)")
        return False

    def takeoff(self, alt_m: float, climb_rate: float = 1.0,
                timeout: float = 180.0) -> bool:
        """
        Issue MAV_CMD_NAV_TAKEOFF to break ArduPilot out of "landed" state,
        then track climb with rate-limited position setpoints.

        Position setpoints alone cannot trigger liftoff — ArduPilot keeps motors
        at idle while in landed state regardless of the setpoint altitude.
        NAV_TAKEOFF is the required signal to transition landed → flying.
        """
        self.get_logger().info(
            f"Climbing to {alt_m:.0f} m AGL at {climb_rate:.1f} m/s …")

        # NAV_TAKEOFF breaks ArduPilot out of "landed" state so the position
        # controller is allowed to command above-hover throttle.
        req = CommandTOL.Request()
        req.altitude = alt_m
        future = self._tof_cli.call_async(req)
        tof_deadline = time.time() + 10.0
        while not future.done() and time.time() < tof_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if future.done():
            self.get_logger().info(
                f"NAV_TAKEOFF {'accepted' if future.result().success else 'rejected'}")
        else:
            self.get_logger().warn("NAV_TAKEOFF service timed out — continuing anyway")

        # Altitude is read from /mavros/local_position/pose (ENU z = AGL).
        # Altitude read from /mavros/local_position/pose (ENU z = AGL).
        STEP_DT    = 0.2
        setpt_alt  = 0.0
        deadline   = time.time() + timeout
        last_print = 0.0

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=STEP_DT)

            if setpt_alt < alt_m:
                setpt_alt = min(setpt_alt + climb_rate * STEP_DT, alt_m)

            msg = PoseStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.z = setpt_alt
            msg.pose.orientation.w = 1.0
            self._pos_pub.publish(msg)

            if self._local_pos is not None:
                agl = self._local_pos.pose.position.z   # ENU z = up = AGL
                now = time.time()
                if now - last_print > 2.0:
                    mot = self._last_motor_pwm
                    mot_str = (f"  motors={list(mot)}" if mot else "")
                    print(f"[Commander] AGL = {agl:.1f} m  "
                          f"(setpt {setpt_alt:.1f} m  target {alt_m:.0f} m){mot_str}")
                    last_print = now
                if setpt_alt >= alt_m and abs(agl - alt_m) < 5.0:
                    self.get_logger().info(f"Reached {alt_m:.0f} m AGL ✓")
                    return True

        self.get_logger().warn("Takeoff altitude timeout")
        return False

    def go_to_ned(self, north: float, east: float, down: float,
                  timeout=60.0) -> bool:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.orientation.w = 1.0

        # /mavros/setpoint_position/local expects ENU: x=east, y=north, z=up
        # /mavros/local_position/pose is also ENU
        # Our waypoints are NED (north, east, down=-alt_agl)
        enu_x = east            # east
        enu_y = north           # north
        enu_z = -down           # up = -down

        deadline = time.time() + timeout
        while time.time() < deadline:
            msg.header.stamp     = self.get_clock().now().to_msg()
            msg.pose.position.x  = enu_x
            msg.pose.position.y  = enu_y
            msg.pose.position.z  = enu_z
            self._pos_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._local_pos is not None:
                p = self._local_pos.pose.position
                dist = math.sqrt((p.x - enu_x)**2 +
                                 (p.y - enu_y)**2 +
                                 (p.z - enu_z)**2)
                if dist <= WAYPOINT_RADIUS:
                    return True
        return False


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    cmd = FlightCommander()

    # Step 1: start VPE thread immediately so VisOdom stays healthy
    stop_ev = threading.Event()
    cmd.start_vision_thread(stop_ev)

    # Step 2: wait for MAVROS2 to connect to SITL
    if not cmd.wait_connected(timeout=60.0):
        print("[Commander] MAVROS2 not connected — is SITL + launch_mavros.sh running?")
        stop_ev.set(); rclpy.shutdown(); return

    # Step 3: send EKF origin and wait for ArduPilot to echo GPS_GLOBAL_ORIGIN.
    cmd.set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT_MSL, timeout=30.0)
    time.sleep(0.3)

    # Step 4: arm in STABILIZE first — only needs EKF attitude, not position.
    # This bypasses the GPS/VisOdom position requirement that blocks GUIDED arming.
    cmd.set_mode("STABILIZE")
    time.sleep(0.5)
    print("[Commander] Arming in STABILIZE (no EKF position required) …")
    if not cmd.arm():
        print("[Commander] Arm failed in STABILIZE — check IMU/bridge")
        stop_ev.set(); cmd.destroy_node(); rclpy.shutdown(); return

    # Step 5: switch to GUIDED now that motors are armed
    cmd.set_mode("GUIDED")
    time.sleep(0.5)

    # Step 6: wait for EKF POS_ABS — VPE must be accepted before takeoff command
    if not cmd.wait_ekf_pos(timeout=60.0):
        print("[Commander] EKF POS_ABS not reached — check VPE flow")
        stop_ev.set(); cmd.destroy_node(); rclpy.shutdown(); return

    # Step 7: Takeoff
    if not cmd.takeoff(TAKEOFF_ALT):
        print("[Commander] Takeoff failed")
        stop_ev.set(); cmd.destroy_node(); rclpy.shutdown(); return

    # Hold altitude for 3 s with continuous setpoints — GUIDED mode needs a
    # steady stream of position targets or the drone drifts after NAV_TAKEOFF.
    print(f"[Commander] Holding {TAKEOFF_ALT:.0f} m …")
    cmd.go_to_ned(0.0, 0.0, -TAKEOFF_ALT, timeout=3.0)

    # Step 8: Waypoints
    for i, (n, e, d) in enumerate(WAYPOINTS):
        print(f"[Commander] WP {i+1}/{len(WAYPOINTS)}  "
              f"N={n:+.0f} E={e:+.0f} ALT={-d:.0f} m AGL")
        reached = cmd.go_to_ned(n, e, d, timeout=WAYPOINT_TIMEOUT)
        print(f"[Commander] WP {i+1} {'✓' if reached else 'timeout'}")
        time.sleep(1.0)

    # Step 9: RTL
    print("[Commander] Mission complete — RTL")
    cmd.set_mode("RTL")

    # Wait for disarm — 90 m AGL descent at ~1.5 m/s takes ~60 s + landing buffer
    cmd._spin_until(lambda: not cmd._state.armed, timeout=150.0)
    print("[Commander] Disarmed — landed ✓")

    stop_ev.set()
    cmd.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
