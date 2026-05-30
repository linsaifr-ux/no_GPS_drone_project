#!/usr/bin/env python3
"""
Autonomous flight commander — ROS2 / MAVROS2 version.

Replaces run_flight.py. Uses MAVROS2 topics and services for all flight
commands. Vision position injection is handled by anyloc/ros2_node.py
(publishes /mavros/vision_pose/pose → MAVROS2 → VISION_POSITION_ESTIMATE).

Architecture:
  anyloc/ros2_node.py  →  /mavros/vision_pose/pose  →  MAVROS2  →  ArduPilot EKF3
  this node            →  /mavros/setpoint_position/local  →  MAVROS2  →  ArduPilot

Uses pymavlink only for SET_GPS_GLOBAL_ORIGIN + SET_HOME_POSITION at startup
(MAVROS2 Jazzy 2.14 has no /mavros/global_position/set_gp_origin service).

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
import sys
import threading
import time

_ROS2_SITE = "/opt/ros/jazzy/lib/python3.12/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode
from pymavlink import mavutil

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
TAKEOFF_ALT     = 90.0   # metres AGL
WAYPOINT_RADIUS =  3.0   # metres — how close counts as reached
WAYPOINT_TIMEOUT = 60.0  # seconds per waypoint

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


# ── EKF origin via pymavlink (no MAVROS2 service for this in Jazzy 2.14) ───────

def _set_ekf_origin_confirmed(lat: float, lon: float, alt_msl_m: float,
                               timeout: float = 30.0,
                               connection_str: str = "udp:localhost:14550") -> bool:
    """
    Send SET_GPS_GLOBAL_ORIGIN + SET_HOME_POSITION and retry until ArduPilot
    confirms by sending back a GPS_GLOBAL_ORIGIN message.

    ArduPilot broadcasts GPS_GLOBAL_ORIGIN when it accepts the origin —
    this is the only reliable confirmation that the message was processed.
    Returns True on confirmation, False on timeout.
    """
    mav = mavutil.mavlink_connection(connection_str, source_system=254,
                                     dialect="ardupilotmega")
    print("[Commander] Waiting for HEARTBEAT (EKF origin setup) …")
    if not mav.wait_heartbeat(timeout=30):
        print("[Commander] No heartbeat on UDP:14550")
        mav.close()
        return False

    deadline  = time.time() + timeout
    attempt   = 0
    confirmed = False

    while time.time() < deadline:
        attempt += 1
        ts = int(time.time() * 1e6)
        mav.mav.set_gps_global_origin_send(
            mav.target_system,
            int(lat * 1e7), int(lon * 1e7), int(alt_msl_m * 1e3), ts)
        mav.mav.set_home_position_send(
            mav.target_system,
            int(lat * 1e7), int(lon * 1e7), int(alt_msl_m * 1e3),
            0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0, ts)
        print(f"[Commander] SET_GPS_GLOBAL_ORIGIN attempt {attempt} …")

        # Wait up to 2 s for GPS_GLOBAL_ORIGIN echo from ArduPilot
        t_end = time.time() + 2.0
        while time.time() < t_end:
            msg = mav.recv_match(type="GPS_GLOBAL_ORIGIN", blocking=False)
            if msg is not None:
                print(f"[Commander] EKF origin confirmed ✓  "
                      f"(attempt {attempt}, "
                      f"lat={msg.latitude/1e7:.6f} lon={msg.longitude/1e7:.6f})")
                confirmed = True
                break
            time.sleep(0.05)

        if confirmed:
            break

    mav.close()
    if not confirmed:
        print(f"[Commander] EKF origin not confirmed after {attempt} attempts — "
              "check SITL console for 'EKF3 IMU0 origin set'")
    return confirmed


# ── Flight commander ROS2 node ─────────────────────────────────────────────────

class FlightCommander(rclpy.node.Node):
    def __init__(self):
        super().__init__("flight_commander")

        self._state     = State()
        self._local_pos = None

        # Subscribers
        self.create_subscription(State, "/mavros/state",
                                 self._cb_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_local_pos, 10)

        # Publishers
        self._pos_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 1)
        # Use pose_cov so we can set z covariance = infinity → ArduPilot ignores
        # VPE z and relies entirely on barometer for altitude. PoseStamped would
        # send zero z covariance (100% certain), which fails the EKF innovation
        # gate when the stub z differs from barometer altitude.
        self._vpe_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/mavros/vision_pose/pose_cov", 1)

        # Service clients
        self._arm_cli  = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._cmd_cli  = self.create_client(CommandLong, "/mavros/cmd/command")
        self._mode_cli = self.create_client(SetMode,     "/mavros/set_mode")
        self._tof_cli  = self.create_client(CommandTOL,  "/mavros/cmd/takeoff")

        self.get_logger().info("Flight commander ready")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def start_vision_thread(self, stop_event: threading.Event) -> threading.Thread:
        """
        Background thread: reads latest_estimate.json and publishes VPE to
        /mavros/vision_pose/pose at 5 Hz. Sends home-position stub when the
        file doesn't exist or is stale, so VisOdom stays healthy even before
        AnyLoc produces its first anchor.
        """
        def _loop():
            last_mtime  = 0.0
            current_est = (0.0, 0.0, 0.0, 0.0)   # north, east, down, yaw_rad
            n_sent = 0

            # Write stub at home so there's always a fresh file to read
            _write_stub_estimate()

            while not stop_event.is_set():
                t0 = time.time()
                try:
                    mtime = os.path.getmtime(ESTIMATE_JSON)
                    if mtime != last_mtime:
                        with open(ESTIMATE_JSON) as fh:
                            est = json.load(fh)
                        lat = est["est_lat"]; lon = est["est_lon"]
                        alt = est["alt_msl_m"]
                        yaw = math.radians(est.get("yaw_deg", 0.0))
                        north = (lat - HOME_LAT) * M_PER_DEG
                        east  = (lon - HOME_LON) * M_PER_DEG * COS_LAT
                        down  = -(alt - HOME_ALT_MSL)
                        current_est = (north, east, down, yaw)
                        last_mtime  = mtime
                except (FileNotFoundError, KeyError, json.JSONDecodeError):
                    pass

                north, east, down, yaw = current_est

                hy = yaw / 2.0
                msg = PoseWithCovarianceStamped()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.pose.pose.position.x = north
                msg.pose.pose.position.y = east
                msg.pose.pose.position.z = down   # z value irrelevant — huge cov below
                msg.pose.pose.orientation.z = math.sin(hy)
                msg.pose.pose.orientation.w = math.cos(hy)
                # Covariance matrix (6x6 upper-triangle, row-major):
                # x,y: 20m std (400 m²)   z: 1e6 m² = ignore z entirely
                cov = [0.0] * 36
                cov[0]  = 400.0   # x variance (20m std)
                cov[7]  = 400.0   # y variance
                cov[14] = 1e6     # z variance → EKF ignores VPE z, uses baro
                cov[21] = 0.09    # roll variance (0.3 rad std)
                cov[28] = 0.09    # pitch variance
                cov[35] = 0.09    # yaw variance
                msg.pose.covariance = cov
                self._vpe_pub.publish(msg)
                n_sent += 1
                if n_sent == 1:
                    print("[Commander] First VPE stub published to /mavros/vision_pose/pose")

                elapsed = time.time() - t0
                time.sleep(max(0.0, 0.2 - elapsed))   # 5 Hz

        t = threading.Thread(target=_loop, daemon=True)
        t.start()
        return t

    def _cb_state(self, msg: State):
        self._state = msg

    def _cb_local_pos(self, msg: PoseStamped):
        self._local_pos = msg

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
        Block until EKF_POS_HORIZ_ABS is set.
        Reads EKF_STATUS_REPORT directly via pymavlink UDP:14550 rather than
        relying on /mavros/estimator_status (not published in Jazzy 2.14).
        Continues spinning rclpy so the VPE thread keeps publishing.
        """
        EKF_POS_HORIZ_ABS = 1 << 4
        self.get_logger().info(
            "Waiting for EKF POS_ABS — VPE must reach ArduPilot EKF3 …")

        try:
            mav = mavutil.mavlink_connection(
                "udp:localhost:14550", source_system=254,
                dialect="ardupilotmega")
            mav.wait_heartbeat(timeout=10)
        except Exception as e:
            self.get_logger().warn(f"pymavlink EKF monitor: {e}")
            return False

        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            msg = mav.recv_match(type="EKF_STATUS_REPORT", blocking=False)
            if msg and (msg.flags & EKF_POS_HORIZ_ABS):
                mav.close()
                self.get_logger().info("EKF POS_ABS ✓")
                return True
        mav.close()
        self.get_logger().warn("EKF POS_ABS timeout — check VPE + EKF origin")
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
        Rate-limited setpoint ramp instead of NAV_TAKEOFF.

        Publishes a position setpoint that walks up at climb_rate m/s.
        The altitude error is always ≤ 1 step (0.2 s × climb_rate ≈ 0.2 m),
        so the PID integral never winds up and oscillation is prevented.
        """
        self.get_logger().info(
            f"Climbing to {alt_m:.0f} m AGL at {climb_rate:.1f} m/s …")

        try:
            mav = mavutil.mavlink_connection("udp:localhost:14550",
                                             source_system=254,
                                             dialect="ardupilotmega")
            mav.wait_heartbeat(timeout=5)
        except Exception as e:
            self.get_logger().warn(f"Altitude monitor: {e}")
            return False

        STEP_DT    = 0.2          # setpoint publish interval (s)
        setpt_alt  = 0.0          # current commanded altitude (m AGL, ENU z)
        deadline   = time.time() + timeout
        last_print = 0.0

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=STEP_DT)

            # Ramp commanded altitude
            if setpt_alt < alt_m:
                setpt_alt = min(setpt_alt + climb_rate * STEP_DT, alt_m)

            # Publish ENU setpoint (x=east, y=north, z=up)
            msg = PoseStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.z = setpt_alt
            msg.pose.orientation.w = 1.0
            self._pos_pub.publish(msg)

            # Read actual AGL
            lp = mav.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if lp:
                agl = -lp.z
                now = time.time()
                if now - last_print > 2.0:
                    print(f"[Commander] AGL = {agl:.1f} m  "
                          f"(setpt {setpt_alt:.1f} m  target {alt_m:.0f} m)")
                    last_print = now
                if setpt_alt >= alt_m and abs(agl - alt_m) < 5.0:
                    mav.close()
                    self.get_logger().info(f"Reached {alt_m:.0f} m AGL ✓")
                    return True

        mav.close()
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

    # Step 3: send EKF origin and wait for ArduPilot confirmation.
    # Retries until ArduPilot echoes GPS_GLOBAL_ORIGIN (30 s timeout).
    if not _set_ekf_origin_confirmed(HOME_LAT, HOME_LON, HOME_ALT_MSL, timeout=30.0):
        print("[Commander] EKF origin not confirmed — continuing anyway")
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


def _emergency_stabilize():
    """Switch ArduPilot to STABILIZE via pymavlink on Ctrl-C / crash.
    Prevents VisOdom-loss failsafe from commanding descent after commander exits."""
    try:
        mav = mavutil.mavlink_connection("udp:localhost:14550", source_system=254)
        if mav.wait_heartbeat(timeout=3):
            mode_id = mav.mode_mapping().get("STABILIZE", 0)
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id, 0, 0, 0, 0, 0)
            print("[Commander] Switched to STABILIZE on exit")
        mav.close()
    except Exception:
        pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        _emergency_stabilize()
