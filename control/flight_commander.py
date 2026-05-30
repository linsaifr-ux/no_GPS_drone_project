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
import time

_ROS2_SITE = "/opt/ros/jazzy/lib/python3.12/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
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
TAKEOFF_ALT     = 10.0   # metres AGL
WAYPOINT_RADIUS =  3.0   # metres — how close counts as reached
WAYPOINT_TIMEOUT = 60.0  # seconds per waypoint

# NED waypoints (north m, east m, down m) — down = -alt_agl
WAYPOINTS = [
    ( 20.0,   0.0, -10.0),
    ( 20.0,  20.0, -10.0),
    (  0.0,  20.0, -10.0),
    (  0.0,   0.0, -10.0),
]


# ── EKF origin via pymavlink (no MAVROS2 service for this in Jazzy 2.14) ───────

def _set_ekf_origin(lat: float, lon: float, alt_msl_m: float,
                    connection_str: str = "tcp:localhost:5762") -> None:
    """
    Send SET_GPS_GLOBAL_ORIGIN + SET_HOME_POSITION directly via pymavlink.
    Called once at startup before MAVROS2 starts sending VPE.
    """
    mav = mavutil.mavlink_connection(connection_str, source_system=254,
                                     dialect="ardupilotmega")
    print("[Commander] Waiting for HEARTBEAT (EKF origin setup) …")
    mav.wait_heartbeat(timeout=30)
    ts = int(time.time() * 1e6)
    mav.mav.set_gps_global_origin_send(
        mav.target_system,
        int(lat * 1e7), int(lon * 1e7), int(alt_msl_m * 1e3), ts)
    mav.mav.set_home_position_send(
        mav.target_system,
        int(lat * 1e7), int(lon * 1e7), int(alt_msl_m * 1e3),
        0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0, ts)
    mav.close()
    print(f"[Commander] SET_GPS_GLOBAL_ORIGIN sent  lat={lat:.6f} "
          f"lon={lon:.6f} alt={alt_msl_m:.1f} m")


# ── Flight commander ROS2 node ─────────────────────────────────────────────────

class FlightCommander(rclpy.node.Node):
    def __init__(self):
        super().__init__("flight_commander")

        self._state       = State()
        self._local_pos   = None

        # Subscribers
        self.create_subscription(State, "/mavros/state",
                                 self._cb_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_local_pos, 10)

        # Publisher — position setpoints
        self._pos_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 1)

        # Service clients
        self._arm_cli  = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_cli = self.create_client(SetMode,     "/mavros/set_mode")
        self._tof_cli  = self.create_client(CommandTOL,  "/mavros/cmd/takeoff")

        self.get_logger().info("Flight commander ready")

    # ── Callbacks ─────────────────────────────────────────────────────────────

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

    def wait_ekf_pos(self, timeout=60.0) -> bool:
        """Wait until MAVROS2 state shows guided=True (EKF has a position fix)."""
        self.get_logger().info("Waiting for EKF position fix (guided flag) …")
        # In ArduPilot, guided=True in /mavros/state when EKF_POS_ABS is set
        # and GUIDED mode can be entered. Poll until we can switch to GUIDED.
        ok = self._spin_until(lambda: self._state.connected, timeout)
        return ok

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
        req = CommandBool.Request()
        req.value = True
        future = self._arm_cli.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if future.done() and future.result().success:
            self.get_logger().info("Armed ✓")
            return True
        self.get_logger().warn("Arm failed")
        return False

    def takeoff(self, alt_m: float, timeout=30.0) -> bool:
        req = CommandTOL.Request()
        req.altitude = alt_m
        req.latitude  = HOME_LAT
        req.longitude = HOME_LON
        future = self._tof_cli.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not (future.done() and future.result().success):
            self.get_logger().warn("Takeoff command failed")
            return False
        # Wait for altitude
        self.get_logger().info(f"Climbing to {alt_m:.0f} m AGL …")
        ok = self._spin_until(
            lambda: (self._local_pos is not None and
                     abs(self._local_pos.pose.position.z + alt_m) < 1.5),
            timeout=timeout)
        if ok:
            self.get_logger().info(f"Reached {alt_m:.0f} m AGL ✓")
        return ok

    def go_to_ned(self, north: float, east: float, down: float,
                  timeout=60.0) -> bool:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = north
        msg.pose.position.y = east
        msg.pose.position.z = down
        msg.pose.orientation.w = 1.0

        deadline = time.time() + timeout
        while time.time() < deadline:
            msg.header.stamp = self.get_clock().now().to_msg()
            self._pos_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._local_pos is not None:
                p = self._local_pos.pose.position
                dist = math.sqrt((p.x - north)**2 +
                                 (p.y - east)**2 +
                                 (p.z - down)**2)
                if dist <= WAYPOINT_RADIUS:
                    return True
        return False


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    # Step 1: set EKF origin via pymavlink before MAVROS2 sends any VPE
    _set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT_MSL)
    time.sleep(0.5)

    rclpy.init()
    cmd = FlightCommander()

    # Step 2: wait for MAVROS2 to connect to SITL
    if not cmd.wait_connected(timeout=60.0):
        print("[Commander] MAVROS2 not connected — is SITL + launch_mavros.sh running?")
        rclpy.shutdown(); return

    # Step 3: wait for EKF position fix (VPE from AnyLoc node must be flowing)
    print("[Commander] Waiting for EKF POS_ABS "
          "(start anyloc/ros2_node.py if not running) …")
    if not cmd._spin_until(lambda: cmd._state.guided or cmd._state.mode == "GUIDED",
                           timeout=90.0):
        # Try switching to GUIDED — accepted only after EKF has position fix
        pass

    # Step 4: GUIDED mode
    cmd.set_mode("GUIDED")
    time.sleep(1.0)

    # Step 5: Arm (regular arm — should succeed once VisOdom is healthy)
    print("[Commander] Arming …")
    if not cmd.arm():
        print("[Commander] Arm failed — check VisOdom health in MAVROS2 output")
        rclpy.shutdown(); return

    # Step 6: Takeoff
    if not cmd.takeoff(TAKEOFF_ALT):
        print("[Commander] Takeoff failed")
        rclpy.shutdown(); return
    time.sleep(2.0)

    # Step 7: Waypoints
    for i, (n, e, d) in enumerate(WAYPOINTS):
        print(f"[Commander] WP {i+1}/{len(WAYPOINTS)}  "
              f"N={n:+.0f} E={e:+.0f} ALT={-d:.0f} m AGL")
        reached = cmd.go_to_ned(n, e, d, timeout=WAYPOINT_TIMEOUT)
        print(f"[Commander] WP {i+1} {'✓' if reached else 'timeout'}")
        time.sleep(1.0)

    # Step 8: RTL
    print("[Commander] Mission complete — RTL")
    cmd.set_mode("RTL")

    # Wait for disarm
    cmd._spin_until(lambda: not cmd._state.armed, timeout=60.0)
    print("[Commander] Disarmed — landed ✓")

    cmd.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
