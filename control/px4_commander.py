#!/usr/bin/env python3
"""
PX4 flight commander (MAVROS) — external-vision no-GPS.

PX4 counterpart of flight_commander.py.  Differences from the ArduPilot version:
  - injects vision via /mavros/vision_pose/pose (+ /mavros/vision_speed) → PX4 EKF2
    (EKF2_EV_CTRL); PX4 auto-sets its local origin from EV, no set_gp_origin.
  - control is OFFBOARD mode (not GUIDED): stream setpoints ≥2 Hz BEFORE switching.
  - no STABILIZE/force-arm dance.

Default run = the Phase-3 position-hold gate: inject vision, take off to HOLD_AGL via
OFFBOARD, hold, and log drift + EKF-vs-truth.  This is the make-or-break test that
PX4's position loop holds where ArduPilot's AC_PosControl inverted.

Run:
  source /opt/ros/jazzy/setup.bash
  python3 control/px4_commander.py
"""
import math, os, sys, threading, time

_ROS2_SITE = "/opt/ros/jazzy/lib/python3.12/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode

_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

HOME_ALT_MSL = 28.17
HOLD_AGL     = 3.0      # m — gate test hover altitude (low = safe, enough to leave ground)


class PX4Commander(rclpy.node.Node):
    def __init__(self):
        super().__init__("px4_commander")
        self._state      = State()
        self._local_pos  = None
        self._drone      = None       # /drone/state truth (ENU)

        self.create_subscription(State, "/mavros/state", self._cb_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_local, _SENSOR_QOS)
        self.create_subscription(PoseStamped, "/drone/state", self._cb_drone, _SENSOR_QOS)

        self._vpe_pub  = self.create_publisher(PoseStamped, "/mavros/vision_pose/pose", 1)
        self._vspd_pub = self.create_publisher(TwistStamped, "/mavros/vision_speed/speed_twist", 1)
        self._sp_pub   = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 1)

        self._arm_cli  = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_cli = self.create_client(SetMode,     "/mavros/set_mode")
        self.get_logger().info("PX4 commander ready")

    def _cb_state(self, m): self._state = m
    def _cb_local(self, m): self._local_pos = m
    def _cb_drone(self, m): self._drone = m

    # ── Vision injection thread (truth → /mavros/vision_pose + vision_speed) ──
    def start_vision(self, stop):
        def loop():
            last = None
            n = 0
            while not stop.is_set():
                if self._drone is not None:
                    p = self._drone.pose.position
                    q = self._drone.pose.orientation
                    pm = PoseStamped()
                    pm.header.stamp = self.get_clock().now().to_msg()
                    pm.header.frame_id = "map"          # ENU; MAVROS → NED
                    pm.pose.position.x = p.x            # East
                    pm.pose.position.y = p.y            # North
                    pm.pose.position.z = p.z - HOME_ALT_MSL   # Up (AGL)
                    pm.pose.orientation = q
                    self._vpe_pub.publish(pm)
                    now = time.time()
                    if last is not None:
                        dt = now - last[2]
                        if dt > 1e-3:
                            tw = TwistStamped()
                            tw.header.stamp = pm.header.stamp
                            tw.header.frame_id = "map"
                            tw.twist.linear.x = (p.x - last[0]) / dt
                            tw.twist.linear.y = (p.y - last[1]) / dt
                            self._vspd_pub.publish(tw)
                    last = (p.x, p.y, now)
                    n += 1
                    if n == 1:
                        print("[PX4Cmd] vision injection started")
                time.sleep(0.02)   # 50 Hz
        t = threading.Thread(target=loop, daemon=True); t.start(); return t

    def _spin_until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    def set_mode(self, mode, timeout=5.0):
        req = SetMode.Request(); req.custom_mode = mode
        fut = self._mode_cli.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        ok = fut.done() and fut.result().mode_sent
        self.get_logger().info(f"set_mode {mode}: {'ok' if ok else 'FAIL'}")
        return ok

    def arm(self, value=True, timeout=5.0):
        req = CommandBool.Request(); req.value = value
        fut = self._arm_cli.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        ok = fut.done() and fut.result().success
        self.get_logger().info(f"{'arm' if value else 'disarm'}: {'ok' if ok else 'FAIL'}")
        return ok

    def make_sp(self, east, north, up):
        sp = PositionTarget()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                        PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                        PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                        PositionTarget.IGNORE_YAW_RATE)
        sp.position.x = float(east)    # ENU East → MAVROS → NED
        sp.position.y = float(north)   # ENU North
        sp.position.z = float(up)      # ENU Up
        sp.yaw = 0.0
        return sp


def main():
    rclpy.init()
    cmd = PX4Commander()
    stop = threading.Event()
    cmd.start_vision(stop)

    print("[PX4Cmd] waiting for MAVROS connection …")
    if not cmd._spin_until(lambda: cmd._state.connected, 60.0):
        print("[PX4Cmd] MAVROS not connected — is PX4 + MAVROS running?"); return
    print("[PX4Cmd] MAVROS connected ✓")

    # Phase 2: wait for EKF local position to become valid (vision fused)
    print("[PX4Cmd] waiting for EKF local position (vision fusion) …")
    ok = cmd._spin_until(lambda: cmd._local_pos is not None and cmd._drone is not None, 30.0)
    if ok:
        time.sleep(2.0)
        for _ in range(20): rclpy.spin_once(cmd, timeout_sec=0.05)
        lp = cmd._local_pos.pose.position; ds = cmd._drone.pose.position
        err = math.hypot(lp.x - ds.x, lp.y - (ds.y))
        print(f"[PX4Cmd] EKF-vs-truth: EKF=({lp.x:+.1f},{lp.y:+.1f}) "
              f"truth=({ds.x:+.1f},{ds.y:+.1f}) err={err:.2f} m")

    # Stream OFFBOARD setpoint (hold over takeoff point at HOLD_AGL) before switching
    e0 = cmd._drone.pose.position.x if cmd._drone else 0.0
    n0 = cmd._drone.pose.position.y if cmd._drone else 0.0
    sp = cmd.make_sp(e0, n0, HOLD_AGL)
    print(f"[PX4Cmd] streaming OFFBOARD setpoint (E={e0:+.1f} N={n0:+.1f} AGL={HOLD_AGL}) …")
    for _ in range(40):
        sp.header.stamp = cmd.get_clock().now().to_msg()
        cmd._sp_pub.publish(sp)
        rclpy.spin_once(cmd, timeout_sec=0.05)

    cmd.set_mode("OFFBOARD")
    cmd.arm(True)

    # Hold + log drift for 40 s (the position-hold gate)
    print("[PX4Cmd] === HOLD GATE: hold 40 s, logging drift ===")
    t_end = time.time() + 40.0; t_log = 0.0
    while time.time() < t_end:
        sp.header.stamp = cmd.get_clock().now().to_msg()
        cmd._sp_pub.publish(sp)
        rclpy.spin_once(cmd, timeout_sec=0.02)
        if time.time() - t_log > 3.0 and cmd._drone is not None:
            t_log = time.time()
            ds = cmd._drone.pose.position
            agl = ds.z - HOME_ALT_MSL
            print(f"[PX4Cmd] HOLD drift E={ds.x-e0:+6.1f} N={ds.y-n0:+6.1f} "
                  f"AGL={agl:4.1f}  dist={math.hypot(ds.x-e0,ds.y-n0):5.1f} m  "
                  f"mode={cmd._state.mode} armed={cmd._state.armed}")
    print("[PX4Cmd] === gate done ===")
    stop.set(); cmd.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
