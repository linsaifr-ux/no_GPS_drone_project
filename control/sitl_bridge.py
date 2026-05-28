#!/usr/bin/env python3
"""
ArduPilot SITL JSON bridge.

Translates Isaac Sim drone state (ENU) into ArduPilot's JSON SITL format (NED).

Protocol (ArduPilot is the CLIENT, this bridge is the SERVER):
  1. ArduPilot SITL sends binary servo/PWM packet → bridge port 9002  (our server)
  2. Bridge receives servo data, replies  →  ArduPilot's port  (our response)
  3. ArduPilot receives physics state and advances its EKF

  Port 9002  — bridge listens here; ArduPilot connects and sends servo data
  Port 14550 — SITL MAVLink output (for mavlink_ctrl.py / ground station)

Start SITL before running the sim (from project root):
  python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
      -v ArduCopter --model=JSON --no-rebuild --console --map \
      -l 23.450868,120.286135,46,0

Coordinate conventions:
  Isaac Sim  (ENU):  X = East,  Y = North, Z = Up
  ArduPilot  (NED):  X = North, Y = East,  Z = Down

Yaw convention:
  Isaac Sim RotateZ: positive = CCW from above (math / right-hand)
  ArduPilot yaw:     positive = CW from above  (compass / NED)
  Conversion:  ardupilot_yaw_rad = -radians(isaacsim_yaw_deg)

Usage — embed in cesium_scene.py simulation loop:
  bridge = SITLBridge(centre_elev=centre_elev)
  # each step, after drone position is updated:
  servos = bridge.step(float(_p[0]), float(_p[1]), _alt, float(drone_yaw_op.Get()))
"""

import json
import math
import socket
import struct
import time

# ArduPilot SIM_JSON binary servo packet formats (little-endian)
# struct servo_packet_16 { uint16_t magic; uint16_t frame_rate; uint32_t frame_count; uint16_t pwm[16]; }
# struct servo_packet_32 { uint16_t magic; uint16_t frame_rate; uint32_t frame_count; uint16_t pwm[32]; }
_SERVO16_MAGIC = 18458
_SERVO32_MAGIC = 29569
_SERVO16_FMT   = "<HHI16H"
_SERVO32_FMT   = "<HHI32H"
_SERVO16_SIZE  = struct.calcsize(_SERVO16_FMT)   # 40 bytes
_SERVO32_SIZE  = struct.calcsize(_SERVO32_FMT)   # 72 bytes

_GRAVITY   = 9.81    # m/s²
_MAX_SPEED = 30.0    # m/s — clamp computed velocity to avoid keyboard-jump spikes


class SITLBridge:
    """
    UDP server on port 9002.

    Each sim step:
      - drains all pending servo packets from ArduPilot (non-blocking)
      - builds current physics state JSON
      - replies to ArduPilot's address with that state

    ArduPilot prints "No JSON sensor message received, resending servos" until
    the first reply arrives — this is normal while Isaac Sim is starting up.
    """

    LISTEN_PORT = 9002   # bridge binds here; ArduPilot sends servo data here

    def __init__(self, listen_port: int = 9002, centre_elev: float = 0.0):
        """
        listen_port  : UDP port the bridge binds to (must match ArduPilot's sim-address port)
        centre_elev  : terrain elevation (m MSL) at scene origin
        """
        self._centre_elev = centre_elev
        self._ap_addr     = None     # ArduPilot's address, learned from first servo packet
        self._connected   = False

        # Server socket — listens for ArduPilot servo data, replies with physics
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self._sock.bind(("0.0.0.0", listen_port))

        # Derivative state
        self._prev_pos_ned   = None
        self._prev_vel_ned   = None
        self._prev_accel_ned = None
        self._prev_yaw_rad   = None
        self._prev_t         = None

        self._start_t  = time.time()
        self._n_sent   = 0
        self._last_pwm = None   # most recent servo PWM from ArduPilot (for 6b)

        print(f"[SITL] Bridge listening on UDP port {listen_port}  "
              f"(centre_elev={centre_elev:.1f} m MSL)")
        print("[SITL] Waiting for ArduPilot servo packets …")

    # ── Public API ─────────────────────────────────────────────────────────────

    def step(self, x_enu: float, y_enu: float, z_abs: float,
             yaw_deg: float, wall_time: float | None = None) -> dict | None:
        """
        Called each sim step after drone position is updated.

        Drains any pending servo packets from ArduPilot, then sends current
        physics state back. Returns the latest servo dict (keys: 'pwm' list,
        'frame_time_us') or None if no packet arrived this step.

        x_enu, y_enu : ENU metres from scene centre (East, North)
        z_abs        : absolute altitude in metres MSL
        yaw_deg      : Isaac Sim RotateZ degrees (CCW-positive)
        """
        t = wall_time if wall_time is not None else time.time()

        # ── Drain incoming servo packets (non-blocking) ────────────────────
        # ArduPilot sends BINARY servo_packet_16 / servo_packet_32 (not JSON).
        latest_servos = None
        while True:
            try:
                data, addr = self._sock.recvfrom(4096)
                parsed = self._parse_servo_packet(data, addr)
                if parsed is not None:
                    latest_servos = parsed
            except (BlockingIOError, OSError):
                break

        if latest_servos is not None and not self._connected:
            print(f"[SITL] ArduPilot connected from {self._ap_addr}")
            self._connected = True

        # ── Build physics state ────────────────────────────────────────────
        state = self._build_state(x_enu, y_enu, z_abs, yaw_deg, t)

        # ── Reply to ArduPilot ─────────────────────────────────────────────
        if self._ap_addr is not None:
            try:
                # ArduPilot's recv_fdm splits messages on '\n' → '\0'; trailing newline is required.
                self._sock.sendto((json.dumps(state) + "\n").encode(), self._ap_addr)
                self._n_sent += 1
                if self._n_sent == 1:
                    print(f"[SITL] First physics reply sent to {self._ap_addr}")
                elif self._n_sent % 500 == 0:
                    print(f"[SITL] {self._n_sent} physics packets sent")
            except OSError as e:
                print(f"[SITL] UDP send error: {e}")

        # ── Save state for next step ───────────────────────────────────────
        self._prev_t = t

        return latest_servos

    @property
    def last_pwm(self) -> list | None:
        """Most recent PWM values from ArduPilot (16 channels, 1000–2000 µs)."""
        return self._last_pwm

    @property
    def connected(self) -> bool:
        return self._connected

    def close(self) -> None:
        self._sock.close()

    # ── Internal ───────────────────────────────────────────────────────────────

    def _parse_servo_packet(self, data: bytes, addr) -> dict | None:
        """
        Parse ArduPilot's binary servo_packet_16 / servo_packet_32.
        Returns dict with 'pwm', 'frame_rate', 'frame_count' on success,
        or None if the data doesn't match either format.
        Sets _ap_addr only when a valid magic is confirmed.
        """
        n = len(data)
        if n == _SERVO16_SIZE:
            fields = struct.unpack(_SERVO16_FMT, data)
            if fields[0] == _SERVO16_MAGIC:
                self._ap_addr = addr
                pwm = list(fields[3:])
                self._last_pwm = pwm
                return {"pwm": pwm, "frame_rate": fields[1], "frame_count": fields[2]}
        elif n == _SERVO32_SIZE:
            fields = struct.unpack(_SERVO32_FMT, data)
            if fields[0] == _SERVO32_MAGIC:
                self._ap_addr = addr
                pwm = list(fields[3:])
                self._last_pwm = pwm
                return {"pwm": pwm, "frame_rate": fields[1], "frame_count": fields[2]}
        # Unrecognised — log once
        if not getattr(self, '_logged_unknown', False):
            print(f"[SITL] Unknown packet from {addr} ({n} bytes): {data[:16].hex()}")
            self._logged_unknown = True
        return None

    def _build_state(self, x_enu: float, y_enu: float, z_abs: float,
                     yaw_deg: float, t: float) -> dict:
        # ── ENU → NED ──────────────────────────────────────────────────────
        north = y_enu
        east  = x_enu
        agl   = z_abs - self._centre_elev
        down  = -agl

        pos_ned = (north, east, down)

        # ── Velocity NED (m/s) — finite difference, clamped ───────────────
        if self._prev_pos_ned is not None and self._prev_t is not None:
            dt = t - self._prev_t
            if dt > 1e-6:
                raw_vel = tuple((pos_ned[i] - self._prev_pos_ned[i]) / dt
                                for i in range(3))
                vel_ned = tuple(max(-_MAX_SPEED, min(_MAX_SPEED, v))
                                for v in raw_vel)
            else:
                vel_ned = self._prev_vel_ned or (0.0, 0.0, 0.0)
        else:
            vel_ned = (0.0, 0.0, 0.0)

        # ── Acceleration NED (m/s²) — EMA low-pass (α=0.3) ────────────────
        if self._prev_vel_ned is not None and self._prev_t is not None:
            dt = t - self._prev_t
            if dt > 1e-6:
                raw_a = tuple((vel_ned[i] - self._prev_vel_ned[i]) / dt
                              for i in range(3))
                if self._prev_accel_ned is not None:
                    alpha = 0.3
                    accel_ned = tuple(alpha * raw_a[i] + (1 - alpha) * self._prev_accel_ned[i]
                                      for i in range(3))
                else:
                    accel_ned = raw_a
            else:
                accel_ned = self._prev_accel_ned or (0.0, 0.0, 0.0)
        else:
            accel_ned = (0.0, 0.0, 0.0)

        # ── Attitude & yaw rate ────────────────────────────────────────────
        yaw_rad = -math.radians(yaw_deg)   # CCW→CW, Isaac Sim→NED
        yaw_rate = 0.0
        if self._prev_yaw_rad is not None and self._prev_t is not None:
            dt = t - self._prev_t
            if dt > 1e-6:
                dyaw = (yaw_rad - self._prev_yaw_rad + math.pi) % (2 * math.pi) - math.pi
                yaw_rate = dyaw / dt

        # ── IMU specific force in body frame ──────────────────────────────
        # sf_ned = accel_ned - gravity_ned;  gravity_ned = (0, 0, +g)
        sf_ned = (accel_ned[0], accel_ned[1], accel_ned[2] - _GRAVITY)
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        sf_bx  =  sf_ned[0] * cy + sf_ned[1] * sy
        sf_by  = -sf_ned[0] * sy + sf_ned[1] * cy
        sf_bz  =  sf_ned[2]

        # ── Save running state ─────────────────────────────────────────────
        self._prev_pos_ned   = pos_ned
        self._prev_vel_ned   = vel_ned
        self._prev_accel_ned = accel_ned
        self._prev_yaw_rad   = yaw_rad

        # 6b-ii: "position" and "velocity" are intentionally omitted.
        # Sending them would let ArduPilot EKF3 use them as a GPS substitute.
        # Without them the EKF runs on IMU + baro + compass + rangefinder only.
        # (vel_ned / accel_ned are still computed above because accel_body needs them.)
        return {
            "timestamp": t - self._start_t,
            "imu": {
                "gyro":       [0.0, 0.0, yaw_rate],
                "accel_body": [sf_bx, sf_by, sf_bz],
            },
            "attitude":  [0.0, 0.0, yaw_rad],
            "rng_1":     max(0.1, agl),
            "battery":   {"voltage": 12.6, "current": 5.0},
        }
