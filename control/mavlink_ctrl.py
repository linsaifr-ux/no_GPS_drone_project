#!/usr/bin/env python3
"""
ArduPilot MAVLink controller.

6b-i   (this file):  pymavlink connection, non-blocking receive loop,
                     HEARTBEAT / HIGHRES_IMU / ATTITUDE /
                     LOCAL_POSITION_NED / EKF_STATUS_REPORT state.
6b-iii (TODO):       send_vision_position() — AnyLoc → ArduPilot EKF3
6b-iv  (TODO):       arm(), takeoff(), set_position_ned() — flight commands

Connection string examples
  "udp:0.0.0.0:14550"   listen for mavproxy output (default, SITL GCS port)
  "tcp:localhost:5760"   direct SITL TCP output (alternative, no mavproxy)

Usage:
  ctrl = MAVLinkCtrl()
  ctrl.wait_heartbeat()          # blocks until ArduPilot sends first heartbeat
  while True:
      ctrl.recv()                # non-blocking drain
      print(ctrl.ekf_flags, ctrl.imu)
"""

import time

from pymavlink import mavutil

# ── EKF_STATUS_REPORT flag bits (ArduPilot EKF3) ──────────────────────────────
EKF_ATTITUDE           = 1 << 0
EKF_VEL_HORIZ          = 1 << 1
EKF_VEL_VERT           = 1 << 2
EKF_POS_HORIZ_REL      = 1 << 3
EKF_POS_HORIZ_ABS      = 1 << 4   # GPS or vision absolute position fused
EKF_POS_VERT_ABS       = 1 << 5
EKF_POS_VERT_AGL       = 1 << 6
EKF_CONST_POS_MODE     = 1 << 7
EKF_PRED_POS_HORIZ_REL = 1 << 8
EKF_PRED_POS_HORIZ_ABS = 1 << 9   # vision estimate accepted by EKF3
EKF_UNINITIALIZED      = 1 << 10  # EKF has not finished initialising


class MAVLinkCtrl:
    """
    pymavlink wrapper for no-GPS drone control.

    Maintains latest state from the MAVLink stream. Non-blocking: call
    recv() each loop iteration to drain pending messages without stalling
    the main sim or localisation loop.
    """

    def __init__(self, connection_str: str = "tcp:localhost:5762",
                 source_system: int = 255):
        """
        connection_str : pymavlink connection string
            "tcp:localhost:5762"  — direct SITL TCP port 2 (default; no mavproxy needed)
            "tcp:localhost:5760"  — SITL TCP port 1 (mavproxy uses this; avoid conflict)
            "tcp:localhost:5763"  — SITL TCP port 3 (spare)
        source_system  : GCS MAVLink system ID (must not clash with vehicle = 1)
        """
        self._mav = mavutil.mavlink_connection(
            connection_str,
            source_system=source_system,
            dialect="ardupilotmega",
        )
        self._connected = False

        # Latest received messages (None until first arrival)
        self._heartbeat = None
        self._imu       = None   # HIGHRES_IMU
        self._attitude  = None   # ATTITUDE
        self._local_pos = None   # LOCAL_POSITION_NED
        self._ekf       = None   # EKF_STATUS_REPORT

        print(f"[MAVLink] Connecting to {connection_str} …")

    # ── Public API ─────────────────────────────────────────────────────────────

    def wait_heartbeat(self, timeout: float = 60.0) -> bool:
        """
        Block until the first HEARTBEAT arrives or timeout expires.
        Returns True on success, False on timeout.
        """
        print("[MAVLink] Waiting for HEARTBEAT …")
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._mav.recv_match(type="HEARTBEAT", blocking=True,
                                       timeout=1.0)
            if msg is not None:
                self._heartbeat = msg
                self._connected = True
                print(f"[MAVLink] HEARTBEAT  sysid={self._mav.target_system} "
                      f"compid={self._mav.target_component} "
                      f"type={msg.type} autopilot={msg.autopilot}")
                self._request_streams()
                return True
        print("[MAVLink] Timeout waiting for HEARTBEAT")
        return False

    def recv(self) -> list:
        """
        Non-blocking drain of all pending MAVLink messages.
        Updates internal state; returns list of type-name strings received.
        """
        received = []
        while True:
            msg = self._mav.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if   t == "HEARTBEAT":          self._heartbeat = msg; self._connected = True
            elif t == "HIGHRES_IMU":        self._imu       = msg
            elif t == "ATTITUDE":           self._attitude  = msg
            elif t == "LOCAL_POSITION_NED": self._local_pos = msg
            elif t == "EKF_STATUS_REPORT":  self._ekf       = msg
            received.append(t)
        return received

    # ── State properties ───────────────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def imu(self):
        """Latest HIGHRES_IMU message, or None."""
        return self._imu

    @property
    def attitude(self):
        """Latest ATTITUDE message, or None."""
        return self._attitude

    @property
    def local_pos(self):
        """Latest LOCAL_POSITION_NED message, or None."""
        return self._local_pos

    @property
    def ekf_flags(self) -> int:
        """Latest EKF_STATUS_REPORT flags bitmask (0 until first message)."""
        return self._ekf.flags if self._ekf is not None else 0

    @property
    def ekf_pos_valid(self) -> bool:
        """True when EKF3 has accepted an absolute position source (GPS or vision)."""
        return bool(self.ekf_flags & EKF_POS_HORIZ_ABS)

    # ── 6b-iii: Vision position ────────────────────────────────────────────────

    def send_vision_position(self, north: float, east: float, down: float,
                             yaw_rad: float,
                             covariance: list | None = None) -> None:
        """
        Send VISION_POSITION_ESTIMATE to ArduPilot EKF3.

        north, east, down : NED position in metres from the EKF origin
        yaw_rad           : heading (rad, NED / compass convention, CW-positive)
        covariance        : 21-element upper-triangle of 6×6 pose covariance.
                            Default: 5 m position std, 0.2 rad orientation std.
                            Tune once AnyLoc error is characterised.

        TODO (6b-iii): call this from the AnyLoc anchor callback in run_localizer.py
        """
        if covariance is None:
            pv = 5.0 ** 2    # 5 m position std → 25 m² variance
            ov = 0.2 ** 2    # 0.2 rad (~11°) orientation std
            covariance = [
                pv, 0,  0,  0,  0,  0,
                    pv, 0,  0,  0,  0,
                        pv, 0,  0,  0,
                            ov, 0,  0,
                                ov, 0,
                                    ov,
            ]
        usec = int(time.time() * 1e6)
        self._mav.mav.vision_position_estimate_send(
            usec,
            north, east, down,
            0.0, 0.0, yaw_rad,
            covariance,
        )

    # ── 6b-iv: Flight commands ─────────────────────────────────────────────────

    def arm(self, force: bool = False) -> None:
        """
        Send arm command.
        TODO (6b-iv): call only after ekf_pos_valid is True.
        """
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,                  # param1: 1=arm
            21196 if force else 0,  # param2: force-arm magic (bypasses pre-arm checks)
            0, 0, 0, 0, 0,
        )
        print("[MAVLink] ARM command sent")

    def takeoff(self, alt_m: float) -> None:
        """
        Command takeoff to alt_m AGL.
        TODO (6b-iv): call after arm() ACK received.
        """
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt_m,
        )
        print(f"[MAVLink] TAKEOFF {alt_m:.1f} m command sent")

    def set_position_ned(self, north: float, east: float, down: float,
                         yaw_rad: float | None = None) -> None:
        """
        Fly to NED position in metres from EKF origin.
        Optionally sets heading; velocity + acceleration fields are ignored.
        TODO (6b-iv): replaces keyboard control once EKF position is valid.
        """
        # type_mask: bit=1 → ignore that field
        # ignore velocity (bits 3-5), acceleration (bits 6-8), yaw_rate (bit 11)
        # keep position (bits 0-2); optionally keep yaw (bit 10)
        type_mask = 0b111111111000  # ignore vel + accel + yaw + yaw_rate
        if yaw_rad is not None:
            type_mask &= ~(1 << 10)  # clear "ignore yaw" → enable yaw

        self._mav.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (0 = use current time)
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            north, east, down,
            0, 0, 0,                # velocity (ignored)
            0, 0, 0,                # acceleration (ignored)
            yaw_rad if yaw_rad is not None else 0.0,
            0,                      # yaw_rate (ignored)
        )

    def close(self) -> None:
        self._mav.close()

    # ── Internal ───────────────────────────────────────────────────────────────

    def _request_streams(self) -> None:
        """Ask ArduPilot to send data streams at 10 Hz, HIGHRES_IMU at 50 Hz."""
        # Broad request — covers ATTITUDE, LOCAL_POSITION_NED, EKF_STATUS_REPORT
        self._mav.mav.request_data_stream_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  # 10 Hz
            1,   # start
        )
        # HIGHRES_IMU is not included in any standard stream group — request explicitly
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
            20000,  # 20 000 µs = 50 Hz
            0, 0, 0, 0, 0,
        )
        print("[MAVLink] Requested all streams at 10 Hz, HIGHRES_IMU at 50 Hz")
