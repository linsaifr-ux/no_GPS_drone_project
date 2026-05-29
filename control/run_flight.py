#!/usr/bin/env python3
"""
6b-iv: Autonomous flight + vision position bridge in one process.

Vision sending (VISION_POSITION_ESTIMATE at 5 Hz) runs in a background
thread sharing the same MAVLinkCtrl — no second TCP connection needed.
This replaces running run_vision.py separately.

If anyloc/latest_estimate.json does not exist a stub estimate at the home
position is written automatically so EKF3 gets a position fix without the
full AnyLoc pipeline.

Usage:
  # Terminal 1 — SITL (no --out tcp:localhost:5763 needed)
  python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
      -v ArduCopter --model=JSON --no-rebuild --console --map \
      -l 23.450868,120.286135,46,0 \
      --add-param-file=control/no_gps.parm --wipe

  # Terminal 2 — bridge
  python3 control/stub_bridge.py   # or: cd simulator && ./run_chiayi.sh

  # Terminal 3 (optional) — AnyLoc localizer
  DISPLAY=:2 conda run -n isaac_sim_test python anyloc/run_localizer.py

  # Terminal 4 — this script (vision + flight)
  python3 control/run_flight.py
"""

import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control.mavlink_ctrl import MAVLinkCtrl
from pymavlink import mavutil

# ── Home position (must match SITL -l flag) ────────────────────────────────────
HOME_LAT     = 23.450868
HOME_LON     = 120.286135
HOME_ALT_MSL = 46.0
COS_LAT      = math.cos(math.radians(HOME_LAT))
M_PER_DEG    = 111_320.0

ESTIMATE_JSON = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "anyloc", "latest_estimate.json",
)

# ── Mission parameters ──────────────────────────────────────────────────────────
TAKEOFF_ALT      = 10.0   # metres AGL
WAYPOINT_RADIUS  = 3.0    # metres — how close counts as "reached"
WAYPOINT_TIMEOUT = 60.0   # seconds per waypoint before giving up

# NED waypoints (north m, east m, down m) — down = -alt_agl
WAYPOINTS = [
    ( 20.0,   0.0, -10.0),
    ( 20.0,  20.0, -10.0),
    (  0.0,  20.0, -10.0),
    (  0.0,   0.0, -10.0),
]

VISION_HZ = 5   # VISION_POSITION_ESTIMATE send rate


# ── Helpers ────────────────────────────────────────────────────────────────────

def _lat_lon_to_ned(lat: float, lon: float, alt_msl: float):
    north = (lat - HOME_LAT) * M_PER_DEG
    east  = (lon - HOME_LON) * M_PER_DEG * COS_LAT
    down  = -(alt_msl - HOME_ALT_MSL)
    return north, east, down


def _write_stub_estimate() -> None:
    est = {
        "timestamp": time.time(),
        "est_lat":   HOME_LAT,
        "est_lon":   HOME_LON,
        "alt_msl_m": HOME_ALT_MSL + 5.0,
        "agl_m":     5.0,
        "yaw_deg":   0.0,
        "score":     0.9,
        "error_m":   0.0,
    }
    tmp = ESTIMATE_JSON + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(est, fh)
    os.replace(tmp, ESTIMATE_JSON)
    print(f"[Flight] Stub estimate written to {ESTIMATE_JSON}")


def _vision_loop(ctrl: MAVLinkCtrl, stop: threading.Event) -> None:
    """
    Background thread — reads latest_estimate.json and sends
    VISION_POSITION_ESTIMATE at VISION_HZ to keep EKF3 fusion alive.
    Shares ctrl with the main thread; send and recv use independent
    socket directions so no lock is needed.
    """
    last_mtime   = 0.0
    current_est  = None
    send_interval = 1.0 / VISION_HZ
    n_sent = 0

    while not stop.is_set():
        t0 = time.time()

        try:
            mtime = os.path.getmtime(ESTIMATE_JSON)
            if mtime != last_mtime:
                with open(ESTIMATE_JSON) as fh:
                    est = json.load(fh)
                north, east, down = _lat_lon_to_ned(
                    est["est_lat"], est["est_lon"], est["alt_msl_m"])
                yaw_rad     = math.radians(est.get("yaw_deg", 0.0))
                current_est = (north, east, down, yaw_rad)
                last_mtime  = mtime
        except (FileNotFoundError, KeyError, json.JSONDecodeError):
            pass

        if current_est is not None:
            ctrl.send_vision_position(*current_est)
            n_sent += 1
            if n_sent == 1:
                print("[Vision] First VISION_POSITION_ESTIMATE sent")

        elapsed = time.time() - t0
        time.sleep(max(0.0, send_interval - elapsed))


# ── Main flight sequence ────────────────────────────────────────────────────────

def main():
    ctrl = MAVLinkCtrl()   # tcp:localhost:5762

    # 1 — Connect
    if not ctrl.wait_heartbeat(timeout=60.0):
        print("[Flight] No HEARTBEAT — is SITL running?")
        return

    # 2 — Ensure estimate file exists, then start vision thread
    if not os.path.exists(ESTIMATE_JSON):
        print(f"[Flight] {ESTIMATE_JSON} not found — writing stub at home position")
        _write_stub_estimate()

    stop_ev = threading.Event()
    vt = threading.Thread(target=_vision_loop, args=(ctrl, stop_ev), daemon=True)
    vt.start()
    print(f"[Flight] Vision thread started — sending at {VISION_HZ} Hz")

    # 3 — Wait for EKF POS_ABS
    print("[Flight] Waiting for EKF POS_ABS …")
    if not ctrl.wait_ekf_pos(timeout=60.0):
        print("[Flight] EKF never reached POS_ABS — aborting")
        stop_ev.set(); ctrl.close(); return
    print("[Flight] EKF POS_ABS ✓")

    # 4 — GUIDED mode
    ctrl.set_mode("GUIDED")
    time.sleep(1.0)
    ctrl.recv()

    # 5 — Arm
    print("[Flight] Arming …")
    ctrl.arm()
    result = ctrl.wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=10.0)

    if result is None:
        print("[Flight] No ARM ACK — retrying with force …")
        ctrl.arm(force=True)
        result = ctrl.wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=10.0)

    if result != 0:
        names = {0:"ACCEPTED",1:"TEMPORARILY_REJECTED",2:"DENIED",3:"UNSUPPORTED",4:"FAILED"}
        print(f"[Flight] ARM rejected: {names.get(result, result)}")
        stop_ev.set(); ctrl.close(); return
    print("[Flight] Armed ✓")

    # 6 — Takeoff
    print(f"[Flight] Takeoff → {TAKEOFF_ALT} m AGL …")
    ctrl.takeoff(TAKEOFF_ALT)
    if ctrl.wait_altitude(TAKEOFF_ALT, tolerance=1.5, timeout=30.0):
        print(f"[Flight] Reached {TAKEOFF_ALT} m AGL ✓")
    else:
        print(f"[Flight] Altitude timeout — continuing")
    time.sleep(2.0)

    # 7 — Waypoints
    for i, (n, e, d) in enumerate(WAYPOINTS):
        print(f"[Flight] WP {i+1}/{len(WAYPOINTS)}  N={n:+.0f} E={e:+.0f} ALT={-d:.0f} m AGL")
        ctrl.set_position_ned(n, e, d)
        if ctrl.wait_position(n, e, d, radius=WAYPOINT_RADIUS, timeout=WAYPOINT_TIMEOUT):
            print(f"[Flight] WP {i+1} ✓")
        else:
            print(f"[Flight] WP {i+1} timeout")
        time.sleep(1.0)

    # 8 — RTL
    print("[Flight] Mission complete — RTL")
    ctrl.set_mode("RTL")

    deadline = time.time() + 60.0
    while time.time() < deadline:
        ctrl.recv()
        if not ctrl.is_armed:
            print("[Flight] Disarmed — landed ✓")
            break
        time.sleep(0.2)
    else:
        print("[Flight] Disarm timeout")

    stop_ev.set()
    ctrl.close()
    print("[Flight] Done.")


if __name__ == "__main__":
    main()
