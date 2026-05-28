#!/usr/bin/env python3
"""
Minimal SITL bridge stub — keeps ArduPilot SITL alive without Isaac Sim.

Use this when you want to test MAVLink (run_mavlink.py, mavlink_ctrl.py)
without starting the full Isaac Sim scene. It replies to SITL's servo
packets with a static hover state at the home position.

The real bridge (control/sitl_bridge.py) runs inside Isaac Sim via
cesium_scene.py. Do NOT run both at the same time — they both bind UDP:9002.

Usage:
  # Terminal 1 — SITL
  python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
      -v ArduCopter --model=JSON --no-rebuild --console --map \
      --home=23.450868,120.286135,46,0

  # Terminal 2 — stub bridge (keeps SITL loop alive)
  python3 control/stub_bridge.py

  # Terminal 3 — MAVLink monitor
  python3 control/run_mavlink.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control.sitl_bridge import SITLBridge

_HOME_ELEV = 46.0   # metres MSL — matches --home=23.450868,120.286135,46,0
_HOVER_AGL = 5.0    # metres above ground for "parked" physics state

try:
    bridge = SITLBridge(centre_elev=_HOME_ELEV)
except OSError as e:
    if e.errno == 98:  # EADDRINUSE
        import subprocess
        try:
            who = subprocess.check_output(
                ["fuser", "9002/udp"], stderr=subprocess.DEVNULL
            ).decode().strip()
        except Exception:
            who = "unknown"
        print(f"[stub] ERROR: UDP port 9002 is already in use (pid {who}).")
        print("[stub] Stop Isaac Sim (cesium_scene.py) first, then re-run stub_bridge.py.")
        sys.exit(1)
    raise

print("[stub] Sending static hover state to SITL at 100 Hz. Ctrl-C to stop.")
print("[stub] Start run_mavlink.py in another terminal to verify connection.")

try:
    while True:
        # x_enu=0, y_enu=0 (scene origin), z_abs=home+5m, yaw=0
        bridge.step(0.0, 0.0, _HOME_ELEV + _HOVER_AGL, 0.0)
        time.sleep(0.01)   # 100 Hz — matches SITL's expected physics rate
except KeyboardInterrupt:
    print("\n[stub] Stopped.")
finally:
    bridge.close()
