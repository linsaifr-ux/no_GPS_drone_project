#!/bin/bash
# Launch MAVROS2 connected to ArduPilot SITL on tcp:localhost:5762.
#
# Port allocation:
#   TCP 5760 — mavproxy / MAVProxy console (used by sim_vehicle.py internally)
#   TCP 5762 — MAVROS2 (this script)
#   UDP 14550 — pymavlink in flight_commander.py (EKF origin setup only)
#
# MAVROS2 uses TCP 5762 (single-client). flight_commander.py uses UDP 14550
# for the one-shot SET_GPS_GLOBAL_ORIGIN call so both can coexist.
#
# Prerequisites (one-time):
#   sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs
#   sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
#
# Run order:
#   Terminal 1: SITL         (sim_vehicle.py ... --add-param-file=control/no_gps.parm)
#   Terminal 2: stub_bridge  (python3 control/stub_bridge.py)  ← or Isaac Sim
#   Terminal 3: MAVROS2      (this script)
#   Terminal 4: AnyLoc node  (./anyloc/run_ros2_localizer.sh)
#   Terminal 5: Flight cmd   (python3 control/flight_commander.py)

source /opt/ros/jazzy/setup.bash

ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:="tcp://localhost:5762" \
    -p tgt_system:=1 \
    -p tgt_component:=1 \
    -p log_output:="screen" \
    -p fcu_protocol:="v2.0"
