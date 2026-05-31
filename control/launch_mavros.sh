#!/bin/bash
# Launch MAVROS2 connected to ArduPilot SITL via UDP.
#
# Port allocation:
#   TCP 5760   — MAVProxy master (SITL internal; sim_vehicle.py connects here)
#   UDP 14550  — MAVROS2 (this script, listens for MAVProxy output)
#   UDP 14551  — pymavlink in flight_commander.py (EKF origin, EKF status, altitude)
#
# MAVProxy forwards MAVLink to both ports via --out udp flags in the SITL command.
# MAVROS2 and flight_commander.py coexist because they listen on different ports.
#
# Prerequisites (one-time):
#   sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs
#   sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
#
# Run order:
#   Terminal 1: SITL         (sim_vehicle.py ... --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551)
#   Terminal 2: drone_sim    (python3 control/drone_sim.py)
#   Terminal 3: MAVROS2      (this script)
#   Terminal 4: AnyLoc node  (./anyloc/run_ros2_localizer.sh)
#   Terminal 5: Flight cmd   (python3 control/flight_commander.py)

source /opt/ros/jazzy/setup.bash

ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:="udp://:14550@" \
    -p tgt_system:=1 \
    -p tgt_component:=1 \
    -p log_output:="screen" \
    -p fcu_protocol:="v2.0"
