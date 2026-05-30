#!/bin/bash
# Launch MAVROS2 connected to ArduPilot SITL on tcp:localhost:5762.
#
# Prerequisites (one-time):
#   sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs
#   sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
#
# Run order:
#   Terminal 1: SITL         (sim_vehicle.py ... --add-param-file=control/no_gps.parm)
#   Terminal 2: stub_bridge  (python3 control/stub_bridge.py)  ← or Isaac Sim
#   Terminal 3: MAVROS2      (this script)
#   Terminal 4: AnyLoc node  (python3 anyloc/ros2_node.py)
#   Terminal 5: Flight cmd   (python3 control/flight_commander.py)

source /opt/ros/jazzy/setup.bash

ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:="tcp://localhost:5762" \
    -p gcs_url:="" \
    -p tgt_system:=1 \
    -p tgt_component:=1 \
    -p log_output:="screen" \
    -p fcu_protocol:="v2.0"
