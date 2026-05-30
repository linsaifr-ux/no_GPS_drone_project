#!/bin/bash
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
DISPLAY=:2 conda run -n isaac_sim_test python3 anyloc/ros2_node.py "$@"
