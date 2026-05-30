#!/bin/bash
# Run the Chiayi scene in Isaac Sim
# Pure Cesium ion 3D Tiles: World Terrain (asset 1) + OSM Buildings (asset 96188)
# Satellite imagery: ESRI World Imagery.  No OSM / SRTM patchwork.
# Centre: 23.450868, 120.286135 — Radius: 2 km

# Source ROS2 Jazzy so Isaac Sim can use system rclpy (Python 3.12 compatible).
# ROS2 env vars (AMENT_PREFIX_PATH, ROS_DISTRO, LD_LIBRARY_PATH) are inherited
# by the conda process, and cesium_scene.py adds the site-packages path to sys.path.
source /opt/ros/jazzy/setup.bash

cd "$(dirname "$0")"
DISPLAY=:2 OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test --no-capture-output \
    python cesium_scene.py "$@"
