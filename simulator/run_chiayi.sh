#!/bin/bash
# Run the Chiayi scene in Isaac Sim
# Pure Cesium ion 3D Tiles: World Terrain (asset 1) + OSM Buildings (asset 96188)
# Satellite imagery: ESRI World Imagery.  No OSM / SRTM patchwork.
# Centre: 23.450868, 120.286135 — Radius: 2 km

cd "$(dirname "$0")"
DISPLAY=:2 OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python cesium_scene.py "$@"
