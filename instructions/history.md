# Project History

## 2026-05-15 — Simulator working

### What was done

Built a working Isaac Sim 6.0.0 scene for Chiayi, Taiwan centred at 23.450868°N, 120.286135°E.

**Data sources (all via Cesium ion REST API — no Cesium for Omniverse extension):**
- Terrain: Cesium World Terrain (asset 1), quantized-mesh-1.0, 9 tiles at level 13
- Buildings: Cesium OSM Buildings (asset 96188), B3DM format, 83 buildings from 4 tiles at level 12
- Imagery: Taiwan NLSC PHOTO2 aerial orthophoto WMTS, zoom 18, resized to 4096×4096

**Why no Cesium for Omniverse extension:**
Cesium for Omniverse v0.22–0.26 targets Kit 105.1/106.5 with Python 3.10. Isaac Sim 6.0.0 uses Kit 106 / Python 3.12. No compatible version exists.

---

### Bugs fixed

**1. Quantized mesh triangle count always 0**
- Cause: erroneous 4-byte alignment padding inserted between vertex data and triangle count in `parse_quantized_mesh()`
- Fix: removed `if off % 4: off += 4 - (off % 4)` — Cesium terrain tiles have no padding there
- File: `simulator/cesium_scene.py` → `parse_quantized_mesh()`

**2. `np.arange` TypeError in building parser**
- Cause: `np.arange(len(vi), np.int32)` passes `np.int32` as stop value, not dtype
- Fix: `np.arange(len(vi), dtype=np.int32)`
- File: `simulator/cesium_scene.py` → `parse_b3dm_buildings()`

**3. Stale terrain tile list with bad URLs**
- Cause: `cesium_terrain_list.json` was cached with relative URLs containing literal `{version}` placeholder
- Fix: deleted the stale cache file; added URL resolution logic in `fetch_terrain_tiles()` to prepend `base_url` for relative templates and replace `{version}` with `"1.2.0"`
- File: `simulator/cesium_scene.py` → `fetch_terrain_tiles()`

**4. Satellite imagery — switched from ESRI to Bing to NLSC**
- ESRI World Imagery and Bing Maps Aerial both use Maxar source for Taiwan — visually identical
- Switched to Taiwan NLSC PHOTO2 orthophoto WMTS (free, no API key, up to zoom 20)
- URL pattern: `https://wmts.nlsc.gov.tw/wmts/PHOTO2/default/GoogleMapsCompatible/{z}/{y}/{x}`
- Note: Bing Maps Aerial via Cesium ion asset 2 returns `externalType: BING` with a Bing API key (not a Cesium tile server) — requires quadkey conversion and Bing Imagery Metadata API call to get tile URL template
- File: `simulator/cesium_scene.py` → `fetch_satellite()`

**5. White wash / overexposure**
- Cause: RTX auto-exposure histogram boosting gain on bright outdoor scene until everything washed white
- Fix:
  - DomeLight intensity: 500 → 200
  - DistantLight intensity: 6000 → 2500
  - Enabled RTX histogram auto-exposure with clamped range: `exposureMin=-4.0`, `exposureMax=0.0`
  - Set ACES filmic tonemapper: `/rtx/post/tonemap/op = 6`
- File: `simulator/cesium_scene.py` → lights section + `carb.settings`

**6. Terrain texture mirrored**
- Cause: USD `UsdUVTexture` uses OpenGL convention where `v=0` = bottom of image. Our JPEG has north at the top, but we were mapping north to `v=0`, so north terrain got south pixels — entire texture was north-south flipped, appearing as a mirror from the camera's viewpoint
- Fix: `v = 1.0 - (SAT_NW_LAT - lat_arr) / (SAT_NW_LAT - SAT_SE_LAT)`
- File: `simulator/cesium_scene.py` → `geo_to_uv()`

---

### Project structure created

```
no_GPS_drone_project/
├── instructions/
│   ├── project_plan.md    # module plans + milestones
│   └── history.md         # this file
├── simulator/             # Isaac Sim — WORKING
├── localization/          # AnyLoc — TODO
├── detection/             # YOLO — TODO
├── control/               # ArduPilot — TODO
└── .gitignore
```

---

---

## 2026-05-17 — Drone + nadir camera added (Milestone 2)

### What was done

Added a controllable drone with a downward-looking camera to `simulator/cesium_scene.py`.

**USD prims created:**
- `/World/Drone` — `Xform` with TranslateOp + RotateZOp (yaw); starts at `centre_elev + 50 m`
- `/World/Drone/Body` — flat `Cube` (0.4 × 0.4 × 0.1 m), dark-grey material
- `/World/Drone/Camera` — `Camera` prim, 24 mm focal length, 36×27 mm aperture → 84°×65° FOV, clipping 0.1–5000 m

**Nadir orientation:** In a Z-up stage, the default USD camera looks along its local −Z = world −Z (straight down). No rotation op is needed; yawing the parent `Xform` rotates the image around the nadir axis.

**Frame output (omni.replicator.core):**
- `rep.create.render_product("/World/Drone/Camera", (640, 480))`
- RGB annotator captures RGBA → strips alpha → saves as JPEG
- `simulator/drone_frames/latest.jpg` — overwritten every 5 sim steps
- `simulator/drone_frames/latest_meta.json` — `{step, lat, lon, alt_m, yaw_deg, frame_w, frame_h}`

**Keyboard drone control (carb.input):**
- W/S = move north/south (Y axis, +5 m/step)
- A/D = move west/east (X axis)
- Q/E = descend/ascend (Z axis)
- Z/X = yaw left/right (1°/step)

**New constants added:**
- `DRONE_FRAME_DIR`, `DRONE_CAM_W/H = 640/480`, `DRONE_SAVE_EVERY = 5`, `DRONE_SPEED_M = 5.0`

---

## Next session — Milestone 3

Wire the drone camera frames into AnyLoc (localization) and YOLO (object detection).

Tasks:
- Set up AnyLoc in `localization/` to read `drone_frames/latest.jpg` and return a geo estimate
- Set up YOLO in `detection/` to read the same frame and return bounding boxes
- Define shared frame interface (file-based now, upgrade to shared memory later)
- Test end-to-end: fly drone over buildings, verify detections appear
