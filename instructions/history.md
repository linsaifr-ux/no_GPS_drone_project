# Project History

## 2026-05-31 — Separate drone physics from Isaac Sim; fix VPE + takeoff

### What was done

**Separated `drone_sim.py` from `cesium_scene.py`**

The kinematic physics model and SITL bridge (previously embedded in `cesium_scene.py`) were extracted into a standalone ROS2 node `control/drone_sim.py`. Isaac Sim is now a pure visualiser: it subscribes to `/drone/state` (ENU PoseStamped, 100 Hz) and moves the USD drone mesh. This makes headless flight possible without Isaac Sim running.

- **New:** `control/drone_sim.py` — 6-DOF kinematic model + `SITLBridge` + `/drone/state` publisher
- **Modified:** `simulator/cesium_scene.py` — removed kinematic physics, SITL bridge, and keyboard control; added `/drone/state` subscriber + `_cb_drone_state()` callback
- **Deprecated:** `control/stub_bridge.py` — replaced by `drone_sim.py`

**Switched MAVROS2 and pymavlink from TCP to UDP**

`tcpin:localhost:5762` in `launch_mavros.sh` caused `PermissionError: [Errno 13] Permission denied` on socket bind inside `mavproxy`. Root cause unresolved (pure Python socket tests passed), so switched to UDP to avoid MAVProxy's `tcpin:` binding path entirely.

- `launch_mavros.sh`: `fcu_url:="tcp://localhost:5762"` → `fcu_url:="udp://:14550@"`
- `flight_commander.py`: all `udp:localhost:14550` → `udpin:0.0.0.0:14551`
- SITL command: `--out tcpin:localhost:5762` → `--out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551`
- Removed `--console --map` flags (MAVProxy GUI modules not installed)

**Fixed `MAV_CMD_NAV_TAKEOFF` missing from takeoff sequence**

`flight_commander.py`'s `takeoff()` was publishing position setpoints but never sending `MAV_CMD_NAV_TAKEOFF`. ArduPilot keeps motors at idle in "landed" state regardless of setpoint altitude. Added a `CommandTOL` call at the top of `takeoff()` before the position ramp. `_tof_cli` was already wired up but unused.

**Fixed VPE coordinate order and covariance (EKF POS_ABS was never set)**

Two bugs in `flight_commander.py`'s VPE thread:

1. **x/y swap** — `position.x = north, position.y = east` instead of ENU (x=East, y=North).
2. **Covariance 400 m² too large** — EKF3 only sets `EKF_POS_HORIZ_ABS` when internal position uncertainty is below a few metres. With 20 m std dev measurement covariance, the EKF's uncertainty stays ~20 m and the flag is never set. Reduced to 1 m² (1 m std dev). z covariance unchanged at 1e6 m².

Added diagnostic logging to `wait_ekf_pos()`: prints active EKF flags every 5 s if stuck, e.g. `EKF flags 0x00f: [ATT | VEL_H | VEL_V | POS_H_REL] — waiting for POS_H_ABS`.

### Bugs fixed

| Bug | Symptom | Fix |
|-----|---------|-----|
| Drone mesh frozen at ground in Isaac Sim | `flight_commander` sent setpoints but Isaac Sim AGL didn't change | Extracted kinematic model to `drone_sim.py`; cesium_scene.py subscribes `/drone/state` |
| `PermissionError` on MAVProxy `tcpin:` bind | SITL crashes with `[Errno 13] Permission denied` | Switched to UDP 14550/14551 |
| `Connection refused` on `--out tcp:localhost:5763` | MAVProxy exits immediately | Changed to `--out udp:` |
| `No module named 'console'`/`'map'` | MAVProxy exits | Removed `--console --map` flags |
| `link 1 down` after SITL start | ArduPilot waiting for physics bridge | `drone_sim.py` must start within ~10 s of SITL |
| EKF POS_ABS never set | `flight_commander` stuck on "Waiting for EKF POS_ABS" | Fixed VPE x/y order + reduced covariance to 1 m² |
| Drone never lifts off | AGL stays near 0 despite climbing setpoints | Added `MAV_CMD_NAV_TAKEOFF` to `takeoff()` |

---

## 2026-05-30 — ROS2 migration (Milestone 6e)

### What was done

Migrated all IPC from file polling + direct pymavlink to ROS2 topics + MAVROS2.

- **New:** `control/flight_commander.py` — full ROS2 node replacing `run_flight.py`
  - EKF origin via pymavlink (MAVROS2 Jazzy 2.14 has no service for this)
  - VPE thread publishes `PoseWithCovarianceStamped` to `/mavros/vision_pose/pose_cov`
  - Position setpoints via `/mavros/setpoint_position/local`
  - STABILIZE arm → GUIDED → EKF POS_ABS → takeoff → waypoints → RTL
- **New:** `control/launch_mavros.sh` — MAVROS2 launch script
- **Modified:** `simulator/cesium_scene.py` — added ROS2 node, publishes `/drone/camera/image_raw`, `/drone/pose`, `/drone/agl`; kinematic model driven by ArduPilot PWM via embedded SITL bridge
- **Modified:** `anyloc/ros2_node.py` — subscribes ROS2 camera/pose; publishes VPE + AnyLoc estimates

### Key design: VPE with z=1e6 covariance

`PoseWithCovarianceStamped` on `/mavros/vision_pose/pose_cov` allows setting per-axis covariance. z covariance = 1e6 m² tells EKF3 to ignore VPE altitude and rely on barometer. This prevents EKF innovation gate failures when the stub VPE z differs from baro.

---

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

## 2026-05-17 — Drone + camera + HUD (Milestone 2)

### What was done

Added a controllable quadcopter drone with nadir camera, viewport HUD, and camera toggle to `simulator/cesium_scene.py`.

**USD prims — quadcopter model (~0.8 m span):**
- `/World/Drone` — `Xform` with `TranslateOp` + `RotateZOp` (yaw); starts at `centre_elev + 50 m`
- `/World/Drone/Body` — flat `Cube` (0.28 × 0.28 × 0.08 m), dark-grey
- `/World/Drone/Arm_NE/NW/SW/SE` — thin `Cube` arms at 45°/135°/225°/315°, dark-grey
- `/World/Drone/Motor_NE/…` — upright `Cylinder` pods at arm tips (r=0.035 m)
- `/World/Drone/Prop_NE/…` — flat `Cylinder` propeller discs above each motor (r=0.13 m)
- `/World/Drone/Beacon` — `SphereLight` (orange, 5000 cd) — visible as a coloured dot from the overview camera
- `/World/Drone/Camera` — `Camera` prim, 18 mm focal length, 36×27 mm aperture → **90°×73.7° FOV**, 640×480, clipping 0.1–5000 m

**Nadir orientation:** In a Z-up stage, default USD camera looks along local −Z = world −Z (straight down). No rotation op needed; yawing the parent `Xform` rotates the image around the nadir axis.

**Frame output (`omni.replicator.core`):**
- `rep.create.render_product("/World/Drone/Camera", (640, 480))`
- RGB annotator: RGBA → strip alpha → JPEG → `drone_frames/latest.jpg` every 5 sim steps
- `drone_frames/latest_meta.json` — `{step, lat, lon, alt_m, yaw_deg, frame_w, frame_h}`
- Viewport (Tab, 1920×1080) and render product (640×480) are **intentionally separate** — same camera and 90° HFOV, different aspect ratio and resolution. Viewport is for visual inspection; render product is the ML input.

**HUD overlay (`omni.ui`):**
- Semi-transparent dark window pinned to top-left corner, always on top
- Shows live: `LAT` / `LON` (5 dp) · `ALT` (MSL + AGL) · active `CAM` name
- Updates every sim step; wrapped in try/except so sim still runs if `omni.ui` fails

**Keyboard controls (`carb.input` + `omni.appwindow`):**
- Tab = toggle viewport: overview ↔ drone nadir (edge-detected, one press = one toggle)
- W/S = N/S · A/D = W/E · Q/E = down/up · Z/X = yaw ±1°/step · all ±5 m/step

---

### Bugs fixed

**1. `carb.input.IInput` has no `get_keyboard()` method**
- Cause: `get_keyboard()` lives on the app window, not the input interface
- Fix: `omni.appwindow.get_default_app_window().get_keyboard()`
- File: `simulator/cesium_scene.py` → keyboard setup block

**2. Camera FOV stated as 84°×65° — wrong**
- Cause: arithmetic error; 24 mm / 36×27 mm aperture gives 73.7°×58.7°, not 84°×65°
- Fix: corrected FOV formula `2 × arctan(aperture / (2 × focalLength))` and changed focal length to 18 mm to achieve the desired 90°×73.7°
- Files: `cesium_scene.py` comment, `project_plan.md`, `README.md`

---

---

## 2026-05-18 — Frame capture fix

### Bug fixed

**`_rgb.get_data()` silently returning `None` — no frames saved**
- Cause: `omni.replicator.core` does not render into the render product automatically during a manual `simulation_app.update()` loop. Without an explicit replicator step, `get_data()` always returns `None` and the save block was silently skipped.
- Fix: call `rep.orchestrator.step(rt_subframes=1, delta_time=0.0)` immediately before `get_data()` each capture cycle. This forces the RTX renderer to produce one frame into the render product.
- Added explicit `print` warnings when `get_data()` returns `None` or an empty array, so silent failures are visible in the terminal.
- Added a one-time confirmation message (`[DRONE] Frame capture working`) on the first successful save.
- File: `simulator/cesium_scene.py` → frame capture block in simulation loop

---

---

## 2026-05-20 — AnyLoc localization + dual postview (Milestone 3)

### What was done

Created `anyloc/` with a working AnyLoc visual localization pipeline and two live postview windows.

**Files created:**
- `anyloc/build_database.py` — builds a geo-tagged image database from the NLSC satellite orthophoto
- `anyloc/localizer.py` — AnyLocLocalizer class: DINOv2 ViT-B/14 + intra-normalised VLAD + FAISS nearest-neighbour; `localize(img, agl_m)` re-crops satellite at drone's actual AGL
- `anyloc/run_localizer.py` — main loop: watches `drone_frames/latest.jpg`, runs localisation, shows two matplotlib windows
- `anyloc/requirements.txt` — dependency notes
- `anyloc/database/` — built database (172 entries, VLAD dim=49,152)

**Modified:**
- `simulator/cesium_scene.py` — `latest_meta.json` now also writes `agl_m` and `centre_elev`

**Database:**
- Grid: 200 m step, ±1500 m from scene centre → 172 positions
- Drone AGL: 50 m (sets ground footprint size for satellite crops)
- Satellite crop per position → resize to 640×480 → DINOv2 ViT-B/14 patch features (768-dim)
- faiss.Kmeans k=64 codebook → intra-normalised VLAD → 64×768=49,152-dim descriptors
- FAISS IndexFlatIP (cosine similarity)

**Two postview windows (`run_localizer.py`):**
- `[Drone Camera]` — live `latest.jpg` with ground-truth geo overlay (LAT/LON/ALT MSL/AGL/YAW)
- `[AnyLoc Match]` — satellite crop re-cropped at **drone's actual AGL** at the matched position, with estimated geo overlay (LAT/LON/ALT AGL/ERR/time)
- Text colour: green if error < 200 m, blue otherwise
- Display: matplotlib TkAgg (not cv2 — cv2 in this env is headless)

**Measured performance (RTX 2080 Ti, cuda):**
- DINOv2 inference + VLAD + FAISS search: ~183 ms per frame
- Typical localisation error at 50 m AGL: ~65 m (≈ 1 grid step = 200 m)

---

### Bugs fixed

**1. numpy dual-install conflict (pip numpy 2.3.1 vs conda numpy 1.26.4)**
- Cause: conda-forge faiss-cpu installation pulled in numpy 2.x files over the Isaac Sim numpy 1.26.4, corrupting `numpy/core/_dtype.py`. ANY numpy operation failed.
- Fix:
  - Avoided all numpy operations in the VLAD pipeline — use torch tensors throughout
  - Used `pil.tobytes() → torch.frombuffer()` instead of `np.array(pil_img)` everywhere
  - Used `torch.save()` / `torch.load()` instead of `np.savez_compressed()` for database
  - Used `tensor.numpy()` only at faiss call sites (torch's numpy binding is ABI-compatible with cv2)
  - Force-reinstalled numpy 1.26.4 from conda-forge to restore numpy itself
- Files: `anyloc/build_database.py`, `anyloc/localizer.py`, `anyloc/run_localizer.py`

**2. torchvision.ToTensor() TypeError**
- Cause: `T.ToTensor()` internally calls `np.array(pic, dtype, copy=True)` then `torch.from_numpy()`, both fail with the dual-numpy conflict
- Fix: replaced transform pipeline with `pil.tobytes() + torch.frombuffer()` approach
- Files: `anyloc/build_database.py`, `anyloc/localizer.py`

**3. torch.from_numpy() TypeError**
- Cause: torch checks `isinstance(obj, <torch-numpy>.ndarray)` but the array was from a different numpy install
- Fix: for faiss centroid output, copy via `bytearray(arr.tobytes()) → frombuffer`
- File: `anyloc/build_database.py`

**4. cv2.namedWindow crash — OpenCV headless**
- Cause: cv2 in `isaac_sim_test` was built without GUI support (`GUI: NONE`)
- Fix: replaced all cv2 display calls with **matplotlib (TkAgg backend)**; text overlays drawn with PIL `ImageDraw` to avoid numpy ops
- File: `anyloc/run_localizer.py`

**5. tight_layout UserWarning**
- Cause: `plt.tight_layout()` incompatible with image axes that have no labels
- Fix: replaced with `layout='constrained'` on the figure constructor
- File: `anyloc/run_localizer.py`

**6. UnidentifiedImageError — mid-write race condition**
- Cause: localiser reads `latest.jpg` while the simulator is still writing it, producing a truncated JPEG
- Fix: wrapped `Image.open` + `frame.load()` in `try/except`; on error, prints a warning and retries next 150 ms tick without updating `last_mtime`
- File: `anyloc/run_localizer.py`

**7. AnyLoc match altitude always 50 m**
- Cause: `localize()` returned the DB entry's fixed 50 m AGL regardless of the drone's actual altitude; the match image was a 50 m footprint crop
- Fix: `localize(img, agl_m)` now accepts the drone's AGL, re-crops the satellite orthophoto at that altitude centred on the matched position, and returns `agl_m` as `est_alt`
- Files: `anyloc/localizer.py` (added `_sat_crop`, `_load_sat`, `agl_m` param), `anyloc/run_localizer.py` (passes `drone_agl`)

---

---

---

## 2026-05-20 — AnyLoc grid densification + VO refinement

### What was done

**Grid step reduced 200 m → 50 m (`anyloc/build_database.py`):**
- Changed `--grid-step` default from 200 to 50
- Rebuilt database: 2,821 entries (was 172), VLAD dim=49,152 unchanged
- Expected localisation error: ~15–20 m (was ~65 m)
- Hard accuracy floor at this AGL: ~50 m grid ≈ camera footprint width (~100 m × 75 m at 50 m AGL); going finer produces overlapping images that are indistinguishable

Accuracy table (for reference):

| Grid step | Entries | Expected error |
|-----------|---------|----------------|
| 200 m | 172 | ~65 m |
| 100 m | ~688 | ~30–40 m |
| **50 m (current)** | **2,821** | **~15–20 m** |
| 25 m | ~11,000 | ~8–12 m |

**Visual Odometry (VO) refinement implemented:**

New file `anyloc/vo_refiner.py` — `VORefiner` class using LK optical flow:
- Detects Shi-Tomasi corner features (`cv2.goodFeaturesToTrack`)
- Tracks them with Lucas-Kanade optical flow (`cv2.calcOpticalFlowPyrLK`)
- Median pixel displacement → ground metres → Δlat/Δlon via AGL + FOV + yaw rotation:
  - `raw_east = -dx_px × m_per_px_x` (feature right → drone moved west)
  - `raw_north = +dy_px × m_per_px_y` (feature down → drone moved north)
  - World ENU: `east = raw_east·cos(yaw) + raw_north·sin(yaw)`, `north = -raw_east·sin(yaw) + raw_north·cos(yaw)`
- `reset()` clears tracked state after each AnyLoc re-anchor

Updated `anyloc/run_localizer.py`:
- `ANYLOC_INTERVAL = 10` — full AnyLoc retrieval every 10 frames (~2 s at 5-step sim)
- Between anchors: VO accumulates `accum_dlat / accum_dlon`; final position = anchor + accumulated delta
- Panel 2 mode tag: `ANYLOC` on anchor frames, `VO +Nf` otherwise; also shows tracked point count
- Expected combined accuracy: ~5–10 m between anchor fixes

**Docs and .gitignore updated:**
- `.gitignore` — added `anyloc/database/` and `anyloc/test_output/`
- `README.md`, `project_plan.md` — reflect Milestone 3 done, new database size, VO documented

---

### Bug fixed

**`ok.sum()` hits broken numpy `_core/_methods.py` (numpy 2.x stub)**
- Cause: `cv2.calcOpticalFlowPyrLK` returns a numpy array `status`; calling `.sum()` on `status.flatten() == 1` triggers numpy's Python-level dispatch in `_core/_methods.py`, which is a numpy 2.x file still present in the env
- Fix: `sum(ok.tolist())` — `.tolist()` is C-level (safe), `sum()` on a Python list is pure Python
- File: `anyloc/vo_refiner.py` → `update()`

---

## 2026-05-22 — Geo-constrained AnyLoc search

### Motivation

The original AnyLoc retrieval searches all 2,821 database entries every time. Because VLAD descriptors can confuse visually similar tiles (rice paddies, rooftops, road intersections), the top-1 match occasionally jumps hundreds of metres to the wrong tile on the other side of the scene. Once that happens, the VO accumulation starts from the wrong anchor and the error compounds.

The fix: after the first anchor is established, restrict the FAISS / similarity search to only the database entries that are geographically plausible given how far the drone could have moved since the last anchor.

---

### Implementation

**`anyloc/localizer.py` — `AnyLocLocalizer.localize()`**

New optional parameters:
```
center_lat  float  — latitude of the search centre (VO-refined estimate)
center_lon  float  — longitude of the search centre
radius_m    float  — search radius in metres (default unused = full search)
```

When all three are provided the method skips the FAISS index and does:

```python
# 1. Flat-Earth distance from every DB entry to the search centre
dlat     = (self.lats - center_lat) * 111_320.0          # metres north
dlon     = (self.lons - center_lon) * 111_320.0 * COS_LAT  # metres east
in_range = ((dlat**2 + dlon**2) <= radius_m**2)           # boolean mask
           .nonzero(as_tuple=False).squeeze(1)             # index tensor

# 2. Cosine similarity on the subset (both desc and vlads are L2-normalised)
sims  = self.vlads[in_range] @ desc   # (M,) — inner product = cosine sim
best  = int(sims.argmax())
idx   = int(in_range[best])           # index back into full DB
score = float(sims[best])
```

The flat-Earth approximation (`111,320 m per degree lat`, scaled by `cos(lat)` for lon) introduces < 0.1 % error over the 2 km scene radius — negligible.

All operations are pure torch tensors, keeping the numpy-safety rules of the `isaac_sim_test` env (no `np.array`, no numpy reductions). The subset is typically ~50 entries at 200 m radius, down from 2,821 — making this path faster than FAISS even without the index.

If `in_range` is empty (VO drifted badly or first frame), the code falls back to the full FAISS IndexFlatIP search automatically.

**`anyloc/run_localizer.py` — main loop**

On every AnyLoc frame (every 10th frame after the first), the VO-accumulated offset is added to the last anchor to form the search centre:

```python
clat = (anchor_lat + accum_dlat) if anchor_lat is not None else None
clon = (anchor_lon + accum_dlon) if anchor_lat is not None else None
loc.localize(frame, agl_m=drone_agl,
             center_lat=clat, center_lon=clon, radius_m=200.0)
```

- Frame 1: `anchor_lat is None` → `clat = None` → full FAISS search (2,821 entries)
- Frame 10+: `clat` = VO estimate → constrained torch search (~50 entries within 200 m)

---

### Why 200 m radius

| Factor | Value |
|--------|-------|
| DB grid spacing | 50 m |
| Grid steps covered by 200 m radius | 4 in each direction |
| Entries inside 200 m circle (approx.) | π × (200/50)² ≈ 50 |
| Max drone speed (sim) | ~20 m/s |
| Time between AnyLoc runs (10 f @ ~5 fps) | ~2 s |
| Max real displacement between runs | ~40 m |
| VO error on 40 m displacement | < 10 m typical |
| Safety margin (200 m vs 50 m max displacement) | ~4× |

200 m is the smallest radius that is robustly larger than any plausible true displacement + VO error, while still covering only ~2 % of the full database (50 / 2,821).

Going smaller (e.g. 100 m) risks clipping the true position when the drone moves fast or VO drifts. Going larger (e.g. 500 m) reduces the benefit — more wrong tiles enter the candidate set.

---

### Effect on accuracy

Without the constraint, a single wrong anchor propagates until the next large-error AnyLoc run corrects it — but that run is also unconstrained and can jump again. The constrained search makes each AnyLoc run self-correcting: even if the previous anchor was slightly off, the new search centre (anchor + VO) is close enough to the true position that the correct tile is almost always in the 200 m window.

---

## 2026-05-22 — YOLO vehicle detection module (Milestone 5)

### What was done

Created `detection/` with a working YOLOv8 vehicle detection pipeline and live postview.

**Files created:**
- `detection/detector.py` — `YOLODetector` class
- `detection/run_detector.py` — mtime-polling postview loop

**`YOLODetector` (`detector.py`):**
- Loads `yolov8n.pt` (ultralytics YOLOv8 nano, COCO pretrained, ~6 MB, auto-downloaded on first run)
- `detect(pil_img)` — runs inference, filters to COCO vehicle class IDs `{2: car, 3: motorcycle, 5: bus, 7: truck}`, returns list of `{label, conf, x1, y1, x2, y2}` dicts; coordinates extracted via `box.xyxy[0].tolist()` (torch-level, avoids numpy dispatch)
- `draw(pil_img, detections)` — PIL `ImageDraw` bounding boxes + filled label chips per class colour; returns new PIL RGB image; numpy-safe

**`run_detector.py`:**
- Same mtime-polling pattern as `run_localizer.py` (polls `drone_frames/latest.jpg` every 50 ms)
- Single matplotlib TkAgg window; `fig.canvas.draw()` + `flush_events()` for synchronous render
- Window title: vehicle count + inference time + drone lat/lon; green title when detections present
- Terminal: one `[YOLO]` line per detected vehicle with label, confidence, bounding box

**Dependency installed:**
- `ultralytics 8.4.52` installed via `python -m pip install ultralytics` in `isaac_sim_test`

**Known limitation:**
YOLOv8n was trained on eye-level COCO images. Nadir (top-down) vehicle views differ substantially in appearance and aspect ratio — detection confidence is lower from directly above. Fine-tuning on aerial imagery (DOTA, VisDrone) is needed for production accuracy.

---

---

## 2026-05-23 — Top-down YOLO fine-tuning pipeline

### What was done

Built a complete fine-tuning pipeline for adapting YOLOv8 to nadir (top-down) aerial vehicle detection. The existing `yolov8n.pt` was trained on eye-level COCO photos; this session adds the infrastructure to train on aerial imagery.

**Files created:**

- `detection/label_writer.py` — pure-Python nadir camera projection; given drone ENU position + vehicle position / yaw / class, projects the 4 footprint corners through the camera (fx=fy=320, 640×480) and returns a normalised YOLO bounding box. No numpy — safe inside `isaac_sim_test` env.

- `detection/collect_training_data.py` — Isaac Sim headless synthetic data collector. Builds a flat scene with 43 coloured vehicle boxes (25 cars, 8 motos, 4 buses, 6 trucks at random positions / yaws). Flies a grid at 30 m / 60 m / 100 m AGL with 35 % lateral overlap. At each of ~70 grid positions, captures a frame and writes a YOLO label via `label_writer`. Uses `Image.frombytes("RGBA", ...)` instead of `.astype()` to safely convert the replicator buffer inside the broken-numpy env.

- `detection/prepare_dataset.py` — downloads VisDrone 2019 DET via `ultralytics.data.utils.check_det_dataset("VisDrone.yaml")`; remaps 7 VisDrone classes to 4 targets (`car/motorcycle/bus/truck`); symlinks images and writes YOLO `.txt` labels into `detection/dataset/{images,labels}/{train,val}/`; merges any synthetic data from `detection/dataset/synth/`; writes `data.yaml`.

  VisDrone → canonical map: car(4)→car, van(5)→car, truck(6)→truck, tricycle(7)→moto, awning-tricycle(8)→moto, bus(9)→bus, motor(10)→moto.

- `detection/finetune.py` — loads `yolov8n.pt`, trains 100 epochs with augmentations tuned for nadir aerial: `degrees=45`, `flipud=0.5`, `scale=0.5` (altitude variation), `mosaic=1.0` (small objects), `hsv_v=0.4` (lighting variation). Saves to `detection/runs/topdown_v1/weights/best.pt`.

---

## 2026-05-24 — Switched to yolov8l_visdrone.pt; auto class-map in detector

### What was done

Switched the active detection model from `yolov8n.pt` (COCO) to `yolov8l_visdrone.pt` (YOLOv8-large, pre-trained on VisDrone 2019 DET). This immediately improves aerial vehicle detection without any training.

**`detection/detector.py` — refactored class mapping:**

Replaced the hardcoded COCO class ID dict `{2: 'car', 3: 'motorcycle', ...}` with a name-based lookup built at load time:

```python
_NAME_TO_LABEL = {
    'car': 'car', 'van': 'car',
    'truck': 'truck',
    'bus': 'bus',
    'motorcycle': 'motorcycle', 'motor': 'motorcycle',
    'tricycle': 'motorcycle', 'awning-tricycle': 'motorcycle',
}

self._filter = {
    cid: _NAME_TO_LABEL[name]
    for cid, name in self.model.names.items()
    if name in _NAME_TO_LABEL
}
```

`self._filter` is built from `model.names` so the same `YOLODetector` class works for both COCO and VisDrone models — no code change needed when swapping models.

VisDrone model class map: `{3: car, 4: car, 5: truck, 6: motorcycle, 7: motorcycle, 8: bus, 9: motorcycle}` — 7 aerial vehicle classes covered.

**`detection/run_detector.py`:**
- Added `MODEL_PT = os.path.join(ROOT, 'yolov8l_visdrone.pt')`
- Changed `YOLODetector('yolov8n.pt', conf=0.35)` → `YOLODetector(MODEL_PT, conf=0.30)` (lower threshold appropriate for a model already trained on aerial imagery)

---

## 2026-05-27 — Architecture decisions: ArduPilot SITL + MAVLink + IMU

### Decisions made

**1. ArduPilot SITL + MAVLink before IMU implementation**

On real hardware, IMU data arrives via MAVLink `HIGHRES_IMU` messages from the flight controller. Building the IMU reader against MAVLink now means zero interface changes at deployment. ArduPilot SITL's sensor pipeline also provides realistic noise, bias drift, and temperature effects that analytical position derivatives cannot replicate.

Build order:
1. `control/sitl_bridge.py` — Isaac Sim → ArduPilot SITL JSON/UDP physics state bridge
2. `control/mavlink_ctrl.py` — pymavlink subscriber + `SET_POSITION_TARGET_LOCAL_NED` sender
3. `control/imu_reader.py` — reads `HIGHRES_IMU` from MAVLink stream
4. `control/imu_fusion.py` — uses IMU to validate AnyLoc anchors + gate VO quality

**2. Physics-based IMU via ArduPilot SITL JSON backend (not analytical derivatives)**

ArduPilot SITL receives the drone's physics state from Isaac Sim each step (position, velocity, acceleration, attitude in NED), runs its own sensor models, and outputs `HIGHRES_IMU` over MAVLink — the same message format a real ArduPilot FC sends.

**3. IMU role in localization: sanity check on AnyLoc anchors**

Context: the geo-constrained AnyLoc search (200 m window) prevents most bad jumps, but if the constraint window itself drifts (wrong anchor accepted), the system cannot self-correct. IMU dead-reckoning provides an independent position estimate to validate new anchors:

- If new AnyLoc anchor deviates > `jump_threshold` from IMU-predicted position → reject anchor
- If IMU detects high angular velocity / acceleration spike → skip VO accumulation for that frame
- If both AnyLoc and VO fail → use IMU dead-reckoning for short bridging intervals

### Architecture

```
Isaac Sim physics state (JSON/UDP, each step)
    ↓
ArduPilot SITL (JSON backend)
    ↓ MAVLink UDP:14550
    ├─ HIGHRES_IMU → imu_reader.py → imu_fusion.py (anchor validator + VO gate)
    ├─ ATTITUDE, LOCAL_POSITION_NED → state estimation
    └─ accepts SET_POSITION_TARGET_LOCAL_NED (replaces keyboard control)
```

---

## 2026-05-27 — Milestone 6a: ArduPilot SITL JSON bridge

### What was done

Created `control/sitl_bridge.py` and wired it into `simulator/cesium_scene.py`.

**Files created:**
- `control/__init__.py` — makes `control/` a Python package
- `control/sitl_bridge.py` — `SITLBridge` class

**`SITLBridge` class:**
- Sends drone physics state to ArduPilot SITL JSON backend via UDP (port 9002) every sim step
- Receives servo/motor outputs from SITL on port 9003 (`recv_servos()`) — used in milestone 6b
- Takes Isaac Sim ENU state `(x_enu, y_enu, z_abs, yaw_deg)` each step and converts to ArduPilot NED JSON

**Coordinate conversions:**
- ENU → NED: `north = y_enu`, `east = x_enu`, `down = -(z_abs - centre_elev)`
- Yaw: Isaac Sim RotateZ CCW-positive → ArduPilot NED CW-positive: `yaw_rad = -radians(yaw_deg)`

**Computed quantities (no physics engine — finite difference):**
- Velocity NED: `Δpos / Δt`, clamped to ±30 m/s (prevents spikes when keyboard moves 5 m/step)
- Acceleration NED: `Δvel / Δt`, low-pass filtered (α=0.3 EMA) to smooth keyboard jump artifacts
- IMU specific force (body frame): `accel_ned - (0, 0, +g)` rotated by yaw into body frame
  - At hover: `[0, 0, -9.81]` ✓
- Yaw rate: `Δyaw / Δt` with wrap-to-`[-π, π]`
- Barometric pressure: ISA approximation `101325 × exp(-alt_msl / 8500)`

**`simulator/cesium_scene.py` changes (3 edits):**
1. Added `sys` to imports; added `sys.path.insert` so `control/` is importable from `simulator/`
2. After terrain load (when `centre_elev` is known): `_sitl = SITLBridge(centre_elev=centre_elev)`; wrapped in `try/except ImportError` so sim still runs without the bridge
3. In simulation loop after HUD update: `_sitl.step(x, y, alt, yaw, time.time())` called every step (not gated to DRONE_SAVE_EVERY)

**Known limitations:**
- The drone is a scripted Xform — position jumps 5 m per key press. Velocity/acceleration clamp and EMA filter prevent SITL from seeing implausible IMU values, but the motion is not physically realistic. Milestone 6b-iv replaces keyboard control with `SET_POSITION_TARGET_LOCAL_NED` commands from ArduPilot.
- The JSON bridge currently sends `position_xyz` (ground-truth position from Isaac Sim), which ArduPilot EKF3 treats as a GPS substitute. This is **not** the no-GPS pipeline. Milestone 6b-ii removes `position_xyz` from the bridge and milestone 6b-iii replaces it with AnyLoc estimates sent via `VISION_POSITION_ESTIMATE` MAVLink messages.

**Run order:**
```bash
# Terminal 1 — start ArduPilot SITL first
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --console --map

# Terminal 2 — start Isaac Sim (bridge auto-connects on first step)
cd simulator && ./run_chiayi.sh
```

---

## 2026-05-28 — ArduPilot SITL build, protocol fix, milestone restructure

### ArduPilot SITL build

`--depth=1` clone does not pull submodules. Required submodules and fixes:

```bash
# All submodules at once (avoids discovering them one by one as build fails)
git submodule update --init --depth=1 --recursive   # ~5 min

# Configure and build ArduCopter SITL binary
python3 waf configure --board sitl
python3 waf copter   # ~2 min, binary → build/sitl/bin/arducopter
```

System Python3 dependencies installed for SITL tooling:
```bash
pip3 install --user --break-system-packages pexpect mavproxy pymavlink future
```

### Bug fixed: sitl_bridge.py protocol was backwards

**Original (wrong):** bridge was a UDP client that pushed physics state to port 9002 (as if ArduPilot was listening there).

**Correct:** ArduPilot is the JSON **client** — it sends `{"pwm": [...], "frame_time_us": N}` to the simulator and waits for physics state back. The bridge must be a UDP **server** listening on port 9002, receiving servo packets, and replying with physics state.

Fix: rewrote `SITLBridge` from a push client to a request-response server:
- `self._sock.bind(("0.0.0.0", 9002))` — server binds
- Each `step()`: drains incoming servo packets (non-blocking), learns `_ap_addr` from first packet, replies to that address with current physics state
- `step()` returns the latest servo dict for use in milestone 6b-iv

The message "No JSON sensor message received, resending servos" is ArduPilot's normal retry output while waiting for the simulator — it stops once Isaac Sim is running and the bridge replies.

### Architecture clarification: position_xyz is GPS, not no-GPS

`position_xyz` and `velocity_xyz` in the JSON bridge packet act as a GPS substitute in ArduPilot's EKF3. Sending them defeats the no-GPS goal. They are intentionally omitted from the bridge.

The no-GPS position source is `VISION_POSITION_ESTIMATE` MAVLink messages, sent from `mavlink_ctrl.py` using AnyLoc position estimates. ArduPilot EKF3 fuses this as an external vision source — same mechanism as Intel RealSense T265 or OptiTrack on real hardware.

### Milestone 6b restructured into 4 ordered sub-steps

| Sub-step | What |
|---|---|
| 6b-i | pymavlink connection to ArduPilot MAVLink output (UDP:14550) |
| 6b-ii | Disable GPS (`GPS_TYPE=0`); bridge sends IMU+baro only |
| 6b-iii | `VISION_POSITION_ESTIMATE` from AnyLoc → ArduPilot EKF3 |
| 6b-iv | `SET_POSITION_TARGET_LOCAL_NED` flight commands (replaces keyboard) |

6b-iii must precede 6b-iv: ArduPilot refuses position commands until EKF3 has a valid position fix.

---

## 2026-05-28 — Milestone 6b-i: pymavlink connection (control/mavlink_ctrl.py)

### Files created

**`control/mavlink_ctrl.py`** — `MAVLinkCtrl` class:
- `__init__(connection_str="tcp:localhost:5762")` — connects directly to ArduPilot SITL
  TCP port 5762 (no mavproxy needed; UDP:14550 was found to not deliver packets reliably)
- `wait_heartbeat(timeout=60)` — blocking; learns `target_system` / `target_component`
  from first HEARTBEAT, then requests data streams
- `recv()` — non-blocking drain; updates `_imu`, `_attitude`, `_local_pos`, `_ekf`,
  `_heartbeat` from incoming MAVLink messages; returns list of type strings received
- `_request_streams()` — asks ArduPilot for all data streams at 10 Hz via
  `REQUEST_DATA_STREAM_ALL`; requests `HIGHRES_IMU` separately at 50 Hz via
  `MAV_CMD_SET_MESSAGE_INTERVAL`
- Properties: `connected`, `imu`, `attitude`, `local_pos`, `ekf_flags`, `ekf_pos_valid`
- Stubs for 6b-iii: `send_vision_position(north, east, down, yaw_rad, covariance)`
  — sends `VISION_POSITION_ESTIMATE`; default covariance 5 m position std, 0.2 rad
  orientation std (needs tuning once AnyLoc error is characterised)
- Stubs for 6b-iv: `arm()`, `takeoff(alt_m)`, `set_position_ned(north, east, down, yaw_rad)`
  — `set_position_ned` uses `SET_POSITION_TARGET_LOCAL_NED` with type_mask
  `0b111111111000` (position only) or `0b110111111000` (position + yaw)

**`control/run_mavlink.py`** — terminal monitor:
- Connects, waits for HEARTBEAT, then prints rolling single-line display at 10 Hz:
  roll/pitch/yaw (degrees), NED position (metres), IMU accelerations (m/s²), EKF flags
- EKF flags decoded to named labels: ATT, VEL, POS_REL, POS_ABS, PRED_ABS

### EKF_STATUS_REPORT flag constants (exported from mavlink_ctrl.py)

| Constant | Bit | Hex | Meaning |
|---|---|---|---|
| `EKF_ATTITUDE` | 0 | 0x0001 | Attitude valid |
| `EKF_VEL_HORIZ` | 1 | 0x0002 | Horizontal velocity valid |
| `EKF_POS_HORIZ_REL` | 3 | 0x0008 | Relative horizontal position valid |
| `EKF_POS_HORIZ_ABS` | 4 | 0x0010 | Absolute horizontal position valid (GPS or vision fused) |
| `EKF_PRED_POS_HORIZ_ABS` | 9 | 0x0200 | Vision position estimate accepted by EKF3 |
| `EKF_UNINITIALIZED` | 10 | 0x0400 | EKF has not finished initialising (normal at startup) |

`EKF_POS_HORIZ_ABS` going high is the signal that `VISION_POSITION_ESTIMATE` is being
fused — needed before 6b-iv flight commands will be accepted.

### Notes
- `"position"` and `"velocity"` are sent in the JSON bridge for now (GPS substitute using correct SIM_JSON key names).
  They are removed in milestone 6b-ii after `VISION_POSITION_ESTIMATE` is working.
- The bridge's `step()` returns the latest parsed servo dict; 6b-iv reads PWM from there.

---

## 2026-05-28 — Three SITL bridge bugs fixed; EKF_UNINITIALIZED added

### Root cause of "No JSON sensor message received, resending servos"

Three compounding bugs in `control/sitl_bridge.py` prevented ArduPilot from ever receiving physics replies:

**Bug 1 — Binary servo packets were being parsed as JSON (root cause of _ap_addr never set)**

ArduPilot's `SIM_JSON::output_servos()` sends a C struct `servo_packet_16` (40 bytes, little-endian):
```c
struct servo_packet_16 { uint16_t magic=18458; uint16_t frame_rate; uint32_t frame_count; uint16_t pwm[16]; };
```
The bridge called `json.loads(data.decode('utf-8'))` on this binary data — always failing.
With the previous session's `_ap_addr` fix (only set after valid JSON parse), `_ap_addr` was never learned, so no physics replies were ever sent.

Fix: added `_parse_servo_packet()` which uses `struct.unpack("<HHI16H", data)` to parse the binary packet and validates the magic number (18458 for 16-channel, 29569 for 32-channel) before setting `_ap_addr`.

**Bug 2 — Missing `\n` terminator on physics JSON**

ArduPilot's `recv_fdm()` (in `SIM_JSON.cpp`) processes messages by replacing `\n` with `\0` as a delimiter, then uses `memrchr(..., 0, ...)` to locate the last complete message. Without a trailing `\n`, `memrchr` returns `nullptr` and the function returns early without parsing — every physics packet silently discarded.

Fix: append `"\n"` to every physics JSON packet before sending.

**Bug 3 — Wrong JSON key names**

The bridge sent keys like `"imu_angular_velocity_rpy"`, `"velocity_xyz"`, `"attitude_rpy"` which don't exist in ArduPilot's `SIM_JSON` keytable. Required keys are:
- `"timestamp"` (root, required)
- `"imu": {"gyro": [...], "accel_body": [...]}` (section required)
- `"velocity": [vn, ve, vd]` (root, required)
- `"attitude": [roll, pitch, yaw]` (root, required for either attitude or quaternion)

Fix: rewrote `_build_state()` return dict to use the exact key names from `SIM_JSON.h`.

### Files modified

- `control/sitl_bridge.py` — all three fixes; removed `import json` fallback path for servos; added `struct` import and binary constants; physics send now appends `\n`
- `control/mavlink_ctrl.py` — added `EKF_UNINITIALIZED = 1 << 10`
- `control/run_mavlink.py` — `_ekf_label()` now returns `"UNINIT"` for bit 10 instead of `"none"`

---

## 2026-05-28 — Milestone 6b-ii: disable GPS, strip position from bridge

### What was done

Removed `"position"` and `"velocity"` from the JSON physics packet and added a SITL parameter file to disable the GPS sensor.

**`control/sitl_bridge.py`** — `_build_state()` no longer includes `"position"` or `"velocity"` in the returned dict. `vel_ned` and `accel_ned` are still computed internally because `accel_body` (the IMU specific force) is derived from them; they just aren't sent to ArduPilot.

**`control/no_gps.parm`** — ArduPilot SITL parameter file:
```
GPS_TYPE 0    # disable GPS sensor
```
Loaded at SITL startup with `--add-param-file=control/no_gps.parm`. Parameters persist in SITL's `eeprom.bin` after first load.

### Effect on EKF

Without `"position"` and `"velocity"`, ArduPilot EKF3 receives:
- IMU (`imu.gyro`, `imu.accel_body`) — attitude + short-term dead reckoning
- Attitude (`"attitude"`) — direct yaw/roll/pitch reference
- Rangefinder (`"rng_1"`) — altitude AGL
- Barometer — simulated internally from last-known Aircraft altitude (static after position is dropped)
- Compass — synthesised from attitude + Earth field model (approx. correct for small area)

Expected EKF state: `ATT` (attitude valid) without `VEL_HORIZ` or `POS_ABS`. Horizontal position will drift — that is the correct no-GPS baseline before 6b-iii adds `VISION_POSITION_ESTIMATE`.

### Next step

6b-iii: send AnyLoc position estimates to ArduPilot EKF3 via `VISION_POSITION_ESTIMATE` MAVLink messages. This requires setting `EK3_SRC1_POSXY=6` (ExtNav) and `VISO_TYPE=1` in `no_gps.parm`.

---

## 2026-05-28 — 6b-ii velocity fix; multi-client TCP; EKF UNINIT root cause; 6b-iii wired up

### Bug fixed: `"velocity"` incorrectly removed in 6b-ii

Milestone 6b-ii had removed `"velocity"` from the bridge JSON alongside `"position"`. This caused ArduPilot to print "Failed to find key /velocity" and revert to "resending servos".

Root cause: `"velocity"` is `required=true` in ArduPilot's `SIM_JSON.h` keytable — omitting it causes `received_bitmask==0` and the entire packet is rejected. `"position"` is `required=false` and IS a GPS substitute; `"velocity"` is not (with `GPS_TYPE=0` it feeds only SITL's internal physics model and is never fused by EKF3 via GPS).

Fix: added `"velocity": list(vel_ned)` back to `_build_state()` with an explanatory comment.

---

### Bug fixed: only one MAVLink client could connect at a time

Both `run_mavlink.py` and `run_vision.py` connected to `tcp:localhost:5762`. TCP port 5762 accepts one client at a time — the second connection hung waiting for HEARTBEAT forever.

Fix: `run_vision.py` now connects to `tcp:localhost:5763`. SITL must be started with `--out tcp:localhost:5763` to open that port:

```bash
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild --console --map \
    -l 23.450868,120.286135,46,0 \
    --add-param-file=control/no_gps.parm \
    --out tcp:localhost:5763
```

| Script | Port | Role |
|--------|------|------|
| `run_mavlink.py` | `tcp:localhost:5762` | monitor |
| `run_vision.py` | `tcp:localhost:5763` | vision sender |

---

### Bug fixed: EKF reset to UNINIT every ~20 seconds

After reaching `ATT,VEL`, EKF flags would drop back to UNINIT (0x0400) approximately every 20 seconds.

Root cause: `EK3_SRC1_POSXY` defaults to **3 (GPS)**. With `GPS_TYPE=0`, EKF3 expects GPS position fusion but never receives any. After its internal timeout (~20 s) it declares the lane unhealthy and resets.

Fix: uncomment EK3 ExtNav params in `control/no_gps.parm`:
```
EK3_SRC1_POSXY  6   # horizontal position from ExternalNav
EK3_SRC1_VELXY  0   # no horizontal velocity source
EK3_SRC1_POSZ   1   # altitude from barometer
EK3_SRC1_YAW    1   # yaw from compass
VISO_TYPE       1   # enable MAVLink ExternalNav processing
```

With `EK3_SRC1_POSXY=6`, EKF3 expects ExtNav position (from `VISION_POSITION_ESTIMATE`) instead of GPS. Always restart SITL with `--wipe` after changing `no_gps.parm` to flush old `eeprom.bin`.

---

### `run_mavlink.py` display improvements

- **TIME column**: was `time.time() % 10000` (wall clock, never resets). Changed to `time.time() - t0` (seconds since monitor started, resets each run).
- **EKF label**: added `VEL_V` (bit 2) and `ALT` (bit 5 = `EKF_POS_VERT_ABS`) to `_ekf_label()`.

---

### Milestone 6b-iii: run_vision.py deployed, EK3 params set

`control/run_vision.py` is now wired up and `no_gps.parm` has the full ExtNav params. The pipeline is ready to test end-to-end:

1. SITL with `--out tcp:localhost:5763 --add-param-file=control/no_gps.parm --wipe`
2. Isaac Sim (`run_chiayi.sh`) or `stub_bridge.py`
3. `run_localizer.py` (writes `anyloc/latest_estimate.json`)
4. `run_vision.py` (reads estimate, sends `VISION_POSITION_ESTIMATE` at 5 Hz on port 5763)
5. `run_mavlink.py` (monitors EKF flags on port 5762)

Watch for `POS_ABS` (0x0010) in EKF flags to confirm EKF3 is fusing the vision position.

**Confirmed:** EKF flags reached `ATT,VEL_H,VEL_V,POS_REL,POS_ABS,ALT,PRED_ABS` — all flags healthy, vision position fully fused. Milestone 6b-iii done.

---

## 2026-05-29 — 6b-iv bug fixes: GPS failsafe, physics accuracy, stale estimate, debug tooling

### Bugs fixed

**1. GPS failsafe silently switches GUIDED→LAND after arming**

Root cause: `FS_GPS_ENABLE` is enabled by default. After force-arming with GPS bad fix, the failsafe fires within seconds and changes GUIDED → LAND. The TAKEOFF command arrives in LAND mode and is ignored — drone stays on the ground. Motors output landing throttle (~30 %), below hover threshold (~50 % mean PWM), so the kinematic model produces no upward thrust.

Fix: added to `control/no_gps.parm`:
```
FS_GPS_ENABLE   0   # prevent GPS failsafe GUIDED→LAND switch after arming
FENCE_ENABLE    0   # prevent geofence blocking flight near origin
```

**2. ARM rejected with FAILED (result=4) but force-arm not triggered**

Original code only triggered force-arm when `wait_command_ack` returned `None` (timeout). A `FAILED` result (4) returned immediately and bypassed force-arm entirely.

Fix: changed condition from `if result is None` to `if result != 0` — triggers force-arm on any non-zero MAV_RESULT (TEMPORARILY_REJECTED, DENIED, UNSUPPORTED, FAILED).

**3. EKF initialises at wrong position — stale estimate file**

Cause: `anyloc/latest_estimate.json` left over from a previous AnyLoc run. Old check was `if not os.path.exists(...)` — a 30-minute-old file would init EKF at (350 m N, 1352 m E) from home.

Fix: added age check — if file older than 10 seconds, overwrite with stub at home position.

**4. VisOdom not healthy at arm time**

`EKF_POS_ABS` fires on the very first VPE message, but `AP_VisualOdom::healthy()` requires a continuous 1-second window of VPE messages. Without waiting, the VisOdom pre-arm health check could still block arming.

Fix: added 3-second settle wait after EKF_POS_ABS — 3 s @ 5 Hz = 15 VPEs, well above the 1-second health window.

**5. HIGHRES_IMU "rate too fast" warning**

Requested 50 Hz equals `SCHED_LOOP_RATE` limit. ArduPilot logged a warning and may silently cap it.

Fix: reduced to 25 Hz (40 000 µs interval) in `mavlink_ctrl.py`.

**6. Gyro missing roll/pitch rates**

`sitl_bridge.py` sent `[0, 0, yaw_rate]` as the gyro vector. When the drone tilted, the EKF saw attitude changing (from the `attitude` field) but gyro showed no rotation — innovation mismatch, degraded EKF attitude tracking.

Fix: added `_prev_roll_rad` and `_prev_pitch_rad` state; compute p and q from finite difference alongside r. Gyro now sends `[roll_rate, pitch_rate, yaw_rate]`.

**7. Accel body frame — yaw-only rotation**

IMU specific force was rotated from NED to body using yaw only. At 20° tilt this introduced ≈12 % horizontal force error, causing wrong heading dynamics during autonomous flight.

Fix: full 3-axis DCM: R_bn = (R_z(yaw)·R_y(pitch)·R_x(roll))ᵀ

### Feature added: SITLBridge.debug_hz

New `debug_hz` property prints the physics state being sent at the specified rate. `stub_bridge.py` sets `bridge.debug_hz = 1.0` by default for cross-checking.

Sample output (stationary on ground):
```
[SITL] t=   3.12s  gyro p=+0.000 q=+0.000 r=+0.000 rad/s  accel bx=+0.00 by=+0.00 bz=-9.81 m/s²  vel N=+0.00 E=+0.00 D=+0.00 m/s  att r=+0.0° p=+0.0°  rng=0.10m
```

Cross-check: `accel bz ≈ −9.81` on ground confirms correct specific force sign convention. Compare `accel bz` against `Az` column in `run_mavlink.py` — should match within 0.05 m/s².

---

## 2026-05-29 — Milestone 6b-iv: flight command pipeline implemented

### mavlink_ctrl.py — new methods

| Method | Purpose |
|--------|---------|
| `set_mode(mode_name)` | Set ArduPilot flight mode by name ('GUIDED', 'RTL', 'LAND', …) |
| `wait_ekf_pos(timeout)` | Block until EKF_POS_HORIZ_ABS is set |
| `wait_command_ack(cmd_id, timeout)` | Block until COMMAND_ACK for cmd_id; returns MAV_RESULT |
| `wait_altitude(target_agl, tolerance, timeout)` | Block until LOCAL_POSITION_NED.z ≈ -target_agl |
| `wait_position(n, e, d, radius, timeout)` | Block until drone is within radius m of NED target |
| `is_armed` | True when HEARTBEAT base_mode has MAV_MODE_FLAG_SAFETY_ARMED |

COMMAND_ACK messages are now tracked in `recv()` via `self._last_ack[cmd_id] = result`.
Armed status is updated from every HEARTBEAT.

### stub_bridge.py — kinematic altitude model

Replaced static hover with a kinematic simulation:
- Drone starts on the ground (AGL = 0, z_abs = HOME_ELEV)
- Each step: `mean_pwm` of 4 motors → `thrust_norm` (0–1) → `thrust_accel` (0–2g)
- Net vertical acceleration: `GRAVITY - thrust_accel` (NED down)
- Integrates vertical velocity and altitude at 100 Hz
- Ground constraint: z_abs ≥ HOME_ELEV, vd clamped to ≤ 0 on contact

This lets ArduPilot arm and take off in SITL without Isaac Sim. Horizontal position stays at origin — full horizontal kinematics require Isaac Sim.

### run_flight.py — merged vision + flight

`run_vision.py` functionality merged into `run_flight.py` as a background thread:
- Vision thread: polls `anyloc/latest_estimate.json`, sends `VISION_POSITION_ESTIMATE` at 5 Hz
- Main thread: wait POS_ABS → GUIDED → arm → takeoff → waypoints → RTL → wait disarm
- Both share one `MAVLinkCtrl` on `tcp:localhost:5762` — no second TCP port needed
- If `latest_estimate.json` doesn't exist, a stub estimate at home is written automatically

`run_vision.py` kept as standalone alternative for vision-only testing.

SITL command simplified — `--out tcp:localhost:5763` no longer needed:
```bash
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild --console --map \
    -l 23.450868,120.286135,28.17,0 \
    --add-param-file=control/no_gps.parm --wipe
```

---

## 2026-05-30 — EKF origin fix; VisOdom health; confirmed first autonomous flight

### Bugs fixed

**1. `SET_GPS_GLOBAL_ORIGIN` never sent — root cause of all arming failures**

- Cause: `run_flight.py` connected and immediately started sending `VISION_POSITION_ESTIMATE`, but ArduPilot's EKF3 had no NED reference frame. Without a known origin, VPE messages cannot be anchored to absolute coordinates — EKF3 discards them, reports "EKF attitude is bad" and "VisOdom: not healthy", and blocks arming.
- Fix: added `set_ekf_origin()` and `set_home_position()` to `MAVLinkCtrl`; both are called right after `wait_heartbeat()` in `run_flight.py` and `run_vision.py`, before the vision thread starts.
- Confirmed: SITL console shows `EKF3 IMU0 origin set`, `EKF3 IMU1 origin set`, `Field Elevation Set: 28m` immediately after connection.

```python
# mavlink_ctrl.py — new methods
def set_ekf_origin(lat, lon, alt_msl_m)   # sends SET_GPS_GLOBAL_ORIGIN
def set_home_position(lat, lon, alt_msl_m) # sends SET_HOME_POSITION
```

**2. Regular arm FAILED even with `ARMING_CHECK 0` — VisOdom mandatory check**

- Cause: In ArduPilot 4.x+, the VisOdom health pre-arm check is mandatory when `EK3_SRC1_POSXY=6`. `ARMING_CHECK 0` does not bypass it. `AP_VisualOdom::healthy()` requires a continuous 1-second window of VPE messages — the previous fixed 3-second sleep was not tight enough to guarantee this.
- Fix: replaced the fixed sleep with `wait_visodom_healthy()` which polls `EKF_PRED_POS_HORIZ_ABS` (bit 9). This flag is set only when EKF3 is predicting future position from VPE, which implies `AP_VisualOdom::healthy()` is satisfied. Regular arm now succeeds without needing force arm.

```python
# mavlink_ctrl.py — new method
def wait_visodom_healthy(timeout=30.0)  # waits for EKF_POS_ABS | EKF_PRED_POS_ABS
```

### Confirmed flight output

```
AP: EKF3 IMU0 origin set
AP: EKF3 IMU1 origin set
AP: Field Elevation Set: 28m
AP: EKF3 IMU0 is using external nav data
AP: EKF3 IMU0 initial pos NED = 350.4,1351.6,0.0 (m)   ← stale AnyLoc estimate
ARMED
AP: EKF3 IMU0 MAG0 in-flight yaw alignment complete

[Flight] EKF POS_ABS ✓
[Flight] VisOdom healthy ✓
[Flight] Armed ✓
[Flight] Takeoff → 10.0 m AGL …
[Flight] Reached 10.0 m AGL ✓
[Flight] WP 1/4  N=+20 E=+0 ALT=10 m AGL
```

**Note on initial NED offset (350.4, 1351.6):** The first VPE sent was a stale `anyloc/latest_estimate.json` from a previous AnyLoc run (position was not at home). Delete or overwrite this file before each test to ensure EKF initialises at NED (0, 0, 0).

### Files modified

| File | Change |
|------|--------|
| `control/mavlink_ctrl.py` | Added `set_ekf_origin()`, `set_home_position()`, `wait_visodom_healthy()` |
| `control/run_flight.py` | Calls `set_ekf_origin` + `set_home_position` after heartbeat; replaces 3 s sleep with `wait_visodom_healthy()` |
| `control/run_vision.py` | Same origin/home calls added; stale `HOME_ALT_MSL=46.0` → `28.17` |
| `README.md`, `project_plan.md`, `history.md` | SITL `-l` altitude placeholder `<centre_elev>`/`46` → `28.17` throughout |

---

## 2026-05-30 — Milestone 6e: ROS2 migration (all IPC via topics + MAVROS2)

### Motivation

All previous inter-process communication was file-based (JPEG frames + JSON estimates) or raw sockets (pymavlink TCP). This introduced polling latency, file-write race conditions, and non-standard interfaces. ROS2 pub/sub eliminates polling, provides introspectability (`ros2 topic echo`), and matches the standard deployment interface for real hardware.

### Environment

- **ROS2 Jazzy** already installed at `/opt/ros/jazzy` (Ubuntu 24.04)
- **MAVROS2 2.14.0** already installed (`ros-jazzy-mavros`, `ros-jazzy-mavros-extras`)
- **vision_msgs 4.1.1** already installed (`ros-jazzy-vision-msgs`)
- **rclpy** uses Python 3.12 — same as Isaac Sim 6.0 — so system rclpy is used directly inside Isaac Sim by adding `/opt/ros/jazzy/lib/python3.12/site-packages` to `sys.path`

### New ROS2 topic map

| Topic | Type | Publisher | Subscriber(s) |
|-------|------|-----------|---------------|
| `/drone/camera/image_raw` | `sensor_msgs/Image` (rgb8) | Isaac Sim | AnyLoc node, YOLO node |
| `/drone/pose` | `geometry_msgs/PoseStamped` (frame=wgs84, pos=lat/lon/alt) | Isaac Sim | AnyLoc node, YOLO node |
| `/drone/agl` | `std_msgs/Float64` | Isaac Sim | AnyLoc node |
| `/anyloc/pose_estimate` | `geometry_msgs/PoseWithCovarianceStamped` | AnyLoc node | (mission planner) |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` (frame=map, NED) | AnyLoc node | MAVROS2 → `VISION_POSITION_ESTIMATE` |
| `/yolo/detections` | `vision_msgs/Detection2DArray` | YOLO node | (mission planner) |
| `/mavros/state` | `mavros_msgs/State` | MAVROS2 | Flight commander |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | MAVROS2 | Flight commander |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | Flight commander | MAVROS2 → `SET_POSITION_TARGET` |

### Files created

| File | Purpose |
|------|---------|
| `simulator/cesium_scene.py` (modified) | Publishes `/drone/camera/image_raw`, `/drone/pose`, `/drone/agl` via system rclpy; falls back to file output if ROS2 unavailable |
| `simulator/run_chiayi.sh` (modified) | Sources `/opt/ros/jazzy/setup.bash` before `conda run` so ROS2 shared libs are on `LD_LIBRARY_PATH` |
| `anyloc/ros2_node.py` | rclpy node: subscribes to camera + pose → runs AnyLoc+VO → publishes to `/anyloc/pose_estimate` and `/mavros/vision_pose/pose` |
| `detection/ros2_node.py` | rclpy node: subscribes to camera → runs YOLOv8 → publishes to `/yolo/detections` |
| `control/launch_mavros.sh` | Starts MAVROS2 connected to SITL `tcp:localhost:5762` |
| `control/flight_commander.py` | rclpy node: GUIDED → arm → takeoff → waypoints → RTL via MAVROS2 services/topics |

### Architecture decision: pymavlink for EKF origin only

MAVROS2 Jazzy 2.14 has no `/mavros/global_position/set_gp_origin` service. `flight_commander.py` uses a thin pymavlink call only for `SET_GPS_GLOBAL_ORIGIN` + `SET_HOME_POSITION` at startup, then hands off to MAVROS2 for everything else.

### Legacy files kept (non-ROS2 fallback)

`anyloc/run_localizer.py`, `detection/run_detector.py`, `control/run_flight.py`, `control/run_vision.py`, `control/run_mavlink.py`, `control/mavlink_ctrl.py` — all kept as file-based / pymavlink fallbacks. Remove when ROS2 pipeline is validated on hardware.

### Run order (ROS2 mode)

```bash
# Terminal 1 — SITL
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild --console --map \
    -l 23.450868,120.286135,28.17,0 --add-param-file=control/no_gps.parm

# Terminal 2 — physics bridge (or Isaac Sim)
python3 control/stub_bridge.py

# Terminal 3 — MAVROS2
bash control/launch_mavros.sh

# Terminal 4 — AnyLoc ROS2 node
source /opt/ros/jazzy/setup.bash && python3 anyloc/ros2_node.py

# Terminal 5 — YOLO ROS2 node (optional)
source /opt/ros/jazzy/setup.bash && python3 detection/ros2_node.py

# Terminal 6 — Isaac Sim (publishes camera + pose topics)
cd simulator && ./run_chiayi.sh

# Terminal 7 — Flight commander
source /opt/ros/jazzy/setup.bash && python3 control/flight_commander.py
```

---

## 2026-05-31 — Remove pymavlink; MAVROS2 raw MAVLink for EKF origin + status; two-phase VPE

### What was done

**Removed all pymavlink dependencies from `flight_commander.py`**

The old code used pymavlink on UDP 14551 for three things: setting the EKF global origin, monitoring EKF status flags, and reading altitude during takeoff. All three are replaced by MAVROS2 infrastructure:

- **EKF origin**: publish `GeoPointStamped` to `/mavros/global_position/set_gp_origin`. Confirmed by monitoring GPS_GLOBAL_ORIGIN (msg 49) on `/uas1/mavlink_source` with BEST_EFFORT QoS. No extra UDP port required — MAVROS2's global_position plugin forwards to ArduPilot.

- **EKF status**: read EKF_STATUS_REPORT (msg 193) from `/uas1/mavlink_source`. Flags decoded at **byte offset 20** (after 5 floats × 4 bytes). `/mavros/estimator_status` is advertised in MAVROS2 Jazzy 2.14 but publishes no messages at a useful rate — confirmed by `ros2 topic echo` producing no output. The `/uas1/mavlink_source` approach works.

- **Altitude**: already reading `/mavros/local_position/pose` (was the case since 6f/6g).

- **Motor PWM**: also decoded from SERVO_OUTPUT_RAW (msg 36) via `_cb_mavlink` and printed alongside each AGL line during takeoff for diagnostics.

**Why TCP 5760 (MAVProxy master) cannot be used:**
ArduPilot SITL's TCP 5760 only serves one client (MAVProxy). Additional connections are accepted at the TCP level but receive no MAVLink data. Confirmed by pymavlink `wait_heartbeat` timing out despite a successful TCP socket connect.

**Two-phase VPE strategy**

The VPE thread now uses altitude-dependent covariance and position:
- **Phase 1 (below 50 m AGL):** position = home (east=0, north=0), cov_xy = 0.1 m². EKF sets POS_HORIZ_ABS immediately because the drone IS at the known home position on the ground.
- **Phase 2 (above 50 m AGL):** position = AnyLoc estimate from `latest_estimate.json`, cov_xy = max(1.0, error_m²). Only estimates with `agl_m >= 50` accepted (rejects ground-level stubs).

**`launch_mavros.sh` updated:**
Only `--out udp:127.0.0.1:14550` needed in the SITL command. The `--out udp:127.0.0.1:14551` line is removed.

### Key diagnostic findings from debugging session

| Finding | Detail |
|---------|--------|
| `/uas1/mavlink_source` QoS | Publisher uses BEST_EFFORT — subscription must match |
| EKF_STATUS_REPORT flags offset | Byte 20 (not 0) — after 5 floats (velocity_variance, pos_horiz_variance, pos_vert_variance, compass_variance, terrain_alt_variance) |
| GPS_GLOBAL_ORIGIN msg ID | 49 — only echoed when EKF successfully accepts the origin |
| SERVO_OUTPUT_RAW struct | 4 uint16 motors at byte offset 4 (after uint32 time_usec) |
| "Mode change to Guided failed: requires position" | MAVROS2 returns success but ArduPilot silently rejects — indicates EKF flags=0x000 (degraded SITL state) |
| SITL degradation pattern | After 180s failed takeoff, EKF flags drop to 0x000; GPS_GLOBAL_ORIGIN no longer echoed; must restart SITL + drone_sim + MAVROS2 |

### Status

Arming pipeline fully working: connect → EKF origin confirmed → STABILIZE arm → GUIDED → EKF POS_ABS → NAV_TAKEOFF accepted. Takeoff (actual climb) is **not yet working** — motors read from SERVO_OUTPUT_RAW during the climb will be printed in the next run to determine whether ArduPilot is commanding throttle.

### Files modified

| File | Change |
|------|--------|
| `control/flight_commander.py` | Removed pymavlink; added `/uas1/mavlink_source` subscription; `set_ekf_origin()` via GeoPointStamped + GPS_GLOBAL_ORIGIN confirmation; `wait_ekf_pos()` via EKF_STATUS_REPORT flags; two-phase VPE; motor PWM logging |
| `control/launch_mavros.sh` | Updated comments: only `--out udp:127.0.0.1:14550` needed |

---

## 2026-05-31 — flight_commander.py: dead code removed, cleanup fixes

### Bugs fixed

**1. `AltMonitor` class defined but never used**

- Cause: `AltMonitor` (a persistent pymavlink thread exposing live AGL) was created as a planned helper for the VPE thread, but `takeoff()` ended up with its own inline pymavlink connection for altitude polling. The class was left as dead code.
- Fix: deleted the class entirely (~30 lines).
- File: `control/flight_commander.py`

**2. Takeoff failure path missing cleanup**

- Cause: when `takeoff()` returns False, the code called `rclpy.shutdown()` and returned, but did not call `stop_ev.set()` or `cmd.destroy_node()`. Every other failure path (MAVROS2 not connected, EKF timeout) calls all three. The VPE daemon thread was left running and the node was not destroyed.
- Fix: added `stop_ev.set(); cmd.destroy_node()` before `rclpy.shutdown()` in the takeoff failure branch — matching all other failure paths.
- File: `control/flight_commander.py` → `main()` Step 7

**3. RTL disarm timeout too short**

- Cause: `_spin_until(lambda: not cmd._state.armed, timeout=60.0)` — the drone takes off to 90 m AGL and descends at ~1–1.5 m/s during RTL, which takes ~60–90 s to descend plus landing time. The 60 s timeout would expire during descent.
- Fix: increased to 150 s to cover the full 90 m descent + landing sequence.
- File: `control/flight_commander.py` → Step 9

---

## 2026-05-30 — ROS2 node bugs fixed; postview added; dual file+ROS2 output

### Bugs fixed

**1. `ros2_node.py` crashed with `ModuleNotFoundError: faiss`**

- Cause: run command was `source /opt/ros/jazzy/setup.bash && python3 anyloc/ros2_node.py`, which uses system Python 3. System Python has rclpy but not faiss, torch, or PIL (those are in `isaac_sim_test` conda env).
- Fix: run with `conda run -n isaac_sim_test python3` so ML libraries are available. Add `/opt/ros/jazzy/lib/python3.12/site-packages` to `sys.path` inside the script so rclpy is importable from the conda env. Same fix applied to `detection/ros2_node.py` and `control/flight_commander.py`.
- New launch script: `anyloc/run_ros2_localizer.sh` — sources ROS2 then calls `conda run -n isaac_sim_test`.

**2. `VORefiner.update()` called with wrong arguments**

- Cause: `ros2_node.py` was calling `self._vo.update(prev_bgr, curr_bgr, agl_m, yaw_rad)` — passing two BGR numpy arrays, 4 positional args, and yaw in radians.
- Actual signature: `update(self, frame_pil: PIL.Image, agl_m: float, yaw_deg: float)` — takes a single PIL image (stores previous frame internally), and expects yaw in degrees.
- Fix: `dlat, dlon, _ = self._vo.update(pil_img, agl_m, math.degrees(self._drone_yaw))`

**3. `cesium_scene.py` stopped writing files when ROS2 was available**

- Cause: ROS2 publish and file write were in an if/else — when `_ros2_node is not None`, files were never written, so `run_localizer.py` (legacy, polls files by mtime) saw no new frames and stayed stuck at starting position.
- Fix: always write files unconditionally; publish to ROS2 on top when available (dual output).

### Feature added: `ros2_node.py` postview

`anyloc/ros2_node.py` now includes the same dual-window matplotlib postview as `run_localizer.py`:
- Left panel: live drone camera with LAT/LON/ALT/AGL/YAW overlay
- Right panel: AnyLoc satellite match crop with ERR/score/VO pts; green < 200 m, blue otherwise

ROS2 spin runs in a daemon background thread; matplotlib owns the main thread.
Node also writes `anyloc/latest_estimate.json` on each anchor for legacy `run_flight.py` compatibility.

| File | Change |
|------|--------|
| `anyloc/ros2_node.py` | Added sys.path ROS2 fix; fixed VORefiner.update() call; added full postview |
| `anyloc/run_ros2_localizer.sh` | New launch script: `source /opt/ros/jazzy/setup.bash && conda run -n isaac_sim_test python3 anyloc/ros2_node.py` |
| `detection/ros2_node.py` | Added sys.path ROS2 fix |
| `control/flight_commander.py` | Added sys.path ROS2 fix |
| `simulator/cesium_scene.py` | Always writes files; publishes ROS2 on top (dual output) |
