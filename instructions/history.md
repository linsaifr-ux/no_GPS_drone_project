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
    -l 23.450868,120.286135,46,0 \
    --add-param-file=control/no_gps.parm --wipe
```
