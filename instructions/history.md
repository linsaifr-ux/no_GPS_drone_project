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

## Next session — Milestone 4 / 5

- Add YOLO detection module in `detection/` (reads same `drone_frames/latest.jpg`)
- Show detection bounding-box overlay as a third postview window
- Connect AnyLoc estimate + YOLO detections into `main.py` orchestrator
