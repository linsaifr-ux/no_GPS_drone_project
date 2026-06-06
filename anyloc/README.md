# anyloc/ — Visual Localisation (GPS-Denied)

Visual place recognition for GPS-denied drone navigation.  
Uses **DINOv2** patch features + **VLAD** aggregation + **FAISS** nearest-neighbour search against a geo-tagged satellite image database.

In the full pipeline, AnyLoc provides Phase 2 VPE (visual position estimates) when the drone is above 50 m AGL. Below that threshold the flight commander uses kinematic truth as Phase 1 VPE. AnyLoc estimates are published via `/mavros/vision_pose/pose_cov` with covariance proportional to retrieval error, so the EKF weights them appropriately.

---

## Requirements

| Component | Version | Notes |
|---|---|---|
| OS | Ubuntu 22.04 / 24.04 | Tested on 24.04 |
| Python | 3.10–3.12 | Via conda `isaac_sim_test` env |
| PyTorch | ≥ 2.0 | Pre-installed by Isaac Sim |
| torchvision | ≥ 0.15 | Pre-installed by Isaac Sim |
| Pillow | ≥ 9.0 | |
| NumPy | ≥ 1.24 | |
| faiss-cpu | ≥ 1.7 | Install via conda-forge |
| OpenCV (cv2) | ≥ 4.7 | Pre-installed by Isaac Sim |
| requests | any | For NLSC tile download |
| ROS2 Jazzy | — | Required for `ros2_node.py` |
| MAVROS2 | — | Required for VPE publishing |

---

## 1. Conda Environment Setup

Uses the `isaac_sim_test` conda environment created by Isaac Sim. For standalone use without Isaac Sim:

```bash
conda create -n isaac_sim_test python=3.10 -y
conda activate isaac_sim_test
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install pillow numpy requests opencv-python
conda install -n isaac_sim_test -c conda-forge faiss-cpu -y
```

---

## 2. Build the Image Database

Build once before running the localizer. Downloads Taiwan NLSC satellite tiles and encodes them as VLAD vectors.

Run from the **project root**:

```bash
conda run -n isaac_sim_test python anyloc/build_database.py
```

### Build options

| Flag | Default | Description |
|---|---|---|
| `--grid-step N` | 50 | Grid spacing in metres |
| `--agl-min N` | 60 | Minimum AGL altitude |
| `--agl-max N` | 120 | Maximum AGL altitude |
| `--agl-step N` | 5 | AGL increment |
| `--rebuild` | off | Overwrite existing database |

### What the build does

1. Downloads Taiwan NLSC PHOTO2 orthophoto tiles at zoom 18 and stitches them into a mosaic
2. Crops satellite patches for each (lat, lon, AGL) grid point simulating a nadir drone camera
3. Encodes each crop with DINOv2 ViT-B/14 (~400 MB download on first run)
4. Clusters patch descriptors into a VLAD codebook (k=64) with FAISS k-means
5. Saves to `anyloc/database/`

### Output

```
anyloc/database/
├── database.pt          # lats, lons, alts, codebook
├── database_meta.pt     # entry metadata
├── database_vlads.pt    # VLAD vectors (N × 512)
└── db_meta.json         # build cache (skip re-download if present)
```

Current database: **36 673 entries**, 50 m grid, AGL 60–120 m.

---

## 3. Run the Localizer (standalone)

```bash
conda run -n isaac_sim_test python anyloc/run_localizer.py
```

---

## 4. Accuracy Benchmark (Esri World Imagery)

Measures localizer accuracy at known ground-truth coordinates using Esri imagery.

```bash
conda run -n isaac_sim_test python anyloc/test_accuracy_esri.py
```

| Flag | Default | Description |
|---|---|---|
| `--samples N` | 20 | Number of random test points |
| `--agl N` | 80 | AGL in metres (0 = randomise 60–120 m) |
| `--seed N` | 42 | Random seed |
| `--output FILE` | — | Save results to JSON |
| `--plot` | off | Show error histogram + spatial map |

Typical results at 80 m AGL: mean ~58 m, RMSE ~64 m.

---

## 5. Constrained-Search Benchmark

Measures accuracy and speed of the **anchor-chain constrained search** used in `ros2_node.py`. Instead of searching all 36k entries, each retrieval considers only DB entries within 200 m of the previous estimate.

```bash
conda run -n isaac_sim_test python anyloc/test_accuracy_constrained.py
```

| Flag | Default | Description |
|---|---|---|
| `--steps N` | 20 | Trajectory steps |
| `--agl N` | 80 | AGL in metres |
| `--radius N` | 200 | Search radius in metres |
| `--seed N` | 42 | Random seed |
| `--output FILE` | — | Save results to JSON |
| `--plot` | off | Show per-step error and latency charts |

Typical speedup: **~4×** faster vs global search; constrained RMSE lower than global (anchor eliminates far-away false positives).

---

## 6. Run the ROS2 Node (full pipeline)

The ROS2 node processes camera frames, runs AnyLoc retrieval, and publishes VPE to MAVROS.

**Prerequisites:** the autopilot SITL and MAVROS2 must be running first (see `run.sh`).

```bash
bash anyloc/run_ros2_localizer.sh
```

Or manually:
```bash
source /opt/ros/jazzy/setup.bash
DISPLAY=:2 conda run -n isaac_sim_test --no-capture-output python3 -u anyloc/ros2_node.py
```

### ROS2 topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `/drone/camera/image_raw` | `sensor_msgs/Image` | rgb8, 640×480 |
| Subscribe | `/drone/pose` | `geometry_msgs/PoseStamped` | WGS84 (lat, lon, alt_msl) |
| Subscribe | `/drone/agl` | `std_msgs/Float64` | AGL in metres |
| Publish | `/anyloc/pose_estimate` | `geometry_msgs/PoseWithCovarianceStamped` | AnyLoc estimate (monitoring) |

**VPE to MAVROS is not published by this node.** `px4_commander.py` reads `latest_estimate.json` and publishes `/mavros/vision_pose/pose_cov` with correct per-axis covariance. Publishing from both processes caused duplicate EKF2 inputs.

### VO yaw convention

The VO refiner (`VORefiner`) expects a `yaw_deg` argument equal to the compass bearing of the camera's image-top direction (0 = North, 90 = East). Because the camera gimbal preserves drone yaw (top of image = drone nose), this equals the drone's compass bearing:

```python
_vo_yaw = -math.degrees(self._drone_yaw)   # _drone_yaw = −_kyaw_rad (NED CW)
```

In simulation (`_kyaw_rad = 0`, drone faces North): `_vo_yaw = 0` — matches VORefiner's North-pointing convention. On real hardware with drone yaw, the formula holds automatically.

### latest_estimate.json format

```json
{
  "est_lat": 23.4512,
  "est_lon": 120.2847,
  "yaw_deg": 0.0,
  "agl_m": 82.3,
  "error_m": 55.1,
  "timestamp": 1748991234.5
}
```

> **Note on `yaw_deg`:** `/drone/pose` encodes orientation as `qz = sin(−_kyaw_rad / 2)` (should be `π/2 − _kyaw_rad`), so `yaw_deg = 0.0` for a North-facing drone — a 90° encoding error. `px4_commander.py` ignores this field and hardcodes ENU yaw = π/2 (North) for VPE in both phases.

---

## VPE Integration with Flight Commander

`px4_commander.py` reads `latest_estimate.json` in its vision thread:
- Phase 1 (AGL < `MIN_LOCALISATION_AGL` = 50 m): sends kinematic truth, cov = 0.1 m²
- Phase 2 (AGL ≥ 50 m): sends AnyLoc estimate, cov = max(1, error_m²)

The covariance difference lets PX4's EKF2 automatically weight the two sources: tight covariance on ground truth during climb, loose covariance on AnyLoc during cruise.

---

## Troubleshooting

**`ImportError: No module named 'faiss'`**  
→ `conda install -n isaac_sim_test -c conda-forge faiss-cpu`

**`RuntimeError: CUDA out of memory`**  
→ DINOv2 runs on CPU by default. GPU: ensure PyTorch+CUDA is installed.

**Database build fails on tile download**  
→ Check connectivity to `wmts.nlsc.gov.tw`. Script retries 3× per tile; persists on failure.

**`latest_estimate.json` not updating**  
→ The node only publishes when AGL ≥ 50 m. The file is written as a stub at startup; the VPE guard requires the altitude threshold to be satisfied.
