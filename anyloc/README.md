# AnyLoc — Build & Run Guide (Linux)

Visual place recognition for GPS-denied drone navigation.  
Uses **DINOv2** patch features + **VLAD** aggregation + **FAISS** nearest-neighbour search against a geo-tagged satellite image database.

---

## Requirements

| Component | Version | Notes |
|---|---|---|
| OS | Ubuntu 22.04 / 24.04 | Tested on 24.04 |
| Python | 3.10 – 3.12 | Via conda `isaac_sim_test` env |
| PyTorch | ≥ 2.0 | Pre-installed by Isaac Sim |
| torchvision | ≥ 0.15 | Pre-installed by Isaac Sim |
| Pillow | ≥ 9.0 | |
| NumPy | ≥ 1.24 | |
| faiss-cpu | ≥ 1.7 | Install via conda-forge |
| OpenCV (cv2) | ≥ 4.7 | Pre-installed by Isaac Sim |
| requests | any | For NLSC tile download |
| ROS2 Jazzy | — | Required for `ros2_node.py` only |
| MAVROS2 | — | Required for VPE publishing only |

---

## 1. Conda Environment Setup

The project uses the `isaac_sim_test` conda environment created by NVIDIA Isaac Sim.
If you are running without Isaac Sim, create the environment manually:

```bash
conda create -n isaac_sim_test python=3.10 -y
conda activate isaac_sim_test
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install pillow numpy requests opencv-python
```

### Install faiss-cpu

```bash
conda install -n isaac_sim_test -c conda-forge faiss-cpu -y
```

> `pip install faiss-cpu` is an alternative but the conda-forge build is
> more reliable on Linux.

---

## 2. Build the Image Database

The database must be built **once** before running the localizer.
It downloads Taiwan NLSC satellite orthophoto tiles and encodes them as VLAD vectors.

Run from the **project root** (not from inside `anyloc/`):

```bash
conda run -n isaac_sim_test python anyloc/build_database.py
```

### Build options

| Flag | Default | Description |
|---|---|---|
| `--grid-step N` | 50 | Grid spacing in metres |
| `--agl-min N` | 60 | Minimum AGL altitude in metres |
| `--agl-max N` | 120 | Maximum AGL altitude in metres |
| `--agl-step N` | 5 | AGL increment in metres |
| `--rebuild` | off | Overwrite an existing database |

Example — finer grid, wider altitude range:

```bash
conda run -n isaac_sim_test python anyloc/build_database.py \
    --grid-step 25 --agl-min 50 --agl-max 150 --agl-step 10
```

### What the build does

1. **Downloads** Taiwan NLSC PHOTO2 orthophoto tiles at zoom 18 from
   `wmts.nlsc.gov.tw` and stitches them into a mosaic JPEG.
2. **Crops** satellite patches for each (lat, lon, AGL) grid point simulating
   a nadir drone camera (90° × 73.7° FOV).
3. **Encodes** each crop with DINOv2 ViT-B/14 (downloaded from torch hub on
   first run — requires internet).
4. **Clusters** patch descriptors into a VLAD codebook (k = 64) with FAISS k-means.
5. **Saves** the database to `anyloc/database/`.

> **First run** downloads ~400 MB of DINOv2 weights from torch hub.
> Subsequent runs reuse the cache (`~/.cache/torch/hub/`).

### Output files

```
anyloc/database/
├── database.pt          # main file: lats, lons, alts, codebook
├── database_meta.pt     # entry metadata
├── database_vlads.pt    # VLAD vectors (N × 512)
├── db_meta.json         # build cache — skip re-download if present
└── db_images/           # satellite crop JPEGs (one per grid point)
```

---

## 3. Run the Localizer (standalone)

For testing outside of ROS2:

```bash
conda run -n isaac_sim_test python anyloc/run_localizer.py
```

---

## 4. Accuracy Benchmark (Esri World Imagery)

`test_accuracy_esri.py` measures localizer accuracy against known ground-truth
coordinates. It fetches **Esri World Imagery** tiles (no API key required),
feeds each image to AnyLoc, and reports the Euclidean error in metres between
the true and estimated position.

**Build the database first** (Section 2), then run from the project root:

```bash
conda run -n isaac_sim_test python anyloc/test_accuracy_esri.py
```

### Options

| Flag | Default | Description |
|---|---|---|
| `--samples N` | 20 | Number of random test points |
| `--agl N` | 80 | Drone altitude AGL in metres. Pass `0` to randomise across [60, 120] m |
| `--seed N` | 42 | Random seed for reproducible test points |
| `--output FILE` | — | Save full results to a JSON file |
| `--plot` | off | Show error histogram and spatial error map |

Examples:

```bash
# Quick 10-point check at 80 m AGL
conda run -n isaac_sim_test python anyloc/test_accuracy_esri.py --samples 10

# 50 samples, mixed altitudes, save results, show plot
conda run -n isaac_sim_test python anyloc/test_accuracy_esri.py \
    --samples 50 --agl 0 --output results.json --plot
```

### Sample output

```
  #    True lat    True lon     Est lat     Est lon   Err (m)   Score  AGL
  -----------------------------------------------------------------------
   1  23.447201  120.282031  23.447650  120.282500      63.2   0.821  80 m
   2  23.453819  120.290174  23.453400  120.289800      52.7   0.847  80 m
  ...

  Mean error   :    58.40 m
  Median error :    55.10 m
  RMSE         :    64.20 m
  Std dev      :    26.80 m
```

---

## 5. Run the ROS2 Node

The ROS2 node publishes pose estimates to MAVROS2 and writes
`anyloc/latest_estimate.json`.

**Prerequisites:** SITL + MAVROS2 must be running first (see `run.sh`).

```bash
# Option A — convenience script (sets DISPLAY and sources ROS2)
bash anyloc/run_ros2_localizer.sh

# Option B — manual
source /opt/ros/jazzy/setup.bash
DISPLAY=:2 conda run -n isaac_sim_test --no-capture-output \
    python3 -u anyloc/ros2_node.py
```

### ROS2 topics

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/drone/camera/image_raw` | `sensor_msgs/Image` (rgb8, 640×480) |
| Subscribe | `/drone/pose` | `geometry_msgs/PoseStamped` |
| Subscribe | `/drone/agl` | `std_msgs/Float64` (metres AGL) |
| Publish | `/anyloc/pose_estimate` | `geometry_msgs/PoseWithCovarianceStamped` |
| Publish | `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` |

---

## 6. Full System Launch

See the project root `run.sh` for the full tmux-based launch sequence
(SITL → MAVROS2 → flight commander → AnyLoc → YOLO).

```bash
bash run.sh --tmux
```

---

## Troubleshooting

**`ImportError: No module named 'faiss'`**
→ Install faiss-cpu: `conda install -n isaac_sim_test -c conda-forge faiss-cpu`

**`RuntimeError: CUDA out of memory` or slow encoding**
→ DINOv2 runs on CPU by default. Ensure PyTorch is installed with CUDA if you
have a GPU and want faster database builds.

**Database build fails on tile download**
→ Check internet access to `wmts.nlsc.gov.tw`. The script retries 3 times per
tile and skips on persistent failure.

**`latest_estimate.json` shows stale data**
→ The localizer only updates estimates when AGL ≥ 50 m. The file is written as
a stub at startup and updated once the altitude guard is satisfied.
