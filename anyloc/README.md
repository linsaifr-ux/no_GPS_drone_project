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

## 5. Constrained-Search Benchmark (no VO)

`test_accuracy_constrained.py` measures the accuracy and speed benefit of the
**anchor-chain constrained search** used in `ros2_node.py` — in isolation from
the VO layer.

In the live pipeline, AnyLoc restricts each retrieval to DB entries within
`SEARCH_RADIUS_M = 200 m` of the previous frame's estimate instead of
searching all 36 K entries.  This test runs **both** modes (global FAISS and
constrained) on every step of a simulated linear flight trajectory so the
results are directly comparable.

**Build the database first** (Section 2), then run from the project root:

```bash
conda run -n isaac_sim_test python anyloc/test_accuracy_constrained.py
```

### Options

| Flag | Default | Description |
|---|---|---|
| `--steps N` | 20 | Number of trajectory steps |
| `--agl N` | 80 | Drone altitude AGL in metres. Pass `0` to randomise across [60, 120] m |
| `--radius N` | 200 | Constrained search radius in metres (matches `SEARCH_RADIUS_M` in `ros2_node.py`) |
| `--seed N` | 42 | Random seed for trajectory generation |
| `--output FILE` | — | Save full results to a JSON file |
| `--plot` | off | Show per-step error & latency charts and spatial error map |
| `--no-viewport` | off | Disable the live three-panel image window |

Examples:

```bash
# Quick 10-step run at 80 m AGL
conda run -n isaac_sim_test python anyloc/test_accuracy_constrained.py --steps 10

# 30 steps, save results, show plots, no live window
conda run -n isaac_sim_test python anyloc/test_accuracy_constrained.py \
    --steps 30 --agl 80 --radius 200 --output results_constrained.json \
    --plot --no-viewport
```

### Benchmark methodology

#### Background — the optimization being tested

The live localization node (`ros2_node.py`) runs at two timescales:

- **Every frame** — VO (LK optical flow) accumulates small `(Δlat, Δlon)` deltas from the previous anchor.
- **Every `ANYLOC_INTERVAL = 10` frames** — AnyLoc performs a full VLAD retrieval and resets the anchor.

When an anchor exists, the AnyLoc retrieval is not a full global search.  It
only considers DB entries within `SEARCH_RADIUS_M = 200 m` of
`anchor + VO_accumulated_delta`.  This reduces the candidate set from ~36 K
entries to a few hundred, cutting inference time and eliminating false-positive
matches from distant parts of the scene.

This test isolates the **search constraint alone** (no VO contribution) by
using the previous step's AnyLoc estimate directly as the center of the next
step's search window.

#### Trajectory design

The test generates a **linear flight path** across the scene rather than random
independent points.  This matters because:

- Consecutive positions are spatially correlated, exactly as in real flight.
- The inter-step distance (total path length ÷ N steps) controls how fast the
  anchor drifts relative to the search radius.
- A random scatter of independent points would never exercise the anchor-chain;
  every step would be a cold start.

The start and end of the path are drawn randomly within 30–70 % of `RADIUS_M`
from the scene centre, on opposite sides, giving a diagonal transect of
roughly 1–2 km.

#### Per-step execution

For each trajectory step both modes run on the **same Esri image**:

| | Global search | Constrained search |
|---|---|---|
| **Step 0 (cold start)** | full FAISS `IndexFlatIP` over all N entries | same as global (no anchor yet) |
| **Steps 1 … N** | full FAISS search | dot-product over the subset of DB entries within `--radius` m of the previous step's constrained estimate |

The constrained dot-product uses torch tensors directly:

```
dlat = (all_lats − anchor_lat) × 111 320
dlon = (all_lons − anchor_lon) × 111 320 × cos(lat)
in_range = (dlat² + dlon²) ≤ radius²
sims = vlads[in_range] @ query_vlad
best = argmax(sims)
```

If `in_range` is empty (anchor has drifted far outside the database coverage),
the full index is used as a fallback — the same safety net as in `ros2_node.py`.

The **anchor updates from the constrained result** after every step.  This
propagates any retrieval error forward, simulating realistic anchor drift
without VO to correct it.  It is the **hardest case** for constrained search:
any single bad match degrades every subsequent step.

#### The `in_window` flag

`InWin = Y` means the true position was within `--radius` metres of the anchor
used for that step's constrained search.  When `InWin = N`:

- The window has drifted far enough that the correct DB entry is outside the
  candidate set.
- Constrained search must rely on the fallback (full index) and will likely
  return a worse score than usual.
- Persistent `N` values indicate the step size is too large for the chosen
  radius, or that a bad retrieval at an earlier step sent the anchor off course.

`in_window_pct` in the aggregate summary is the primary health metric for the
constraint: values above ~90 % indicate the anchor-chain is stable.

#### Interpreting the results

**RMSE vs mean error:**
RMSE (Root Mean Square Error) squares each per-step error before averaging,
so a single badly-drifted step raises it far more than it raises the mean.
It is the primary accuracy metric because it is sensitive to the outlier steps
that matter most in practice — a localizer that is usually good but occasionally
wildly wrong is dangerous for flight control.  Mean error masks those outliers.

Concretely: `RMSE = sqrt( mean( error_m² ) )`.  A constrained RMSE lower than
the global RMSE means the anchor-chain not only improves typical accuracy but
also reduces catastrophic mismatches.

**Accuracy (`Err_const − Err_glob`):**
- Negative delta (constrained < global) — restricting the search eliminated
  far-away false positives; the closest correct match won.
- Near-zero delta — the global search already found the correct match; the
  constraint helped speed but not accuracy.
- Positive delta — the anchor has drifted and the correct entry was outside
  the window; the constrained search settled for a sub-optimal candidate.

**Speed (`Speedup`):**
- Step 0 always shows ~1× (both modes run full search).
- Steps 1+ typically show 3–6× speedup for a 200 m radius at 50 m grid
  spacing, because ~200 m radius × (π) ÷ (50 m grid)² ≈ 25 candidates vs
  36 K total.
- Speedup degrades if `--radius` is large relative to the scene (more
  candidates) or if the FAISS index is already in L2 cache.

#### Relationship to the full pipeline

In production (`ros2_node.py`), the window center is
`anchor + VO_accumulated_delta`, not just `anchor`.  VO reduces the effective
anchor age from 10 frames to near-zero between re-anchors, so the true
position is almost always inside the window.  This test **removes** VO to show
the worst-case drift: if AnyLoc alone had to chain estimates with no VO
correction between steps, how quickly does the anchor drift?  A good result
here (high `in_window_pct`, negative accuracy delta) means the constrained
search is robust even when VO is unavailable or unreliable.

---

### What it measures

| Column | Meaning |
|---|---|
| `Err_glob` | Euclidean error (m) with full FAISS global search |
| `Err_const` | Euclidean error (m) with constrained search (anchor-chain, no VO) |
| `T_glob ms` | Inference wall-time for global search |
| `T_const ms` | Inference wall-time for constrained search |
| `Speedup` | `T_glob / T_const` — how many times faster constrained is |
| `InWin` | `Y` if the true position fell inside the search window; `N` means the window drifted too far |

### Sample output

```
  #    True lat    True lon   Err_glob  Err_const   T_glob ms  T_const ms  Speedup  InWin  AGL
  ------------------------------------------------------------------------------------------------
   1  23.448201  120.284031      87.2       87.2      1240.1      1238.6     1.00x      Y  80 m
   2  23.449015  120.283714      71.4       68.9      1231.8       312.4     3.94x      Y  80 m
   3  23.449829  120.283397      59.1       55.3      1228.5       308.1     3.99x      Y  80 m
  ...

  Metric                    Global  Constrained     Delta
  --------------------------------------------------------
  Mean error (m)             74.20        61.80    -12.40
  Median error (m)           68.50        57.20    -11.30
  RMSE (m)                   81.30        68.10    -13.20
  Mean latency (ms)        1235.0        310.5     3.98x speedup
  True pos in window           —          95.0 %
```

---

## 6. Run the ROS2 Node

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

## 7. Full System Launch

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
