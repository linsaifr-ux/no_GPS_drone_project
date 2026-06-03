# AnyLoc — Build & Run Guide (Windows)

Visual place recognition for GPS-denied drone navigation.  
Uses **DINOv2** patch features + **VLAD** aggregation + **FAISS** nearest-neighbour search against a geo-tagged satellite image database.

> **Note:** The full autonomous flight stack (ROS2, MAVROS2, ArduPilot SITL) is
> Linux-only. On Windows you can build the database and run the standalone
> localizer. For the complete system, use **WSL2** (see Section 5).

---

## Requirements

| Component | Version | Notes |
|---|---|---|
| OS | Windows 10 / 11 (64-bit) | |
| Python | 3.10 – 3.12 | Via Miniconda/Anaconda |
| PyTorch | ≥ 2.0 | CPU or CUDA build |
| torchvision | ≥ 0.15 | |
| Pillow | ≥ 9.0 | |
| NumPy | ≥ 1.24 | |
| faiss-cpu | ≥ 1.7 | Install via conda-forge |
| OpenCV | ≥ 4.7 | |
| requests | any | For NLSC tile download |

---

## 1. Install Miniconda

Download and install Miniconda for Windows from:
https://docs.conda.io/en/latest/miniconda.html

During installation, select **"Add Miniconda3 to my PATH"** or use
**Anaconda Prompt** for all commands below.

---

## 2. Create the Conda Environment

Open **Anaconda Prompt** and run:

```bat
conda create -n anyloc python=3.10 -y
conda activate anyloc
```

### Install PyTorch

CPU-only (smaller download, works everywhere):

```bat
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

With CUDA 12.x (faster database build if you have an NVIDIA GPU):

```bat
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

### Install remaining dependencies

```bat
pip install pillow numpy requests opencv-python
conda install -c conda-forge faiss-cpu -y
```

> `faiss-gpu` is not available on Windows via conda-forge.
> Use the CPU build above.

---

## 3. Build the Image Database

Run from the **project root** directory (the folder containing `anyloc/`).
Open Anaconda Prompt, `cd` to the project root, then:

```bat
conda activate anyloc
python anyloc\build_database.py
```

### Build options

| Flag | Default | Description |
|---|---|---|
| `--grid-step N` | 50 | Grid spacing in metres |
| `--agl-min N` | 60 | Minimum AGL altitude in metres |
| `--agl-max N` | 120 | Maximum AGL altitude in metres |
| `--agl-step N` | 5 | AGL increment in metres |
| `--rebuild` | off | Overwrite an existing database |

Example — finer grid:

```bat
python anyloc\build_database.py --grid-step 25 --agl-min 50 --agl-max 150
```

### What the build does

1. **Downloads** Taiwan NLSC PHOTO2 orthophoto tiles at zoom 18 from
   `wmts.nlsc.gov.tw` and stitches them into a mosaic JPEG.
2. **Crops** satellite patches for each (lat, lon, AGL) grid point simulating
   a nadir drone camera (90° × 73.7° FOV).
3. **Encodes** each crop with DINOv2 ViT-B/14 (downloaded from torch hub on
   first run — requires internet, ~400 MB).
4. **Clusters** patch descriptors into a VLAD codebook (k = 64) with FAISS k-means.
5. **Saves** the database to `anyloc\database\`.

> DINOv2 weights are cached in `%USERPROFILE%\.cache\torch\hub\` after the
> first download.

### Output files

```
anyloc\database\
├── database.pt          # main file: lats, lons, alts, codebook
├── database_meta.pt     # entry metadata
├── database_vlads.pt    # VLAD vectors (N × 512)
├── db_meta.json         # build cache — skip re-download if present
└── db_images\           # satellite crop JPEGs (one per grid point)
```

---

## 4. Run the Standalone Localizer

```bat
conda activate anyloc
python anyloc\run_localizer.py
```

This runs inference only — no ROS2 or MAVROS2 needed.

---

## 5. Full System on Windows via WSL2

The ROS2 node (`ros2_node.py`) and the complete flight stack require Linux.
The recommended approach on Windows is **WSL2 with Ubuntu 24.04**.

### Install WSL2

```powershell
# Run in PowerShell as Administrator
wsl --install -d Ubuntu-24.04
```

Restart when prompted, then follow the **Linux README** (`README.md`) inside
the WSL2 Ubuntu terminal.

### Access project files from WSL2

Your Windows files are available under `/mnt/c/` in WSL2.
Clone or copy the project there, or work directly from the Windows path:

```bash
# Inside WSL2 terminal
cd /mnt/c/Users/<YourName>/path/to/no_GPS_drone_project
```

### GPU passthrough in WSL2

NVIDIA CUDA is supported in WSL2 via **CUDA on WSL** drivers.
Install the WSL-compatible driver from NVIDIA's website before using
`faiss-gpu` or CUDA PyTorch inside WSL2.

---

## Troubleshooting

**`ModuleNotFoundError: No module named 'faiss'`**
→ Run: `conda install -c conda-forge faiss-cpu -y` inside the `anyloc` env.

**`OSError: [WinError 1455] The paging file is too small`** during DINOv2 load
→ Increase Windows virtual memory: System Properties → Advanced → Performance
Settings → Advanced → Virtual Memory → increase to at least 16 GB.

**Database build hangs on tile download**
→ Check internet access to `wmts.nlsc.gov.tw`. Verify no proxy or firewall is
blocking the HTTPS connection. The script retries 3 times per tile.

**Slow database build on CPU**
→ Expected — DINOv2 encoding is compute-heavy. A full build at default settings
takes 20–60 minutes on CPU. Use `--grid-step 100` for a faster test build:
`python anyloc\build_database.py --grid-step 100 --rebuild`

**`latest_estimate.json` not updating**
→ The localizer only writes estimates when AGL ≥ 50 m. The file is created as
a stub at startup and updated once the altitude guard passes.

**Long file paths cause errors**
→ Enable long path support in Windows:
`reg add HKLM\SYSTEM\CurrentControlSet\Control\FileSystem /v LongPathsEnabled /t REG_DWORD /d 1 /f`
Then restart. Also enable in Git: `git config --global core.longpaths true`.
