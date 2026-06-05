# Isaac Sim — Simulator README

`cesium_scene.py` is the **physics engine and visualiser** for the no-GPS drone project. It runs a 100 Hz background physics thread (same 6-DOF kinematic model as `drone_sim.py`) + the ArduPilot SITL bridge, and publishes `/drone/state` for the flight commander and AnyLoc nodes. The render loop (~13 Hz) reads the current physics state and updates the drone mesh.

**`drone_sim.py` is not used when Isaac Sim is running.** Both bind UDP 9002 — run only one at a time.

**Start `cesium_scene.py` before the autopilot SITL.** It must open the bridge port before SITL starts, or SITL exits immediately.

Launch: `cd simulator && ./run_chiayi.sh`

### Autopilot toggle: ArduPilot (default) vs PX4 (`PX4_SIM=1`)
`cesium_scene.py` honours the `PX4_SIM` env var (same as `control/drone_sim.py`):
- **unset** → ArduPilot `SITLBridge` on **UDP 9002** (binary servo in / JSON physics out); motor
  decode uses the ArduCopter QUAD-X order.
- **`PX4_SIM=1`** → PX4 `PX4SimBridge` on **TCP 4560** (HIL_ACTUATOR_CONTROLS in / HIL_SENSOR out);
  motor decode uses the PX4 none_iris CA_ROTOR geometry. The kinematic model below the decode is
  identical. See `control/README.md` for the PX4 launch sequence and migration status.

For fast PX4 control-loop iteration use the headless `control/drone_sim.py` (`PX4_SIM=1`) instead
of the full Isaac render.

---

## Architecture inside cesium_scene.py

```
Background thread (100 Hz)          Render loop (~13 Hz)
─────────────────────────────        ─────────────────────────────
bridge.step() ←→ ArduPilot SITL     read _k* state under lock
kinematic 6-DOF integration          update drone_pos_op / drone_orient_op
publish /drone/state                 HUD, frame capture, ROS2 spin
```

## What the physics thread does each step (100 Hz)

1. Send current kinematic state to ArduPilot SITL via `SITLBridge` (UDP 9002)
2. Receive latest motor PWM from ArduPilot
3. Integrate 6-DOF kinematic model: PWM → thrust → roll/pitch → NED velocity → ENU position
4. Apply ground constraint (no sliding on ground)
5. Update shared state variables (protected by `threading.Lock`)
6. Publish `/drone/state` (ENU PoseStamped, frame `local_enu`)

## Kinematic physics constants

| Parameter | Value | Notes |
|-----------|-------|-------|
| Mass | 1.0 kg | Sets hover at PWM 1500 matching `MOT_THST_HOVER=0.5` |
| Max tilt | 0.35 rad (~20°) | From PWM differential |
| Attitude τ | 0.15 s | First-order response |
| Drag | 0.35 s⁻¹ | Aerodynamic drag |
| Physics rate | 100 Hz | Background thread — ArduPilot sees 100 Hz physics |
| Motor layout | ch1=FR(NE), ch2=RL(SW), ch3=RR(SE), ch4=FL(NW) | ArduCopter X-frame FRAME_TYPE=1 |

---

# Isaac Sim Environment Setup

A Python environment for running NVIDIA Isaac Sim 6.0.0 simulations.

## Requirements

- NVIDIA GPU (RTX 2080 Ti or better, 8 GB+ VRAM)
- NVIDIA driver 520+ (tested with 580.142)
- [Miniconda](https://docs.conda.io/en/latest/miniconda.html)
- Ubuntu 22.04 / 24.04

## Setup

### 1. Create the conda environment

```bash
conda create -n isaac_sim_test python=3.12 -y
conda run -n isaac_sim_test python -m ensurepip --upgrade
```

### 2. Install Isaac Sim packages

```bash
OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python -m pip install \
    isaacsim==6.0.0.0 \
    isaacsim-app==6.0.0.0 \
    isaacsim-core==6.0.0.0 \
    isaacsim-extscache-kit==6.0.0.0 \
    isaacsim-extscache-kit-sdk==6.0.0.0 \
    isaacsim-extscache-physics==6.0.0.0 \
    isaacsim-robot==6.0.0.0 \
    isaacsim-sensor==6.0.0.0 \
    isaacsim-asset==6.0.0.0 \
    isaacsim-replicator==6.0.0.0 \
    isaacsim-gui==6.0.0.0 \
    isaacsim-utils==6.0.0.0 \
    isaacsim-cortex==6.0.0.0 \
    isaacsim-rl==6.0.0.0 \
    isaacsim-code-editor==6.0.0.0 \
    --extra-index-url https://pypi.nvidia.com
```

> **Note:** Total download is ~20 GB. The extscache packages alone are ~6 GB.

### 3. Accept the EULA

```bash
echo "yes" > $(conda run -n isaac_sim_test python -c \
    "import isaacsim; import os; print(os.path.join(os.path.dirname(isaacsim.__file__), 'kit', 'EULA_ACCEPTED'))")
```

### 4. Create missing extension stubs

Isaac Sim 6.0.0.0 pip packages reference test-only and GUI-only extensions as hard dependencies. Run the included script to create lightweight stubs:

```bash
OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python create_stubs.py
```

Fix the version constraint on one stub (`carb.graphics_optional` requires `^2`):

```bash
sed -i 's/version = "1.0.0"/version = "2.0.0"/' \
    $(conda run -n isaac_sim_test python -c \
    "import isaacsim, os; print(os.path.join(os.path.dirname(isaacsim.__file__), 'extsUser', 'carb.graphics_optional', 'config', 'extension.toml'))")
```

Also create stubs for extensions required by the full GUI experience (`isaacsim.exp.full`) that are not installed:

```bash
ISAACSIM=$(conda run -n isaac_sim_test python -c "import isaacsim, os; print(os.path.dirname(isaacsim.__file__))")
for ext in \
    isaacsim.examples.interactive \
    isaacsim.exp.base \
    isaacsim.robot_setup.assembler \
    isaacsim.robot_setup.collision_detector \
    isaacsim.robot_setup.gain_tuner \
    isaacsim.robot_setup.grasp_editor \
    isaacsim.robot_setup.xrdf_editor; do
  mkdir -p "$ISAACSIM/extsUser/$ext/config"
  printf '[package]\nversion = "1.0.0"\ntitle = "%s stub"\n\n[dependencies]\n' "$ext" \
    > "$ISAACSIM/extsUser/$ext/config/extension.toml"
done
```

## Usage

### Launch the full simulator (standard)

```bash
cd simulator && ./run_chiayi.sh
```

This sources ROS2 Jazzy and runs `cesium_scene.py` in the `isaac_sim_test` conda environment.

### Run the headless test

```bash
./run_test.sh
# or
OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python test_isaac.py
```

### Launch the full GUI simulator standalone

```bash
DISPLAY=:0 OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test isaacsim
```

Replace `:0` with your active X display (check with `echo $DISPLAY`).

## Project structure

```
simulator/
├── README.md           # This file
├── cesium_scene.py     # Physics + visualiser: Cesium terrain, 100 Hz kinematic thread, SITL bridge
├── run_chiayi.sh       # Launch script (sources ROS2 Jazzy, runs in conda env)
├── test_isaac.py       # Headless test: creates a World and steps 5 times
├── city_scene.py       # GUI city scene (standalone demo — not used in flight pipeline)
├── geo_utils.py        # Geo helpers shared between scene scripts
└── create_stubs.py     # One-time script to stub missing test extensions
```

## Performance notes

- **First run:** ~4 minutes — Warp compiles CUDA kernels and caches them at `~/.cache/warp/`
- **Subsequent runs:** ~15 seconds startup
- **CPU powersave warning:** Set your CPU governor to `performance` for better throughput:
  ```bash
  echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
  ```

## Known non-fatal warnings

| Warning | Cause | Impact |
|---------|-------|--------|
| `isaacsim.robot.manipulators.examples` failed to load | Stub `isaacsim.examples.base` has no Python code | None — example UI only |
| `omni.platforminfo` CPU core errors | Kernel 6.x topology reporting | None — cosmetic |
| `Unexpected reference count` on UsdStage close | Normal during shutdown | None |
