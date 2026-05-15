# Isaac Sim Test

A Python environment for running NVIDIA Isaac Sim 6.0.0.0 simulations.

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

### Run the headless test

```bash
./run_test.sh
# or
OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python test_isaac.py
```

### Launch the full GUI simulator

```bash
DISPLAY=:0 OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test isaacsim
```

Replace `:0` with your active X display (check with `echo $DISPLAY`). This launches the full Isaac Sim GUI (`isaacsim.exp.full` experience).

> **Note:** Do **not** use `python -m isaacsim` — that runs a VS Code settings generator, not the simulator.

### Write your own headless script

```python
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import World

world = World()
world.reset()

for i in range(100):
    world.step(render=False)

simulation_app.close()
```

Always run with:

```bash
OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python your_script.py
```

### Write your own GUI script

```python
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World

world = World()
world.reset()

for i in range(100):
    world.step(render=True)

simulation_app.close()
```

Run with a display set:

```bash
DISPLAY=:0 OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python your_script.py
```

### Run the city scene

```bash
DISPLAY=:0 OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python city_scene.py
```

Opens the full Isaac Sim GUI with:
- Two-lane asphalt road with yellow centre dashes and white edge lines
- 14 office buildings of varying heights with glass window bands
- Red car (chassis + cabin + 4 wheels) placed as a rigid body on the road
- 8 street lights with warm point lights
- 6 trees
- Directional sun + sky dome lighting
- Viewport camera pre-aimed at the car from a 45° angle

The car has mass and gravity — it rests on its wheels. You can pause and apply forces or extend the script with vehicle controls.

## Project structure

```
isaac_sim_test/
├── README.md           # This file
├── test_isaac.py       # Headless test: creates a World and steps 5 times
├── city_scene.py       # GUI city scene with car, buildings, lights and trees
├── run_test.sh         # Convenience wrapper (sets OMNI_KIT_ACCEPT_EULA=Y)
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
