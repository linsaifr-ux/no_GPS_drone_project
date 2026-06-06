# Plan: Distributed Sim — PC runs Isaac Sim, Jetson runs AnyLoc + YOLO

## Goal

Run heavy inference (AnyLoc, YOLO) on Jetson while Isaac Sim, PX4 SITL, MAVROS, and the
commander remain on the PC. Camera frames flow PC → Jetson; VPE flows Jetson → PC → MAVROS.

---

## What Runs Where

| Component | Machine | Notes |
|---|---|---|
| Isaac Sim (`cesium_scene.py`) | PC | publishes camera + pose + agl |
| PX4 SITL | PC | TCP 4560 |
| MAVROS | PC | UDP 14540/14580 |
| `px4_commander.py` | PC | Phase 1 VPE from local kinematic truth; Phase 2 VPE from AnyLoc topic |
| `anyloc/ros2_node.py` | Jetson | subscribes to camera; publishes `/anyloc/pose_estimate` |
| `detection/ros2_node.py` | Jetson | subscribes to camera; publishes `/yolo/detections` |

---

## Network Requirements

- Same LAN — Gigabit Ethernet strongly preferred
- `ROS_DOMAIN_ID=0` on both machines (must match)
- Both source `/opt/ros/jazzy/setup.bash`

**Bandwidth (raw image):** 1024 × 768 × 3 B × 5 fps ≈ 11.5 MB/s ≈ 92 Mbps  
→ Fine on Gigabit Ethernet. On WiFi, add compressed image transport (see section 5).

---

## Topic Data Flows

```
PC (Isaac Sim)                              Jetson
──────────────────────────────────────────────────────────────
cesium_scene.py
  /drone/camera/image_raw  ──────────────► anyloc/ros2_node.py
  /drone/pose              ──────────────► anyloc/ros2_node.py
  /drone/agl               ──────────────► anyloc/ros2_node.py
                                           detection/ros2_node.py
                                                │
                           ◄──────────────  /anyloc/pose_estimate
                           ◄──────────────  /yolo/detections

px4_commander.py
  subscribes /anyloc/pose_estimate  (arrives from Jetson over DDS)
  publishes  /mavros/vision_pose/pose_cov  (to local MAVROS)
```

ROS2 DDS peer discovery handles cross-machine topic routing automatically — no extra
brokers or configuration needed beyond matching `ROS_DOMAIN_ID`.

---

## Code Changes Required

### 1. `anyloc/ros2_node.py` — pass `error_m` into `_publish()`

Currently `_publish()` uses a hardcoded covariance (20 m² XY). The commander needs
dynamic covariance `max(1, error_m²)` to weight AnyLoc estimates correctly. Fix: pass
`error_m` to `_publish()` and compute the same covariance the commander currently
computes from JSON.

```python
# _cb_image calls:
self._publish(est_lat, est_lon, drone_alt, error_m=err_m)

# _publish signature:
def _publish(self, lat, lon, alt_msl, error_m: float = 20.0):
    ...
    xy_var = max(1.0, error_m ** 2)
    cov[0] = cov[7] = xy_var
    cov[14] = 0.25   # altitude: 0.5 m std (kept tight — baro handles this)
```

### 2. `px4_commander.py` — subscribe to `/anyloc/pose_estimate` instead of reading JSON

Phase 1 VPE (kinematic truth) is unchanged — commander still reads `/drone/state`
locally. Phase 2 VPE switches from JSON polling to a ROS2 subscription:

```python
# Add subscriber in __init__:
self.create_subscription(
    PoseWithCovarianceStamped, "/anyloc/pose_estimate",
    self._cb_anyloc, 1)

def _cb_anyloc(self, msg):
    self._anyloc_msg = msg          # store latest; VPE thread reads this
    self._anyloc_stamp = time.time()

# In VPE thread, Phase 2:
msg = self._anyloc_msg
if msg is not None and (time.time() - self._anyloc_stamp) < 2.0:
    # use msg.pose.pose.position.{x,y,z} and msg.pose.covariance directly
    # no JSON file read needed
```

Remove the JSON file polling loop entirely from the VPE thread.

### 3. (WiFi only) Compressed image transport

If using WiFi, publish a JPEG-compressed image alongside the raw topic to cut bandwidth
from ~92 Mbps to ~5–15 Mbps. Requires `ros-jazzy-image-transport` and
`ros-jazzy-compressed-image-transport` on both machines.

In `cesium_scene.py`, publish `sensor_msgs/CompressedImage` on
`/drone/camera/image_raw/compressed`. AnyLoc and YOLO nodes subscribe to the compressed
topic. No other changes needed — `image_transport` handles encode/decode.

---

## Jetson Setup

```bash
# 1. Copy repo to Jetson (or git clone)
rsync -av --exclude=third_party/ /path/to/no_GPS_drone_project/ jetson:~/no_GPS_drone_project/

# 2. Copy AnyLoc database (~small now: ~2820 entries at AGL 65 m)
rsync -av anyloc/database/ jetson:~/no_GPS_drone_project/anyloc/database/

# 3. Copy YOLO model weights
rsync -av yolov8l_visdrone.pt jetson:~/no_GPS_drone_project/

# 4. On Jetson — set domain ID and source ROS2
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc

# 5. On Jetson — run inference nodes
bash anyloc/run_ros2_localizer.sh
bash detection/run_ros2_detector.sh   # in another terminal
```

```bash
# On PC — set same domain ID (if not already set)
export ROS_DOMAIN_ID=0

# Run simulation as normal — no other changes
bash run.sh --tmux --px4
# (do NOT pass --anyloc or --detection — those run on Jetson now)
```

---

## Verification Steps

1. On Jetson: `ros2 topic hz /drone/camera/image_raw` — should show ~5 Hz arriving from PC
2. On PC: `ros2 topic hz /anyloc/pose_estimate` — should show ~5 Hz arriving from Jetson
3. On PC: `ros2 topic echo /anyloc/pose_estimate` — check lat/lon values are plausible
4. Fly the mission; watch commander window for `[PX4Cmd] Phase 2` switching on AnyLoc estimate

---

## Real Hardware Transition (future)

When moving to real hardware, the split is the same (Jetson runs AnyLoc + YOLO) but:
- No Isaac Sim — real camera driver publishes `/drone/camera/image_raw`
- No Phase 1 kinematic truth VPE — EKF2 uses real IMU + baro during climb
- MAVROS connects to Pixhawk over serial, not UDP
- Commander gets a `--real-hw` flag to skip Phase 1 VPE injection
