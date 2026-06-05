# detection/ — YOLOv8 Vehicle Detection

Real-time vehicle detection from the drone's nadir camera. Uses YOLOv8 with a VisDrone-trained model and automatically maps model-specific class names to four canonical vehicle labels.

---

## Files

| File | Purpose |
|------|---------|
| `detector.py` | `YOLODetector` class — wraps ultralytics YOLOv8 |
| `ros2_node.py` | ROS2 node — subscribes to camera, publishes detections |
| `run_ros2_detector.sh` | Launch script (sources ROS2 Jazzy, runs in conda env) |
| `run_detector.py` | Standalone test runner (no ROS2) |
| `collect_training_data.py` | Collect frames + pseudo-labels for fine-tuning |
| `prepare_dataset.py` | Convert collected data to YOLO format |
| `finetune.py` | Fine-tune YOLOv8 on collected dataset |
| `label_writer.py` | Write YOLO label files from detection dicts |

Active model: `yolov8l_visdrone.pt` (project root) — YOLOv8l fine-tuned on VisDrone aerial imagery.

---

## Requirements

```bash
conda run -n isaac_sim_test pip install ultralytics
```

ROS2 Jazzy + `ros-jazzy-vision-msgs` for the ROS2 node.

---

## Canonical Labels

`YOLODetector` maps both COCO and VisDrone class names to four labels:

| Canonical | COCO name | VisDrone names |
|-----------|-----------|----------------|
| `car` | `car` | `car`, `van` |
| `motorcycle` | `motorcycle` | `motor`, `tricycle`, `awning-tricycle` |
| `bus` | `bus` | `bus` |
| `truck` | `truck` | `truck` |

All other classes are filtered out. This makes the node model-agnostic — drop in any COCO or VisDrone model without code changes.

---

## Run the ROS2 Node

**Prerequisites:** `cesium_scene.py` (or `drone_sim.py`) must be publishing `/drone/camera/image_raw`.

```bash
bash detection/run_ros2_detector.sh
```

Or manually:
```bash
source /opt/ros/jazzy/setup.bash
conda run -n isaac_sim_test --no-capture-output python3 detection/ros2_node.py
```

### ROS2 Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `/drone/camera/image_raw` | `sensor_msgs/Image` | rgb8, 640×480 |
| Subscribe | `/drone/pose` | `geometry_msgs/PoseStamped` | ENU pose (for geo-tagging) |
| Publish | `/yolo/detections` | `vision_msgs/Detection2DArray` | bounding boxes + class + confidence |

### Detection2D fields

Each detection in the array:
- `bbox.center.position.x/y` — bounding box centre in pixels
- `bbox.size_x/y` — bounding box width/height in pixels
- `results[0].hypothesis.class_id` — canonical label (`car`, `bus`, etc.)
- `results[0].hypothesis.score` — confidence [0, 1]

---

## Standalone Usage

```python
from PIL import Image
from detection.detector import YOLODetector

det = YOLODetector(model_name='yolov8l_visdrone.pt', conf=0.35)

img = Image.open('frame.jpg')
detections = det.detect(img)
# [{'label': 'car', 'conf': 0.72, 'box': [x1, y1, x2, y2]}, ...]

annotated = det.draw(img, detections)
annotated.save('annotated.jpg')
```

---

## Fine-Tuning Pipeline

To fine-tune on simulated data from Isaac Sim:

```bash
# 1. Collect frames during a flight (saves jpg + pseudo-labels)
python3 detection/collect_training_data.py

# 2. Convert to YOLO dataset format
python3 detection/prepare_dataset.py

# 3. Fine-tune
python3 detection/finetune.py
```

The fine-tuning pipeline is ready to run; training data collection requires an active Isaac Sim session with the drone flying over the scene.

---

## Performance Notes

- `yolov8l_visdrone.pt` (large model): ~40 ms/frame on RTX 2080 Ti; ~200 ms/frame on CPU
- `yolov8n.pt` (nano model): ~5 ms/frame GPU; ~30 ms/frame CPU
- The ROS2 node runs inference on every received frame (~6 Hz from Isaac Sim) and keeps a live postview window showing the latest annotated frame
