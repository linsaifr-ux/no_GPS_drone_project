"""
Fine-tune YOLOv8n for top-down vehicle detection.

Prerequisites:
    python detection/prepare_dataset.py   # build the dataset first

Run (outside Isaac Sim, GPU recommended):
    python detection/finetune.py

Best weights saved to:
    detection/runs/topdown_v1/weights/best.pt

To use the fine-tuned model:
    det = YOLODetector('detection/runs/topdown_v1/weights/best.pt', conf=0.30)
"""
from pathlib import Path
from ultralytics import YOLO

_HERE      = Path(__file__).parent
DATA_YAML  = _HERE / "dataset" / "data.yaml"
BASE_MODEL = _HERE.parent / "yolov8n.pt"
RUN_DIR    = _HERE / "runs"
RUN_NAME   = "topdown_v1"

if not DATA_YAML.exists():
    raise FileNotFoundError(
        f"{DATA_YAML} not found — run  python detection/prepare_dataset.py  first"
    )

model = YOLO(str(BASE_MODEL))

model.train(
    data     = str(DATA_YAML),
    epochs   = 100,
    imgsz    = 640,
    batch    = 16,
    lr0      = 1e-3,
    lrf      = 0.01,
    warmup_epochs = 3,

    # — Augmentation tuned for top-down aerial view —
    degrees  = 45,    # vehicles appear at all headings; add rotational variety
    flipud   = 0.5,   # valid for nadir (meaningless for side-view, valid here)
    fliplr   = 0.5,
    scale    = 0.5,   # zoom 0.5×–2× simulates altitude variation
    mosaic   = 1.0,   # critical: improves small-object recall
    hsv_h    = 0.015,
    hsv_s    = 0.7,
    hsv_v    = 0.4,   # handle overcast / harsh sun conditions

    project  = str(RUN_DIR),
    name     = RUN_NAME,
    exist_ok = True,
)

best = RUN_DIR / RUN_NAME / "weights" / "best.pt"
print(f"\n[finetune] Best weights → {best}")
print(f"[finetune] Update your detector call to:\n"
      f"  det = YOLODetector('{best}', conf=0.30)")
