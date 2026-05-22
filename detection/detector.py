"""
YOLO vehicle detector — wraps ultralytics YOLOv8 and filters to vehicle classes.
"""
from __future__ import annotations

from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

# COCO class IDs for vehicles
_VEHICLE_IDS = {2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck'}

_COLORS = {
    'car':        '#ff4444',
    'motorcycle': '#ff8800',
    'bus':        '#cc44ff',
    'truck':      '#ffee00',
}


class YOLODetector:
    """
    YOLOv8 vehicle detector.

    Usage:
        det = YOLODetector()
        detections = det.detect(pil_img)   # list of dicts
        annotated  = det.draw(pil_img, detections)
    """

    def __init__(self, model_name: str = 'yolov8n.pt', conf: float = 0.35):
        print(f"[YOLO] Loading {model_name} …")
        self.model = YOLO(model_name)
        self.conf  = conf
        print(f"[YOLO] Model ready  conf_threshold={conf}")

    def detect(self, pil_img: Image.Image) -> list[dict]:
        """
        Run inference on a PIL image.

        Returns list of dicts:
            label  str    — 'car', 'motorcycle', 'bus', or 'truck'
            conf   float  — confidence score
            x1 y1 x2 y2  float — bounding box pixels (xyxy, top-left origin)
        """
        results = self.model(pil_img, conf=self.conf, verbose=False)[0]
        out = []
        for box in results.boxes:
            cls_id = int(box.cls[0])
            if cls_id not in _VEHICLE_IDS:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            out.append({
                'label': _VEHICLE_IDS[cls_id],
                'conf':  float(box.conf[0]),
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
            })
        return out

    def draw(self, pil_img: Image.Image,
             detections: list[dict]) -> Image.Image:
        """Draw bounding boxes + labels on a PIL image. Returns new PIL RGB image."""
        img  = pil_img.copy().convert('RGB')
        draw = ImageDraw.Draw(img)
        try:
            font = ImageFont.truetype(
                '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 13)
        except Exception:
            font = ImageFont.load_default()

        for det in detections:
            color = _COLORS.get(det['label'], '#ffffff')
            x1, y1, x2, y2 = det['x1'], det['y1'], det['x2'], det['y2']

            draw.rectangle([x1, y1, x2, y2], outline=color, width=2)

            label = f"{det['label']} {det['conf']:.2f}"
            lw    = draw.textlength(label, font=font)
            ty    = max(0, y1 - 16)
            draw.rectangle([x1, ty, x1 + lw + 4, ty + 16], fill=color)
            draw.text((x1 + 2, ty + 1), label, fill='black', font=font)

        return img
