from pathlib import Path

from ultralytics import YOLO

from config.vision_config import DEVICE, BUOY_MODEL_PATH, VESSEL_MODEL_PATH
from vision.depth_utils import get_distance_from_bbox


class BaseYOLODetector:
    def __init__(self, model_path, device=DEVICE):
        model_p = Path(model_path)
        if not model_p.is_absolute():
            project_root = Path(__file__).resolve().parent.parent
            model_p = project_root / model_path

        self.model = YOLO(str(model_p))
        self.device = device

        self.class_names = self.model.names

    def detect(self, bgr_image, depth_array):
        results = self.model(bgr_image, device=self.device, verbose=False)
        detections = []

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
            cls_id = int(box.cls[0].cpu().numpy())
            conf = float(box.conf[0].cpu().numpy())

            class_name = self.class_names.get(cls_id, f"unknown_{cls_id}")
            bbox = [x1, y1, x2, y2]

            distance = get_distance_from_bbox(depth_array, bbox, method="median")

            detections.append({
                "class": class_name,
                "confidence": round(conf, 3),
                "distance": round(distance, 2),
                "bbox": bboxf
            })

        return detections


class BuoyDetector(BaseYOLODetector):
    def __init__(self, model_path=BUOY_MODEL_PATH, device=DEVICE):
        super().__init__(model_path, device)


class VesselDetector(BaseYOLODetector):
    def __init__(self, model_path=VESSEL_MODEL_PATH, device=DEVICE):
        super().__init__(model_path, device)

    def detect(self, bgr_image, depth_array):
        detections = super().detect(bgr_image, depth_array)
        for det in detections:
            det["your_measurement"] = self._compute_measurement(det, depth_array)
        return detections

    def _compute_measurement(self, detection, depth_array):
        bbox = detection["bbox"]
        # TODO: Gelen geminin araca göre olan açısı hesaplanacak.

        side = None
        return side
