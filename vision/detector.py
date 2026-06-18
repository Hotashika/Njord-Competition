from pathlib import Path
from ultralytics import YOLO
from vision.depth_utils import get_distance_from_bbox

class BuoyDetector:
    def __init__(self, model_path="models/best.pt", device=None):
        model_p = Path(model_path)
        if not model_p.is_absolute():
            project_root = Path(__file__).resolve().parent.parent
            model_p = project_root / model_path

        self.model = YOLO(str(model_p))
        self.device = device

        self.class_names = {
            0: "red_buoy",
            1: "green_buoy",
            2: "black_buoy",
            3: "orange_buoy",
            4: "yellow_buoy"
        }

    def detect(self, bgr_image, depth_array):
        """
        BGR görüntüyü ve derinlik matrisini alıp tespitleri döndürür.
        """
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
                "bbox": bbox
            })

        return detections