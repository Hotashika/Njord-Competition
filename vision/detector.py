import os
import numpy as np
from ultralytics import YOLO


class BuoyDetector:
    def __init__(self, model_path="models/best.pt", device=None):
        # Eğer path göreceli ise, project root'u bulup mutlak yola çevir
        if not os.path.isabs(model_path):
            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_path = os.path.join(project_root, model_path)

        self.model = YOLO(model_path)
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
        # Cihaz belirtilmişse onu kullan, belirtilmemişse varsayılan YOLO davranışını (GPU varsa GPU) kullan.
        results = self.model(bgr_image, device=self.device, verbose=False)
        detections = []

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
            cls_id = int(box.cls[0].cpu().numpy())
            conf = float(box.conf[0].cpu().numpy())

            class_name = self.class_names.get(cls_id, f"unknown_{cls_id}")

            h, w = depth_array.shape
            x1_c, x2_c = max(0, x1), min(w, x2)
            y1_c, y2_c = max(0, y1), min(h, y2)

            if y2_c > y1_c and x2_c > x1_c:
                distance = float(np.nanmedian(depth_array[y1_c:y2_c, x1_c:x2_c]))
            else:
                distance = -1.0

            if not np.isfinite(distance):
                distance = -1.0

            detections.append({
                "class": class_name,
                "confidence": round(conf, 3),
                "distance": round(distance, 2),
                "bbox": [x1, y1, x2, y2]
            })

        return detections