import os
import numpy as np
from ultralytics import YOLO


class BuoyDetector:
    def __init__(self, model_path="models/best.pt"):
        # Eğer path göreceli ise, project root'u bulup mutlak yola (absolute path) çevir
        if not os.path.isabs(model_path):
            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_path = os.path.join(project_root, model_path)

        self.model = YOLO(model_path)

        # Belirttiğin şamandıra sınıfları
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
        # verbose=False: Konsolu her frame'de YOLO çıktılarıyla boğmamak için
        results = self.model(bgr_image, verbose=False)
        detections = []

        for box in results[0].boxes:
            # Bounding box koordinatları
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
            cls_id = int(box.cls[0].cpu().numpy())
            conf = float(box.conf[0].cpu().numpy())

            # Sözlükte olmayan bir id gelirse hatayı önle
            class_name = self.class_names.get(cls_id, f"unknown_{cls_id}")

            # --- Derinlik (Mesafe) Hesabı ---
            # Bounding box'ın derinlik matrisi sınırları dışına taşmasını engelle (Güvenlik)
            h, w = depth_array.shape
            x1_c, x2_c = max(0, x1), min(w, x2)
            y1_c, y2_c = max(0, y1), min(h, y2)

            # Geçerli bir alan varsa medyanı al, aksi halde mesafe -1 (geçersiz)
            if y2_c > y1_c and x2_c > x1_c:
                distance = float(np.nanmedian(depth_array[y1_c:y2_c, x1_c:x2_c]))
            else:
                distance = -1.0

            # Sonsuz (inf) veya tanımsız (NaN) mesafe okumalarını filtrele
            if not np.isfinite(distance):
                distance = -1.0

            detections.append({
                "class": class_name,
                "confidence": round(conf, 3),
                "distance": round(distance, 2),
                "bbox": [x1, y1, x2, y2]
            })

        return detections