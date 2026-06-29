import os
import sys
import unittest

import numpy as np

from config.camera_config import CAMERA_WIDTH, CAMERA_HEIGHT
from config.vision_config import BUOY_MODEL_PATH
from vision.detector import BuoyDetector

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)


class TestVisionDetector(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        print("\n[TEST] YOLO Modeli yükleniyor (CPU Modunda)...")
        try:
            # Using cpu for test env
            cls.detector = BuoyDetector(model_path=BUOY_MODEL_PATH, device="cpu")
            print("[TEST] Model başarıyla yüklendi.")
        except Exception as e:
            cls.detector = None
            print(f"[TEST - HATA] Model yüklenemedi: {e}")

    def test_01_initialization(self):
        self.assertIsNotNone(self.detector, "BuoyDetector başlatılamadı.")
        self.assertIsNotNone(self.detector.model, "YOLO modeli yüklenemedi.")

        expected_classes = {0: "red_buoy", 1: "green_buoy", 2: "black_buoy", 3: "orange_buoy", 4: "yellow_buoy"}
        self.assertEqual(self.detector.class_names, expected_classes, "Sınıf isimleri uyuşmuyor.")

    def test_02_dummy_detection(self):
        if self.detector is None:
            self.skipTest("Model yüklenmediği için bu test atlanıyor.")

        print("\n[TEST] Sahte veri ile tespit testi yapılıyor...")

        dummy_bgr = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
        dummy_depth = np.full((CAMERA_HEIGHT, CAMERA_WIDTH), 5.0, dtype=np.float32)

        try:
            detections = self.detector.detect(dummy_bgr, dummy_depth)
        except Exception as e:
            self.fail(f"detect() fonksiyonu sahte veri ile çöktü: {e}")

        self.assertIsInstance(detections, list, "Dönen değer bir liste olmalı.")

        for det in detections:
            self.assertIn("class", det)
            self.assertIn("confidence", det)
            self.assertIn("distance", det)
            self.assertIn("bbox", det)

    def test_03_depth_nan_handling(self):
        if self.detector is None:
            self.skipTest("Model yüklenmediği için bu test atlanıyor.")

        dummy_bgr = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
        dummy_depth = np.full((CAMERA_HEIGHT, CAMERA_WIDTH), np.nan, dtype=np.float32)

        try:
            detections = self.detector.detect(dummy_bgr, dummy_depth)
            self.assertIsInstance(detections, list)
        except Exception as e:
            self.fail(f"NaN içeren derinlik verisiyle detect() çöktü: {e}")


if __name__ == '__main__':
    unittest.main()
