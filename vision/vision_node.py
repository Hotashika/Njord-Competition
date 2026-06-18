import time
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from multiprocessing import shared_memory
import numpy as np
from vision.detector import BuoyDetector

RGB_SHAPE = (376, 672, 4)
DEPTH_SHAPE = (376, 672)


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # YOLO modelini detector sınıfı üzerinden başlatıyoruz
        self.detector = BuoyDetector(model_path="models/best.pt")

        # main.py shared memory'i daha önce oluşturmuş olmalı; race durumuna karşı retry
        self.rgb_shm = self._attach_with_retry("zed_rgb")
        self.depth_shm = self._attach_with_retry("zed_depth")
        self.meta_shm = self._attach_with_retry("zed_meta")

        self.rgb = np.ndarray(RGB_SHAPE, dtype=np.uint8, buffer=self.rgb_shm.buf)
        self.depth = np.ndarray(DEPTH_SHAPE, dtype=np.float32, buffer=self.depth_shm.buf)
        self.meta = np.ndarray((2,), dtype=np.int64, buffer=self.meta_shm.buf)

        self.last_frame_id = -1
        self.pub = self.create_publisher(String, '/vision/detections', 10)

        # 15 FPS (saniyede 15 kere çalışacak)
        self.create_timer(1 / 15, self.process_frame)

    def _attach_with_retry(self, name, retries=20, delay=0.5):
        for _ in range(retries):
            try:
                return shared_memory.SharedMemory(name=name)
            except FileNotFoundError:
                time.sleep(delay)
        raise RuntimeError(f"{name} shared memory bulunamadı, main.py çalışıyor mu?")

    def process_frame(self):
        frame_id = int(self.meta[0])
        if frame_id == self.last_frame_id:
            return  # yeni frame yok, tekrar işlemeye gerek yok
        self.last_frame_id = frame_id

        # Shared memory'den numpy dizilerine kopyala
        bgr_image = self.rgb[:, :, :3].copy()  # BGRA'dan BGR'a geçiş
        depth_array = self.depth.copy()

        # Detector sınıfını çağırarak tespitleri al
        detections = self.detector.detect(bgr_image, depth_array)

        # Eğer tespit edilen bir nesne varsa JSON olarak MAVLink köprüsüne yayınla
        if detections:
            msg = String()
            msg.data = json.dumps(detections)
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()