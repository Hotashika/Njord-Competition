import os
import sys
import unittest

# Testlerin kök dizindeki modülleri görebilmesi için
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

import pyzed.sl as sl
from main import init_camera
from core import shared_state


class TestCameraHardware(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Sınıf başlatıldığında (test serisi başlarken) kamerayı sadece BİR KERE açar."""
        print("\n[TEST] Kamera başlatılıyor (Tek seferlik)...")
        try:
            cls.zed = init_camera()
        except Exception as e:
            cls.zed = None
            print(f"Kamera başlatılamadı: {e}")

    @classmethod
    def tearDownClass(cls):
        """Tüm testler bittikten sonra kamerayı kapatır ve paylaşımlı belleği temizler."""
        if hasattr(cls, 'zed') and cls.zed and cls.zed.is_opened():
            cls.zed.close()
            print("\n[TEST] Kamera güvenli bir şekilde kapatıldı.")

        # Olası sızıntıları engellemek için Shared Memory bloklarını yok ediyoruz
        try:
            shared_state._rgb_shm.close()
            shared_state._rgb_shm.unlink()
            shared_state._depth_shm.close()
            shared_state._depth_shm.unlink()
            shared_state._meta_shm.close()
            shared_state._meta_shm.unlink()
            print("[TEST] Paylaşımlı bellek (Shared Memory) temizlendi.")
        except Exception:
            pass

    def test_01_initialization(self):
        """Kameranın başarıyla açıldığını doğrular."""
        self.assertIsNotNone(self.zed, "init_camera() None döndürdü.")
        self.assertTrue(self.zed.is_opened(), "Kamera açık değil.")
        print("\n[TEST - 01] Kamera başarıyla ilklendirildi.")

    def test_02_frame_grab(self):
        """Kameradan görüntü çekilebildiğini doğrular."""
        runtime_params = sl.RuntimeParameters()
        image = sl.Mat()

        # Kamera ilk açıldığında sensörlerin oturması için ilk birkaç frame boş dönebilir,
        # bu yüzden grab işlemini birkaç kez deniyoruz.
        success = False
        for _ in range(5):
            if self.zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                success = True
                break

        self.assertTrue(success, "Kameradan frame yakalanamadı (grab hatası).")

        retrieve_status = self.zed.retrieve_image(image, sl.VIEW.LEFT)
        self.assertEqual(retrieve_status, sl.ERROR_CODE.SUCCESS, "Görüntü çekilemedi.")
        self.assertTrue(image.is_init(), "Çekilen görüntü boş veya başlatılamadı.")
        print("\n[TEST - 02] Frame başarıyla yakalandı.")


if __name__ == '__main__':
    unittest.main()