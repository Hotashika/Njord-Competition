import os
import threading
import sys
import subprocess
import time
import pyzed.sl as sl

# Klasör yapısına uygun modül importları
from servers import video_server
from servers import data_server
from core import data_writer
from core import shared_state


def init_camera():
    zed = sl.Camera()
    init = sl.InitParameters()
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_units = sl.UNIT.METER
    init.camera_resolution = sl.RESOLUTION.VGA
    init.camera_fps = 15

    print("[SYSTEM] ZED Kamera başlatılıyor...")
    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("ZED açılamadı. Kamera bağlantısını kontrol edin.")
    return zed


if __name__ == '__main__':
    # 1. Kamerayı İlklendir
    zed = init_camera()

    # 2. Flask Sunucularını Thread olarak başlat (Zaten izoleler)
    threading.Thread(target=video_server.start, args=(5000,), daemon=True).start()
    threading.Thread(target=data_server.start, args=(5001,), daemon=True).start()

    print("[SYSTEM] ZED başarıyla başlatıldı.")
    print("[SYSTEM] Video stream  -> http://0.0.0.0:5000/video_feed")
    print("[SYSTEM] Data stream   -> http://0.0.0.0:5001/data/stream")

    # 3. Alt Süreçleri (Subprocesses) Başlat
    child_processes = []
    print("\n[SYSTEM] Alt süreçler (Vision ve Bridge) ROS2 ortamında başlatılıyor...")

    # Shared memory'nin (data_writer içinde) tam oturması için ufak bir bekleme
    time.sleep(1)

    try:
        # Kendi ROS2 sürümüne göre
        ros2_setup = "source /opt/ros/foxy/setup.bash"

        # Projenin ana kök dizinini (Njord Competition) buluyoruz
        PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

        # BÜYÜK KURTARICI: Alt süreçlere ana klasörümüzü modül yolu olarak tanıtıyoruz
        python_path_setup = f"export PYTHONPATH='{PROJECT_ROOT}':$PYTHONPATH"

        vision_path = os.path.join(PROJECT_ROOT, "vision", "vision_node.py")
        bridge_path = os.path.join(PROJECT_ROOT, "bridge", "gcs_bridge.py")

        # Komutları Bash kabuğu üzerinden sırayla çalıştıracak şekilde birleştiriyoruz
        # Önce ROS2 source edilir, sonra PYTHONPATH ayarlanır, sonra script çalışır
        cmd_vision = f"{ros2_setup} && {python_path_setup} && {sys.executable} '{vision_path}'"
        cmd_bridge = f"{ros2_setup} && {python_path_setup} && {sys.executable} '{bridge_path}'"

        p_vision = subprocess.Popen(cmd_vision, shell=True, executable="/bin/bash")
        child_processes.append(p_vision)
        print(f" -> Vision Node başlatıldı (PID: {p_vision.pid})")

        p_bridge = subprocess.Popen(cmd_bridge, shell=True, executable="/bin/bash")
        child_processes.append(p_bridge)
        print(f" -> GCS Bridge başlatıldı (PID: {p_bridge.pid})\n")

        print("[SYSTEM] Sistem aktif. Kapatmak için terminalde Ctrl+C yapın.")

        # 4. Ana döngüyü (Frame yakalama ve SHM yazma) başlat
        data_writer.run(zed)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Kullanıcı tarafından durduruluyor (Ctrl+C)...")
    finally:
        print("[SYSTEM] Temizlik işlemi başlatılıyor...")

        # Alt süreçleri güvenlice kapat (Zombi process kalmaması için)
        for p in child_processes:
            try:
                p.terminate()
                p.wait(timeout=2)  # Kapanmaları için en fazla 2 saniye bekle
            except Exception as e:
                print(f"[SYSTEM] Alt süreç kapatılırken hata oluştu: {e}")

        print("[SYSTEM] Alt süreçler (Vision & Bridge) kapatıldı.")

        # Kamerayı kapat
        zed.close()

        # Shared Memory (Paylaşımlı Bellek) segmentlerini temizleme
        try:
            shared_state._rgb_shm.close()
            shared_state._rgb_shm.unlink()
            shared_state._depth_shm.close()
            shared_state._depth_shm.unlink()
            shared_state._meta_shm.close()
            shared_state._meta_shm.unlink()
            print("[SYSTEM] Paylaşımlı bellek (Shared Memory) temizlendi.")
        except Exception:
            pass  # Eğer daha önceden temizlenmişse sessizce geç

        print("[SYSTEM] ZED kapatıldı. Tüm sistem güvenle durduruldu. İyi günler!")