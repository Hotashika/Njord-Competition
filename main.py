import os
import shlex
import subprocess
import sys
import threading
import time

import pyzed.sl as sl

from core import data_writer
from core import shared_state
from servers import data_server
from servers import video_server

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))


def init_camera():
    zed = sl.Camera()
    init = sl.InitParameters()
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_units = sl.UNIT.METER
    init.camera_resolution = sl.RESOLUTION.VGA
    init.camera_fps = 15

    print("[SYSTEM] ZED Kamera baslatiliyor...")
    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("ZED acilamadi. Kamera baglantisini kontrol edin.")
    return zed


if __name__ == "__main__":
    zed = None
    child_processes = []

    try:
        zed = init_camera()
        # Flask
        threading.Thread(target=video_server.start, args=(5000,), daemon=True).start()
        threading.Thread(target=data_server.start, args=(5001,), daemon=True).start()

        print("[SYSTEM] ZED basariyla baslatildi.")
        print("[SYSTEM] Video stream  -> http://0.0.0.0:5000/video_feed")
        print("[SYSTEM] Data stream   -> http://0.0.0.0:5001/data/stream")

        print("\n[SYSTEM] Vision ve bridge node ROS2 ortaminda baslatiliyor...")
        time.sleep(1)

        ros2_setup = "source /opt/ros/kilted/setup.bash"
        python_path_setup = f"export PYTHONPATH={shlex.quote(PROJECT_ROOT)}:$PYTHONPATH"

        vision_path = os.path.join(PROJECT_ROOT, "vision", "vision_node.py")
        bridge_path = os.path.join(PROJECT_ROOT, "bridge", "bridge_node.py")

        task1_path = os.path.join(PROJECT_ROOT, "missions", "task1_maneuvering_and_path_finding.py")

        cmd_vision = (
            f"{ros2_setup} && {python_path_setup} && {shlex.quote(sys.executable)} {shlex.quote(vision_path)}"
        )
        cmd_bridge = (
            f"{ros2_setup} && {python_path_setup} && {shlex.quote(sys.executable)} {shlex.quote(bridge_path)}"
        )

        # ------------------------------
        #   TASK START # ---------------
        # ------------------------------
        cmd_task1 = (
            f"{ros2_setup} && {python_path_setup} && {shlex.quote(sys.executable)} {shlex.quote(task1_path)}"
        )
        # ------------------------------

        p_bridge = subprocess.Popen(cmd_bridge, shell=True, executable="/bin/bash")
        child_processes.append(p_bridge)
        print(f" -> Bridge Node baslatildi (PID: {p_bridge.pid})")

        p_vision = subprocess.Popen(cmd_vision, shell=True, executable="/bin/bash")
        child_processes.append(p_vision)
        print(f" -> Vision Node baslatildi (PID: {p_vision.pid})")

        time.sleep(2)

        # ------------------------------
        #   TASK START # ---------------
        # ------------------------------

        p_task1 = subprocess.Popen(cmd_task1, shell=True, executable="/bin/bash")
        child_processes.append(p_task1)
        print(f" -> Task 1 Node baslatildi (PID: {p_task1.pid})\n")
        # ------------------------------

        print("[SYSTEM] Sistem aktif. Kapatmak icin terminalde Ctrl+C yapin.")

        data_writer.run(zed)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Kullanici tarafindan durduruluyor (Ctrl+C)...")
    except Exception as exc:
        print(f"[SYSTEM] Hata olustu: {exc}")
        raise
    finally:
        print("[SYSTEM] Temizlik islemi baslatildi...")

        for p in child_processes:
            try:
                p.terminate()
                p.wait(timeout=2)
            except Exception as exc:
                print(f"[SYSTEM] Alt surec kapatilirken hata olustu: {exc}")

        print("[SYSTEM] Alt surecler kapatildi.")

        if zed is not None:
            zed.close()
            print("[SYSTEM] ZED kapatildi.")

        try:
            shared_state._rgb_shm.close()
            shared_state._rgb_shm.unlink()
            shared_state._depth_shm.close()
            shared_state._depth_shm.unlink()
            shared_state._meta_shm.close()
            shared_state._meta_shm.unlink()
            print("[SYSTEM] Paylasimli bellek temizlendi.")
        except Exception:
            pass

        print("[SYSTEM] Tum sistem guvenle durduruldu.")
