import threading
import pyzed.sl as sl
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
    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("ZED açılamadı")
    return zed

if __name__ == '__main__':
    zed = init_camera()

    threading.Thread(
        target=video_server.start,
        args=(5000,),
        daemon=True
    ).start()

    threading.Thread(
        target=data_server.start,
        args=(5001,),
        daemon=True
    ).start()

    print("ZED başlatıldı")
    print("Video stream  -> http://0.0.0.0:5000/video_feed")
    print("Data stream   -> http://0.0.0.0:5001/data/stream")

    try:
        data_writer.run(zed)
    except KeyboardInterrupt:
        print("Durduruluyor...")
    finally:
        zed.close()
        
        # Shared Memory segmentlerini temizleme işlemi (Plan Madde 5.3)
        try:
            shared_state._rgb_shm.close()
            shared_state._rgb_shm.unlink()
            shared_state._depth_shm.close()
            shared_state._depth_shm.unlink()
            shared_state._meta_shm.close()
            shared_state._meta_shm.unlink()
        except Exception as e:
            print(f"Shared memory temizlenirken hata: {e}")
            
        print("ZED kapatıldı, shared memory temizlendi")
