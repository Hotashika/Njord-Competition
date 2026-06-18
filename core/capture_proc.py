import pyzed.sl as sl
import numpy as np
from multiprocessing import shared_memory

def run_capture(rgb_shm_name, depth_shm_name, lock, frame_ready_event):
    zed = sl.Camera()
    init = sl.InitParameters()
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_units = sl.UNIT.METER
    init.camera_resolution = sl.RESOLUTION.HD2K
    init.camera_fps = 15
    zed.open(init)

    runtime = sl.RuntimeParameters()
    rgb_mat, depth_mat = sl.Mat(), sl.Mat()

    rgb_shm = shared_memory.SharedMemory(name=rgb_shm_name)
    depth_shm = shared_memory.SharedMemory(name=depth_shm_name)
    rgb_buf = np.ndarray((1080, 1920, 4), dtype=np.uint8, buffer=rgb_shm.buf)
    depth_buf = np.ndarray((1080, 1920), dtype=np.float32, buffer=depth_shm.buf)

    while True:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(rgb_mat, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
            with lock:
                rgb_buf[:] = rgb_mat.get_data()
                depth_buf[:] = depth_mat.get_data()
            frame_ready_event.set()
