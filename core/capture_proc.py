from multiprocessing import shared_memory

import numpy as np
import pyzed.sl as sl

from config.camera_config import (
    CAMERA_RESOLUTION,
    CAMERA_FPS,
    DEPTH_MODE,
    COORDINATE_UNITS,
    COORDINATE_SYSTEM,
    RGB_SHAPE,
    DEPTH_SHAPE,
)


def run_capture(rgb_shm_name, depth_shm_name, lock, frame_ready_event):
    # ------------------------------------------------------------------
    # Open ZED Camera
    # ------------------------------------------------------------------
    zed = sl.Camera()

    init = sl.InitParameters()
    init.camera_resolution = CAMERA_RESOLUTION
    init.camera_fps = CAMERA_FPS
    init.depth_mode = DEPTH_MODE
    init.coordinate_units = COORDINATE_UNITS
    init.coordinate_system = COORDINATE_SYSTEM

    status = zed.open(init)

    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Failed to open ZED camera: {status}")

    runtime = sl.RuntimeParameters()

    rgb_mat = sl.Mat()
    depth_mat = sl.Mat()

    # ------------------------------------------------------------------
    # Create Shared Memory
    # ------------------------------------------------------------------
    rgb_shm = shared_memory.SharedMemory(
        name=rgb_shm_name,
        create=True,
        size=int(np.prod(RGB_SHAPE) * np.dtype(np.uint8).itemsize),
    )

    depth_shm = shared_memory.SharedMemory(
        name=depth_shm_name,
        create=True,
        size=int(np.prod(DEPTH_SHAPE) * np.dtype(np.float32).itemsize),
    )

    rgb_buf = np.ndarray(
        RGB_SHAPE,
        dtype=np.uint8,
        buffer=rgb_shm.buf,
    )

    depth_buf = np.ndarray(
        DEPTH_SHAPE,
        dtype=np.float32,
        buffer=depth_shm.buf,
    )

    # ------------------------------------------------------------------
    # Capture Loop
    # ------------------------------------------------------------------
    try:
        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(rgb_mat, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

            with lock:
                rgb_buf[:] = rgb_mat.get_data()
                depth_buf[:] = depth_mat.get_data()

            frame_ready_event.set()

    finally:
        zed.close()

        rgb_shm.close()
        depth_shm.close()

        try:
            rgb_shm.unlink()
        except FileNotFoundError:
            pass

        try:
            depth_shm.unlink()
        except FileNotFoundError:
            pass
