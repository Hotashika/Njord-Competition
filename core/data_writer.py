import csv
import os
import time
import numpy as np
import pyzed.sl as sl
import cv2
import shared_state

OUTPUT_DIR = "output"
DEPTH_DIR = os.path.join(OUTPUT_DIR, "depth_frames")
CSV_PATH = os.path.join(OUTPUT_DIR, "imu_log.csv")

def setup_output_dirs():
    os.makedirs(DEPTH_DIR, exist_ok=True)

def run(zed):
    setup_output_dirs()

    runtime = sl.RuntimeParameters()
    image = sl.Mat()
    depth = sl.Mat()
    sensors_data = sl.SensorsData()

    frame_index = 0

    with open(CSV_PATH, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["timestamp", "pitch", "yaw", "roll", "depth_file"])

        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.IMAGE)
            timestamp_ms = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds()

            frame_bgr = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)

            depth_array = depth.get_data().copy()
            downsampled_depth = cv2.resize(depth_array, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

            imu_pose = sensors_data.get_imu_data().get_pose()
            pitch, yaw, roll = imu_pose.get_euler_angles()

            # numpy kaydet
            depth_filename = f"depth_{frame_index:05d}.npy"
            depth_path = os.path.join(DEPTH_DIR, depth_filename)
            np.save(depth_path, downsampled_depth)

            # CSV'ye yaz
            writer.writerow([timestamp_ms, pitch, yaw, roll, depth_filename])
            csvfile.flush()

            # shared_state güncelle
            with shared_state.data_lock:
                shared_state.latest_depth_array = downsampled_depth
                shared_state.latest_imu = {"pitch": pitch, "yaw": yaw, "roll": roll}
                shared_state.latest_timestamp = timestamp_ms

            with shared_state.frame_lock:
                shared_state.latest_frame = frame_bgr
                
                # Shared Memory'ye ZED'in orijinal VGA (376x672) çözünürlüklü verisini koyuyoruz
                shared_state.shm_rgb[:] = image.get_data()
                shared_state.shm_depth[:] = depth.get_data()
                shared_state.shm_meta[0] += 1
                shared_state.shm_meta[1] = int(time.time() * 1000)

            shared_state.data_event.set()
            shared_state.data_event.clear()
            shared_state.frame_event.set()
            shared_state.frame_event.clear()

            frame_index += 1
