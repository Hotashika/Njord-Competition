# mavlink_utilites.py
from pymavlink import mavutil


def arm_vehicle(master):
    """Aracı ARM eder (Motorlara güç verir)"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )

def arm_vehicle_force(connection):
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,      # Arm
        21196,  # Force arm param
        0, 0, 0, 0, 0
    )

def disarm_vehicle(master):
    """Aracı DISARM eder (Motorları kapatır)"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )


def set_mode(master, mode_name):
    """Aracın uçuş/sürüş modunu değiştirir (Örn: 'AUTO', 'MANUAL', 'HOLD')"""
    # MAVLink'in araç tipine göre mod ID'sini bul
    if mode_name not in master.mode_mapping():
        return False

    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    return True



def download_mission(master, logger=None):
    """Pixhawk'tan GPS noktalarını (Waypoints) indirir ve liste olarak döndürür."""
    waypoints = []

    # 1. Görev sayısını iste
    master.mav.mission_request_list_send(master.target_system, master.target_component)
    msg = master.recv_match(type=['MISSION_COUNT'], blocking=True, timeout=5)

    if not msg:
        if logger: logger.error("Görev sayısı alınamadı!")
        return waypoints

    count = msg.count
    if logger: logger.info(f"Otopilottan {count} adet görev noktası indiriliyor...")

    # 2. Döngüyle tüm noktaları çek
    for i in range(count):
        master.mav.mission_request_int_send(master.target_system, master.target_component, i)
        item_msg = master.recv_match(type=['MISSION_ITEM_INT', 'MISSION_ITEM'], blocking=True, timeout=5)

        if item_msg:
            lat = item_msg.x / 1e7
            lon = item_msg.y / 1e7
            command = item_msg.command
            waypoints.append({"seq": i, "lat": lat, "lon": lon, "cmd": command})

    return waypoints
