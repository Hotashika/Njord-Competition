from pathlib import Path

from utils.mavlink_utilities import (
    call_trigger_service,
    publish_cmd_vel,
    stop_vehicle,
    calculate_gps_distance,
    calculate_bearing
)
from utils.read_json import main as read_json

BASE_DIR = Path(__file__).resolve().parent.parent
WAYPOINT_PATH = BASE_DIR / "tests" / "waypoints" / "waypoints.json"


class Task1Maneuvering:
    def __init__(self, node, mission_topics, mission_clients, waypoints):
        self.node = node
        self.logger = node.get_logger()

        self.topics = mission_topics
        self.clients = mission_clients

        self.waypoints = read_json(WAYPOINT_PATH)
        self.current_target_index = 0
        self.waypoint_tolerance = 2.0

        self.current_lat = None
        self.current_lon = None
        self.current_heading = None

    def update_gps(self, lat, lon, heading=0.0):
        self.current_lat = lat
        self.current_lon = lon
        self.current_heading = heading

    # noinspection D
    def update(self, detections):
        if not self.waypoints:
            self.logger.warn("Görev listesi boş! Lütfen YKİ'den GPS rotası yükleyin.")
            return

        if self.current_target_index >= len(self.waypoints):
            self.logger.info("TÜM WAYPOINTLERE ULAŞILDI! GÖREV TAMAMLANDI!")
            stop_vehicle(self.topics.cmd_vel_pub)
            call_trigger_service(self.node, self.clients.disarm_client, 'DISARM')
            return

        if self.current_lat is None or self.current_lon is None:
            self.logger.info("GPS Data Waiting...")
            publish_cmd_vel(self.topics.cmd_vel_pub, linear_x=0.0, angular_z=0.0)
            return

        target_gps = self.waypoints[self.current_target_index]
        target_lat = target_gps["lat"]
        target_lon = target_gps["lon"]

        # ---------------------------------------------------------

        if detections:
            for obj in detections:
                if (
                        obj["class"] == "red_buoy" and
                        0 < obj["distance"] < 3.0
                ):
                    self.logger.info("Kırmızı şamandıra tespit edildi! Sancaktan (Sağdan) kaçınılıyor.")
                    publish_cmd_vel(self.topics.cmd_vel_pub, linear_x=0.5, angular_z=-0.6)

                    # TODO: Geçici waypoint atama
                    return

                if (
                        obj["class"] == "green_buoy" and
                        0 < obj["distance"] < 3.0
                ):
                    self.logger.info("Yeşil şamandıra tespit edildi! İskeleden (Soldan) kaçınılıyor.")
                    publish_cmd_vel(self.topics.cmd_vel_pub, linear_x=0.5, angular_z=0.6)

                    # TODO: Geçici waypoint atama

                    return

        # TODO: Rotada ilerle

        # ---------------------------------------------------------

        distance = calculate_gps_distance(
            self.current_lat, self.current_lon,
            target_lat, target_lon
        )

        if distance < self.waypoint_tolerance:
            self.logger.info(
                f"Waypoint {self.current_target_index} noktasına varıldı! Kalan mesafe: {distance:.2f}m")
            self.current_target_index += 1
            return

        target_bearing = calculate_bearing(
            self.current_lat, self.current_lon,
            target_lat, target_lon
        )

        heading_error = target_bearing - self.current_heading

        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        # KP value
        kp_angular = 0.02

        # Calculate angular velocity
        cmd_angular_z = heading_error * kp_angular

        # Limit turn speed to avoid too sharp turns
        max_turn_speed = 0.6
        cmd_angular_z = max(-max_turn_speed, min(max_turn_speed, cmd_angular_z))

        cmd_linear_x = 0.8
        if abs(heading_error) > 30:
            cmd_linear_x = 0.4

        publish_cmd_vel(self.topics.cmd_vel_pub, linear_x=cmd_linear_x, angular_z=cmd_angular_z)
