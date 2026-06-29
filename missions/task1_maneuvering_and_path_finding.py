from pathlib import Path

import rclpy
from rclpy.node import Node

# Yardımcı fonksiyonlar (Kendi yazdıklarımız ve mavlink_utilities içindekiler)
from utils.mavlink_utilities import (
    create_mission_topics,
    create_mission_clients,
    wait_for_mission_services,
    call_set_mode,
    call_trigger_service,
    publish_cmd_vel,
    stop_vehicle,
    calculate_gps_distance,
    calculate_bearing
)
from utils.read_json import main as read_json

BASE_DIR = Path(__file__).resolve().parent.parent
WAYPOINT_PATH = BASE_DIR / "tests" / "waypoints" / "waypoints.json"


# ============================================================
# GÖREV MANTIĞI (STATE MACHINE)
# ============================================================
class Task1Maneuvering:
    def __init__(self, node, mission_topics, mission_clients):
        self.node = node
        self.is_armed = False
        self.logger = node.get_logger()

        self.topics = mission_topics
        self.clients = mission_clients

        # Waypointleri JSON'dan oku
        self.waypoints = read_json(WAYPOINT_PATH)
        self.current_target_index = 0
        self.waypoint_tolerance = 1

        # Anlık konum verileri
        self.current_lat = None
        self.current_lon = None
        self.current_heading = 0.0
        self.last_angular_z = 0.0
        self.finished = False

    def update_gps(self, lat, lon, heading):
        """ROS 2 Node'undan gelen güncel GPS ve yönelim verilerini kaydeder."""
        self.current_lat = lat
        self.current_lon = lon
        self.current_heading = heading

    # noinspection D
    def update(self, detections):
        """Sürekli çalışan ana kontrol döngüsü."""
        if not self.waypoints:
            self.logger.warn("Görev listesi boş! Lütfen rotayı kontrol edin.")
            return

        if self.current_target_index >= len(self.waypoints):
            if not self.finished:
                self.logger.info("TÜM WAYPOINTLERE ULAŞILDI! GÖREV TAMAMLANDI!")
                stop_vehicle(self.topics.cmd_vel_pub)
                self.finished = True
            return

        if self.current_lat is None or self.current_lon is None:
            self.logger.info("GPS Verisi Bekleniyor...", throttle_duration_sec=2.0)
            publish_cmd_vel(self.topics.cmd_vel_pub, linear_x=0.0, angular_z=0.0)
            return

        target_gps = self.waypoints[self.current_target_index]
        target_lat = target_gps["lat"]
        target_lon = target_gps["lon"]

        # ---------------------------------------------------------
        # 1. ENGELLERDEN KAÇINMA KONTROLÜ
        # ---------------------------------------------------------
        if detections:
            for obj in detections:
                if obj["class"] == "red_buoy" and 0 < obj["distance"] < 3.0:
                    self.logger.info("Kırmızı şamandıra! Sancaktan kaçınılıyor.", throttle_duration_sec=1.0)
                    publish_cmd_vel(self.topics.cmd_vel_pub, linear_x=0.5, angular_z=-0.6)
                    return

                if obj["class"] == "green_buoy" and 0 < obj["distance"] < 3.0:
                    self.logger.info("Yeşil şamandıra! İskeleden kaçınılıyor.", throttle_duration_sec=1.0)
                    publish_cmd_vel(self.topics.cmd_vel_pub, linear_x=0.5, angular_z=0.6)
                    return

        # ---------------------------------------------------------
        # 2. MESAFE VE HEDEF KONTROLÜ
        # ---------------------------------------------------------
        distance = calculate_gps_distance(
            self.current_lat, self.current_lon,
            target_lat, target_lon
        )

        if self.current_target_index == 0 and distance < (self.waypoint_tolerance + 2.0):
            self.logger.info("WP0 (Start) noktasındayız, doğrudan bir sonraki hedefe geçiliyor.")
            self.current_target_index += 1
            return

        if distance < self.waypoint_tolerance:
            self.logger.info(f"Waypoint {self.current_target_index} varıldı! Kalan: {distance:.2f}m")
            self.current_target_index += 1
            return

        # ---------------------------------------------------------
        # 3. YÖN (HEADING) KONTROLÜ (P-Kontrolcü)
        # ---------------------------------------------------------
        target_bearing = calculate_bearing(
            self.current_lat, self.current_lon,
            target_lat, target_lon
        )

        # HATA AYIKLAMA LOGU (Sorunu görmek için her saniye ekrana basar)
        self.logger.info(
            f"Hedef WP{self.current_target_index} | Mesafe: {distance:.2f}m | Hata Açısı: {(target_bearing - self.current_heading):.1f}°",
            throttle_duration_sec=1.0
        )

        heading_error = target_bearing - self.current_heading

        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        abs_error = abs(heading_error)

        kp_angular = 0.015
        max_turn_speed = 0.4
        deadband_deg = 5.0

        if abs_error < deadband_deg:
            raw_angular_z = 0.0
        else:
            # KRİTİK DÜZELTME: Dönüş komutunu tersine çeviriyoruz (-1 ile çarpıyoruz)
            # ROS'ta +Z sola dönmektir. Eğer aracınız zıt yöne savruluyorsa sorun buradadır.
            raw_angular_z = -1.0 * (heading_error * kp_angular)

        raw_angular_z = max(-max_turn_speed, min(max_turn_speed, raw_angular_z))

        alpha = 0.25
        cmd_angular_z = self.last_angular_z + alpha * (raw_angular_z - self.last_angular_z)
        self.last_angular_z = cmd_angular_z

        proximity_threshold = self.waypoint_tolerance + 1.5

        if distance < proximity_threshold:
            cmd_linear_x = 0.20
        else:
            if abs_error > 70:
                cmd_linear_x = 0.12
            elif abs_error > 40:
                cmd_linear_x = 0.17
            elif abs_error > 20:
                cmd_linear_x = 0.20
            else:
                cmd_linear_x = 0.23

        publish_cmd_vel(
            self.topics.cmd_vel_pub,
            linear_x=cmd_linear_x,
            angular_z=cmd_angular_z
        )


# ============================================================
# ROS 2 NODE (GÖREV YÖNETİCİSİ)
# ============================================================
class Task1Node(Node):
    def __init__(self):
        super().__init__('task1_mission_node')
        self.get_logger().info("Task 1 (Maneuvering) Node Başlatılıyor...")

        # 1. Servis İstemcilerini (Clients) Oluştur ve Bekle
        self.mission_clients = create_mission_clients(self)
        wait_for_mission_services(self, self.mission_clients)

        # 2. Topic Aboneliklerini (Subscribers/Publishers) Oluştur
        self.mission_topics = create_mission_topics(
            self,
            gps_callback=self.gps_callback,
            heading_callback=self.heading_callback,
            state_callback=self.state_callback
        )

        # 3. Görev Sınıfını Başlat
        self.task = Task1Maneuvering(self, self.mission_topics, self.mission_clients)

        # Anlık Yönelim Değişkeni (GPS Callback'e aktarmak için)
        self.current_heading = 0.0

        # 4. Ana Kontrol Döngüsünü Başlat (Saniyede 10 kez çalışır: 0.1 sn)
        self.control_timer = self.create_timer(0.1, self.timer_callback)

    def gps_callback(self, msg):
        """Araçtan gelen NavSatFix verisini dinler."""
        self.task.update_gps(msg.latitude, msg.longitude, self.current_heading)

    def heading_callback(self, msg):
        """Araçtan gelen Float32 yön verisini dinler."""
        self.current_heading = msg.data

    def state_callback(self, msg):
        """Bridge'den gelen durum mesajlarını dinler (Gerekirse kullanılır)."""
        pass

    def timer_callback(self):
        """Görev mantığını sürekli tetikler."""
        # TODO: ZED kamerasından gelen tespitler (detections) buraya aktarılacak
        # Şimdilik boş bir liste göndererek sadece rotada ilerlemeyi test ediyoruz.
        current_detections = []

        self.task.update(detections=current_detections)


# ============================================================
# ANA ÇALIŞTIRMA BLOĞU
# ============================================================
def main(args=None):
    rclpy.init(args=args)

    node = Task1Node()

    try:
        node.get_logger().info("Araç MANUAL moda alınıyor...")
        call_set_mode(node, node.mission_clients.set_mode_client, "MANUAL")

        node.get_logger().info("Araç ARM ediliyor...")
        call_trigger_service(node, node.mission_clients.arm_client, "ARM")

        node.get_logger().info("Görev döngüsü başladı.")

        while rclpy.ok() and not node.task.finished:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.get_logger().info("Görev bitti. Araç durduruluyor.")
        stop_vehicle(node.mission_topics.cmd_vel_pub)

        node.get_logger().info("Araç DISARM ediliyor...")
        call_trigger_service(node, node.mission_clients.disarm_client, "DISARM")

    except KeyboardInterrupt:
        node.get_logger().info("Görev manuel olarak sonlandırıldı.")
        stop_vehicle(node.mission_topics.cmd_vel_pub)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
    Araç waypoint geçişlerini log vermiyor
    Araç 1. waypoint noktasına ilerlerdi ve saçmaladı
"""
