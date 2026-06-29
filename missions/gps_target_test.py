#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


# ============================================================
# TEST AYARLARI
# Hedef GPS, hiz, tolerans degerlerini buradan degistir.
# ============================================================

TARGET_LAT = -35.3625000
TARGET_LON = 149.1642000

TARGET_TOLERANCE = 30


FORWARD_SPEED = 0.20
SLOW_SPEED = 0.12
SLOW_DOWN_DISTANCE = 5.0

KP_TURN = 0.005
MAX_TURN = 0.22

# Arac ters yonde donerse bunu 1.0 yap.
TURN_SIGN = -1.0

MAX_TIME = 180.0


EARTH_RADIUS_M = 6371000.0


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def distance_between_gps(lat1, lon1, lat2, lon2):
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)

    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)

    a = (
        math.sin(d_lat / 2.0) ** 2
        + math.cos(lat1_rad)
        * math.cos(lat2_rad)
        * math.sin(d_lon / 2.0) ** 2
    )

    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS_M * c


def bearing_between_gps(lat1, lon1, lat2, lon2):
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    d_lon = math.radians(lon2 - lon1)

    x = math.sin(d_lon) * math.cos(lat2_rad)
    y = (
        math.cos(lat1_rad) * math.sin(lat2_rad)
        - math.sin(lat1_rad)
        * math.cos(lat2_rad)
        * math.cos(d_lon)
    )

    return math.degrees(math.atan2(x, y)) % 360.0


def is_valid_gps(lat, lon):
    if lat is None or lon is None:
        return False

    if abs(lat) < 0.000001 and abs(lon) < 0.000001:
        return False

    if lat < -90.0 or lat > 90.0:
        return False

    if lon < -180.0 or lon > 180.0:
        return False

    return True


class GpsTargetTest(Node):
    def __init__(self):
        super().__init__("gps_target_test")

        self.target_lat = TARGET_LAT
        self.target_lon = TARGET_LON
        self.target_tolerance = TARGET_TOLERANCE

        self.forward_speed = FORWARD_SPEED
        self.slow_speed = SLOW_SPEED
        self.slow_down_distance = SLOW_DOWN_DISTANCE

        self.kp_turn = KP_TURN
        self.max_turn = MAX_TURN
        self.turn_sign = TURN_SIGN

        self.max_time = MAX_TIME

        self.current_lat = None
        self.current_lon = None
        self.current_heading = None

        self.create_subscription(NavSatFix, "/cube/gps", self.gps_callback, 10)
        self.create_subscription(Float32, "/cube/gps/heading", self.heading_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cube/cmd_vel", 10)

        self.get_logger().info("GPS hedef test node baslatildi.")
        self.get_logger().info(
            f"Hedef GPS: lat={self.target_lat:.7f}, "
            f"lon={self.target_lon:.7f}, "
            f"tolerans={self.target_tolerance:.2f} m"
        )

    def gps_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude

        if not is_valid_gps(lat, lon):
            self.get_logger().warn(f"Gecersiz GPS yok sayildi: lat={lat}, lon={lon}")
            return

        self.current_lat = lat
        self.current_lon = lon

    def heading_callback(self, msg):
        self.current_heading = msg.data

    def publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def stop_vehicle(self):
        for _ in range(10):
            self.publish_cmd_vel(0.0, 0.0)
            time.sleep(0.05)

    def wait_for_data(self):
        self.get_logger().info("GPS ve heading bekleniyor...")

        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.current_lat is not None and self.current_heading is not None:
                self.get_logger().info(
                    f"Veri geldi: lat={self.current_lat:.7f}, "
                    f"lon={self.current_lon:.7f}, "
                    f"heading={self.current_heading:.2f}"
                )
                return True

            if time.time() - start > 20.0:
                self.get_logger().error("GPS veya heading gelmedi.")
                return False

        return False

    def calculate_control(self):
        distance = distance_between_gps(
            self.current_lat,
            self.current_lon,
            self.target_lat,
            self.target_lon,
        )

        bearing = bearing_between_gps(
            self.current_lat,
            self.current_lon,
            self.target_lat,
            self.target_lon,
        )

        heading_error = normalize_angle_deg(bearing - self.current_heading)

        raw_turn = heading_error * self.kp_turn

        angular_z = self.turn_sign * clamp(
            raw_turn,
            -self.max_turn,
            self.max_turn,
        )

        abs_error = abs(heading_error)

        if distance < self.slow_down_distance:
            linear_x = self.slow_speed
        elif abs_error > 30.0:
            linear_x = self.slow_speed
        else:
            linear_x = self.forward_speed

        return linear_x, angular_z, distance, bearing, heading_error

    def run_test(self):
        if self.target_lat == 0.0 and self.target_lon == 0.0:
            self.get_logger().error("Hedef GPS verilmedi.")
            return

        if not self.wait_for_data():
            return

        start_time = time.time()
        last_log = 0.0

        self.get_logger().info("GPS hedefe ilerleme basladi.")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.current_lat is None or self.current_heading is None:
                self.get_logger().warn("GPS/heading yok. Arac durduruluyor.")
                self.publish_cmd_vel(0.0, 0.0)
                time.sleep(0.2)
                continue

            linear_x, angular_z, distance, bearing, heading_error = self.calculate_control()

            if distance <= self.target_tolerance:
                self.get_logger().info(f"Hedefe ulasildi. Mesafe={distance:.2f} m")
                break

            if time.time() - start_time > self.max_time:
                self.get_logger().warn("Maksimum sure doldu. Test durduruluyor.")
                break

            self.publish_cmd_vel(linear_x, angular_z)

            if time.time() - last_log > 1.0:
                self.get_logger().info(
                    f"mesafe={distance:.2f} m, "
                    f"hedef_yon={bearing:.2f}, "
                    f"heading={self.current_heading:.2f}, "
                    f"hata={heading_error:.2f}, "
                    f"linear={linear_x:.2f}, "
                    f"angular={angular_z:.2f}"
                )
                last_log = time.time()

            time.sleep(0.2)

        self.stop_vehicle()
        self.get_logger().info("GPS hedef test tamamlandi.")


def main(args=None):
    rclpy.init(args=args)
    node = GpsTargetTest()

    try:
        node.run_test()
    except KeyboardInterrupt:
        node.stop_vehicle()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
