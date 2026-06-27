#!/usr/bin/env python3

import math
import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from pymavlink import mavutil

from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from std_msgs.msg import String, Float32

from bridge.gcs_bridge import (
    DEFAULT_BAUD,
    DEFAULT_CONNECTION_STRING,
    DEFAULT_HEARTBEAT_TIMEOUT,
    connect_mavlink,
)
from utils.mavlink_utilities import create_bridge_topics, create_bridge_services


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class OrangeCubeBridgeNode(Node):
    def __init__(self):
        super().__init__("orange_cube_bridge")

        self.declare_parameter(
            "connection_string",
            os.getenv("MAVLINK_CONNECTION_STRING", DEFAULT_CONNECTION_STRING),
        )
        self.declare_parameter("baud", int(os.getenv("MAVLINK_BAUD", str(DEFAULT_BAUD))))
        self.declare_parameter(
            "heartbeat_timeout",
            int(os.getenv("MAVLINK_HEARTBEAT_TIMEOUT", str(DEFAULT_HEARTBEAT_TIMEOUT))),
        )

        self.connection_string = self.get_parameter("connection_string").value
        self.baud = int(self.get_parameter("baud").value)
        self.heartbeat_timeout = int(self.get_parameter("heartbeat_timeout").value)

        self.master = None
        self.connected = False
        self.armed = False
        self.mode = "UNKNOWN"
        self.last_heartbeat_time = 0.0
        self.connection_timeout_sec = 3.0
        self.connection_lost_reported = False
        self.cmd_vel_ignored_reported = False

        self.gps_lat = None
        self.gps_lon = None
        self.gps_alt = None
        self.relative_alt = None
        self.heading_deg = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.voltage_v = None
        self.current_a = None
        self.battery_remaining = None

        self.last_cmd_vel_time = 0.0
        self.cmd_timeout_sec = 0.5
        self.last_steering_pwm = 1500
        self.last_throttle_pwm = 1500

        self.topics = create_bridge_topics(self, self._cmd_vel_callback)
        self.bridge_services = create_bridge_services(
            self,
            self._set_mode_callback,
            self._arm_callback,
            self._disarm_callback,
        )

        self._connect()

        self.create_timer(0.02, self._read_mavlink_messages)
        self.create_timer(0.2, self._publish_telemetry)
        self.create_timer(0.1, self._send_rc_override_loop)
        self.create_timer(1.0, self._connection_watchdog)

        self.get_logger().info("/cube topic ve servisleri aktif.")

    def _publish_error(self, text):
        msg = String()
        msg.data = str(text)
        self.topics.error_pub.publish(msg)
        self.get_logger().error(str(text))

    def _connect(self):
        try:
            self.master = connect_mavlink(
                connection_string=self.connection_string,
                baud=self.baud,
                heartbeat_timeout=self.heartbeat_timeout,
                logger=self.get_logger(),
            )
            self.connected = True
            self.last_heartbeat_time = time.time()
            self.connection_lost_reported = False
            self.cmd_vel_ignored_reported = False
        except Exception as exc:
            self.connected = False
            self._publish_error(f"MAVLink baglanti hatasi: {exc}")

    def _connection_watchdog(self):
        if self.master is None:
            self.connected = False
            if not self.connection_lost_reported:
                self.connection_lost_reported = True
                self.get_logger().warn("MAVLink baglantisi yok: master None")
            return

        if self.last_heartbeat_time == 0.0:
            return

        elapsed = time.time() - self.last_heartbeat_time
        if elapsed <= self.connection_timeout_sec:
            return

        self.connected = False
        if self.connection_lost_reported:
            return

        self.connection_lost_reported = True
        self.last_steering_pwm = 1500
        self.last_throttle_pwm = 1500
        self.last_cmd_vel_time = 0.0
        self._publish_error(
            f"MAVLink heartbeat kesildi. Son heartbeat {elapsed:.1f} saniye once alindi."
        )

    def _read_mavlink_messages(self):
        if self.master is None:
            return

        for _ in range(50):
            try:
                msg = self.master.recv_match(blocking=False)
                if msg is None:
                    return

                msg_type = msg.get_type()
                if msg_type == "BAD_DATA":
                    continue

                if msg_type == "HEARTBEAT":
                    self.mode = mavutil.mode_string_v10(msg)
                    self.armed = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    self.last_heartbeat_time = time.time()
                    if not self.connected or self.connection_lost_reported:
                        self.get_logger().info("MAVLink heartbeat tekrar alindi.")
                    self.connected = True
                    self.connection_lost_reported = False
                    self.cmd_vel_ignored_reported = False

                elif msg_type == "GLOBAL_POSITION_INT":
                    self.gps_lat = msg.lat / 1e7
                    self.gps_lon = msg.lon / 1e7
                    self.gps_alt = msg.alt / 1000.0
                    self.relative_alt = msg.relative_alt / 1000.0
                    if hasattr(msg, "hdg") and msg.hdg != 65535:
                        self.heading_deg = msg.hdg / 100.0

                elif msg_type == "VFR_HUD" and hasattr(msg, "heading"):
                    self.heading_deg = float(msg.heading)

                elif msg_type == "ATTITUDE":
                    self.roll = float(msg.roll)
                    self.pitch = float(msg.pitch)
                    self.yaw = float(msg.yaw)

                elif msg_type == "SYS_STATUS":
                    if msg.voltage_battery != 65535:
                        self.voltage_v = msg.voltage_battery / 1000.0
                    if msg.current_battery != -1:
                        self.current_a = msg.current_battery / 100.0
                    if msg.battery_remaining != -1:
                        self.battery_remaining = float(msg.battery_remaining)

            except Exception as exc:
                self._publish_error(f"MAVLink okuma hatasi: {exc}")
                return

    def _publish_telemetry(self):
        now = self.get_clock().now().to_msg()

        if self.gps_lat is not None and self.gps_lon is not None:
            gps_msg = NavSatFix()
            gps_msg.header.stamp = now
            gps_msg.header.frame_id = "gps"
            gps_msg.latitude = float(self.gps_lat)
            gps_msg.longitude = float(self.gps_lon)
            gps_msg.altitude = float(self.gps_alt) if self.gps_alt is not None else 0.0
            self.topics.gps_pub.publish(gps_msg)

        if self.heading_deg is not None:
            heading_msg = Float32()
            heading_msg.data = float(self.heading_deg)
            self.topics.gps_heading_pub.publish(heading_msg)

        if self.relative_alt is not None:
            alt_msg = Float32()
            alt_msg.data = float(self.relative_alt)
            self.topics.relative_alt_pub.publish(alt_msg)

        if self.roll is not None and self.pitch is not None and self.yaw is not None:
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = "base_link"
            qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            self.topics.imu_pub.publish(imu_msg)

        if self.voltage_v is not None:
            battery_msg = BatteryState()
            battery_msg.header.stamp = now
            battery_msg.voltage = float(self.voltage_v)
            if self.current_a is not None:
                battery_msg.current = float(self.current_a)
            if self.battery_remaining is not None:
                battery_msg.percentage = float(self.battery_remaining) / 100.0
            self.topics.battery_pub.publish(battery_msg)

        state_msg = String()
        state_msg.data = (
            f"connected={self.connected}, armed={self.armed}, mode={self.mode}"
        )
        self.topics.state_pub.publish(state_msg)

    def _set_mode_callback(self, request, response):
        if self.master is None:
            response.mode_sent = False
            return response

        mode_name = request.custom_mode
        mapping = self.master.mode_mapping()
        if mode_name not in mapping:
            self.get_logger().error(f"Bilinmeyen mod: {mode_name}")
            response.mode_sent = False
            return response

        try:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mapping[mode_name],
            )
            self.get_logger().info(f"Mod komutu gonderildi: {mode_name}")
            response.mode_sent = True
            return response
        except Exception as exc:
            self._publish_error(f"Mod degistirme hatasi: {exc}")
            response.mode_sent = False
            return response

    def _arm_callback(self, request, response):
        success = self._arm_disarm(True)
        response.success = success
        response.message = "ARM komutu gonderildi." if success else "ARM komutu basarisiz."
        return response

    def _disarm_callback(self, request, response):
        success = self._arm_disarm(False)
        response.success = success
        response.message = "DISARM komutu gonderildi." if success else "DISARM komutu basarisiz."
        return response

    def _arm_disarm(self, arm):
        if self.master is None:
            return False

        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1 if arm else 0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            self.get_logger().info("ARM komutu gonderildi." if arm else "DISARM komutu gonderildi.")
            return True
        except Exception as exc:
            self._publish_error(f"ARM/DISARM hatasi: {exc}")
            return False

    def _cmd_vel_callback(self, msg):
        if not self.connected:
            self.last_steering_pwm = 1500
            self.last_throttle_pwm = 1500
            self.last_cmd_vel_time = 0.0
            if not self.cmd_vel_ignored_reported:
                self.cmd_vel_ignored_reported = True
                self.get_logger().warn(
                    "MAVLink baglantisi yok. /cube/cmd_vel komutlari yok sayiliyor."
                )
            return

        self.cmd_vel_ignored_reported = False
        linear_x = max(-1.0, min(1.0, float(msg.linear.x)))
        angular_z = max(-1.0, min(1.0, float(msg.angular.z)))
        self.last_throttle_pwm = max(1100, min(1900, int(1500 + linear_x * 300)))
        self.last_steering_pwm = max(1100, min(1900, int(1500 - angular_z * 300)))
        self.last_cmd_vel_time = time.time()

    def _send_rc_override_loop(self):
        if self.master is None:
            return

        if not self.connected:
            self.last_steering_pwm = 1500
            self.last_throttle_pwm = 1500
            return

        if time.time() - self.last_cmd_vel_time > self.cmd_timeout_sec:
            steering_pwm = 1500
            throttle_pwm = 1500
        else:
            steering_pwm = self.last_steering_pwm
            throttle_pwm = self.last_throttle_pwm

        try:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                steering_pwm,
                0,
                throttle_pwm,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as exc:
            self._publish_error(f"RC override hatasi: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = OrangeCubeBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Bridge kapatiliyor...")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
