import rclpy
from pymavlink import mavutil
from rclpy.node import Node
from std_msgs.msg import String
from utils.mavlink_utilities import arm_vehicle, disarm_vehicle, set_mode, download_mission

class GCSBridgeNode(Node):
    def __init__(self):
        super().__init__('gcs_bridge')

        # Pixhawk bağlantısı
        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        self.master.wait_heartbeat()

        self.waypoints = []
        self.mission_started = False

        self.create_timer(0.1, self.mavlink_loop)

        self.waypoints = download_mission(self.master, self.get_logger())

    def mavlink_loop(self):
        msg = self.master.recv_match(blocking=False)
        if not msg:
            return

        if msg.get_type() == 'HEARTBEAT':
            is_auto = self.master.flightmode == 'AUTO'

            if is_auto and not self.mission_started:
                self.mission_started = True
                self.get_logger().info("Görev başladı, otonom sürüşe geçiliyor!")
                self.start_autonomous_tasks()

            elif not is_auto and self.mission_started:
                self.mission_started = False
                self.get_logger().info("Görev durduruldu!")
                self.stop_autonomous_tasks()

    def start_autonomous_tasks(self):
        arm_vehicle(self.master)
        self.get_logger().info("Motorlar ARM edildi.")

    def stop_autonomous_tasks(self):
        disarm_vehicle(self.master)
        self.get_logger().info("Motorlar DISARM edildi.")

def main():
    rclpy.init()
    node = GCSBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
