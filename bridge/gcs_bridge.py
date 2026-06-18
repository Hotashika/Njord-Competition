import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class GCSBridgeNode(Node):
    def __init__(self):
        super().__init__('gcs_bridge')
        self.subscription = self.create_subscription(
            String,
            '/vision/detections',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        try:
            detections = json.loads(msg.data)
            for det in detections:
                class_name = det['class']
                distance = det['distance']
                bbox = det['bbox']
                # MAVLink kodları buraya eklenecek
        except Exception as e:
            self.get_logger().error(f"Detections verisi işlenirken hata: {e}")

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
