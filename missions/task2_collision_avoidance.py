# task2_collision_avoidance
from utils.mavlink_utilities import call_trigger_service, publish_cmd_vel, stop_vehicle


class Task1Maneuvering:
    def __init__(self, node, mission_topics, mission_clients, waypoints):
        # Setup connection & get waypoints from ROS 2 Mission Node
        self.node = node
        self.logger = node.get_logger()

        # Get services
        self.topics = mission_topics
        self.clients = mission_clients

        self.waypoints = waypoints
        self.current_target_index = 0

        def update(self, detections):

            if not self.waypoints:
                self.logger.warn("Görev listesi boş! Lütfen YKİ'den GPS rotası yükleyin.")
                return

            if self.current_target_index >= len(self.waypoints):
                self.logger.info("GÖREV TAMAMLANDI!")

                # Aracı durdur ve DISARM et
                stop_vehicle(self.topics.cmd_vel_pub)
                call_trigger_service(self.node, self.clients.disarm_client, 'DISARM')
                return

            target_gps = self.waypoints[self.current_target_index]

            # ---------------------------------------------------------

            if detections:
                for obj in detections:
                    if obj["class"] != "vessel" or not (0 < obj["distance"] < 4.5):
                        continue

                    side = self._classify_side(obj["bearing"])  # "head_on" | "left" | "right"

                    # TODO: Fill code
                    match side:
                        case "head_on":
                            self.logger.info("Vessel comes from across! Avoiding from right side.")
                            publish_cmd_vel(self.topics.cmd_vel_pub, linear_x=0.5, angular_z=-0.6)
                        case "left":
                            self.logger.info("Vessel comes from left side! Avoiding from right side.")
                            publish_cmd_vel(self.topics.cmd_vel_pub, linear_x=0.5, angular_z=0.6)
                        case "right":
                            self.logger.info("Vessel comes from right side! Avoiding from right side.")
                            publish_cmd_vel(self.topics.cmd_vel_pub, linear_x=0.5, angular_z=0.6)

                    # TODO: Geçici waypoint atama
                    return

            # TODO: Rotada ilerle

            publish_cmd_vel(self.topics.cmd_vel_pub, linear_x=0.8, angular_z=0.0)
