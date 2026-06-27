Kontrol:
ros2 topic list
ros2 service list
Beklemen gereken topic’ler:
/cube/gps
/cube/gps/heading
/cube/gps/relative_altitude
/cube/imu
/cube/battery
/cube/state
/cube/error
/cube/cmd_vel
Servisler:
/cube/arm
/cube/disarm
/cube/set_mode_service
Topic okumak için:
ros2 topic echo /cube/state
ros2 topic echo /cube/gps
ros2 topic echo /cube/gps/heading
ros2 topic echo /cube/battery
Hareket komutu göndermek için /cube/cmd_vel kullanacaksın:
ros2 topic pub --once /cube/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
Durmak için:
ros2 topic pub --once /cube/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
Sağa/sola dönüş için angular.z değişir:
ros2 topic pub --once /cube/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.3}}"
ARM:
ros2 service call /cube/arm std_srvs/srv/Trigger
DISARM:
ros2 service call /cube/disarm std_srvs/srv/Trigger
Mod değiştirme:
ros2 service call /cube/set_mode_service mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'MANUAL'}"
Örnek diğer modlar:
ros2 service call /cube/set_mode_service mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO'}"
ros2 service call /cube/set_mode_service mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
Kod içinden kullanmak istersen utils/mavlink_utilities.py içindeki hazır helper’ları kullanacaksın:
from utils.mavlink_utilities import (
    create_mission_topics,
    create_mission_clients,
    wait_for_mission_services,
    call_set_mode,
    call_trigger_service,
    publish_cmd_vel,
)
Mission node içinde mantık şu:
self.topics = create_mission_topics(
    self,
    self.gps_callback,
    self.heading_callback,
    self.state_callback,
)

self.clients = create_mission_clients(self)

publish_cmd_vel(self.topics.cmd_vel_pub, 0.2, 0.0)
call_trigger_service(self, self.clients.arm_client, "ARM")
call_set_mode(self, self.clients.set_mode_client, "AUTO")