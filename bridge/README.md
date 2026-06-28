# ROS2 Kullanım Kılavuzu

## Kontrol

### Topic'leri Listele

```bash
ros2 topic list
```

### Servisleri Listele

```bash
ros2 service list
```

### Beklenen Topic'ler

```text
/cube/gps
/cube/gps/heading
/cube/gps/relative_altitude
/cube/imu
/cube/battery
/cube/state
/cube/error
/cube/cmd_vel
```

### Beklenen Servisler

```text
/cube/arm
/cube/disarm
/cube/set_mode_service
```

---

# Topic Verilerini Okuma

## Araç Durumu

```bash
ros2 topic echo /cube/state
```

## GPS

```bash
ros2 topic echo /cube/gps
```

## Heading

```bash
ros2 topic echo /cube/gps/heading
```

## Batarya

```bash
ros2 topic echo /cube/battery
```

---

# Hareket Komutları

## İleri Hareket

```bash
ros2 topic pub --once /cube/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

## Dur

```bash
ros2 topic pub --once /cube/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Dönüş (Sağ / Sol)

`angular.z` değeri dönüş yönünü ve hızını belirler.

```bash
ros2 topic pub --once /cube/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.3}}"
```

---

# ARM / DISARM

## ARM

```bash
ros2 service call /cube/arm std_srvs/srv/Trigger
```

## DISARM

```bash
ros2 service call /cube/disarm std_srvs/srv/Trigger
```

---

# Mod Değiştirme

## MANUAL

```bash
ros2 service call /cube/set_mode_service mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'MANUAL'}"
```

## AUTO

```bash
ros2 service call /cube/set_mode_service mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO'}"
```

## GUIDED

```bash
ros2 service call /cube/set_mode_service mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
```

---

# Python İçinden Kullanım

Hazır yardımcı fonksiyonlar:

```
from utils.mavlink_utilities import (
    create_mission_topics,
    create_mission_clients,
    wait_for_mission_services,
    call_set_mode,
    call_trigger_service,
    publish_cmd_vel,
)
```

## Mission Node Kullanımı

```
self.topics = create_mission_topics(
    self,
    self.gps_callback,
    self.heading_callback,
    self.state_callback,
)

self.clients = create_mission_clients(self)
```

### Hareket Komutu

```
publish_cmd_vel(self.topics.cmd_vel_pub, 0.2, 0.0)
```

### ARM

```
call_trigger_service(self, self.clients.arm_client, "ARM")
```

### Mod Değiştirme

```
call_set_mode(self, self.clients.set_mode_client, "AUTO")
```
