# `hello_world` u ROS 2 Humble

Paket sadrži tri jednostavna primera:

- publisher na temi `hello_topic`
- subscriber na temi `hello_topic`
- servis `add_value_file`

## Struktura paketa

ROS 2 koristi `rclpy`, `colcon` i `ament`. Pošto paket definiše sopstveni servis, servis fajl je preimenovan u ROS 2 format:

```text
srv/AddValueFile.srv
```

## Build

Iz root direktorijuma repozitorijuma:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/hello_world
source install/setup.bash
```

## Pokretanje svih nodova pomoću launch fajla

Paket sadrži launch fajl koji pokreće sva tri noda odjednom:

- publisher
- subscriber
- servis

Pokretanje:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hello_world hello_world.launch.py
```

## Pokretanje publisher/subscriber primera

U prvom terminalu:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hello_world hello_world_publisher
```

U drugom terminalu:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hello_world hello_world_subscriber
```

## Pokretanje servisa

Serverska strana:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hello_world hello_world_service
```

Prikaz interfejsa:

```bash
ros2 interface show hello_world/srv/AddValueFile
```

Poziv servisa iz terminala:

```bash
ros2 service call /add_value_file hello_world/srv/AddValueFile "{value: 115}"
```

Servis upisuje vrednosti u sledeći fajl:

```text
~/.ros/hello_world/value_file.txt
```
