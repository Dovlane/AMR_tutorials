# `move_robot` u ROS 2 Humble

Paket izlaže servis `move_robot_service` koji pokreće robota dok ne pređe približno 1 metar na osnovu poruka sa teme `odom`.

## ROS 2 zavisnosti

Za TurtleBot3 simulaciju na ROS 2 Humble tipično su potrebni:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-simulations
```

Po potrebi podesiti model:

```bash
export TURTLEBOT3_MODEL=burger
```

## Build

Iz root direktorijuma repozitorijuma:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/move_robot
source install/setup.bash
```

## Pokretanje simulacije

U jednom terminalu pokrenuti Gazebo:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Ako lokalna instalacija koristi drugi launch fajl, zameniti ga odgovarajućim fajlom iz instalirane verzije `turtlebot3_gazebo`.

## Pokretanje noda

U drugom terminalu:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run move_robot move_robot_node
```

## Provera interfejsa i poziv servisa

Prikaz interfejsa:

```bash
ros2 interface show move_robot/srv/MoveRobotService
```

Poziv servisa:

```bash
ros2 service call /move_robot_service move_robot/srv/MoveRobotService "{x_speed: 0.2, angular_speed: 0.0}"
```

Servis vraća `move_done: true` kada prihvati zahtev. Robot zatim nastavlja kretanje dok ne pređe oko 1 metar, a zatim šalje nultu `Twist` poruku da bi se zaustavio.
