# ROS 2 Humble Tutorials

Ovaj branch (`ROS2`) prebacuje postojeće primere sa ROS 1 / Noetic na ROS 2 Humble uz zadržavanje iste organizacije paketa unutar direktorijuma `Kodovi/`.

Repo trenutno sadrži tri ROS 2 paketa:

- `Kodovi/hello_world`
- `Kodovi/move_robot`
- `Kodovi/line_fitting`

## VM Setup

Za kompletan vodič za podešavanje virtuelne mašine pogledati [VM_SETUP.md](VM_SETUP.md).

## Preduslovi

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- `colcon`

Instalacija osnovnih alata:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

Ako ROS 2 Humble još nije instaliran, pratiti zvaničnu proceduru i zatim učitati okruženje:

```bash
source /opt/ros/humble/setup.bash
```

## Build repozitorijuma

Pošto paketi ostaju u direktorijumu `Kodovi/`, build se pokreće iz root direktorijuma repozitorijuma:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/hello_world Kodovi/move_robot Kodovi/line_fitting
```

Posle uspešnog build-a:

```bash
source install/setup.bash
```

## ROS 2 radni tok

U ROS 2 više nije potreban `roscore`. Čvorovi komuniciraju preko DDS middlewara čim je ROS 2 okruženje pravilno učitano.

Najvažnije zamene u odnosu na ROS 1:

- `catkin_make` -> `colcon build`
- `rosrun` -> `ros2 run`
- `rosservice call` -> `ros2 service call`
- `rossrv show` -> `ros2 interface show`
- `roslaunch` -> `ros2 launch`

## Paketi

### `hello_world`

Primer publisher/subscriber komunikacije i jednostavnog servisa.

```bash
ros2 run hello_world hello_world_publisher
ros2 run hello_world hello_world_subscriber
ros2 run hello_world hello_world_service
```

Interfejs servisa:

```bash
ros2 interface show hello_world/srv/AddValueFile
```

Poziv servisa:

```bash
ros2 service call /add_value_file hello_world/srv/AddValueFile "{value: 115}"
```

### `move_robot`

Servis koji prihvata linearu i ugaonu brzinu, objavljuje `cmd_vel`, prati `odom` i zaustavlja robota posle približno 1 metra.

```bash
ros2 run move_robot move_robot_node
ros2 interface show move_robot/srv/MoveRobotService
ros2 service call /move_robot_service move_robot/srv/MoveRobotService "{x_speed: 0.2, angular_speed: 0.0}"
```

Za TurtleBot3 simulaciju pogledati README u paketu `move_robot`.

### `line_fitting`

Pretplata na `LaserScan` temu `/kobuki/laser/scan` i računanje parametara prave na osnovu prvih 10 merenja.

```bash
ros2 run line_fitting line_fitting_node
```

## Napomena o interfejsima

ROS 2 zahteva CamelCase nazive fajlova za custom interfejse, pa su servis fajlovi preimenovani u:

- `hello_world/srv/AddValueFile.srv`
- `move_robot/srv/MoveRobotService.srv`
