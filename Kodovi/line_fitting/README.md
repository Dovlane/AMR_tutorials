# `line_fitting` u ROS 2 Humble

Paket služi za obradu prvih 10 tačaka sa `LaserScan` teme `/kobuki/laser/scan` i računanje parametara prave.

## Build

Iz root direktorijuma repozitorijuma:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/line_fitting
source install/setup.bash
```

## Pokretanje noda

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run line_fitting line_fitting_node
```

Node loguje izračunate vrednosti `r` i `alpha` čim pristignu poruke na temi `/kobuki/laser/scan`.
