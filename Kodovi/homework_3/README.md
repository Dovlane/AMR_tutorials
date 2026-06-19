# Domaci 3 - Line Fitting

This folder contains the files needed for homework 3 from
`assignments/Domaći 3.docx`.

The assignment asks for line extraction from TurtleBot3 laser scan data using a
Split-and-Merge algorithm, terminal output of line distance/angle parameters,
and RViz visualization through `visualization_msgs/MarkerArray`.

## Contents

```text
line_fitting/
turtlebot3_simulations/
```

- `line_fitting` is the ROS 2 package with the iterative Split-and-Merge node.
- `turtlebot3_simulations` contains the local TurtleBot3 maze launch file,
  world, and maze wall model used for testing.

## Build

From the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/homework_3/line_fitting
source install/setup.bash
```

## Launch The Maze Simulation

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch Kodovi/homework_3/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_maze.launch.py
```

## Launch The Line Fitting Node

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch line_fitting line_fitting.launch.py
```

The node subscribes to `/scan`, prints detected line parameters, and publishes
markers on `/line_markers`.

## RViz

```bash
source /opt/ros/humble/setup.bash
rviz2 -d Kodovi/homework_3/line_fitting/rviz/line_fitting.rviz
```
