# Domaci 3 - Iterative Split-and-Merge Line Fitting

ROS 2 Humble package for extracting straight-line segments from TurtleBot3
`LaserScan` data in the maze simulation.

The node:

- subscribes to `/scan`,
- converts valid laser ranges to 2D points,
- runs the iterative Split-and-Merge algorithm,
- prints line parameters `rho` and `alpha` in the terminal,
- prints algorithm execution time,
- publishes detected line segments as `visualization_msgs/MarkerArray` on `/line_markers`.

## Files

Main implementation files:

```text
Kodovi/line_fitting/line_fitting/split_and_merge.py
Kodovi/line_fitting/line_fitting/line_fitting_node.py
Kodovi/line_fitting/launch/line_fitting.launch.py
```

Maze simulation files:

```text
Kodovi/turtlebot3_simulations/turtlebot3_gazebo/worlds/maze.world
Kodovi/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_maze.launch.py
Kodovi/turtlebot3_simulations/turtlebot3_gazebo/models/nist_maze_wall_120
```

## Build

Run from the workspace root:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/line_fitting
source install/setup.bash
```

## Prepare TurtleBot3 Maze

Copy the provided maze files into the `turtlebot3_gazebo` package used by your
ROS 2 workspace.

If you have a source workspace:

```bash
cp Kodovi/turtlebot3_simulations/turtlebot3_gazebo/worlds/maze.world \
  ~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/

cp Kodovi/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_maze.launch.py \
  ~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/

cp -r Kodovi/turtlebot3_simulations/turtlebot3_gazebo/models/nist_maze_wall_120 \
  ~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/
```

Then rebuild that workspace if needed:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Run Simulation

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/AMR_tutorials/install/setup.bash
ros2 launch line_fitting line_fitting.launch.py
```

## RViz Visualization

Start RViz:

```bash
rviz2
```

In RViz:

1. Set `Fixed Frame` to the laser scan frame, usually `base_scan`.
2. Add a display of type `LaserScan` and select `/scan`.
3. Add a display of type `MarkerArray` and select `/line_markers`.

The green marker lines represent the detected Split-and-Merge line segments.

## Parameters

The launch file supports these parameters:

```bash
ros2 launch line_fitting line_fitting.launch.py split_threshold:=0.03
ros2 launch line_fitting line_fitting.launch.py merge_threshold:=0.03
ros2 launch line_fitting line_fitting.launch.py min_points:=8
ros2 launch line_fitting line_fitting.launch.py max_point_gap:=0.20
ros2 launch line_fitting line_fitting.launch.py scan_topic:=/kobuki/laser/scan
```

Default values:

```text
scan_topic       /scan
marker_topic     /line_markers
split_threshold  0.04 m
merge_threshold  0.04 m
min_points       6
max_point_gap    0.25 m
```

## Expected Output

When scan data is received, the terminal prints output similar to:

```text
Iterative Split-and-Merge: 4 lines, 1.240 ms
Line parameters:
01: rho=0.843 m, alpha=1.570 rad (90.0 deg), points=36, max_error=0.018 m
02: rho=1.514 m, alpha=0.002 rad (0.1 deg), points=42, max_error=0.021 m
```

Here `rho` is the perpendicular distance from the robot frame origin to the
line, and `alpha` is the line normal angle in radians.
