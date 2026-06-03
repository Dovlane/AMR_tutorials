# Domaci 3 - Iterative Split-and-Merge

This package solves the line fitting task from `assignments/Domaci 3.docx`.
The assignment asks for a Split-and-Merge algorithm on TurtleBot3 laser scan
data, terminal output of line parameters, and RViz visualization through
`visualization_msgs/MarkerArray`.

This solution uses the iterative Split-and-Merge algorithm.

## What It Does

- Subscribes to TurtleBot3 `LaserScan` data on `/scan`.
- Converts valid scan ranges into 2D points.
- Splits and merges scan segments into straight lines.
- Prints each line as distance and angle:
  `rho` is distance in meters, `alpha` is angle in radians.
- Prints execution time for the iterative algorithm.
- Publishes detected lines as green RViz markers on `/line_markers`.

## Important Files

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

## 1. Build The Line Fitting Package

Open a terminal:

```bash
cd /home/vladimir/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/line_fitting
source install/setup.bash
```

You only need to rebuild after changing Python/package files.

## 2. Start The TurtleBot3 Maze Simulation

Open a second terminal:

```bash
cd /home/vladimir/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch Kodovi/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_maze.launch.py
```

This launches the local maze file from this repository. You do not need to copy
`maze.world` into `/opt/ros` or another workspace for this command.

After Gazebo starts, check that laser data exists:

```bash
ros2 topic list | grep scan
```

Expected topic:

```text
/scan
```

## 3. Start The Line Fitting Node

Open a third terminal:

```bash
cd /home/vladimir/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch line_fitting line_fitting.launch.py
```

When `/scan` messages arrive, the terminal should print something like:

```text
Iterative Split-and-Merge: 4 lines, 1.240 ms
Line parameters:
01: rho=0.843 m, alpha=1.570 rad (90.0 deg), points=36, max_error=0.018 m
02: rho=1.514 m, alpha=0.002 rad (0.1 deg), points=42, max_error=0.021 m
```

## 4. Visualize Lines In RViz

Open a fourth terminal:

```bash
source /opt/ros/humble/setup.bash
rviz2
```

In RViz:

1. Set `Fixed Frame` to `base_scan`.
2. Add display type `LaserScan`, then set topic to `/scan`.
3. Add display type `MarkerArray`, then set topic to `/line_markers`.

The detected Split-and-Merge lines are shown as green line markers.

If `base_scan` does not work, check the scan frame:

```bash
ros2 topic echo /scan --once | grep frame_id
```

Then use that frame as the RViz `Fixed Frame`.

## Useful Parameters

The default launch parameters are:

```text
scan_topic       /scan
marker_topic     /line_markers
split_threshold  0.04
merge_threshold  0.04
min_points       6
max_point_gap    0.25
```

Examples:

```bash
ros2 launch line_fitting line_fitting.launch.py split_threshold:=0.03 merge_threshold:=0.03
ros2 launch line_fitting line_fitting.launch.py min_points:=8
ros2 launch line_fitting line_fitting.launch.py max_point_gap:=0.20
```

If your laser topic is not `/scan`, pass the topic explicitly:

```bash
ros2 launch line_fitting line_fitting.launch.py scan_topic:=/kobuki/laser/scan
```

## Assignment-Style Launch Command

The assignment document shows this command:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py
```

That command only works if `turtlebot3_maze.launch.py`, `maze.world`, and the
maze wall model are installed inside a `turtlebot3_gazebo` package in a sourced
ROS 2 workspace.

If you specifically want that command, copy the files into your TurtleBot3
source workspace:

```bash
TB3_GAZEBO=/home/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo

cp /home/vladimir/workspace/AMR_tutorials/Kodovi/turtlebot3_simulations/turtlebot3_gazebo/worlds/maze.world \
  "$TB3_GAZEBO/worlds/"

cp /home/vladimir/workspace/AMR_tutorials/Kodovi/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_maze.launch.py \
  "$TB3_GAZEBO/launch/"

cp -r /home/vladimir/workspace/AMR_tutorials/Kodovi/turtlebot3_simulations/turtlebot3_gazebo/models/nist_maze_wall_120 \
  "$TB3_GAZEBO/models/"
```

Then source that workspace and run the assignment command. For this repository,
the direct local launch command in step 2 is simpler.

## Troubleshooting

If `line_fitting` is not found:

```bash
cd /home/vladimir/workspace/AMR_tutorials
source install/setup.bash
ros2 pkg executables line_fitting
```

If no lines are printed, confirm that `/scan` is publishing:

```bash
ros2 topic hz /scan
```

If RViz shows no markers, confirm that `/line_markers` is publishing:

```bash
ros2 topic echo /line_markers --once
```
