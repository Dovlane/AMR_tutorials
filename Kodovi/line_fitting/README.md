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
rviz2 -d /home/vladimir/workspace/AMR_tutorials/Kodovi/line_fitting/rviz/line_fitting.rviz
```

This opens RViz with `base_scan`, `RobotModel`, `/scan`, and `/line_markers`
already configured.

If RViz fails with a `/snap/core20/...libpthread...` error, start it with the
Snap environment variables removed:

```bash
source /opt/ros/humble/setup.bash
env -u SNAP -u SNAP_NAME -u SNAP_ARCH -u SNAP_INSTANCE_NAME \
  -u SNAP_REVISION -u SNAP_VERSION -u SNAP_COMMON -u SNAP_CONTEXT \
  -u SNAP_COOKIE -u SNAP_DATA -u SNAP_EUID -u SNAP_LIBRARY_PATH \
  -u SNAP_REAL_HOME -u SNAP_UID -u SNAP_USER_COMMON -u SNAP_USER_DATA \
  -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE -u GTK_PATH \
  rviz2 -d /home/vladimir/workspace/AMR_tutorials/Kodovi/line_fitting/rviz/line_fitting.rviz
```

To configure RViz manually instead:

1. In the left `Displays` panel, open `Global Options`.
2. Click the `Fixed Frame` value. It is usually `map` by default.
3. Replace it with `base_scan` and press Enter.
4. Click `Add`, choose display type `RobotModel`, and click `OK`.
5. In the `RobotModel` display, set `Description Topic` to `/robot_description`
   if it is not already selected.
6. Click `Add`, choose display type `LaserScan`, and set topic to `/scan`.
7. Click `Add`, choose display type `MarkerArray`, and set topic to
   `/line_markers`.

The detected Split-and-Merge lines are shown as green line markers.

RViz does not automatically show the TurtleBot3 just because Gazebo is running.
The robot appears only after the `RobotModel` display is added and RViz can see
the `/robot_description` and `/tf` topics from the simulation.

If `base_scan` is red or does not work, check the scan frame:

```bash
ros2 topic echo /scan --once | grep frame_id
```

Then use that frame as the RViz `Fixed Frame`.

If the robot model still does not appear, check that these topics exist:

```bash
ros2 topic list | grep -E '(/tf|/robot_description|/scan)'
```

You should see at least:

```text
/tf
/tf_static
/robot_description
/scan
```

If `/robot_description` or `/tf` is missing, the TurtleBot3 Gazebo launch is not
publishing the robot model. Stop the simulation terminal, start it again, and
check that it has no errors:

```bash
cd /home/vladimir/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch Kodovi/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_maze.launch.py
```

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
