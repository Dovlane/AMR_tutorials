# Homework 4: Line-Based EKF Localization

This package implements the fourth AMR assignment:

- EKF prediction from TurtleBot3 wheel joint angles.
- Line measurement prediction from a global line map.
- Mahalanobis-gated line association.
- EKF correction and covariance update.
- ROS 2 node `ekf_line_localization` publishing `/ekf_pose`.
- Waypoint controller that can close the loop on `/ekf_pose` or `/odom`.

The scan line extraction is reused from homework 3 through the `line_fitting`
package.

## Important Files

```text
ekf_line_localization/ekf.py
ekf_line_localization/ekf_line_localization_node.py
ekf_line_localization/waypoint_controller.py
config/turtlebot3_maze_lines.yaml
config/waypoints.yaml
launch/homework_4.launch.py
```

The EKF functions from tasks 1-4 are available in
`ekf_line_localization/ekf.py` as Python-style functions:

```text
transition_function(...)
measurement_function(...)
associate_measurements(...)
filter_step(...)
```

## Build

From the workspace root:

```bash
cd /home/vladimir/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/homework_3/line_fitting Kodovi/homework_4/ekf_line_localization
source install/setup.bash
```

## Start The Maze

```bash
cd /home/vladimir/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch Kodovi/homework_3/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_maze.launch.py
```

## Run EKF Only

Use this for teleoperation verification from task 5:

```bash
source install/setup.bash
ros2 launch ekf_line_localization ekf_line_localization.launch.py
```

Then teleoperate in another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Useful topics:

```text
/ekf_pose
/ekf_map_lines
/ekf_association_count
/ekf_update_applied
```

## Run The Waypoint Mission

Configuration (b), EKF feedback with correction:

```bash
source install/setup.bash
ros2 launch ekf_line_localization homework_4.launch.py \
  feedback_topic:=/ekf_pose feedback_type:=pose enable_correction:=true
```

Configuration (c), EKF feedback with prediction only:

```bash
source install/setup.bash
ros2 launch ekf_line_localization homework_4.launch.py \
  feedback_topic:=/ekf_pose feedback_type:=pose enable_correction:=false
```

Configuration (a), odometry feedback while the EKF still runs for recording:

```bash
source install/setup.bash
ros2 launch ekf_line_localization homework_4.launch.py \
  feedback_topic:=/odom feedback_type:=odom enable_correction:=true
```

The waypoint list is in `config/waypoints.yaml`. The controller publishes
velocity commands to `/cmd_vel` and respects the TurtleBot3 Burger limits:
`|v| <= 0.22 m/s`, `|omega| <= 2.84 rad/s`.

## Record Experiment Bags

Run one bag per configuration:

```bash
ros2 bag record -o bags/hw4_b_ekf_corrected \
  /odom /ekf_pose /cmd_vel /scan /tf /tf_static /gazebo/model_states \
  /ekf_association_count /ekf_update_applied /waypoint_index
```

Use different output names for configurations (a), (b), and (c).

## Analysis Checklist

For task 7, compare the same waypoint mission in all three configurations.

1. Plot the Gazebo ground-truth trajectory from `/gazebo/model_states` and the
   reference waypoints. Compute the ground-truth distance to each waypoint when
   `/waypoint_index` advances.
2. For configuration (b), compare `/ekf_pose` and `/odom` against ground truth:
   position error `sqrt(dx^2 + dy^2)` and yaw error.
3. Plot EKF covariance diagonal terms from `/ekf_pose.pose.covariance`
   (`x`, `y`, `yaw`) and compare them with `/odom.pose.covariance`.
4. Mark correction times from `/ekf_update_applied`; inspect their effect on
   `/cmd_vel`. The controller includes speed saturation and optional rate
   limiting to reduce command jumps after EKF pose discontinuities.
5. Repeat configuration (b) with at least two `validation_gate` values, for
   example `2.0` and `4.0`, and compare `/ekf_association_count`.

The EKF uses constant measurement covariance
`sigma_alpha = 0.05 rad`, `sigma_r = 0.02 m`. This assumption is weakest when
scan lines are short, partly occluded, or observed at a shallow angle, so those
mission segments should show larger innovations and fewer stable associations.
