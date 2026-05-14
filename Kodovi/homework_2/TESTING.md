# Homework 2 Testing Guide

This file explains how to test the `homework_2` package in Gazebo with
TurtleBot3 Burger.

The goal of testing is to confirm that:

- Gazebo is publishing robot odometry on `/odom`,
- the Homework 2 controller accepts service requests,
- the controller publishes velocity commands to `/cmd_vel`,
- manual mode moves and stops the robot,
- automatic mode reaches global goal points,
- reverse mode drives backward when the goal is behind the robot,
- constant-speed mode reaches the goal without slowing heavily near the end.

## 1. Build the Package

From the workspace root:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/homework_2
source install/setup.bash
```

Expected result:

```text
Summary: 1 package finished
```

Check that ROS generated the service:

```bash
ros2 interface show homework_2/srv/RobotCommand
```

Expected result: the interface should show fields such as `mode`,
`manual_command`, `controller_type`, `goal_x`, and `goal_y`.

## 2. Start Gazebo

Open terminal 1:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Expected result:

- Gazebo GUI opens.
- TurtleBot3 Burger appears in the world.
- The robot is standing still.

If Gazebo opens but the robot does not appear, wait a few seconds and check
whether the spawn command timed out. The controller cannot work until the robot
exists in Gazebo.

## 3. Check Odometry

Open terminal 2:

```bash
source /opt/ros/humble/setup.bash
ros2 topic info /odom -v
```

Expected result:

```text
Type: nav_msgs/msg/Odometry
Publisher count: 1
```

This is important. The controller uses `/odom` to know where the robot is. If
`Publisher count` is `0`, automatic mode will reject the goal because the robot
pose is unknown.

You can also print one odometry message:

```bash
ros2 topic echo /odom --once --field pose.pose.position
```

Near the beginning of a fresh Gazebo world, the pose should be close to:

```text
x: 0.0
y: 0.0
```

Small non-zero values are normal because of simulation noise.

## 4. Start the Homework 2 Controller

Open terminal 3:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch homework_2 homework_2.launch.py
```

Expected result:

```text
Homework 2 controller ready: service /robot_command_service, topic /cmd_vel, topic /odom.
```

Check that the service exists:

```bash
ros2 service list | grep robot_command
```

Expected result:

```text
/robot_command_service
```

## 5. Reset the World Before Each Test

Before each movement test, reset the robot to the origin:

```bash
source /opt/ros/humble/setup.bash
ros2 service call /reset_world std_srvs/srv/Empty "{}"
sleep 1
```

Then confirm the pose:

```bash
ros2 topic echo /odom --once --field pose.pose.position
```

Expected result: `x` and `y` should be close to `0.0`.

## 6. Manual Mode Test

This test checks whether a manual service command moves the robot forward.

Record the start position:

```bash
ros2 topic echo /odom --once --field pose.pose.position
```

Send repeated forward manual commands for about two seconds:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
for i in {1..20}; do
  ros2 service call /robot_command_service homework_2/srv/RobotCommand \
    "{mode: 1, manual_command: 1, controller_type: 1, goal_x: 0.0, goal_y: 0.0, linear_speed: 0.0, angular_speed: 0.0}"
  sleep 0.1
done
```

Then stop the robot:

```bash
ros2 service call /robot_command_service homework_2/srv/RobotCommand \
  "{mode: 3, manual_command: 5, controller_type: 1, goal_x: 0.0, goal_y: 0.0, linear_speed: 0.0, angular_speed: 0.0}"
```

Record the end position:

```bash
ros2 topic echo /odom --once --field pose.pose.position
```

Expected result:

- `x` should increase.
- The robot should visibly move forward in Gazebo.
- The stop command should stop the robot.
- A single manual service call is not enough for a long movement. The
  controller has a `0.6 s` manual command timeout, so manual commands must be
  refreshed repeatedly.

Example observed result:

```text
start x ~= 0.00006
end   x ~= 0.09186
```

## 7. Command Script Manual Test

This test checks the terminal menu script.

Open terminal 4:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run homework_2 homework_2_command
```

Choose:

```text
1 - manual mode
```

Use:

```text
w - forward
s - backward
a - rotate left
d - rotate right
space - stop
q - return to menu
```

Expected result:

- `w` moves the robot forward.
- `s` moves the robot backward.
- `a` and `d` rotate the robot.
- `space` stops the robot.
- `q` returns to the main menu.

If the robot stops after you stop pressing keys, that is expected. The
controller has a manual command timeout for safety.

## 8. Basic Automatic Controller Test

Reset the world:

```bash
ros2 service call /reset_world std_srvs/srv/Empty "{}"
sleep 1
```

Send a goal in front of the robot:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /robot_command_service homework_2/srv/RobotCommand \
  "{mode: 2, manual_command: 5, controller_type: 1, goal_x: 0.5, goal_y: 0.0, linear_speed: 0.0, angular_speed: 0.0}"
```

Wait about 8 seconds, then check the final pose:

```bash
ros2 topic echo /odom --once --field pose.pose.position
```

Expected result:

- The robot should move forward.
- Final `x` should be close to `0.5`.
- The allowed tolerance is `0.05 m`, so anything from about `0.45` to `0.55`
  is acceptable.

Example observed result:

```text
goal: (0.5, 0.0)
end:  (0.466, 0.0)
```

## 9. Reverse Automatic Controller Test

Reset the world:

```bash
ros2 service call /reset_world std_srvs/srv/Empty "{}"
sleep 1
```

Send a goal behind the robot:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /robot_command_service homework_2/srv/RobotCommand \
  "{mode: 2, manual_command: 5, controller_type: 2, goal_x: -0.5, goal_y: 0.0, linear_speed: 0.0, angular_speed: 0.0}"
```

Wait about 8 seconds, then check the final pose:

```bash
ros2 topic echo /odom --once --field pose.pose
```

Expected result:

- The robot should move backward.
- Final `x` should be close to `-0.5`.
- The robot should not need to rotate all the way around before moving.

Example observed result:

```text
goal: (-0.5, 0.0)
end:  (-0.466, 0.0)
```

## 10. Constant-Speed Controller Test

Reset the world:

```bash
ros2 service call /reset_world std_srvs/srv/Empty "{}"
sleep 1
```

Send a diagonal goal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /robot_command_service homework_2/srv/RobotCommand \
  "{mode: 2, manual_command: 5, controller_type: 3, goal_x: 0.5, goal_y: 0.3, linear_speed: 0.0, angular_speed: 0.0}"
```

Wait about 12 seconds, then check the final pose:

```bash
ros2 topic echo /odom --once --field pose.pose.position
```

Expected result:

- The robot should move along a curved path toward the goal.
- Final position should be close to `(0.5, 0.3)`.
- The final error should be less than `0.05 m`.

Example observed result:

```text
goal: (0.5, 0.3)
end:  (0.485, 0.287)
```

## 11. Check Controller Logs

In the controller terminal, successful automatic tests should print messages
like:

```text
Goal reached: x=0.500, y=0.000.
Goal reached: x=-0.500, y=0.000.
Goal reached: x=0.500, y=0.300.
```

Manual timeout may print:

```text
Manual command timeout. Robot stopped.
```

That warning is normal if manual commands stop arriving.

## 12. Things That Can Go Wrong

### `/odom` has `Publisher count: 0`

The TurtleBot3 model is probably not spawned. Restart Gazebo or manually spawn
the robot.

### The service does not exist

The controller is probably not running, or the workspace was not sourced.

Check:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service list | grep robot_command
```

### The robot does not move

Check whether another node is also publishing to `/cmd_vel`:

```bash
ros2 topic info /cmd_vel -v
```

Also confirm the controller node is active:

```bash
ros2 node list | grep homework_2_controller
```

### Automatic command returns odometry error

If the service response says:

```text
Odometry has not been received yet.
```

wait a second and check `/odom` again. The controller must receive at least one
odometry message before accepting automatic goals.

## 13. Cleanup

Stop the robot before closing terminals:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /robot_command_service homework_2/srv/RobotCommand \
  "{mode: 3, manual_command: 5, controller_type: 1, goal_x: 0.0, goal_y: 0.0, linear_speed: 0.0, angular_speed: 0.0}"
```

Then close the command script, controller launch, and Gazebo launch terminals
with `Ctrl+C`.
