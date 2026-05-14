# Homework 2: Differential-Drive Robot Control

This directory contains the solution for AMR Homework 2. The task was to
implement kinematic control for a TurtleBot3 Burger robot with differential
drive in ROS 2 Humble and Gazebo.

The solution is implemented as one ROS 2 package named `homework_2`.

## What Was Implemented

The package implements all required parts from the assignment:

- manual mode, where the robot is controlled from the keyboard,
- automatic mode, where the user enters a goal point in global coordinates,
- communication between the command script and the controller through a ROS 2 Service,
- robot control through ROS 2 Topics,
- odometry feedback from `/odom`,
- velocity commands through `/cmd_vel`,
- a basic closed-loop controller,
- a reverse-capable controller for goals behind the robot,
- a constant-speed controller that keeps the same type of path while avoiding very slow motion near the goal.

The command script does not publish directly to `/cmd_vel`. It only sends
requests to the controller service. The controller node is the only part of this
package that communicates directly with Gazebo topics.

## Package Structure

```text
Kodovi/homework_2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── homework_2.launch.py
├── src/
│   ├── homework_2_command.py
│   └── homework_2_controller.py
└── srv/
    └── RobotCommand.srv
```

Important files:

- `src/homework_2_controller.py`: main robot controller node.
- `src/homework_2_command.py`: terminal command client for manual and automatic mode.
- `srv/RobotCommand.srv`: service definition used between the command script and controller.
- `launch/homework_2.launch.py`: starts the controller node.

## ROS Communication

The data flow is:

```text
homework_2_command
        |
        | /robot_command_service
        v
homework_2_controller
        |
        | publishes /cmd_vel
        v
Gazebo TurtleBot3 model
        |
        | publishes /odom
        v
homework_2_controller
```

The controller uses:

- `/odom` to read the robot position and orientation,
- `/cmd_vel` to send linear and angular velocity commands,
- `/robot_command_service` to receive manual, automatic, and stop commands.

## Service Interface

The service type is:

```bash
homework_2/srv/RobotCommand
```

The service name is:

```bash
/robot_command_service
```

The service definition is:

```srv
uint8 MODE_MANUAL=1
uint8 MODE_AUTO=2
uint8 MODE_STOP=3

uint8 MANUAL_FORWARD=1
uint8 MANUAL_BACKWARD=2
uint8 MANUAL_LEFT=3
uint8 MANUAL_RIGHT=4
uint8 MANUAL_STOP=5

uint8 CONTROLLER_BASIC=1
uint8 CONTROLLER_REVERSE=2
uint8 CONTROLLER_CONSTANT_SPEED=3

uint8 mode
uint8 manual_command
uint8 controller_type

float64 goal_x
float64 goal_y
float64 linear_speed
float64 angular_speed
---
bool success
string message
```

Useful values:

- `mode: 1`: manual mode,
- `mode: 2`: automatic mode,
- `mode: 3`: stop robot,
- `manual_command: 1`: forward,
- `manual_command: 2`: backward,
- `manual_command: 3`: rotate left,
- `manual_command: 4`: rotate right,
- `manual_command: 5`: stop,
- `controller_type: 1`: basic closed-loop controller,
- `controller_type: 2`: reverse-capable controller,
- `controller_type: 3`: constant-speed controller.

## Controller Details

The controller extracts the robot pose from `/odom`. The orientation quaternion
is converted to yaw, because the controller only needs planar motion.

Default parameters:

```text
k_rho = 0.8
k_alpha = 2.8
k_beta = -0.6
position_tolerance = 0.05 m
manual_linear_speed = 0.15 m/s
manual_angular_speed = 0.8 rad/s
max_linear_speed = 0.22 m/s
max_angular_speed = 2.0 rad/s
```

### Manual Mode

Manual mode supports:

- `w`: move forward,
- `s`: move backward,
- `a`: rotate left,
- `d`: rotate right,
- `space`: stop,
- `q`: return to the main menu.

The command script sends repeated service requests while manual mode is active.
The controller also has a timeout, so the robot stops automatically if fresh
manual commands stop arriving.

### Automatic Mode

In automatic mode the user enters a global goal point:

```text
goal_x
goal_y
```

The controller computes:

```text
rho = distance to the goal
heading_to_goal = atan2(goal_y - y, goal_x - x)
alpha = normalized angle from robot heading to goal direction
beta = normalized angle from goal direction back to the inertial x-axis
```

For the basic controller:

```text
v = k_rho * rho
omega = k_alpha * alpha + k_beta * beta
```

Following the polar-coordinate controller from *Introduction to Autonomous
Mobile Robots*, the angles are computed as:

```text
alpha = normalize(heading_to_goal - yaw)
beta = normalize(-yaw - alpha)
```

So `beta` is not the robot's global yaw and it is not set to zero. Since
`alpha` already contains `yaw`, the same expression can also be read as
`beta = normalize(-heading_to_goal)`.

For the reverse-capable controller, if the goal is behind the robot, the
controller chooses negative linear velocity and drives backward instead of
turning the robot around first.

For the constant-speed controller, the controller first computes the same
speed-limited command as the reverse-capable controller. It then scales linear
and angular velocity together so `linear.x` stays constant. This preserves the
published velocity ratio, so the path curvature matches the reverse-capable
controller.

## Build

From the workspace root:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/homework_2
source install/setup.bash
```

Check the generated service:

```bash
ros2 interface show homework_2/srv/RobotCommand
```

## Run With Gazebo

Open terminal 1 and start Gazebo:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Wait until the Gazebo GUI opens and TurtleBot3 Burger is spawned.

In another terminal, check odometry:

```bash
source /opt/ros/humble/setup.bash
ros2 topic info /odom -v
```

For automatic mode, `/odom` must show:

```text
Publisher count: 1
```

Open terminal 2 and start the controller:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch homework_2 homework_2.launch.py
```

Expected controller message:

```text
Homework 2 controller ready: service /robot_command_service, topic /cmd_vel, topic /odom.
```

Open terminal 3 and start the command script:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run homework_2 homework_2_command
```

## Manual Service Calls

You can also call the service without the menu script.

Stop the robot:

```bash
ros2 service call /robot_command_service homework_2/srv/RobotCommand \
  "{mode: 3, manual_command: 5, controller_type: 1, goal_x: 0.0, goal_y: 0.0, linear_speed: 0.0, angular_speed: 0.0}"
```

Move to a point in front of the robot with the basic controller:

```bash
ros2 service call /robot_command_service homework_2/srv/RobotCommand \
  "{mode: 2, manual_command: 5, controller_type: 1, goal_x: 0.5, goal_y: 0.0, linear_speed: 0.0, angular_speed: 0.0}"
```

Move backward to a goal behind the robot:

```bash
ros2 service call /robot_command_service homework_2/srv/RobotCommand \
  "{mode: 2, manual_command: 5, controller_type: 2, goal_x: -0.5, goal_y: 0.0, linear_speed: 0.0, angular_speed: 0.0}"
```

Use the constant-speed controller:

```bash
ros2 service call /robot_command_service homework_2/srv/RobotCommand \
  "{mode: 2, manual_command: 5, controller_type: 3, goal_x: 0.5, goal_y: 0.3, linear_speed: 0.0, angular_speed: 0.0}"
```

## Tested Behavior

The package was built and tested with Gazebo GUI running.

Build and interface checks:

```bash
colcon build --base-paths Kodovi/homework_2
ros2 interface show homework_2/srv/RobotCommand
ros2 run homework_2 homework_2_controller
```

All of these checks passed.

Movement tests in Gazebo:

```text
Manual forward test:
start x ~= 0.00006
end   x ~= 0.09186
result: robot moved forward

Basic automatic controller:
goal: (0.5, 0.0)
end:  (0.466, 0.0)
result: reached goal inside 0.05 m tolerance

Reverse automatic controller:
goal: (-0.5, 0.0)
end:  (-0.466, 0.0)
result: reached goal by driving backward

Constant-speed automatic controller:
goal: (0.5, 0.3)
end:  (0.485, 0.287)
result: reached goal inside 0.05 m tolerance
```

The controller log confirmed:

```text
Goal reached: x=0.500, y=0.000.
Goal reached: x=-0.500, y=0.000.
Goal reached: x=0.500, y=0.300.
```

## Troubleshooting

If automatic mode says odometry has not been received, check:

```bash
ros2 topic info /odom -v
```

If `Publisher count` is `0`, the TurtleBot3 model is not publishing odometry.
Usually this means the robot was not spawned correctly in Gazebo.

If the robot does not move, check:

```bash
ros2 topic info /cmd_vel -v
ros2 service list | grep robot_command
ros2 node list
```

Expected nodes include:

```text
/gazebo
/homework_2_controller
/turtlebot3_diff_drive
```

If manual mode stops by itself, that is expected. The controller stops the robot
when it does not receive fresh manual commands, so the robot does not keep moving
forever after the command script exits.
