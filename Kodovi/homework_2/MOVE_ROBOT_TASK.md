# Task: `move_robot`

This document explains the `move_robot` task implemented in the
`homework_2` ROS 2 package. In this package there is no separate function named
`move_robot`; the movement task is implemented by the controller node in
`src/homework_2_controller.py`.

The goal of the task is to move a TurtleBot3 Burger robot in Gazebo using:

- manual keyboard commands,
- automatic motion to a target point,
- odometry feedback from the robot,
- velocity commands sent to the robot.

## Main Idea

The robot is controlled through the ROS 2 topic `/cmd_vel`. The controller
publishes `geometry_msgs/msg/Twist` messages to this topic.

A `Twist` message contains:

- `linear.x`: forward or backward velocity,
- `angular.z`: rotation velocity around the vertical axis.

For a differential-drive robot like TurtleBot3:

- positive `linear.x` moves the robot forward,
- negative `linear.x` moves the robot backward,
- positive `angular.z` rotates the robot left,
- negative `angular.z` rotates the robot right.

The controller does not guess where the robot is. It subscribes to `/odom`,
which provides `nav_msgs/msg/Odometry` messages from Gazebo. From odometry, the
controller extracts:

- robot position `x`,
- robot position `y`,
- robot heading angle `yaw`.

## ROS Nodes and Communication

The package uses two Python programs:

- `homework_2_controller`: the node that actually moves the robot,
- `homework_2_command`: the terminal client used by the user.

The launch file starts only the controller:

```bash
ros2 launch homework_2 homework_2.launch.py
```

The command client is normally started separately:

```bash
ros2 run homework_2 homework_2_command
```

The communication flow is:

```text
User keyboard / goal input
        |
        v
homework_2_command
        |
        | /robot_command_service
        v
homework_2_controller
        |
        | /cmd_vel
        v
TurtleBot3 in Gazebo
        |
        | /odom
        v
homework_2_controller
```

The important ROS interfaces are:

- service `/robot_command_service`,
- topic `/cmd_vel`,
- topic `/odom`.

## Service Request

The command client sends requests using the custom service
`homework_2/srv/RobotCommand`.

The service contains:

```srv
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

The available modes are:

| Value | Name | Meaning |
| --- | --- | --- |
| `1` | `MODE_MANUAL` | Control the robot from the keyboard |
| `2` | `MODE_AUTO` | Move the robot to a goal point |
| `3` | `MODE_STOP` | Stop the robot |

## Manual Mode

Manual mode is selected from the command client menu. The user controls the
robot with these keys:

| Key | Command | Robot Motion |
| --- | --- | --- |
| `w` | `MANUAL_FORWARD` | Move forward |
| `s` | `MANUAL_BACKWARD` | Move backward |
| `a` | `MANUAL_LEFT` | Rotate left |
| `d` | `MANUAL_RIGHT` | Rotate right |
| `space` | `MANUAL_STOP` | Stop |

When a key is pressed, the command client sends a service request to the
controller. The controller converts that request into a `Twist` message.

Examples:

```python
twist.linear.x = linear_speed
```

makes the robot move forward.

```python
twist.linear.x = -linear_speed
```

makes the robot move backward.

```python
twist.angular.z = angular_speed
```

makes the robot rotate left.

```python
twist.angular.z = -angular_speed
```

makes the robot rotate right.

The controller republishes the selected manual `Twist` every control cycle. If
new manual commands stop arriving for more than `0.6 s`, the controller stops
the robot automatically. This timeout is a safety feature.

## Automatic Mode

Automatic mode moves the robot to a target point in the global odometry frame.
The user enters:

- `goal_x`,
- `goal_y`,
- controller type.

Before accepting an automatic goal, the controller checks that:

- odometry has already been received,
- the controller type is valid,
- the goal coordinates are finite numbers.

If these checks pass, the controller stores the goal and enters automatic mode.

## Robot Pose From Odometry

Odometry gives the robot orientation as a quaternion. The controller converts
the quaternion to yaw because the robot moves on a 2D plane.

The yaw calculation is:

```python
siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
yaw = math.atan2(siny_cosp, cosy_cosp)
```

The controller then stores:

```python
pose.x
pose.y
pose.yaw
```

These values are used by the automatic controller.

## Automatic Control Algorithm

For a goal point `(goal_x, goal_y)` and current robot pose `(x, y, yaw)`, the
controller computes:

```text
dx = goal_x - x
dy = goal_y - y
rho = sqrt(dx^2 + dy^2)
```

`rho` is the distance from the robot to the goal.

The desired heading is:

```text
heading_to_goal = atan2(dy, dx)
```

The heading error is:

```text
alpha = heading_to_goal - yaw
```

The angle is normalized to the interval `[-pi, pi]`, so the robot always uses
the shortest angular correction.

The second angle from the polar-coordinate controller is:

```text
beta = -yaw - alpha
```

After normalization, this is equivalent to:

```text
beta = -heading_to_goal
```

`beta` is therefore the angle from the goal direction back to the inertial
x-axis. It is not the robot's global orientation and it is not set to zero.

The basic velocity law is:

```text
v = k_rho * rho
omega = k_alpha * alpha + k_beta * beta
```

Both `alpha` and `beta` are normalized before they are used by the controller.

## Controller Types

The package implements three automatic controller types.

### 1. Basic Closed-Loop Controller

The basic controller always drives forward toward the goal.

It uses:

```text
v = k_rho * rho
omega = k_alpha * alpha + k_beta * beta
```

This works well when the goal is in front of the robot. If the goal is behind
the robot, the robot first rotates toward the goal and then moves forward.

### 2. Reverse-Capable Controller

The reverse-capable controller checks whether the goal is behind the robot.

If:

```text
abs(alpha) > pi / 2
```

then the goal is mostly behind the robot. Instead of turning all the way around,
the controller drives backward.

It does this by:

- setting the motion direction to `-1`,
- adjusting `alpha` by approximately `pi`,
- producing a negative linear velocity.

This allows the robot to reach goals behind it more directly.

### 3. Constant-Speed Reverse-Capable Controller

The constant-speed controller starts with the same logic as the reverse-capable
controller. Then it scales the computed velocities so the linear speed stays
close to the selected target speed.

This avoids the robot becoming very slow when it approaches the goal. The
controller first computes the same speed-limited command as the reverse-capable
controller, then keeps the relationship between linear and angular velocity the
same while scaling to constant linear speed. That way the path shape matches the
normal reverse-capable controller.

## Velocity Limits

The controller clamps velocities for safety.

Default limits are:

```text
max_linear_speed = 0.22 m/s
max_angular_speed = 2.0 rad/s
```

After the normal controllers compute `v` and `omega`, they limit them before
publishing:

```python
linear_velocity = clamp_abs(linear_velocity, goal.linear_speed)
angular_velocity = clamp_abs(angular_velocity, goal.angular_speed)
```

This prevents the robot from receiving commands that are too large.
The constant-speed controller then scales this already-limited command, so it
preserves the published `v / omega` ratio that defines the path curvature.

## Goal Reached Condition

The controller checks the distance to the goal every control cycle.

If:

```text
rho <= position_tolerance
```

then the robot is considered to have reached the goal.

The default tolerance is:

```text
position_tolerance = 0.05 m
```

When the goal is reached, the controller:

- switches to stop mode,
- clears the active automatic goal,
- publishes a zero `Twist` message.

Publishing a zero `Twist` means:

```text
linear.x = 0
angular.z = 0
```

so the robot stops.

## Control Loop

The controller runs every:

```text
control_period = 0.05 s
```

This is equivalent to:

```text
20 Hz
```

At each cycle:

1. If the mode is manual, publish the current manual velocity command.
2. If the mode is automatic, compute a new velocity command from odometry and
   the goal.
3. If the mode is stop, do nothing unless a stop command was just received.

## Stop Behavior

The function `stop_robot()` is used whenever the robot must stop.

It:

- sets the mode to `MODE_STOP`,
- removes the active automatic goal,
- clears the manual command,
- publishes a zero velocity command.

This function is called when:

- the user selects stop,
- the user presses space in manual mode,
- manual commands time out,
- the automatic goal is reached,
- the ROS node shuts down.

## Important Constants

| Constant | Value | Meaning |
| --- | ---: | --- |
| `k_rho` | `0.8` | Gain for distance error |
| `k_alpha` | `2.8` | Gain for heading error |
| `k_beta` | `-0.6` | Gain for the `beta` angle in the polar-coordinate controller |
| `position_tolerance` | `0.05 m` | Distance at which the goal is considered reached |
| `manual_linear_speed` | `0.15 m/s` | Default manual forward/backward speed |
| `manual_angular_speed` | `0.8 rad/s` | Default manual turning speed |
| `max_linear_speed` | `0.22 m/s` | Maximum allowed linear speed |
| `max_angular_speed` | `2.0 rad/s` | Maximum allowed angular speed |
| `manual_timeout` | `0.6 s` | Safety timeout for manual mode |
| `control_period` | `0.05 s` | Control loop period |

## Summary

The `move_robot` task is the complete movement behavior of the homework
controller. It receives high-level user commands through a service, reads the
robot state from odometry, computes suitable velocity commands, and publishes
those commands to `/cmd_vel`.

Manual mode maps keyboard input directly to simple velocity commands. Automatic
mode uses odometry feedback and a closed-loop controller to drive the robot to a
goal point. The reverse-capable and constant-speed variants improve the basic
controller for goals behind the robot and for smoother motion near the goal.
