# `move_robot_node.py` Explanation

This document explains the task implemented by
`Kodovi/move_robot/src/move_robot_node.py`.

The node exposes a ROS 2 service called `move_robot_service`. When the service
is called, the robot starts moving with the requested linear and angular
velocity. It keeps moving until odometry shows that the robot has traveled about
`1.0 m` from the position where the request started. Then the node publishes a
zero velocity command to stop the robot.

## Purpose of the Node

The task of `move_robot_node` is simple:

1. Wait for odometry from the robot.
2. Wait for a service request.
3. Save the robot's current position as the start position.
4. Publish velocity commands to move the robot.
5. Continuously measure the distance from the start position.
6. Stop the robot after it travels `1.0 m`.
7. Return a successful service response.

The node is written for a TurtleBot3-style differential-drive robot in ROS 2.

## Main ROS Interfaces

The node uses three ROS interfaces:

| Interface | Name | Type | Purpose |
| --- | --- | --- | --- |
| Publisher | `cmd_vel` | `geometry_msgs/msg/Twist` | Sends velocity commands to the robot |
| Subscriber | `odom` | `nav_msgs/msg/Odometry` | Reads the robot's current pose |
| Service | `move_robot_service` | `move_robot/srv/MoveRobotService` | Receives movement requests |

Although the code writes topic names as `cmd_vel` and `odom`, these normally
appear as `/cmd_vel` and `/odom` when the node is started in the root namespace.

## Service Definition

The custom service is defined in:

```text
Kodovi/move_robot/srv/MoveRobotService.srv
```

Its contents are:

```srv
float64 x_speed
float64 angular_speed
---
bool move_done
```

The request contains:

- `x_speed`: linear velocity in the robot's forward direction,
- `angular_speed`: angular velocity around the robot's vertical axis.

The response contains:

- `move_done`: `true` if the robot completed the movement, otherwise `false`.

Example service call:

```bash
ros2 service call /move_robot_service move_robot/srv/MoveRobotService "{x_speed: 0.2, angular_speed: 0.0}"
```

This asks the robot to move forward at `0.2 m/s` with no rotation.

## Class: `MoveRobotServiceNode`

The main class is:

```python
class MoveRobotServiceNode(Node):
```

It inherits from `rclpy.node.Node`, so it is a ROS 2 node.

Inside `__init__()`, the node creates:

- a publisher for velocity commands,
- a subscriber for odometry,
- a service server,
- a timer for the control loop.

The node name is:

```python
super().__init__("move_robot_service")
```

So the ROS node itself is named `move_robot_service`.

## Important Variables

The node stores several pieces of state:

```python
self.robot_pose = Pose()
```

Stores the latest robot pose received from odometry.

```python
self.initialized_pose = False
```

Becomes `True` after the first odometry message is received. This prevents the
service from starting before the node knows where the robot is.

```python
self.target_distance = 1.0
```

The robot should move until it has traveled `1.0 m` from the start pose.

```python
self.start_pose = None
```

Stores the robot pose at the moment the service request starts.

```python
self.active_request = None
```

Stores the active requested speeds while the robot is moving.

```python
self.motion_done = threading.Event()
```

Used to let the service callback wait until the timer-based control loop has
finished the movement.

## Odometry Callback

The odometry callback is:

```python
def odometry_callback(self, data_odom: Odometry) -> None:
    self.robot_pose = data_odom.pose.pose
    self.initialized_pose = True
```

Every time a new odometry message arrives, the node saves the robot's latest
pose.

The pose includes:

- position `x`,
- position `y`,
- orientation.

For this task, only `x` and `y` are used to calculate distance traveled.

## Service Callback

The service callback is:

```python
def move_robot_go(self, request, response):
```

This function runs when the user calls `/move_robot_service`.

First, it checks whether odometry has been received:

```python
if not self.initialized_pose:
    response.move_done = False
    return response
```

Without odometry, the node cannot know where the robot started, so it rejects
the request.

Next, it checks whether another motion is already active:

```python
if self.active_request is not None:
    response.move_done = False
    return response
```

This prevents two movement commands from running at the same time.

If the request is valid, the node saves the current position as the start pose:

```python
self.start_pose = Pose()
self.start_pose.position.x = self.robot_pose.position.x
self.start_pose.position.y = self.robot_pose.position.y
```

Then it stores the requested speeds:

```python
self.active_request = {
    "x_speed": request.x_speed,
    "angular_speed": request.angular_speed,
}
```

After that, the service callback waits until the movement is finished:

```python
while rclpy.ok() and not self.motion_done.wait(0.1):
    pass
```

When the movement finishes, the callback returns:

```python
response.move_done = True
```

This means the service call blocks until the robot has finished moving.

## Control Loop

The control loop runs every `0.1 s`:

```python
self.control_timer = self.create_timer(0.1, self.control_loop)
```

That is a control frequency of `10 Hz`.

If there is no active request, the control loop does nothing:

```python
if self.active_request is None or self.start_pose is None:
    return
```

If there is an active request, it calculates how far the robot has moved from
the start pose.

## Distance Calculation

The node calculates distance in the `x-y` plane:

```python
distance = math.sqrt(
    math.pow(self.robot_pose.position.x - self.start_pose.position.x, 2)
    + math.pow(self.robot_pose.position.y - self.start_pose.position.y, 2)
)
```

This is the Euclidean distance:

```text
distance = sqrt((current_x - start_x)^2 + (current_y - start_y)^2)
```

Only position is used. The robot's orientation does not affect the distance
calculation.

## Moving the Robot

If the measured distance is still less than or equal to `1.0 m`, the node
publishes the requested velocity:

```python
pub_cmd = Twist()
pub_cmd.linear.x = self.active_request["x_speed"]
pub_cmd.angular.z = self.active_request["angular_speed"]
self.publisher.publish(pub_cmd)
```

The meaning is:

- `linear.x` controls forward or backward speed,
- `angular.z` controls turning speed.

For example:

- `x_speed = 0.2`, `angular_speed = 0.0`: drive straight forward,
- `x_speed = -0.2`, `angular_speed = 0.0`: drive backward,
- `x_speed = 0.2`, `angular_speed = 0.5`: drive forward while turning left,
- `x_speed = 0.0`, `angular_speed = 0.5`: rotate in place.

Important detail: if the robot rotates in place with `x_speed = 0.0`, its
`x-y` position may barely change. Since this node stops based on traveled
position distance, pure rotation may not reach the `1.0 m` target.

## Stopping the Robot

When the distance becomes greater than `1.0 m`, the node stops the robot:

```python
self.publisher.publish(Twist())
```

An empty `Twist()` means:

```text
linear.x = 0
angular.z = 0
```

Then it clears the active movement:

```python
self.active_request = None
self.start_pose = None
self.motion_done.set()
```

The `motion_done.set()` call wakes the service callback, allowing it to return
`move_done: true`.

## Why a Multi-Threaded Executor Is Used

The node uses:

```python
executor = MultiThreadedExecutor()
```

It also uses:

```python
self.callback_group = ReentrantCallbackGroup()
```

This is important because the service callback waits until the robot finishes
moving. While it is waiting, the node still needs to process:

- odometry callbacks,
- timer callbacks.

Without a multi-threaded executor and reentrant callback group, the blocking
service callback could prevent the control loop from running. If the control
loop could not run, the robot would never move or finish the request.

## Shutdown Behavior

When the node is interrupted, the `finally` block publishes a zero `Twist`:

```python
node.publisher.publish(Twist())
```

This is a safety step. It tries to stop the robot before shutting down the node.

## Full Behavior Summary

The behavior of `move_robot_node` can be summarized as:

```text
Start node
  |
  v
Wait for odometry
  |
  v
Wait for service call
  |
  v
Save current pose as start pose
  |
  v
Publish requested velocity
  |
  v
Measure distance from start pose using odometry
  |
  v
If distance <= 1.0 m, keep moving
  |
  v
If distance > 1.0 m, publish zero velocity
  |
  v
Return move_done = true
```

## Limitations

This node is intentionally simple. Some limitations are:

- the target distance is fixed at `1.0 m`,
- the service does not allow the user to choose a target distance,
- there is no validation of maximum velocity,
- pure rotation may never finish because the stop condition depends on `x-y`
  displacement,
- the service call blocks until the movement is finished,
- the robot stops only after the measured distance becomes greater than
  `1.0 m`, so it may move slightly more than the target.

## Key Takeaway

`move_robot_node.py` is a service-based movement node. The user sends desired
linear and angular speeds through `/move_robot_service`; the node publishes
those speeds to `/cmd_vel`; it watches `/odom` to measure how far the robot has
moved; and after approximately one meter, it stops the robot and returns
`move_done: true`.
