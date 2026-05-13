# Why the `move_robot` example is started in this order

This file explains the startup order from `README_new.md` and gives a more
general explanation of the Gazebo simulation environment used by this example.

## Big picture

The `move_robot` package does not create a robot by itself. It only creates a
ROS 2 node that:

- subscribes to `/odom`
- publishes velocity commands to `/cmd_vel`
- provides the `/move_robot_service` service

The simulated TurtleBot3 robot comes from Gazebo. Gazebo must be running, the
robot must be spawned inside the Gazebo world, and the robot's Gazebo plugins
must be publishing odometry before `move_robot_node` can do anything useful.

The data flow looks like this:

```text
ros2 service call
        |
        v
/move_robot_service
        |
        v
move_robot_node
        |
        | publishes velocity
        v
/cmd_vel
        |
        v
TurtleBot3 Gazebo diff-drive plugin
        |
        | publishes position estimate
        v
/odom
        |
        v
move_robot_node checks distance traveled
```

## Why each step comes before the next one

### 1. Source ROS 2 first

```bash
source /opt/ros/humble/setup.bash
```

This adds ROS 2 commands and packages to the current terminal environment.
Without this step, commands such as `ros2`, `colcon`, and ROS package lookup may
not work correctly.

This must be done in every new terminal because environment variables are local
to each terminal session.

### 2. Build the local package

```bash
colcon build --base-paths Kodovi/move_robot
source install/setup.bash
```

The package defines a custom service type:

```text
move_robot/srv/MoveRobotService
```

ROS 2 has to generate code for that service before the node can import and use
it. After building, `source install/setup.bash` tells the terminal about the
newly built local package.

If you skip this step, commands like these may fail:

```bash
ros2 run move_robot move_robot_node
ros2 interface show move_robot/srv/MoveRobotService
```

### 3. Start Gazebo before calling the service

```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Gazebo provides the simulated world, physics engine, and robot simulation. The
`move_robot` node expects a real or simulated robot to already exist somewhere
on the ROS graph.

If Gazebo is not running, there is no simulated robot, no `/odom` publisher, and
nothing listening to `/cmd_vel`.

### 4. Spawn the TurtleBot3 model

Starting Gazebo opens the world, but the robot model also has to exist inside
that world.

The TurtleBot3 launch file normally tries to spawn the robot automatically using
the Gazebo service:

```text
/spawn_entity
```

Sometimes Gazebo opens slowly. In that case, the spawn process may time out
before `/spawn_entity` is ready. The GUI can be visible while the robot is still
missing.

That is why the manual spawn command can be needed:

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity burger \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  -x 0.0 -y 0.0 -z 0.01
```

If Gazebo says:

```text
Entity [burger] already exists.
```

that means the robot has already been spawned. It is not an error that needs to
be fixed; it only means you should continue to the next step.

### 5. Check `/odom`

```bash
ros2 topic info /odom -v
```

The `/odom` topic tells `move_robot_node` where the robot is. The node uses this
information to measure when the robot has traveled about 1 meter.

The important line is:

```text
Publisher count: 1
```

That means some node is publishing odometry. In this example, the publisher
should be the TurtleBot3 Gazebo diff-drive plugin.

If the publisher count is `0`, then `move_robot_node` can subscribe to `/odom`,
but it will never receive any odometry messages. In that case, the service call
returns:

```text
move_done=False
```

### 6. Start `move_robot_node`

```bash
ros2 run move_robot move_robot_node
```

This starts the node from this package. It creates:

- a publisher on `/cmd_vel`
- a subscriber on `/odom`
- a service server on `/move_robot_service`

The node can start before or after the robot is spawned, but the service should
only be called after odometry is available. Otherwise, the node does not know the
robot's starting position.

### 7. Call the service last

```bash
ros2 service call /move_robot_service move_robot/srv/MoveRobotService "{x_speed: 0.2, angular_speed: 0.0}"
```

The service call is the final trigger. At this point:

- Gazebo should be running
- the TurtleBot3 robot should exist in the world
- `/odom` should have a publisher
- `move_robot_node` should be running
- `/move_robot_service` should exist

When the service is called, `move_robot_node` stores the current odometry pose as
the start pose. Then it publishes velocity commands on `/cmd_vel` until the
distance between the current pose and the start pose is about 1 meter.

## What Gazebo is

Gazebo is a robot simulation environment. It provides a virtual world where
robots can move, collide, sense, and interact with objects.

In this example, Gazebo simulates:

- the empty world
- the floor and lighting
- the TurtleBot3 model
- the robot's wheels
- the robot's differential-drive motion
- odometry
- the connection between simulated robot motion and ROS 2 topics

## Gazebo server and client

Gazebo is usually split into two main parts:

```text
gzserver
```

The server runs the simulation. It handles physics, models, sensors, plugins,
and time.

```text
gzclient
```

The client is the graphical window. It lets you see and interact with the
simulation.

This distinction is important: the GUI can be open even if some ROS-facing parts
of the simulation are not ready yet.

## Gazebo world

A Gazebo world describes the environment. It can contain things like:

- ground plane
- lights
- walls
- obstacles
- physics settings
- starting camera position

The example uses an empty TurtleBot3 world:

```text
/opt/ros/humble/share/turtlebot3_gazebo/worlds/empty_world.world
```

The world starts the environment, but it does not automatically guarantee that
the robot is already fully available on ROS topics.

## Gazebo models

A model describes an object inside Gazebo. The TurtleBot3 model contains the
robot body, wheels, links, joints, sensors, and plugins.

The Burger model used here is:

```text
/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
```

When you run `spawn_entity.py`, Gazebo reads this model file and inserts the
robot into the running simulation.

## Gazebo plugins

Plugins connect Gazebo simulation behavior to ROS 2.

For this example, the most important robot plugin is the differential-drive
plugin. It does two important things:

- subscribes to `/cmd_vel`
- publishes `/odom`

This is why the `move_robot` node can control the simulated robot without
directly talking to Gazebo internals. It only uses normal ROS 2 topics.

## ROS 2 topics used in this example

### `/cmd_vel`

This topic carries velocity commands.

`move_robot_node` publishes messages of type:

```text
geometry_msgs/msg/Twist
```

The important fields are:

```text
linear.x
angular.z
```

For example, this service request:

```text
{x_speed: 0.2, angular_speed: 0.0}
```

makes the robot drive forward with no rotation.

### `/odom`

This topic carries odometry, which is the robot's estimate of its movement.

The message type is:

```text
nav_msgs/msg/Odometry
```

`move_robot_node` reads the robot's current `x` and `y` position from `/odom`.
It compares the current position to the starting position to estimate how far
the robot has traveled.

## ROS 2 service used in this example

The service is:

```text
/move_robot_service
```

The custom service type is:

```text
move_robot/srv/MoveRobotService
```

The request contains:

```text
float64 x_speed
float64 angular_speed
```

The response contains:

```text
bool move_done
```

In this implementation, `move_done=True` means the node accepted the request,
moved the robot until the target distance was reached, and then stopped it.

## Common startup mistakes

### Calling the service before `/odom` is ready

Symptom:

```text
move_done=False
```

Reason:

`move_robot_node` has not received odometry yet, so it cannot know the robot's
starting pose.

### Thinking the Gazebo GUI means everything is ready

Symptom:

Gazebo is open, but the service still fails.

Reason:

The GUI is only the visual client. The robot may not be spawned yet, or the
robot plugins may not be publishing ROS topics yet.

### Spawning the robot twice

Symptom:

```text
Entity [burger] already exists.
```

Reason:

The robot is already inside the Gazebo world. Continue with the node and service
steps instead of spawning again.

## Practical rule

Before calling `/move_robot_service`, always check:

```bash
ros2 topic info /odom -v
```

If `/odom` has `Publisher count: 1`, the simulation side is ready enough for
this example.
