# `move_robot` ROS 2 Humble example

This package starts a ROS 2 service named `/move_robot_service`. When the service
is called, the TurtleBot3 robot drives forward until odometry says it has moved
about 1 meter, then it stops.

## 1. Install dependencies

Install the TurtleBot3 simulation packages if they are not already installed:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-simulations
```

## 2. Build the package

Run this from the repository root:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/move_robot
source install/setup.bash
```

## 3. Terminal 1: start Gazebo

Leave this terminal running:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Gazebo should open. If the terminal prints an error like this:

```text
Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?
```

wait until Gazebo finishes loading, then manually spawn the robot from another
terminal:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run gazebo_ros spawn_entity.py \
  -entity burger \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  -x 0.0 -y 0.0 -z 0.01
```

If you see this message:

```text
Entity [burger] already exists.
```

that means the robot is already in Gazebo. Continue with the next step.

## 4. Terminal 2: start the `move_robot` node

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run move_robot move_robot_node
```

Expected output:

```text
Publisher, subscriber, and service are ready.
```

Keep this terminal running.

## 5. Terminal 3: check odometry

Before calling the service, check that Gazebo is publishing `/odom`:

```bash
source /opt/ros/humble/setup.bash
ros2 topic info /odom -v
```

You should see:

```text
Publisher count: 1
```

If the publisher count is `0`, the robot was not spawned correctly. Go back to
Terminal 1 and run the manual spawn command from step 3.

## 6. Call the service

From Terminal 3:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /move_robot_service move_robot/srv/MoveRobotService "{x_speed: 0.2, angular_speed: 0.0}"
```

Expected result:

```text
move_done=True
```

The robot should drive forward about 1 meter and then stop.

## Troubleshooting

### `move_done=False`

This usually means that `move_robot_node` has not received odometry yet.

Check:

```bash
ros2 topic info /odom -v
```

If `Publisher count` is `0`, spawn the robot manually:

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity burger \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  -x 0.0 -y 0.0 -z 0.01
```

If `Publisher count` is `1`, wait a few seconds and call the service again.

### `Entity [burger] already exists`

This is not a problem. It means the TurtleBot3 robot has already been spawned in
Gazebo. Do not spawn it again; continue with the node and service call.

### Gazebo opened, but the service still fails

The Gazebo window can open before the TurtleBot3 model is fully spawned. The
important check is not just whether the GUI is visible, but whether `/odom` has a
publisher:

```bash
ros2 topic info /odom -v
```

Only call `/move_robot_service` after `/odom` shows `Publisher count: 1`.
