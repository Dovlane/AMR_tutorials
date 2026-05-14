# Homework 2 Quick Start

This file contains the essential commands to run the homework_2 package with TurtleBot3 in Gazebo.

## Prerequisites
- ROS 2 Humble installed
- TurtleBot3 packages installed
- Gazebo installed

## 1. Build or Rebuild the Package

From the workspace root:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/homework_2
source install/setup.bash
```

Run the same commands again after changing Python files, launch files, or the
`RobotCommand.srv` service. If Gazebo, the controller, or the command client is
already running, stop the affected nodes and start them again after the rebuild.

If the service definition changed, source the workspace again in every terminal:

```bash
source ~/workspace/AMR_tutorials/install/setup.bash
```

For a clean rebuild of only `homework_2`:

```bash
cd ~/workspace/AMR_tutorials
rm -rf build/homework_2 install/homework_2
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/homework_2
source install/setup.bash
```

For a clean rebuild of the whole workspace:

```bash
cd ~/workspace/AMR_tutorials
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 2. Start Gazebo with TurtleBot3

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

## 3. Start the Homework 2 Controller

In a new terminal:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch homework_2 homework_2.launch.py
```

## 4. Run the Command Client

In a new terminal:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run homework_2 homework_2_command
```

## Optional: Reset Robot Position

Before testing:

```bash
ros2 service call /reset_world std_srvs/srv/Empty "{}"
```

## Usage
- Use the command client menu to choose manual or automatic mode
- Manual mode: Use WASD keys to control the robot
- Automatic mode: Enter goal coordinates and controller type

## Stop the Robot

Send stop command from the client menu or:

```bash
ros2 service call /robot_command_service homework_2/srv/RobotCommand \
  "{mode: 3, manual_command: 5, controller_type: 1, goal_x: 0.0, goal_y: 0.0, linear_speed: 0.0, angular_speed: 0.0}"
```
