# Homework 2 Quick Start

This file contains the essential commands to run the homework_2 package with TurtleBot3 in Gazebo.

## Prerequisites
- ROS 2 Humble installed
- TurtleBot3 packages installed
- Gazebo installed

## 1. Build the Package

From the workspace root:

```bash
cd ~/workspace/AMR_tutorials
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/homework_2
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
```</content>
<parameter name="filePath">/home/vladimir/workspace/AMR_tutorials/Kodovi/homework_2/RUN.md