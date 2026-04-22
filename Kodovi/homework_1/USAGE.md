# homework_1 Usage

This file describes how to build and launch `homework_1`, and how to test the `brava` node.

## 1. Build the package

Open a terminal in the workspace root:

```bash
colcon build --packages-select homework_1
source install/setup.sh
ros2 launch homework_1 homework_1.launch.py
```

## 2. Check whether `brava` works

Run the brava individual:

```bash
ros2 run homework_1 brava
```

Lock the door:

```bash
ros2 service call /kuca/brava/zakljucaj std_srvs/srv/SetBool "{data: true}"
```

Unlock the door:

```bash
ros2 service call /kuca/brava/zakljucaj std_srvs/srv/SetBool "{data: false}"
```
