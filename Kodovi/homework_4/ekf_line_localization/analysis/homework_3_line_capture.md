# Homework 3 Line Extraction Capture

This was captured by running the homework 3 `line_fitting` node against the
local `turtlebot3_maze` world with the robot spawned at `x=0`, `y=0`,
`yaw=0`.

Command:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 8 ros2 launch line_fitting line_fitting.launch.py log_period:=1.0
```

Observed output:

```text
Iterative Split-and-Merge: 13 lines, 10.704 ms
Line parameters:
01: rho=3.204 m, alpha=-0.014 rad (-0.8 deg), points=14, max_error=0.017 m
02: rho=0.830 m, alpha=1.551 rad (88.9 deg), points=9, max_error=0.024 m
03: rho=2.023 m, alpha=-0.001 rad (-0.1 deg), points=24, max_error=0.016 m
04: rho=1.988 m, alpha=1.573 rad (90.1 deg), points=56, max_error=0.035 m
05: rho=2.711 m, alpha=1.589 rad (91.1 deg), points=16, max_error=0.025 m
06: rho=1.318 m, alpha=3.129 rad (179.3 deg), points=16, max_error=0.009 m
07: rho=1.541 m, alpha=1.572 rad (90.0 deg), points=16, max_error=0.015 m
08: rho=1.969 m, alpha=1.776 rad (101.7 deg), points=6, max_error=0.066 m
09: rho=2.508 m, alpha=3.131 rad (179.4 deg), points=48, max_error=0.027 m
10: rho=0.801 m, alpha=-1.570 rad (-90.0 deg), points=14, max_error=0.013 m
11: rho=1.351 m, alpha=3.141 rad (180.0 deg), points=27, max_error=0.030 m
12: rho=1.994 m, alpha=-1.563 rad (-89.6 deg), points=89, max_error=0.024 m
13: rho=3.210 m, alpha=-0.001 rad (-0.1 deg), points=24, max_error=0.015 m
```

The original homework 3 logger prints only line parameters. To expose the final
wall segment coordinates, the same `extract_lines(...)` function was run on one
`/scan` message and each `LineSegment.start_point` / `LineSegment.end_point`
was printed.

These coordinates are in the laser frame `base_scan`:

| Line | rho [m] | alpha [rad] | start x [m] | start y [m] | end x [m] | end y [m] | points | max error [m] |
| ---: | ------: | ----------: | ----------: | ----------: | --------: | --------: | -----: | ------------: |
| 01 | 3.207 | 0.002 | 3.207 | -0.000 | 3.205 | 0.742 | 14 | 0.017 |
| 02 | 0.823 | 1.553 | 3.205 | 0.764 | 2.028 | 0.786 | 9 | 0.023 |
| 03 | 2.030 | 0.004 | 2.026 | 0.781 | 2.021 | 2.002 | 25 | 0.026 |
| 04 | 1.987 | 1.571 | 1.995 | 1.988 | -0.323 | 1.987 | 55 | 0.022 |
| 05 | 2.686 | 1.562 | -0.484 | 2.691 | -1.270 | 2.698 | 16 | 0.011 |
| 06 | 1.321 | 3.128 | -1.286 | 2.691 | -1.301 | 1.533 | 16 | 0.016 |
| 07 | 1.540 | 1.572 | -1.300 | 1.539 | -2.210 | 1.539 | 16 | 0.011 |
| 08 | 2.037 | 1.811 | -2.216 | 1.554 | -2.515 | 1.481 | 6 | 0.058 |
| 09 | 2.508 | 3.131 | -2.494 | 1.424 | -2.517 | -0.789 | 48 | 0.024 |
| 10 | 0.818 | -1.580 | -2.511 | -0.796 | -1.349 | -0.806 | 15 | 0.021 |
| 11 | 1.358 | -3.137 | -1.355 | -0.826 | -1.350 | -1.997 | 26 | 0.030 |
| 12 | 1.991 | -1.566 | -1.320 | -1.997 | 2.808 | -1.979 | 89 | 0.026 |
| 13 | 3.212 | -0.006 | 3.203 | -1.376 | 3.212 | -0.010 | 24 | 0.016 |

At the initial spawn pose, `base_scan` is approximately translated by
`x=-0.032 m`, `y=0.000 m` relative to `base_footprint`/`odom`. With the robot at
`x=0`, `y=0`, `yaw=0`, the approximate global segment coordinates are therefore
the same table with `0.032 m` subtracted from every x coordinate:

| Line | global start x [m] | global start y [m] | global end x [m] | global end y [m] |
| ---: | -----------------: | -----------------: | ---------------: | ---------------: |
| 01 | 3.175 | -0.000 | 3.173 | 0.742 |
| 02 | 3.173 | 0.764 | 1.996 | 0.786 |
| 03 | 1.994 | 0.781 | 1.989 | 2.002 |
| 04 | 1.963 | 1.988 | -0.355 | 1.987 |
| 05 | -0.516 | 2.691 | -1.302 | 2.698 |
| 06 | -1.318 | 2.691 | -1.333 | 1.533 |
| 07 | -1.332 | 1.539 | -2.242 | 1.539 |
| 08 | -2.248 | 1.554 | -2.547 | 1.481 |
| 09 | -2.526 | 1.424 | -2.549 | -0.789 |
| 10 | -2.543 | -0.796 | -1.381 | -0.806 |
| 11 | -1.387 | -0.826 | -1.382 | -1.997 |
| 12 | -1.352 | -1.997 | 2.776 | -1.979 |
| 13 | 3.171 | -1.376 | 3.180 | -0.010 |

These were originally observations `Z_t` in the robot-local laser frame at the
initial pose. After applying the initial `base_scan` offset, the corresponding
global line parameters were copied into `config/turtlebot3_maze_lines.yaml` as
the map `M` used by the EKF.

The EKF map YAML now uses the compact assignment-style `walls:` format, where
each entry is only `[alpha, r]`. The finite segment endpoints are kept in this
analysis note for traceability to the Homework 3 Split-and-Merge result.

The visible extracted lines match the derived map features around:

```text
x =  2.00 m, x =  3.18 m, x = -1.34 m, x = -2.54 m
y = -1.99 m, y = -0.81 m, y =  0.79 m, y =  1.55 m, y = 1.99 m, y = 2.70 m
```

Line 08 in the capture is short and has a larger fitting error, so it is marked
with a note in the YAML map.
