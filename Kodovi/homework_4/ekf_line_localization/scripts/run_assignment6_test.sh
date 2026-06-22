#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(realpath "$SCRIPT_DIR/../../../..")"
PACKAGE_DIR="$ROOT_DIR/Kodovi/homework_4/ekf_line_localization"
RVIZ_CONFIG="$PACKAGE_DIR/rviz/ekf_line_localization.rviz"
LOG_DIR="${LOG_DIR:-/tmp/amr_hw4_assignment6_test}"
WAYPOINT_FILE="${WAYPOINT_FILE:-$PACKAGE_DIR/config/waypoints.yaml}"
FEEDBACK_TOPIC="${FEEDBACK_TOPIC:-/ekf_pose}"
FEEDBACK_TYPE="${FEEDBACK_TYPE:-pose}"
ENABLE_CORRECTION="${ENABLE_CORRECTION:-true}"
VALIDATION_GATE="${VALIDATION_GATE:-3.0}"
MISSION_TIMEOUT="${MISSION_TIMEOUT:-240}"
START_RVIZ="${START_RVIZ:-1}"
PIDS=()

usage() {
  cat <<EOF
Usage:
  $0 all       Build, start Gazebo, map TF, EKF waypoint mission, RViz, and monitor completion.
  $0 build     Build homework 3 line_fitting and homework 4 EKF packages.
  $0 sim       Start TurtleBot3 maze simulation.
  $0 mission   Start map->odom TF and the assignment 6 waypoint mission.
  $0 rviz      Start RViz with the EKF visualization config.
  $0 check     Print key topics and current mission messages.
  $0 stop      Stop stale processes started by the homework 4 test helpers.

Environment:
  SKIP_BUILD=1          Skip build in "all" mode.
  CLEAN_START=0         Do not stop stale test processes before "all"/"mission" mode.
  START_RVIZ=0          Do not open RViz in "all" mode.
  MISSION_TIMEOUT=sec   Maximum time to wait for all waypoints. Default: 240.
  WAYPOINT_FILE=path    Waypoint YAML to run. Default: config/waypoints.yaml.
  FEEDBACK_TOPIC=topic  Controller feedback topic. Default: /ekf_pose.
  FEEDBACK_TYPE=pose    Use "pose" for /ekf_pose or "odom" for /odom.
  ENABLE_CORRECTION=0   Disable EKF correction. Default: true.
  VALIDATION_GATE=val   EKF association gate. Default: 3.0.
  LOG_DIR=path          Directory for logs from "all" mode.
EOF
}

source_ros() {
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble setup not found at /opt/ros/humble/setup.bash" >&2
    exit 1
  fi

  source_setup_file /opt/ros/humble/setup.bash

  if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
    source_setup_file "$ROOT_DIR/install/setup.bash"
  fi

  export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
}

source_setup_file() {
  local setup_file="$1"

  # ROS setup files may read optional variables that are unset. Keep nounset for
  # this script, but relax it while sourcing ROS-generated setup code.
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

build_workspace() {
  source_ros
  cd "$ROOT_DIR"
  colcon build --base-paths \
    Kodovi/homework_2 \
    Kodovi/homework_3/line_fitting \
    Kodovi/homework_4/ekf_line_localization
}

wait_for_topic() {
  local topic="$1"
  local timeout_seconds="${2:-30}"
  local start_time
  start_time="$(date +%s)"

  while true; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi

    if (( "$(date +%s)" - start_time >= timeout_seconds )); then
      echo "Timed out waiting for topic $topic" >&2
      return 1
    fi

    sleep 1
  done
}

start_background() {
  local name="$1"
  shift
  mkdir -p "$LOG_DIR"
  echo "Starting $name. Log: $LOG_DIR/$name.log"
  "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PIDS+=("$!")
}

cleanup() {
  if ((${#PIDS[@]})); then
    echo
    echo "Stopping background processes..."
    for pid in "${PIDS[@]}"; do
      kill "$pid" 2>/dev/null || true
    done
    wait "${PIDS[@]}" 2>/dev/null || true
  fi
}

stop_existing() {
  echo "Stopping stale homework 4 test processes..."
  pkill -TERM -f "ekf_line_localization/lib/ekf_line_localization/ekf_line_localization" 2>/dev/null || true
  pkill -TERM -f "ekf_line_localization/lib/ekf_line_localization/ekf_waypoint_controller" 2>/dev/null || true
  pkill -TERM -f "ros2 launch ekf_line_localization homework_4.launch.py" 2>/dev/null || true
  pkill -TERM -f "tf2_ros/static_transform_publisher .* map odom" 2>/dev/null || true
  pkill -TERM -f "turtlebot3_maze.launch.py" 2>/dev/null || true
  pkill -TERM -f "rviz2 .*ekf_line_localization.rviz" 2>/dev/null || true
  pkill -TERM -f "turtlebot3_teleop.*teleop_keyboard" 2>/dev/null || true
  pkill -TERM -f "robot_state_publisher .*__node:=robot_state_publisher" 2>/dev/null || true
  pkill -TERM -f "gzserver .*turtlebot3_gazebo/worlds/maze.world" 2>/dev/null || true
  pkill -TERM -f "gzclient" 2>/dev/null || true
  sleep 1
}

waypoint_count() {
  python3 - "$WAYPOINT_FILE" <<'PY'
from pathlib import Path
import sys
import yaml

path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
waypoints = data.get("waypoints", data)
print(len(waypoints))
PY
}

monitor_mission() {
  local total_waypoints="$1"
  local timeout_seconds="$2"

  echo
  echo "Monitoring /waypoint_index. Mission is complete at index $total_waypoints."
  echo

  python3 - "$total_waypoints" "$timeout_seconds" <<'PY'
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


target_index = int(sys.argv[1])
timeout_seconds = float(sys.argv[2])
start_time = time.monotonic()
last_index = None
completed = False


class WaypointMonitor(Node):
    def __init__(self):
        super().__init__("assignment6_waypoint_monitor")
        self.create_subscription(Int32, "/waypoint_index", self.callback, 10)

    def callback(self, message):
        global completed, last_index
        current_index = int(message.data)
        if current_index != last_index:
            print(f"Waypoint index: {current_index} / {target_index}", flush=True)
            last_index = current_index
        if current_index >= target_index:
            completed = True


rclpy.init()
node = WaypointMonitor()
try:
    while rclpy.ok() and not completed:
        rclpy.spin_once(node, timeout_sec=0.5)
        if time.monotonic() - start_time >= timeout_seconds:
            print(
                f"Mission timed out after {timeout_seconds:.0f}s. "
                f"Last waypoint index: {last_index}/{target_index}",
                file=sys.stderr,
                flush=True,
            )
            sys.exit(1)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

print()
print("Assignment 6 waypoint mission completed.")
PY
}

run_stop() {
  stop_existing
}

run_sim() {
  source_ros
  cd "$ROOT_DIR"
  ros2 launch Kodovi/homework_3/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_maze.launch.py
}

run_map_tf() {
  source_ros
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
}

run_mission() {
  trap cleanup EXIT INT TERM
  source_ros

  if [[ "${CLEAN_START:-1}" == "1" ]]; then
    stop_existing
    source_ros
  fi

  cd "$ROOT_DIR"
  start_background map_to_odom_tf run_map_tf
  ros2 launch ekf_line_localization homework_4.launch.py \
    waypoint_file:="$WAYPOINT_FILE" \
    feedback_topic:="$FEEDBACK_TOPIC" \
    feedback_type:="$FEEDBACK_TYPE" \
    enable_correction:="$ENABLE_CORRECTION" \
    validation_gate:="$VALIDATION_GATE"
}

run_rviz() {
  source_ros
  rviz2 -d "$RVIZ_CONFIG"
}

run_check() {
  source_ros
  echo "Key topics:"
  ros2 topic list | grep -E '^/(scan|joint_states|odom|ekf_pose|cmd_vel|waypoint_index|ekf_map_lines|ekf_association_count|ekf_update_applied|tf|tf_static)$' || true
  echo
  echo "Current /waypoint_index:"
  timeout 5 ros2 topic echo /waypoint_index --once || true
  echo
  echo "One /cmd_vel message:"
  timeout 5 ros2 topic echo /cmd_vel --once || true
  echo
  echo "One /ekf_pose message:"
  timeout 5 ros2 topic echo /ekf_pose --once || true
}

run_all() {
  trap cleanup EXIT INT TERM
  source_ros

  if [[ "${CLEAN_START:-1}" == "1" ]]; then
    stop_existing
  fi

  if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
    build_workspace
    source_ros
  fi

  local total_waypoints
  total_waypoints="$(waypoint_count)"
  if (( total_waypoints <= 0 )); then
    echo "No waypoints found in $WAYPOINT_FILE" >&2
    exit 1
  fi

  cd "$ROOT_DIR"
  start_background gazebo run_sim
  wait_for_topic /scan 45
  wait_for_topic /joint_states 45

  start_background map_to_odom_tf run_map_tf
  start_background assignment6_mission ros2 launch ekf_line_localization homework_4.launch.py \
    waypoint_file:="$WAYPOINT_FILE" \
    feedback_topic:="$FEEDBACK_TOPIC" \
    feedback_type:="$FEEDBACK_TYPE" \
    enable_correction:="$ENABLE_CORRECTION" \
    validation_gate:="$VALIDATION_GATE"
  wait_for_topic /ekf_pose 30
  wait_for_topic /cmd_vel 30
  wait_for_topic /waypoint_index 30

  if [[ "$START_RVIZ" == "1" ]]; then
    start_background rviz run_rviz
  fi

  echo
  echo "Assignment 6 test is running with $total_waypoints waypoints."
  echo "Logs are in $LOG_DIR."

  monitor_mission "$total_waypoints" "$MISSION_TIMEOUT"
}

main() {
  local command="${1:-all}"
  case "$command" in
    all) run_all ;;
    build) build_workspace ;;
    sim) run_sim ;;
    mission) run_mission ;;
    rviz) run_rviz ;;
    check) run_check ;;
    stop) run_stop ;;
    -h|--help|help) usage ;;
    *)
      usage
      exit 2
      ;;
  esac
}

main "$@"
