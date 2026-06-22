#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(realpath "$SCRIPT_DIR/../../../..")"
PACKAGE_DIR="$ROOT_DIR/Kodovi/homework_4/ekf_line_localization"
RVIZ_CONFIG="$PACKAGE_DIR/rviz/ekf_line_localization.rviz"
LOG_DIR="${LOG_DIR:-/tmp/amr_hw4_assignment5_test}"
PIDS=()

usage() {
  cat <<EOF
Usage:
  $0 all       Build, start Gazebo, EKF, map TF, RViz, then teleop.
  $0 build     Build homework 3 line_fitting and homework 4 EKF packages.
  $0 sim       Start TurtleBot3 maze simulation.
  $0 ekf       Start map->odom TF and EKF localization.
  $0 rviz      Start RViz with the EKF visualization config.
  $0 teleop    Start TurtleBot3 teleop_keyboard.
  $0 check     Print key topics and one /ekf_pose message.

Environment:
  SKIP_BUILD=1     Skip build in "all" mode.
  LOG_DIR=path     Directory for logs from "all" mode.
EOF
}

source_ros() {
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble setup not found at /opt/ros/humble/setup.bash" >&2
    exit 1
  fi

  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash

  if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROOT_DIR/install/setup.bash"
  fi

  export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
}

build_workspace() {
  source_ros
  cd "$ROOT_DIR"
  colcon build --base-paths \
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

run_sim() {
  source_ros
  cd "$ROOT_DIR"
  ros2 launch Kodovi/homework_3/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_maze.launch.py
}

run_map_tf() {
  source_ros
  # Visualization helper: the assignment map is aligned with odom at the
  # default simulator spawn pose. This creates the "map" frame RViz needs.
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
}

run_ekf() {
  trap cleanup EXIT INT TERM
  source_ros
  cd "$ROOT_DIR"
  start_background map_to_odom_tf run_map_tf
  ros2 launch ekf_line_localization ekf_line_localization.launch.py
}

run_rviz() {
  source_ros
  rviz2 -d "$RVIZ_CONFIG"
}

run_teleop() {
  source_ros
  ros2 run turtlebot3_teleop teleop_keyboard
}

run_check() {
  source_ros
  echo "Key topics:"
  ros2 topic list | grep -E '^/(scan|joint_states|odom|ekf_pose|ekf_map_lines|ekf_association_count|ekf_update_applied|tf|tf_static)$' || true
  echo
  echo "One /ekf_pose message:"
  timeout 5 ros2 topic echo /ekf_pose --once || true
  echo
  echo "TF map -> odom:"
  timeout 5 ros2 run tf2_ros tf2_echo map odom || true
}

run_all() {
  trap cleanup EXIT INT TERM
  source_ros

  if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
    build_workspace
    source_ros
  fi

  cd "$ROOT_DIR"
  start_background gazebo run_sim
  wait_for_topic /scan 45
  wait_for_topic /joint_states 45

  start_background map_to_odom_tf run_map_tf
  start_background ekf ros2 launch ekf_line_localization ekf_line_localization.launch.py
  wait_for_topic /ekf_pose 30

  start_background rviz run_rviz

  echo
  echo "Ready. RViz Fixed Frame should be 'map'."
  echo "Drive slowly with teleop; press Ctrl-C here to stop everything."
  echo
  run_teleop
}

main() {
  local command="${1:-all}"
  case "$command" in
    all) run_all ;;
    build) build_workspace ;;
    sim) run_sim ;;
    ekf) run_ekf ;;
    rviz) run_rviz ;;
    teleop) run_teleop ;;
    check) run_check ;;
    -h|--help|help) usage ;;
    *)
      usage
      exit 2
      ;;
  esac
}

main "$@"
