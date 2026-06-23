#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(realpath "$SCRIPT_DIR/../../../..")"
PACKAGE_DIR="$ROOT_DIR/Kodovi/homework_4/ekf_line_localization"
RVIZ_CONFIG="$PACKAGE_DIR/rviz/ekf_line_localization.rviz"
BAGS_DIR="${BAGS_DIR:-$ROOT_DIR/bags}"
WAYPOINT_FILE="${WAYPOINT_FILE:-$PACKAGE_DIR/config/waypoints.yaml}"
MISSION_TIMEOUT="${MISSION_TIMEOUT:-240}"
START_RVIZ="${START_RVIZ:-0}"
BAG_START_DELAY="${BAG_START_DELAY:-5.0}"
RECORD_BAG="${RECORD_BAG:-1}"
VALIDATION_GATE="${VALIDATION_GATE:-5.0}"
BAG_TOPICS="${BAG_TOPICS:-/clock /joint_states /odom /ekf_pose /cmd_vel /scan /tf /tf_static /gazebo/model_states /ekf_association_count /ekf_update_applied /waypoint_index /ekf_map_lines}"
ANALYZE_AFTER_RUN="${ANALYZE_AFTER_RUN:-0}"
ANALYSIS_OUTPUT_DIR="${ANALYSIS_OUTPUT_DIR:-$PACKAGE_DIR/analysis/results}"
PIDS=()

usage() {
  cat <<EOF
Usage:
  $0 a|odom        Run assignment 7 configuration (a): odometry feedback.
  $0 b|corrected   Run assignment 7 configuration (b): EKF feedback with correction.
  $0 c|prediction  Run assignment 7 configuration (c): EKF feedback, prediction only.
  $0 build         Build homework 2, homework 3 line_fitting, and homework 4.
  $0 stop          Stop stale assignment 7/helper processes.
  $0 check         Print key topics and one sample mission message.

Environment:
  SKIP_BUILD=1          Skip build before a run.
  CLEAN_START=0         Do not stop stale processes before a run.
  START_RVIZ=1          Open RViz during the run. Default: 0.
  RECORD_BAG=0          Do not record a rosbag. Default: 1.
  BAGS_DIR=path         Bag output directory. Default: ./bags.
  BAG_NAME=name         Override generated bag folder name.
  BAG_TOPICS="..."      Override recorded topics.
  BAG_START_DELAY=sec   Controller delay used to let bag recording start. Default: 5.0.
  MISSION_TIMEOUT=sec   Maximum time to wait for all waypoints. Default: 240.
  VALIDATION_GATE=val   EKF association gate. Default: 5.0.
  WAYPOINT_FILE=path    Waypoint YAML to run. Default: config/waypoints.yaml.
  ANALYZE_AFTER_RUN=1   Generate analysis outputs for this bag after the mission.
  ANALYSIS_OUTPUT_DIR=path  Analysis output directory. Default: analysis/results.
  LOG_DIR=path          Directory for process logs. Default: /tmp/amr_hw4_assignment7_test/<config>.
EOF
}

source_setup_file() {
  local setup_file="$1"

  # ROS setup files may touch optional variables that are unset. Keep nounset
  # for this script, but relax it while sourcing ROS-generated setup code.
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
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
  echo "Stopping stale homework 4 assignment 7 processes..."
  pkill -TERM -f "ekf_line_localization/lib/ekf_line_localization/ekf_line_localization" 2>/dev/null || true
  pkill -TERM -f "ekf_line_localization/lib/ekf_line_localization/ekf_waypoint_controller" 2>/dev/null || true
  pkill -TERM -f "ros2 launch ekf_line_localization homework_4.launch.py" 2>/dev/null || true
  pkill -TERM -f "ros2 launch ekf_line_localization ekf_line_localization.launch.py" 2>/dev/null || true
  pkill -TERM -f "ros2 launch ekf_line_localization waypoint_mission.launch.py" 2>/dev/null || true
  pkill -TERM -f "ros2 bag record .*hw4_" 2>/dev/null || true
  pkill -TERM -f "tf2_ros/static_transform_publisher .* map odom" 2>/dev/null || true
  pkill -TERM -f "turtlebot3_maze.launch.py" 2>/dev/null || true
  pkill -TERM -f "rviz2 .*ekf_line_localization.rviz" 2>/dev/null || true
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
        super().__init__("assignment7_waypoint_monitor")
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
print("Assignment 7 run completed.")
PY
}

configure_assignment() {
  local selected="$1"
  local timestamp
  timestamp="$(date +%Y%m%d_%H%M%S)"

  case "$selected" in
    a|odom|odom_feedback)
      CONFIG_KEY="a"
      CONFIG_LABEL="Configuration (a): odometry feedback"
      CONFIG_ID="hw4_a_odom_feedback"
      CONFIG_FEEDBACK_TOPIC="/odom"
      CONFIG_FEEDBACK_TYPE="odom"
      CONFIG_ENABLE_CORRECTION="true"
      ;;
    b|corrected|ekf_corrected|ekf)
      CONFIG_KEY="b"
      CONFIG_LABEL="Configuration (b): EKF feedback with correction"
      CONFIG_ID="hw4_b_ekf_corrected"
      CONFIG_FEEDBACK_TOPIC="/ekf_pose"
      CONFIG_FEEDBACK_TYPE="pose"
      CONFIG_ENABLE_CORRECTION="true"
      ;;
    c|prediction|prediction_only|ekf_prediction)
      CONFIG_KEY="c"
      CONFIG_LABEL="Configuration (c): EKF feedback with prediction only"
      CONFIG_ID="hw4_c_ekf_prediction_only"
      CONFIG_FEEDBACK_TOPIC="/ekf_pose"
      CONFIG_FEEDBACK_TYPE="pose"
      CONFIG_ENABLE_CORRECTION="false"
      ;;
    *)
      usage
      exit 2
      ;;
  esac

  FEEDBACK_TOPIC="${FEEDBACK_TOPIC:-$CONFIG_FEEDBACK_TOPIC}"
  FEEDBACK_TYPE="${FEEDBACK_TYPE:-$CONFIG_FEEDBACK_TYPE}"
  ENABLE_CORRECTION="${ENABLE_CORRECTION:-$CONFIG_ENABLE_CORRECTION}"

  local gate_label
  gate_label="${VALIDATION_GATE//./p}"
  BAG_NAME="${BAG_NAME:-${CONFIG_ID}_gate_${gate_label}_${timestamp}}"
  BAG_OUTPUT="$BAGS_DIR/$BAG_NAME"
  LOG_DIR="${LOG_DIR:-/tmp/amr_hw4_assignment7_test/$CONFIG_ID}"
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

run_ekf() {
  source_ros
  cd "$ROOT_DIR"
  ros2 launch ekf_line_localization ekf_line_localization.launch.py \
    enable_correction:="$ENABLE_CORRECTION" \
    validation_gate:="$VALIDATION_GATE"
}

run_waypoint_controller() {
  source_ros
  cd "$ROOT_DIR"
  ros2 launch ekf_line_localization waypoint_mission.launch.py \
    waypoint_file:="$WAYPOINT_FILE" \
    feedback_topic:="$FEEDBACK_TOPIC" \
    feedback_type:="$FEEDBACK_TYPE" \
    start_delay_seconds:="$BAG_START_DELAY"
}

run_rviz() {
  source_ros
  rviz2 -d "$RVIZ_CONFIG"
}

run_bag_record() {
  source_ros
  mkdir -p "$BAGS_DIR"
  cd "$ROOT_DIR"
  # shellcheck disable=SC2086
  ros2 bag record --use-sim-time --include-unpublished-topics -o "$BAG_OUTPUT" $BAG_TOPICS
}

write_manifest() {
  if [[ "$RECORD_BAG" != "1" || ! -d "$BAG_OUTPUT" ]]; then
    return 0
  fi

  ASSIGNMENT7_BAG_TOPICS="$BAG_TOPICS" python3 - \
    "$BAG_OUTPUT" \
    "$CONFIG_KEY" \
    "$CONFIG_ID" \
    "$FEEDBACK_TOPIC" \
    "$FEEDBACK_TYPE" \
    "$ENABLE_CORRECTION" \
    "$VALIDATION_GATE" \
    "$WAYPOINT_FILE" <<'PY'
from pathlib import Path
import os
import sys
import yaml

bag_path = Path(sys.argv[1])
data = {
    "assignment": 7,
    "run_id": bag_path.name,
    "config_key": sys.argv[2],
    "config_id": sys.argv[3],
    "feedback_topic": sys.argv[4],
    "feedback_type": sys.argv[5],
    "enable_correction": sys.argv[6],
    "validation_gate": float(sys.argv[7]),
    "waypoint_file": sys.argv[8],
    "topics": os.environ["ASSIGNMENT7_BAG_TOPICS"].split(),
}
(bag_path / "assignment7_run.yaml").write_text(
    yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
    encoding="utf-8",
)
PY
}

run_analysis_current() {
  source_ros
  python3 "$PACKAGE_DIR/analysis/assignment7_analysis.py" \
    --bags "$BAG_OUTPUT" \
    --output-dir "$ANALYSIS_OUTPUT_DIR" \
    --waypoint-file "$WAYPOINT_FILE" \
    --map-file "$PACKAGE_DIR/config/turtlebot3_maze_lines.yaml"
}

run_check() {
  source_ros
  echo "Key topics:"
  ros2 topic list | grep -E '^/(clock|scan|joint_states|odom|ekf_pose|cmd_vel|waypoint_index|ekf_map_lines|ekf_association_count|ekf_update_applied|gazebo/model_states|tf|tf_static)$' || true
  echo
  echo "Current /waypoint_index:"
  timeout 5 ros2 topic echo /waypoint_index --once || true
  echo
  echo "One /ekf_pose message:"
  timeout 5 ros2 topic echo /ekf_pose --once || true
}

run_assignment() {
  local selected="$1"
  configure_assignment "$selected"
  trap cleanup EXIT INT TERM
  source_ros

  if [[ "${CLEAN_START:-1}" == "1" ]]; then
    stop_existing
    source_ros
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

  echo
  echo "$CONFIG_LABEL"
  echo "  feedback_topic=$FEEDBACK_TOPIC"
  echo "  feedback_type=$FEEDBACK_TYPE"
  echo "  enable_correction=$ENABLE_CORRECTION"
  echo "  validation_gate=$VALIDATION_GATE"
  if [[ "$RECORD_BAG" == "1" ]]; then
    echo "  bag=$BAG_OUTPUT"
  fi
  echo

  cd "$ROOT_DIR"
  start_background gazebo run_sim
  wait_for_topic /clock 45
  wait_for_topic /scan 45
  wait_for_topic /joint_states 45
  wait_for_topic /odom 45
  wait_for_topic /gazebo/model_states 45

  start_background map_to_odom_tf run_map_tf
  start_background assignment7_ekf run_ekf
  wait_for_topic /ekf_pose 30
  wait_for_topic /ekf_map_lines 30
  wait_for_topic /ekf_association_count 30
  wait_for_topic /ekf_update_applied 30

  start_background assignment7_waypoints run_waypoint_controller
  wait_for_topic /cmd_vel 30
  wait_for_topic /waypoint_index 30

  if [[ "$RECORD_BAG" == "1" ]]; then
    start_background assignment7_bag run_bag_record
    sleep 2
  fi

  if [[ "$START_RVIZ" == "1" ]]; then
    start_background rviz run_rviz
  fi

  echo "Logs are in $LOG_DIR."
  monitor_mission "$total_waypoints" "$MISSION_TIMEOUT"

  cleanup
  PIDS=()
  trap - EXIT INT TERM

  if [[ "$RECORD_BAG" == "1" ]]; then
    write_manifest
    echo
    echo "Bag saved in: $BAG_OUTPUT"
    echo "Manifest saved in: $BAG_OUTPUT/assignment7_run.yaml"
    if [[ "$ANALYZE_AFTER_RUN" == "1" ]]; then
      run_analysis_current
    fi
  fi
}

main() {
  local command="${1:-b}"
  case "$command" in
    a|odom|odom_feedback|b|corrected|ekf_corrected|ekf|c|prediction|prediction_only|ekf_prediction)
      run_assignment "$command"
      ;;
    build) build_workspace ;;
    stop) run_stop ;;
    check) run_check ;;
    -h|--help|help) usage ;;
    *)
      usage
      exit 2
      ;;
  esac
}

main "$@"
