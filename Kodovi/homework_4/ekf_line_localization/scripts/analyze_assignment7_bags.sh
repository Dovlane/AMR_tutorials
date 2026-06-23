#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(realpath "$SCRIPT_DIR/../../../..")"
PACKAGE_DIR="$ROOT_DIR/Kodovi/homework_4/ekf_line_localization"
BAGS_DIR="${BAGS_DIR:-$ROOT_DIR/bags}"
OUTPUT_DIR="${OUTPUT_DIR:-$PACKAGE_DIR/analysis/results}"
WAYPOINT_FILE="${WAYPOINT_FILE:-$PACKAGE_DIR/config/waypoints.yaml}"
MAP_FILE="${MAP_FILE:-$PACKAGE_DIR/config/turtlebot3_maze_lines.yaml}"

usage() {
  cat <<EOF
Usage:
  $0

Environment:
  BAGS_DIR=path       Directory containing assignment 7 rosbag folders. Default: ./bags.
  OUTPUT_DIR=path     Analysis output directory. Default: analysis/results.
  WAYPOINT_FILE=path  Waypoint YAML. Default: config/waypoints.yaml.
  MAP_FILE=path       EKF map YAML. Default: config/turtlebot3_maze_lines.yaml.

The script looks for the newest bags for:
  (a) odom feedback
  (b) EKF corrected, g=5
  (c) EKF prediction-only
  (b) EKF corrected, g=2
  (b) EKF corrected, g=10
EOF
}

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

source_ros() {
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    source_setup_file /opt/ros/humble/setup.bash
  fi
  if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
    source_setup_file "$ROOT_DIR/install/setup.bash"
  fi
}

latest_matching_bag() {
  local label="$1"
  shift
  local matches=()
  local pattern
  for pattern in "$@"; do
    while IFS= read -r path; do
      matches+=("$path")
    done < <(find "$BAGS_DIR" -maxdepth 1 -type d -name "$pattern" -print 2>/dev/null)
  done

  if ((${#matches[@]} == 0)); then
    echo "Missing bag for $label. Checked patterns: $*" >&2
    return 1
  fi

  printf '%s\n' "${matches[@]}" | xargs -r ls -td | head -n 1
}

main() {
  local command="${1:-analyze}"
  case "$command" in
    -h|--help|help)
      usage
      return 0
      ;;
    analyze) ;;
    *)
      usage
      return 2
      ;;
  esac

  source_ros

  local bag_a bag_b bag_c bag_g2 bag_g10
  bag_a="$(latest_matching_bag "configuration (a)" "hw4_a_odom_feedback_gate_*" "hw4_a*")"
  bag_b="$(latest_matching_bag "configuration (b), g=5" "hw4_b_ekf_corrected_gate_5p0_*" "hw4_b_ekf_corrected_gate_5_*" "hw4_b_gate_5*")"
  bag_c="$(latest_matching_bag "configuration (c)" "hw4_c_ekf_prediction_only_gate_*" "hw4_c*")"
  bag_g2="$(latest_matching_bag "configuration (b), g=2" "hw4_b_ekf_corrected_gate_2p0_*" "hw4_b_ekf_corrected_gate_2_*" "hw4_b_gate_2*")"
  bag_g10="$(latest_matching_bag "configuration (b), g=10" "hw4_b_ekf_corrected_gate_10p0_*" "hw4_b_ekf_corrected_gate_10_*" "hw4_b_gate_10*")"

  python3 "$PACKAGE_DIR/analysis/assignment7_analysis.py" \
    --bags "$bag_a" "$bag_b" "$bag_c" "$bag_g2" "$bag_g10" \
    --output-dir "$OUTPUT_DIR" \
    --waypoint-file "$WAYPOINT_FILE" \
    --map-file "$MAP_FILE"
}

main "$@"
