#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
  "$SCRIPT_DIR/run_assignment7_config.sh" build
fi

export SKIP_BUILD=1
export START_RVIZ="${START_RVIZ:-0}"
export ANALYZE_AFTER_RUN=0

"$SCRIPT_DIR/run_assignment7_config.sh" a
"$SCRIPT_DIR/run_assignment7_config.sh" b
"$SCRIPT_DIR/run_assignment7_config.sh" c
VALIDATION_GATE=2.0 "$SCRIPT_DIR/run_assignment7_config.sh" b
VALIDATION_GATE=10.0 "$SCRIPT_DIR/run_assignment7_config.sh" b

"$SCRIPT_DIR/analyze_assignment7_bags.sh"
