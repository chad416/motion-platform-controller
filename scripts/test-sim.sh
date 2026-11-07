#!/usr/bin/env bash
# Run the Robot Framework integration tests in Renode against the built firmware.
# Usage (inside WSL): scripts/test-sim.sh [extra renode-test args: -f "IT03*", --exclude long, --include long]
set -euo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

OUT="${MOTION_ROBOT_OUT:-$WS/robot-out-motion}"
mkdir -p "$OUT"
# renode-test mishandles a current directory containing spaces, and writes
# robot_output.xml, log.html and report.html to the current directory.
cd "$OUT"
renode-test -r "$OUT" --variable "ELF:$WS/build/motion/zephyr/zephyr.elf" "$@" "$REPO/tests/robot/motion.robot"
