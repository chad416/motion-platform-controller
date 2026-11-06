#!/usr/bin/env bash
# Build the firmware for the STM32H7 Renode Reference Platform.
# Usage (inside WSL): scripts/build.sh [extra west build args]
set -euo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

cd "$WS"
west build -p auto -b stm32h7_renode_reference_board -d "$WS/build/motion" "$REPO/app" "$@"
