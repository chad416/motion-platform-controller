#!/usr/bin/env bash
# Run the ztest unit tests of the mcue library on native_sim with twister.
# Usage (inside WSL): scripts/test-unit.sh [extra twister args, e.g. --coverage]
set -euo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

cd "$WS"
west twister -T "$REPO/tests/unit" -p native_sim --outdir "$WS/twister-out-motion" --inline-logs "$@"
