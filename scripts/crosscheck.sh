#!/usr/bin/env bash
# Check generated headers are current, then compare the C library with the NumPy reference.
# Usage (inside WSL): scripts/crosscheck.sh [--n 1000]
set -euo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

cd "$REPO/tools/reference"
python3 gen_headers.py --check
python3 crosscheck.py "$@"
