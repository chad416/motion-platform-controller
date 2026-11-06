#!/usr/bin/env bash
# Bind-mount this repository (on the Windows drive) at a path without spaces
# inside WSL. Zephyr's build and twister break on paths containing spaces, and
# a symlink is not enough because twister resolves it to the real path.
# Usage (inside WSL, as root): bash "<repo>/scripts/wsl-mount.sh" [mount point]
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET="${1:-$HOME/motion-src}"

if mountpoint -q "$TARGET"; then
	echo "already mounted: $TARGET"
	exit 0
fi

[ -L "$TARGET" ] && rm "$TARGET"
mkdir -p "$TARGET"
mount --bind "$REPO" "$TARGET"
echo "mounted $REPO -> $TARGET"
