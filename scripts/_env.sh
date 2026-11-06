# Sourced by the other scripts. Sets REPO, WS and activates the Python venv.
# If the repository path contains a space (Windows folder "New CLI"), it is
# bind-mounted at $HOME/motion-src first, because Zephyr builds break on spaces.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$REPO" == *" "* ]]; then
	bash "$REPO/scripts/wsl-mount.sh" >/dev/null
	REPO="$HOME/motion-src"
fi
# Shares the west workspace (Zephyr checkout and SDK) with S1.
WS="${MOTION_WS:-$HOME/trainer-ws}"

# shellcheck disable=SC1091
source "${MOTION_VENV:-$HOME/zvenv}/bin/activate"
