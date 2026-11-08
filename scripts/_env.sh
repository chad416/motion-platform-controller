# Sourced by the other scripts. Sets REPO, WS and activates the Python venv.
#
#   MOTION_WS    west workspace (zephyr/, modules/, build/), default $HOME/trainer-ws
#   MOTION_VENV  Python venv with requirements.txt and Zephyr's packages,
#                default $HOME/zvenv
#
# If the repository path contains a space (for example a Windows folder seen
# from WSL), it is bind-mounted at $HOME/motion-src first, because the Zephyr
# build and twister break on paths with spaces (scripts/wsl-mount.sh, as root).

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$REPO" == *" "* ]]; then
	bash "$REPO/scripts/wsl-mount.sh" >/dev/null
	REPO="$HOME/motion-src"
fi
# The default workspace name is shared with the trainer-weapon-firmware-twin repository.
WS="${MOTION_WS:-$HOME/trainer-ws}"

# shellcheck disable=SC1091
source "${MOTION_VENV:-$HOME/zvenv}/bin/activate"
