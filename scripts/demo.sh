#!/usr/bin/env bash
# Recorded session for the README: the firmware in Renode bridged to SocketCAN
# vcan0, driven by the host (tools/host/demo_session.py), then the session plots,
# the 3D view screenshots and the video.
# Writes docs/session/ (session.csv.gz, plots, summary.md) and docs/media/.
# Usage (inside WSL, as root for vcan): scripts/demo.sh
set -euo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

OUT="${MOTION_DEMO_OUT:-$WS/demo-out-motion}"
DOCS="$REPO/docs"
PORT=4444
FPS=20
rm -rf "$OUT" && mkdir -p "$OUT" "$DOCS/session" "$DOCS/media"

ip link show vcan0 >/dev/null 2>&1 || { modprobe vcan; ip link add dev vcan0 type vcan; }
ip link set up vcan0

renode --disable-gui -P "$PORT" > "$OUT/renode.log" 2>&1 &
RENODE_PID=$!
trap 'kill $RENODE_PID 2>/dev/null || true' EXIT

cd "$REPO/tools/host"
python3 - <<EOF
from renode_ctl import RenodeMonitor
m = RenodeMonitor(port=$PORT)
m.cmd('\$elf=@$WS/build/motion/zephyr/zephyr.elf')
m.cmd('include @$REPO/sim/motion_live.resc', settle=3)
m.cmd('start', settle=0.5)
m.close()
EOF
python3 demo_session.py --log "$OUT/session.csv" --renode-port "$PORT" | tee "$OUT/session.log"
kill "$RENODE_PID" 2>/dev/null || true
cp /tmp/motion_uart.log "$OUT/uart.log" 2>/dev/null || true

python3 session_plots.py "$OUT/session.csv" --out "$DOCS/session"
gzip -9 -c "$OUT/session.csv" > "$DOCS/session/session.csv.gz"

# screenshots: largest roll in the cornering, smallest workspace scale in the 3 g cue,
# 1.5 s after the E-stop (scenario names are NOTE rows written by the host)
SHOTS=$(python3 - "$OUT/session.csv" <<'EOF'
import csv, json, sys
scene = None; roll = (0.0, 0.0); scale = (2.0, 0.0); estop = None
for r in csv.DictReader(open(sys.argv[1])):
    s = json.loads(r["signals"]); t = float(r["ctl_ms"]) / 1e3
    if r["name"] == "NOTE":
        scene = s["text"]
        if scene == "E-stop":
            estop = t
    elif r["name"] == "POSE_R" and scene == "cornering" and abs(s["roll"]) > roll[0]:
        roll = (abs(s["roll"]), t)
    elif r["name"] == "STATUS" and scene == "excessive" and s["ws_scale"] < scale[0]:
        scale = (s["ws_scale"], t)
print(f"{roll[1]:.2f},{scale[1]:.2f},{estop + 1.5:.2f}")
EOF
)
echo "screenshots at $SHOTS s"
python3 platform_view.py --replay "$OUT/session.csv" --frames-dir "$OUT/frames" --fps "$FPS" \
	--shots "$SHOTS" --shots-dir "$OUT/shots" --video "$DOCS/media/platform_view.mp4"
for i in 1 2 3; do cp "$OUT/shots/shot_$i.png" "$DOCS/media/platform_view_$i.png"; done
echo "demo done: $DOCS/session, $DOCS/media"
