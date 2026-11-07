#!/usr/bin/env python3
"""3D view of the motion platform: base, legs and cabin at the pose the
controller reports (POSE_T/R from its forward kinematics), legs shaded by
utilisation (how far each stroke is from mid-stroke towards a soft limit, from
LEG_POS), with the leg strokes against their limits and the last 10 s of surge
and pitch beside it.

Two sources, one drawing:
  live    listen on SocketCAN while motion_host.py (or anything) drives the bus
  replay  a session CSV written by motion_host.py --log, rendered on the
          controller's clock (deterministic, used for the README video)

Usage:
  platform_view.py --live [--channel vcan0] [--frames-dir out/ --fps 10]
  platform_view.py --replay session.csv --frames-dir out/ --fps 20 [--video view.mp4]
"""
from __future__ import annotations

import argparse
import collections
import csv
import gzip
import json
import pathlib
import subprocess
import sys
import threading
import time

import matplotlib
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "reference"))
from mcue_ref import load_geometry, rot  # noqa: E402

from motion_host import PlatformState  # noqa: E402
from protocol import Protocol  # noqa: E402

# Palette (light surface): sequential blue for utilisation, categorical slots for traces.
SURFACE = "#fcfcfb"
INK, INK_2, MUTED, GRID = "#1f1f1e", "#5c5b57", "#8a8984", "#e6e5e1"
SEQ = ["#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#104281", "#0d366b"]  # steps 250..700
BLUE, ORANGE = "#2a78d6", "#eb6834"
WARN, CRIT = "#fab219", "#d03b3b"
TRACE_S = 10.0


def util_color(u: float) -> str:
    return SEQ[min(len(SEQ) - 1, int(max(0.0, min(1.0, u)) * (len(SEQ) - 1) + 0.5))]


class View:
    def __init__(self, title: str = "motion platform"):
        matplotlib.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK_2,
                                    "xtick.color": MUTED, "ytick.color": MUTED,
                                    "axes.edgecolor": GRID})
        import matplotlib.pyplot as plt
        self.plt = plt
        self.g = load_geometry()
        cfg = json.loads((ROOT / "config" / "platform.json").read_text())["geometry"]
        self.stroke = cfg["stroke_m"]
        self.margin = cfg["soft_margin_m"]
        self.fig = plt.figure(figsize=(12.8, 7.2), dpi=100, facecolor=SURFACE)
        gs = self.fig.add_gridspec(3, 2, width_ratios=[1.75, 1], height_ratios=[1.25, 1, 1],
                                   left=0.02, right=0.97, top=0.85, bottom=0.13, wspace=0.12, hspace=0.55)
        self.ax3 = self.fig.add_subplot(gs[:, 0], projection="3d")
        self.ax_leg = self.fig.add_subplot(gs[0, 1])
        self.ax_x = self.fig.add_subplot(gs[1, 1])
        self.ax_p = self.fig.add_subplot(gs[2, 1])
        self.title = self.fig.text(0.02, 0.955, title, fontsize=13, weight="bold", color=INK)
        self.status = self.fig.text(0.02, 0.915, "", fontsize=10, color=INK_2)
        self.badge = self.fig.text(0.98, 0.955, "", fontsize=12, weight="bold", ha="right", color=INK)
        self.flag_dot = self.fig.text(0.02, 0.878, "", fontsize=11, color=WARN)
        self.flag_txt = self.fig.text(0.037, 0.879, "", fontsize=10, color=INK)
        self.base_title = title
        self.foot = self.fig.text(0.02, 0.045, "", fontsize=8.5, color=INK_2)
        self.fig.text(0.02, 0.015, "Legs and bars shaded by utilisation: light = mid-stroke, dark = at a soft "
                      "limit. Pose from the controller's forward kinematics (POSE_T/R), strokes from LEG_POS.",
                      fontsize=8.5, color=MUTED)
        self._setup_3d()
        self._setup_panels()
        self.hist = collections.deque()

    # ---- static parts -------------------------------------------------------------
    def _setup_3d(self):
        ax = self.ax3
        ax.set_facecolor(SURFACE)
        for a in (ax.xaxis, ax.yaxis, ax.zaxis):
            a.set_pane_color((0.99, 0.99, 0.985, 1.0))
            a._axinfo["grid"]["color"] = GRID
        ax.set_xlim(-0.9, 0.9)
        ax.set_ylim(-0.9, 0.9)
        ax.set_zlim(0.0, 1.35)
        ax.set_box_aspect((1.8, 1.8, 1.35))
        ax.view_init(elev=16, azim=-58)
        ax.set_xlabel("x forward (m)")
        ax.set_ylabel("y left (m)")
        ax.set_zlabel("z (m)")
        base = np.vstack([self.g.base, self.g.base[:1]])
        ax.plot(base[:, 0], base[:, 1], base[:, 2], color=MUTED, lw=2)
        ax.plot_trisurf(self.g.base[:, 0], self.g.base[:, 1], self.g.base[:, 2], color=GRID, alpha=0.6)
        self.legs = [ax.plot([], [], [], lw=5, solid_capstyle="round")[0] for _ in range(6)]
        self.leg_labels = [ax.text(0, 0, 0, str(i + 1), fontsize=8, color=INK_2) for i in range(6)]
        self.plat_line = ax.plot([], [], [], color=INK_2, lw=2)[0]
        self.plat_fill = None
        self.cabin = []
        # cabin: a box on the platform (seat, screen) so the tilt is easy to see
        hx, hy, hz = 0.32, 0.26, 0.42
        self.box = np.array([[sx * hx, sy * hy, z] for z in (0.02, hz) for sx, sy in
                             ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        self.box_edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
                          (0, 4), (1, 5), (2, 6), (3, 7)]
        self.cabin = [ax.plot([], [], [], color=INK_2, lw=1.2)[0] for _ in self.box_edges]
        self.front = ax.plot([], [], [], color=ORANGE, lw=2.5)[0]

    def _setup_panels(self):
        lo, hi = self.margin * 1e3, (self.stroke - self.margin) * 1e3
        ax = self.ax_leg
        ax.set_title("Leg stroke (mm above the lower end stop)", loc="left", fontsize=9.5, color=INK)
        self.bars = ax.bar(range(1, 7), [0] * 6, width=0.62, color=SEQ[0])
        ax.set_ylim(0, self.stroke * 1e3)
        ax.set_xticks(range(1, 7))
        ax.set_xticklabels([f"leg {i}" for i in range(1, 7)])
        for y in (lo, hi):
            ax.axhline(y, color=INK_2, lw=1, ls=(0, (4, 3)))
        ax.text(6.45, hi, "soft limit", fontsize=7.5, color=INK_2, va="bottom", ha="right")
        ax.axhline(self.stroke * 500, color=GRID, lw=1)
        ax.set_facecolor(SURFACE)
        self.bar_text = [ax.text(i + 1, 0, "", ha="center", va="bottom", fontsize=7.5, color=INK_2)
                         for i in range(6)]
        for ax, name, unit, col in ((self.ax_x, "Surge x", "mm", BLUE), (self.ax_p, "Pitch", "deg", ORANGE)):
            ax.set_title(f"{name} ({unit}), last {TRACE_S:.0f} s", loc="left", fontsize=9.5, color=INK)
            ax.set_facecolor(SURFACE)
            ax.grid(color=GRID, lw=0.8)
            ax.set_xlim(-TRACE_S, 0)
            ax.axhline(0, color=MUTED, lw=0.8)
        self.ax_x.set_ylim(-170, 170)
        self.ax_p.set_ylim(-14, 14)
        self.ax_p.set_xlabel("s before now (controller time)")
        self.tr_x = self.ax_x.plot([], [], color=BLUE, lw=2)[0]
        self.tr_p = self.ax_p.plot([], [], color=ORANGE, lw=2)[0]

    # ---- per frame -----------------------------------------------------------------
    def update(self, ps: PlatformState, t_ms: float, note: str = "") -> None:
        x, y, z, r, p, yw = ps.pose
        R = rot(np.radians(r), np.radians(p), np.radians(yw))
        c = np.array([x, y, z + self.g.h0])
        top = c + (R @ self.g.plat.T).T
        mid = self.stroke / 2
        for i, line in enumerate(self.legs):
            a, b = self.g.base[i], top[i]
            line.set_data_3d([a[0], b[0]], [a[1], b[1]], [a[2], b[2]])
            u = abs(ps.strokes[i] - mid) / (mid - self.margin)
            line.set_color(util_color(u))
            m = a + 0.45 * (b - a)
            self.leg_labels[i].set_position_3d((m[0] + 0.03, m[1], m[2]))
        ring = np.vstack([top, top[:1]])
        self.plat_line.set_data_3d(ring[:, 0], ring[:, 1], ring[:, 2])
        if self.plat_fill is not None:
            self.plat_fill.remove()
        self.plat_fill = self.ax3.plot_trisurf(top[:, 0], top[:, 1], top[:, 2], color=BLUE, alpha=0.25,
                                               linewidth=0)
        box = c + (R @ self.box.T).T
        for (i, j), line in zip(self.box_edges, self.cabin):
            line.set_data_3d([box[i, 0], box[j, 0]], [box[i, 1], box[j, 1]], [box[i, 2], box[j, 2]])
        f = box[[5, 6]]  # front top edge (+x)
        self.front.set_data_3d(f[:, 0], f[:, 1], f[:, 2])

        for i, (bar, txt) in enumerate(zip(self.bars, self.bar_text)):
            s = ps.strokes[i] * 1e3
            bar.set_height(s)
            bar.set_color(util_color(abs(ps.strokes[i] - mid) / (mid - self.margin)))
            txt.set_position((i + 1, s + 3))
            txt.set_text(f"{s:.0f}")

        if not self.hist or self.hist[-1][0] < t_ms:
            self.hist.append((t_ms, x * 1e3, p))
        while self.hist and self.hist[0][0] < t_ms - TRACE_S * 1e3:
            self.hist.popleft()
        h = np.array(self.hist)
        self.tr_x.set_data((h[:, 0] - t_ms) / 1e3, h[:, 1])
        self.tr_p.set_data((h[:, 0] - t_ms) / 1e3, h[:, 2])

        fl = ps.flags
        self.badge.set_text(ps.state)
        self.badge.set_color(CRIT if ps.state in ("FAULT", "ESTOP") else INK)
        flags = []
        if fl.get("saturated"):
            flags.append("workspace limit: command scaled to %.2f" % fl.get("ws_scale", 1.0))
        if ps.fault != "NONE":
            flags.append(f"fault {ps.fault}" + (f" on leg {ps.fault_leg}" if ps.fault_leg else ""))
        if fl.get("comms_lost"):
            flags.append("host heartbeat lost")
        self.flag_dot.set_text("\u25a0" if flags else "")
        self.flag_dot.set_color(CRIT if ps.fault != "NONE" else WARN)
        self.flag_txt.set_text("   ".join(flags))
        self.status.set_text(f"t = {t_ms / 1e3:6.2f} s   pose x {x * 1e3:6.1f}  y {y * 1e3:6.1f}  "
                             f"z {z * 1e3:6.1f} mm   roll {r:5.2f}  pitch {p:5.2f}  yaw {yw:5.2f} deg")
        self.title.set_text(self.base_title + (f"  \u00b7  {note}" if note else ""))
        v = ps.vehicle
        tm = ps.timing
        loop = (f"   loops: servo worst {tm['servo_max_us']} us, cueing worst {tm['cue_max_us']} us, "
                f"overruns {tm['servo_overruns']}/{tm['cue_overruns']}") if tm else ""
        self.foot.set_text(f"vehicle f = ({v[0]:5.2f}, {v[1]:5.2f}, {v[2]:5.2f}) m/s^2   "
                           f"w = ({v[3]:5.2f}, {v[4]:5.2f}, {v[5]:5.2f}) rad/s{loop}")

    def push(self, t_ms: float, ps: PlatformState) -> None:
        """Add a pose sample to the traces (call on every POSE_R)."""
        self.hist.append((t_ms, ps.pose[0] * 1e3, ps.pose[4]))

    def save(self, path: pathlib.Path) -> None:
        self.fig.savefig(path, facecolor=SURFACE)


def replay(csv_path: pathlib.Path, frames_dir: pathlib.Path, fps: float, note: str,
           t_from: float = 0.0, t_to: float = 1e18, shots: list[float] | None = None,
           shots_dir: pathlib.Path | None = None) -> int:
    """Render frames every 1/fps s of controller time; also save the frame at
    each time in `shots` (ms) as shots_dir/shot_<i>.png."""
    shots = sorted(shots or [])
    taken = 0
    matplotlib.use("Agg")
    view = View()
    ps = PlatformState()
    frames_dir.mkdir(parents=True, exist_ok=True)
    period = 1000.0 / fps
    next_t = None
    n = 0
    opener = gzip.open if csv_path.suffix == ".gz" else open
    with opener(csv_path, "rt", newline="") as f:
        for row in csv.DictReader(f):
            t = float(row["ctl_ms"])
            if t < 0:
                continue
            if next_t is None:
                next_t = max(t, t_from)
            while t >= next_t and next_t <= t_to:
                if next_t >= t_from:
                    view.update(ps, next_t, note)
                    view.save(frames_dir / f"frame_{n:05d}.png")
                    n += 1
                    while taken < len(shots) and next_t >= shots[taken]:
                        shots_dir.mkdir(parents=True, exist_ok=True)
                        view.save(shots_dir / f"shot_{taken + 1}.png")
                        taken += 1
                next_t += period
            if row["name"] == "NOTE":
                note = json.loads(row["signals"])["text"]
                continue
            ps.apply(int(row["id"], 16), row["name"], json.loads(row["signals"]))
            if row["name"] == "POSE_R":
                view.push(t, ps)
    return n


def live(channel: str, interface: str, frames_dir: pathlib.Path | None, fps: float) -> None:
    import can
    if frames_dir:
        matplotlib.use("Agg")
    proto = Protocol()
    ps = PlatformState()
    lock = threading.Lock()
    clock = {"ms": 0.0}
    view = View("motion platform (live)")

    def on_msg(msg):
        d = proto.decode(msg.arbitration_id, bytes(msg.data))
        if d:
            with lock:
                ps.apply(d.frame_id, d.name, d.signals)
                if d.name == "STATUS":
                    clock["ms"] += 20.0
                elif d.name == "POSE_R":
                    view.push(clock["ms"], ps)

    bus = can.Bus(interface=interface, channel=channel)
    notifier = can.Notifier(bus, [on_msg])
    n = 0
    try:
        if frames_dir:
            frames_dir.mkdir(parents=True, exist_ok=True)
        else:
            view.plt.ion()
            view.plt.show()
        while True:
            with lock:
                view.update(ps, clock["ms"])
            if frames_dir:
                view.save(frames_dir / f"frame_{n:05d}.png")
                n += 1
                time.sleep(1.0 / fps)
            else:
                view.plt.pause(1.0 / fps)
    except KeyboardInterrupt:
        pass
    finally:
        notifier.stop()
        bus.shutdown()


def make_video(frames_dir: pathlib.Path, fps: float, out: pathlib.Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps),
                    "-i", str(frames_dir / "frame_%05d.png"), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-crf", "23", str(out)], check=True)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--live", action="store_true")
    src.add_argument("--replay", type=pathlib.Path)
    p.add_argument("--channel", default="vcan0")
    p.add_argument("--interface", default="socketcan")
    p.add_argument("--frames-dir", type=pathlib.Path, default=None)
    p.add_argument("--fps", type=float, default=20.0)
    p.add_argument("--video", type=pathlib.Path, default=None, help="replay: MP4 from the frames")
    p.add_argument("--from-s", type=float, default=0.0)
    p.add_argument("--to-s", type=float, default=1e15)
    p.add_argument("--note", default="", help="text at the start of the footer")
    p.add_argument("--shots", default="", help="replay: controller times (s) to keep as PNG, comma-separated")
    p.add_argument("--shots-dir", type=pathlib.Path, default=None)
    a = p.parse_args()
    if a.live:
        live(a.channel, a.interface, a.frames_dir, a.fps)
        return 0
    if not a.frames_dir:
        p.error("--replay needs --frames-dir")
    shots = [float(x) * 1e3 for x in a.shots.split(",") if x]
    n = replay(a.replay, a.frames_dir, a.fps, a.note, a.from_s * 1e3, a.to_s * 1e3, shots,
               a.shots_dir or a.frames_dir)
    print(f"{n} frames in {a.frames_dir}")
    if a.video:
        make_video(a.frames_dir, a.fps, a.video)
        print(f"video {a.video}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
