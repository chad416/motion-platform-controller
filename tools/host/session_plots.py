#!/usr/bin/env python3
"""Plots and a summary of a recorded session (motion_host.py --log CSV).

- cue: the specific force asked for (vehicle force after the soft limit and the
  force gain, the closed-loop definition in tools/reference/closed_loop.py)
  against the one the platform delivers at its centre, R^T (a + g), estimated
  from the reported FK pose (50 Hz, 0.1 mm; second derivative by a
  Savitzky-Golay fit over 0.22 s)
- tilt: roll and pitch, and the tilt rate against the 3 deg/s limit
- legs: strokes against the soft limits, currents
- loop timing (TIMING frames), states, faults, commands -> summary.md

Usage: session_plots.py session.csv --out docs/session
"""
from __future__ import annotations

import argparse
import csv
import gzip
import json
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.signal import savgol_filter  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "reference"))
from mcue_ref import G, rot  # noqa: E402
from washout_ref import softlim  # noqa: E402

from protocol import FAULTS, OPNAMES, PARAMS, STATES, STATUS  # noqa: E402

SURFACE, INK, INK_2, MUTED, GRID = "#fcfcfb", "#1f1f1e", "#5c5b57", "#8a8984", "#e6e5e1"
CAT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
STATE_SHADE = {"ENGAGED": None, "FAULT": "#f6d4d4", "ESTOP": "#f6d4d4"}


def load(path: pathlib.Path) -> dict:
    out = {k: [] for k in ("veh", "pose", "pos", "cur", "status", "timing", "fault", "ack", "note")}
    pend = {}
    seq_base = {}
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", newline="") as f:
        for r in csv.DictReader(f):
            name, s, t = r["name"], json.loads(r["signals"]), float(r["ctl_ms"]) / 1e3
            if name == "NOTE":
                out["note"].append((t, s["text"]))
            elif name == "VEHICLE_F":
                pend["f"] = (t, [s["f_x"], s["f_y"], s["f_z"]], s["seq"])
            elif name == "VEHICLE_W" and pend.get("f") and pend["f"][2] == s["seq"]:
                t0, fv, _ = pend.pop("f")
                out["veh"].append([t0, *fv, s["w_x"], s["w_y"], s["w_z"]])
            elif name in ("POSE_T", "LEG_POS_A", "LEG_CUR_A"):
                pend[name] = (s["seq"], s)
            elif name in ("POSE_R", "LEG_POS_B", "LEG_CUR_B"):
                a_name = {"POSE_R": "POSE_T", "LEG_POS_B": "LEG_POS_A", "LEG_CUR_B": "LEG_CUR_A"}[name]
                a = pend.pop(a_name, None)
                if not a or a[0] != s["seq"]:
                    continue
                # 50 Hz reports share STATUS's sequence number: 20 ms of controller time each
                if "base" not in seq_base:
                    continue
                b_seq, b_t = seq_base["base"]
                tt = b_t + (((s["seq"] - b_seq) & 0xFFFF) * 0.02)
                a = a[1]
                if name == "POSE_R":
                    out["pose"].append([tt, a["x"], a["y"], a["z"], s["roll"], s["pitch"], s["yaw"]])
                elif name == "LEG_POS_B":
                    out["pos"].append([tt, a["stroke_1"], a["stroke_2"], a["stroke_3"],
                                       s["stroke_4"], s["stroke_5"], s["stroke_6"]])
                else:
                    out["cur"].append([tt, a["current_1"], a["current_2"], a["current_3"],
                                       s["current_4"], s["current_5"], s["current_6"]])
            elif name == "STATUS":
                seq_base.setdefault("base", (s["seq"], t))
                out["status"].append((t, STATES[s["state"]], s))
            elif name == "TIMING":
                out["timing"].append([t, s["servo_max_us"], s["cue_max_us"], s["servo_overruns"],
                                      s["cue_overruns"]])
            elif name == "FAULT":
                out["fault"].append((t, FAULTS[s["code"]] if s["code"] < len(FAULTS) else s["code"],
                                     s["leg"], s["value"]))
            elif name == "CMD_ACK":
                out["ack"].append((t, OPNAMES.get(s["opcode"], s["opcode"]), s["key"],
                                   STATUS[s["status"]] if s["status"] < len(STATUS) else s["status"],
                                   s["value"]))
    for k in ("veh", "pose", "pos", "cur", "timing"):
        out[k] = np.array(out[k]) if out[k] else np.zeros((0, 7))
    return out


def force_gain_at(d: dict, t: np.ndarray) -> np.ndarray:
    """The force gain in effect (config default, then every SET that was accepted)."""
    cfg = json.loads((ROOT / "config" / "platform.json").read_text())["washout"]
    gain = np.full(len(t), cfg["force_gain"])
    key, scale = PARAMS["force_gain"]
    for ta, op, k, st, v in d["ack"]:
        if op == "set" and k == key and st == "OK":
            gain[t >= ta] = v * scale
    return gain


def cue(d: dict):
    cfg = json.loads((ROOT / "config" / "platform.json").read_text())["washout"]
    pose = d["pose"]
    t = pose[:, 0]
    dt = 0.02
    acc = savgol_filter(pose[:, 1:4], 11, 3, deriv=2, delta=dt, axis=0)
    delivered = np.array([rot(*np.radians(q[4:7])).T @ (a + [0, 0, G]) - [0, 0, G]
                          for q, a in zip(pose, acc)])
    veh = d["veh"]
    f = np.column_stack([np.interp(t, veh[:, 0], veh[:, k]) for k in (1, 2, 3)])
    f[:, 2] -= G
    asked = force_gain_at(d, t)[:, None] * softlim(f, cfg["force_limit_m_s2"])
    return t, asked, delivered


def shade_states(ax, d, t_end):
    st = d["status"]
    for i, (t, name, _s) in enumerate(st):
        t1 = st[i + 1][0] if i + 1 < len(st) else t_end
        if name == "ENGAGED":
            continue
        ax.axvspan(t, t1, color=STATE_SHADE.get(name) or "#f0efec", lw=0, zorder=0)


def scenario_marks(axs, d):
    """Scenario names (NOTE rows from the host) above the first panel, lines on all."""
    for i, (t, text) in enumerate(d["note"]):
        axs[0].text(t, 1.02 + 0.09 * (i % 2), " " + text, transform=axs[0].get_xaxis_transform(),
                    fontsize=8, color=INK, va="bottom")
        for ax in axs:
            ax.axvline(t, color=MUTED, lw=0.8, ls=(0, (1, 2)))
    if d["note"]:
        axs[0].set_title(axs[0].get_title(loc="left"), loc="left", fontsize=9.5, color=INK, pad=34)


def style(ax, title, unit):
    ax.set_facecolor(SURFACE)
    ax.grid(color=GRID, lw=0.7)
    ax.set_title(title, loc="left", fontsize=9.5, color=INK, pad=14)
    ax.set_ylabel(unit, color=INK_2)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.tick_params(colors=MUTED)


def plot_cue(d, out: pathlib.Path):
    t, asked, delivered = cue(d)
    tilt_rate = np.gradient(savgol_filter(d["pose"][:, 4:6], 11, 2, axis=0), 0.02, axis=0)
    fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True, facecolor=SURFACE)
    for ax, k, name in ((axs[0], 0, "x (surge)"), (axs[1], 1, "y (sway)")):
        ax.plot(t, asked[:, k], color=CAT[0], lw=1.6, label="asked (vehicle x force gain, soft-limited)")
        ax.plot(t, delivered[:, k], color=CAT[1], lw=1.4, label="delivered at the platform centre")
        style(ax, f"Specific force {name}. Shaded: not engaged (grey), E-stop or fault (red)", "m/s²")
        ax.legend(loc="upper right", fontsize=8, frameon=False)
    ax = axs[2]
    ax.plot(d["pose"][:, 0], d["pose"][:, 4], color=CAT[0], lw=1.6, label="roll")
    ax.plot(d["pose"][:, 0], d["pose"][:, 5], color=CAT[1], lw=1.6, label="pitch")
    style(ax, "Platform roll and pitch (FK pose)", "deg")
    ax.legend(loc="upper right", fontsize=8, frameon=False)
    ax = axs[3]
    ax.plot(d["pose"][:, 0], tilt_rate[:, 0], color=CAT[0], lw=1.2, label="roll rate")
    ax.plot(d["pose"][:, 0], tilt_rate[:, 1], color=CAT[1], lw=1.2, label="pitch rate")
    for y in (-3, 3):
        ax.axhline(y, color=INK_2, lw=1, ls=(0, (4, 3)))
    ax.text(t[-1], 3, "3 deg/s: limit on the tilt part only (IT03 checks it alone)", fontsize=7.5,
            color=INK_2, va="bottom", ha="right")
    peak = float(np.max(np.abs(tilt_rate)))
    ax.set_ylim(-8, 8)
    style(ax, "Roll and pitch rate: tilt plus the rotational channel (axis clipped at \u00b18 deg/s; "
              f"peak {peak:.0f} deg/s when the workspace limit scales the 3 g cue)", "deg/s")
    ax.legend(loc="upper left", fontsize=8, frameon=False)
    ax.set_xlabel("controller time (s)", color=INK_2)
    for ax in axs:
        shade_states(ax, d, t[-1])
    scenario_marks(axs, d)
    fig.tight_layout()
    fig.savefig(out, dpi=110, facecolor=SURFACE)
    plt.close(fig)
    return t, asked, delivered


def plot_legs(d, out: pathlib.Path):
    cfg = json.loads((ROOT / "config" / "platform.json").read_text())["geometry"]
    lo, hi = cfg["soft_margin_m"] * 1e3, (cfg["stroke_m"] - cfg["soft_margin_m"]) * 1e3
    fig, axs = plt.subplots(3, 1, figsize=(12, 8.5), sharex=True, facecolor=SURFACE,
                            gridspec_kw={"height_ratios": [1.3, 1, 0.8]})
    ax = axs[0]
    for i in range(6):
        ax.plot(d["pos"][:, 0], d["pos"][:, i + 1] * 1e3, color=CAT[i], lw=1.3, label=f"leg {i + 1}")
    for y in (lo, hi):
        ax.axhline(y, color=INK_2, lw=1, ls=(0, (4, 3)))
    ax.text(d["pos"][-1, 0], hi, "soft limit ", fontsize=7.5, color=INK_2, va="bottom", ha="right")
    style(ax, "Leg stroke (LEG_POS). Shaded: not engaged (grey), E-stop or fault (red)", "mm")
    ax.set_ylim(0, cfg["stroke_m"] * 1e3)
    ax.legend(loc="upper left", ncol=6, fontsize=8, framealpha=0.9, facecolor=SURFACE, edgecolor="none")
    ax = axs[1]
    for i in range(6):
        ax.plot(d["cur"][:, 0], d["cur"][:, i + 1], color=CAT[i], lw=1.1)
    style(ax, "Leg motor current (LEG_CUR; 2.95 A holds the payload at neutral)", "A")
    ax = axs[2]
    tm = d["timing"]
    if len(tm):
        ax.step(tm[:, 0], tm[:, 1], where="post", color=CAT[0], lw=1.6, label="servo loop, period 1000 us")
        ax.step(tm[:, 0], tm[:, 2], where="post", color=CAT[1], lw=1.6, label="cueing loop, period 2000 us")
    style(ax, "Worst cycle time since boot (TIMING, virtual time)", "us")
    ax.set_ylim(0, 1000)
    ax.legend(loc="lower right", fontsize=8, frameon=False)
    ax.set_xlabel("controller time (s)", color=INK_2)
    for ax in axs:
        shade_states(ax, d, d["pos"][-1, 0])
    scenario_marks(axs, d)
    fig.tight_layout()
    fig.savefig(out, dpi=110, facecolor=SURFACE)
    plt.close(fig)


def summary(d, csv_path, cue_res, out: pathlib.Path):
    t, asked, delivered = cue_res
    st = d["status"]
    engaged = np.array([s[1] == "ENGAGED" for s in st])
    states = []
    for tt, name, _s in st:
        if not states or states[-1][1] != name:
            states.append((tt, name))
    pos = d["pos"]
    tm = d["timing"]
    veh = d["veh"]
    dts = np.diff(veh[:, 0]) * 1e3
    lines = [f"# Session `{csv_path.name}`", "",
             "Recorded by `tools/host/motion_host.py` over SocketCAN from the firmware in Renode; "
             "times are controller (virtual) time.", "",
             f"- Duration: {st[-1][0] - st[0][0]:.1f} s of controller time; "
             f"{len(st)} STATUS frames, {len(veh)} vehicle samples sent "
             f"(spacing on the controller clock: mean {dts.mean():.2f} ms, "
             f"min {dts.min():.1f}, max {dts.max():.1f}).",
             "- States: " + " → ".join(f"{n} ({tt:.1f} s)" for tt, n in states),
             "- Faults: " + (", ".join(f"{c}{f' leg {leg}' if leg else ''} at {tt:.1f} s"
                                       for tt, c, leg, _v in d["fault"]) or "none"),
             "- Scenarios: " + (", ".join(f"{n} ({tt:.1f} s)" for tt, n in d["note"]) or "-"),
             f"- Leg strokes: {pos[:, 1:].min() * 1e3:.2f} … {pos[:, 1:].max() * 1e3:.2f} mm "
             "(soft limits 10 / 240 mm).",
             f"- Engaged {engaged.mean() * 100:.0f} % of the time; workspace scale down to "
             f"{min(s[2]['ws_scale'] for s in st):.3f}.",
             ]
    if len(tm):
        lines.append(f"- Loop timing (TIMING, virtual): servo worst {int(tm[-1, 1])} µs of 1000, cueing "
                     f"worst {int(tm[-1, 2])} µs of 2000, overruns {int(tm[-1, 3])} / {int(tm[-1, 4])}.")
    lines += ["", "## Commands", "", "| t (s) | command | key | status | value |", "|---|---|---|---|---|"]
    lines += [f"| {max(tt, 0.0):.2f} | {op} | {k} | {s} | {v} |" for tt, op, k, s, v in d["ack"]]
    err = delivered - asked
    lines += ["", "## Cue (whole session)", "",
              "| axis | asked RMS (m/s²) | error RMS (m/s²) |", "|---|---|---|"]
    for k, ax in enumerate("xyz"):
        lines.append(f"| {ax} | {np.sqrt(np.mean(asked[:, k] ** 2)):.3f} | {np.sqrt(np.mean(err[:, k] ** 2)):.3f} |")
    lines += ["", "Delivered = R^T (a + g) at the platform centre, from the reported pose; see "
              "`tools/host/session_plots.py`.", ""]
    out.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", type=pathlib.Path)
    p.add_argument("--out", type=pathlib.Path, required=True)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 9})
    d = load(a.csv)
    res = plot_cue(d, a.out / "session_cue.png")
    plot_legs(d, a.out / "session_legs.png")
    summary(d, a.csv, res, a.out / "summary.md")
    print(f"wrote {a.out}/session_cue.png, session_legs.png, summary.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
