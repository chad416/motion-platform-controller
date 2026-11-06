#!/usr/bin/env python3
"""Offline closed loop on the host (SPEC week 2): the real C library through
ctypes, no servo yet.

  vehicle scenario (100 Hz) -> linear interpolation to 500 Hz
  -> mc_washout_step -> mc_ws_limit -> mc_ik -> mc_shaper_step (leg setpoints)
  -> mc_fk (pose the legs actually produce, warm start)

The cue the platform delivers is the specific force at the platform centre,
Rᵀ(a + g), from the FK pose; it is compared with the cue the washout was asked
for (the vehicle force after soft limit and gain). Writes one figure per
scenario to results/plots/ and a summary to results/closed_loop.md.

Usage: closed_loop.py [--only launch,cornering] [--no-plots]
"""
from __future__ import annotations

import argparse
import ctypes as C
import json
import math
import sys

import numpy as np

import mcue_c as M
import scenarios
import washout_ref
from mcue_ref import CONFIG, G, ROOT, rot

OUT = ROOT / "results"

# Reference categorical palette, first three slots (validated all-pairs, light):
# x/y/z and roll/pitch/yaw share the slot of their axis.
AX = ["#2a78d6", "#eb6834", "#1baf7a"]
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0"
SURFACE = "#fcfcfb"


def simulate(lib, g, s: scenarios.Scenario, overrides: dict | None = None) -> dict:
    cfg = json.loads(CONFIG.read_text())
    p = M.washout_params(lib)
    for name, value in (overrides or {}).items():  # tuning: names as in mc_washout_params
        setattr(p, name, value)
    rate = p.rate_hz
    dt = 1.0 / rate
    t, f = scenarios.upsample(s.t, s.f, rate)
    _, w = scenarios.upsample(s.t, s.w, rate)
    n = len(t)

    wo = M.Washout()
    assert lib.mc_washout_init(C.byref(wo), C.byref(p))
    sh = M.LegShaper()
    lp = M.Limit2Params(cfg["limits"]["leg_speed_m_s"], cfg["limits"]["leg_accel_m_s2"],
                        g.soft_lo, g.soft_hi, dt)
    lib.mc_shaper_init(C.byref(sh), M.D6(*([g.l0] * 6)), C.byref(lp))

    cmd, lim, q = M.Pose(), M.Pose(), M.Pose()
    scale = C.c_double()
    target = M.D6()
    info = M.FkInfo()
    rec = {k: np.zeros((n, 6)) for k in ("cmd", "lim", "pose", "target", "legs", "vel", "acc")}
    rec["scale"] = np.zeros(n)
    rec["tilt_rate"] = np.zeros((n, 2))
    fk_iter = 0
    for k in range(n):
        lib.mc_washout_step(C.byref(wo), M.D3(*f[k]), M.D3(*w[k]), C.byref(cmd))
        lib.mc_ws_limit(C.byref(g), C.byref(cmd), C.byref(lim), C.byref(scale))
        lib.mc_ik(C.byref(g), C.byref(lim), target)
        lib.mc_shaper_step(C.byref(sh), target)
        ok = lib.mc_fk(C.byref(g), sh.pos, C.byref(q), 6, C.c_double(1e-10), C.byref(info))
        assert ok, f"FK did not converge at step {k}"
        fk_iter = max(fk_iter, info.iterations)
        rec["cmd"][k] = cmd.as_list()
        rec["lim"][k] = lim.as_list()
        rec["pose"][k] = q.as_list()
        rec["target"][k] = list(target)
        rec["legs"][k] = list(sh.pos)
        rec["vel"][k] = list(sh.vel)
        rec["acc"][k] = list(sh.acc)
        rec["scale"][k] = scale.value
        rec["tilt_rate"][k] = list(wo.tilt_rate)

    # cue asked for: vehicle force after soft limit and gain (gravity removed on z)
    fa = f.copy()
    fa[:, 2] -= G
    asked = p.force_gain * washout_ref.softlim(fa, p.force_limit)
    # cue delivered: R^T (a + g) at the platform centre, gravity removed on z
    pos = rec["pose"][:, :3]
    acc = np.gradient(np.gradient(pos, dt, axis=0), dt, axis=0)
    delivered = np.zeros((n, 3))
    for k in range(n):
        R = rot(*rec["pose"][k, 3:])
        delivered[k] = R.T @ (acc[k] + [0, 0, G]) - [0, 0, G]
    rec.update(t=t, f=f, asked=asked, delivered=delivered, fk_iter=fk_iter, dt=dt,
               soft=(g.soft_lo, g.soft_hi), l0=g.l0)
    return rec


def metrics(s: scenarios.Scenario, r: dict) -> dict:
    t, pose, legs = r["t"], r["pose"], r["legs"]
    lo, hi = r["soft"]
    err = r["delivered"] - r["asked"]
    m = {
        "peak_mm": (np.max(np.abs(pose[:, :3]), axis=0) * 1e3).round(1).tolist(),
        "peak_deg": np.degrees(np.max(np.abs(pose[:, 3:]), axis=0)).round(2).tolist(),
        "tilt_rate_deg_s": float(np.degrees(np.max(np.abs(r["tilt_rate"])))),
        "leg_speed": float(np.max(np.abs(r["vel"]))),
        "leg_accel": float(np.max(np.abs(np.diff(r["vel"], axis=0)) / r["dt"])),
        "margin_mm": float(min(np.min(legs - lo), np.min(hi - legs)) * 1e3),
        "saturated_pct": float(100 * np.mean(r["scale"] < 1.0)),
        "min_scale": float(np.min(r["scale"])),
        "shaper_lag_mm": float(np.max(np.abs(legs - r["target"])) * 1e3),
        "cue_rms": np.sqrt(np.mean(err ** 2, axis=0)).round(3).tolist(),
        # relative to the cue asked for, on axes that carry one (None otherwise)
        "cue_rel": [round(float(e / a), 3) if a > 0.05 else None for e, a in
                    zip(np.sqrt(np.mean(err ** 2, axis=0)), np.sqrt(np.mean(r["asked"] ** 2, axis=0)))],
        "fk_iter": int(r["fk_iter"]),
    }
    if s.name == "launch":
        # P05: from the end of the onset ramp, |x| within 5 mm by 6 s and held while the cue lasts
        onset, end = 1.3, 9.0
        win = (t >= onset + 6.0) & (t < end)
        late = np.abs(pose[win, 0])
        m["p05_max_x_after_6s_mm"] = float(late.max() * 1e3)
        out = np.abs(pose[:, 0]) > 0.005
        settle = t[(t < end) & out]
        m["p05_settled_s"] = float(settle.max() - onset) if len(settle) else 0.0
        m["pitch_at_8s_deg"] = float(np.degrees(pose[np.searchsorted(t, 8.9), 4]))
        m["pitch_cue_deg"] = float(-np.degrees(math.asin(r["asked"][np.searchsorted(t, 8.9), 0] / G)))
    return m


def plot(s: scenarios.Scenario, r: dict, m: dict, path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({
        "font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": INK2,
        "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False, "axes.titlesize": 9.5,
        "axes.titlecolor": INK, "axes.titleweight": "bold", "axes.titlelocation": "left",
        "legend.frameon": False, "legend.fontsize": 8, "lines.linewidth": 1.6,
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    })
    t = r["t"]
    fig, axs = plt.subplots(3, 2, figsize=(11, 9), sharex=True, constrained_layout=True)
    fig.suptitle(s.title, x=0.01, ha="left", fontsize=12, fontweight="bold", color=INK)

    a = axs[0, 0]
    for i, name in enumerate("xyz"):
        a.plot(t, r["asked"][:, i], color=AX[i], lw=1.2, ls=(0, (4, 2)), alpha=0.9,
               label=f"{name} asked")
        a.plot(t, r["delivered"][:, i], color=AX[i], label=f"{name} delivered")
    a.set_title("Specific force cue (m/s², gravity removed on z)")
    a.legend(ncol=3, loc="upper right")

    a = axs[0, 1]
    for i, name in enumerate(("surge x", "sway y", "heave z")):
        a.plot(t, r["pose"][:, i] * 1e3, color=AX[i], label=name)
    a.set_title("Platform translation (mm)")
    a.legend(ncol=3, loc="upper right")

    a = axs[1, 0]
    for i, name in enumerate(("roll", "pitch", "yaw")):
        a.plot(t, np.degrees(r["pose"][:, 3 + i]), color=AX[i], label=name)
    a.set_title("Platform angles (°), tilt coordination included")
    a.legend(ncol=3, loc="upper right")

    a = axs[1, 1]
    lim = 3.0
    a.axhline(lim, color=MUTED, lw=1, ls=(0, (2, 2)))
    a.axhline(-lim, color=MUTED, lw=1, ls=(0, (2, 2)))
    a.text(t[-1], lim, "  ±3 °/s limit", color=INK2, va="bottom", ha="right", fontsize=8)
    for i, name in enumerate(("roll tilt", "pitch tilt")):
        a.plot(t, np.degrees(r["tilt_rate"][:, i]), color=AX[i], label=name)
    a.set_title(f"Tilt-coordination rate (°/s): max {m['tilt_rate_deg_s']:.3f}")
    a.set_ylim(-4, 4)
    a.legend(ncol=2, loc="lower right")

    a = axs[2, 0]
    l0 = r["l0"]
    lo, hi = r["soft"]
    st = (r["legs"] - l0) * 1e3
    a.fill_between(t, st.min(axis=1), st.max(axis=1), color=AX[0], alpha=0.25, lw=0,
                   label="range of the six legs")
    a.plot(t, st.max(axis=1), color=AX[0], lw=1.2)
    a.plot(t, st.min(axis=1), color=AX[0], lw=1.2)
    for v, txt in (((hi - l0) * 1e3, "soft limit"), ((lo - l0) * 1e3, "soft limit")):
        a.axhline(v, color=MUTED, lw=1, ls=(0, (2, 2)))
        a.text(t[0], v, f" {txt}", color=INK2, va="bottom" if v > 0 else "top", fontsize=8)
    a.set_title(f"Leg stroke from neutral (mm): closest to a soft limit {m['margin_mm']:.1f} mm")
    a.set_xlabel("time (s)")
    a.legend(loc="lower right")

    a = axs[2, 1]
    a.plot(t, r["scale"], color=AX[0])
    a.set_ylim(min(0.0, r["scale"].min() - 0.05), 1.05)
    a.set_title(f"Workspace scale (1 = reachable): saturated {m['saturated_pct']:.1f} % of steps")
    a.set_xlabel("time (s)")

    fig.savefig(path, dpi=110)
    plt.close(fig)


def table(rows: list[tuple[scenarios.Scenario, dict]]) -> str:
    out = ["# Offline closed loop: results", "",
           "Generated by `tools/reference/closed_loop.py` (C library through ctypes, 500 Hz, no servo).",
           "Cue error is delivered minus asked specific force (at the platform centre), per axis: "
           "RMS, and RMS relative to the asked cue's RMS (– where an axis carries no cue).", "",
           "| Scenario | Peak x/y/z (mm) | Peak roll/pitch/yaw (°) | Max tilt rate (°/s) | Max leg speed (m/s) "
           "| Max leg accel (m/s²) | Closest to soft limit (mm) | Saturated | Cue error RMS x/y/z (m/s²) "
           "| Relative x/y/z |",
           "|---|---|---|---|---|---|---|---|---|---|"]
    for s, m in rows:
        rel = " / ".join("–" if v is None else f"{v:.2f}" for v in m["cue_rel"])
        out.append(
            f"| {s.name} | {' / '.join(f'{v:.0f}' for v in m['peak_mm'])} "
            f"| {' / '.join(f'{v:.1f}' for v in m['peak_deg'])} | {m['tilt_rate_deg_s']:.3f} "
            f"| {m['leg_speed']:.3f} | {m['leg_accel']:.2f} | {m['margin_mm']:.1f} "
            f"| {m['saturated_pct']:.1f} % | {' / '.join(f'{v:.2f}' for v in m['cue_rms'])} | {rel} |")
    launch = next((m for s, m in rows if s.name == "launch"), None)
    if launch:
        out += ["", "**P05 (launch, sustained 3 m/s²):** translation within 5 mm of neutral "
                f"{launch['p05_settled_s']:.2f} s after the onset (limit 6 s); largest |x| from 6 s until the "
                f"cue ends: {launch['p05_max_x_after_6s_mm']:.2f} mm. Pitch holds "
                f"{launch['pitch_at_8s_deg']:.2f}° for a sustained cue of {launch['pitch_cue_deg']:.2f}°."]
    return "\n".join(out) + "\n"


def acceptance(rows: list[tuple[scenarios.Scenario, dict]]) -> list[str]:
    """SPEC P03/P04/P05 and the tuning rule, on the closed-loop metrics. Returns failures."""
    cfg = json.loads(CONFIG.read_text())
    lim, wo = cfg["limits"], cfg["washout"]
    fails = []
    for s, m in rows:
        if m["margin_mm"] < 0.0:
            fails.append(f"{s.name}: P03 leg {-m['margin_mm']:.4f} mm past a soft limit")
        if m["leg_speed"] > lim["leg_speed_m_s"] * (1 + 1e-9):
            fails.append(f"{s.name}: P03 leg speed {m['leg_speed']:.4f} m/s")
        if m["leg_accel"] > lim["leg_accel_m_s2"] * (1 + 1e-6):
            fails.append(f"{s.name}: P03 leg acceleration {m['leg_accel']:.4f} m/s²")
        if m["tilt_rate_deg_s"] > wo["tilt_rate_limit_deg_s"] * (1 + 1e-9):
            fails.append(f"{s.name}: P04 tilt rate {m['tilt_rate_deg_s']:.4f} °/s")
        if s.name != "excessive" and m["saturated_pct"] > 0.0:
            fails.append(f"{s.name}: saturates the workspace ({m['saturated_pct']:.1f} %)")
        if s.name == "launch" and m["p05_settled_s"] > 6.0:
            fails.append(f"launch: P05 washout took {m['p05_settled_s']:.2f} s")
    return fails


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if P03/P04/P05 or the no-saturation rule fail (CI)")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override a washout parameter (mc_washout_params field, SI units) for "
                         "tuning; results are then printed only, not written")
    a = ap.parse_args()
    overrides = {k: float(v) for k, v in (kv.split("=", 1) for kv in a.set)}
    lib = M.load()
    g = M.geometry(lib)
    (OUT / "plots").mkdir(parents=True, exist_ok=True)
    rows = []
    for make in scenarios.ALL:
        s = make()
        if a.only and s.name not in a.only.split(","):
            continue
        r = simulate(lib, g, s, overrides)
        m = metrics(s, r)
        rows.append((s, m))
        print(f"{s.name:12s} {json.dumps(m)}")
        if not a.no_plots and not overrides:
            plot(s, r, m, OUT / "plots" / f"{s.name}.png")
    fails = acceptance(rows)
    print("acceptance (P03, P04, P05, no saturation in normal scenarios): "
          + ("PASS" if not fails else "FAIL\n  " + "\n  ".join(fails)))
    if overrides:
        print(table(rows))
    elif not a.only:
        (OUT / "closed_loop.md").write_text(table(rows), encoding="utf-8")
        (OUT / "closed_loop.json").write_text(json.dumps({s.name: m for s, m in rows}, indent=1)
                                              + "\n")
        print(f"wrote {OUT.relative_to(ROOT)}/closed_loop.md, closed_loop.json"
              + ("" if a.no_plots else ", plots/"))
    return 1 if (a.check and fails) else 0


if __name__ == "__main__":
    sys.exit(main())
