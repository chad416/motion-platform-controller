#!/usr/bin/env python3
"""Check a platform geometry: neutral leg length, single-axis and combined-pose
workspace, Jacobian conditioning, and leg forces under the payload.

Usage: geometry_check.py                       # config/platform.json
       geometry_check.py --compare 10 20 30    # compare platform half-angles
"""
from __future__ import annotations

import argparse
import itertools
import json
import math

import numpy as np

from mcue_ref import CONFIG, build_geometry, ik, jacobian, load_geometry, statics, valid

AXES = [("surge m", 0, 1.0, 0.6), ("sway m", 1, 1.0, 0.6), ("heave m", 2, 1.0, 0.4),
        ("roll deg", 3, math.pi / 180, 60), ("pitch deg", 4, math.pi / 180, 60),
        ("yaw deg", 5, math.pi / 180, 90)]


def single_axis(g):
    out = {}
    for name, idx, unit, top in AXES:
        rng = []
        for sign in (+1, -1):
            ok = 0.0
            for v in np.linspace(0, top, 6001):
                q = np.zeros(6)
                q[idx] = sign * v * unit
                if valid(g, q):
                    ok = v
                else:
                    break
            rng.append(sign * ok)
        out[name] = rng
    return out


def combined_ok(g):
    """Fraction of combined demands (all sign combinations) that are reachable."""
    demands = {"surge 0.10 m + pitch 8 deg": (0.10, 0, 0, 0, math.radians(8), 0),
               "sway 0.10 m + roll 8 deg": (0, 0.10, 0, math.radians(8), 0, 0),
               "heave 0.06 m + pitch 8 + roll 8": (0, 0, 0.06, math.radians(8), math.radians(8), 0),
               "yaw 15 deg + sway 0.08 m": (0, 0.08, 0, 0, 0, math.radians(15))}
    res = {}
    for name, d in demands.items():
        d = np.array(d)
        signs = list(itertools.product([-1, 1], repeat=6))
        ok = sum(valid(g, d * np.array(s)) for s in signs)
        res[name] = f"{ok}/{len(signs)}"
    return res


def conditioning(g, n=2000, seed=1):
    rng = np.random.default_rng(seed)
    conds = [np.linalg.cond(jacobian(g, np.zeros(6)))]
    lim = np.array([0.15, 0.15, 0.10, math.radians(12), math.radians(12), math.radians(20)])
    while len(conds) < n:
        q = rng.uniform(-lim, lim)
        if valid(g, q):
            conds.append(np.linalg.cond(jacobian(g, q)))
    return conds[0], max(conds)


def report(g, mass, com, label=""):
    print(f"== {label}  l0 = {g.l0:.4f} m, legs {g.l_min:.3f}..{g.l_max:.3f} m "
          f"(soft {g.soft_lo:.3f}..{g.soft_hi:.3f}), leg angle {math.degrees(math.acos(g.h0 / g.l0)):.1f} deg from vertical")
    for k, v in single_axis(g).items():
        print(f"   {k:10s} +{v[0]:.3f} / {v[1]:.3f}")
    for k, v in combined_ok(g).items():
        print(f"   {k:34s} reachable in {v} sign combinations")
    c0, cmax = conditioning(g)
    f0 = statics(g, np.zeros(6), mass, com)
    print(f"   Jacobian condition number: neutral {c0:.1f}, worst in workspace {cmax:.1f}")
    print(f"   leg forces at neutral, {mass:.0f} kg payload: {np.round(f0).astype(int)} N (+ = compression)")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--compare", nargs="*", type=float, help="platform half-angles to compare")
    a = p.parse_args()
    cfg = json.loads(CONFIG.read_text())
    gc, pl = cfg["geometry"], cfg["payload"]
    if a.compare:
        for hp in a.compare:
            g = build_geometry(gc["base_radius_m"], gc["platform_radius_m"], gc["base_half_angle_deg"],
                               hp, gc["neutral_height_m"], gc["stroke_m"], gc["soft_margin_m"])
            report(g, pl["mass_kg"], pl["com_platform_m"], f"platform half-angle {hp:g} deg")
    else:
        report(load_geometry(), pl["mass_kg"], pl["com_platform_m"], "config/platform.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
