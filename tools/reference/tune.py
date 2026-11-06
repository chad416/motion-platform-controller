#!/usr/bin/env python3
"""Grid search of the washout parameters on the offline closed loop (closed_loop.py).

Objective: mean relative cue error (RMS of delivered - asked, over RMS of asked)
on every axis that carries a cue, over the normal scenarios.
Constraints: no workspace saturation in the normal scenarios, no leg past a soft
limit, and P05 (launch translation within 5 mm of neutral within 6 s).

Surge/sway + tilt and heave are independent channels and are searched apart
(heave is scored on the bump only).

Usage: tune.py [--quick]     # prints the ranking; does not change config/platform.json
"""
from __future__ import annotations

import argparse
import itertools
import math
import sys

import numpy as np

import closed_loop
import mcue_c as M
import scenarios

NORMAL = [scenarios.launch, scenarios.braking, scenarios.lane_change, scenarios.cornering]


def score(lib, g, makers, overrides) -> tuple[float, bool, dict]:
    rel, feasible, info = [], True, {}
    for make in makers:
        s = make()
        m = closed_loop.metrics(s, closed_loop.simulate(lib, g, s, overrides))
        rel += [v for v in m["cue_rel"] if v is not None]
        feasible &= m["saturated_pct"] == 0.0 and m["margin_mm"] >= 0.0
        if s.name == "launch":
            feasible &= m["p05_settled_s"] <= 6.0
            info["p05_s"] = round(m["p05_settled_s"], 2)
        info[s.name] = m["cue_rel"]
        info[f"{s.name}_margin_mm"] = round(m["margin_mm"], 1)
    return float(np.mean(rel)), feasible, info


def search(lib, g, makers, grid: dict, top: int = 8) -> list:
    names = list(grid)
    rows = []
    for values in itertools.product(*(grid[n] for n in names)):
        ov = dict(zip(names, values))
        if "tilt_rate_limit" in ov:
            ov["tilt_rate_limit"] = math.radians(ov["tilt_rate_limit"])
        J, ok, info = score(lib, g, makers, ov)
        rows.append((not ok, J, dict(zip(names, values)), info))
        print(f"{'ok ' if ok else 'BAD'} J={J:.3f} {dict(zip(names, values))}", flush=True)
    rows.sort(key=lambda r: (r[0], r[1]))
    return rows[:top]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="small grid, for a smoke run")
    a = ap.parse_args()
    lib = M.load()
    g = M.geometry(lib)

    if a.quick:
        trans = {"force_gain": [0.5, 0.6], "tilt_wn": [1.5, 4.0]}
        heave = {"heave_wn": [2.0, 4.0]}
    else:
        trans = {"force_gain": [0.5, 0.55, 0.6], "surge_sway_wn": [2.0, 2.5, 3.0],
                 "surge_sway_wb": [0.5, 1.0], "tilt_wn": [1.0, 1.5, 2.0, 4.0]}
        heave = {"heave_wn": [1.5, 2.0, 3.0, 4.0], "heave_wb": [0.5, 1.0]}

    print("== surge / sway / tilt ==")
    best_t = search(lib, g, NORMAL, trans)
    print("== heave (bump) ==")
    best_h = search(lib, g, [scenarios.bump], heave)

    for title, best in (("surge/sway/tilt", best_t), ("heave", best_h)):
        print(f"\nbest {title}:")
        for bad, J, ov, info in best:
            print(f"  {'INFEASIBLE ' if bad else ''}J={J:.3f} {ov} {info}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
