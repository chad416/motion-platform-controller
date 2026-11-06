#!/usr/bin/env python3
"""Cross-check the C library (built as a host shared library) against the
independent references:

  kinematics  NumPy (mcue_ref.py) on random poses: IK, FK, statics, workspace scale
  washout     SciPy (washout_ref.py) on every vehicle scenario (scenarios.py)

Usage: crosscheck.py [--n 1000]
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
from mcue_ref import CONFIG, fk, ik, load_geometry, statics, ws_limit


def check_kinematics(lib, n: int) -> bool:
    g = M.geometry(lib)
    ref = load_geometry()
    payload = json.loads(CONFIG.read_text())["payload"]
    com = M.Vec3(*payload["com_platform_m"])

    rng = np.random.default_rng(7)
    lim = np.array([0.4, 0.4, 0.25, math.radians(25), math.radians(25), math.radians(50)])
    worst = {"ik": 0.0, "fk": 0.0, "statics": 0.0, "scale": 0.0}
    for _ in range(n):
        q = rng.uniform(-lim, lim)
        pose = M.Pose(*q)

        legs = M.D6()
        lib.mc_ik(C.byref(g), C.byref(pose), legs)
        worst["ik"] = max(worst["ik"], np.max(np.abs(np.array(legs) - ik(ref, q))))

        start = M.Pose()
        info = M.FkInfo()
        lib.mc_fk(C.byref(g), legs, C.byref(start), 10, C.c_double(1e-12), C.byref(info))
        q_ref, _, _ = fk(ref, np.array(legs), iters=10, tol=1e-12)
        worst["fk"] = max(worst["fk"], np.max(np.abs(np.array(start.as_list()) - q_ref)))

        f = M.D6()
        assert lib.mc_statics(C.byref(g), C.byref(pose), C.c_double(payload["mass_kg"]),
                              C.byref(com), f)
        f_ref = statics(ref, q, payload["mass_kg"], payload["com_platform_m"])
        worst["statics"] = max(worst["statics"], np.max(np.abs(np.array(f) - f_ref)))

        out, scale = M.Pose(), C.c_double()
        lib.mc_ws_limit(C.byref(g), C.byref(pose), C.byref(out), C.byref(scale))
        worst["scale"] = max(worst["scale"], abs(scale.value - ws_limit(ref, q)[1]))

    print(f"kinematics, C vs NumPy on {n} random poses: max |IK| {worst['ik']:.2e} m, "
          f"max |FK| {worst['fk']:.2e}, max |statics| {worst['statics']:.2e} N, "
          f"max |scale| {worst['scale']:.2e}")
    return (worst["ik"] < 1e-12 and worst["fk"] < 1e-9 and worst["statics"] < 1e-6
            and worst["scale"] <= 1 / 4096)


def run_c_washout(lib, f: np.ndarray, w: np.ndarray) -> np.ndarray:
    wo = M.Washout()
    p = M.washout_params(lib)
    assert lib.mc_washout_init(C.byref(wo), C.byref(p)), "washout filters unstable"
    out = np.zeros((len(f), 6))
    cmd = M.Pose()
    for k in range(len(f)):
        lib.mc_washout_step(C.byref(wo), M.D3(*f[k]), M.D3(*w[k]), C.byref(cmd))
        out[k] = cmd.as_list()
    return out


def check_washout(lib) -> bool:
    p = M.washout_params(lib)
    rate = p.rate_hz
    ok = True
    for make in scenarios.ALL:
        s = make()
        _, f = scenarios.upsample(s.t, s.f, rate)
        _, w = scenarios.upsample(s.t, s.w, rate)
        c = run_c_washout(lib, f, w)
        r = washout_ref.washout(f, w)["pose"]
        err = np.max(np.abs(c - r), axis=0)
        good = err[:3].max() < 1e-9 and err[3:].max() < 1e-9
        ok &= good
        print(f"washout, C vs SciPy, {s.name:12s} {len(f):5d} steps: max |pos| {err[:3].max():.1e} m, "
              f"max |angle| {err[3:].max():.1e} rad {'' if good else '  <-- FAIL'}")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1000)
    a = ap.parse_args()
    lib = M.load()
    ok = check_kinematics(lib, a.n)
    ok &= check_washout(lib)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
