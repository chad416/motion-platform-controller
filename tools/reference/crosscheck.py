#!/usr/bin/env python3
"""Cross-check the C library (built as a host shared library) against the NumPy
reference on random poses: IK, FK, statics and workspace scaling.

Usage: crosscheck.py [--n 1000] [--lib build/libmcue.so]
"""
from __future__ import annotations

import argparse
import ctypes as C
import json
import math
import pathlib
import subprocess
import sys

import numpy as np

from mcue_ref import CONFIG, ROOT, fk, ik, load_geometry, statics, ws_limit

D6 = C.c_double * 6


class Vec3(C.Structure):
    _fields_ = [("x", C.c_double), ("y", C.c_double), ("z", C.c_double)]


class Pose(C.Structure):
    _fields_ = [(n, C.c_double) for n in ("x", "y", "z", "roll", "pitch", "yaw")]


class Geometry(C.Structure):
    _fields_ = [("base", Vec3 * 6), ("plat", Vec3 * 6)] + [
        (n, C.c_double) for n in ("h0", "l0", "l_min", "l_max", "soft_lo", "soft_hi")]


class Params(C.Structure):
    _fields_ = [(n, C.c_double) for n in ("base_radius", "platform_radius", "base_half_deg",
                                          "platform_half_deg", "neutral_height", "stroke",
                                          "soft_margin")]


class FkInfo(C.Structure):
    _fields_ = [("iterations", C.c_int), ("residual", C.c_double), ("converged", C.c_bool)]


def build(lib: pathlib.Path) -> None:
    lib.parent.mkdir(parents=True, exist_ok=True)
    src = sorted(str(p) for p in (ROOT / "mcue" / "src").glob("*.c"))
    subprocess.run(["gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                    "-I", str(ROOT / "mcue" / "include"), *src, "-lm", "-o", str(lib)], check=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--lib", type=pathlib.Path, default=ROOT / "build" / "libmcue.so")
    a = ap.parse_args()
    build(a.lib)
    lib = C.CDLL(str(a.lib))
    lib.mc_geometry_build.restype = C.c_bool
    lib.mc_fk.restype = C.c_bool
    lib.mc_statics.restype = C.c_bool
    lib.mc_ws_limit.restype = C.c_int

    params = Params()
    lib.mc_geometry_default_params(C.byref(params))
    g = Geometry()
    assert lib.mc_geometry_build(C.byref(g), C.byref(params))
    ref = load_geometry()
    payload = json.loads(CONFIG.read_text())["payload"]
    com = Vec3(*payload["com_platform_m"])

    rng = np.random.default_rng(7)
    lim = np.array([0.4, 0.4, 0.25, math.radians(25), math.radians(25), math.radians(50)])
    worst = {"ik": 0.0, "fk": 0.0, "statics": 0.0, "scale": 0.0}
    for _ in range(a.n):
        q = rng.uniform(-lim, lim)
        pose = Pose(*q)

        legs = D6()
        lib.mc_ik(C.byref(g), C.byref(pose), legs)
        worst["ik"] = max(worst["ik"], np.max(np.abs(np.array(legs) - ik(ref, q))))

        start = Pose()
        info = FkInfo()
        lib.mc_fk(C.byref(g), legs, C.byref(start), 10, C.c_double(1e-12), C.byref(info))
        q_ref, _, _ = fk(ref, np.array(legs), iters=10, tol=1e-12)
        c_fk = np.array([start.x, start.y, start.z, start.roll, start.pitch, start.yaw])
        worst["fk"] = max(worst["fk"], np.max(np.abs(c_fk - q_ref)))

        f = D6()
        assert lib.mc_statics(C.byref(g), C.byref(pose), C.c_double(payload["mass_kg"]), C.byref(com), f)
        worst["statics"] = max(worst["statics"], np.max(np.abs(np.array(f) - statics(ref, q, payload["mass_kg"], payload["com_platform_m"]))))

        out, scale = Pose(), C.c_double()
        lib.mc_ws_limit(C.byref(g), C.byref(pose), C.byref(out), C.byref(scale))
        worst["scale"] = max(worst["scale"], abs(scale.value - ws_limit(ref, q)[1]))

    print(f"C vs NumPy on {a.n} random poses: max |IK| {worst['ik']:.2e} m, max |FK| {worst['fk']:.2e}, "
          f"max |statics| {worst['statics']:.2e} N, max |scale| {worst['scale']:.2e}")
    ok = worst["ik"] < 1e-12 and worst["fk"] < 1e-9 and worst["statics"] < 1e-6 and worst["scale"] <= 1 / 4096
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
