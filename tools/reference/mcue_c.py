"""ctypes bindings for the mcue C library, built as a host shared library.

Used by crosscheck.py (C vs reference) and closed_loop.py (offline simulation).
Structures mirror mcue/include/mcue/*.h; keep them in step.
"""
from __future__ import annotations

import ctypes as C
import pathlib
import subprocess

from mcue_ref import ROOT

D6 = C.c_double * 6
D3 = C.c_double * 3
LIB = ROOT / "build" / "libmcue.so"


class Vec3(C.Structure):
    _fields_ = [("x", C.c_double), ("y", C.c_double), ("z", C.c_double)]


class Pose(C.Structure):
    _fields_ = [(n, C.c_double) for n in ("x", "y", "z", "roll", "pitch", "yaw")]

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]


class Geometry(C.Structure):
    _fields_ = [("base", Vec3 * 6), ("plat", Vec3 * 6)] + [
        (n, C.c_double) for n in ("h0", "l0", "l_min", "l_max", "soft_lo", "soft_hi")]


class Params(C.Structure):
    _fields_ = [(n, C.c_double) for n in ("base_radius", "platform_radius", "base_half_deg",
                                          "platform_half_deg", "neutral_height", "stroke",
                                          "soft_margin")]


class FkInfo(C.Structure):
    _fields_ = [("iterations", C.c_int), ("residual", C.c_double), ("converged", C.c_bool)]


class WashoutParams(C.Structure):
    _fields_ = [(n, C.c_double) for n in (
        "rate_hz", "force_gain", "force_limit",
        "surge_sway_wn", "surge_sway_zeta", "surge_sway_wb",
        "heave_wn", "heave_zeta", "heave_wb",
        "tilt_wn", "tilt_zeta", "tilt_rate_limit", "tilt_accel_limit",
        "rot_gain", "rot_wn", "rot_zeta")]


class Biquad(C.Structure):
    _fields_ = [(n, C.c_double) for n in ("b0", "b1", "b2", "a1", "a2", "z1", "z2")]


class Washout(C.Structure):
    _fields_ = [("p", WashoutParams), ("dt", C.c_double),
                ("trans_hp1", Biquad * 3), ("trans_res", Biquad * 3), ("tilt_lp", Biquad * 2),
                ("rot_hp", Biquad * 3), ("tilt", C.c_double * 2), ("angle", C.c_double * 3),
                ("tilt_rate", C.c_double * 2), ("tilt_prev", C.c_double * 2)]


class Limit2Params(C.Structure):
    _fields_ = [(n, C.c_double) for n in ("v_max", "a_max", "lo", "hi", "dt")]


class LegShaper(C.Structure):
    _fields_ = [("pos", D6), ("vel", D6), ("acc", D6), ("prev", D6), ("lim", Limit2Params)]


def build(lib: pathlib.Path = LIB) -> None:
    lib.parent.mkdir(parents=True, exist_ok=True)
    src = sorted(str(p) for p in (ROOT / "mcue" / "src").glob("*.c"))
    subprocess.run(["gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                    "-I", str(ROOT / "mcue" / "include"), *src, "-lm", "-o", str(lib)], check=True)


def load(lib: pathlib.Path = LIB, rebuild: bool = True) -> C.CDLL:
    if rebuild or not lib.exists():
        build(lib)
    h = C.CDLL(str(lib))
    for name in ("mc_geometry_build", "mc_fk", "mc_statics", "mc_washout_init", "mc_biquad_tustin"):
        getattr(h, name).restype = C.c_bool
    h.mc_ws_limit.restype = C.c_int
    h.mc_scurve.restype = C.c_double
    h.mc_scurve.argtypes = [C.c_double, C.c_double]
    return h


def geometry(h: C.CDLL) -> Geometry:
    params = Params()
    h.mc_geometry_default_params(C.byref(params))
    g = Geometry()
    if not h.mc_geometry_build(C.byref(g), C.byref(params)):
        raise RuntimeError("mc_geometry_build failed")
    return g


def washout_params(h: C.CDLL) -> WashoutParams:
    p = WashoutParams()
    h.mc_washout_default_params(C.byref(p))
    return p
