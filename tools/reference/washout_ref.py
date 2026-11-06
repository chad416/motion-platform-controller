"""SciPy reference for the classical washout (SPEC F04), written independently of
mcue/src/washout.c: filters from scipy.signal.bilinear, run with lfilter; the
tilt rate limit and the Euler-rate integration are explicit loops.

Parameters come from config/platform.json ("washout"), same as the C defaults.
"""
from __future__ import annotations

import json
import math

import numpy as np
from scipy import signal

from mcue_ref import CONFIG, G


def params() -> dict:
    w = dict(json.loads(CONFIG.read_text())["washout"])
    w["tilt_rate_limit"] = math.radians(w.pop("tilt_rate_limit_deg_s"))
    w["tilt_accel_limit"] = math.radians(w.pop("tilt_accel_limit_deg_s2"))
    w["force_limit"] = w.pop("force_limit_m_s2")
    return w


def softlim(x: np.ndarray, lim: float) -> np.ndarray:
    """Linear up to lim/2, then a tanh knee with slope 1 at the joint, towards +-lim."""
    knee = lim / 2
    ax = np.abs(x)
    y = np.where(ax <= knee, ax, knee + (lim - knee) * np.tanh((ax - knee) / (lim - knee)))
    return np.sign(x) * y


def limit2(x: float, v: float, prev: float, target: float, vmax: float, amax: float, dt: float,
           lo: float = -math.inf, hi: float = math.inf):
    """Second-order limiter (SPEC F05), same law as mc_limit2_step: follow the
    target exactly when every limit allows it (speed, acceleration, matching the
    target's own velocity, and still able to stop before lo / hi); otherwise
    brake towards it for the distance less amax dt^2 / 8. Returns (x, v)."""
    e = target - x
    vt = (target - prev) / dt
    vl = e / dt
    reach = vl * vl / (2 * amax) + abs(vl) * dt / 2 + amax * dt * dt / 8
    can_stop = target + reach <= hi if vl >= 0 else target - reach >= lo
    if abs(vl) <= vmax and abs((vl - v) / dt) <= amax and abs(vl - vt) / dt <= amax and can_stop:
        return target, vl
    d = 1.0 if e >= 0 else -1.0
    room = max(0.0, abs(e) - amax * dt * dt / 8)
    vb = amax * (-dt / 2 + math.sqrt(dt * dt / 4 + 2 * room / amax))
    a = min(amax, max(-amax, (d * min(vmax, vb) - v) / dt))
    v2 = v + a * dt
    return x + v2 * dt, v2


def washout(f: np.ndarray, w: np.ndarray, p: dict | None = None) -> dict[str, np.ndarray]:
    """f: (N, 3) specific force incl. +g on z, w: (N, 3) body rates.
    Returns pose (N, 6) and the intermediate signals used for plots."""
    p = p or params()
    fs = p["rate_hz"]
    dt = 1.0 / fs
    n = len(f)

    fa = np.array(f, dtype=float)
    fa[:, 2] -= G
    fsc = p["force_gain"] * softlim(fa, p["force_limit"])

    pos = np.zeros((n, 3))
    for a in range(3):
        wn, z, wb = ((p["surge_sway_wn"], p["surge_sway_zeta"], p["surge_sway_wb"]) if a < 2
                     else (p["heave_wn"], p["heave_zeta"], p["heave_wb"]))
        b1, a1 = signal.bilinear([1, 0], [1, wb], fs)
        b2, a2 = signal.bilinear([1], [1, 2 * z * wn, wn * wn], fs)
        pos[:, a] = signal.lfilter(b2, a2, signal.lfilter(b1, a1, fsc[:, a]))

    wn, z = p["tilt_wn"], p["tilt_zeta"]
    bl, al = signal.bilinear([wn * wn], [1, 2 * z * wn, wn * wn], fs)
    f_lp = np.stack([signal.lfilter(bl, al, fsc[:, a]) for a in (0, 1)], axis=1)
    target = np.stack([np.arcsin(np.clip(f_lp[:, 1] / G, -1, 1)),    # roll  = +asin(fy/g)
                       -np.arcsin(np.clip(f_lp[:, 0] / G, -1, 1))],  # pitch = -asin(fx/g)
                      axis=1)
    tilt = np.zeros((n, 2))
    cur, rate, prev = [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]
    for k in range(n):
        for a in (0, 1):
            cur[a], rate[a] = limit2(cur[a], rate[a], prev[a], target[k, a], p["tilt_rate_limit"],
                                     p["tilt_accel_limit"], dt)
            prev[a] = target[k, a]
        tilt[k] = cur

    wn, z = p["rot_wn"], p["rot_zeta"]
    bh, ah = signal.bilinear([1, 0, 0], [1, 2 * z * wn, wn * wn], fs)
    r = np.stack([signal.lfilter(bh, ah, p["rot_gain"] * np.asarray(w)[:, a]) for a in range(3)],
                 axis=1)
    ang = np.zeros((n, 3))
    cur = np.zeros(3)
    for k in range(n):
        phi, th = cur[0], cur[1]  # rotational channel's own angles, not the tilt
        pp, q, rr = r[k]
        s = q * math.sin(phi) + rr * math.cos(phi)
        cur = cur + dt * np.array([pp + s * math.tan(th),
                                   q * math.cos(phi) - rr * math.sin(phi),
                                   s / math.cos(th)])
        ang[k] = cur

    pose = np.column_stack([pos, ang[:, 0] + tilt[:, 0], ang[:, 1] + tilt[:, 1], ang[:, 2]])
    return {"pose": pose, "f_scaled": fsc, "tilt": tilt, "target_tilt": target}
