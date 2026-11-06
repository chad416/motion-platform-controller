"""NumPy reference for the mcue kinematics: geometry, IK, Jacobian, FK, statics,
workspace limiting. Written independently of the C code, so the two can check
each other (tools/reference/gen_headers.py, tools/reference/crosscheck.py).

Conventions (SPEC §3): base frame x forward, y left, z up. Pose = (x, y, z, roll,
pitch, yaw), position relative to neutral, angles ZYX: R = Rz(yaw) Ry(pitch) Rx(roll).
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import dataclass

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
CONFIG = ROOT / "config" / "platform.json"
G = 9.80665


@dataclass
class Geometry:
    base: np.ndarray  # (6, 3) base joints a_i, base frame
    plat: np.ndarray  # (6, 3) platform joints b_i, platform frame
    h0: float
    l0: float
    l_min: float
    l_max: float
    margin: float

    @property
    def soft_lo(self) -> float:
        return self.l_min + self.margin

    @property
    def soft_hi(self) -> float:
        return self.l_max - self.margin


def build_geometry(rb, rp, half_base_deg, half_plat_deg, h0, stroke, margin) -> Geometry:
    """Base joint pairs at 120k +- half_base; platform joints at 120k +- (60 - half_plat).
    Leg 2k joins base 120k - half_base to platform 120k - (60 - half_plat); leg 2k+1 mirrors it."""
    base, plat = [], []
    for k in range(3):
        c = 120.0 * k
        for sign in (-1.0, 1.0):
            ab = math.radians(c + sign * half_base_deg)
            ap = math.radians(c + sign * (60.0 - half_plat_deg))
            base.append([rb * math.cos(ab), rb * math.sin(ab), 0.0])
            plat.append([rp * math.cos(ap), rp * math.sin(ap), 0.0])
    base, plat = np.array(base), np.array(plat)
    l0 = float(np.linalg.norm(np.array([0, 0, h0]) + plat[0] - base[0]))
    return Geometry(base, plat, h0, l0, l0 - stroke / 2, l0 + stroke / 2, margin)


def load_geometry(path: pathlib.Path = CONFIG) -> Geometry:
    c = json.loads(path.read_text())["geometry"]
    return build_geometry(c["base_radius_m"], c["platform_radius_m"], c["base_half_angle_deg"],
                          c["platform_half_angle_deg"], c["neutral_height_m"], c["stroke_m"],
                          c["soft_margin_m"])


def rot(roll, pitch, yaw) -> np.ndarray:
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    return Rz @ Ry @ Rx


def leg_vectors(g: Geometry, q) -> np.ndarray:
    p = np.array([q[0], q[1], q[2] + g.h0])
    return p + (rot(q[3], q[4], q[5]) @ g.plat.T).T - g.base


def ik(g: Geometry, q) -> np.ndarray:
    return np.linalg.norm(leg_vectors(g, q), axis=1)


def jacobian(g: Geometry, q) -> np.ndarray:
    """d l_i / d q, q = (x, y, z, roll, pitch, yaw)."""
    L = leg_vectors(g, q)
    n = L / np.linalg.norm(L, axis=1)[:, None]
    R = rot(q[3], q[4], q[5])
    r = (R @ g.plat.T).T
    roll, pitch, yaw = q[3], q[4], q[5]
    # omega = E @ (roll_dot, pitch_dot, yaw_dot) for R = Rz Ry Rx
    E = np.array([[math.cos(yaw) * math.cos(pitch), -math.sin(yaw), 0.0],
                  [math.sin(yaw) * math.cos(pitch), math.cos(yaw), 0.0],
                  [-math.sin(pitch), 0.0, 1.0]])
    rxn = np.cross(r, n)
    return np.hstack([n, rxn @ E])


def fk(g: Geometry, legs, q0=None, iters=6, tol=1e-9):
    q = np.zeros(6) if q0 is None else np.array(q0, float)
    for i in range(iters):
        dq = np.linalg.solve(jacobian(g, q), legs - ik(g, q))
        q = q + dq
        if np.max(np.abs(dq)) < tol:
            return q, True, i + 1
    return q, bool(np.max(np.abs(legs - ik(g, q))) < 1e-9), iters


def statics(g: Geometry, q, mass, com_plat) -> np.ndarray:
    """Leg forces (N, + = compression) holding a payload of `mass` at `com_plat` (platform frame)."""
    L = leg_vectors(g, q)
    n = L / np.linalg.norm(L, axis=1)[:, None]
    R = rot(q[3], q[4], q[5])
    r = (R @ g.plat.T).T
    Jt = np.hstack([n, np.cross(r, n)])  # twist Jacobian rows
    w = np.array([0.0, 0.0, mass * G])
    W = np.concatenate([w, np.cross(R @ np.asarray(com_plat), w)])
    return np.linalg.solve(Jt.T, W)


def valid(g: Geometry, q) -> bool:
    legs = ik(g, q)
    return bool(np.all((legs >= g.soft_lo) & (legs <= g.soft_hi)))


def ws_limit(g: Geometry, q_cmd, iters=12):
    """Scale the command towards neutral by bisection until every leg is inside the soft limits.
    Returns (q_out, scale)."""
    q_cmd = np.asarray(q_cmd, float)
    if valid(g, q_cmd):
        return q_cmd.copy(), 1.0
    lo, hi = 0.0, 1.0  # lo always valid (neutral), hi invalid
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if valid(g, mid * q_cmd):
            lo = mid
        else:
            hi = mid
    return lo * q_cmd, lo
