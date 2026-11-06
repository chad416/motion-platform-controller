"""Vehicle scenarios for motion cueing: specific force and body angular rate at
the driver's head, sampled at 100 Hz (the VEHICLE_STATE rate, SPEC F09).

Frames as in SPEC §3: x forward, y left, z up. Specific force f = a - g_vec, so
a vehicle at rest reads f = (0, 0, +g). A left turn gives f_y > 0 and yaw rate
r > 0. The vehicle body's own pitch/roll are modelled through the body rates
only; the gravity component of the vehicle's tilt is left out of f (it is small
next to the manoeuvre forces and keeps the scenarios easy to read).

Usage: scenarios.py            # list scenarios with their peak values
       scenarios.py --export   # write sim/data/<name>.csv for the Renode tests
       scenarios.py --check    # fail if those files are stale (CI)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from mcue_ref import G

RATE_HZ = 100.0


@dataclass
class Scenario:
    name: str
    title: str
    t: np.ndarray  # (N,)
    f: np.ndarray  # (N, 3) specific force, m/s^2
    w: np.ndarray  # (N, 3) body rate, rad/s
    note: str = ""


def smoothstep(t: np.ndarray, t0: float, rise: float) -> np.ndarray:
    """0 before t0, 1 after t0 + rise, quintic in between (jerk-limited onset)."""
    u = np.clip((t - t0) / rise, 0.0, 1.0)
    return u ** 3 * (10 - 15 * u + 6 * u * u)


def pulse(t: np.ndarray, t0: float, t1: float, rise: float) -> np.ndarray:
    return smoothstep(t, t0, rise) - smoothstep(t, t1, rise)


def _base(duration: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t = np.arange(0.0, duration, 1.0 / RATE_HZ)
    f = np.zeros((len(t), 3))
    f[:, 2] = G
    return t, f, np.zeros((len(t), 3))


def launch() -> Scenario:
    """Full-throttle launch: 3 m/s^2 for 8 s (SPEC P05 / IT03), then coast."""
    t, f, w = _base(16.0)
    f[:, 0] = 3.0 * pulse(t, 1.0, 9.0, 0.3)
    w[:, 1] = np.gradient(-0.01 * f[:, 0], t)  # body squats nose-up 0.01 rad per m/s^2
    return Scenario("launch", "Launch: 3 m/s² for 8 s", t, f, w,
                    "sustained surge: onset by translation, then held by pitch tilt; washout ≤ 6 s")


def braking() -> Scenario:
    """Hard stop from 20 m/s: -7 m/s^2 for ~2.9 s, then standing."""
    t, f, w = _base(12.0)
    f[:, 0] = -7.0 * pulse(t, 1.0, 3.85, 0.25)
    w[:, 1] = np.gradient(-0.01 * f[:, 0], t)  # nose dive
    return Scenario("braking", "Emergency braking: −7 m/s² from 20 m/s", t, f, w,
                    "strong negative surge, past the soft-limit knee: 7 → 5.6 m/s², 3.4 after the gain")


def lane_change() -> Scenario:
    """Double lane change at 20 m/s: lateral acceleration one full sine, +-3 m/s^2."""
    t, f, w = _base(10.0)
    v = 20.0
    period = 3.0
    m = (t >= 1.0) & (t < 1.0 + period)
    ay = np.zeros_like(t)
    ay[m] = 3.0 * np.sin(2 * math.pi * (t[m] - 1.0) / period)
    f[:, 1] = ay
    w[:, 2] = ay / v                              # yaw rate r = a_y / v
    w[:, 0] = np.gradient(-0.006 * ay, t)         # body rolls out of the turn
    return Scenario("lane_change", "Lane change at 20 m/s: ±3 m/s² lateral", t, f, w,
                    "transient sway and yaw: mostly cued by translation and rotation")


def cornering() -> Scenario:
    """Long left-hand bend at 20 m/s: 4 m/s^2 lateral for 6 s."""
    t, f, w = _base(14.0)
    v = 20.0
    ay = 4.0 * pulse(t, 1.0, 8.0, 1.0)
    f[:, 1] = ay
    w[:, 2] = ay / v
    w[:, 0] = np.gradient(-0.006 * ay, t)
    return Scenario("cornering", "Cornering at 20 m/s: 4 m/s² for 6 s", t, f, w,
                    "sustained sway held by roll tilt, sustained yaw rate washed out")


def bump() -> Scenario:
    """Road bump at 15 m/s: front then rear axle, heave and pitch, damped at ~1.5 Hz."""
    t, f, w = _base(6.0)
    for t0, sgn in ((1.0, 1.0), (1.18, 0.7)):     # front, then rear axle (2.7 m wheelbase)
        tau = np.clip(t - t0, 0, None)
        h = np.where(t >= t0, np.exp(-2.5 * tau) * np.sin(2 * math.pi * 1.5 * tau), 0.0)
        f[:, 2] += 5.0 * sgn * h
        w[:, 1] += (0.25 if sgn > 0.9 else -0.25) * h
    return Scenario("bump", "Road bump at 15 m/s: heave ±5 m/s², pitch", t, f, w,
                    "short transient in heave and pitch")


def excessive() -> Scenario:
    """Deliberately excessive cue (IT05): 3 g on all translational axes plus fast rotations."""
    t, f, w = _base(8.0)
    f[:, 0] = 3 * G * pulse(t, 1.0, 3.0, 0.1)
    f[:, 1] = -3 * G * pulse(t, 2.0, 4.0, 0.1)
    f[:, 2] += 3 * G * pulse(t, 1.5, 2.0, 0.05)
    w[:, 0] = 2.0 * pulse(t, 4.5, 5.0, 0.05)
    w[:, 2] = -3.0 * pulse(t, 5.0, 6.0, 0.05)
    return Scenario("excessive", "Excessive cue: 3 g on x/y/z, 2–3 rad/s rates", t, f, w,
                    "saturation test: soft limit and workspace scaling must keep every leg inside")


ALL = [launch, braking, lane_change, cornering, bump, excessive]


def surge_step() -> Scenario:
    """IT03: a pure 3 m/s^2 surge step for 8 s (0.3 s onset), no body rotation, so the
    platform pitch is tilt coordination alone (P04 is a limit on the tilt rate)."""
    t, f, w = _base(16.0)
    f[:, 0] = 3.0 * pulse(t, 1.0, 9.0, 0.3)
    return Scenario("surge_step", "Surge step: 3 m/s² for 8 s", t, f, w,
                    "the launch without its body pitch: tilt coordination alone")


# Streamed by the Renode tests (sim/data): the six tuning scenarios plus the IT03 step.
EXPORT = ALL + [surge_step]


def upsample(t: np.ndarray, x: np.ndarray, rate_hz: float) -> tuple[np.ndarray, np.ndarray]:
    """Linear interpolation from 100 Hz to the cueing rate (as the controller does, F09)."""
    t2 = np.arange(t[0], t[-1], 1.0 / rate_hz)
    return t2, np.column_stack([np.interp(t2, t, x[:, a]) for a in range(x.shape[1])])


EXPORT_DIR = Path(__file__).resolve().parents[2] / "sim" / "data"


def render_csv(s: Scenario) -> str:
    """t,fx,fy,fz,wx,wy,wz at 100 Hz: what the Renode host stand-in streams."""
    rows = ["t,fx,fy,fz,wx,wy,wz"]
    for k in range(len(s.t)):
        rows.append(",".join([f"{s.t[k]:.2f}"] + [f"{v:.6f}" for v in (*s.f[k], *s.w[k])]))
    return "\n".join(rows) + "\n"


def export(check: bool) -> int:
    stale = []
    for make in EXPORT:
        s = make()
        path = EXPORT_DIR / f"{s.name}.csv"
        text = render_csv(s)
        if check:
            if not path.exists() or path.read_text() != text:
                stale.append(path.name)
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
            print(f"wrote sim/data/{path.name} ({len(s.t)} samples)")
    if check:
        print(f"sim/data out of date: {stale}" if stale else "sim/data scenarios are up to date")
    return 1 if stale else 0


def main() -> int:
    import sys

    if "--export" in sys.argv or "--check" in sys.argv:
        return export("--check" in sys.argv)
    for make in ALL:
        s = make()
        print(f"{s.name:12s} {s.t[-1] + 1 / RATE_HZ:5.1f} s  max|f-g| "
              f"{np.max(np.abs(s.f - [0, 0, G]), axis=0).round(2)} m/s²  "
              f"max|w| {np.max(np.abs(s.w), axis=0).round(3)} rad/s  {s.note}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
