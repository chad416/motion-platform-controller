"""Robot Framework keywords that analyse the controller's CAN output.

sim/host_feed.py records every frame the controller sends, with its virtual
time, and dumps them as CSV (t_ms,id,data). These keywords decode that log
with the layouts of protocol/motion.dbc and check it against SPEC §4.2.
"""
from __future__ import annotations

import csv

ID_FAULT, ID_STATUS, ID_POSE_T, ID_POSE_R = 0x180, 0x181, 0x182, 0x183
ID_LEG_POS_A, ID_LEG_POS_B = 0x184, 0x185
ID_TIMING, ID_CMD_ACK = 0x188, 0x18F

STATES = ["OFF", "HOMING", "PARKED", "ENGAGING", "ENGAGED", "PARKING", "FAULT", "ESTOP"]


def _s16(b, i):
    v = b[i] | (b[i + 1] << 8)
    return v - 0x10000 if v >= 0x8000 else v


def _u16(b, i):
    return b[i] | (b[i + 1] << 8)


def _s32(b, i):
    v = b[i] | (b[i + 1] << 8) | (b[i + 2] << 16) | (b[i + 3] << 24)
    return v - 0x100000000 if v >= 0x80000000 else v


def _frames(path):
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            yield float(row["t_ms"]), int(row["id"], 16), [int(x, 16) for x in row["data"].split()]


def _poses(path):
    """(t_ms, x, y, z m, roll, pitch, yaw deg), pairing POSE_T and POSE_R by seq."""
    pending = {}
    out = []
    for t, fid, b in _frames(path):
        if fid == ID_POSE_T:
            pending[_u16(b, 6)] = (t, [_s16(b, 0) * 1e-4, _s16(b, 2) * 1e-4, _s16(b, 4) * 1e-4])
        elif fid == ID_POSE_R and _u16(b, 6) in pending:
            t0, xyz = pending.pop(_u16(b, 6))
            out.append((t0, *xyz, _s16(b, 0) * 0.01, _s16(b, 2) * 0.01, _s16(b, 4) * 0.01))
    return out


def _strokes(path):
    """(t_ms, [stroke m, legs 1..6]) pairing LEG_POS_A and _B by seq."""
    pending = {}
    out = []
    for t, fid, b in _frames(path):
        if fid == ID_LEG_POS_A:
            pending[_u16(b, 6)] = (t, [_u16(b, i) * 1e-5 for i in (0, 2, 4)])
        elif fid == ID_LEG_POS_B and _u16(b, 6) in pending:
            t0, a = pending.pop(_u16(b, 6))
            out.append((t0, a + [_u16(b, i) * 1e-5 for i in (0, 2, 4)]))
    return out


def _status(path):
    out = []
    for t, fid, b in _frames(path):
        if fid == ID_STATUS:
            out.append({"t": t, "state": STATES[b[0]] if b[0] < len(STATES) else str(b[0]),
                        "saturated": bool(b[1] & 1), "fk_ok": bool(b[1] & 2),
                        "estop": bool(b[1] & 4), "comms_lost": bool(b[1] & 8),
                        "descended": bool(b[1] & 16), "fault": b[2], "fault_leg": b[3],
                        "scale": b[4] * 0.005, "event": b[5]})
    return out


# ---- keywords -----------------------------------------------------------------

def frame_count(path, frame_id):
    fid = int(frame_id, 0) if isinstance(frame_id, str) else int(frame_id)
    return sum(1 for _t, i, _b in _frames(path) if i == fid)


def state_at(path, t_ms):
    """State in the last STATUS frame at or before t_ms."""
    st = None
    for s in _status(path):
        if s["t"] <= float(t_ms):
            st = s["state"]
    return st


def first_state_time(path, state, after_ms=0):
    """Virtual time (ms) of the first STATUS frame in `state` after `after_ms`, or -1."""
    for s in _status(path):
        if s["t"] >= float(after_ms) and s["state"] == state:
            return s["t"]
    return -1.0


def status_flag_seen(path, flag, after_ms=0):
    return any(s[flag] for s in _status(path) if s["t"] >= float(after_ms))


def last_status(path):
    return _status(path)[-1]


def min_ws_scale(path):
    return min(s["scale"] for s in _status(path))


def fault_frames(path):
    """FAULT frames as 't_ms code leg state value' strings."""
    return [f"{t:.1f} {b[0]} {b[1]} {b[2]} {_s32(b, 4)}"
            for t, fid, b in _frames(path) if fid == ID_FAULT]


def pose_after(path, t_ms):
    """First reported pose at or after t_ms: [x, y, z (mm), roll, pitch, yaw (deg)]."""
    for p in _poses(path):
        if p[0] >= float(t_ms):
            return [p[1] * 1e3, p[2] * 1e3, p[3] * 1e3, p[4], p[5], p[6]]
    raise AssertionError(f"no pose after {t_ms} ms")


def pose_extremes(path, t0_ms, t1_ms):
    """Max |x|,|y|,|z| (mm) and |roll|,|pitch|,|yaw| (deg) between t0 and t1."""
    ps = [p for p in _poses(path) if float(t0_ms) <= p[0] <= float(t1_ms)]
    return [max(abs(p[k]) * (1e3 if k <= 3 else 1.0) for p in ps) for k in range(1, 7)]


def max_tilt_rate(path, t0_ms, t1_ms, window_ms=100):
    """Max roll/pitch rate (deg/s) of the FK pose over `window_ms` windows.

    POSE_R has 0.01 deg resolution at 50 Hz, so the rate over 100 ms is good to
    +-0.1 deg/s."""
    ps = [p for p in _poses(path) if float(t0_ms) <= p[0] <= float(t1_ms)]
    best = 0.0
    j = 0
    for i in range(len(ps)):
        while j < len(ps) and ps[j][0] - ps[i][0] < float(window_ms) - 1e-6:
            j += 1
        if j >= len(ps):
            break
        dt = (ps[j][0] - ps[i][0]) / 1e3
        best = max(best, abs(ps[j][4] - ps[i][4]) / dt, abs(ps[j][5] - ps[i][5]) / dt)
    return best


def stroke_range(path, t0_ms=0, t1_ms=1e12):
    """[min, max] leg stroke (mm above the lower end stop) over all legs."""
    ss = [s for t, s in _strokes(path) if float(t0_ms) <= t <= float(t1_ms)]
    if not ss:
        raise AssertionError("no LEG_POS frames in range")
    return [min(min(s) for s in ss) * 1e3, max(max(s) for s in ss) * 1e3]


def strokes_after(path, t_ms):
    for t, s in _strokes(path):
        if t >= float(t_ms):
            return [v * 1e3 for v in s]
    raise AssertionError(f"no LEG_POS after {t_ms} ms")


def _windows(ss, window_ms):
    """Pairs of samples `window_ms` apart.

    A LEG_POS sample is the controller's last 500 Hz cue step when the 20 ms
    report ran, so it can be 0 or 2 ms old, and the frame time (send time)
    jitters by 1-2 ms as well. Over a 20 ms step that is a 10 % speed error,
    over 100 ms 2 %, over 500 ms 0.4 %."""
    j = 0
    for i in range(len(ss)):
        while j < len(ss) and ss[j][0] - ss[i][0] < float(window_ms) - 1e-6:
            j += 1
        if j >= len(ss):
            return
        yield ss[i], ss[j]


def max_leg_speed(path, t0_ms, t1_ms, legs="123456", window_ms=100):
    """Max leg speed (m/s) from LEG_POS frames `window_ms` apart. Over the
    100 ms default the sampling error (see _windows) is up to about 4 %; the
    controller commands 0.045 m/s for homing and the descent, 10 % below the
    0.05 m/s limit, so the checks need no tolerance on top of the limit."""
    ss = [(t, s) for t, s in _strokes(path) if float(t0_ms) <= t <= float(t1_ms)]
    idx = [int(c) - 1 for c in str(legs)]
    best = 0.0
    for (ta, a), (tb, b) in _windows(ss, window_ms):
        dt = (tb - ta) / 1e3
        best = max(best, max(abs(b[i] - a[i]) / dt for i in idx))
    return best


def standstill_time(path, t0_ms, speed=0.002, window_ms=100):
    """Start (ms) of the first `window_ms` window after t0 over which every leg
    moves slower than `speed` m/s on average, or -1."""
    ss = [(t, s) for t, s in _strokes(path) if t >= float(t0_ms)]
    for (ta, a), (tb, b) in _windows(ss, window_ms):
        dt = (tb - ta) / 1e3
        if max(abs(b[i] - a[i]) / dt for i in range(6)) < float(speed):
            return ta
    return -1.0


def acks(path):
    """CMD_ACK frames as 'opcode key status value' strings, in order."""
    return [f"{b[1]} {b[2]} {b[3]} {_s32(b, 4)}"
            for _t, fid, b in _frames(path) if fid == ID_CMD_ACK]


def seq_gaps(path, frame_id):
    """Frames of `frame_id` lost between the first and the last one, from the
    16-bit seq in bytes 6-7 (STATUS, POSE_*, LEG_*)."""
    fid = int(frame_id, 0) if isinstance(frame_id, str) else int(frame_id)
    seqs = [_u16(b, 6) for _t, i, b in _frames(path) if i == fid]
    return sum(((b - a) & 0xFFFF) - 1 for a, b in zip(seqs, seqs[1:]))


def timing_frames(path):
    return [[_u16(b, 0), _u16(b, 2), _u16(b, 4), _u16(b, 6)]
            for _t, fid, b in _frames(path) if fid == ID_TIMING]
