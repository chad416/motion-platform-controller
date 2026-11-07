#!/usr/bin/env python3
"""Host for the motion platform controller over SocketCAN.

- streams the vehicle state (VEHICLE_F / VEHICLE_W, 100 Hz) from a scenario CSV
  (t,fx,fy,fz,wx,wy,wz, as written by tools/reference/scenarios.py --export)
- sends commands (ENABLE, ENGAGE, PARK, RESET_FAULT, SET_PARAM, GET_PARAM) and
  waits for their CMD_ACK
- logs every frame, both directions, decoded with protocol/motion.dbc, to a CSV

Pacing. Against real hardware the 100 Hz stream runs on the host's wall clock
(--clock wall). The emulated controller in Renode runs slower than real time
(about 0.6x), so by default the stream is paced on the controller's own clock
(--clock ctl): its 50 Hz STATUS frames carry a sequence number, i.e. 20 ms of
controller time each. The controller then sees 100 samples per second of its
time, as on hardware, and a pause in the emulation pauses the stream instead of
tripping the 200 ms heartbeat.

Usage:
  motion_host.py cmd enable|engage|park|reset          [--channel vcan0]
  motion_host.py get force_gain
  motion_host.py set force_gain 0.4
  motion_host.py play launch cornering --log session.csv   (ENABLE, ENGAGE, play, PARK)
  motion_host.py log --log session.csv --duration 30
"""
from __future__ import annotations

import argparse
import csv
import json
import pathlib
import signal
import threading
import time

import can

from protocol import (FAULTS, ID_CMD_ACK, ID_FAULT, ID_LEG_CUR_A, ID_LEG_CUR_B, ID_LEG_POS_A,
                      ID_LEG_POS_B, ID_POSE_R, ID_POSE_T, ID_STATUS, ID_TIMING, OPCODES, PARAMS,
                      STATES, STATUS, Protocol)

ROOT = pathlib.Path(__file__).resolve().parents[2]
SCENARIOS = ROOT / "sim" / "data"
G = 9.80665
REST = (0.0, 0.0, G, 0.0, 0.0, 0.0)
STATUS_PERIOD_MS = 20.0
SAMPLE_PERIOD_MS = 10.0


def load_scenario(name_or_path: str) -> list[tuple[float, ...]]:
    p = pathlib.Path(name_or_path)
    if not p.exists():
        p = SCENARIOS / f"{name_or_path}.csv"
    with p.open(newline="") as f:
        rows = list(csv.reader(f))
    return [tuple(float(x) for x in r[1:7]) for r in rows[1:] if r]


class ControllerClock:
    """Controller time in ms from the STATUS sequence number, interpolated
    between frames with the measured controller/wall rate."""

    def __init__(self):
        self.lock = threading.Lock()
        self.seq = None
        self.ticks = 0
        self.ms = None
        self.wall = None
        self.rate = 1.0
        self._w0 = None
        self._m0 = None
        self._out = None

    def on_status(self, seq: int) -> None:
        now = time.monotonic()
        with self.lock:
            if self.seq is not None:
                self.ticks += (seq - self.seq) & 0xFFFF
            self.seq = seq
            self.ms = self.ticks * STATUS_PERIOD_MS
            self.wall = now
            if self._w0 is None:
                self._w0, self._m0 = now, self.ms
            elif now - self._w0 >= 1.0:  # rate over the last second of wall time
                r = (self.ms - self._m0) / 1000.0 / (now - self._w0)
                self.rate = 0.7 * self.rate + 0.3 * r
                self._w0, self._m0 = now, self.ms

    def now(self) -> float | None:
        with self.lock:
            if self.ms is None:
                return None
            ahead = (time.monotonic() - self.wall) * 1000.0 * self.rate
            t = self.ms + min(ahead, 2 * STATUS_PERIOD_MS)  # never run far ahead of a stall
            # a STATUS that arrives "early" corrects the estimate: hold, never step back
            self._out = t if self._out is None else max(self._out, t)
            return self._out


class WallClock:
    def __init__(self):
        self.t0 = time.monotonic()

    def now(self) -> float:
        return (time.monotonic() - self.t0) * 1000.0


class PlatformState:
    """Latest controller state, built from decoded frames (live or replayed)."""

    def __init__(self):
        self.state = "?"
        self.flags: dict = {}
        self.fault = "NONE"
        self.fault_leg = 0
        self.pose = [0.0] * 6  # x, y, z (m), roll, pitch, yaw (deg)
        self.strokes = [0.0] * 6  # m above the lower end stop
        self.currents = [0.0] * 6  # A
        self.timing: dict = {}
        self.vehicle = list(REST)
        self.faults: list[str] = []
        self._pose_t = None
        self._pos_a = None
        self._cur_a = None

    def apply(self, frame_id: int, name: str, s: dict) -> None:
        if frame_id == ID_STATUS:
            self.state = STATES[s["state"]] if s["state"] < len(STATES) else str(s["state"])
            self.flags = {k: bool(s[k]) for k in ("saturated", "fk_ok", "estop", "comms_lost", "descended")}
            self.flags["ws_scale"] = s["ws_scale"]
            self.fault = FAULTS[s["fault"]] if s["fault"] < len(FAULTS) else str(s["fault"])
            self.fault_leg = s["fault_leg"]
        elif frame_id == ID_POSE_T:
            self._pose_t = (s["seq"], [s["x"], s["y"], s["z"]])
        elif frame_id == ID_POSE_R and self._pose_t and self._pose_t[0] == s["seq"]:
            self.pose = self._pose_t[1] + [s["roll"], s["pitch"], s["yaw"]]
        elif frame_id == ID_LEG_POS_A:
            self._pos_a = (s["seq"], [s["stroke_1"], s["stroke_2"], s["stroke_3"]])
        elif frame_id == ID_LEG_POS_B and self._pos_a and self._pos_a[0] == s["seq"]:
            self.strokes = self._pos_a[1] + [s["stroke_4"], s["stroke_5"], s["stroke_6"]]
        elif frame_id == ID_LEG_CUR_A:
            self._cur_a = (s["seq"], [s["current_1"], s["current_2"], s["current_3"]])
        elif frame_id == ID_LEG_CUR_B and self._cur_a and self._cur_a[0] == s["seq"]:
            self.currents = self._cur_a[1] + [s["current_4"], s["current_5"], s["current_6"]]
        elif frame_id == ID_TIMING:
            self.timing = dict(s)
        elif frame_id == ID_FAULT:
            code = FAULTS[s["code"]] if s["code"] < len(FAULTS) else str(s["code"])
            self.faults.append(f"{code} leg={s['leg']} value={s['value']}")
        elif name == "VEHICLE_F":
            self.vehicle[0:3] = [s["f_x"], s["f_y"], s["f_z"]]
        elif name == "VEHICLE_W":
            self.vehicle[3:6] = [s["w_x"], s["w_y"], s["w_z"]]


class MotionHost:
    def __init__(self, bus: can.BusABC, log_path: pathlib.Path | None = None, clock: str = "ctl"):
        self.bus = bus
        self.proto = Protocol()
        self.ctl_clock = ControllerClock()
        self.clock = self.ctl_clock if clock == "ctl" else WallClock()
        self.ps = PlatformState()
        self.lock = threading.Lock()
        self.acks: list[tuple[int, int, int, int]] = []
        self.ack_event = threading.Condition(self.lock)
        self.t0 = time.monotonic()
        self.log_file = log_path.open("w", newline="") if log_path else None
        self.log = csv.writer(self.log_file) if self.log_file else None
        if self.log:
            self.log.writerow(["host_s", "ctl_ms", "dir", "id", "name", "signals"])
        self.seq = 0
        self.samples: list[tuple[float, ...]] = []
        self.k = 0
        self.streaming = False
        self.sent_samples = 0
        self._stop = threading.Event()
        self._stream_thread: threading.Thread | None = None
        self.notifier = can.Notifier(bus, [self._on_msg])

    # ---- frames -------------------------------------------------------------
    def _ctl_ms(self) -> float:
        t = self.ctl_clock.now()
        return -1.0 if t is None else t

    def _record(self, direction: str, frame_id: int, name: str, sig: dict) -> None:
        if self.log:
            self.log.writerow([f"{time.monotonic() - self.t0:.4f}", f"{self._ctl_ms():.1f}", direction,
                               f"0x{frame_id:03X}", name,
                               json.dumps({k: (round(v, 6) if isinstance(v, float) else v)
                                           for k, v in sig.items()})])

    def _on_msg(self, msg: can.Message) -> None:
        d = self.proto.decode(msg.arbitration_id, bytes(msg.data))
        if d is None:
            return
        with self.lock:
            if d.frame_id == ID_STATUS:
                self.ctl_clock.on_status(d.signals["seq"])
            self.ps.apply(d.frame_id, d.name, d.signals)
            self._record("rx", d.frame_id, d.name, d.signals)
            if d.frame_id == ID_CMD_ACK:
                s = d.signals
                self.acks.append((s["opcode"], s["key"], s["status"], s["value"]))
                self.ack_event.notify_all()

    def _send(self, frame_id: int, data: bytes) -> None:
        self.bus.send(can.Message(arbitration_id=frame_id, data=data, is_extended_id=False))
        d = self.proto.decode(frame_id, data)
        with self.lock:
            self.ps.apply(frame_id, d.name, d.signals)
            self._record("tx", frame_id, d.name, d.signals)

    def mark(self, text: str) -> None:
        """A note in the log (scenario names etc.), on the controller clock."""
        with self.lock:
            self._record("note", 0, "NOTE", {"text": text})

    # ---- commands -------------------------------------------------------------
    def command(self, op: str, key: int = 0, value: int = 0, timeout: float = 5.0) -> tuple[str, int]:
        """Send a CMD and wait for its CMD_ACK: (status name, value)."""
        opcode = OPCODES[op]
        with self.lock:
            n = len(self.acks)
        self._send(*self.proto.cmd(opcode, key, value))
        deadline = time.monotonic() + timeout
        with self.lock:
            while True:
                for a in self.acks[n:]:
                    if a[0] == opcode and a[1] == key:
                        return STATUS[a[2]] if a[2] < len(STATUS) else str(a[2]), a[3]
                left = deadline - time.monotonic()
                if left <= 0:
                    raise TimeoutError(f"no CMD_ACK for {op}")
                self.ack_event.wait(left)

    def set_param(self, name: str, value: float) -> str:
        key, scale = PARAMS[name]
        return self.command("set", key, round(value / scale))[0]

    def get_param(self, name: str) -> float:
        key, scale = PARAMS[name]
        st, v = self.command("get", key)
        if st != "OK":
            raise RuntimeError(f"GET {name}: {st}")
        return v * scale

    def wait_alive(self, timeout: float = 10.0) -> None:
        """Wait for the controller's first STATUS frame (its clock)."""
        deadline = time.monotonic() + timeout
        while self.ctl_clock.now() is None:
            if time.monotonic() > deadline:
                raise TimeoutError("no STATUS from the controller")
            time.sleep(0.01)

    def wait_state(self, state: str, timeout: float = 15.0) -> None:
        deadline = time.monotonic() + timeout
        while self.ps.state != state:
            if time.monotonic() > deadline:
                raise TimeoutError(f"state {self.ps.state}, waited for {state}")
            time.sleep(0.01)

    # ---- vehicle stream ---------------------------------------------------------
    def start_stream(self) -> None:
        if self._stream_thread is None:
            self.streaming = True
            self._stream_thread = threading.Thread(target=self._stream, daemon=True)
            self._stream_thread.start()

    def stop_stream(self) -> None:
        self.streaming = False

    def play(self, samples: list[tuple[float, ...]], wait: bool = True) -> None:
        with self.lock:
            self.samples = list(samples)
            self.k = 0
        while wait and self.k < len(self.samples):
            time.sleep(0.02)

    def _stream(self) -> None:
        due = None
        while not self._stop.is_set():
            now = self.clock.now()
            if now is None:
                time.sleep(0.005)
                continue
            if due is None:
                due = now
            while self.streaming and due <= now:
                with self.lock:
                    if self.k < len(self.samples):
                        s = self.samples[self.k]
                        self.k += 1
                    else:
                        s = REST
                for fid, data in self.proto.vehicle(self.seq, s[0:3], s[3:6]):
                    self._send(fid, data)
                self.seq = (self.seq + 1) & 0xFFFF
                self.sent_samples += 1
                due += SAMPLE_PERIOD_MS
            if not self.streaming:
                due = None
            time.sleep(0.001)

    def close(self) -> None:
        self._stop.set()
        if self._stream_thread:
            self._stream_thread.join(timeout=1)
        self.notifier.stop()
        if self.log_file:
            self.log_file.close()


def _raise_interrupt(signum, frame) -> None:
    raise KeyboardInterrupt


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("action", choices=["cmd", "get", "set", "play", "log"])
    p.add_argument("args", nargs="*")
    p.add_argument("--channel", default="vcan0")
    p.add_argument("--interface", default="socketcan")
    p.add_argument("--log", type=pathlib.Path, default=None, help="CSV of every frame")
    p.add_argument("--clock", choices=["ctl", "wall"], default="ctl")
    p.add_argument("--duration", type=float, default=0.0, help="log: seconds, 0 = until Ctrl-C")
    a = p.parse_args()
    signal.signal(signal.SIGTERM, _raise_interrupt)

    bus = can.Bus(interface=a.interface, channel=a.channel)
    host = MotionHost(bus, a.log, a.clock)
    try:
        if a.action != "log":
            host.wait_alive()
        if a.action == "cmd":
            print(host.command(a.args[0])[0])
        elif a.action == "get":
            print(f"{a.args[0]} = {host.get_param(a.args[0]):g}")
        elif a.action == "set":
            print(host.set_param(a.args[0], float(a.args[1])))
        elif a.action == "play":
            host.start_stream()
            if host.ps.state != "PARKED":
                print("ENABLE", host.command("enable")[0])
                host.wait_state("PARKED")
            print("ENGAGE", host.command("engage")[0])
            host.wait_state("ENGAGED")
            for name in a.args:
                print("play", name)
                host.play(load_scenario(name))
            print("PARK", host.command("park")[0])
            host.wait_state("PARKED")
        elif a.action == "log":
            t_end = time.monotonic() + a.duration if a.duration > 0 else None
            while t_end is None or time.monotonic() < t_end:
                time.sleep(0.1)
        print("faults:", host.ps.faults or "none")
    except KeyboardInterrupt:
        pass
    finally:
        host.close()
        bus.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
