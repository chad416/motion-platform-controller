# Deterministic host stand-in for the Renode tests (IronPython 2.7, runs inside
# the Renode monitor). It replaces the Python host on the CAN bus:
#
#   - streams VEHICLE_F / VEHICLE_W at 100 Hz on *virtual* time (a scenario CSV,
#     or a constant sample), straight into the controller's FDCAN1
#   - records every frame the controller sends, with its virtual time
#
# Robot keywords drive it with `python "mh...."` monitor commands (common.resource).
# Driving the frames from inside the emulation keeps the tests repeatable: a
# host on the far side of a socket would see wall-clock jitter instead.

import clr

clr.AddReference("System")
from System import Action, Array, Byte  # noqa: E402

from Antmicro.Renode.Core.CAN import CANMessageFrame  # noqa: E402
from Antmicro.Renode.Time import TimeInterval  # noqa: E402

ID_VEHICLE_F = 0x100
ID_VEHICLE_W = 0x101
G = 9.80665


def _s16(v, scale):
    r = int(round(v / scale))
    r = max(-32768, min(32767, r))
    return r & 0xFFFF


def _u16_bytes(v):
    return [v & 0xFF, (v >> 8) & 0xFF]


class MotionHost(object):
    def __init__(self, machine):
        self.m = machine
        self.fd = machine["sysbus.fdcan1"]
        self.samples = []
        self.k = 0
        self.seq = 0
        self.streaming = False
        self.hold = (0.0, 0.0, G, 0.0, 0.0, 0.0)
        self.sent = 0
        self.log = []
        self._tick_action = Action[TimeInterval](self._tick)
        self.fd.FrameSent += self._on_sent

    # ---- time -------------------------------------------------------------
    def now_ms(self):
        return self.m.ElapsedVirtualTime.TimeElapsed.TotalMilliseconds

    # ---- controller output --------------------------------------------------
    def _on_sent(self, frame):
        self.log.append((self.now_ms(), int(frame.Id), list(frame.Data)))

    def clear_log(self):
        self.log = []

    def dump(self, path):
        """Write the recorded controller frames: t_ms,id,b0..b7 (hex id)."""
        f = open(path, "w")
        f.write("t_ms,id,data\n")
        for t, fid, data in self.log:
            f.write("%.3f,0x%03X,%s\n" % (t, fid, " ".join("%02X" % b for b in data)))
        f.close()
        return len(self.log)

    # ---- vehicle input ------------------------------------------------------
    def load(self, path):
        """Scenario CSV at 100 Hz: t,fx,fy,fz,wx,wy,wz (header line first)."""
        rows = []
        for line in open(path).read().splitlines()[1:]:
            if line.strip():
                rows.append(tuple(float(x) for x in line.split(",")[1:7]))
        self.samples = rows
        self.k = 0
        return len(rows)

    def const(self, fx, fy, fz, wx=0.0, wy=0.0, wz=0.0):
        """Stream a constant sample (after the scenario, if one is loaded)."""
        self.hold = (fx, fy, fz, wx, wy, wz)

    def start(self):
        if not self.streaming:
            self.streaming = True
            self.m.ScheduleAction(TimeInterval.FromMilliseconds(10), self._tick_action)

    def stop(self):
        self.streaming = False

    def done(self):
        return self.k >= len(self.samples)

    # ---- commands -------------------------------------------------------------
    def cmd(self, opcode, key=0, value=0):
        """CMD as an ISO-TP single frame; the CMD_ACK lands in the log."""
        v = int(value) & 0xFFFFFFFF
        data = [0x06, int(opcode), int(key)] + [(v >> (8 * i)) & 0xFF for i in range(4)]
        self.fd.OnFrameReceived(CANMessageFrame(0x10F, Array[Byte]([Byte(b) for b in data])))

    def acks(self):
        """All CMD_ACKs so far as 'opcode,key,status,value' strings."""
        out = []
        for _t, fid, d in self.log:
            if fid == 0x18F and len(d) == 8:
                v = d[4] | (d[5] << 8) | (d[6] << 16) | (d[7] << 24)
                if v >= 0x80000000:
                    v -= 0x100000000
                out.append("%d,%d,%d,%d" % (d[1], d[2], d[3], v))
        return out

    def _send(self, fid, vals, scale):
        data = []
        for v in vals:
            data += _u16_bytes(_s16(v, scale))
        data += _u16_bytes(self.seq)
        self.fd.OnFrameReceived(CANMessageFrame(fid, Array[Byte]([Byte(b) for b in data])))

    def _tick(self, _interval):
        if not self.streaming:
            return
        if self.k < len(self.samples):
            s = self.samples[self.k]
            self.k += 1
        else:
            s = self.hold
        self._send(ID_VEHICLE_F, s[0:3], 0.002)
        self._send(ID_VEHICLE_W, s[3:6], 0.0001)
        self.seq = (self.seq + 1) & 0xFFFF
        self.sent += 1
        self.m.ScheduleAction(TimeInterval.FromMilliseconds(10), self._tick_action)
