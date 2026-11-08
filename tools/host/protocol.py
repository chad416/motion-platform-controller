"""CAN protocol for the host tools, decoded and encoded straight from
protocol/motion.dbc with cantools (the same source the firmware's proto unit
tests are checked against). Names and codes follow mcue/ctl.h.
"""
from __future__ import annotations

import pathlib
from dataclasses import dataclass

import cantools

ROOT = pathlib.Path(__file__).resolve().parents[2]
DBC = ROOT / "protocol" / "motion.dbc"

STATES = ["OFF", "HOMING", "PARKED", "ENGAGING", "ENGAGED", "PARKING", "FAULT", "ESTOP"]
FAULTS = ["NONE", "FOLLOWING", "HARD_LIMIT", "CURRENT", "OVERRUN", "HOMING", "ESTOP", "COMMS_LOST",
          "INTERNAL"]
OPCODES = {"enable": 1, "engage": 2, "park": 3, "reset": 4, "set": 5, "get": 6}
OPNAMES = {v: k for k, v in OPCODES.items()}
STATUS = ["OK", "BAD_STATE", "ESTOP", "BAD_KEY", "BAD_VALUE"]
# SET_PARAM / GET_PARAM keys and their integer scale (mc_param_key)
PARAMS = {
    "force_gain": (1, 1e-3),
    "tilt_rate_limit": (2, 1e-3),  # deg/s
    "rot_gain": (3, 1e-3),
    "surge_sway_wn": (4, 1e-3),
    "surge_sway_wb": (5, 1e-3),
    "heave_wn": (6, 1e-3),
    "tilt_wn": (7, 1e-3),
}

ID_VEHICLE_F, ID_VEHICLE_W, ID_CMD = 0x100, 0x101, 0x10F
ID_FAULT, ID_STATUS, ID_POSE_T, ID_POSE_R = 0x180, 0x181, 0x182, 0x183
ID_LEG_POS_A, ID_LEG_POS_B, ID_LEG_CUR_A, ID_LEG_CUR_B = 0x184, 0x185, 0x186, 0x187
ID_TIMING, ID_CMD_ACK = 0x188, 0x18F


@dataclass
class Decoded:
    frame_id: int
    name: str
    signals: dict


class Protocol:
    def __init__(self, dbc: pathlib.Path = DBC):
        self.db = cantools.database.load_file(str(dbc))
        self._by_id = {m.frame_id: m for m in self.db.messages}

    def decode(self, frame_id: int, data: bytes) -> Decoded | None:
        m = self._by_id.get(frame_id)
        if m is None or len(data) < m.length:
            return None
        sig = m.decode(bytes(data[:m.length]), decode_choices=False, scaling=True)
        return Decoded(frame_id, m.name, dict(sig))

    def encode(self, name: str, **signals) -> tuple[int, bytes]:
        m = self.db.get_message_by_name(name)
        return m.frame_id, m.encode(signals, strict=True)

    def vehicle(self, seq: int, f, w) -> list[tuple[int, bytes]]:
        """VEHICLE_F and VEHICLE_W for one 100 Hz sample (values saturate like the firmware)."""
        def sat(v, lo, hi):
            return min(hi, max(lo, float(v)))
        seq &= 0xFFFF
        return [
            self.encode("VEHICLE_F", f_x=sat(f[0], -65.536, 65.534), f_y=sat(f[1], -65.536, 65.534),
                        f_z=sat(f[2], -65.536, 65.534), seq=seq),
            self.encode("VEHICLE_W", w_x=sat(w[0], -3.2768, 3.2767), w_y=sat(w[1], -3.2768, 3.2767),
                        w_z=sat(w[2], -3.2768, 3.2767), seq=seq),
        ]

    def cmd(self, opcode: int, key: int = 0, value: int = 0) -> tuple[int, bytes]:
        return self.encode("CMD", isotp_pci=6, opcode=opcode, key=key, value=int(value))
