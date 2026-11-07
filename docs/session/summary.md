# Session `session.csv`

Recorded by `tools/host/motion_host.py` over SocketCAN from the firmware in Renode; times are controller (virtual) time.

- Duration: 71.6 s of controller time; 3579 STATUS frames, 7116 vehicle samples sent (spacing on the controller clock: mean 10.00 ms, min 8.6, max 11.4).
- States: OFF (0.0 s) → HOMING (0.0 s) → PARKED (0.4 s) → ENGAGING (0.9 s) → ENGAGED (3.9 s) → ESTOP (61.1 s) → PARKED (70.5 s)
- Faults: ESTOP at 61.1 s
- Scenarios: launch (4.9 s), cornering (20.9 s), lane_change (35.0 s), bump (45.0 s), excessive (51.0 s), braking (59.0 s), E-stop (61.0 s)
- Leg strokes: 0.00 … 239.91 mm (soft limits 10 / 240 mm).
- Engaged 80 % of the time; workspace scale down to 0.405.
- Loop timing (TIMING, virtual): servo worst 133 µs of 1000, cueing worst 591 µs of 2000, overruns 0 / 0.

## Commands

| t (s) | command | key | status | value |
|---|---|---|---|---|
| 0.02 | get | 1 | OK | 500 |
| 0.02 | enable | 0 | OK | 0 |
| 0.91 | engage | 0 | OK | 0 |
| 61.30 | reset | 0 | ESTOP | 0 |
| 70.53 | reset | 0 | OK | 0 |

## Cue (whole session)

| axis | asked RMS (m/s²) | error RMS (m/s²) |
|---|---|---|
| x | 0.904 | 0.798 |
| y | 0.823 | 0.698 |
| z | 0.281 | 0.325 |

Delivered = R^T (a + g) at the platform centre, from the reported pose; see `tools/host/session_plots.py`.
