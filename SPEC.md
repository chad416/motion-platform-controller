# S2 — 6-DOF Motion Platform Controller

Specification, revision A (6 Nov 2026)

Embedded controller for a 6-DOF electric motion platform (a Stewart platform) of the kind used under vehicle and crew simulators. It does the following:
- turns vehicle motion into platform motion (classical washout motion cueing),
- solves the platform kinematics,
- keeps every actuator inside its limits,
- runs six servo loops,
- handles the safety states (engage, park, E-stop, communication loss).

It runs as Zephyr firmware on the same emulated STM32H753 board as S1, inside Renode. A Python host sends vehicle motion over CAN and animates the platform in 3D.

> Keep company names out of this repository, the README and the commit messages. It is a public portfolio project.

---

## 1. Why this project

| Junior electronics & firmware profile | Where S2 shows it |
|---|---|
| C/C++ firmware for embedded systems | Fixed-rate control loops (1 kHz servo, 500 Hz cueing) in C on Zephyr, with a measured cycle budget |
| Motors, sensors (the "big plus") | Six servo axes: cascaded position/velocity control, current limit, encoder feedback, actuator plant model |
| CAN communication | DBC-defined protocol: vehicle state in; platform status and actuator feedback out |
| Integration with mechanics and software | Platform geometry, kinematics and workspace; host simulator interface; 3D visualisation |
| Reliable electronics for real use | Safety state machine: engage/park ramps, E-stop, heartbeat loss, following-error and limit faults |

It also builds directly on the user's robot-kinematics background (industrial robot programming): inverse and forward kinematics, workspace limits and motion profiles are daily robotics work.

S2 is the counterpart of S1 for vehicle and crew simulators: S1 is the trainee's weapon, S2 is the motion base under the cabin. Together they cover both simulator families that 3- and 6-axis training-simulator builders make.

---

## 2. Scope

### In scope
- **Portable C library `mcue`**, unit-tested on `native_sim`:
  - inverse kinematics (IK),
  - forward kinematics (FK),
  - workspace limiting,
  - classical washout,
  - the actuator plant model.
- **Zephyr application `motion_fw`** for `stm32h7_renode_reference_board`:
  - cueing loop (500 Hz) and servo loops (1 kHz),
  - safety state machine,
  - CAN protocol,
  - shell diagnostics.
- **Simulated actuators:** a plant model behind a hardware abstraction layer (HAL). On real hardware the HAL becomes the drive interface; the control code does not change.
- **Python host:**
  - vehicle scenarios (launch, braking, lane change, cornering, bump),
  - CAN sender,
  - live 3D platform view,
  - session plots.
- **Tests:** unit tests, a Python reference cross-check, Renode Robot integration tests and CI, reusing the S1 infrastructure.

### Out of scope (v2)
- Real drives (CANopen CiA 402 or EtherCAT)
- Hydraulic actuators
- High-frequency vibration channel (road texture, engine)
- Vestibular perception model and motion-cueing tuning against human data
- Model-predictive cueing

---

## 3. Platform geometry (reference design)

A common semi-regular hexagon layout for a cabin of about 500 kg. All values sit in `config/platform.json` and are used by both the firmware and the Python reference.

| Parameter | Value |
|---|---|
| Base joint radius `r_b` | 0.80 m |
| Platform joint radius `r_p` | 0.50 m |
| Joint pair half-angle (base / platform) | 12° / 30° |
| Neutral height (base plane to platform plane) | 0.70 m |
| Actuator length at neutral `l_0` | 0.787 m (computed; legs 27° from vertical) |
| Actuator stroke | 0.25 m, i.e. `l_0 ± 0.125 m` (0.662–0.912 m) |
| Soft limit margin | 10 mm each end |
| Leg speed limit | 0.5 m/s |
| Leg acceleration limit | 6 m/s² |
| Payload (cabin + crew) | 500 kg, centre of mass 0.3 m above the platform plane |
| Actuator | electric, ball screw, 10 mm/rev lead, encoder 4096 counts/rev (0.0024 mm/count) |

Joint layout (k = 0, 1, 2):
- Base joints sit at 120°·k ± 12°.
- Platform joints sit at 120°·k ± (60° − h_p), where h_p is the platform half-angle. With h_p = 30° they are evenly spaced (a regular hexagon).
- Leg 2k joins base joint 120°·k − 12° to platform joint 120°·k − 30°; leg 2k+1 is its mirror image.

**Choice of the platform half-angle** (week 1, `tools/reference/geometry_check.py --compare`). Smaller angles make the legs more inclined: they stiffen the platform but cost translational travel. Larger angles do the opposite.

| h_p | Surge | Sway | Yaw | Jacobian condition number (neutral / worst) |
|---|---|---|---|---|
| 10° | +0.204 / −0.191 m | ±0.178 m | ±20° | 3.0 / 4.1 |
| 30° | +0.258 / −0.213 m | ±0.215 m | ±27° | 5.7 / 8.9 |
| 40° | +0.303 / −0.223 m | ±0.241 m | ±33° | 12.6 / 27.4 |

30° keeps the kinematics well conditioned and gives the most surge for that, which matters most for cueing. Leg force at neutral with the 500 kg payload is 919 N per leg (= m·g / 6 / cos 27.2°, which checks the statics).

**Single-axis workspace** inside the soft limits, computed for revision A with `tools/reference/geometry_check.py` (a NumPy check, to be added in week 1):

| Axis | Range |
|---|---|
| Surge | +0.258 / −0.213 m |
| Sway | ±0.215 m |
| Heave | +0.127 / −0.132 m |
| Roll | ±14.1° |
| Pitch | +16.7 / −16.1° |
| Yaw | ±26.9° |

Combined poses reach less. That is why F03 scales whole poses.

**Consequence for tilt coordination:** a sustained 3 m/s² surge, scaled by 0.6 (§6.3), needs 10.6° of pitch, which fits. Unscaled, it would need 17.8° and saturate. The specific-force scaling and limit are therefore part of the workspace design, not just tuning.

Frames are right-handed, with x forward, y left and z up, for both the vehicle and the platform. The pose is `p = (x, y, z)` relative to neutral, plus roll φ, pitch θ and yaw ψ (ZYX order).

---

## 4. Requirements

### 4.1 Functional
| ID | Requirement |
|---|---|
| F01 | IK: leg lengths from pose, `l_i = ‖p + p_0 + R(φ,θ,ψ)·b_i − a_i‖`, for all six legs every cueing cycle. |
| F02 | FK: pose from measured leg lengths by Newton–Raphson with the analytic Jacobian, converging to < 1e-6 m and 1e-6 rad within 6 iterations from the last pose. Used for monitoring and status. |
| F03 | Workspace limiting: if a commanded pose would put any leg outside its soft limits, scale it towards neutral by bisection along the line from neutral to the command (12 iterations). The result is a valid pose of the same "shape", never individually clamped legs. Report a SATURATED flag. |
| F04 | Classical washout (Reid–Nahon structure), from vehicle specific force `f` and angular rate `ω` at the driver's head: scaling and limiting, 3rd-order high-pass translational channel → platform displacement, 2nd-order low-pass tilt coordination → pitch/roll with a 3°/s tilt-rate limit, 2nd-order high-pass rotational channel → platform angles. Discretised with Tustin at 500 Hz; all gains and break frequencies are configurable. |
| F05 | Leg trajectory shaping: leg setpoints respect the speed and acceleration limits (§3) with a second-order rate limiter in leg space, after workspace limiting. |
| F06 | Servo loop per leg at 1 kHz: P position → PI velocity → current command with velocity and gravity feed-forward. The current limit is configurable. |
| F07 | Actuator plant (simulation HAL): motor torque constant, rotor + screw inertia, viscous and Coulomb friction, ball-screw lead, gravity load from statics (`τ = Jᵀ·F`: leg forces from the payload weight at the current pose), current limit, encoder quantisation. |
| F08 | State machine: OFF → HOMING → PARKED → ENGAGING → ENGAGED → PARKING → PARKED, plus FAULT and ESTOP. ENGAGING and PARKING follow S-curves over 3 s; cueing runs only in ENGAGED. |
| F09 | CAN input: VEHICLE_STATE (specific force xyz, angular rate xyz, time) at 100 Hz; the controller interpolates to 500 Hz. Commands: ENABLE, ENGAGE, PARK, RESET_FAULT, SET_PARAM / GET_PARAM (ISO-TP framed like S1). |
| F10 | CAN output: PLATFORM_STATUS (state, pose from FK, flags) at 50 Hz, ACTUATOR_FB (length, speed, current per leg; two frames) at 50 Hz, FAULT on event. |
| F11 | E-stop input (button SW1, normally closed in real hardware): immediate controlled stop of the cueing, then a descent to parked at reduced speed (0.05 m/s). On hardware the brakes engage at the bottom. |
| F12 | Host heartbeat: no VEHICLE_STATE for 200 ms in ENGAGED → washout to neutral, then PARKING (fault code COMMS_LOST). |
| F13 | Faults: following error > 5 mm for 50 ms, leg outside the hard limit, current saturated > 500 ms, cycle overrun > 3 in a row, a thread stalling (task watchdog, as in S1). Every fault → FAULT → controlled descent; RESET_FAULT only when PARKED. |
| F14 | Diagnostics: shell `motion status|legs|pose|params|fault|plant` and per-loop timing statistics (min/max/mean cycle time, overruns). |

### 4.2 Performance and safety
| ID | Requirement |
|---|---|
| P01 | Servo loop 1 kHz and cueing loop 500 Hz with zero overruns in a 10-minute scenario (virtual time; §9.4). |
| P02 | Leg position tracking error ≤ 1 mm RMS and ≤ 3 mm peak in the reference scenarios. |
| P03 | No leg ever exceeds its soft limits, and no leg speed or acceleration exceeds its limits, in any scenario, including a deliberately excessive one (property test over 10 000 random commands). |
| P04 | Tilt-coordination rate never exceeds 3°/s. |
| P05 | A sustained acceleration step returns the platform's translation to neutral (washout) within 6 s, with tilt holding the sustained cue. |
| P06 | From E-stop to standstill: ≤ 0.5 s; then the descent never exceeds 0.05 m/s. |

---

## 5. Architecture

```
 Host (Python)                             motion_fw (Zephyr, STM32H753 in Renode)
 ─────────────                             ──────────────────────────────────────────
 scenario → vehicle model                  CAN RX ─► input interpolation (100 → 500 Hz)
   (f, ω at driver head) ─ VEHICLE_STATE ─►                    │
 commands (ENABLE/ENGAGE/PARK) ─ CMD ─────►        cueing thread 500 Hz
                                                     washout → pose → workspace limit
                                                     → IK → leg rate limiter
                                                               │ leg setpoints
                                                   servo thread 1 kHz (x6 legs)
                                                     P(pos) → PI(vel) → current
                                                     + feed-forward ──► actuator HAL
                                                                         ├─ sim plant (now)
                                                                         └─ drive I/F (hardware)
 3D view + plots ◄─ PLATFORM_STATUS, ACTUATOR_FB ◄─ FK (pose), status, faults
                                                   safety state machine, E-stop, task watchdog
```

**Design choices:**
- **`mcue` is plain C99** with no Zephyr calls. The firmware, the `native_sim` unit tests and the Python reference checks all build or bind the same code (Python through `ctypes` on a host shared library).
- **Workspace limiting happens in pose space by scaling**, not by clamping legs, so the cabin never makes a motion that was not commanded in shape.
- **The plant is behind a HAL interface** (`act_set_current()`, `act_read_position()`). The control code is identical in simulation and on hardware.
- **Fixed-rate loops** are driven by `k_timer`, with cycle-time statistics. Loop timing proves scheduling in virtual time, not silicon latency (same limit as S1).

---

## 6. Algorithms

### 6.1 Inverse kinematics
For leg `i`: `L_i = p + p_0 + R·b_i − a_i`, `l_i = ‖L_i‖`, where `a_i` are the base joints, `b_i` the platform joints in the platform frame, and `p_0 = (0, 0, h_0)`. With `n_i = L_i / l_i`, the Jacobian row of leg `i` is `[n_iᵀ, (R·b_i × n_i)ᵀ]`. This Jacobian is used by FK (Newton step) and by statics (leg forces from the payload wrench).

### 6.2 Forward kinematics
Newton–Raphson: `q_{k+1} = q_k + J(q_k)⁻¹ · (l_measured − IK(q_k))`, starting from the last pose. A 6×6 solve by LU with partial pivoting. Stop when the step is under 1e-9 or after 6 iterations, and report "not converged" as a status flag rather than a fault.

### 6.3 Classical washout (per axis, configurable)
| Channel | Filter | Starting values |
|---|---|---|
| Specific force scaling | gain + soft limit | k = 0.6, limit ±6 m/s² |
| Translational | 3rd-order high-pass `s³ / ((s+ω_b)(s² + 2ζω_n s + ω_n²))`, then double integration | ω_n = 3.0 rad/s, ζ = 1, ω_b = 1.0 rad/s |
| Tilt coordination | 2nd-order low-pass on surge/sway force → `θ = asin(f_x/g)`, `φ = −asin(f_y/g)` | ω = 4 rad/s, ζ = 1; rate limit 3°/s |
| Rotational | gain + 2nd-order high-pass on ω → Euler-rate transform → integrate | k = 0.7, ω_n = 1.0 rad/s, ζ = 1 |

These are typical textbook starting values, to be tuned in week 2 against the reference scenarios.

The translational values were checked while writing this spec. For a sustained 3 m/s² surge (scaled to 1.8), the platform displacement is `X(s) = k·a / ((s+ω_b)(s+ω_n)²)`. The first draft used ω_n = 2.5 and ω_b = 0.3; that peaks at 200 mm (near the −213 mm surge limit) and takes 14.4 s to return within 5 mm, which fails P05.

| ω_n | ω_b | Peak | Back within 5 mm |
|---|---|---|---|
| 2.5 | 0.3 | 200 mm | 14.4 s (fails P05) |
| 2.5 | 0.8 | 144 mm | 6.0 s |
| 3.0 | 1.0 | 99 mm | 4.5 s |

3.0 / 1.0 is chosen: it passes P05 and leaves room for combined motion. The cost is a shorter onset cue, which tilt coordination then carries (it reaches the 10.6° sustained cue in about 3.5 s at the 3°/s rate limit). Tilt is added to the rotational angles before workspace limiting. All filters are discretised with Tustin at 500 Hz and implemented as second-order sections in double precision (the M7 has a double-precision FPU).

### 6.4 Servo
- Position loop P: `Kp_pos`.
- Velocity loop PI with anti-windup: `Kp_vel`, `Ki_vel`.
- Current feed-forward: gravity (from statics) and acceleration (`m_reflected · a_setpoint`).

Gains come from the plant parameters, for a velocity-loop bandwidth of about 60 Hz and a position-loop bandwidth of about 15 Hz. They are verified by step responses in the unit tests.

### 6.5 Actuator plant (simulation)
State: motor angle and speed. Integration: semi-implicit Euler at 10 kHz, 10 sub-steps per servo cycle.

`J_m·ω̇ = K_t·i − b·ω − T_c·sign(ω) − (lead / 2π)·F_load / η`

Leg position `l = l_min + θ·lead/2π`. Encoder reading = `floor(counts)`. Coulomb friction uses a smooth `tanh(ω/ω_ε)` to avoid chattering at zero speed. All parameters are in `platform.json`, with values chosen for a typical 1–2 kW servo actuator and documented.

---

## 7. CAN protocol (`protocol/motion.dbc`)

11-bit IDs: the host uses 0x100–0x10F and the controller 0x180–0x18F. Values are little-endian and scaled integers.

| Frame | Dir | Rate | Content |
|---|---|---|---|
| VEHICLE_STATE_F | host → ctl | 100 Hz | f_x, f_y, f_z (0.001 m/s²), seq (u16) |
| VEHICLE_STATE_W | host → ctl | 100 Hz | ω_x, ω_y, ω_z (0.0001 rad/s), seq (u16) |
| CMD / CMD_ACK | both | on demand | ISO-TP single frames: ENABLE, ENGAGE, PARK, RESET_FAULT, SET/GET_PARAM |
| PLATFORM_STATUS | ctl → host | 50 Hz | state, flags (saturated, FK converged, e-stop), fault code |
| PLATFORM_POSE | ctl → host | 50 Hz | x, y, z (0.1 mm), φ, θ, ψ (0.01°) from FK |
| ACTUATOR_FB_A / _B | ctl → host | 50 Hz | legs 1–3 / 4–6: length (0.1 mm), current (0.1 A) |
| TIMING | ctl → host | 1 Hz | max servo and cueing cycle time (µs), overrun counts |
| FAULT | ctl → host | on event | code, leg, value |

As in S1: the DBC is the source of truth, a generator makes C test vectors, and `--check` runs in CI.

---

## 8. Firmware modules

| Module | Responsibility |
|---|---|
| `mcue/kin` | IK, Jacobian, FK, statics (pure C) |
| `mcue/workspace` | soft limits, bisection scaling |
| `mcue/washout` | filters (SOS, Tustin), tilt coordination with rate limit, Euler-rate transform |
| `mcue/shaping` | leg-space speed/acceleration limiter, S-curve engage/park |
| `mcue/servo` | cascaded position/velocity controller with anti-windup and feed-forward |
| `mcue/plant` | actuator model (simulation HAL backend) |
| `app/cue_task` | 500 Hz loop: input interpolation → washout → workspace → IK → shaping |
| `app/servo_task` | 1 kHz loop: six servo axes, HAL I/O, following-error monitor |
| `app/safety` | state machine, E-stop, heartbeat, fault latching (SMF, as in S1) |
| `app/canproto` | CAN RX/TX, ISO-TP commands (reuses the S1 canlink pattern) |
| `app/diag` | shell, timing statistics, task watchdog |

---

## 9. Verification

### 9.1 Unit tests (ztest on native_sim)
| Suite | Covers |
|---|---|
| kin | F01, F02: neutral pose gives six equal legs = `l_0`; pure heave matches the closed form; IK vs Python/NumPy reference vectors (generated, like S1's DBC vectors); FK round trip on 1000 random valid poses within 1e-6; Jacobian vs finite differences |
| workspace | F03, P03: 10 000 random commands (including excessive ones) → every leg within soft limits; valid commands untouched; scaled pose is collinear with neutral and the command |
| washout | F04, P04, P05: sustained-acceleration step → translation back to neutral within 6 s, tilt converges to `asin(a/g)`, tilt rate ≤ 3°/s; frequency response of the high-pass sections at three frequencies vs analytic values; Tustin stability (poles inside the unit circle) |
| shaping | F05: leg speed/acceleration never above limits on step and random inputs; S-curve duration and end points |
| servo + plant | F06, F07, P02: step 10 mm → settles within 100 ms, overshoot < 5 %; current limit respected; static hold error under full gravity load < 0.1 mm |

Coverage of `mcue` (gcov on native_sim) target: ≥ 90 % lines.

### 9.2 Integration tests (Robot Framework + Renode)
| Test | Covers | Scenario |
|---|---|---|
| IT01 boot | F08, F14 | Boot → OFF; ENABLE → HOMING → PARKED; legs at bottom |
| IT02 engage | F08, F02 | ENGAGE → neutral within 3.5 s; FK pose ≈ 0 (≤ 0.5 mm, 0.05°); PARK → PARKED |
| IT03 launch | F04, P04, P05 | Host sends a 3 m/s² surge step for 8 s → platform surges then washes out; pitch reaches the tilt cue; tilt rate ≤ 3°/s (from PLATFORM_POSE) |
| IT04 lane change | F04, P02 | Sway/roll scenario → tracking error ≤ 1 mm RMS (from ACTUATOR_FB vs setpoints in the shell log) |
| IT05 excessive cue | F03, P03 | 3 g cue → SATURATED flag, legs never beyond soft limits (all ACTUATOR_FB frames checked) |
| IT06 E-stop | F11, P06 | Press SW1 while ENGAGED → standstill ≤ 0.5 s, descent ≤ 0.05 m/s, ESTOP state, RESET only when parked |
| IT07 heartbeat | F12 | Stop VEHICLE_STATE for 300 ms → COMMS_LOST, washout to neutral, PARKING |
| IT08 actuator fault | F13 | Shell command makes leg 3's plant stick → following-error fault within 60 ms → controlled descent |
| IT09 timing | P01 | 10-minute virtual scenario → zero overruns; max cycle times reported in TIMING |
| IT10 params | F09 | SET_PARAM washout gain over CAN → applied, GET returns it; invalid value rejected |

### 9.3 Python reference (`tools/reference/`)
- NumPy implementation of IK, FK and the washout filters (via `scipy.signal`).
- Generates the test vectors and compares the `mcue` shared library on random inputs in CI.

### 9.4 Honest limits
- **Renode timing** is virtual, not cycle-accurate. Loop timing proves scheduling, not silicon latency.
- **The plant model is inside the firmware** (simulation HAL). It is useful for control design and fault tests, but a real actuator brings drive dynamics, compliance and backlash that need commissioning on hardware.
- **Cueing quality is not validated with people.** The washout parameters are textbook starting points, tuned on scenario plots.

---

## 10. Host tools and deliverables

- **`tools/host/motion_host.py`:**
  - runs a scenario (CSV or generated) and sends VEHICLE_STATE over SocketCAN (Renode bridge, as in S1),
  - sends commands,
  - logs everything to CSV.
- **`tools/host/platform_view.py`:**
  - live 3D view of base, legs and cabin, coloured by leg utilisation, from PLATFORM_POSE and ACTUATOR_FB,
  - records frames for screenshots and video.
- **`tools/host/session_plots.py`:** vehicle versus platform specific force, tilt angle and rate, leg strokes against limits, tracking error, loop timing.
- **Repository deliverables, as in S1:**
  - README with real screenshots, a short video and CI badge,
  - `docs/traceability.md`,
  - recorded session with plots.

---

## 11. Milestones (about 8–10 h/week)

| Week | Work | Done when |
|---|---|---|
| 1 | Repository from the S1 template; `platform.json`; `mcue/kin` (IK, Jacobian, FK, statics) and `workspace`; NumPy reference and vector generator | kin and workspace suites green; FK round trip and the 10 000-command property test pass |
| 2 | `washout`, `shaping`; scenario generator; offline closed loop (cueing → kinematics, no servo) run on the host with plots; tune the parameters | washout suite green; P04/P05 shown on plots |
| 3 | `servo`, `plant`; firmware tasks, state machine, CAN protocol and DBC; Renode tests IT01–IT08 | Integration tests green in Renode |
| 4 | Host app, 3D view, video; IT09–IT10; timing; traceability; CI; README | Portfolio-ready repository with green CI |

---

## 12. Open points to verify early (week 1)

- [ ] Renode speed with two fixed-rate loops (1 kHz + 500 Hz, double precision) on one STM32H753: wall-clock cost of a 10-minute virtual run, for P01 and CI time.
- [ ] Zephyr `k_timer` jitter at 1 kHz in Renode, and whether the servo loop needs a hardware timer ISR (counter API) instead.
- [x] Geometry sanity check (done while writing this spec): neutral legs equal at 0.787 m, single-axis workspace as in §3. Week 1 turns the script into `tools/reference/geometry_check.py` and adds combined-pose checks.
- [ ] E-stop button behaviour on the board model (normally closed vs the board's normally open button): emulate NC in firmware config, and document it.
- [ ] CAN load: 2 × 100 Hz in plus about 4 × 50 Hz out ≈ 400 frames/s, well inside 500 kbit/s; confirm with the S1 bridge.

---

## 13. Interview story (60 seconds)

> "I wrote the controller for a 6-DOF motion platform, the kind under a vehicle simulator. Vehicle motion comes in over CAN, a classical washout turns it into platform motion with tilt coordination limited to 3 degrees per second, the inverse kinematics gives six leg lengths, and when a command is outside the workspace I scale the whole pose back by bisection instead of clamping legs, so the cabin never moves in a shape nobody commanded. Six servo loops run at 1 kHz against an actuator model with gravity load from the statics. E-stop, comms loss and following-error faults all end in a controlled descent. The kinematics are checked against a NumPy reference, and a property test throws 10 000 random commands at the limiter without a single leg going past its limit. It is the same robot kinematics I use at work, applied to a simulator."
