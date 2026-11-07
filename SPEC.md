# S2 — 6-DOF Motion Platform Controller

Specification, revision A (6 Nov 2026), with the week 2, 3 and 4 changes marked where they apply

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
| Actuator | electric, ball screw, 10 mm/rev lead, 17-bit motor encoder (131 072 counts/rev, 0.076 µm per count; revision A had 4096, see §6.5) |

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
| F04 | Classical washout (Reid–Nahon structure), from vehicle specific force `f` and angular rate `ω` at the driver's head: scaling and limiting, 3rd-order high-pass translational channel → platform displacement, 2nd-order low-pass tilt coordination → pitch/roll with a 3°/s tilt-rate limit (and a 20°/s² tilt-acceleration limit, added in week 2), 2nd-order high-pass rotational channel → platform angles. Discretised with Tustin at 500 Hz; all gains and break frequencies are configurable. |
| F05 | Leg trajectory shaping: leg setpoints respect the speed and acceleration limits (§3) with a second-order rate limiter in leg space, after workspace limiting, and never pass the soft limits while the targets are inside them (§6.3b). |
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
| Channel | Filter | Starting values (rev. A) | Tuned in week 2 (§6.3a) |
|---|---|---|---|
| Specific force scaling | gain + soft limit (linear up to half the limit, then a smooth knee) | k = 0.6, limit ±6 m/s² | **k = 0.5** |
| Translational | 3rd-order high-pass `s³ / ((s+ω_b)(s² + 2ζω_n s + ω_n²))`, then double integration | surge/sway ω_n = 3.0, ζ = 1, ω_b = 1.0 rad/s; heave the same with ω_n = 4.0 | surge/sway unchanged; **heave ω_n = 2.0** |
| Tilt coordination | 2nd-order low-pass on surge/sway force → `θ = −asin(f_x/g)`, `φ = +asin(f_y/g)` | ω = 4 rad/s, ζ = 1; rate limit 3°/s | unchanged, plus **acceleration limit 20°/s²** |
| Rotational | gain + 2nd-order high-pass on ω → Euler-rate transform (on this channel's own angles) → integrate | k = 0.7, ω_n = 1.0 rad/s, ζ = 1 | unchanged |

The rev. A values were typical textbook starting points. Week 2 tuned them against the reference scenarios (§6.3a).

**Tilt signs** (corrected in week 2; revision A had them reversed). With R = Rz·Ry·Rx, a positive pitch puts the nose down. The specific force the occupant feels in the platform frame at rest is `Rᵀ·(0, 0, g)`, which gives `f_x = −g·sin θ` and `f_y = +g·sin φ`. A sustained forward acceleration (`f_x > 0`) therefore needs `θ = −asin(f_x/g)` (nose up), and a leftward one needs `φ = +asin(f_y/g)`. The `washout` unit tests check this by computing `Rᵀ·(0, 0, g)` for the commanded tilt.

The translational values were checked while writing this spec. For a sustained 3 m/s² surge (scaled to 1.8), the platform displacement is `X(s) = k·a / ((s+ω_b)(s+ω_n)²)`. The first draft used ω_n = 2.5 and ω_b = 0.3; that peaks at 200 mm (near the −213 mm surge limit) and takes 14.4 s to return within 5 mm, which fails P05.

| ω_n | ω_b | Peak | Back within 5 mm |
|---|---|---|---|
| 2.5 | 0.3 | 200 mm | 14.4 s (fails P05) |
| 2.5 | 0.8 | 144 mm | 6.0 s |
| 3.0 | 1.0 | 99 mm | 4.5 s |

3.0 / 1.0 is chosen: it passes P05 and leaves room for combined motion. The cost is a shorter onset cue, which tilt coordination then carries (it reaches the 10.6° sustained cue in about 3.5 s at the 3°/s rate limit). Tilt is added to the rotational angles before workspace limiting. All filters are discretised with Tustin at 500 Hz and implemented as second-order sections in double precision (the M7 has a double-precision FPU).

### 6.3a Week 2: tuning and what the closed loop found
`tools/reference/closed_loop.py` runs the C library through ctypes on six vehicle scenarios (`scenarios.py`: launch, braking, lane change, cornering, road bump, and an excessive 3 g cue) at 500 Hz: washout → workspace limit → IK → leg shaping → FK. It measures the cue the platform actually delivers, `Rᵀ(a + g)` at the platform centre, against the cue asked for (vehicle force after soft limit and gain). Results and plots are in `results/`.

**Tuning rule.** Use the highest force gain (in 0.05 steps) at which none of the five normal scenarios saturates the workspace. Saturation is safe, because the whole pose is scaled, but it distorts every axis at once. `tools/reference/tune.py` grid-searches gain, ω_n, ω_b and the tilt ω, minimising the mean relative cue error under three constraints: no saturation, no leg past a soft limit, and P05.
- **Gain 0.6 → 0.5.** At 0.6, braking and a 4 m/s² bend saturate the workspace: the bend asks for 14.2° of roll tilt against a ±14° roll range. 0.5 is the highest gain with no saturation.
- **Heave ω_n 4 → 2.** The relative error on the bump falls from 0.96 to 0.76 (with ω_n = 4 the high-pass leads by about 50° at 1.5 Hz), and 48 mm of stroke remains.
- **Tilt ω.** Lowering it below 4 rad/s made every scenario worse, so it stays at 4.

**Defects the closed loop found, each fixed with a regression test:**

| Found | Cause | Fix |
|---|---|---|
| Single-sample force spikes up to ~2 m/s² at every corner of the tilt-rate trace | A rate limit alone steps the tilt rate (0 → 3°/s in one sample); the legs turn the step into a jolt | Tilt is also acceleration-limited (20°/s²), using the same second-order limiter as the legs |
| Smaller spikes while legs track slow targets | The leg limiter switched between "brake to the target" and "land on it" whenever the target speed crossed `a_max·dt` = 12 mm/s | Exact tracking when every limit allows it (§6.3b) |
| A leg 10.8 mm past its soft limit (excessive cue) after that change | Tracking exactly at speed leaves no reserve to stop when the target halts at the limit | Track only while the leg can still stop before the soft limit (§6.3b) |
| 0.8° pitch left after a bend: a permanent false surge cue | The Euler-rate transform used the total roll, including tilt, so yaw rate leaked into pitch | The rotational channel transforms on its own angles |
| 3 µm of overshoot past a fixed target | The braking law `v²/2a + v·dt/2` is exact only at whole multiples of `a·dt` | Brake for the distance less `a·dt²/8` (§6.3b) |
| Unstable-looking first-order high-pass | Built through the biquad formula, which leaves a cancelled pole at z = −1 | A direct first-order bilinear transform |

**A limit no tuning fixes.** The lane change (±3 m/s² at 0.33 Hz) has a relative error of 1.38, which is worse than not moving. At that frequency a ±1.5 m/s² cue needs ±0.35 m of sway, against ±0.21 m of workspace. The translational high-pass then leads by about 135°, and the 3°/s rate limit allows only about ±1.4° of tilt. That is a property of classical washout on this workspace. A model-predictive or optimal washout (v2) is the usual answer.

### 6.3b Leg shaping (F05)
A scalar second-order limiter, `mc_limit2_step`, is used for each leg (and for the tilt). Each step it chooses between two modes:
- **Tracking:** output = target, if the step needed stays within v_max and a_max, the target's own velocity can be matched from it in one step, and the output could still stop before the soft limit in its direction of travel.
- **Braking:** otherwise, move towards the target at the largest speed from which it can still stop there. The braking distance for the semi-implicit update is `v²/2a + v·dt/2`, plus at most `a·dt²/8`, which is why the law brakes for `|e| − a·dt²/8`.

Speed and acceleration limits always hold. A fixed target is reached within 2 ms of the time-optimal trapezoid without overshoot. No leg passes a soft limit while the targets are inside, including targets that run into the limit at full speed and stop dead.

### 6.4 Servo (week 3)
Per leg, in leg units (m, m/s, A), at 1 kHz:

- **Position loop:** `v_cmd = v_sp + sign(e)·min(Kp_pos·|e|, √(2·a_brake·|e|))`, with `e = l_sp − l`.
- **Velocity loop:** PI on `v_cmd − v`. The velocity `v` is the backward difference of the encoder. The integral stops while the output is saturated in the direction it would grow (anti-windup).
- **Feed-forward current:** `a_sp/G + r·F_grav/(η·K_t)`, where:
  - `G = r·K_t/J` is the leg acceleration per amp, and `r = lead/2π`;
  - `F_grav` is the leg force from the statics at the FK pose.
- **Current limit:** 20 A.

**Gains**, designed from the plant model (`mc_servo_design`):

| Gain | Value | Comes from |
|---|---|---|
| Kp_pos | 94.2 1/s | 15 Hz position bandwidth |
| Kp_vel | 265.5 A/(m/s) | 60 Hz velocity bandwidth, G = 1.42 m/s²/A |
| Ki_vel | 20 014 A/m | PI zero at 0.2 × the velocity bandwidth |

**The square-root branch is a week 3 finding.** It only acts above 2·a_brake/Kp_pos² = 2.7 mm of error, with a_brake = 12 m/s².
- Without it, a raw 10 mm step overshot by 8 % and a 100 mm step by 25 %. Once the current saturates, a linear position loop asks for more deceleration than the drive can give.
- With it, both steps have 0 % overshoot. The 10 mm step settles to 2 % in 61 ms.
- Normal tracking errors (µm) stay in the linear branch.

### 6.5 Actuator plant (simulation)
State: motor angle and speed. Integration: semi-implicit Euler at 10 kHz, 10 sub-steps per servo cycle.

`J·ω̇ = K_t·i − b·ω − T_c·tanh(ω/ω_ε) − r·F_load/η`, with `J = J_m + m_load·r²/η`

**What the plant contains:**
- **Leg position and encoder.** `l = l_min + θ·r`. The encoder reports whole counts.
- **End stops** at the stroke ends. They are inelastic: the leg stops dead.
- **Load.** `F_load` is the payload's gravity on this leg, from the statics at the *true* pose (FK of the true leg lengths). It is updated at the 500 Hz cueing rate.
- **Payload inertia.** `m_load` is the payload mass reflected to one leg, `M / (6·cos²α)` = 105 kg (α = 27.2°, the leg angle at neutral).
- **Seized-leg fault injection** (`stuck`), for IT08.

| Parameter (platform.json `actuator`) | Value |
|---|---|
| Screw lead, efficiency | 10 mm/rev, 0.90 |
| K_t, J_m (rotor + screw) | 0.55 Nm/A, 3.2e-4 kg m² (a 1.5 kW, 3000 rpm class servo motor) |
| Viscous, Coulomb friction (smoothing) | 2e-4 Nm/(rad/s), 0.10 Nm (ω_ε = 1 rad/s) |
| Drive current limit | 20 A (11 Nm, about 6.2 kN on the leg) |
| Encoder | 17 bits, 131 072 counts/rev (0.076 µm per count) |

**Encoder, changed in week 3.** Revision A had 4096 counts/rev, but a 1 kHz velocity estimate from it is quantised to 1.5 rad/s, which the 60 Hz velocity loop turns into ±0.65 A (about 200 N) of current noise. Servo motors of this class have 17–23 bit absolute encoders.

Holding the payload at neutral takes 919 N per leg, which is 2.95 A.

### 6.6 Controller core and safety state machine (week 3)
`mcue/ctl` holds the state machine, setpoint generation and the six servos with their monitors. It is plain C with no RTOS calls, so the same code runs in three places:
- in the firmware threads,
- in the native_sim unit tests (`ctl` suite, against six plant models),
- through ctypes on the host.

It has two halves:
- `mc_ctl_step` runs at 500 Hz: commands, E-stop, heartbeat, faults, states, setpoints, FK pose and the gravity feed-forward.
- `mc_bank_step` runs at 1 kHz: six servos, plus the following-error and current-saturation monitors.

They exchange one small struct in each direction (`mc_servo_cmd`, `mc_servo_fb`). Two details make that exchange safe:
- **Setpoint interpolation.** The cueing step sends two setpoints one period apart. The servo interpolates between them, so its setpoint is linear in time, and its velocity feed-forward is their difference.
- **Stale trips.** The feedback carries the RESET counter it belongs to, and a trip from before the last RESET is ignored. Without this, the first step after a RESET faulted again on the servo's still-latched trip (found by the `ctl` tests).

| State | Leg setpoints |
|---|---|
| OFF | servos off; legs rest on the lower end stops |
| HOMING | encoder plausibility check, then an S-curve to park at ≤ 0.05 m/s |
| PARKED | hold at park (the lower soft limit) |
| ENGAGING | S-curve to neutral in 3 s (peak 72 mm/s) |
| ENGAGED | washout → workspace → IK → leg shaping |
| PARKING | stop at the acceleration limit, then an S-curve to park (≥ 3 s) |
| FAULT, ESTOP | stop at the acceleration limit, stand still 0.25 s, then descend to park at ≤ 0.05 m/s |

- **Fault sources (F13):**
  - following error > 5 mm for 50 ms;
  - current saturated for 500 ms;
  - a leg outside its hard limits, 5 mm inside the end stops;
  - more than 3 overruns in a row of either loop;
  - an implausible encoder at ENABLE;
  - the task watchdog, which reboots as in S1.
- **RESET** is accepted only when parked and with the E-stop released. It goes straight to PARKED if every leg is within 1 mm of park. Otherwise it re-homes through HOMING.
- **Released leg (week 3 finding).** A leg that tripped its own monitor (seized, or too weak for the load) is *released*: its setpoint follows its encoder and its integral is cleared, so it gets only the gravity feed-forward.
  - Before this, the servo kept pushing a seized leg at the current limit while the others descended.
  - Its wound-up integral then dropped the leg once it was freed.
- **Comms loss (F12)** is an event, not a latched fault. The input is set to rest, and the washout returns to neutral (within 5 mm and 0.5°, at most 10 s). Then the controller enters PARKING.
- **Servo margin (week 3 finding).** While cueing, leg setpoints stay 0.1 mm inside the soft limits.
  - The workspace limit and the shaper keep the *setpoints* inside the soft limits.
  - In IT05, the servo's few µm of tracking error still carried a *measured* leg to 240.01 mm against the 240 mm soft limit.

---

## 7. CAN protocol (`protocol/motion.dbc`)

11-bit IDs: the host uses 0x100–0x10F and the controller 0x180–0x18F. A lower ID means higher bus priority. Values are little-endian and scaled integers; encoders round and saturate.

| ID | Frame | Dir | Rate | Content |
|---|---|---|---|---|
| 0x100 | VEHICLE_F | host → ctl | 100 Hz | f_x, f_y, f_z (0.002 m/s², ±65 m/s²), seq |
| 0x101 | VEHICLE_W | host → ctl | 100 Hz | ω_x, ω_y, ω_z (0.0001 rad/s), seq |
| 0x10F | CMD | host → ctl | on demand | ISO-TP single frame: opcode (1 ENABLE, 2 ENGAGE, 3 PARK, 4 RESET_FAULT, 5 SET_PARAM, 6 GET_PARAM), key, value (i32) |
| 0x180 | FAULT | ctl → host | on event | code, leg (1–6), state, count, value (µm or mA) |
| 0x181 | STATUS | ctl → host | 50 Hz | state, flags (saturated, FK converged, E-stop, comms lost, descended), fault, fault leg, workspace scale, last event, seq |
| 0x182 / 0x183 | POSE_T / POSE_R | ctl → host | 50 Hz | x, y, z (0.1 mm) / roll, pitch, yaw (0.01°), from FK; seq |
| 0x184 / 0x185 | LEG_POS_A / _B | ctl → host | 50 Hz | stroke above the lower end stop (0.01 mm), legs 1–3 / 4–6; seq |
| 0x186 / 0x187 | LEG_CUR_A / _B | ctl → host | 50 Hz | current (0.01 A), legs 1–3 / 4–6; seq |
| 0x188 | TIMING | ctl → host | 1 Hz | worst servo and cueing cycle time (µs), overrun counts |
| 0x18F | CMD_ACK | ctl → host | per CMD | opcode, key, status (0 OK, 1 BAD_STATE, 2 ESTOP, 3 BAD_KEY, 4 BAD_VALUE), value |

**Changes from revision A, where its frames did not fit:**
- The pose and the leg feedback needed 12 bytes. They are now two frames each (POSE_T/R, LEG_POS, LEG_CUR).
- 0.001 m/s² of specific force overflows int16 at the 3 g heave cue (39 m/s²). The resolution is now 0.002 m/s².

**Load:** about 550 frames/s, roughly 12 % of 500 kbit/s.

As in S1: the DBC is the source of truth. `protocol/gen_vectors.py` encodes test vectors with cantools, the `proto` unit tests compare the C encoders and decoders byte for byte, and `--check` runs in CI.

---

## 8. Firmware modules

| Module | Responsibility |
|---|---|
| `mcue/kin` | IK, Jacobian, FK, statics (pure C) |
| `mcue/workspace` | soft limits, bisection scaling |
| `mcue/washout` | filters (SOS, Tustin), tilt coordination with rate limit, Euler-rate transform |
| `mcue/shaping` | leg-space speed/acceleration limiter, stop point, S-curve leg moves |
| `mcue/servo` | cascaded position/velocity controller with anti-windup and feed-forward |
| `mcue/plant` | actuator model (simulation HAL backend) |
| `mcue/ctl` | controller core: state machine, setpoints, servo bank, monitors (§6.6) |
| `app/motion` | servo thread (1 kHz `k_timer`, priority 1) and cue thread (500 Hz, released by every second servo tick, priority 2); spinlock exchange; timing and overrun statistics; vehicle frame pairing by seq |
| `app/hal` | actuator HAL; `act_sim.c` is the simulation backend (six plants, gravity load from the statics at the true pose) |
| `app/can`, `app/proto` | CAN RX/TX (the S1 canlink pattern); payload encode/decode (unit-tested against the DBC) |
| `app/host` | 50 Hz reports, FAULT on events, 1 Hz TIMING, CMD → CMD_ACK |
| `app/shell`, `app/sys` | `motion status\|legs\|pose\|params\|fault\|plant\|timing\|track\|cmd\|set\|get\|seize`; task watchdog (from S1) |

The state machine is a plain switch in `mcue/ctl`, not Zephyr SMF. That keeps it portable to the unit tests and the ctypes host build.

---

## 9. Verification

### 9.1 Unit tests (ztest on native_sim)
| Suite | Covers |
|---|---|
| kin | F01, F02: neutral pose gives six equal legs = `l_0`; pure heave matches the closed form; IK vs Python/NumPy reference vectors (generated, like S1's DBC vectors); FK round trip on 1000 random valid poses within 1e-6; Jacobian vs finite differences |
| workspace | F03, P03: 10 000 random commands (including excessive ones) → every leg within soft limits; valid commands untouched; scaled pose is collinear with neutral and the command |
| filters | Tustin sections vs the continuous prototype at the pre-warped frequency (three frequencies, re and im to 1e-9); DC/Nyquist gains; unstable prototypes rejected |
| washout | F04, P04, P05: sustained-acceleration step vs the closed form (to 1e-5 m), translation back to neutral within 6 s, tilt converges to `asin(a/g)`; tilt signs via `Rᵀ·(0, 0, g)`; tilt rate ≤ 3°/s and acceleration ≤ 20°/s² under reversals; heave and yaw-rate steps vs closed form; no residual angle after a bend |
| shaping | F05, P03: limits on step, random-jump and sine inputs; fixed targets reached in near-minimum time without overshoot (2000 random moves); feasible moving targets followed exactly; targets that stop dead at a soft limit never passed; S-curve shape |
| servo + plant | F06, F07, P02: plant vs closed forms (free acceleration, holding current, end stop, encoder); 10 mm step → 0 % overshoot, settles (2 %) in 61 ms; a saturated 100 mm step stays at the current limit with no windup overshoot; static hold under the full 919 N load without feed-forward < 0.1 µm after 0.5 s; full-stroke S-curve at the leg limits tracked to 9.3 µm peak |
| ctl | F08, F11–F13, P02, P06 at the firmware rates against six plant models (the unit-level twins of IT01–IT08): homing, engage/park, surge-step cueing (tracking 0.3 µm RMS, 4.1 µm peak), E-stop (standstill 56 ms from 0.26 m/s, descent ≤ 0.05 m/s), heartbeat loss, seized leg (fault 50 ms after the error passes 5 mm; released, re-homed after RESET), hard limit, current saturation (trip at 500 ms), overrun, implausible encoder, E-stop blocking ENABLE and RESET, parameters, input interpolation |
| proto | F09, F10: every frame type against vectors encoded from `motion.dbc` with cantools; IDs match the DBC; ISO-TP framing; saturation instead of wrap-around |

Coverage of `mcue` (gcov on native_sim) target: ≥ 90 % lines. **Week 4:** 96.3 % of lines (902 of 937) and 100 % of functions (gcovr through `twister --coverage`); CI fails below 90 %.

### 9.2 Integration tests (Robot Framework + Renode)
The firmware runs on the emulated STM32H753 with its simulated actuators. `sim/host_feed.py` is an IronPython object inside Renode that stands in for the host on the CAN bus, so the tests are deterministic:
- It streams VEHICLE_F/W at 100 Hz on **virtual time**, straight into FDCAN1, from scenario CSVs (`sim/data`, exported by `scenarios.py`).
- It sends CMD frames.
- It records every frame the controller sends, with its virtual time.

`tests/robot/motionlib.py` decodes that record with the DBC layouts. Speeds are taken over 100 ms windows: the 50 Hz frames carry 1–2 ms of send jitter, which over a single 20 ms step would be a 10 % error.

| Test | Covers | Scenario |
|---|---|---|
| IT01 boot | F08, F14 | Boot → OFF with the legs on the end stops; ENGAGE refused; ENABLE → HOMING → PARKED at the lower soft limit (± 0.02 mm), homing ≤ 0.05 m/s; every shell report (`status`, `legs`, `pose`, `params`, `plant`, `timing`) answers |
| IT02 engage | F08, F02 | ENGAGE → ENGAGED within 3.5 s; FK pose neutral (≤ 0.5 mm, 0.05°); PARK → PARKED |
| IT03 surge step | F04, P04, P05 | A pure 3 m/s² surge step for 8 s (`surge_step`, no body rotation, so the pitch is the tilt alone). Checks: platform surges > 50 mm, back within 5 mm 6 s after the step, pitch −8.80 ± 0.1°, tilt rate ≤ 3.1°/s from POSE_R (the 0.01° resolution over 100 ms) |
| IT04 lane change | F04, P02 | Lane change; `motion track` (setpoint − encoder in the 1 kHz servo thread): ≤ 1 mm RMS and ≤ 3 mm peak |
| IT05 excessive cue | F03, P03 | 3 g cue → SATURATED; every LEG_POS frame inside the soft limits (10–240 mm); no fault |
| IT06 E-stop | F11, P06 | SW1 while moving → ESTOP and a FAULT frame; standstill ≤ 0.5 s; descent ≤ 0.05 m/s; RESET answers ESTOP while held, BAD_STATE until parked, then OK → PARKED |
| IT07 heartbeat | F12 | Stop VEHICLE_STATE while ENGAGED → COMMS_LOST FAULT frame 200–230 ms later; washout to neutral; PARKING → PARKED; no latched fault |
| IT08 seized leg | F13 | Leg 3 seized half-way through ENGAGING (all legs at 72 mm/s) → FOLLOWING fault on leg 3 within 150 ms (≈ 70 ms to pass 5 mm + 50 ms); other legs descend ≤ 0.05 m/s; freed → RESET → HOMING → PARKED |
| IT09 timing | P01 | 600 s ENGAGED, all seven scenarios in turn (including the 3 g cue) → zero overruns, worst cycle times inside their periods (TIMING and `motion timing`), no fault, no vehicle sample dropped, no STATUS sequence gap. Tagged `long`: about 11 min wall, run weekly and on demand in CI |
| IT10 params | F09 | GET force gain (0.500); SET 0.250 → GET 0.250; SET 1.5 → BAD_VALUE, unknown key → BAD_KEY, both change nothing; SET while ENGAGED → BAD_STATE. Applied: the surge step's pitch holds asin(0.25·3/g) = −4.39° (± 0.1°) and the onset peak halves (40.95 ± 2 mm) |

**Week 3 results in Renode** (all eight pass, 118 s wall for about 75 s of virtual time):

| Test | Measured |
|---|---|
| IT01 | homing peak 0.047 m/s; parked at 10.00 mm stroke |
| IT02 | engage 3.00 s; FK pose 0.0 mm / 0.00° |
| IT03 | onset peak 81.9 mm; pitch −8.80°; tilt rate 3.099°/s (3.0 + resolution) |
| IT04 | 0.5 µm RMS, 5.5 µm peak over 10 499 servo samples |
| IT05 | minimum workspace scale 0.405; strokes 10.09–239.91 mm |
| IT06 | standstill 44 ms after SW1 (incl. 30 ms debounce); descent 0.0500 m/s |
| IT07 | COMMS_LOST reported 201 ms after the last sample |
| IT08 | fault 122 ms after the seize; descent 0.0500 m/s |

**Loop timing** (TIMING frames, virtual time): servo worst 127 µs of 1000, cueing worst 581 µs of 2000 (3 g cue: the bisection runs every step), zero overruns in every test.

**Week 4 results** (IT01–IT08 and IT10 pass in 127 s wall; IT09 in 11 min):

| Test | Measured |
|---|---|
| IT09 | 601 s virtual: 604 481 servo cycles, worst 119 µs, mean 104 µs; 302 240 cueing cycles, worst 592 µs, mean 473 µs; 0 overruns; 60 410 vehicle samples, 0 dropped; 30 225 STATUS frames, no gap |
| IT10 | ACKs as specified; onset peak 40.9 mm (IT03: 81.9 mm); pitch −4.39° |

**Live host session** (`scripts/demo.sh`, `docs/session/`): the same firmware in Renode, bridged to SocketCAN `vcan0` and driven by `motion_host.py` through five scenarios, braking and an E-stop. 71.6 s of controller time, 7116 vehicle samples at a mean 10.00 ms on the controller clock (8.6–11.4 ms), no fault apart from the E-stop, servo worst 133 µs, cueing worst 591 µs, 0 overruns. The surge step played the same way reproduces IT03: onset 81.9 mm, pitch −8.80°.

### 9.3 Python reference (`tools/reference/`)
- NumPy implementation of IK, FK and the washout filters (via `scipy.signal`).
- Generates the test vectors and compares the `mcue` shared library on random inputs in CI.

### 9.4 Honest limits
- **Renode timing** is virtual, not cycle-accurate. Loop timing proves scheduling, not silicon latency.
- **The plant model is inside the firmware** (simulation HAL). It is useful for control design and fault tests, but a real actuator brings drive dynamics, compliance and backlash that need commissioning on hardware.
- **Cueing quality is not validated with people.** The washout parameters are tuned on six scenarios against an objective metric (relative error of the delivered specific force at the platform centre, not at the head), not by motion-sickness or fidelity ratings. Classical washout cannot cue a 0.33 Hz lane change on this workspace (§6.3a).
- **The scenarios are simplified.** Vehicle body roll and pitch enter as rates only; their gravity component is left out of the specific force.
- **The legs are modelled decoupled** (week 3). Each plant carries its gravity load from the statics at the true pose, updated at 500 Hz, plus a constant reflected payload mass.
  - A real platform couples the legs through the full 6×6 mass matrix and the payload's own inertial forces.
  - The tracking figures (µm) are therefore for this model. Expect more on hardware.
- **No brakes or drive enable chain is modelled.** OFF means zero current, and the end stops hold the legs. On hardware, OFF and the bottom of an E-stop descent engage the brakes and the drives' safe torque off.

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

**Week 4, as built (`tools/host/`):**
- **`protocol.py`** decodes and encodes every frame from `motion.dbc` with cantools, the same source the firmware's proto unit tests check against.
- **`motion_host.py`** (python-can, SocketCAN):
  - `cmd`, `get`, `set` and `play` actions; every CMD waits for its CMD_ACK.
  - A log of every frame in both directions (decoded, with the controller time), plus NOTE rows for the scenario names.
  - **Pacing.** On hardware the 100 Hz stream runs on the wall clock (`--clock wall`). The emulated controller runs slower than real time, so by default the host paces the stream on the controller's own clock (`--clock ctl`): the 50 Hz STATUS seq is 20 ms of controller time, interpolated between frames with the measured rate and never stepping back. A pause in the emulation then pauses the stream instead of tripping the 200 ms heartbeat.
- **`platform_view.py`:** a matplotlib 3D view of base, legs (shaded by utilisation) and cabin at the FK pose, with leg strokes against the soft limits and 10 s traces of surge and pitch. Live from SocketCAN, or replayed from a session log on the controller clock into PNG frames and an MP4.
- **`session_plots.py`:**
  - asked vs delivered specific force (the closed-loop definition, from the pose by a Savitzky–Golay second derivative);
  - roll and pitch, and their rates;
  - strokes, currents and loop timing;
  - `summary.md`.
- **`demo_session.py` + `scripts/demo.sh`:** Renode with `sim/motion_live.resc` (SocketCAN bridge), the scripted session, then plots, screenshots and video into `docs/`.
- **Not in CI:** the SocketCAN path, because the hosted runners have no `vcan` module. CI runs the plots and the 3D view on the recorded session (`docs/session/session.csv.gz`).

---

## 11. Milestones (about 8–10 h/week)

| Week | Work | Done when |
|---|---|---|
| 1 | Repository from the S1 template; `platform.json`; `mcue/kin` (IK, Jacobian, FK, statics) and `workspace`; NumPy reference and vector generator | kin and workspace suites green; FK round trip and the 10 000-command property test pass |
| 2 | `washout`, `shaping`; scenario generator; offline closed loop (cueing → kinematics, no servo) run on the host with plots; tune the parameters | washout suite green; P04/P05 shown on plots |
| 3 | `servo`, `plant`; firmware tasks, state machine, CAN protocol and DBC; Renode tests IT01–IT08 | Integration tests green in Renode: **done 6 Nov** (61 unit tests, IT01–IT08) |
| 4 | Host app, 3D view, video; IT09–IT10; timing; traceability; CI; README | Portfolio-ready repository with green CI: **done 6 Nov** except the first CI run, which needs the push (IT01–IT08 and IT10, IT09, coverage 96.3 %, live host session, `docs/traceability.md`) |

---

## 12. Open points to verify early (week 1; answered in week 3)

- [x] Renode speed with two fixed-rate loops (1 kHz + 500 Hz, double precision) on one STM32H753: wall-clock cost of a 10-minute virtual run, for P01 and CI time. **Week 3:** about 1.6 s of wall time per virtual second with both loops and the 50 Hz reports running (75 s virtual in 118 s), so a 10-minute IT09 run takes about 16 minutes: fine for a nightly or manual job, too long for every push. **Week 4:** IT09 took 11 min for 601 s of virtual time (1.1 s wall per virtual second; the short tests spend more of their time on boot and keyword round trips). It runs weekly and on demand in CI.
- [x] Zephyr `k_timer` jitter at 1 kHz in Renode, and whether the servo loop needs a hardware timer ISR (counter API) instead. **Week 3:** `k_timer` with a 10 kHz tick is enough. Every test ran with zero overruns, the servo loop worst case was 127 µs and the cueing loop 581 µs (virtual time).
- [x] Geometry sanity check (done while writing this spec): neutral legs equal at 0.787 m, single-axis workspace as in §3. Week 1 turns the script into `tools/reference/geometry_check.py` and adds combined-pose checks.
- [x] E-stop button behaviour on the board model (normally closed vs the board's normally open button): emulate NC in firmware config, and document it. **Week 3:** SW1 pressed stands for "circuit open" (main.c). ESTOP latches in the state machine, and RESET needs the button released and the platform parked.
- [x] CAN load: 2 × 100 Hz in plus about 4 × 50 Hz out ≈ 400 frames/s, well inside 500 kbit/s; confirm with the S1 bridge. **Week 3:** with the split frames (§7) it is 200 in plus 350 out, about 550 frames/s (about 12 % of the bus). No 50 Hz frame lost a sequence number in any Renode test recording (for example, IT03 has 997 STATUS frames in 19.9 s). **Week 4:** the SocketCAN bridge (`sim/motion_live.resc`) carries the live host session with no lost STATUS sequence number either.

---

## 13. Interview story (60 seconds)

> "I wrote the controller for a 6-DOF motion platform, the kind under a vehicle simulator. Vehicle motion comes in over CAN, a classical washout turns it into platform motion with tilt coordination limited to 3 degrees per second, the inverse kinematics gives six leg lengths, and when a command is outside the workspace I scale the whole pose back by bisection instead of clamping legs, so the cabin never moves in a shape nobody commanded. Six servo loops run at 1 kHz against an actuator model with gravity load from the statics. E-stop, comms loss and following-error faults all end in a controlled descent. The kinematics are checked against a NumPy reference, and a property test throws 10 000 random commands at the limiter without a single leg going past its limit. It is the same robot kinematics I use at work, applied to a simulator."
