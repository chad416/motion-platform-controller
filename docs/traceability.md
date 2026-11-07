# Requirement traceability

Every requirement in [SPEC.md](../SPEC.md) §4, the tests that cover it, and its status.
- **Unit tests** are ztest suites in `tests/unit/src/`, run on `native_sim`.
- **IT tests** are Robot Framework tests in `tests/robot/motion.robot`, run against the firmware in Renode.
- **Closed loop** is `tools/reference/closed_loop.py --check`, which runs the C library through ctypes.

Results are from the week 4 runs on 6 Nov 2026 (firmware 0.3.0):
- 61/61 unit tests; line coverage of `mcue` 96.3 % (gcovr, 902 of 937 lines);
- IT01–IT08 and IT10 all passing;
- IT09 (601 s of virtual time, 11 min wall) passing.

**Status key:**
- **Met:** implemented and tested.
- **Partial:** part of the requirement is implemented or tested; the note says which part.

## Functional

| ID | Requirement (short) | Tests | Status | Note |
|---|---|---|---|---|
| F01 | IK for all six legs every cueing cycle | unit `kin` (`test_neutral_and_pure_heave_closed_form`, `test_ik_matches_numpy_reference`); crosscheck (1000 random poses vs NumPy, 2e-16 m) | Met | |
| F02 | FK by Newton–Raphson, < 1e-6 in ≤ 6 iterations from the last pose | unit `kin` (`test_fk_round_trip_1000_random_poses`, `test_fk_warm_start_meets_spec`); IT02 (FK pose neutral within 0.5 mm / 0.05°) | Met | Worst case 3 iterations warm, 6 cold |
| F03 | Workspace limiting by scaling the whole pose (bisection), SATURATED flag | unit `workspace` (3, including the 10 000-command property test); IT05 (SATURATED set, scale down to 0.405) | Met | |
| F04 | Classical washout, Tustin at 500 Hz, 3°/s tilt rate, 20°/s² tilt acceleration, configurable | unit `filters` (6), `washout` (9); IT03; IT10 (gain changed over CAN); closed loop (six scenarios) | Met | C vs SciPy ≤ 1.6e-11 on every output |
| F05 | Leg shaping: speed and acceleration limits, soft limits never passed | unit `shaping` (7); closed loop | Met | |
| F06 | Servo per leg at 1 kHz: P → PI → current + feed-forward, configurable current limit | unit `servo` (8); IT04 (tracking in the firmware) | Met | The current limit is a build-time setting (`config/platform.json`) |
| F07 | Actuator plant: Kt, inertia, friction, lead, gravity load from statics, current limit, encoder | unit `servo` (`test_free_acceleration_matches_closed_form`, `test_balance_current_holds_the_load`, `test_current_limit_and_encoder`, `test_unpowered_leg_rests_on_the_lower_stop`), unit `kin` (`test_statics_*`) | Met | The legs are modelled decoupled (SPEC §9.4) |
| F08 | State machine OFF → HOMING → PARKED → ENGAGING → ENGAGED → PARKING, FAULT, ESTOP | unit `ctl` (`test_boot_enable_homes_to_park`, `test_engage_and_park`); IT01, IT02 | Met | |
| F09 | CAN input at 100 Hz, interpolated to 500 Hz; ENABLE/ENGAGE/PARK/RESET/SET/GET as ISO-TP frames | unit `ctl` (`test_input_interpolation`, `test_params`), unit `proto` (4); IT01, IT06, IT10; live host session (100 Hz over SocketCAN) | Met | |
| F10 | CAN output: status and FK pose at 50 Hz, leg feedback at 50 Hz, FAULT on event | unit `proto` (every frame against the DBC); IT02 (POSE), IT05 (LEG_POS), IT06–IT08 (FAULT), IT09 (STATUS, no sequence gap over 10 min); session plots (LEG_CUR) | Partial | Length and current are sent per leg; speed is not, because it did not fit with the split frames (SPEC §7). The host derives it from LEG_POS (0.01 mm at 50 Hz), as the tests do over 100 ms windows. |
| F11 | E-stop: controlled stop, then descent at ≤ 0.05 m/s | unit `ctl` (`test_estop_stops_then_descends`, `test_estop_blocks_enable`); IT06; demo session | Met | Simulated as SW1 = normally-closed circuit open. Brakes and safe torque off are hardware (SPEC §9.4). |
| F12 | Heartbeat: no vehicle data for 200 ms → washout to neutral, PARKING, COMMS_LOST | unit `ctl` (`test_heartbeat_loss_washes_out_then_parks`); IT07 | Met | Reported 201 ms after the last sample |
| F13 | Faults: following error, hard limit, current saturation, overruns, task watchdog → controlled descent; RESET only when parked | unit `ctl` (`test_seized_leg_faults_and_descends`, `test_hard_limit_fault`, `test_current_saturation_trips_after_500ms`, `test_overrun_faults`, `test_implausible_encoder_at_enable`); IT06, IT08 | Partial | The task watchdog is the S1 module, registered on the servo, cueing, host and CAN TX threads, but no S2 test stalls a thread (S1's IT11 does). A watchdog stall resets the controller, so it ends with the drives off, not in a controlled descent; on hardware the brakes hold the platform. |
| F14 | Diagnostics: shell `status`, `legs`, `pose`, `params`, `fault`, `plant`; loop timing statistics | IT01 (every report answers), IT04 (`track`), IT06/IT08 (`fault`), IT09 (`timing`); TIMING frame (IT09) | Met | |

## Performance and safety

| ID | Requirement (short) | Tests | Status | Note |
|---|---|---|---|---|
| P01 | Zero overruns in a 10-minute scenario, servo 1 kHz, cueing 500 Hz | IT09 | Met | 604 481 servo cycles (worst 119 µs of 1000, mean 104 µs) and 302 240 cueing cycles (worst 592 µs of 2000, mean 473 µs), all seven scenarios in turn including the 3 g cue; 0 overruns, 0 of 60 410 vehicle samples dropped, no STATUS sequence gap. Virtual time (SPEC §9.4). |
| P02 | Leg tracking ≤ 1 mm RMS, ≤ 3 mm peak | unit `servo` (`test_s_curve_tracking`), unit `ctl` (`test_surge_step_cueing_and_tracking`); IT04 | Met | 0.5 µm RMS, 5.5 µm peak in the firmware (lane change). For this plant model; expect more on hardware (SPEC §9.4). |
| P03 | No leg past its soft limits, speed or acceleration limits, in any scenario; 10 000-command property test | unit `workspace` (`test_property_10000_random_commands`), unit `shaping` (`test_random_moving_targets_keep_limits`, `test_targets_stopping_at_the_soft_limit_are_not_passed`); IT05 (strokes 10.09–239.91 mm under the 3 g cue); closed loop | Met | The servo margin (0.1 mm) keeps the measured leg inside even with its tracking error |
| P04 | Tilt-coordination rate ≤ 3°/s | unit `washout` (`test_tilt_rate_never_exceeds_limit`); IT03 (≤ 3.1°/s: 3.0 plus the POSE_R resolution); closed loop | Met | |
| P05 | Sustained step: translation back to neutral within 6 s, tilt holds the cue | unit `washout` (`test_surge_step_washes_out`); IT03 (pitch −8.80°, back within 5 mm); IT10 (−4.39° at half gain) | Met | |
| P06 | E-stop to standstill ≤ 0.5 s, then descent ≤ 0.05 m/s | unit `ctl` (`test_estop_stops_then_descends`); IT06 | Met | Standstill 44 ms after SW1 (including the 30 ms debounce); descent at most 0.0496 m/s (500 ms windows) |

## Not covered in simulation

- **Silicon timing.** Renode is not cycle-accurate, so the loop times are scheduling in virtual time, not latency on an STM32H753.
- **Coupled leg dynamics, drive dynamics, compliance, backlash.** The plant is six decoupled ball-screw models with the payload's gravity load from the statics (SPEC §9.4).
- **Brakes and safe torque off.** Not modelled; OFF is zero current with the legs on their end stops.
- **Cueing quality with people.** The washout is tuned against an objective metric on six scenarios, not rated by drivers.
