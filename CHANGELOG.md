# Changelog

All notable changes to this project are listed here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/). The firmware version is `app/VERSION`;
0.1.0 and 0.2.0 are library milestones from before the firmware existed.

## [Unreleased]

## [0.4.0] - 2026-11-08

### Fixed
- Servo: a NaN or infinite setpoint, measurement or feed-forward gave the full
  current limit (fmin/fmax turn NaN into the limit) with `saturated` false. It
  now gives 0 A, clears the integral and trips an INTERNAL fault on that leg.
- FK: a NaN leg length could be reported as converged, and a small last step
  counted as converged even with a large leg residual. Convergence now needs a
  finite pose and the residual within tolerance; a non-finite case restores the
  starting pose.
- Controller: the braking stop ended only on exact float equality and had no
  timeout. It now ends on a tolerance (10 µm, 0.1 mm/s) and must finish within
  0.5 s; a timeout while PARKING raises an INTERNAL fault, during FAULT/ESTOP
  the latched fault stays and the descent starts.
- Controller: no monitoring ran during the FAULT/ESTOP stop and descent, and the
  servo bank kept only the first trip. A further leg trip is now reported as its
  own FAULT frame and that leg is released; a leg leaving its hard limits is
  reported too.
- Heartbeat: a host repeating the same VEHICLE frames counted as alive. A sample
  now counts only if its sequence number advances (modulo 2^16).
- Firmware: a failed task-watchdog start was ignored; ENABLE is now refused.
  The actuator HAL and host-report start-up results are checked.
- CAN: `canlink_send` copied `len` bytes without checking `len <= 8`.
- Descent and homing speed: commanded 0.045 m/s instead of exactly the 0.05 m/s
  limit (P06), so the Renode checks use 100 ms windows and the limit itself.

### Changed
- P04 is measured on the delivered tilt (workspace scale × tilt), not on the
  tilt limiter's state. Under workspace saturation (the 3 g cue) it reaches
  29.8°/s, so P04 is now marked partial.
- The servo step and the plant integration run outside the spinlocks; only the
  copies are made under them.
- New fault code 8 (INTERNAL); the FAULT frame carries the leg and value of
  each event, not only of the latched fault.

### Added
- LICENSE (MIT), this changelog, `requirements.txt` (CI installs from it).
- Unit tests for each fix above (72 test cases in total).

## [0.3.0] - 2026-11-07

Milestones M3 (controller and firmware) and M4 (host and long run).

### Added
- Servo (cascaded P/PI with feed-forward and a square-root braking branch),
  ball-screw actuator model with gravity load from the statics, controller core
  (`mcue/ctl`: safety state machine, setpoints, servo bank and monitors).
- Zephyr firmware on the emulated STM32H753 in Renode: 1 kHz servo and 500 Hz
  cueing threads, DBC-defined CAN protocol, host reports, shell, task watchdog.
- Renode integration tests IT01–IT10, including 10 minutes of cueing (IT09).
- SocketCAN host, 3D view, session plots, recorded session and video.
- Requirement traceability (`docs/traceability.md`), coverage gate in CI.

### Fixed
- CI checks: leg speed over longer windows; `gen_headers.py --check` compares
  numbers to 1e-9 instead of bytes.

## [0.2.0] - 2026-11-06

Milestone M2 (motion cueing), library only.

### Added
- Classical washout (Tustin biquads, tilt coordination with rate and
  acceleration limits, rotational channel), leg shaping, S-curves.
- Vehicle scenarios, offline closed loop through ctypes, parameter tuning.

## [0.1.0] - 2026-11-06

Milestone M1 (kinematics), library only.

### Added
- `config/platform.json` and the generated C header; IK, analytic Jacobian,
  FK (Newton–Raphson), statics, workspace limiting by scaling the whole pose.
- NumPy reference, test vectors and the unit-test suites for them.

[Unreleased]: https://github.com/chad416/motion-platform-controller/compare/v0.4.0...HEAD
[0.4.0]: https://github.com/chad416/motion-platform-controller/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/chad416/motion-platform-controller/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/chad416/motion-platform-controller/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/chad416/motion-platform-controller/releases/tag/v0.1.0
