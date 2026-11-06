# 6-DOF motion platform controller

Embedded controller for a 6-DOF electric motion platform (a Stewart platform) of the kind under vehicle and crew simulators. It covers motion cueing (classical washout), platform kinematics, workspace limiting, six servo loops and the safety states. It is written in C for Zephyr on an emulated STM32H753 in Renode. Design and requirements are in [SPEC.md](SPEC.md).

**Status: week 1 of 4 (kinematics).** The portable library `mcue` now does the following:
- inverse kinematics,
- analytic Jacobian,
- forward kinematics (Newton–Raphson),
- statics (leg forces from the payload),
- workspace limiting by scaling the whole pose.

Motion cueing comes in week 2, the firmware with servo loops and Renode tests in week 3, and the host tools in week 4.

## Results so far

| Check | Result |
|---|---|
| Unit tests (ztest on `native_sim`) | 14 of 14 pass, no warnings |
| IK, statics and workspace against the NumPy reference vectors | exact to 1e-12 m / 1e-6 N |
| Jacobian against finite differences | within 1e-6 on 20 poses |
| FK round trip, 1000 random poses, cold start from neutral | ≤ 1e-9 everywhere, worst case 6 iterations |
| FK warm start (SPEC F02: last pose ≤ 1 mm / 0.1° away) | ≤ 3 iterations, residual < 1e-9 m |
| Workspace property test: 10,000 random commands up to ~2× the workspace on all axes | no leg ever outside its soft limits; output always the command scaled towards neutral; 5,000 scaled, 5,000 untouched |
| C library vs NumPy on 1000 random poses (up to 25° / 50° yaw) | IK 2e-16 m, FK 3e-14, statics 1.3e-7 N, workspace scale identical |

## Reference geometry

The geometry lives in `config/platform.json`. It is the single source of truth for the C header and the test vectors.
- **Dimensions:** base radius 0.80 m, platform radius 0.50 m, neutral height 0.70 m, stroke 0.25 m.
- **Leg length:** 0.787 m at neutral, 27° from vertical.
- **Single-axis reach:** surge +0.26 / −0.21 m, sway ±0.21 m, heave ±0.13 m, roll ±14°, pitch +17 / −16°, yaw ±27°.
- **Load:** a 500 kg payload puts 919 N on each leg at neutral.

The 30° platform half-angle was chosen by comparing workspace and Jacobian conditioning (SPEC §3).

```bash
python3 tools/reference/geometry_check.py               # workspace, conditioning, leg forces
python3 tools/reference/geometry_check.py --compare 10 20 30 40
```

## Run it (WSL2 Ubuntu, same setup as the S1 trainer firmware)

```bash
bash scripts/test-unit.sh     # ztest unit tests on native_sim
bash scripts/crosscheck.sh    # generated headers current + C vs NumPy on 1000 random poses
python3 tools/reference/gen_headers.py   # after changing config/platform.json
```

## Layout

```
config/platform.json    geometry, payload, limits (single source of truth)
mcue/                   portable C library: include/mcue/*.h, src/*.c, generated platform_config.h
tests/unit/             ztest suites (kin, workspace, linalg) + generated reference vectors
tools/reference/        NumPy reference, header/vector generator, geometry check, C cross-check
scripts/                test and cross-check helpers (bind-mount the repo in WSL: path has a space)
```
