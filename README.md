# 6-DOF motion platform controller

Embedded controller for a 6-DOF electric motion platform (a Stewart platform) of the kind under vehicle and crew simulators. It covers motion cueing (classical washout), platform kinematics, workspace limiting, six servo loops and the safety states. It is written in C for Zephyr on an emulated STM32H753 in Renode. Design and requirements are in [SPEC.md](SPEC.md).

**Status: week 2 of 4 (motion cueing).** The portable library `mcue` now does the following:
- **Kinematics (week 1):** inverse kinematics, analytic Jacobian, forward kinematics (Newton–Raphson), statics, and workspace limiting by scaling the whole pose.
- **Cueing (week 2):** classical washout (Tustin biquads, tilt coordination limited in rate and acceleration, rotational channel), and leg shaping (a second-order speed/acceleration limiter that never passes the soft limits), with S-curves for engage/park.

An offline closed loop runs the C library on six vehicle scenarios and measures the cue the platform actually delivers. Tuning on it changed two parameters, added one (the tilt acceleration limit) and exposed six defects; each was fixed and now has a regression test (SPEC §6.3a). The firmware with servo loops and Renode tests comes in week 3, and the host tools in week 4.

## Results so far

| Check | Result |
|---|---|
| Unit tests (ztest on `native_sim`) | 36 of 36 pass, no warnings |
| IK, statics and workspace against the NumPy reference vectors | exact to 1e-12 m / 1e-6 N |
| FK round trip, 1000 random poses, cold start / warm start | ≤ 1e-9, worst case 6 / 3 iterations |
| Workspace property test: 10,000 random commands up to ~2× the workspace | no leg ever outside its soft limits; output always the command scaled towards neutral |
| C library vs NumPy on 1000 random poses | IK 2e-16 m, FK 3e-14, statics 1.3e-7 N, workspace scale identical |
| Tustin sections vs continuous prototypes at the pre-warped frequency | within 1e-9 (re and im), three frequencies, all section types |
| C washout vs an independent SciPy implementation, all six scenarios | ≤ 1.6e-11 on every output |
| P05: sustained 3 m/s² surge | translation back within 5 mm after 4.2 s (limit 6 s); pitch holds −8.79° for a −8.80° cue |
| P04: tilt-coordination rate | ≤ 3.000°/s in every scenario; acceleration ≤ 20°/s² |
| P03: leg soft limits in the closed loop, including a 3 g cue | never passed (closest +0.00015 mm, cue scaled down to 0.41) |
| Leg shaper: fixed targets / feasible moving targets | reached within about 2 ms of time-optimal without overshoot / followed exactly |

## Closed loop (week 2)

`tools/reference/closed_loop.py` runs washout → workspace limit → IK → leg shaping → FK at 500 Hz, through ctypes on the real C library. It compares the specific force the platform delivers, `Rᵀ(a + g)` at the platform centre, with the cue asked for. The full table is in [results/closed_loop.md](results/closed_loop.md).

| Scenario | Peak x/y/z (mm) | Peak roll/pitch/yaw (°) | Saturated | Closest to soft limit | Relative cue error |
|---|---|---|---|---|---|
| launch, 3 m/s² for 8 s | 82 / 0 / 0 | 0.0 / 8.8 / 0.0 | 0 % | 34.5 mm | 0.50 |
| braking, −7 m/s² | 153 / 0 / 0 | 0.0 / 8.9 / 0.0 | 0 % | 4.9 mm | 0.91 |
| lane change, ±3 m/s² at 0.33 Hz | 0 / 111 / 0 | 5.2 / 0.1 / 2.7 | 0 % | 44.6 mm | 1.38 |
| cornering, 4 m/s² for 6 s | 0 / 105 / 0 | 11.7 / 0.0 / 2.9 | 0 % | 0.5 mm | 0.58 |
| road bump, heave up to 3.7 m/s² | 0 / 0 / 70 | 0.0 / 0.9 / 0.0 | 0 % | 47.7 mm | 0.80 |
| excessive, 3 g on x/y/z | 153 / 164 / 97 | 17.1 / 12.3 / 17.8 | 58 % | 0.0 mm (never past) | — |

The lane change scores worse than not moving, and no tuning changes that. At 0.33 Hz the cue needs ±0.35 m of sway against ±0.21 m available, and the 3°/s rate limit allows only about ±1.4° of tilt (SPEC §6.3a).

![Launch](results/plots/launch.png)
![Cornering](results/plots/cornering.png)
![Excessive cue](results/plots/excessive.png)

Plots for all six scenarios are in [results/plots/](results/plots/).

## Reference geometry

The geometry lives in `config/platform.json`. It is the single source of truth for the C header and the test vectors.
- **Dimensions:** base radius 0.80 m, platform radius 0.50 m, neutral height 0.70 m, stroke 0.25 m.
- **Leg length:** 0.787 m at neutral, 27° from vertical.
- **Single-axis reach:** surge +0.26 / −0.21 m, sway ±0.21 m, heave ±0.13 m, roll ±14°, pitch +17 / −16°, yaw ±27°.
- **Load:** a 500 kg payload puts 919 N on each leg at neutral.

The 30° platform half-angle was chosen by comparing workspace and Jacobian conditioning (SPEC §3).

## Run it (WSL2 Ubuntu, same setup as the S1 trainer firmware)

```bash
bash scripts/test-unit.sh     # ztest unit tests on native_sim
bash scripts/crosscheck.sh    # headers current + C vs NumPy (kinematics) and SciPy (washout)
```

```bash
cd tools/reference
python3 closed_loop.py                     # all scenarios → results/closed_loop.md, results/plots/
python3 closed_loop.py --set force_gain=0.6 --no-plots   # try a parameter (prints only)
python3 tune.py                            # grid search of the washout parameters (~7 min)
python3 scenarios.py                       # list the scenarios
python3 gen_headers.py                     # after changing config/platform.json
python3 geometry_check.py --compare 10 20 30 40
```

## Layout

```
config/platform.json    geometry, payload, limits, washout (single source of truth)
mcue/                   portable C library: include/mcue/*.h, src/*.c, generated platform_config.h
tests/unit/             ztest suites (linalg, kin, workspace, filters, washout, shaping) + generated vectors
tools/reference/        NumPy/SciPy references, ctypes bindings, scenarios, closed loop, tuning, header generator
results/                closed-loop table and plots (generated by closed_loop.py)
scripts/                test and cross-check helpers (bind-mount the repo in WSL: path has a space)
```
