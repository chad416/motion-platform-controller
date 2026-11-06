/* SPEC F05, F08, P03: leg trajectory shaping and the engage/park S-curve. */
#include <math.h>
#include <zephyr/ztest.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "mcue/platform_config.h"
#include "mcue/shaping.h"

#define VMAX MC_CFG_LEG_SPEED_M_S
#define AMAX MC_CFG_LEG_ACCEL_M_S2
#define DT   (1.0 / MC_CFG_WO_RATE_HZ)

/* limits without bounds on the output */
static const mc_limit2_params FREE = {VMAX, AMAX, -1e9, 1e9, DT};

ZTEST_SUITE(shaping, NULL, NULL, NULL, NULL, NULL);

static uint32_t rng_state = 24681357U;

static double urand(double lo, double hi)
{
	rng_state ^= rng_state << 13;
	rng_state ^= rng_state >> 17;
	rng_state ^= rng_state << 5;
	return lo + ((hi - lo) * ((double)rng_state / 4294967295.0));
}

/* One step; checks the speed and acceleration limits on every leg. */
static void step_checked(mc_leg_shaper *s, const double target[MC_LEGS])
{
	double v_prev[MC_LEGS];

	for (int i = 0; i < MC_LEGS; i++) {
		v_prev[i] = s->vel[i];
	}
	mc_shaper_step(s, target);
	for (int i = 0; i < MC_LEGS; i++) {
		zassert_true(fabs(s->vel[i]) <= VMAX * (1.0 + 1e-12), "leg %d speed %g", i, s->vel[i]);
		zassert_true(fabs(s->vel[i] - v_prev[i]) / DT <= AMAX * (1.0 + 1e-9), "leg %d accel %g",
			     i, (s->vel[i] - v_prev[i]) / DT);
	}
}

/* Minimum time for a rest-to-rest move of d under VMAX, AMAX */
static double t_min(double d)
{
	d = fabs(d);
	return (d > (VMAX * VMAX / AMAX)) ? ((d / VMAX) + (VMAX / AMAX)) : (2.0 * sqrt(d / AMAX));
}

ZTEST(shaping, test_steps_arrive_in_near_minimum_time)
{
	static const double moves[MC_LEGS] = {0.2, -0.15, 0.03, -0.001, 1e-5, 0.0};
	const double start[MC_LEGS] = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8};
	double target[MC_LEGS];
	double overshoot = 0.0;
	int arrived[MC_LEGS];
	mc_leg_shaper s;

	mc_shaper_init(&s, start, &FREE);
	for (int i = 0; i < MC_LEGS; i++) {
		target[i] = start[i] + moves[i];
		arrived[i] = -1;
	}
	for (int k = 0; k < 1000; k++) {
		step_checked(&s, target);
		for (int i = 0; i < MC_LEGS; i++) {
			double dir = (moves[i] >= 0.0) ? 1.0 : -1.0;

			overshoot = fmax(overshoot, (s.pos[i] - target[i]) * dir);
			if ((arrived[i] < 0) && (s.pos[i] == target[i]) && (s.vel[i] == 0.0)) {
				arrived[i] = k + 1;
			}
		}
	}
	for (int i = 0; i < MC_LEGS; i++) {
		zassert_true(arrived[i] >= 0, "leg %d never settled exactly", i);
		double t = arrived[i] * DT;

		TC_PRINT("move %+.5f m: settled at %.3f s (minimum %.3f s)\n", moves[i], t,
			 t_min(moves[i]));
		zassert_true(t <= t_min(moves[i]) + (3.0 * DT), "leg %d slow: %g s", i, t);
		zassert_equal(s.pos[i], target[i]);
	}
	zassert_true(overshoot <= 1e-12, "overshoot %g m", overshoot);
}

/*
 * P03: a fixed target is never passed (beyond floating-point rounding, ~1e-17 m
 * seen in 2000 moves), so a target at a soft limit is never exceeded. Before the
 * a dt^2 / 8 braking correction the overshoot was up to 3 um.
 */
#define PASS_TOL 1e-12
ZTEST(shaping, test_random_rest_to_rest_moves_never_overshoot)
{
	for (int n = 0; n < 2000; n++) {
		const double start[MC_LEGS] = {0};
		double target[MC_LEGS];
		mc_leg_shaper s;

		for (int i = 0; i < MC_LEGS; i++) {
			target[i] = urand(-0.25, 0.25);
		}
		mc_shaper_init(&s, start, &FREE);
		for (int k = 0; k < 400; k++) { /* 0.8 s: longer than any 0.25 m move */
			step_checked(&s, target);
			for (int i = 0; i < MC_LEGS; i++) {
				double dir = (target[i] >= 0.0) ? 1.0 : -1.0;

				zassert_true((s.pos[i] - target[i]) * dir <= PASS_TOL, "move %d leg %d passed by %g m",
					     n, i, (s.pos[i] - target[i]) * dir);
			}
		}
		for (int i = 0; i < MC_LEGS; i++) {
			zassert_equal(s.pos[i], target[i], "move %d leg %d did not settle", n, i);
		}
	}
}

/*
 * A target moving within the limits is followed exactly, and the applied
 * acceleration is the target's own: no chatter from switching between braking
 * and landing (that chatter showed as force spikes in the closed loop).
 */
ZTEST(shaping, test_feasible_target_followed_exactly)
{
	const double start[MC_LEGS] = {0};
	double target[MC_LEGS];
	double worst_err = 0.0, worst_acc = 0.0;
	mc_leg_shaper s;

	mc_shaper_init(&s, start, &FREE);
	for (int k = 1; k <= 5000; k++) {
		double t = k * DT;

		for (int i = 0; i < MC_LEGS; i++) {
			/*
			 * 0.5 Hz, 50 mm, faded in over 1 s so it starts at rest: peak
			 * 0.16 m/s and about 1 m/s^2; its speed crosses a_max dt often.
			 */
			target[i] = mc_scurve(t, 1.0) * 0.05 * sin((2.0 * M_PI * 0.5 * t) + i);
		}
		step_checked(&s, target);
		for (int i = 0; i < MC_LEGS; i++) {
			worst_err = fmax(worst_err, fabs(s.pos[i] - target[i]));
			worst_acc = fmax(worst_acc, fabs(s.acc[i]));
		}
	}
	TC_PRINT("tracking 0.5 Hz: max error %.2e m, max applied accel %.3f m/s^2\n", worst_err,
		 worst_acc);
	zassert_true(worst_err < 1e-12, "lags by %g m", worst_err);
	zassert_true(worst_acc < 1.5, "chatter: %g m/s^2", worst_acc);
}

ZTEST(shaping, test_random_moving_targets_keep_limits)
{
	const double start[MC_LEGS] = {0};
	double target[MC_LEGS] = {0};
	mc_leg_shaper s;

	mc_shaper_init(&s, start, &FREE);
	for (int k = 0; k < 50000; k++) {
		if ((k % 23) == 0) { /* jumps, including ones inside the braking distance */
			for (int i = 0; i < MC_LEGS; i++) {
				target[i] = urand(-0.12, 0.12);
			}
		}
		step_checked(&s, target);
	}
}

/*
 * P03 with moving targets: a target that runs into a soft limit at speed and
 * stops dead there (as workspace scaling does) must not carry the leg past it.
 * Exact tracking at speed would overshoot by the braking distance (21 mm at
 * 0.5 m/s); the closed loop found this at 10.8 mm before the bound was added.
 */
ZTEST(shaping, test_targets_stopping_at_the_soft_limit_are_not_passed)
{
	const mc_limit2_params lim = {VMAX, AMAX, -0.115, 0.115, DT};
	double worst = -1.0;

	for (int n = 0; n < 300; n++) {
		const double start[MC_LEGS] = {0};
		double target[MC_LEGS] = {0};
		double speed[MC_LEGS];
		mc_leg_shaper s;

		for (int i = 0; i < MC_LEGS; i++) {
			speed[i] = urand(-VMAX, VMAX); /* within the speed limit: trackable */
		}
		mc_shaper_init(&s, start, &lim);
		for (int k = 0; k < 1500; k++) {
			if ((k % 200) == 199) { /* new random speed now and then */
				for (int i = 0; i < MC_LEGS; i++) {
					speed[i] = urand(-VMAX, VMAX);
				}
			}
			for (int i = 0; i < MC_LEGS; i++) {
				/* ramps that end abruptly at the limit, like a clamped command */
				target[i] = fmin(lim.hi, fmax(lim.lo, target[i] + (speed[i] * DT)));
			}
			step_checked(&s, target);
			for (int i = 0; i < MC_LEGS; i++) {
				worst = fmax(worst, fmax(s.pos[i] - lim.hi, lim.lo - s.pos[i]));
			}
		}
	}
	TC_PRINT("ramps into the soft limits: closest approach %.3f mm inside\n", -worst * 1e3);
	zassert_true(worst <= 0.0, "passed a soft limit by %g m", worst);
}

ZTEST(shaping, test_fast_sine_is_limited_not_unstable)
{
	const double start[MC_LEGS] = {0};
	double target[MC_LEGS];
	double worst = 0.0;
	mc_leg_shaper s;

	mc_shaper_init(&s, start, &FREE);
	for (int k = 0; k < 20000; k++) {
		for (int i = 0; i < MC_LEGS; i++) {
			target[i] = 0.12 * sin((2.0 * M_PI * (0.5 + i) * k * DT));
		}
		step_checked(&s, target);
		for (int i = 0; i < MC_LEGS; i++) {
			worst = fmax(worst, fabs(s.pos[i]));
		}
	}
	zassert_true(worst < 0.125, "output grew to %g m", worst);
}

ZTEST(shaping, test_scurve_shape)
{
	const double T = 3.0;
	double prev = 0.0, peak_rate = 0.0;

	zassert_equal(mc_scurve(-1.0, T), 0.0);
	zassert_equal(mc_scurve(0.0, T), 0.0);
	zassert_equal(mc_scurve(T, T), 1.0);
	zassert_equal(mc_scurve(T + 1.0, T), 1.0);
	zassert_within(mc_scurve(T / 2.0, T), 0.5, 1e-15);
	for (int k = 1; k <= 3000; k++) {
		double y = mc_scurve(k * (T / 3000.0), T);

		zassert_true(y >= prev, "not monotonic at %d", k);
		peak_rate = fmax(peak_rate, (y - prev) / (T / 3000.0));
		prev = y;
	}
	zassert_within(peak_rate, 1.875 / T, 1e-4);
	/* zero slope and curvature at both ends: tiny first steps */
	zassert_true(mc_scurve(0.01, T) < 1e-6);
	zassert_true(1.0 - mc_scurve(T - 0.01, T) < 1e-6);
}
