/*
 * SPEC F04, P04, P05: classical washout. Step responses are compared with the
 * closed-form continuous responses (all channels use zeta = 1 by default).
 */
#include <math.h>
#include <zephyr/ztest.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "mcue/kin.h"
#include "mcue/washout.h"

#define DEG (M_PI / 180.0)

static mc_washout_params P;

static void *setup(void)
{
	mc_washout_default_params(&P);
	return NULL;
}

ZTEST_SUITE(washout, NULL, setup, NULL, NULL, NULL);

static const double rest_f[3] = {0.0, 0.0, MC_GRAVITY};
static const double no_rate[3] = {0.0, 0.0, 0.0};

/* x(t) for a force step A through k s / ((s + b)(s + n)^2), b != n */
static double trans_step(double A, double b, double n, double t)
{
	double Pc = 1.0 / ((n - b) * (n - b));
	double R = 1.0 / (b - n);

	return A * ((Pc * exp(-b * t)) - (Pc * exp(-n * t)) + (R * t * exp(-n * t)));
}

ZTEST(washout, test_defaults_are_stable)
{
	mc_washout w;

	zassert_true(mc_washout_init(&w, &P));
	zassert_within(P.tilt_rate_limit, 3.0 * DEG, 1e-15);
	zassert_within(P.tilt_accel_limit, 20.0 * DEG, 1e-15);
}

ZTEST(washout, test_rest_is_neutral)
{
	mc_washout w;
	mc_pose q;

	zassert_true(mc_washout_init(&w, &P));
	for (int k = 0; k < 5000; k++) {
		mc_washout_step(&w, rest_f, no_rate, &q);
	}
	const double *v = &q.x;

	for (int i = 0; i < 6; i++) {
		zassert_within(v[i], 0.0, 1e-12, "axis %d moves at rest", i);
	}
}

/* P05: sustained 3 m/s^2 surge step */
ZTEST(washout, test_surge_step_washes_out)
{
	mc_washout w;
	mc_pose q;
	const double a = 3.0;
	const double A = P.force_gain * a; /* below the soft-limit knee: unchanged */
	const double f[3] = {a, 0.0, MC_GRAVITY};
	double peak = 0.0, ref_peak = 0.0, err = 0.0, late = 0.0, max_rate = 0.0, prev_pitch = 0.0;

	zassert_true(P.surge_sway_zeta == 1.0, "closed form assumes zeta = 1");
	zassert_true(mc_washout_init(&w, &P));
	for (int k = 0; k < (int)(10.0 * P.rate_hz); k++) {
		mc_washout_step(&w, f, no_rate, &q);
		/* the Tustin trapezoid sees the step half a sample early */
		double t = (k + 0.5) / P.rate_hz;
		double ref = trans_step(A, P.surge_sway_wb, P.surge_sway_wn, t);

		err = fmax(err, fabs(q.x - ref));
		peak = fmax(peak, q.x);
		ref_peak = fmax(ref_peak, ref);
		if (t >= 6.0) {
			late = fmax(late, fabs(q.x));
		}
		max_rate = fmax(max_rate, fabs(q.pitch - prev_pitch) * P.rate_hz);
		prev_pitch = q.pitch;
	}
	TC_PRINT("surge step: peak %.1f mm, |x| after 6 s <= %.2f mm, model error %.2e m, "
		 "pitch %.3f deg, max tilt rate %.4f deg/s\n",
		 peak * 1e3, late * 1e3, err, q.pitch / DEG, max_rate / DEG);
	zassert_true(err < 1e-5, "differs from the closed form by %g m", err);
	zassert_within(peak, ref_peak, 1e-5);
	/* the peak must leave room in the surge workspace (-213 mm, SPEC §3) */
	zassert_true(peak < 0.15, "peak %g m", peak);
	zassert_true(late < 0.005, "P05: still %g m from neutral after 6 s", late);
	/* tilt holds the sustained cue (P05) at the rate limit (P04) */
	zassert_within(q.pitch, -asin(A / MC_GRAVITY), 1e-6);
	zassert_true(max_rate <= P.tilt_rate_limit * (1.0 + 1e-9));
}

/* The tilt must reproduce the cue's direction: gravity seen in the platform frame. */
ZTEST(washout, test_tilt_signs_reproduce_the_force)
{
	static const double cues[][2] = {{3.0, 0.0}, {-2.0, 0.0}, {0.0, 2.5}, {0.0, -1.5}, {2.0, 2.0}};

	for (size_t c = 0; c < ARRAY_SIZE(cues); c++) {
		mc_washout w;
		mc_pose q;
		double R[3][3];
		const double f[3] = {cues[c][0], cues[c][1], MC_GRAVITY};

		zassert_true(mc_washout_init(&w, &P));
		for (int k = 0; k < (int)(15.0 * P.rate_hz); k++) {
			mc_washout_step(&w, f, no_rate, &q);
		}
		mc_rotation(&q, R);
		/* felt specific force = R^T (0, 0, g) */
		double fx = R[2][0] * MC_GRAVITY;
		double fy = R[2][1] * MC_GRAVITY;

		zassert_within(fx, P.force_gain * f[0], 0.02 * fabs(f[0]) + 1e-9, "cue %zu x", c);
		zassert_within(fy, P.force_gain * f[1], 0.02 * fabs(f[1]) + 1e-9, "cue %zu y", c);
	}
}

/* P04 under hard reversals and an extreme (soft-limited) cue */
ZTEST(washout, test_tilt_rate_never_exceeds_limit)
{
	mc_washout w;
	mc_pose q;
	double prev[2] = {0.0, 0.0}, prev_rate[2] = {0.0, 0.0}, worst = 0.0, worst_acc = 0.0;

	zassert_true(mc_washout_init(&w, &P));
	for (int k = 0; k < (int)(30.0 * P.rate_hz); k++) {
		double t = k / P.rate_hz;
		double sx = (fmod(t, 8.0) < 4.0) ? 9.0 : -9.0;
		const double f[3] = {sx, 7.0 * sin(1.3 * t), MC_GRAVITY};

		mc_washout_step(&w, f, no_rate, &q);
		double rate[2] = {(q.roll - prev[0]) * P.rate_hz, (q.pitch - prev[1]) * P.rate_hz};

		for (int a = 0; a < 2; a++) {
			worst = fmax(worst, fabs(rate[a]));
			worst_acc = fmax(worst_acc, fabs(rate[a] - prev_rate[a]) * P.rate_hz);
			prev_rate[a] = rate[a];
		}
		prev[0] = q.roll;
		prev[1] = q.pitch;
	}
	TC_PRINT("reversals: max tilt rate %.6f deg/s, max tilt acceleration %.4f deg/s^2\n",
		 worst / DEG, worst_acc / DEG);
	zassert_true(worst <= P.tilt_rate_limit * (1.0 + 1e-9), "%g deg/s", worst / DEG);
	/* the rate itself is smooth: no steps, which the legs would turn into a jolt */
	zassert_true(worst_acc <= P.tilt_accel_limit * (1.0 + 1e-6), "%g deg/s^2", worst_acc / DEG);
	/* soft limit: never more than force_gain * force_limit of cue */
	zassert_true(fabs(q.pitch) <= asin(P.force_gain * P.force_limit / MC_GRAVITY) + 1e-12);
}

/* Heave: f_z above g is an upward cue; gravity itself is not. */
ZTEST(washout, test_heave_step)
{
	mc_washout w;
	mc_pose q;
	const double a = 2.0;
	const double f[3] = {0.0, 0.0, MC_GRAVITY + a};
	double err = 0.0, late = 0.0;

	zassert_true(mc_washout_init(&w, &P));
	for (int k = 0; k < (int)(10.0 * P.rate_hz); k++) {
		mc_washout_step(&w, f, no_rate, &q);
		double t = (k + 0.5) / P.rate_hz;

		err = fmax(err, fabs(q.z - trans_step(P.force_gain * a, P.heave_wb, P.heave_wn, t)));
		if (t >= 6.0) {
			late = fmax(late, fabs(q.z));
		}
	}
	zassert_true(err < 1e-5, "%g m", err);
	zassert_true(late < 0.005);
	zassert_within(q.roll, 0.0, 1e-15);
	zassert_within(q.pitch, 0.0, 1e-15);
}

/* Sustained yaw rate: angle = k r0 t e^{-wn t} (zeta = 1), back to neutral. */
ZTEST(washout, test_yaw_rate_washes_out)
{
	mc_washout w;
	mc_pose q;
	const double r0 = 0.2;
	const double rate[3] = {0.0, 0.0, r0};
	double err = 0.0, late = 0.0;

	zassert_true(P.rot_zeta == 1.0);
	zassert_true(mc_washout_init(&w, &P));
	for (int k = 0; k < (int)(20.0 * P.rate_hz); k++) {
		mc_washout_step(&w, rest_f, rate, &q);
		/* rectangle integration of the filtered rate: one sample ahead of the trapezoid */
		double t = (k + 1.0) / P.rate_hz;
		double ref = P.rot_gain * r0 * t * exp(-P.rot_wn * t);

		err = fmax(err, fabs(q.yaw - ref));
		if (t >= 15.0) {
			late = fmax(late, fabs(q.yaw));
		}
	}
	TC_PRINT("yaw rate step: model error %.2e rad, |yaw| after 15 s <= %.4f deg\n", err,
		 late / DEG);
	zassert_true(err < 2e-4, "%g rad", err);
	zassert_true(late < 0.1 * DEG);
}

/*
 * A bend: sustained sway (held by roll tilt) with a sustained yaw rate. After
 * it, every angle must wash back to neutral; yaw rate must not leak through the
 * tilt into a lasting pitch offset (a false surge cue).
 */
ZTEST(washout, test_bend_leaves_no_residual_angle)
{
	mc_washout w;
	mc_pose q;
	double worst_pitch = 0.0;

	zassert_true(mc_washout_init(&w, &P));
	for (int k = 0; k < (int)(30.0 * P.rate_hz); k++) {
		double t = k / P.rate_hz;
		bool in_bend = (t >= 1.0) && (t < 9.0);
		const double f[3] = {0.0, in_bend ? 4.0 : 0.0, MC_GRAVITY};
		const double rate[3] = {0.0, 0.0, in_bend ? 0.2 : 0.0};

		mc_washout_step(&w, f, rate, &q);
		worst_pitch = fmax(worst_pitch, fabs(q.pitch));
	}
	TC_PRINT("bend: max |pitch| %.3f deg, after 21 s roll %.4f pitch %.4f yaw %.4f deg\n",
		 worst_pitch / DEG, q.roll / DEG, q.pitch / DEG, q.yaw / DEG);
	zassert_true(fabs(q.roll) < 0.01 * DEG);
	zassert_true(fabs(q.pitch) < 0.01 * DEG, "residual pitch %g deg", q.pitch / DEG);
	zassert_true(fabs(q.yaw) < 0.01 * DEG);
	zassert_true(worst_pitch < 0.01 * DEG, "yaw leaked into pitch: %g deg", worst_pitch / DEG);
}

ZTEST(washout, test_reset_clears_state)
{
	mc_washout w;
	mc_pose q;
	const double f[3] = {4.0, -3.0, MC_GRAVITY + 1.0};
	const double rate[3] = {0.1, -0.1, 0.3};

	zassert_true(mc_washout_init(&w, &P));
	for (int k = 0; k < 1000; k++) {
		mc_washout_step(&w, f, rate, &q);
	}
	mc_washout_reset(&w);
	mc_washout_step(&w, rest_f, no_rate, &q);
	const double *v = &q.x;

	for (int i = 0; i < 6; i++) {
		zassert_within(v[i], 0.0, 0.0);
	}
}
