/*
 * SPEC F06, F07, P02: actuator model and leg servo (§9.1 "servo + plant").
 */
#include <math.h>
#include <zephyr/ztest.h>

#include "mcue/geometry.h"
#include "mcue/kin.h"
#include "mcue/plant.h"
#include "mcue/servo.h"
#include "mcue/shaping.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static mc_geometry G;
static mc_plant_params PP;
static mc_servo_params SP;
static double F_NEUTRAL; /* gravity load per leg at neutral, N */

static void *setup(void)
{
	mc_geometry_params gp;
	double f[MC_LEGS];
	const mc_pose zero = {0};
	const mc_vec3 com = {0.0, 0.0, 0.3};

	mc_geometry_default_params(&gp);
	zassert_true(mc_geometry_build(&G, &gp));
	mc_plant_default_params(&PP, &G);
	mc_servo_default_params(&SP, &PP);
	zassert_true(mc_statics(&G, &zero, 500.0, &com, f));
	F_NEUTRAL = f[0];
	TC_PRINT("plant: J %.3e kg m^2, load mass %.1f kg/leg, G %.3f m/s^2/A; "
		 "servo Kp_pos %.1f 1/s, Kp_vel %.1f A/(m/s), Ki_vel %.0f A/m\n",
		 mc_plant_inertia(&PP), PP.load_mass, SP.accel_per_amp, SP.kp_pos, SP.kp_vel,
		 SP.ki_vel);
	return NULL;
}

ZTEST_SUITE(servo, NULL, setup, NULL, NULL, NULL);

ZTEST(servo, test_unpowered_leg_rests_on_the_lower_stop)
{
	mc_plant pl;

	mc_plant_init(&pl, &PP, PP.stop_lo + 0.002);
	for (int k = 0; k < 2000; k++) {
		mc_plant_step(&pl, 0.0, F_NEUTRAL);
	}
	zassert_within(mc_plant_length(&pl), PP.stop_lo, 1e-15);
	zassert_within(mc_plant_speed(&pl), 0.0, 1e-15);
	zassert_within(mc_plant_encoder(&pl), PP.stop_lo, 1e-15);
}

/* Without friction or load, a constant current gives w = Kt i t / J. */
ZTEST(servo, test_free_acceleration_matches_closed_form)
{
	mc_plant_params p = PP;
	mc_plant pl;
	const double i = 2.0;

	p.viscous = 0.0;
	p.coulomb = 0.0;
	mc_plant_init(&pl, &p, G.l0);
	for (int k = 0; k < 20; k++) {
		mc_plant_step(&pl, i, 0.0);
	}
	double t = 20 * p.dt;
	double w = (p.kt * i * t) / mc_plant_inertia(&p);
	double r = p.lead / (2.0 * M_PI);

	zassert_within(mc_plant_speed(&pl), w * r, 1e-12 * w * r);
	/* semi-implicit Euler: x = a h^2 n (n + 1) / 2 over n sub-steps */
	int n = 20 * p.substeps;
	double h = p.dt / p.substeps;
	double a = ((p.kt * i) / mc_plant_inertia(&p)) * r;

	zassert_within(mc_plant_length(&pl) - G.l0, a * h * h * n * (n + 1) / 2.0, 1e-12);
}

/* The current that balances the load holds the leg still. */
ZTEST(servo, test_balance_current_holds_the_load)
{
	mc_plant pl;
	double i_hold = SP.amp_per_newton * F_NEUTRAL;

	mc_plant_init(&pl, &PP, G.l0);
	for (int k = 0; k < 1000; k++) {
		mc_plant_step(&pl, i_hold, F_NEUTRAL);
	}
	TC_PRINT("holding %.0f N needs %.3f A\n", F_NEUTRAL, i_hold);
	zassert_within(mc_plant_length(&pl), G.l0, 1e-9);
}

ZTEST(servo, test_current_limit_and_encoder)
{
	mc_plant pl;

	mc_plant_init(&pl, &PP, G.l0);
	mc_plant_step(&pl, 1000.0, 0.0);
	zassert_within(pl.current, PP.i_max, 0.0);
	mc_plant_step(&pl, -1000.0, 0.0);
	zassert_within(pl.current, -PP.i_max, 0.0);

	double res = PP.lead / PP.counts_per_rev;

	for (int k = 0; k < 100; k++) {
		mc_plant_init(&pl, &PP, G.l_min + (k * 0.00247));
		double e = mc_plant_length(&pl) - mc_plant_encoder(&pl);

		zassert_true((e >= -1e-12) && (e < res + 1e-12), "encoder error %g", e);
	}
}

struct step_result {
	double overshoot; /* fraction of the step */
	double settle;    /* s, last time outside +-2 % */
	double i_peak;
};

/* A raw position step (no shaping) under the full gravity load. */
static struct step_result run_step(double step, bool gravity_ff)
{
	mc_plant pl;
	mc_servo s;
	struct step_result r = {0};
	double l = G.l0;
	double ff = gravity_ff ? F_NEUTRAL : 0.0;

	mc_plant_init(&pl, &PP, l);
	mc_servo_reset(&s, &SP, mc_plant_encoder(&pl));
	/* settle the hold first */
	for (int k = 0; k < 1000; k++) {
		double i = mc_servo_step(&s, l, 0.0, 0.0, ff, mc_plant_encoder(&pl));

		mc_plant_step(&pl, i, F_NEUTRAL);
	}
	for (int k = 0; k < 500; k++) {
		double i = mc_servo_step(&s, l + step, 0.0, 0.0, ff, mc_plant_encoder(&pl));

		zassert_true(fabs(i) <= PP.i_max);
		r.i_peak = fmax(r.i_peak, fabs(i));
		mc_plant_step(&pl, i, F_NEUTRAL);

		double x = (mc_plant_length(&pl) - l) / step;

		r.overshoot = fmax(r.overshoot, x - 1.0);
		if (fabs(x - 1.0) > 0.02) {
			r.settle = (k + 1) * SP.dt;
		}
	}
	return r;
}

ZTEST(servo, test_10mm_step_settles_within_100ms)
{
	for (int dir = -1; dir <= 1; dir += 2) {
		struct step_result r = run_step(dir * 0.010, true);

		TC_PRINT("10 mm step %s: overshoot %.2f %%, settled (2 %%) after %.0f ms, "
			 "peak current %.1f A\n",
			 (dir > 0) ? "up" : "down", r.overshoot * 100.0, r.settle * 1e3, r.i_peak);
		zassert_true(r.overshoot < 0.05, "overshoot %g", r.overshoot);
		zassert_true(r.settle <= 0.100, "settle %g s", r.settle);
	}
}

/* A large step saturates the drive: the current stays at the limit, no windup. */
ZTEST(servo, test_saturated_step_respects_the_limit)
{
	struct step_result r = run_step(0.100, true);

	TC_PRINT("100 mm step: overshoot %.2f %%, settled after %.0f ms, peak %.1f A\n",
		 r.overshoot * 100.0, r.settle * 1e3, r.i_peak);
	zassert_within(r.i_peak, PP.i_max, 1e-12);
	zassert_true(r.overshoot < 0.05, "windup overshoot %g", r.overshoot);
}

/* Without gravity feed-forward the integral alone must remove the sag. */
ZTEST(servo, test_static_hold_under_full_load)
{
	mc_plant pl;
	mc_servo s;
	double worst = 0.0;

	mc_plant_init(&pl, &PP, G.l0);
	mc_servo_reset(&s, &SP, mc_plant_encoder(&pl));
	for (int k = 0; k < 2000; k++) {
		/* full load applied at once at t = 0 */
		double i = mc_servo_step(&s, G.l0, 0.0, 0.0, 0.0, mc_plant_encoder(&pl));

		mc_plant_step(&pl, i, F_NEUTRAL);
		if (k >= 500) {
			worst = fmax(worst, fabs(mc_plant_length(&pl) - G.l0));
		}
	}
	TC_PRINT("hold %.0f N with no feed-forward: error after 0.5 s <= %.4f mm\n", F_NEUTRAL,
		 worst * 1e3);
	zassert_true(worst < 0.0001, "hold error %g m", worst);
}

/* A full-stroke S-curve at the leg limits, with feed-forward: P02-level tracking. */
ZTEST(servo, test_s_curve_tracking)
{
	mc_plant pl;
	mc_servo s;
	mc_leg_traj tr;
	double a[MC_LEGS], b[MC_LEGS], p[MC_LEGS], v[MC_LEGS], acc[MC_LEGS];
	double worst = 0.0, sum2 = 0.0;
	int n = 0;

	for (int i = 0; i < MC_LEGS; i++) {
		a[i] = G.soft_lo;
		b[i] = G.soft_hi;
	}
	mc_traj_init(&tr, a, b, 0.0, 0.5, 6.0, SP.dt);
	TC_PRINT("full stroke S-curve: %.3f s\n", tr.T);
	mc_plant_init(&pl, &PP, G.soft_lo);
	mc_servo_reset(&s, &SP, mc_plant_encoder(&pl));
	for (int k = 0; k < 200; k++) {
		mc_plant_step(&pl, mc_servo_step(&s, a[0], 0, 0, F_NEUTRAL, mc_plant_encoder(&pl)),
			      F_NEUTRAL);
	}
	bool done = false;

	while (!done) {
		done = mc_traj_step(&tr, p, v, acc);
		double e = p[0] - mc_plant_length(&pl);

		worst = fmax(worst, fabs(e));
		sum2 += e * e;
		n++;
		double i = mc_servo_step(&s, p[0], v[0], acc[0], F_NEUTRAL, mc_plant_encoder(&pl));

		zassert_true(fabs(v[0]) <= 0.5 * (1.0 + 1e-9));
		zassert_true(fabs(acc[0]) <= 6.0 * (1.0 + 1e-9));
		mc_plant_step(&pl, i, F_NEUTRAL);
	}
	TC_PRINT("tracking: rms %.4f mm, peak %.4f mm\n", sqrt(sum2 / n) * 1e3, worst * 1e3);
	zassert_true(worst < 0.0005, "peak %g m", worst);
}
