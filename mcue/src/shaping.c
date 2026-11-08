#include "mcue/shaping.h"

#include <math.h>
#include <stdbool.h>

static double clamp(double x, double lim)
{
	return (x > lim) ? lim : ((x < -lim) ? -lim : x);
}

/*
 * Distance needed to stop from speed v braking at a every step, with
 * p += v dt after the velocity update: v^2 / (2 a) + v dt / 2 when v is a
 * whole number of a dt steps, up to a dt^2 / 8 more otherwise. This bound
 * is never below the true distance.
 */
static double stop_distance(double v, double a, double dt)
{
	double s = fabs(v);

	return ((s * s) / (2.0 * a)) + ((s * dt) / 2.0) + ((a * dt * dt) / 8.0);
}

double mc_limit2_step(double *pos, double *vel, double *prev_target, double target,
		      const mc_limit2_params *lp)
{
	const double dt = lp->dt;
	const double am = lp->a_max;
	double e = target - *pos;
	double v_target = (target - *prev_target) / dt;

	*prev_target = target;

	/* Tracking: land on the target this step if every limit allows it. */
	double v_land = e / dt;
	double a_land = (v_land - *vel) / dt;
	double reach = stop_distance(v_land, am, dt);
	bool can_stop = (v_land >= 0.0) ? ((target + reach) <= lp->hi) : ((target - reach) >= lp->lo);

	if ((fabs(v_land) <= lp->v_max) && (fabs(a_land) <= am) &&
	    ((fabs(v_land - v_target) / dt) <= am) && can_stop) {
		*vel = v_land;
		*pos = target;
		return a_land;
	}

	/*
	 * Braking: the largest speed from which the output can still stop within
	 * |e| (solve stop_distance(v) = |e| for v), so it never passes a fixed
	 * target, and therefore never passes lo / hi while the targets are inside.
	 */
	double dir = (e >= 0.0) ? 1.0 : -1.0;
	double room = fmax(0.0, fabs(e) - ((am * dt * dt) / 8.0));
	double v_brake = am * (-(dt / 2.0) + sqrt(((dt * dt) / 4.0) + ((2.0 * room) / am)));
	double a = clamp(((dir * fmin(lp->v_max, v_brake)) - *vel) / dt, am);

	*vel += a * dt;
	*pos += *vel * dt;
	return a;
}

void mc_shaper_init(mc_leg_shaper *s, const double start[MC_LEGS], const mc_limit2_params *lp)
{
	for (int i = 0; i < MC_LEGS; i++) {
		s->pos[i] = start[i];
		s->vel[i] = 0.0;
		s->acc[i] = 0.0;
		s->prev[i] = start[i];
	}
	s->lim = *lp;
}

void mc_shaper_step(mc_leg_shaper *s, const double target[MC_LEGS])
{
	for (int i = 0; i < MC_LEGS; i++) {
		s->acc[i] = mc_limit2_step(&s->pos[i], &s->vel[i], &s->prev[i], target[i], &s->lim);
	}
}

double mc_scurve(double t, double T)
{
	if (t <= 0.0) {
		return 0.0;
	}
	if (t >= T) {
		return 1.0;
	}
	double u = t / T;

	return u * u * u * (10.0 + (u * (-15.0 + (6.0 * u))));
}

double mc_limit2_stop_point(double pos, double vel, const mc_limit2_params *lp)
{
	double d = stop_distance(vel, lp->a_max, lp->dt);
	double p = pos + ((vel >= 0.0) ? d : -d);

	return fmax(lp->lo, fmin(lp->hi, p));
}

void mc_traj_init(mc_leg_traj *tr, const double start[MC_LEGS], const double end[MC_LEGS],
		  double t_min, double v_max, double a_max, double dt)
{
	double d = 0.0;

	for (int i = 0; i < MC_LEGS; i++) {
		tr->start[i] = start[i];
		tr->end[i] = end[i];
		d = fmax(d, fabs(end[i] - start[i]));
	}
	/*
	 * Quintic S-curve s(u) = 10u^3 - 15u^4 + 6u^5 over T, u = t/T:
	 *   peak ds/du   = 30 u^2 (1-u)^2 at u = 1/2               = 15/8
	 *   peak d2s/du2 = 60 u (1-u)(1-2u) at u = (3 - sqrt 3)/6   = 10/sqrt(3)
	 * so speed d*15/(8T) <= v_max and acceleration d*10/(sqrt(3) T^2) <= a_max.
	 */
	const double peak_vel = 15.0 / 8.0;
	const double peak_acc = 10.0 / sqrt(3.0);

	tr->T = fmax(t_min, fmax((peak_vel * d) / v_max, sqrt((peak_acc * d) / a_max)));
	tr->t = 0.0;
	tr->dt = dt;
}

bool mc_traj_step(mc_leg_traj *tr, double pos[MC_LEGS], double vel[MC_LEGS],
		  double acc[MC_LEGS])
{
	tr->t = fmin(tr->t + tr->dt, tr->T);

	double T = tr->T;
	double u = (T > 0.0) ? (tr->t / T) : 1.0;
	double s = mc_scurve(tr->t, T);
	double ds = (T > 0.0) ? ((30.0 * u * u * (1.0 - u) * (1.0 - u)) / T) : 0.0;
	double dds = (T > 0.0) ? ((60.0 * u * (1.0 - u) * (1.0 - (2.0 * u))) / (T * T)) : 0.0;

	for (int i = 0; i < MC_LEGS; i++) {
		double d = tr->end[i] - tr->start[i];

		pos[i] = tr->start[i] + (d * s);
		vel[i] = d * ds;
		acc[i] = d * dds;
	}
	return tr->t >= T;
}
