/*
 * Leg trajectory shaping (SPEC F05) and engage/park S-curves (F08).
 *
 * The limiter moves an output towards a target with |v| <= v_max and
 * |a| <= a_max, which always hold, and never past lo / hi (the soft limits)
 * when the targets are inside them (SPEC P03). It has two modes:
 *
 *   tracking  output = target, when that is within v_max / a_max (relative to
 *             the target's own velocity too) and the output could still stop
 *             before lo / hi from there; no lag, no chatter
 *   braking   towards the target at the largest speed from which it can still
 *             stop at the target (exact for the discrete update), so a fixed
 *             target is reached in near-minimum time without overshoot
 *
 * A target that jumps closer than the braking distance (v^2 / 2 a_max, 21 mm at
 * full leg speed) is overshot - the limits win - but never past lo / hi.
 */
#ifndef MCUE_SHAPING_H_
#define MCUE_SHAPING_H_

#include "mcue/geometry.h"

typedef struct {
	double v_max, a_max;
	double lo, hi; /* hard bounds for the output (soft limits); +-HUGE_VAL for none */
	double dt;
} mc_limit2_params;

/*
 * One step on a scalar. *prev_target holds the last target (for its velocity):
 * start it at the initial position. Returns the applied acceleration.
 * Used for the legs and for the washout's tilt coordination.
 */
double mc_limit2_step(double *pos, double *vel, double *prev_target, double target,
		      const mc_limit2_params *lp);

typedef struct {
	double pos[MC_LEGS];
	double vel[MC_LEGS];
	double acc[MC_LEGS];  /* last applied, for tests and status */
	double prev[MC_LEGS]; /* last target */
	mc_limit2_params lim;
} mc_leg_shaper;

void mc_shaper_init(mc_leg_shaper *s, const double start[MC_LEGS], const mc_limit2_params *lp);

/* One step towards target; s->pos is the new setpoint. */
void mc_shaper_step(mc_leg_shaper *s, const double target[MC_LEGS]);

/*
 * Quintic S-curve (minimum-jerk shape): fraction 0..1 over duration T with
 * zero velocity and acceleration at both ends. Peak rate is 1.875 / T.
 */
double mc_scurve(double t, double T);

#endif /* MCUE_SHAPING_H_ */
