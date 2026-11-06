/*
 * Classical washout motion cueing (Reid-Nahon structure). SPEC F04, P04, P05.
 *
 * Input: vehicle specific force f (m/s^2, includes +g on z at rest) and angular
 * rate w (rad/s, body frame) at the driver's head. Output: platform pose
 * command (before workspace limiting).
 *
 *   translational: k_f * softlim(f), gravity removed on z, then the displacement
 *                  filter k s / ((s + wb)(s^2 + 2 z wn s + wn^2)): the double
 *                  integral of a 3rd-order high-pass, with no free integrator,
 *                  so it cannot drift
 *   tilt:          low-pass of the scaled surge/sway force, theta = -asin(fx/g),
 *                  phi = +asin(fy/g), rate- and acceleration-limited (a rate
 *                  limit alone steps the angular rate, which the legs turn
 *                  into a jolt)
 *   rotational:    k_r * w, 2nd-order high-pass, Euler-rate transform (on this
 *                  channel's own angles, not the tilt), integrate
 */
#ifndef MCUE_WASHOUT_H_
#define MCUE_WASHOUT_H_

#include <stdbool.h>

#include "mcue/filters.h"
#include "mcue/geometry.h"

typedef struct {
	double rate_hz;
	double force_gain, force_limit;
	double surge_sway_wn, surge_sway_zeta, surge_sway_wb;
	double heave_wn, heave_zeta, heave_wb;
	double tilt_wn, tilt_zeta;
	double tilt_rate_limit;  /* rad/s */
	double tilt_accel_limit; /* rad/s^2 */
	double rot_gain, rot_wn, rot_zeta;
} mc_washout_params;

typedef struct {
	mc_washout_params p;
	double dt;
	mc_biquad trans_hp1[3]; /* s/(s+wb), per axis */
	mc_biquad trans_res[3]; /* k/(s^2+2zwn s+wn^2) */
	mc_biquad tilt_lp[2];   /* surge, sway */
	mc_biquad rot_hp[3];    /* body rates */
	double tilt[2];         /* roll, pitch from tilt coordination (rate-limited) */
	double angle[3];        /* roll, pitch, yaw from the rotational channel */
	double tilt_rate[2];    /* tilt rate, rad/s (limiter state; also for tests and status) */
	double tilt_prev[2];    /* last tilt target (limiter state) */
} mc_washout;

void mc_washout_default_params(mc_washout_params *p);

/* Returns false if any filter would be unstable at this rate. */
bool mc_washout_init(mc_washout *w, const mc_washout_params *p);
void mc_washout_reset(mc_washout *w);

/* One cueing step. f: specific force xyz, w: body angular rate xyz. */
void mc_washout_step(mc_washout *w, const double f[3], const double rate[3], mc_pose *cmd);

#endif /* MCUE_WASHOUT_H_ */
