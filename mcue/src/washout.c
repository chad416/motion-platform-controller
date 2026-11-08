#include "mcue/washout.h"

#include <math.h>
#include <string.h>

#include "mcue/kin.h"
#include "mcue/platform_config.h"
#include "mcue/shaping.h"

void mc_washout_default_params(mc_washout_params *p)
{
	p->rate_hz = MC_CFG_WO_RATE_HZ;
	p->force_gain = MC_CFG_WO_FORCE_GAIN;
	p->force_limit = MC_CFG_WO_FORCE_LIMIT_M_S2;
	p->surge_sway_wn = MC_CFG_WO_SURGE_SWAY_WN;
	p->surge_sway_zeta = MC_CFG_WO_SURGE_SWAY_ZETA;
	p->surge_sway_wb = MC_CFG_WO_SURGE_SWAY_WB;
	p->heave_wn = MC_CFG_WO_HEAVE_WN;
	p->heave_zeta = MC_CFG_WO_HEAVE_ZETA;
	p->heave_wb = MC_CFG_WO_HEAVE_WB;
	p->tilt_wn = MC_CFG_WO_TILT_WN;
	p->tilt_zeta = MC_CFG_WO_TILT_ZETA;
	p->tilt_rate_limit = MC_CFG_WO_TILT_RATE_LIMIT_DEG_S * (MC_PI / 180.0);
	p->tilt_accel_limit = MC_CFG_WO_TILT_ACCEL_LIMIT_DEG_S2 * (MC_PI / 180.0);
	p->rot_gain = MC_CFG_WO_ROT_GAIN;
	p->rot_wn = MC_CFG_WO_ROT_WN;
	p->rot_zeta = MC_CFG_WO_ROT_ZETA;
}

bool mc_washout_init(mc_washout *w, const mc_washout_params *p)
{
	bool ok = true;
	double fs = p->rate_hz;

	(void)memset(w, 0, sizeof(*w));
	w->p = *p;
	w->dt = 1.0 / fs;

	for (int a = 0; a < 3; a++) {
		double wn = (a < 2) ? p->surge_sway_wn : p->heave_wn;
		double z = (a < 2) ? p->surge_sway_zeta : p->heave_zeta;
		double wb = (a < 2) ? p->surge_sway_wb : p->heave_wb;

		ok = ok && mc_biquad_hp1(&w->trans_hp1[a], wb, fs);
		/* k / (s^2 + 2 z wn s + wn^2); the gain k is applied at the input */
		ok = ok && mc_biquad_tustin(&w->trans_res[a], 0.0, 0.0, 1.0, 1.0, 2.0 * z * wn,
					    wn * wn, fs);
		ok = ok && mc_biquad_hp2(&w->rot_hp[a], p->rot_wn, p->rot_zeta, fs);
	}
	for (int a = 0; a < 2; a++) {
		ok = ok && mc_biquad_lp2(&w->tilt_lp[a], p->tilt_wn, p->tilt_zeta, fs);
	}
	return ok;
}

void mc_washout_reset(mc_washout *w)
{
	mc_washout_params p = w->p;

	(void)mc_washout_init(w, &p);
}

/*
 * Smooth saturation: exactly linear up to lim/2, then a tanh knee that meets
 * the line with slope 1 and approaches +-lim. Typical cues pass unchanged;
 * only extreme ones are compressed.
 */
static double softlim(double x, double lim)
{
	double knee = 0.5 * lim;
	double ax = fabs(x);

	if (ax <= knee) {
		return x;
	}
	double y = knee + ((lim - knee) * tanh((ax - knee) / (lim - knee)));

	return (x < 0.0) ? -y : y;
}

static double clamp(double x, double lo, double hi)
{
	return (x < lo) ? lo : ((x > hi) ? hi : x);
}

void mc_washout_step(mc_washout *w, const double f[3], const double rate[3], mc_pose *cmd)
{
	const mc_washout_params *p = &w->p;
	double fs[3];
	double pos[3];

	/* Scaled specific force; the translational channel works on f - g on z. */
	for (int a = 0; a < 3; a++) {
		double fa = (a == 2) ? (f[a] - MC_GRAVITY) : f[a];

		fs[a] = p->force_gain * softlim(fa, p->force_limit);
		pos[a] = mc_biquad_step(&w->trans_res[a], mc_biquad_step(&w->trans_hp1[a], fs[a]));
	}

	/* Tilt coordination: sustained surge/sway cues from gravity, rate-limited. */
	double fx_lp = mc_biquad_step(&w->tilt_lp[0], fs[0]);
	double fy_lp = mc_biquad_step(&w->tilt_lp[1], fs[1]);
	double target[2] = {
		asin(clamp(fy_lp / MC_GRAVITY, -1.0, 1.0)),  /* roll  = +asin(fy/g) */
		-asin(clamp(fx_lp / MC_GRAVITY, -1.0, 1.0)), /* pitch = -asin(fx/g) */
	};
	/* no angle bounds here: the workspace limiter after the washout owns those */
	const mc_limit2_params tilt_lim = {p->tilt_rate_limit, p->tilt_accel_limit, -HUGE_VAL, HUGE_VAL,
					   w->dt};

	for (int a = 0; a < 2; a++) {
		(void)mc_limit2_step(&w->tilt[a], &w->tilt_rate[a], &w->tilt_prev[a], target[a], &tilt_lim);
	}

	/* Rotational channel: high-passed body rates -> Euler rates -> angles. */
	double r[3];

	for (int a = 0; a < 3; a++) {
		r[a] = mc_biquad_step(&w->rot_hp[a], p->rot_gain * rate[a]);
	}

	/*
	 * Euler-rate transform on the rotational channel's own angles. The tilt
	 * is an artificial attitude: transforming with it included leaks yaw rate
	 * into pitch (theta_dot = q cos(phi) - r sin(phi)), which integrates into an
	 * offset the high-pass cannot remove - 0.8 deg of pitch, a false surge cue,
	 * after a 6 s bend in the closed loop.
	 */
	double phi = w->angle[0];
	double th = w->angle[1];
	double cphi = cos(phi), sphi = sin(phi), cth = cos(th);
	/* ZYX Euler rates from body rates (p, q, r) */
	double phi_dot = r[0] + (((r[1] * sphi) + (r[2] * cphi)) * tan(th));
	double th_dot = (r[1] * cphi) - (r[2] * sphi);
	double psi_dot = ((r[1] * sphi) + (r[2] * cphi)) / cth;

	w->angle[0] += phi_dot * w->dt;
	w->angle[1] += th_dot * w->dt;
	w->angle[2] += psi_dot * w->dt;

	cmd->x = pos[0];
	cmd->y = pos[1];
	cmd->z = pos[2];
	cmd->roll = w->angle[0] + w->tilt[0];
	cmd->pitch = w->angle[1] + w->tilt[1];
	cmd->yaw = w->angle[2];
}
