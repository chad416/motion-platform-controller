#include "mcue/servo.h"

#include <math.h>
#include <string.h>

#include "mcue/platform_config.h"

void mc_servo_design(mc_servo_params *sp, const mc_plant_params *pp, double rate_hz,
		     double pos_bw_hz, double vel_bw_hz, double integral_ratio, double a_brake)
{
	sp->a_brake = a_brake;
	double r = pp->lead / (2.0 * MC_PI);
	double wv = 2.0 * MC_PI * vel_bw_hz;

	sp->accel_per_amp = (r * pp->kt) / mc_plant_inertia(pp);
	sp->amp_per_newton = r / (pp->efficiency * pp->kt);
	sp->kp_pos = 2.0 * MC_PI * pos_bw_hz;
	sp->kp_vel = wv / sp->accel_per_amp;
	sp->ki_vel = sp->kp_vel * integral_ratio * wv;
	sp->i_max = pp->i_max;
	sp->dt = 1.0 / rate_hz;
}

void mc_servo_default_params(mc_servo_params *sp, const mc_plant_params *pp)
{
	mc_servo_design(sp, pp, MC_CFG_SRV_RATE_HZ, MC_CFG_SRV_POSITION_BW_HZ,
			MC_CFG_SRV_VELOCITY_BW_HZ, MC_CFG_SRV_VELOCITY_INTEGRAL_RATIO,
			MC_CFG_SRV_POSITION_BRAKE_M_S2);
}

void mc_servo_reset(mc_servo *s, const mc_servo_params *p, double pos)
{
	(void)memset(s, 0, sizeof(*s));
	s->p = *p;
	s->prev_pos = pos;
}

/*
 * A NaN or infinite input (or result) gives no current, clears the integral
 * and sets `invalid`. Without this, fmin/fmax would turn a NaN into +i_max
 * with `saturated` false, and the integral would latch at the limit.
 */
static double invalid_step(mc_servo *s, double pos)
{
	s->invalid = true;
	s->saturated = false;
	s->integ = 0.0;
	s->vel = 0.0;
	if (isfinite(pos)) {
		s->prev_pos = pos;
	}
	s->current = 0.0;
	return 0.0;
}

double mc_servo_step(mc_servo *s, double pos_sp, double vel_sp, double acc_sp, double force_ff,
		     double pos)
{
	const mc_servo_params *p = &s->p;

	if (!isfinite(pos_sp) || !isfinite(vel_sp) || !isfinite(acc_sp) || !isfinite(force_ff) ||
	    !isfinite(pos) || !isfinite(s->prev_pos)) {
		return invalid_step(s, pos);
	}
	s->vel = (pos - s->prev_pos) / p->dt;
	s->prev_pos = pos;

	double e = pos_sp - pos;
	double v_fb = fmin(p->kp_pos * fabs(e), sqrt(2.0 * p->a_brake * fabs(e)));
	double v_cmd = vel_sp + ((e >= 0.0) ? v_fb : -v_fb);
	double ev = v_cmd - s->vel;
	double ff = (acc_sp / p->accel_per_amp) + (p->amp_per_newton * force_ff);
	double i_try = (p->kp_vel * ev) + s->integ + ff;

	/* anti-windup: integrate unless that would push further into the limit */
	bool high = (i_try >= p->i_max) && (ev > 0.0);
	bool low = (i_try <= -p->i_max) && (ev < 0.0);

	if (!high && !low) {
		s->integ += p->ki_vel * ev * p->dt;
		s->integ = fmax(-p->i_max, fmin(p->i_max, s->integ));
	}

	double i = (p->kp_vel * ev) + s->integ + ff;

	if (!isfinite(i)) {
		return invalid_step(s, pos);
	}
	s->invalid = false;
	s->saturated = fabs(i) >= p->i_max;
	i = fmax(-p->i_max, fmin(p->i_max, i));
	s->current = i;
	return i;
}
