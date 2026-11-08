/*
 * Cascaded leg servo. SPEC F06, §6.4.
 *
 *   v_cmd = v_sp + sign(e) min(Kp_pos |e|, sqrt(2 a_brake |e|)),  e = l_sp - l
 *   i     = Kp_vel (v_cmd - v) + Ki_vel * integral(v_cmd - v) + i_ff
 *   i_ff  = a_sp / G + r F_ff / (eta Kt)       (acceleration + gravity)
 *
 * The square-root branch only acts on errors above 2 a_brake / Kp_pos^2
 * (2.7 mm): a linear position loop would then ask for more deceleration than
 * the saturated drive gives, and overshoot. Normal tracking errors stay in
 * the linear branch.
 *
 * G = r Kt / J is the leg acceleration per amp (J from the plant model).
 * The velocity v is the backward difference of the encoder position. The
 * current is limited to i_max; the integral stops while the output is
 * saturated in the direction it would grow (anti-windup).
 *
 * Gains come from bandwidths: Kp_pos = w_p, Kp_vel = w_v / G,
 * Ki_vel = Kp_vel * ratio * w_v (PI zero at ratio * w_v).
 */
#ifndef MCUE_SERVO_H_
#define MCUE_SERVO_H_

#include <stdbool.h>

#include "mcue/plant.h"

typedef struct {
	double kp_pos;         /* 1/s */
	double a_brake;        /* m/s^2, square-root branch of the position loop */
	double kp_vel;         /* A/(m/s) */
	double ki_vel;         /* A/m */
	double accel_per_amp;  /* G, m/s^2 per A */
	double amp_per_newton; /* r / (eta Kt), A per N of axial load */
	double i_max;          /* A */
	double dt;             /* s */
} mc_servo_params;

typedef struct {
	mc_servo_params p;
	double integ;    /* integral term, A */
	double prev_pos; /* last measured position */
	double vel;      /* velocity estimate, m/s */
	double current;  /* last output, A */
	bool saturated;  /* last output was at the limit */
	bool invalid;    /* last step had a NaN / infinite input or result: output 0 A */
} mc_servo;

/* Design the gains for a plant model. Bandwidths in Hz, rate in Hz. */
void mc_servo_design(mc_servo_params *sp, const mc_plant_params *pp, double rate_hz,
		     double pos_bw_hz, double vel_bw_hz, double integral_ratio, double a_brake);

/* Gains from platform.json for the default plant. */
void mc_servo_default_params(mc_servo_params *sp, const mc_plant_params *pp);

/* Start (or restart) at the measured position: integral cleared. */
void mc_servo_reset(mc_servo *s, const mc_servo_params *p, double pos);

/*
 * One servo period. Returns the current command (limited). A non-finite input
 * or result returns 0 A, clears the integral and sets s->invalid; the servo
 * bank turns that into an INTERNAL fault on the leg.
 */
double mc_servo_step(mc_servo *s, double pos_sp, double vel_sp, double acc_sp, double force_ff,
		     double pos);

#endif /* MCUE_SERVO_H_ */
