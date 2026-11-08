#include "mcue/ctl.h"

#include <math.h>
#include <string.h>

#include "mcue/kin.h"
#include "mcue/platform_config.h"
#include "mcue/workspace.h"

#define DEG (MC_PI / 180.0)

/* Comms loss: the washout is "back at neutral" inside these bounds. */
#define NEUTRAL_POS_M   0.005
#define NEUTRAL_ANG_RAD (0.5 * DEG)

/* Encoder plausibility at ENABLE: within the stroke, plus this much. */
#define HOMING_SLACK_M 0.001

/* RESET goes straight to PARKED only if every leg is this close to park. */
#define PARK_TOL_M 0.001

#define FK_ITER      6
#define FK_ITER_COLD 30
#define FK_TOL       1e-9

/* VEHICLE_F / VEHICLE_W rate (SPEC F09); the cueing loop interpolates between samples. */
#define VEHICLE_RATE_HZ 100.0

/*
 * The braking stop has ended when every leg is this close to rest at its
 * stop point. It must end within STOP_TIMEOUT_S (the P06 standstill bound);
 * past that the controller stops waiting (see the MC_PH_STOP case).
 */
#define STOP_POS_TOL_M   1e-5
#define STOP_VEL_TOL_M_S 1e-4
#define STOP_TIMEOUT_S   0.5

void mc_ctl_default_params(mc_ctl_params *p, const mc_geometry *g)
{
	p->engage_time = MC_CFG_SAF_ENGAGE_TIME_S;
	p->park_time = MC_CFG_SAF_PARK_TIME_S;
	p->stop_dwell = MC_CFG_SAF_STOP_DWELL_S;
	p->descent_speed = MC_CFG_SAF_DESCENT_SPEED_M_S;
	p->hard_lo = g->l_min + MC_CFG_SAF_HARD_MARGIN_M;
	p->hard_hi = g->l_max - MC_CFG_SAF_HARD_MARGIN_M;
	p->heartbeat_timeout = MC_CFG_SAF_HEARTBEAT_TIMEOUT_S;
	p->comms_return = MC_CFG_SAF_COMMS_LOST_RETURN_S;
	p->leg_speed = MC_CFG_LEG_SPEED_M_S;
	p->leg_accel = MC_CFG_LEG_ACCEL_M_S2;
	p->cue_lo = g->soft_lo + MC_CFG_SAF_SERVO_MARGIN_M;
	p->cue_hi = g->soft_hi - MC_CFG_SAF_SERVO_MARGIN_M;
	for (int i = 0; i < MC_LEGS; i++) {
		p->park[i] = g->soft_lo;
		p->neutral[i] = g->l0;
	}
	p->mass = MC_CFG_PAYLOAD_KG;
	p->com.x = MC_CFG_PAYLOAD_COM_X_M;
	p->com.y = MC_CFG_PAYLOAD_COM_Y_M;
	p->com.z = MC_CFG_PAYLOAD_COM_Z_M;
	p->rate_hz = MC_CFG_WO_RATE_HZ;
	p->input_steps = (int)lround(MC_CFG_WO_RATE_HZ / VEHICLE_RATE_HZ);
}

static void rest_input(double in[6])
{
	in[0] = 0.0;
	in[1] = 0.0;
	in[2] = MC_GRAVITY;
	in[3] = 0.0;
	in[4] = 0.0;
	in[5] = 0.0;
}

static void reset_input(mc_ctl *c)
{
	rest_input(c->in_from);
	rest_input(c->in_to);
	rest_input(c->in_now);
	c->in_step = c->p.input_steps;
	c->since_input = 0U;
	c->have_seq = false; /* the first sample after this starts the sequence */
}

bool mc_ctl_init(mc_ctl *c, const mc_geometry *g, const mc_ctl_params *p,
		 const mc_washout_params *wop)
{
	(void)memset(c, 0, sizeof(*c));
	c->g = *g;
	c->p = *p;
	c->wop = *wop;
	c->dt = 1.0 / p->rate_hz;
	c->state = MC_ST_OFF;
	c->phase = MC_PH_OFF;
	c->fault_leg = -1;
	c->event_leg = -1;
	c->ws_scale = 1.0;
	reset_input(c);

	const mc_limit2_params lim = {p->leg_speed, p->leg_accel, g->soft_lo, g->soft_hi, c->dt};

	mc_shaper_init(&c->sh, p->park, &lim);
	return mc_washout_init(&c->wo, wop);
}

const char *mc_state_name(mc_state s)
{
	static const char *const names[] = {"OFF",     "HOMING",  "PARKED", "ENGAGING",
					    "ENGAGED", "PARKING", "FAULT",  "ESTOP"};

	return ((unsigned)s < (sizeof(names) / sizeof(names[0]))) ? names[s] : "?";
}

const char *mc_fault_name(mc_fault f)
{
	static const char *const names[] = {"NONE",    "FOLLOWING", "HARD_LIMIT", "CURRENT",
					    "OVERRUN", "HOMING",    "ESTOP",      "COMMS_LOST",
					    "INTERNAL"};

	return ((unsigned)f < (sizeof(names) / sizeof(names[0]))) ? names[f] : "?";
}

/* ---- phases -------------------------------------------------------------- */

static void start_traj(mc_ctl *c, const double end[MC_LEGS], double t_min, double v_max)
{
	mc_traj_init(&c->tr, c->sp, end, t_min, v_max, c->p.leg_accel, c->dt);
	c->phase = MC_PH_TRAJ;
}

/* Brake every leg to rest from its current setpoint motion. */
static void start_stop(mc_ctl *c)
{
	const mc_limit2_params lim = {c->p.leg_speed, c->p.leg_accel, c->g.soft_lo, c->g.soft_hi,
				      c->dt};

	mc_shaper_init(&c->sh, c->sp, &lim);
	for (int i = 0; i < MC_LEGS; i++) {
		c->sh.vel[i] = c->sp_vel[i];
		c->stop_target[i] = mc_limit2_stop_point(c->sp[i], c->sp_vel[i], &lim);
		c->sh.prev[i] = c->stop_target[i];
	}
	c->stop_steps = 0U;
	c->phase = MC_PH_STOP;
}

static void set_event(mc_ctl *c, mc_fault f, int leg, double value)
{
	c->event = f;
	c->event_leg = leg;
	c->event_value = value;
	c->event_count++;
}

static bool halting(mc_state s)
{
	return (s == MC_ST_FAULT) || (s == MC_ST_ESTOP);
}

static bool servo_trip(mc_fault f)
{
	return (f == MC_FAULT_FOLLOWING) || (f == MC_FAULT_CURRENT) || (f == MC_FAULT_INTERNAL);
}

static bool inside_hard(const mc_ctl *c, int i)
{
	return (c->meas[i] >= c->p.hard_lo) && (c->meas[i] <= c->p.hard_hi);
}

/* FAULT or ESTOP: stop, then descend to park. */
static void halt(mc_ctl *c, mc_state st, mc_fault f, int leg, double value)
{
	if (halting(c->state)) {
		/* already halting: an E-stop overrides the state, the first fault stays */
		if (st == MC_ST_ESTOP) {
			c->state = MC_ST_ESTOP;
		}
		return;
	}
	c->state = st;
	c->fault = f;
	c->fault_leg = leg;
	c->fault_value = value;
	c->descended = false;
	c->comms_lost = false;
	/* the monitors that keep running while halting (monitor_halt) start from here */
	c->trips_seen = ((leg >= 0) && servo_trip(f)) ? (uint8_t)(1U << leg) : 0U;
	c->hard_inside = 0U;
	c->hard_seen = 0U;
	for (int i = 0; i < MC_LEGS; i++) {
		if (inside_hard(c, i)) {
			c->hard_inside |= (uint8_t)(1U << i);
		}
	}
	set_event(c, f, leg, value);
	if (c->phase != MC_PH_OFF) {
		start_stop(c);
	}
}

static void stop_done(mc_ctl *c)
{
	if (c->state == MC_ST_PARKING) {
		start_traj(c, c->p.park, c->p.park_time, c->p.leg_speed);
	} else {
		/* FAULT, ESTOP: stand still, then descend slowly */
		c->dwell_steps = (uint32_t)lround(c->p.stop_dwell / c->dt);
		c->phase = MC_PH_DWELL;
	}
}

static void traj_done(mc_ctl *c)
{
	switch (c->state) {
	case MC_ST_HOMING:
	case MC_ST_PARKING:
		c->state = MC_ST_PARKED;
		c->phase = MC_PH_HOLD;
		break;
	case MC_ST_ENGAGING: {
		const mc_limit2_params lim = {c->p.leg_speed, c->p.leg_accel, c->p.cue_lo,
					      c->p.cue_hi, c->dt};

		mc_washout_reset(&c->wo);
		mc_shaper_init(&c->sh, c->sp, &lim);
		reset_input(c);
		c->comms_lost = false;
		c->state = MC_ST_ENGAGED;
		c->phase = MC_PH_CUE;
		break;
	}
	default: /* FAULT, ESTOP */
		c->descended = true;
		c->phase = MC_PH_HOLD;
		break;
	}
}

/* ---- commands ------------------------------------------------------------ */

static bool legs_plausible(const mc_ctl *c)
{
	for (int i = 0; i < MC_LEGS; i++) {
		if ((c->meas[i] < (c->g.l_min - HOMING_SLACK_M)) ||
		    (c->meas[i] > (c->g.l_max + HOMING_SLACK_M))) {
			return false;
		}
	}
	return true;
}

static double max_park_error(const mc_ctl *c)
{
	double e = 0.0;

	for (int i = 0; i < MC_LEGS; i++) {
		e = fmax(e, fabs(c->meas[i] - c->p.park[i]));
	}
	return e;
}

int mc_ctl_command(mc_ctl *c, mc_cmd cmd)
{
	switch (cmd) {
	case MC_CMD_ENABLE:
		if (c->state != MC_ST_OFF) {
			return MC_CST_BAD_STATE;
		}
		if (c->estop) {
			return MC_CST_ESTOP;
		}
		if (c->inhibit) {
			return MC_CST_BAD_STATE;
		}
		/* encoder plausibility: checked here only, before the servos first move */
		if (!legs_plausible(c)) {
			c->state = MC_ST_FAULT;
			c->fault = MC_FAULT_HOMING;
			c->fault_leg = -1;
			c->descended = true; /* never moved: nothing to descend */
			set_event(c, MC_FAULT_HOMING, -1, 0.0);
			return MC_CST_OK;
		}
		(void)memcpy(c->sp, c->meas, sizeof(c->sp));
		(void)memset(c->sp_vel, 0, sizeof(c->sp_vel));
		c->state = MC_ST_HOMING;
		start_traj(c, c->p.park, 0.0, c->p.descent_speed);
		return MC_CST_OK;

	case MC_CMD_ENGAGE:
		if (c->state != MC_ST_PARKED) {
			return MC_CST_BAD_STATE;
		}
		c->state = MC_ST_ENGAGING;
		start_traj(c, c->p.neutral, c->p.engage_time, c->p.leg_speed);
		return MC_CST_OK;

	case MC_CMD_PARK:
		if ((c->state != MC_ST_ENGAGED) && (c->state != MC_ST_ENGAGING)) {
			return MC_CST_BAD_STATE;
		}
		c->state = MC_ST_PARKING;
		c->comms_lost = false;
		start_stop(c);
		return MC_CST_OK;

	case MC_CMD_RESET_FAULT:
		if ((c->state != MC_ST_FAULT) && (c->state != MC_ST_ESTOP)) {
			return MC_CST_BAD_STATE;
		}
		if (c->estop) {
			return MC_CST_ESTOP;
		}
		if (!c->descended) {
			return MC_CST_BAD_STATE;
		}
		c->fault = MC_FAULT_NONE;
		c->fault_leg = -1;
		c->fault_value = 0.0;
		c->cmd.fault_epoch++;
		if (c->phase == MC_PH_OFF) {
			c->state = MC_ST_OFF;
			return MC_CST_OK;
		}
		/* restart from where the legs are: a released leg may not be at park */
		(void)memcpy(c->sp, c->meas, sizeof(c->sp));
		(void)memset(c->sp_vel, 0, sizeof(c->sp_vel));
		if (max_park_error(c) < PARK_TOL_M) {
			c->state = MC_ST_PARKED;
			c->phase = MC_PH_HOLD;
		} else {
			c->state = MC_ST_HOMING;
			start_traj(c, c->p.park, 0.0, c->p.descent_speed);
		}
		return MC_CST_OK;

	default:
		return MC_CST_BAD_KEY;
	}
}

void mc_ctl_inhibit(mc_ctl *c, bool inhibit)
{
	c->inhibit = inhibit;
}

bool mc_ctl_vehicle(mc_ctl *c, const double f[3], const double w[3], uint16_t seq)
{
	uint16_t advance = (uint16_t)(seq - c->last_seq);

	if (c->have_seq && ((advance == 0U) || (advance >= 0x8000U))) {
		c->stale_samples++;
		return false; /* repeated or older: not a sign of life */
	}
	c->have_seq = true;
	c->last_seq = seq;
	(void)memcpy(c->in_from, c->in_now, sizeof(c->in_from));
	for (int k = 0; k < 3; k++) {
		c->in_to[k] = f[k];
		c->in_to[3 + k] = w[k];
	}
	c->in_step = 0;
	c->since_input = 0U;
	return true;
}

/* ---- step ---------------------------------------------------------------- */

static bool active(mc_state s)
{
	return (s == MC_ST_HOMING) || (s == MC_ST_PARKED) || (s == MC_ST_ENGAGING) ||
	       (s == MC_ST_ENGAGED) || (s == MC_ST_PARKING);
}

/*
 * FAULT / ESTOP: the first fault stays latched, but the servo trips and the
 * hard limits are still watched. Each further leg trip is reported once (and
 * the leg is released in mc_ctl_step); a leg that was inside its hard limits
 * and leaves them is reported once. Loop overruns are not re-reported.
 */
static void monitor_halt(mc_ctl *c, const mc_servo_fb *fb)
{
	if (c->phase == MC_PH_OFF) {
		return; /* servos off (implausible encoder at ENABLE): nothing moves */
	}
	for (int i = 0; i < MC_LEGS; i++) {
		const uint8_t bit = (uint8_t)(1U << i);
		const int code = fb->leg_trip[i];

		if ((code != 0) && (fb->fault_epoch == c->cmd.fault_epoch) &&
		    ((c->trips_seen & bit) == 0U)) {
			c->trips_seen |= bit;
			set_event(c, (mc_fault)code, i,
				  (code == (int)MC_FAULT_FOLLOWING) ? fb->ferr[i] : fb->current[i]);
		}
		if (inside_hard(c, i)) {
			c->hard_inside |= bit;
		} else if (((c->hard_inside & bit) != 0U) && ((c->hard_seen & bit) == 0U)) {
			c->hard_seen |= bit;
			set_event(c, MC_FAULT_HARD_LIMIT, i, c->meas[i]);
		}
	}
}

static void check_faults(mc_ctl *c, const mc_servo_fb *fb, bool overrun)
{
	if (c->estop && (c->state != MC_ST_OFF) && (c->state != MC_ST_ESTOP)) {
		halt(c, MC_ST_ESTOP, MC_FAULT_ESTOP, -1, 0.0);
		return;
	}
	if (halting(c->state)) {
		monitor_halt(c, fb);
		return;
	}
	if (!active(c->state)) {
		return;
	}
	/* a trip from before the last RESET is stale until the bank has seen the RESET */
	if ((fb->trip != 0) && (fb->fault_epoch == c->cmd.fault_epoch)) {
		halt(c, MC_ST_FAULT, (mc_fault)fb->trip, fb->trip_leg, fb->trip_value);
		return;
	}
	if (overrun) {
		halt(c, MC_ST_FAULT, MC_FAULT_OVERRUN, -1, 0.0);
		return;
	}
	if (c->state == MC_ST_HOMING) {
		return; /* homing starts on the lower end stop, below the hard limit */
	}
	for (int i = 0; i < MC_LEGS; i++) {
		if (!inside_hard(c, i)) { /* also catches a NaN reading */
			halt(c, MC_ST_FAULT, MC_FAULT_HARD_LIMIT, i, c->meas[i]);
			return;
		}
	}
}

static bool near_neutral(const mc_pose *q)
{
	return (fabs(q->x) < NEUTRAL_POS_M) && (fabs(q->y) < NEUTRAL_POS_M) &&
	       (fabs(q->z) < NEUTRAL_POS_M) && (fabs(q->roll) < NEUTRAL_ANG_RAD) &&
	       (fabs(q->pitch) < NEUTRAL_ANG_RAD) && (fabs(q->yaw) < NEUTRAL_ANG_RAD);
}

static void cue(mc_ctl *c, double sp[MC_LEGS], double acc[MC_LEGS])
{
	/* heartbeat (F12) */
	c->since_input++;
	if (!c->comms_lost && ((c->since_input * c->dt) > c->p.heartbeat_timeout)) {
		c->comms_lost = true;
		c->lost_steps = 0U;
		set_event(c, MC_FAULT_COMMS_LOST, -1, 0.0);
		(void)memcpy(c->in_from, c->in_now, sizeof(c->in_from));
		rest_input(c->in_to);
		c->in_step = 0;
	}

	/* 100 Hz samples, linear interpolation to the cueing rate (one sample late) */
	if (c->in_step < c->p.input_steps) {
		c->in_step++;
		double u = (double)c->in_step / (double)c->p.input_steps;

		for (int k = 0; k < 6; k++) {
			c->in_now[k] = c->in_from[k] + (u * (c->in_to[k] - c->in_from[k]));
		}
	}

	double legs[MC_LEGS];

	mc_washout_step(&c->wo, &c->in_now[0], &c->in_now[3], &c->cue_pose);
	c->ws_saturated =
		mc_ws_limit(&c->g, &c->cue_pose, &c->cmd_pose, &c->ws_scale) == MC_WS_SCALED;
	mc_ik(&c->g, &c->cmd_pose, legs);
	for (int i = 0; i < MC_LEGS; i++) {
		/* the workspace limit allows up to the soft limit; stay the margin inside */
		legs[i] = fmax(c->p.cue_lo, fmin(c->p.cue_hi, legs[i]));
	}
	mc_shaper_step(&c->sh, legs);
	(void)memcpy(sp, c->sh.pos, sizeof(c->sh.pos));
	(void)memcpy(acc, c->sh.acc, sizeof(c->sh.acc));

	if (c->comms_lost) {
		c->lost_steps++;
		if (near_neutral(&c->cue_pose) || ((c->lost_steps * c->dt) > c->p.comms_return)) {
			c->state = MC_ST_PARKING;
			c->comms_lost = false;
			start_stop(c);
		}
	}
}

void mc_ctl_step(mc_ctl *c, const mc_servo_fb *fb, bool estop, bool overrun, mc_servo_cmd *out)
{
	double sp[MC_LEGS], vel[MC_LEGS], acc[MC_LEGS];

	c->steps++;
	(void)memcpy(c->meas, fb->pos, sizeof(c->meas));
	c->estop = estop;

	/* pose and gravity load from the measured legs */
	mc_fk_info info;

	(void)mc_fk(&c->g, c->meas, &c->pose, c->fk_ok ? FK_ITER : FK_ITER_COLD, FK_TOL, &info);
	c->fk_ok = info.converged;
	if (!c->fk_ok || !mc_statics(&c->g, &c->pose, c->p.mass, &c->p.com, c->force)) {
		(void)memset(c->force, 0, sizeof(c->force));
		if (!c->fk_ok) {
			(void)memset(&c->pose, 0, sizeof(c->pose)); /* cold start next time */
		}
	}

	check_faults(c, fb, overrun);

	(void)memcpy(sp, c->sp, sizeof(sp));
	(void)memset(acc, 0, sizeof(acc));

	switch (c->phase) {
	case MC_PH_OFF:
		(void)memcpy(sp, c->meas, sizeof(sp));
		break;
	case MC_PH_HOLD:
		break;
	case MC_PH_DWELL:
		if (c->dwell_steps > 0U) {
			c->dwell_steps--;
		} else {
			start_traj(c, c->p.park, 0.0, c->p.descent_speed);
		}
		break;
	case MC_PH_STOP: {
		if ((c->stop_steps * c->dt) > STOP_TIMEOUT_S) {
			/*
			 * The stop should take at most leg_speed / leg_accel (83 ms).
			 * PARKING: raise a FAULT, which restarts the stop from the
			 * setpoints. FAULT / ESTOP: keep the latched fault and go on
			 * to the standstill and the descent. Either way hold the
			 * setpoint for this step.
			 */
			if (c->state == MC_ST_PARKING) {
				halt(c, MC_ST_FAULT, MC_FAULT_INTERNAL, -1, 0.0);
			} else {
				stop_done(c);
			}
			break;
		}
		c->stop_steps++;

		bool done = true;

		mc_shaper_step(&c->sh, c->stop_target);
		for (int i = 0; i < MC_LEGS; i++) {
			sp[i] = c->sh.pos[i];
			acc[i] = c->sh.acc[i];
			done = done && (fabs(c->sh.vel[i]) <= STOP_VEL_TOL_M_S) &&
			       (fabs(c->sh.pos[i] - c->stop_target[i]) <= STOP_POS_TOL_M);
		}
		if (done) {
			stop_done(c);
		}
		break;
	}
	case MC_PH_TRAJ:
		if (mc_traj_step(&c->tr, sp, vel, acc)) {
			traj_done(c);
		}
		break;
	case MC_PH_CUE:
		cue(c, sp, acc);
		break;
	default:
		break;
	}

	/*
	 * A leg that tripped its own following-error or current monitor (seized,
	 * or too weak for the load) is released while halting: its setpoint
	 * follows the measurement, so the servo no longer fights it at full
	 * current while the others descend. This covers the first trip and any
	 * later one during the descent.
	 */
	bool released[MC_LEGS];
	const bool halt_now = halting(c->state) && (fb->fault_epoch == c->cmd.fault_epoch);

	for (int i = 0; i < MC_LEGS; i++) {
		released[i] = halt_now && ((fb->leg_trip[i] == (int)MC_FAULT_FOLLOWING) ||
					   (fb->leg_trip[i] == (int)MC_FAULT_CURRENT));
		if (released[i]) {
			sp[i] = c->meas[i];
			acc[i] = 0.0;
		}
	}

	for (int i = 0; i < MC_LEGS; i++) {
		c->sp_vel[i] = (sp[i] - c->sp[i]) / c->dt;
		c->sp[i] = sp[i];
		c->sp_acc[i] = acc[i];
	}

	mc_servo_cmd *cmd = &c->cmd;

	cmd->seq++;
	cmd->on = c->phase != MC_PH_OFF;
	cmd->track = (c->state == MC_ST_ENGAGED) && (c->phase == MC_PH_CUE);
	for (int i = 0; i < MC_LEGS; i++) {
		cmd->prev[i] = cmd->on ? cmd->next[i] : c->meas[i];
		cmd->next[i] = sp[i];
		cmd->acc[i] = acc[i];
		cmd->force[i] = c->force[i];
		cmd->released[i] = released[i];
	}
	*out = *cmd;
}

/* ---- parameters ---------------------------------------------------------- */

static double *param_slot(mc_washout_params *w, int key, double *scale)
{
	switch (key) {
	case MC_PARAM_FORCE_GAIN:
		*scale = 1e-3;
		return &w->force_gain;
	case MC_PARAM_TILT_RATE_LIMIT:
		*scale = 1e-3 * DEG;
		return &w->tilt_rate_limit;
	case MC_PARAM_ROT_GAIN:
		*scale = 1e-3;
		return &w->rot_gain;
	case MC_PARAM_SURGE_SWAY_WN:
		*scale = 1e-3;
		return &w->surge_sway_wn;
	case MC_PARAM_SURGE_SWAY_WB:
		*scale = 1e-3;
		return &w->surge_sway_wb;
	case MC_PARAM_HEAVE_WN:
		*scale = 1e-3;
		return &w->heave_wn;
	case MC_PARAM_TILT_WN:
		*scale = 1e-3;
		return &w->tilt_wn;
	default:
		return NULL;
	}
}

static bool param_in_range(int key, double v)
{
	switch (key) {
	case MC_PARAM_FORCE_GAIN:
	case MC_PARAM_ROT_GAIN:
		return (v >= 0.0) && (v <= 1.0);
	case MC_PARAM_TILT_RATE_LIMIT:
		return (v >= (0.5 * DEG)) && (v <= (10.0 * DEG));
	default: /* break frequencies, rad/s */
		return (v >= 0.1) && (v <= 20.0);
	}
}

int mc_ctl_set_param(mc_ctl *c, int key, int32_t value)
{
	mc_washout_params w = c->wop;
	double scale;
	double *slot = param_slot(&w, key, &scale);

	if (slot == NULL) {
		return MC_CST_BAD_KEY;
	}
	*slot = value * scale;
	if (!param_in_range(key, *slot)) {
		return MC_CST_BAD_VALUE;
	}
	/* filters are re-initialised: only while the cueing is not running */
	if ((c->state != MC_ST_OFF) && (c->state != MC_ST_PARKED)) {
		return MC_CST_BAD_STATE;
	}

	mc_washout probe;

	if (!mc_washout_init(&probe, &w)) {
		return MC_CST_BAD_VALUE;
	}
	c->wop = w;
	c->wo = probe;
	return MC_CST_OK;
}

int mc_ctl_get_param(const mc_ctl *c, int key, int32_t *value)
{
	mc_washout_params w = c->wop;
	double scale;
	double *slot = param_slot(&w, key, &scale);

	if (slot == NULL) {
		return MC_CST_BAD_KEY;
	}
	*value = (int32_t)lround(*slot / scale);
	return MC_CST_OK;
}

/* ---- servo bank ---------------------------------------------------------- */

void mc_bank_init(mc_servo_bank *b, const mc_servo_params *sp, const mc_ctl_params *cp)
{
	(void)memset(b, 0, sizeof(*b));
	b->sp = *sp;
	b->cue_dt = 1.0 / cp->rate_hz;
	b->ferr_max = MC_CFG_SAF_FOLLOWING_ERROR_M;
	b->ferr_ticks_max = (int)lround(MC_CFG_SAF_FOLLOWING_ERROR_TIME_S / sp->dt);
	b->sat_ticks_max = (int)lround(MC_CFG_SAF_CURRENT_SAT_TIME_S / sp->dt);
	b->fb.trip_leg = -1;
}

static void trip(mc_servo_bank *b, mc_fault f, int leg, double value)
{
	if (b->fb.leg_trip[leg] == 0) {
		b->fb.leg_trip[leg] = (int)f;
	}
	if (b->fb.trip == 0) {
		b->fb.trip = (int)f;
		b->fb.trip_leg = leg;
		b->fb.trip_value = value;
	}
}

void mc_bank_step(mc_servo_bank *b, const mc_servo_cmd *cmd, const double pos[MC_LEGS],
		  double current[MC_LEGS])
{
	(void)memcpy(b->fb.pos, pos, sizeof(b->fb.pos));

	if (cmd->fault_epoch != b->fault_epoch) {
		b->fault_epoch = cmd->fault_epoch;
		b->fb.fault_epoch = cmd->fault_epoch;
		b->fb.trip = 0;
		b->fb.trip_leg = -1;
		b->fb.trip_value = 0.0;
		(void)memset(b->fb.leg_trip, 0, sizeof(b->fb.leg_trip));
		(void)memset(b->ferr_ticks, 0, sizeof(b->ferr_ticks));
		(void)memset(b->sat_ticks, 0, sizeof(b->sat_ticks));
	}
	if (cmd->stats_epoch != b->stats_epoch) {
		b->stats_epoch = cmd->stats_epoch;
		(void)memset(b->sum2, 0, sizeof(b->sum2));
		(void)memset(b->peak, 0, sizeof(b->peak));
		b->n = 0U;
	}

	if (!cmd->on) {
		b->on = false;
		for (int i = 0; i < MC_LEGS; i++) {
			current[i] = 0.0;
			b->fb.current[i] = 0.0;
			b->fb.ferr[i] = 0.0;
			b->ferr_ticks[i] = 0;
			b->sat_ticks[i] = 0;
		}
		return;
	}
	if (!b->on) {
		for (int i = 0; i < MC_LEGS; i++) {
			mc_servo_reset(&b->axis[i], &b->sp, pos[i]);
		}
		b->on = true;
		b->seq = cmd->seq;
		b->sub = 0;
	}
	if (cmd->seq != b->seq) {
		b->seq = cmd->seq;
		b->sub = 0;
	}

	/* setpoints one cueing period apart, interpolated at the servo rate */
	double ratio = b->cue_dt / b->sp.dt;
	double u = fmin(1.0, (double)(b->sub + 1) / ratio);

	b->sub++;
	for (int i = 0; i < MC_LEGS; i++) {
		double sp = cmd->prev[i] + (u * (cmd->next[i] - cmd->prev[i]));
		double vel = (cmd->next[i] - cmd->prev[i]) / b->cue_dt;
		double e = sp - pos[i];

		if (cmd->released[i]) {
			/* no integral, no velocity history: gravity feed-forward only */
			mc_servo_reset(&b->axis[i], &b->sp, pos[i]);
			sp = pos[i];
			vel = 0.0;
			e = 0.0;
		}
		current[i] = mc_servo_step(&b->axis[i], sp, vel, cmd->acc[i], cmd->force[i], pos[i]);
		b->fb.current[i] = current[i];
		b->fb.ferr[i] = e;
		if (b->axis[i].invalid) {
			/* NaN / infinite setpoint, measurement or feed-forward: 0 A and a fault at once */
			trip(b, MC_FAULT_INTERNAL, i, 0.0);
		}

		b->ferr_ticks[i] = (fabs(e) > b->ferr_max) ? (b->ferr_ticks[i] + 1) : 0;
		if (b->ferr_ticks[i] >= b->ferr_ticks_max) {
			trip(b, MC_FAULT_FOLLOWING, i, e);
		}
		b->sat_ticks[i] = b->axis[i].saturated ? (b->sat_ticks[i] + 1) : 0;
		if (b->sat_ticks[i] >= b->sat_ticks_max) {
			trip(b, MC_FAULT_CURRENT, i, current[i]);
		}
		if (cmd->track) {
			b->sum2[i] += e * e;
			b->peak[i] = fmax(b->peak[i], fabs(e));
		}
	}
	if (cmd->track) {
		b->n++;
	}
}

void mc_bank_tracking(const mc_servo_bank *b, double rms[MC_LEGS], double peak[MC_LEGS])
{
	for (int i = 0; i < MC_LEGS; i++) {
		rms[i] = (b->n > 0U) ? sqrt(b->sum2[i] / b->n) : 0.0;
		peak[i] = b->peak[i];
	}
}
