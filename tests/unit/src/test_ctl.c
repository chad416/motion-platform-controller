/*
 * SPEC F08, F11-F13, P02, P06: the controller core against six simulated
 * actuators, at the firmware rates (cueing 500 Hz, servo 1 kHz). These are the
 * unit-level twins of the Renode tests IT01-IT08.
 */
#include <math.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "mcue/ctl.h"
#include "mcue/kin.h"
#include "mcue/plant.h"

#define DEG (MC_PI / 180.0)

struct sim {
	mc_geometry g;
	mc_ctl c;
	mc_servo_bank b;
	mc_plant pl[MC_LEGS];
	mc_servo_cmd cmd;
	mc_pose true_pose;
	bool estop;
	bool overrun;
	double t;
	/* vehicle input */
	bool streaming;
	bool repeat_seq; /* a host stuck on one frame: the sequence number does not advance */
	uint16_t seq;
	double f[3], w[3];
	int step_no;
	/* records */
	double max_speed;  /* true leg speed, m/s, since cleared */
};

static struct sim S;

static void loads(struct sim *s, double load[MC_LEGS])
{
	double l[MC_LEGS];
	mc_fk_info info;

	for (int i = 0; i < MC_LEGS; i++) {
		l[i] = mc_plant_length(&s->pl[i]);
	}
	(void)mc_fk(&s->g, l, &s->true_pose, 30, 1e-12, &info);
	zassert_true(info.converged);
	zassert_true(mc_statics(&s->g, &s->true_pose, 500.0, &s->c.p.com, load));
}

static void sim_init(struct sim *s)
{
	mc_geometry_params gp;
	mc_ctl_params cp;
	mc_washout_params wp;
	mc_plant_params pp;
	mc_servo_params sp;

	(void)memset(s, 0, sizeof(*s));
	mc_geometry_default_params(&gp);
	zassert_true(mc_geometry_build(&s->g, &gp));
	mc_ctl_default_params(&cp, &s->g);
	mc_washout_default_params(&wp);
	zassert_true(mc_ctl_init(&s->c, &s->g, &cp, &wp));
	mc_plant_default_params(&pp, &s->g);
	mc_servo_default_params(&sp, &pp);
	mc_bank_init(&s->b, &sp, &cp);
	for (int i = 0; i < MC_LEGS; i++) {
		/* power-up: resting on the lower stops */
		mc_plant_init(&s->pl[i], &pp, s->g.l_min);
	}
	s->f[2] = MC_GRAVITY;
	for (int i = 0; i < MC_LEGS; i++) {
		s->b.fb.pos[i] = mc_plant_encoder(&s->pl[i]);
	}
}

/* One cueing period: controller, then two servo periods. */
static void sim_step(struct sim *s)
{
	double load[MC_LEGS];

	if (s->streaming && ((s->step_no % 5) == 0)) {
		(void)mc_ctl_vehicle(&s->c, s->f, s->w, s->seq);
		if (!s->repeat_seq) {
			s->seq++;
		}
	}
	s->step_no++;
	mc_ctl_step(&s->c, &s->b.fb, s->estop, s->overrun, &s->cmd);
	loads(s, load);
	for (int k = 0; k < 2; k++) {
		double pos[MC_LEGS], cur[MC_LEGS];

		for (int i = 0; i < MC_LEGS; i++) {
			pos[i] = mc_plant_encoder(&s->pl[i]);
		}
		mc_bank_step(&s->b, &s->cmd, pos, cur);
		for (int i = 0; i < MC_LEGS; i++) {
			mc_plant_step(&s->pl[i], cur[i], load[i]);
			s->max_speed = fmax(s->max_speed, fabs(mc_plant_speed(&s->pl[i])));
		}
	}
	s->t += s->c.dt;
}

static void run(struct sim *s, double seconds)
{
	int n = (int)lround(seconds / s->c.dt);

	for (int k = 0; k < n; k++) {
		sim_step(s);
	}
}

/* Run until the state is reached; returns the time it took. */
static double run_until(struct sim *s, mc_state st, double timeout)
{
	double t0 = s->t;

	while (s->c.state != st) {
		zassert_true((s->t - t0) < timeout, "no %s after %.2f s (state %s, fault %s)",
			     mc_state_name(st), timeout, mc_state_name(s->c.state),
			     mc_fault_name(s->c.fault));
		sim_step(s);
	}
	return s->t - t0;
}

static double max_leg_error(const struct sim *s, const double target[MC_LEGS])
{
	double e = 0.0;

	for (int i = 0; i < MC_LEGS; i++) {
		e = fmax(e, fabs(mc_plant_length(&s->pl[i]) - target[i]));
	}
	return e;
}

static void engage(struct sim *s)
{
	run(s, 0.01);
	zassert_equal(mc_ctl_command(&s->c, MC_CMD_ENABLE), MC_CST_OK);
	(void)run_until(s, MC_ST_PARKED, 2.0);
	s->streaming = true;
	zassert_equal(mc_ctl_command(&s->c, MC_CMD_ENGAGE), MC_CST_OK);
	(void)run_until(s, MC_ST_ENGAGED, 3.5);
}

static void *setup(void)
{
	return NULL;
}

static void before(void *f)
{
	ARG_UNUSED(f);
	sim_init(&S);
}

ZTEST_SUITE(ctl, NULL, setup, before, NULL, NULL);

/* IT01 twin: OFF with the servos off, ENABLE -> HOMING -> PARKED. */
ZTEST(ctl, test_boot_enable_homes_to_park)
{
	run(&S, 0.5);
	zassert_equal(S.c.state, MC_ST_OFF);
	zassert_false(S.cmd.on);
	zassert_within(max_leg_error(&S, (double[MC_LEGS]){S.g.l_min, S.g.l_min, S.g.l_min,
							    S.g.l_min, S.g.l_min, S.g.l_min}),
		       0.0, 1e-12, "legs moved while OFF");

	zassert_equal(mc_ctl_command(&S.c, MC_CMD_ENGAGE), MC_CST_BAD_STATE);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_ENABLE), MC_CST_OK);
	zassert_equal(S.c.state, MC_ST_HOMING);
	S.max_speed = 0.0;
	double t = run_until(&S, MC_ST_PARKED, 2.0);

	run(&S, 0.2);
	double e = max_leg_error(&S, S.c.p.park);

	TC_PRINT("homing: %.3f s, leg error at park %.4f mm, max speed %.4f m/s\n", t, e * 1e3,
		 S.max_speed);
	zassert_true(e < 1e-4);
	zassert_true(S.max_speed <= S.c.p.descent_speed * 1.02);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_ENABLE), MC_CST_BAD_STATE);
}

/* IT02 twin: ENGAGE -> neutral within 3.5 s; FK pose ~ 0; PARK -> PARKED. */
ZTEST(ctl, test_engage_and_park)
{
	run(&S, 0.01);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_ENABLE), MC_CST_OK);
	(void)run_until(&S, MC_ST_PARKED, 2.0);
	S.streaming = true;
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_ENGAGE), MC_CST_OK);
	double t = run_until(&S, MC_ST_ENGAGED, 3.5);

	run(&S, 0.5);
	const mc_pose *q = &S.c.pose;

	TC_PRINT("engage: %.3f s; FK pose %.4f %.4f %.4f mm, %.4f %.4f %.4f deg\n", t, q->x * 1e3,
		 q->y * 1e3, q->z * 1e3, q->roll / DEG, q->pitch / DEG, q->yaw / DEG);
	zassert_true(S.c.fk_ok);
	zassert_true(fabs(q->x) < 5e-4 && fabs(q->y) < 5e-4 && fabs(q->z) < 5e-4);
	zassert_true(fabs(q->roll) < 0.05 * DEG && fabs(q->pitch) < 0.05 * DEG &&
		     fabs(q->yaw) < 0.05 * DEG);

	zassert_equal(mc_ctl_command(&S.c, MC_CMD_PARK), MC_CST_OK);
	t = run_until(&S, MC_ST_PARKED, 4.0);
	run(&S, 0.2);
	TC_PRINT("park: %.3f s\n", t);
	zassert_true(max_leg_error(&S, S.c.p.park) < 1e-4);
}

/* IT03 twin: sustained surge -> washout with tilt; tracking (P02) with the servos. */
ZTEST(ctl, test_surge_step_cueing_and_tracking)
{
	engage(&S);
	S.c.cmd.stats_epoch++;
	S.f[0] = 3.0;
	double prev_pitch = S.c.pose.pitch, max_rate = 0.0, peak_x = 0.0;

	for (int k = 0; k < 4000; k++) { /* 8 s */
		sim_step(&S);
		max_rate = fmax(max_rate, fabs(S.c.pose.pitch - prev_pitch) / S.c.dt);
		prev_pitch = S.c.pose.pitch;
		peak_x = fmax(peak_x, S.c.pose.x);
	}
	double rms[MC_LEGS], peak[MC_LEGS], worst_rms = 0.0, worst_peak = 0.0;

	mc_bank_tracking(&S.b, rms, peak);
	for (int i = 0; i < MC_LEGS; i++) {
		worst_rms = fmax(worst_rms, rms[i]);
		worst_peak = fmax(worst_peak, peak[i]);
	}
	TC_PRINT("surge 3 m/s^2: peak x %.1f mm, x after 8 s %.2f mm, pitch %.3f deg, "
		 "max pitch rate (FK) %.3f deg/s; tracking rms %.4f mm, peak %.4f mm\n",
		 peak_x * 1e3, S.c.pose.x * 1e3, S.c.pose.pitch / DEG, max_rate / DEG,
		 worst_rms * 1e3, worst_peak * 1e3);
	zassert_equal(S.c.state, MC_ST_ENGAGED);
	zassert_within(S.c.pose.pitch, -asin(1.5 / MC_GRAVITY), 0.02 * DEG);
	zassert_true(fabs(S.c.pose.x) < 0.005);
	/* P04 on the measured pose: the 3 deg/s limit plus servo and encoder ripple */
	zassert_true(max_rate < 3.1 * DEG, "%g deg/s", max_rate / DEG);
	zassert_true(worst_rms < 0.001 && worst_peak < 0.003);
}

/* IT06 twin, P06: E-stop while moving -> standstill <= 0.5 s, descent <= 0.05 m/s. */
ZTEST(ctl, test_estop_stops_then_descends)
{
	engage(&S);
	S.f[0] = 5.0; /* hard surge onset and a bump: legs moving fast */
	S.f[2] = MC_GRAVITY + 3.0;
	run(&S, 0.25);

	double v_before = 0.0;

	for (int i = 0; i < MC_LEGS; i++) {
		v_before = fmax(v_before, fabs(mc_plant_speed(&S.pl[i])));
	}
	S.estop = true;
	sim_step(&S);
	zassert_equal(S.c.state, MC_ST_ESTOP);
	zassert_equal(S.c.fault, MC_FAULT_ESTOP);

	/* standstill: every leg slower than 1 mm/s, and it stays still until the descent */
	double t_stop = -1.0, v_dwell = 0.0;

	for (int k = 0; k < 500 && S.c.phase != MC_PH_TRAJ; k++) {
		sim_step(&S);
		double v = 0.0;

		for (int i = 0; i < MC_LEGS; i++) {
			v = fmax(v, fabs(mc_plant_speed(&S.pl[i])));
		}
		if ((v < 0.001) && (t_stop < 0.0)) {
			t_stop = (k + 1) * S.c.dt;
		}
		if (t_stop > 0.0) {
			v_dwell = fmax(v_dwell, v);
		}
	}
	TC_PRINT("standstill held at <= %.5f m/s until the descent\n", v_dwell);
	zassert_equal(S.c.phase, MC_PH_TRAJ);
	zassert_true(v_dwell < 0.001);
	S.estop = false; /* the button is released; the state stays latched */
	S.max_speed = 0.0;
	sim_step(&S);
	zassert_equal(S.c.state, MC_ST_ESTOP);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_RESET_FAULT), MC_CST_BAD_STATE, "not parked");
	while (!S.c.descended) {
		sim_step(&S);
	}
	run(&S, 0.2);
	TC_PRINT("E-stop at %.3f m/s leg speed: standstill after %.0f ms; descent max speed "
		 "%.4f m/s\n",
		 v_before, t_stop * 1e3, S.max_speed);
	zassert_true(v_before > 0.01, "legs were not moving");
	zassert_true((t_stop > 0.0) && (t_stop <= 0.5));
	zassert_true(S.max_speed <= 0.05 * 1.01, "descent %g m/s", S.max_speed);
	zassert_equal(S.c.state, MC_ST_ESTOP);
	zassert_true(max_leg_error(&S, S.c.p.park) < 1e-4);

	S.estop = true;
	sim_step(&S);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_RESET_FAULT), MC_CST_ESTOP);
	S.estop = false;
	sim_step(&S);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_RESET_FAULT), MC_CST_OK);
	zassert_equal(S.c.state, MC_ST_PARKED);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_ENGAGE), MC_CST_OK);
}

/* IT07 twin, F12: VEHICLE_STATE stops -> COMMS_LOST, washout to neutral, PARKING. */
ZTEST(ctl, test_heartbeat_loss_washes_out_then_parks)
{
	engage(&S);
	S.f[0] = 3.0;
	run(&S, 4.0);
	uint32_t events = S.c.event_count;

	S.streaming = false;
	double t = 0.0;

	while (!S.c.comms_lost) {
		sim_step(&S);
		t += S.c.dt;
		zassert_true(t < 0.3);
	}
	TC_PRINT("comms lost detected %.0f ms after the last sample\n", t * 1e3);
	zassert_true(t > 0.19 && t <= 0.2 + 2 * S.c.dt);
	zassert_equal(S.c.event, MC_FAULT_COMMS_LOST);
	zassert_equal(S.c.event_count, events + 1);
	zassert_equal(S.c.state, MC_ST_ENGAGED, "washes out first");

	double t_park = run_until(&S, MC_ST_PARKING, 11.0);

	TC_PRINT("washed out to neutral after %.2f s, pitch %.3f deg\n", t_park,
		 S.c.cue_pose.pitch / DEG);
	(void)run_until(&S, MC_ST_PARKED, 4.0);
	zassert_equal(S.c.fault, MC_FAULT_NONE, "comms loss is not a latched fault");
}

/* IT08 twin, F13: a seized leg -> following-error fault -> controlled descent. */
ZTEST(ctl, test_seized_leg_faults_and_descends)
{
	engage(&S);
	S.f[1] = 3.0;
	run(&S, 0.3);
	S.pl[2].stuck = true;
	double t = 0.0, t_exceed = -1.0;

	while (S.c.state != MC_ST_FAULT) {
		if ((t_exceed < 0.0) && (fabs(S.b.fb.ferr[2]) > 0.005)) {
			t_exceed = t;
		}
		sim_step(&S);
		t += S.c.dt;
		zassert_true(t < 3.0, "no fault");
	}
	TC_PRINT("seized leg 3: error over 5 mm after %.0f ms, FAULT %s leg %d after %.0f ms\n",
		 t_exceed * 1e3, mc_fault_name(S.c.fault), S.c.fault_leg + 1, t * 1e3);
	zassert_equal(S.c.fault, MC_FAULT_FOLLOWING);
	zassert_equal(S.c.fault_leg, 2);
	zassert_true((t - t_exceed) <= 0.060);

	S.max_speed = 0.0;
	while (!S.c.descended) {
		sim_step(&S);
	}
	run(&S, 0.2);
	double seized_at = mc_plant_length(&S.pl[2]);

	TC_PRINT("descent max speed %.4f m/s; seized leg left at %.1f mm above park, drawing "
		 "%.2f A\n",
		 S.max_speed, (seized_at - S.c.p.park[2]) * 1e3, S.b.fb.current[2]);
	zassert_true(S.max_speed <= 0.05 * 1.01);
	/* released: only the gravity feed-forward, no fight at the current limit */
	zassert_true(fabs(S.b.fb.current[2]) < 5.0, "%g A", S.b.fb.current[2]);
	zassert_true(max_leg_error(&S, S.c.p.park) > 0.001, "seized leg cannot be at park");

	/* freed: RESET re-homes from where the legs are, at the descent speed */
	S.pl[2].stuck = false;
	S.max_speed = 0.0;
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_RESET_FAULT), MC_CST_OK);
	zassert_equal(S.c.state, MC_ST_HOMING);
	(void)run_until(&S, MC_ST_PARKED, 10.0);
	run(&S, 0.2);
	zassert_true(max_leg_error(&S, S.c.p.park) < 1e-4);
	zassert_true(S.max_speed <= 0.05 * 1.01, "%g m/s", S.max_speed);
}

/* F13: a leg outside its hard limits (here: forced past the upper one). */
ZTEST(ctl, test_hard_limit_fault)
{
	engage(&S);
	S.pl[4].theta = (S.c.p.hard_hi + 0.001 - S.g.l_min) / (S.pl[4].p.lead / (2.0 * MC_PI));
	run(&S, 0.01);
	zassert_equal(S.c.state, MC_ST_FAULT);
	zassert_equal(S.c.fault, MC_FAULT_HARD_LIMIT);
	zassert_equal(S.c.fault_leg, 4);
}

/* F13: current saturated for 500 ms (the error stays under the following limit). */
ZTEST(ctl, test_current_saturation_trips_after_500ms)
{
	mc_servo_bank *b = &S.b;
	mc_servo_cmd cmd = {.seq = 1, .on = true};
	double pos[MC_LEGS], cur[MC_LEGS];
	int k;

	for (int i = 0; i < MC_LEGS; i++) {
		pos[i] = S.g.l0;
		cmd.prev[i] = S.g.l0;
		cmd.next[i] = S.g.l0;
	}
	cmd.prev[1] = S.g.l0 + 0.004;
	cmd.next[1] = S.g.l0 + 0.004;
	for (k = 0; k < 2000 && b->fb.trip == 0; k++) {
		mc_bank_step(b, &cmd, pos, cur); /* leg 2 blocked 4 mm short */
	}
	TC_PRINT("blocked leg: CURRENT trip after %d ms at %.1f A\n", k, b->fb.trip_value);
	zassert_equal(b->fb.trip, MC_FAULT_CURRENT);
	zassert_equal(b->fb.trip_leg, 1);
	zassert_true(k >= 500 && k <= 560, "%d ms", k);
}

ZTEST(ctl, test_overrun_faults)
{
	engage(&S);
	S.overrun = true;
	sim_step(&S);
	zassert_equal(S.c.state, MC_ST_FAULT);
	zassert_equal(S.c.fault, MC_FAULT_OVERRUN);
}

ZTEST(ctl, test_implausible_encoder_at_enable)
{
	run(&S, 0.01);
	/* 3.2 mm below the stop: impossible (stuck keeps the model from clamping it) */
	S.pl[0].theta = -2.0;
	S.pl[0].stuck = true;
	run(&S, 0.01);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_ENABLE), MC_CST_OK);
	zassert_equal(S.c.state, MC_ST_FAULT);
	zassert_equal(S.c.fault, MC_FAULT_HOMING);
	zassert_false(S.cmd.on);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_RESET_FAULT), MC_CST_OK);
	zassert_equal(S.c.state, MC_ST_OFF);
}

ZTEST(ctl, test_estop_blocks_enable)
{
	S.estop = true;
	run(&S, 0.01);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_ENABLE), MC_CST_ESTOP);
	zassert_equal(S.c.state, MC_ST_OFF);
}

ZTEST(ctl, test_params)
{
	int32_t v;

	zassert_equal(mc_ctl_get_param(&S.c, MC_PARAM_FORCE_GAIN, &v), MC_CST_OK);
	zassert_equal(v, 500);
	zassert_equal(mc_ctl_get_param(&S.c, MC_PARAM_TILT_RATE_LIMIT, &v), MC_CST_OK);
	zassert_equal(v, 3000);
	zassert_equal(mc_ctl_set_param(&S.c, MC_PARAM_FORCE_GAIN, 450), MC_CST_OK);
	zassert_equal(mc_ctl_get_param(&S.c, MC_PARAM_FORCE_GAIN, &v), MC_CST_OK);
	zassert_equal(v, 450);
	zassert_within(S.c.wo.p.force_gain, 0.45, 1e-15);
	zassert_equal(mc_ctl_set_param(&S.c, MC_PARAM_FORCE_GAIN, 1500), MC_CST_BAD_VALUE);
	zassert_equal(mc_ctl_set_param(&S.c, 99, 1), MC_CST_BAD_KEY);
	zassert_equal(mc_ctl_get_param(&S.c, 99, &v), MC_CST_BAD_KEY);
	engage(&S);
	zassert_equal(mc_ctl_set_param(&S.c, MC_PARAM_FORCE_GAIN, 400), MC_CST_BAD_STATE);
	zassert_equal(mc_ctl_get_param(&S.c, MC_PARAM_FORCE_GAIN, &v), MC_CST_OK);
	zassert_equal(v, 450);
}

/* F09: 100 Hz samples are interpolated linearly over five cueing steps. */
ZTEST(ctl, test_input_interpolation)
{
	engage(&S);
	S.streaming = false;
	double f0[3] = {0.0, 0.0, MC_GRAVITY}, f1[3] = {1.0, -2.0, MC_GRAVITY + 0.5};
	double w0[3] = {0}, w1[3] = {0.1, 0.0, -0.05};

	zassert_true(mc_ctl_vehicle(&S.c, f0, w0, S.seq++));
	run(&S, 0.01);
	zassert_true(mc_ctl_vehicle(&S.c, f1, w1, S.seq++));
	for (int k = 1; k <= 6; k++) {
		sim_step(&S);
		double u = fmin(1.0, k / 5.0);

		zassert_within(S.c.in_now[0], u * 1.0, 1e-12);
		zassert_within(S.c.in_now[1], u * -2.0, 1e-12);
		zassert_within(S.c.in_now[2], MC_GRAVITY + (u * 0.5), 1e-12);
		zassert_within(S.c.in_now[5], u * -0.05, 1e-12);
	}
}

/* F12: a host that keeps sending the same frame is not alive. */
ZTEST(ctl, test_repeated_sequence_number_is_not_a_heartbeat)
{
	engage(&S);
	S.f[0] = 1.0;
	run(&S, 1.0);
	zassert_false(S.c.comms_lost);
	S.repeat_seq = true; /* frames keep coming every 10 ms, all with one seq */
	double t = 0.0;

	while (!S.c.comms_lost) {
		sim_step(&S);
		t += S.c.dt;
		zassert_true(t < 0.3, "repeated frames kept the heartbeat alive");
	}
	TC_PRINT("repeated seq: comms lost after %.0f ms, %u stale samples\n", t * 1e3,
		 S.c.stale_samples);
	zassert_true(t > 0.19 && t <= 0.2 + 2 * S.c.dt);
	zassert_true(S.c.stale_samples >= 19);
	zassert_equal(S.c.event, MC_FAULT_COMMS_LOST);
}

/* The sequence number may wrap; it must still advance, and an older one is refused. */
ZTEST(ctl, test_sequence_number_wraps_and_old_samples_are_refused)
{
	const double f[3] = {0.0, 0.0, MC_GRAVITY}, w[3] = {0};

	zassert_true(mc_ctl_vehicle(&S.c, f, w, 65534U)); /* first sample: any seq */
	zassert_true(mc_ctl_vehicle(&S.c, f, w, 65535U));
	zassert_true(mc_ctl_vehicle(&S.c, f, w, 0U));     /* wrapped */
	zassert_false(mc_ctl_vehicle(&S.c, f, w, 0U));    /* repeated */
	zassert_false(mc_ctl_vehicle(&S.c, f, w, 65000U)); /* older */
	zassert_true(mc_ctl_vehicle(&S.c, f, w, 3U));     /* a gap is fine */
	zassert_equal(S.c.stale_samples, 2U);
}

/* The firmware refuses to move if its task watchdog did not start. */
ZTEST(ctl, test_inhibit_refuses_enable)
{
	run(&S, 0.01);
	mc_ctl_inhibit(&S.c, true);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_ENABLE), MC_CST_BAD_STATE);
	zassert_equal(S.c.state, MC_ST_OFF);
	run(&S, 0.1);
	zassert_false(S.cmd.on, "servos must stay off");
	mc_ctl_inhibit(&S.c, false);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_ENABLE), MC_CST_OK);
}

/*
 * A stop point clamped to the soft limit can sit a few um short of where the
 * shaper is able to land exactly (it brakes for |e| - a dt^2 / 8). The stop
 * ends on a tolerance, not on exact equality, so it cannot hang there.
 */
ZTEST(ctl, test_stop_ends_on_a_tolerance)
{
	engage(&S);
	S.c.sp[0] = S.g.soft_hi - 2e-6;
	S.c.sp_vel[0] = 0.0;
	int k;

	for (k = 0; k < 50; k++) {
		mc_ctl_step(&S.c, &S.b.fb, k == 0, false, &S.cmd); /* E-stop on the first step */
		if (S.c.phase != MC_PH_STOP) {
			break;
		}
	}
	TC_PRINT("clamped stop point: stop ended after %d steps, leg 1 %.1f um from it\n", k + 1,
		 (S.c.stop_target[0] - S.c.sh.pos[0]) * 1e6);
	zassert_equal(S.c.state, MC_ST_ESTOP);
	zassert_equal(S.c.phase, MC_PH_DWELL);
	zassert_equal(S.c.stop_target[0], S.g.soft_hi, "stop point clamped to the soft limit");
	zassert_true(S.c.sh.pos[0] != S.c.stop_target[0], "exact equality would never come");
}

/* A stop that does not finish within 0.5 s while PARKING raises a FAULT and descends. */
ZTEST(ctl, test_stop_timeout_while_parking_faults)
{
	engage(&S);
	S.f[1] = 3.0;
	run(&S, 0.3);
	zassert_equal(mc_ctl_command(&S.c, MC_CMD_PARK), MC_CST_OK);
	zassert_equal(S.c.phase, MC_PH_STOP);
	S.c.sh.lim.a_max = 0.001; /* a broken stop: it would take seconds */
	double t = run_until(&S, MC_ST_FAULT, 0.6);

	TC_PRINT("stop timeout: FAULT %s after %.0f ms\n", mc_fault_name(S.c.fault), t * 1e3);
	zassert_equal(S.c.fault, MC_FAULT_INTERNAL);
	zassert_true(t > 0.5);
	while (S.c.phase != MC_PH_TRAJ) { /* the FAULT stop (normal limits), then the standstill */
		sim_step(&S);
		zassert_true(S.t < 30.0, "no descent");
	}
	S.max_speed = 0.0;
	while (!S.c.descended) {
		sim_step(&S);
		zassert_true(S.t < 30.0, "no descent");
	}
	zassert_true(S.max_speed <= 0.05, "descent %g m/s", S.max_speed);
	zassert_true(max_leg_error(&S, S.c.p.park) < 1e-3);
}

/* While halting, a second leg that seizes is reported and released too. */
ZTEST(ctl, test_second_leg_trip_during_descent)
{
	engage(&S);
	S.f[1] = 3.0; /* the legs must move for a seized one to fall behind */
	run(&S, 0.3);
	S.pl[2].stuck = true;
	(void)run_until(&S, MC_ST_FAULT, 3.0);
	zassert_equal(S.c.fault, MC_FAULT_FOLLOWING);
	zassert_equal(S.c.fault_leg, 2);
	uint32_t events = S.c.event_count;

	while (S.c.phase != MC_PH_TRAJ) { /* stop and standstill done: the descent starts */
		sim_step(&S);
		zassert_true(S.t < 10.0);
	}
	run(&S, 0.2);
	S.pl[4].stuck = true;
	double t = 0.0;

	while (S.c.event_count == events) {
		sim_step(&S);
		t += S.c.dt;
		zassert_true(t < 1.0, "second trip never reported");
	}
	run(&S, 0.05);
	TC_PRINT("second seized leg: %s on leg %d reported %.0f ms later; %.2f A on it\n",
		 mc_fault_name(S.c.event), S.c.event_leg + 1, t * 1e3, S.b.fb.current[4]);
	/* early in the slow descent the current monitor (500 ms) can beat the 5 mm error */
	zassert_true((S.c.event == MC_FAULT_FOLLOWING) || (S.c.event == MC_FAULT_CURRENT));
	zassert_equal(S.c.event_leg, 4);
	zassert_equal(S.c.fault, MC_FAULT_FOLLOWING, "the first fault stays latched");
	zassert_equal(S.c.fault_leg, 2);
	zassert_true(S.cmd.released[2] && S.cmd.released[4]);
	zassert_true(fabs(S.b.fb.current[4]) < 5.0, "%g A", S.b.fb.current[4]);
}

/* While halting, a leg that leaves its hard limits is reported (the fault stays ESTOP). */
ZTEST(ctl, test_hard_limit_reported_while_halting)
{
	engage(&S);
	S.estop = true;
	sim_step(&S);
	zassert_equal(S.c.state, MC_ST_ESTOP);
	uint32_t events = S.c.event_count;

	S.pl[0].theta = (S.c.p.hard_hi + 0.001 - S.g.l_min) / (S.pl[0].p.lead / (2.0 * MC_PI));
	sim_step(&S);
	sim_step(&S);
	zassert_equal(S.c.event_count, events + 1);
	zassert_equal(S.c.event, MC_FAULT_HARD_LIMIT);
	zassert_equal(S.c.event_leg, 0);
	zassert_equal(S.c.state, MC_ST_ESTOP);
	zassert_equal(S.c.fault, MC_FAULT_ESTOP);
}

/* A NaN feed-forward must not become a full-scale current: 0 A and a fault at once. */
ZTEST(ctl, test_nan_in_the_servo_bank_trips_internal)
{
	mc_servo_bank *b = &S.b;
	mc_servo_cmd cmd = {.seq = 1, .on = true};
	double pos[MC_LEGS], cur[MC_LEGS];

	for (int i = 0; i < MC_LEGS; i++) {
		pos[i] = S.g.l0;
		cmd.prev[i] = S.g.l0;
		cmd.next[i] = S.g.l0;
		cmd.force[i] = 919.0;
	}
	cmd.force[3] = NAN;
	mc_bank_step(b, &cmd, pos, cur);
	zassert_equal(cur[3], 0.0);
	zassert_true(b->axis[3].invalid);
	zassert_false(b->axis[3].saturated);
	zassert_equal(b->fb.trip, MC_FAULT_INTERNAL);
	zassert_equal(b->fb.trip_leg, 3);
	zassert_equal(b->fb.leg_trip[3], MC_FAULT_INTERNAL);
	zassert_equal(b->fb.leg_trip[0], 0);
	zassert_true(isfinite(cur[0]) && (cur[0] > 0.0));
}
