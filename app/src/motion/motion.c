#include "motion.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "act.h"
#include "mcue/geometry.h"
#include "mcue/plant.h"
#include "mcue/platform_config.h"
#include "mcue/servo.h"
#include "proto.h"
#include "wdt.h"

LOG_MODULE_REGISTER(motion, LOG_LEVEL_INF);

#define SERVO_PERIOD_US  1000
#define SERVO_STACK      3072
#define CUE_STACK        6144
#define SERVO_PRIO       1 /* preempts the cue thread */
#define CUE_PRIO         2
#define LOOP_WDT_MS      100U
#define OVERRUN_LIMIT    ((uint32_t)MC_CFG_SAF_OVERRUN_LIMIT)

struct vehicle_frame {
	uint16_t id;
	uint8_t len;
	uint8_t data[8];
};

K_MSGQ_DEFINE(vehicle_q, sizeof(struct vehicle_frame), 16, 4);
K_SEM_DEFINE(servo_sem, 0, 4);
K_SEM_DEFINE(cue_sem, 0, 4);
K_MUTEX_DEFINE(ctl_lock);
K_THREAD_STACK_DEFINE(servo_stack, SERVO_STACK);
K_THREAD_STACK_DEFINE(cue_stack, CUE_STACK);

static struct k_thread servo_thread;
static struct k_thread cue_thread;
static struct k_timer servo_timer;

static mc_geometry geo;
static mc_ctl ctl;           /* cue thread, under ctl_lock */
static mc_servo_bank bank;   /* servo thread, under xlock */
static struct k_spinlock xlock;
static mc_servo_cmd xcmd;    /* cue -> servo */
static mc_servo_fb xfb;      /* servo -> cue */

static atomic_t estop_in;

struct loop_stats {
	uint32_t cycles, min_us, max_us, overruns, consec;
	uint64_t sum_us;
};

static struct loop_stats servo_st, cue_st; /* written by their own thread */
static uint32_t vehicle_samples, vehicle_dropped;

static void stats_add(struct loop_stats *s, uint32_t us, bool overrun)
{
	s->cycles++;
	s->sum_us += us;
	s->min_us = (s->cycles == 1U) ? us : MIN(s->min_us, us);
	s->max_us = MAX(s->max_us, us);
	if (overrun) {
		s->overruns++;
		s->consec++;
	} else {
		s->consec = 0U;
	}
}

static void servo_tick(struct k_timer *t)
{
	ARG_UNUSED(t);
	k_sem_give(&servo_sem);
}

static void servo_main(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int wdt = wdt_register("servo", LOOP_WDT_MS);
	uint32_t ticks = 0U;

	for (;;) {
		(void)k_sem_take(&servo_sem, K_FOREVER);
		uint32_t t0 = k_cycle_get_32();
		double pos[MC_LEGS], cur[MC_LEGS];

		act_read(pos);

		k_spinlock_key_t key = k_spin_lock(&xlock);

		mc_bank_step(&bank, &xcmd, pos, cur);
		xfb = bank.fb;
		k_spin_unlock(&xlock, key);

		act_write(cur);

		if ((++ticks & 1U) == 0U) {
			k_sem_give(&cue_sem);
		}
		wdt_feed(wdt);
		stats_add(&servo_st, k_cyc_to_us_floor32(k_cycle_get_32() - t0),
			  k_sem_count_get(&servo_sem) > 0U);
	}
}

/* ---- vehicle input: pair VEHICLE_F and VEHICLE_W by sequence number -------- */

static struct {
	bool have_f, have_w;
	uint16_t seq_f, seq_w;
	double f[3], w[3];
} pend;

void motion_post_vehicle(uint16_t id, const uint8_t *data, uint8_t len)
{
	struct vehicle_frame vf = {.id = id, .len = MIN(len, 8U)};

	(void)memcpy(vf.data, data, vf.len);
	(void)k_msgq_put(&vehicle_q, &vf, K_NO_WAIT); /* a lost sample is bridged by the heartbeat */
}

static void take_vehicle_frames(void)
{
	struct vehicle_frame vf;

	while (k_msgq_get(&vehicle_q, &vf, K_NO_WAIT) == 0) {
		uint16_t seq;

		if (vf.id == PROTO_ID_VEHICLE_F) {
			if (!proto_decode_vehicle_f(vf.data, vf.len, pend.f, &seq)) {
				continue;
			}
			vehicle_dropped += pend.have_f ? 1U : 0U;
			pend.have_f = true;
			pend.seq_f = seq;
		} else {
			if (!proto_decode_vehicle_w(vf.data, vf.len, pend.w, &seq)) {
				continue;
			}
			vehicle_dropped += pend.have_w ? 1U : 0U;
			pend.have_w = true;
			pend.seq_w = seq;
		}
		if (pend.have_f && pend.have_w) {
			if (pend.seq_f == pend.seq_w) {
				mc_ctl_vehicle(&ctl, pend.f, pend.w);
				vehicle_samples++;
				pend.have_f = false;
				pend.have_w = false;
			} else if ((int16_t)(pend.seq_f - pend.seq_w) > 0) {
				pend.have_w = false; /* W is older: drop it */
				vehicle_dropped++;
			} else {
				pend.have_f = false;
				vehicle_dropped++;
			}
		}
	}
}

/* ---- cueing thread ------------------------------------------------------- */

static void report_changes(mc_state *last_state, uint32_t *last_events)
{
	if (ctl.event_count != *last_events) {
		*last_events = ctl.event_count;
		if (ctl.event == MC_FAULT_COMMS_LOST) {
			LOG_WRN("EVENT COMMS_LOST");
		} else {
			LOG_WRN("FAULT %s leg=%d value=%d", mc_fault_name(ctl.event),
				ctl.fault_leg + 1, (int)(ctl.fault_value * 1e6));
		}
	}
	if (ctl.state != *last_state) {
		*last_state = ctl.state;
		LOG_INF("STATE %s", mc_state_name(ctl.state));
	}
}

static void cue_main(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int wdt = wdt_register("cue", LOOP_WDT_MS);
	mc_state last_state = MC_ST_OFF;
	uint32_t last_events = 0U;

	for (;;) {
		(void)k_sem_take(&cue_sem, K_FOREVER);
		uint32_t t0 = k_cycle_get_32();
		mc_servo_fb fb;
		mc_servo_cmd cmd;

		k_spinlock_key_t key = k_spin_lock(&xlock);

		fb = xfb;
		k_spin_unlock(&xlock, key);

		bool overrun = (servo_st.consec > OVERRUN_LIMIT) || (cue_st.consec > OVERRUN_LIMIT);

		(void)k_mutex_lock(&ctl_lock, K_FOREVER);
		take_vehicle_frames();
		mc_ctl_step(&ctl, &fb, atomic_get(&estop_in) != 0, overrun, &cmd);
		report_changes(&last_state, &last_events);
		(void)k_mutex_unlock(&ctl_lock);

		key = k_spin_lock(&xlock);
		xcmd = cmd;
		k_spin_unlock(&xlock, key);

		act_sim_update_load();

		wdt_feed(wdt);
		stats_add(&cue_st, k_cyc_to_us_floor32(k_cycle_get_32() - t0),
			  k_sem_count_get(&cue_sem) > 0U);
	}
}

/* ---- API ----------------------------------------------------------------- */

int motion_start(void)
{
	mc_geometry_params gp;
	mc_ctl_params cp;
	mc_washout_params wp;
	mc_plant_params pp;
	mc_servo_params sp;
	double pos[MC_LEGS];

	mc_geometry_default_params(&gp);
	if (!mc_geometry_build(&geo, &gp)) {
		LOG_ERR("invalid geometry");
		return -EINVAL;
	}
	mc_ctl_default_params(&cp, &geo);
	mc_washout_default_params(&wp);
	if (!mc_ctl_init(&ctl, &geo, &cp, &wp)) {
		LOG_ERR("washout parameters unstable");
		return -EINVAL;
	}
	mc_plant_default_params(&pp, &geo);
	mc_servo_default_params(&sp, &pp);
	mc_bank_init(&bank, &sp, &cp);
	(void)act_init(&geo);

	/* first feedback: the encoders as they are */
	act_read(pos);
	(void)memcpy(xfb.pos, pos, sizeof(pos));
	bank.fb = xfb;

	(void)k_thread_create(&servo_thread, servo_stack, K_THREAD_STACK_SIZEOF(servo_stack),
			      servo_main, NULL, NULL, NULL, SERVO_PRIO, K_FP_REGS, K_NO_WAIT);
	k_thread_name_set(&servo_thread, "servo");
	(void)k_thread_create(&cue_thread, cue_stack, K_THREAD_STACK_SIZEOF(cue_stack), cue_main,
			      NULL, NULL, NULL, CUE_PRIO, K_FP_REGS, K_NO_WAIT);
	k_thread_name_set(&cue_thread, "cue");

	k_timer_init(&servo_timer, servo_tick, NULL);
	k_timer_start(&servo_timer, K_USEC(SERVO_PERIOD_US), K_USEC(SERVO_PERIOD_US));
	LOG_INF("loops up: servo %d Hz, cueing %d Hz", 1000000 / SERVO_PERIOD_US,
		(int)MC_CFG_WO_RATE_HZ);
	return 0;
}

int motion_command(mc_cmd cmd)
{
	(void)k_mutex_lock(&ctl_lock, K_FOREVER);
	int st = mc_ctl_command(&ctl, cmd);
	(void)k_mutex_unlock(&ctl_lock);
	return st;
}

int motion_set_param(int key, int32_t value)
{
	(void)k_mutex_lock(&ctl_lock, K_FOREVER);
	int st = mc_ctl_set_param(&ctl, key, value);
	(void)k_mutex_unlock(&ctl_lock);
	return st;
}

int motion_get_param(int key, int32_t *value)
{
	(void)k_mutex_lock(&ctl_lock, K_FOREVER);
	int st = mc_ctl_get_param(&ctl, key, value);
	(void)k_mutex_unlock(&ctl_lock);
	return st;
}

void motion_set_estop(bool active)
{
	atomic_set(&estop_in, active ? 1 : 0);
}

void motion_get_status(struct motion_status *o)
{
	(void)k_mutex_lock(&ctl_lock, K_FOREVER);
	o->state = ctl.state;
	o->phase = ctl.phase;
	o->fault = ctl.fault;
	o->fault_leg = ctl.fault_leg;
	o->fault_value = ctl.fault_value;
	o->event = ctl.event;
	o->event_count = ctl.event_count;
	o->estop = ctl.estop;
	o->comms_lost = ctl.comms_lost;
	o->descended = ctl.descended;
	o->ws_saturated = ctl.ws_saturated;
	o->fk_ok = ctl.fk_ok;
	o->ws_scale = ctl.ws_scale;
	o->pose = ctl.pose;
	o->cue_pose = ctl.cue_pose;
	o->cmd_pose = ctl.cmd_pose;
	(void)memcpy(o->meas, ctl.meas, sizeof(o->meas));
	(void)memcpy(o->sp, ctl.sp, sizeof(o->sp));
	(void)memcpy(o->force, ctl.force, sizeof(o->force));
	(void)memcpy(o->in_now, ctl.in_now, sizeof(o->in_now));
	o->steps = ctl.steps;
	(void)k_mutex_unlock(&ctl_lock);

	k_spinlock_key_t key = k_spin_lock(&xlock);

	(void)memcpy(o->current, xfb.current, sizeof(o->current));
	(void)memcpy(o->ferr, xfb.ferr, sizeof(o->ferr));
	k_spin_unlock(&xlock, key);
}

static void loop_timing(const struct loop_stats *s, struct motion_loop_timing *o)
{
	o->cycles = s->cycles;
	o->min_us = s->min_us;
	o->max_us = s->max_us;
	o->mean_us = (s->cycles > 0U) ? ((double)s->sum_us / s->cycles) : 0.0;
	o->overruns = s->overruns;
}

void motion_get_timing(struct motion_timing *out)
{
	/* plain counters written by one thread each: a torn read only skews one sample */
	loop_timing(&servo_st, &out->servo);
	loop_timing(&cue_st, &out->cue);
	out->vehicle_samples = vehicle_samples;
	out->vehicle_dropped = vehicle_dropped;
}

void motion_reset_timing(void)
{
	(void)memset(&servo_st, 0, sizeof(servo_st));
	(void)memset(&cue_st, 0, sizeof(cue_st));
}

void motion_get_tracking(double rms[MC_LEGS], double peak[MC_LEGS], uint32_t *samples)
{
	k_spinlock_key_t key = k_spin_lock(&xlock);

	mc_bank_tracking(&bank, rms, peak);
	*samples = bank.n;
	k_spin_unlock(&xlock, key);
}

void motion_reset_tracking(void)
{
	(void)k_mutex_lock(&ctl_lock, K_FOREVER);
	ctl.cmd.stats_epoch++;
	(void)k_mutex_unlock(&ctl_lock);
}

void motion_get_washout(mc_washout_params *out)
{
	(void)k_mutex_lock(&ctl_lock, K_FOREVER);
	*out = ctl.wop;
	(void)k_mutex_unlock(&ctl_lock);
}
