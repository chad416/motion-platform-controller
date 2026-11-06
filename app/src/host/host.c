#include "host.h"

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "canlink.h"
#include "mcue/geometry.h"
#include "motion.h"
#include "proto.h"
#include "wdt.h"

LOG_MODULE_REGISTER(host, LOG_LEVEL_INF);

#define PERIOD_MS     20
#define TIMING_EVERY  50 /* periods: 1 Hz */
#define HOST_STACK    2048
#define HOST_PRIO     5

K_THREAD_STACK_DEFINE(host_stack, HOST_STACK);
static struct k_thread host_thread;

static double stroke_base; /* lower end stop, m */

static void handle_cmd(const struct can_frame *f)
{
	struct proto_cmd c;
	struct proto_cmd_ack a = {0};
	uint8_t b[PROTO_FRAME_LEN];

	if (!proto_decode_cmd(f->data, can_dlc_to_bytes(f->dlc), &c)) {
		return;
	}
	a.opcode = c.opcode;
	a.key = c.key;
	switch (c.opcode) {
	case PROTO_OP_ENABLE:
		a.status = (uint8_t)motion_command(MC_CMD_ENABLE);
		break;
	case PROTO_OP_ENGAGE:
		a.status = (uint8_t)motion_command(MC_CMD_ENGAGE);
		break;
	case PROTO_OP_PARK:
		a.status = (uint8_t)motion_command(MC_CMD_PARK);
		break;
	case PROTO_OP_RESET_FAULT:
		a.status = (uint8_t)motion_command(MC_CMD_RESET_FAULT);
		break;
	case PROTO_OP_SET_PARAM:
		a.status = (uint8_t)motion_set_param(c.key, c.value);
		a.value = c.value;
		break;
	case PROTO_OP_GET_PARAM:
		a.status = (uint8_t)motion_get_param(c.key, &a.value);
		break;
	default:
		a.status = MC_CST_BAD_KEY;
		break;
	}
	LOG_INF("CMD op=%u key=%u value=%d -> status=%u", c.opcode, c.key, c.value, a.status);
	(void)canlink_send(PROTO_ID_CMD_ACK, b, proto_encode_cmd_ack(&a, b));
}

static int32_t fault_value_units(mc_fault f, double v)
{
	/* positions in um, currents in mA */
	return (int32_t)lround(v * ((f == MC_FAULT_CURRENT) ? 1e3 : 1e6));
}

static void send_reports(uint16_t seq, uint32_t *last_events)
{
	struct motion_status s;
	uint8_t b[PROTO_FRAME_LEN];

	motion_get_status(&s);

	if (s.event_count != *last_events) {
		*last_events = s.event_count;
		const struct proto_fault f = {
			.code = (uint8_t)s.event,
			.leg = (uint8_t)((s.event == s.fault) ? (s.fault_leg + 1) : 0),
			.state = (uint8_t)s.state,
			.count = (uint8_t)s.event_count,
			.value = (s.event == s.fault) ? fault_value_units(s.fault, s.fault_value) : 0,
		};

		(void)canlink_send(PROTO_ID_FAULT, b, proto_encode_fault(&f, b));
	}

	const struct proto_status st = {
		.state = (uint8_t)s.state,
		.saturated = s.ws_saturated,
		.fk_ok = s.fk_ok,
		.estop = s.estop,
		.comms_lost = s.comms_lost,
		.descended = s.descended,
		.fault = (uint8_t)s.fault,
		.fault_leg = (uint8_t)(s.fault_leg + 1),
		.ws_scale = s.ws_scale,
		.event = (uint8_t)s.event,
		.seq = seq,
	};
	const double xyz[3] = {s.pose.x, s.pose.y, s.pose.z};
	const double rpy[3] = {s.pose.roll, s.pose.pitch, s.pose.yaw};
	double stroke[MC_LEGS];

	for (int i = 0; i < MC_LEGS; i++) {
		stroke[i] = s.meas[i] - stroke_base;
	}
	(void)canlink_send(PROTO_ID_STATUS, b, proto_encode_status(&st, b));
	(void)canlink_send(PROTO_ID_POSE_T, b, proto_encode_pose_t(xyz, seq, b));
	(void)canlink_send(PROTO_ID_POSE_R, b, proto_encode_pose_r(rpy, seq, b));
	(void)canlink_send(PROTO_ID_LEG_POS_A, b, proto_encode_leg_pos(&stroke[0], seq, b));
	(void)canlink_send(PROTO_ID_LEG_POS_B, b, proto_encode_leg_pos(&stroke[3], seq, b));
	(void)canlink_send(PROTO_ID_LEG_CUR_A, b, proto_encode_leg_cur(&s.current[0], seq, b));
	(void)canlink_send(PROTO_ID_LEG_CUR_B, b, proto_encode_leg_cur(&s.current[3], seq, b));
}

static void send_timing(void)
{
	struct motion_timing t;
	uint8_t b[PROTO_FRAME_LEN];

	motion_get_timing(&t);
	const struct proto_timing m = {
		.servo_max_us = (uint16_t)MIN(t.servo.max_us, 65535U),
		.cue_max_us = (uint16_t)MIN(t.cue.max_us, 65535U),
		.servo_overruns = (uint16_t)MIN(t.servo.overruns, 65535U),
		.cue_overruns = (uint16_t)MIN(t.cue.overruns, 65535U),
	};

	(void)canlink_send(PROTO_ID_TIMING, b, proto_encode_timing(&m, b));
}

static void host_main(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int wdt = wdt_register("host", 500U);
	struct can_frame f;
	uint16_t seq = 0U;
	uint32_t last_events = 0U;
	int64_t next = k_uptime_get();

	for (;;) {
		while (k_msgq_get(canlink_cmd_queue(), &f, K_NO_WAIT) == 0) {
			handle_cmd(&f);
		}
		send_reports(seq, &last_events);
		if ((seq % TIMING_EVERY) == 0U) {
			send_timing();
		}
		seq++;
		wdt_feed(wdt);
		next += PERIOD_MS;
		/* answer commands as they arrive, but keep the 20 ms report grid */
		int64_t now = k_uptime_get();

		if (next > now) {
			struct k_poll_event ev = K_POLL_EVENT_INITIALIZER(
				K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
				canlink_cmd_queue());

			while ((next > now) && (k_poll(&ev, 1, K_MSEC(next - now)) == 0)) {
				ev.state = K_POLL_STATE_NOT_READY;
				while (k_msgq_get(canlink_cmd_queue(), &f, K_NO_WAIT) == 0) {
					handle_cmd(&f);
				}
				now = k_uptime_get();
			}
		}
	}
}

int host_start(void)
{
	mc_geometry_params gp;
	mc_geometry g;

	mc_geometry_default_params(&gp);
	(void)mc_geometry_build(&g, &gp);
	stroke_base = g.l_min;

	(void)k_thread_create(&host_thread, host_stack, K_THREAD_STACK_SIZEOF(host_stack),
			      host_main, NULL, NULL, NULL, HOST_PRIO, K_FP_REGS, K_NO_WAIT);
	k_thread_name_set(&host_thread, "host");
	return 0;
}
