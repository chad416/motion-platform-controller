/*
 * SPEC F09, F10: CAN payloads against vectors encoded from protocol/motion.dbc.
 */
#include <math.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "proto.h"
#include "proto_vectors.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ZTEST_SUITE(proto, NULL, NULL, NULL, NULL, NULL);

static void same(const uint8_t *a, const uint8_t *b, size_t n, const char *what)
{
	for (size_t i = 0; i < n; i++) {
		zassert_equal(a[i], b[i], "%s byte %zu: 0x%02x != 0x%02x (DBC)", what, i, a[i], b[i]);
	}
}

ZTEST(proto, test_ids_match_the_dbc)
{
	zassert_equal(PROTO_ID_VEHICLE_F, VEC_VEHICLE_F_ID);
	zassert_equal(PROTO_ID_VEHICLE_W, VEC_VEHICLE_W_ID);
	zassert_equal(PROTO_ID_CMD, VEC_CMD_SET_ID);
	zassert_equal(PROTO_ID_CMD_ACK, VEC_CMD_ACK_ID);
	zassert_equal(PROTO_ID_FAULT, VEC_FAULT_ID);
	zassert_equal(PROTO_ID_STATUS, VEC_STATUS_ID);
	zassert_equal(PROTO_ID_POSE_T, VEC_POSE_T_ID);
	zassert_equal(PROTO_ID_POSE_R, VEC_POSE_R_ID);
	zassert_equal(PROTO_ID_LEG_POS_A, VEC_LEG_POS_A_ID);
	zassert_equal(PROTO_ID_LEG_CUR_B, VEC_LEG_CUR_B_ID);
	zassert_equal(PROTO_ID_TIMING, VEC_TIMING_ID);
}

ZTEST(proto, test_vehicle_state_round_trip)
{
	uint8_t b[8];
	double f[3], w[3];
	uint16_t seq;
	const double f_in[3] = {1.5, -2.25, 9.81};
	const double w_in[3] = {0.125, -0.05, 0.3};

	zassert_equal(proto_encode_vehicle_f(f_in, 42, b), 8);
	same(b, VEC_VEHICLE_F, 8, "VEHICLE_F");
	zassert_true(proto_decode_vehicle_f(VEC_VEHICLE_F, 8, f, &seq));
	zassert_equal(seq, 42);
	for (int i = 0; i < 3; i++) {
		zassert_within(f[i], f_in[i], 0.001);
	}
	zassert_equal(proto_encode_vehicle_w(w_in, 65535, b), 8);
	same(b, VEC_VEHICLE_W, 8, "VEHICLE_W");
	zassert_true(proto_decode_vehicle_w(VEC_VEHICLE_W, 8, w, &seq));
	zassert_equal(seq, 65535);
	for (int i = 0; i < 3; i++) {
		zassert_within(w[i], w_in[i], 0.00005);
	}
	zassert_false(proto_decode_vehicle_f(VEC_VEHICLE_F, 7, f, &seq), "short frame");

	/* saturation: a 7 g cue clips at the scale limit instead of wrapping */
	const double big[3] = {70.0, -70.0, 0.0};

	(void)proto_encode_vehicle_f(big, 0, b);
	zassert_true(proto_decode_vehicle_f(b, 8, f, &seq));
	zassert_within(f[0], 65.534, 1e-9);
	zassert_within(f[1], -65.536, 1e-9);
}

ZTEST(proto, test_cmd_and_ack)
{
	struct proto_cmd c;
	uint8_t b[8];

	zassert_true(proto_decode_cmd(VEC_CMD_SET, 7, &c));
	zassert_equal(c.opcode, PROTO_OP_SET_PARAM);
	zassert_equal(c.key, 1);
	zassert_equal(c.value, 450);
	zassert_equal(proto_encode_cmd(&c, b), 7);
	same(b, VEC_CMD_SET, 7, "CMD");

	zassert_true(proto_decode_cmd(VEC_CMD_NEG, 7, &c));
	zassert_equal(c.value, -1);

	const uint8_t not_sf[8] = {0x10, 0x06, 5, 1, 0, 0, 0, 0};

	zassert_false(proto_decode_cmd(not_sf, 8, &c), "first frame, not a single frame");
	zassert_false(proto_decode_cmd(VEC_CMD_SET, 6, &c), "short");

	const struct proto_cmd_ack a = {.opcode = 6, .key = 1, .status = 0, .value = 450};
	struct proto_cmd_ack a2;

	zassert_equal(proto_encode_cmd_ack(&a, b), 8);
	same(b, VEC_CMD_ACK, 8, "CMD_ACK");
	zassert_true(proto_decode_cmd_ack(b, 8, &a2));
	zassert_equal(a2.value, 450);
}

ZTEST(proto, test_status_frames)
{
	uint8_t b[8];
	const struct proto_fault f = {.code = 1, .leg = 3, .state = 6, .count = 2, .value = -5123};
	const struct proto_status s = {.state = 4, .saturated = true, .fk_ok = true,
				       .comms_lost = true, .ws_scale = 0.41, .event = 7,
				       .seq = 1000};
	const struct proto_timing t = {312, 1450, 0, 2};
	const double xyz[3] = {0.0825, -0.0005, 0.07};
	const double rpy[3] = {5.25 * M_PI / 180.0, -8.75 * M_PI / 180.0, 2.5 * M_PI / 180.0};
	const double stroke[3] = {0.01, 0.125, 0.24};
	const double cur[3] = {2.95, -0.97, 20.0};

	(void)proto_encode_fault(&f, b);
	same(b, VEC_FAULT, 8, "FAULT");
	(void)proto_encode_status(&s, b);
	same(b, VEC_STATUS, 8, "STATUS");
	(void)proto_encode_pose_t(xyz, 5, b);
	same(b, VEC_POSE_T, 8, "POSE_T");
	(void)proto_encode_pose_r(rpy, 5, b);
	same(b, VEC_POSE_R, 8, "POSE_R");
	(void)proto_encode_leg_pos(stroke, 7, b);
	same(b, VEC_LEG_POS_A, 8, "LEG_POS_A");
	(void)proto_encode_leg_cur(cur, 7, b);
	same(b, VEC_LEG_CUR_B, 8, "LEG_CUR_B");
	(void)proto_encode_timing(&t, b);
	same(b, VEC_TIMING, 8, "TIMING");

	/* a leg resting on the stop (tiny negative stroke) reads 0, not 65535 */
	const double low[3] = {-1e-7, 0.0, 0.7};

	(void)proto_encode_leg_pos(low, 0, b);
	zassert_equal(b[0] | (b[1] << 8), 0);
	zassert_equal(b[4] | (b[5] << 8), 65535, "saturates high");
}
