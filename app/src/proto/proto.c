#include "proto.h"

#include <math.h>
#include <string.h>

#include "mcue/geometry.h" /* MC_PI */

#define F_SCALE     0.002   /* m/s^2 */
#define W_SCALE     0.0001  /* rad/s */
#define POS_SCALE   0.0001  /* m */
#define ANG_SCALE   0.01    /* deg */
#define STROKE_SCALE 0.00001 /* m */
#define CUR_SCALE   0.01    /* A */
#define WS_SCALE    0.005

static int16_t sat_s16(double v, double scale)
{
	double r = round(v / scale);

	if (r > 32767.0) {
		return 32767;
	}
	if (r < -32768.0) {
		return -32768;
	}
	return (int16_t)r;
}

static uint16_t sat_u16(double v, double scale)
{
	double r = round(v / scale);

	if (r > 65535.0) {
		return 65535U;
	}
	if (r < 0.0) {
		return 0U;
	}
	return (uint16_t)r;
}

static void put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
	for (int i = 0; i < 4; i++) {
		p[i] = (uint8_t)(v >> (8 * i));
	}
}

static uint16_t get_u16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	       ((uint32_t)p[3] << 24);
}

/* three signed 16-bit values and a sequence number */
static uint8_t encode3(const double v[3], double scale, uint16_t seq, uint8_t out[8])
{
	for (int i = 0; i < 3; i++) {
		put_u16(&out[2 * i], (uint16_t)sat_s16(v[i], scale));
	}
	put_u16(&out[6], seq);
	return PROTO_FRAME_LEN;
}

static bool decode3(const uint8_t *d, uint8_t len, double scale, double v[3], uint16_t *seq)
{
	if (len != PROTO_FRAME_LEN) {
		return false;
	}
	for (int i = 0; i < 3; i++) {
		v[i] = (int16_t)get_u16(&d[2 * i]) * scale;
	}
	*seq = get_u16(&d[6]);
	return true;
}

uint8_t proto_encode_vehicle_f(const double f[3], uint16_t seq, uint8_t out[PROTO_FRAME_LEN])
{
	return encode3(f, F_SCALE, seq, out);
}

uint8_t proto_encode_vehicle_w(const double w[3], uint16_t seq, uint8_t out[PROTO_FRAME_LEN])
{
	return encode3(w, W_SCALE, seq, out);
}

bool proto_decode_vehicle_f(const uint8_t *data, uint8_t len, double f[3], uint16_t *seq)
{
	return decode3(data, len, F_SCALE, f, seq);
}

bool proto_decode_vehicle_w(const uint8_t *data, uint8_t len, double w[3], uint16_t *seq)
{
	return decode3(data, len, W_SCALE, w, seq);
}

uint8_t proto_encode_cmd(const struct proto_cmd *m, uint8_t out[PROTO_FRAME_LEN])
{
	(void)memset(out, 0, PROTO_FRAME_LEN);
	out[0] = 0x06U; /* ISO-TP single frame, 6 bytes */
	out[1] = m->opcode;
	out[2] = m->key;
	put_u32(&out[3], (uint32_t)m->value);
	return 7U;
}

bool proto_decode_cmd(const uint8_t *data, uint8_t len, struct proto_cmd *out)
{
	if ((len < 7U) || (data[0] != 0x06U)) {
		return false;
	}
	out->opcode = data[1];
	out->key = data[2];
	out->value = (int32_t)get_u32(&data[3]);
	return true;
}

uint8_t proto_encode_cmd_ack(const struct proto_cmd_ack *m, uint8_t out[PROTO_FRAME_LEN])
{
	out[0] = 0x07U; /* ISO-TP single frame, 7 bytes */
	out[1] = m->opcode;
	out[2] = m->key;
	out[3] = m->status;
	put_u32(&out[4], (uint32_t)m->value);
	return PROTO_FRAME_LEN;
}

bool proto_decode_cmd_ack(const uint8_t *data, uint8_t len, struct proto_cmd_ack *out)
{
	if ((len != PROTO_FRAME_LEN) || (data[0] != 0x07U)) {
		return false;
	}
	out->opcode = data[1];
	out->key = data[2];
	out->status = data[3];
	out->value = (int32_t)get_u32(&data[4]);
	return true;
}

uint8_t proto_encode_fault(const struct proto_fault *m, uint8_t out[PROTO_FRAME_LEN])
{
	out[0] = m->code;
	out[1] = m->leg;
	out[2] = m->state;
	out[3] = m->count;
	put_u32(&out[4], (uint32_t)m->value);
	return PROTO_FRAME_LEN;
}

uint8_t proto_encode_status(const struct proto_status *m, uint8_t out[PROTO_FRAME_LEN])
{
	out[0] = m->state;
	out[1] = (uint8_t)((m->saturated ? 0x01U : 0U) | (m->fk_ok ? 0x02U : 0U) |
			   (m->estop ? 0x04U : 0U) | (m->comms_lost ? 0x08U : 0U) |
			   (m->descended ? 0x10U : 0U));
	out[2] = m->fault;
	out[3] = m->fault_leg;

	uint16_t s = sat_u16(m->ws_scale, WS_SCALE);

	out[4] = (uint8_t)((s > 255U) ? 255U : s);
	out[5] = m->event;
	put_u16(&out[6], m->seq);
	return PROTO_FRAME_LEN;
}

uint8_t proto_encode_pose_t(const double xyz[3], uint16_t seq, uint8_t out[PROTO_FRAME_LEN])
{
	return encode3(xyz, POS_SCALE, seq, out);
}

uint8_t proto_encode_pose_r(const double rpy[3], uint16_t seq, uint8_t out[PROTO_FRAME_LEN])
{
	const double deg[3] = {rpy[0] * (180.0 / MC_PI), rpy[1] * (180.0 / MC_PI),
			       rpy[2] * (180.0 / MC_PI)};

	return encode3(deg, ANG_SCALE, seq, out);
}

uint8_t proto_encode_leg_pos(const double stroke[3], uint16_t seq, uint8_t out[PROTO_FRAME_LEN])
{
	for (int i = 0; i < 3; i++) {
		put_u16(&out[2 * i], sat_u16(stroke[i], STROKE_SCALE));
	}
	put_u16(&out[6], seq);
	return PROTO_FRAME_LEN;
}

uint8_t proto_encode_leg_cur(const double current[3], uint16_t seq, uint8_t out[PROTO_FRAME_LEN])
{
	return encode3(current, CUR_SCALE, seq, out);
}

uint8_t proto_encode_timing(const struct proto_timing *m, uint8_t out[PROTO_FRAME_LEN])
{
	put_u16(&out[0], m->servo_max_us);
	put_u16(&out[2], m->cue_max_us);
	put_u16(&out[4], m->servo_overruns);
	put_u16(&out[6], m->cue_overruns);
	return PROTO_FRAME_LEN;
}
