/*
 * Motion CAN protocol: frame IDs and payload encode/decode.
 * Source of truth for the layout: protocol/motion.dbc. Little-endian, scaled
 * integers; encoders round to nearest and saturate.
 *
 * CMD / CMD_ACK are ISO-TP (ISO 15765-2) single frames: byte 0 is the PCI
 * (0x0L, L = payload length), as in the S1 trainer firmware.
 *
 * Pure logic, no Zephyr device access: unit-tested on native_sim.
 */
#ifndef MOTION_PROTO_H_
#define MOTION_PROTO_H_

#include <stdbool.h>
#include <stdint.h>

/* host -> controller */
#define PROTO_ID_VEHICLE_F 0x100U
#define PROTO_ID_VEHICLE_W 0x101U
#define PROTO_ID_CMD       0x10FU
/* controller -> host (lower ID = higher priority) */
#define PROTO_ID_FAULT     0x180U
#define PROTO_ID_STATUS    0x181U
#define PROTO_ID_POSE_T    0x182U
#define PROTO_ID_POSE_R    0x183U
#define PROTO_ID_LEG_POS_A 0x184U
#define PROTO_ID_LEG_POS_B 0x185U
#define PROTO_ID_LEG_CUR_A 0x186U
#define PROTO_ID_LEG_CUR_B 0x187U
#define PROTO_ID_TIMING    0x188U
#define PROTO_ID_CMD_ACK   0x18FU

#define PROTO_FRAME_LEN 8U

enum proto_opcode {
	PROTO_OP_ENABLE = 1,
	PROTO_OP_ENGAGE = 2,
	PROTO_OP_PARK = 3,
	PROTO_OP_RESET_FAULT = 4,
	PROTO_OP_SET_PARAM = 5,
	PROTO_OP_GET_PARAM = 6,
};

struct proto_cmd {
	uint8_t opcode;
	uint8_t key;
	int32_t value;
};

struct proto_cmd_ack {
	uint8_t opcode;
	uint8_t key;
	uint8_t status; /* mc_cmd_status */
	int32_t value;
};

struct proto_fault {
	uint8_t code;  /* mc_fault */
	uint8_t leg;   /* 1..6, 0 = none */
	uint8_t state; /* mc_state */
	uint8_t count; /* events so far (wraps) */
	int32_t value; /* um (positions) or mA (currents) */
};

struct proto_status {
	uint8_t state;
	bool saturated;
	bool fk_ok;
	bool estop;
	bool comms_lost;
	bool descended;
	uint8_t fault;
	uint8_t fault_leg;
	double ws_scale;
	uint8_t event;
	uint16_t seq;
};

struct proto_timing {
	uint16_t servo_max_us;
	uint16_t cue_max_us;
	uint16_t servo_overruns;
	uint16_t cue_overruns;
};

/* Encoders write PROTO_FRAME_LEN bytes (CMD: 7) and return the length to send. */
uint8_t proto_encode_vehicle_f(const double f[3], uint16_t seq, uint8_t out[PROTO_FRAME_LEN]);
uint8_t proto_encode_vehicle_w(const double w[3], uint16_t seq, uint8_t out[PROTO_FRAME_LEN]);
uint8_t proto_encode_cmd(const struct proto_cmd *m, uint8_t out[PROTO_FRAME_LEN]);
uint8_t proto_encode_cmd_ack(const struct proto_cmd_ack *m, uint8_t out[PROTO_FRAME_LEN]);
uint8_t proto_encode_fault(const struct proto_fault *m, uint8_t out[PROTO_FRAME_LEN]);
uint8_t proto_encode_status(const struct proto_status *m, uint8_t out[PROTO_FRAME_LEN]);
/* position in m, angles in rad (sent in 0.01 deg) */
uint8_t proto_encode_pose_t(const double xyz[3], uint16_t seq, uint8_t out[PROTO_FRAME_LEN]);
uint8_t proto_encode_pose_r(const double rpy[3], uint16_t seq, uint8_t out[PROTO_FRAME_LEN]);
/* three legs: stroke above the lower end stop (m), current (A) */
uint8_t proto_encode_leg_pos(const double stroke[3], uint16_t seq, uint8_t out[PROTO_FRAME_LEN]);
uint8_t proto_encode_leg_cur(const double current[3], uint16_t seq, uint8_t out[PROTO_FRAME_LEN]);
uint8_t proto_encode_timing(const struct proto_timing *m, uint8_t out[PROTO_FRAME_LEN]);

bool proto_decode_vehicle_f(const uint8_t *data, uint8_t len, double f[3], uint16_t *seq);
bool proto_decode_vehicle_w(const uint8_t *data, uint8_t len, double w[3], uint16_t *seq);
/* A CMD frame: ISO-TP single frame with a 6-byte payload. */
bool proto_decode_cmd(const uint8_t *data, uint8_t len, struct proto_cmd *out);
bool proto_decode_cmd_ack(const uint8_t *data, uint8_t len, struct proto_cmd_ack *out);

#endif /* MOTION_PROTO_H_ */
