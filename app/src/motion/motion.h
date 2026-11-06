/*
 * Motion control threads around the mcue controller core. SPEC F05-F14, P01.
 *
 *   servo thread  1 kHz, k_timer: encoder -> six servos -> currents (HAL)
 *   cue thread    500 Hz, released by every second servo tick: vehicle input,
 *                 E-stop, state machine, setpoints, FK pose, gravity load
 *
 * The two exchange mc_servo_cmd / mc_servo_fb under a spinlock. Commands from
 * CAN and the shell run under the controller mutex, between cueing steps.
 */
#ifndef MOTION_MOTION_H_
#define MOTION_MOTION_H_

#include <stdbool.h>
#include <stdint.h>

#include "mcue/ctl.h"

int motion_start(void);

/* Commands (mc_cmd) and parameters; return mc_cmd_status. Thread context. */
int motion_command(mc_cmd cmd);
int motion_set_param(int key, int32_t value);
int motion_get_param(int key, int32_t *value);

/* A VEHICLE_F / VEHICLE_W frame (ISR-safe; decoded and paired in the cue thread). */
void motion_post_vehicle(uint16_t id, const uint8_t *data, uint8_t len);

/* E-stop input (active = circuit open). ISR-safe. */
void motion_set_estop(bool active);

struct motion_status {
	mc_state state;
	mc_phase phase;
	mc_fault fault;
	int fault_leg;      /* 0..5, -1 */
	double fault_value;
	mc_fault event;
	uint32_t event_count;
	bool estop, comms_lost, descended, ws_saturated, fk_ok;
	double ws_scale;
	mc_pose pose;       /* FK of the measured legs */
	mc_pose cue_pose;   /* washout output */
	mc_pose cmd_pose;   /* after workspace limiting */
	double meas[MC_LEGS];
	double sp[MC_LEGS];
	double force[MC_LEGS];
	double current[MC_LEGS];
	double ferr[MC_LEGS];
	double in_now[6];   /* interpolated vehicle input */
	uint32_t steps;
};

void motion_get_status(struct motion_status *out);

struct motion_loop_timing {
	uint32_t cycles;
	uint32_t min_us, max_us;
	double mean_us;
	uint32_t overruns;
};

struct motion_timing {
	struct motion_loop_timing servo, cue;
	uint32_t vehicle_samples;  /* complete (paired) samples */
	uint32_t vehicle_dropped;  /* frames that never found their pair */
};

void motion_get_timing(struct motion_timing *out);
void motion_reset_timing(void);

/* Tracking error (setpoint - encoder) while cueing, since the last reset. */
void motion_get_tracking(double rms[MC_LEGS], double peak[MC_LEGS], uint32_t *samples);
void motion_reset_tracking(void);

void motion_get_washout(mc_washout_params *out);

#endif /* MOTION_MOTION_H_ */
