/*
 * Controller core: safety state machine, leg setpoint generation and the leg
 * servos with their fault monitors. SPEC F05, F06, F08, F11-F13.
 *
 * Plain C with no RTOS calls, so the firmware, the native_sim unit tests and
 * the Python closed loop (ctypes) run the same code. Two halves exchange two
 * small structs, so the firmware can run them in two threads and copy the
 * structs under a lock:
 *
 *   mc_ctl_step()   cueing rate (500 Hz): commands, E-stop, heartbeat, faults,
 *                   state machine, setpoints (washout -> workspace -> IK ->
 *                   leg shaping, or S-curve moves), FK pose, gravity load
 *                   -> mc_servo_cmd
 *   mc_bank_step()  servo rate (1 kHz): six cascaded servos, following-error
 *                   and current-saturation monitors -> mc_servo_fb
 *
 * States (F08): OFF -> HOMING -> PARKED -> ENGAGING -> ENGAGED -> PARKING ->
 * PARKED, plus FAULT and ESTOP. Every fault and the E-stop first stop the legs
 * at the acceleration limit, then descend to park at the descent speed. RESET
 * is accepted only once parked (and with the E-stop released).
 */
#ifndef MCUE_CTL_H_
#define MCUE_CTL_H_

#include <stdbool.h>
#include <stdint.h>

#include "mcue/geometry.h"
#include "mcue/servo.h"
#include "mcue/shaping.h"
#include "mcue/washout.h"

typedef enum {
	MC_ST_OFF = 0,
	MC_ST_HOMING = 1,
	MC_ST_PARKED = 2,
	MC_ST_ENGAGING = 3,
	MC_ST_ENGAGED = 4,
	MC_ST_PARKING = 5,
	MC_ST_FAULT = 6,
	MC_ST_ESTOP = 7,
} mc_state;

typedef enum {
	MC_FAULT_NONE = 0,
	MC_FAULT_FOLLOWING = 1,  /* following error over the limit for too long */
	MC_FAULT_HARD_LIMIT = 2, /* a leg outside its hard limits */
	MC_FAULT_CURRENT = 3,    /* current saturated for too long */
	MC_FAULT_OVERRUN = 4,    /* loop overruns in a row (reported by the firmware) */
	MC_FAULT_HOMING = 5,     /* encoder reading implausible at ENABLE */
	MC_FAULT_ESTOP = 6,
	MC_FAULT_COMMS_LOST = 7, /* an event, not a FAULT state: washout, then PARKING */
} mc_fault;

typedef enum {
	MC_CMD_ENABLE = 1,
	MC_CMD_ENGAGE = 2,
	MC_CMD_PARK = 3,
	MC_CMD_RESET_FAULT = 4,
} mc_cmd;

typedef enum {
	MC_CST_OK = 0,
	MC_CST_BAD_STATE = 1, /* not allowed in this state */
	MC_CST_ESTOP = 2,     /* E-stop input active */
	MC_CST_BAD_KEY = 3,
	MC_CST_BAD_VALUE = 4,
} mc_cmd_status;

/* Parameter keys for SET_PARAM / GET_PARAM (values are scaled integers). */
typedef enum {
	MC_PARAM_FORCE_GAIN = 1,      /* x 1000 */
	MC_PARAM_TILT_RATE_LIMIT = 2, /* mdeg/s */
	MC_PARAM_ROT_GAIN = 3,        /* x 1000 */
	MC_PARAM_SURGE_SWAY_WN = 4,   /* mrad/s */
	MC_PARAM_SURGE_SWAY_WB = 5,   /* mrad/s */
	MC_PARAM_HEAVE_WN = 6,        /* mrad/s */
	MC_PARAM_TILT_WN = 7,         /* mrad/s */
} mc_param_key;

/* cue -> servo, once per cueing step */
typedef struct {
	uint32_t seq;             /* +1 every cueing step */
	uint32_t fault_epoch;     /* changes on RESET: the bank clears its trip */
	uint32_t stats_epoch;     /* changes to restart the tracking statistics */
	bool on;                  /* servos active */
	bool track;               /* collect tracking statistics (cueing) */
	double prev[MC_LEGS];     /* setpoint one cueing period ago */
	double next[MC_LEGS];     /* this setpoint (reached one cueing period later) */
	double acc[MC_LEGS];      /* setpoint acceleration, m/s^2 */
	double force[MC_LEGS];    /* gravity feed-forward, N */
	bool released[MC_LEGS];   /* faulted leg: integral cleared, gravity feed-forward only */
} mc_servo_cmd;

/* servo -> cue, every servo step */
typedef struct {
	double pos[MC_LEGS];      /* measured (encoder) */
	double current[MC_LEGS];  /* commanded, A */
	double ferr[MC_LEGS];     /* following error, m */
	int trip;                 /* MC_FAULT_FOLLOWING / MC_FAULT_CURRENT / 0, latched */
	int trip_leg;             /* 0..5 */
	double trip_value;        /* m or A */
	uint32_t fault_epoch;     /* the cmd fault_epoch this feedback belongs to */
} mc_servo_fb;

typedef struct {
	mc_servo axis[MC_LEGS];
	mc_servo_params sp;
	double cue_dt;
	int ferr_ticks_max;   /* following-error time, servo steps */
	int sat_ticks_max;    /* current-saturation time, servo steps */
	double ferr_max;      /* m */
	bool on;
	uint32_t seq, fault_epoch, stats_epoch;
	int sub;              /* servo steps since the last cueing step */
	int ferr_ticks[MC_LEGS];
	int sat_ticks[MC_LEGS];
	/* tracking error while cmd.track: sum of squares and peak, per leg */
	double sum2[MC_LEGS];
	double peak[MC_LEGS];
	uint32_t n;
	mc_servo_fb fb;
} mc_servo_bank;

typedef struct {
	double engage_time;      /* s, minimum S-curve time for ENGAGING */
	double park_time;        /* s, minimum S-curve time for PARKING */
	double stop_dwell;       /* s, standstill after a FAULT / ESTOP stop, before the descent */
	double descent_speed;    /* m/s, peak leg speed for homing and fault descent */
	double hard_lo, hard_hi; /* m, leg hard limits (fault outside) */
	double heartbeat_timeout;/* s */
	double comms_return;     /* s, longest washout to neutral after comms loss */
	double leg_speed;        /* m/s */
	double leg_accel;        /* m/s^2 */
	double cue_lo, cue_hi;   /* m, leg setpoint bounds while cueing: the soft limits less a
				  * margin for the servo tracking error, so the real leg stays
				  * inside the soft limits too (P03) */
	double park[MC_LEGS];    /* parked leg lengths (lower soft limit) */
	double neutral[MC_LEGS]; /* l0 */
	double mass;             /* payload, kg */
	mc_vec3 com;             /* payload centre of mass, platform frame */
	double rate_hz;          /* cueing rate */
	int input_steps;         /* cueing steps per vehicle sample (500 / 100 Hz) */
} mc_ctl_params;

typedef enum {
	MC_PH_OFF = 0,  /* servos off */
	MC_PH_HOLD = 1, /* hold the last setpoint */
	MC_PH_STOP = 2, /* brake every leg to rest at the acceleration limit */
	MC_PH_TRAJ = 3, /* S-curve move */
	MC_PH_CUE = 4,  /* washout -> workspace -> IK -> leg shaping */
	MC_PH_DWELL = 5, /* standstill after a FAULT / ESTOP stop, then the descent */
} mc_phase;

typedef struct {
	mc_geometry g;
	mc_ctl_params p;
	mc_washout_params wop;
	mc_washout wo;
	mc_leg_shaper sh;
	mc_leg_traj tr;
	double dt;

	mc_state state;
	mc_phase phase;
	double sp[MC_LEGS];     /* current leg setpoints */
	double sp_vel[MC_LEGS];
	double sp_acc[MC_LEGS];
	double stop_target[MC_LEGS];
	uint32_t dwell_steps;   /* cueing steps left in MC_PH_DWELL */
	mc_servo_cmd cmd;       /* last command sent */

	/* measurements and status */
	double meas[MC_LEGS];
	mc_pose pose;           /* from FK of the measured legs */
	bool fk_ok;
	double force[MC_LEGS];  /* statics at the FK pose, N */
	mc_pose cue_pose;       /* washout output */
	mc_pose cmd_pose;       /* after workspace limiting */
	double ws_scale;
	bool ws_saturated;

	mc_fault fault;         /* latched fault (FAULT / ESTOP), or NONE */
	int fault_leg;          /* 0..5, or -1 */
	double fault_value;
	mc_fault event;         /* last event for the host (fault or COMMS_LOST) */
	uint32_t event_count;   /* +1 per event: the firmware sends a FAULT frame */
	bool estop;
	bool comms_lost;
	bool descended;         /* FAULT / ESTOP: back at park */

	/* vehicle input (f xyz, w xyz) and its interpolation to the cueing rate */
	double in_from[6], in_to[6], in_now[6];
	int in_step;
	uint32_t since_input;   /* cueing steps since the last vehicle sample */
	uint32_t lost_steps;    /* cueing steps since comms loss */
	uint32_t steps;
} mc_ctl;

void mc_ctl_default_params(mc_ctl_params *p, const mc_geometry *g);

/* Starts in OFF. Returns false if the washout parameters are unstable. */
bool mc_ctl_init(mc_ctl *c, const mc_geometry *g, const mc_ctl_params *p,
		 const mc_washout_params *wop);

/* A command from the host or the shell. Returns mc_cmd_status. */
int mc_ctl_command(mc_ctl *c, mc_cmd cmd);

/* A new vehicle sample (specific force xyz, angular rate xyz). */
void mc_ctl_vehicle(mc_ctl *c, const double f[3], const double w[3]);

/*
 * One cueing step. fb: the latest servo feedback; estop: E-stop input
 * active; overrun: the firmware's loop-overrun limit was reached.
 */
void mc_ctl_step(mc_ctl *c, const mc_servo_fb *fb, bool estop, bool overrun, mc_servo_cmd *out);

int mc_ctl_set_param(mc_ctl *c, int key, int32_t value);
int mc_ctl_get_param(const mc_ctl *c, int key, int32_t *value);

const char *mc_state_name(mc_state s);
const char *mc_fault_name(mc_fault f);

/* ---- servo bank ---------------------------------------------------------- */

void mc_bank_init(mc_servo_bank *b, const mc_servo_params *sp, const mc_ctl_params *cp);

/* One servo step: measured positions in, currents out; b->fb updated. */
void mc_bank_step(mc_servo_bank *b, const mc_servo_cmd *cmd, const double pos[MC_LEGS],
		  double current[MC_LEGS]);

/* Tracking error since the last stats_epoch: RMS and peak per leg (m). */
void mc_bank_tracking(const mc_servo_bank *b, double rms[MC_LEGS], double peak[MC_LEGS]);

#endif /* MCUE_CTL_H_ */
