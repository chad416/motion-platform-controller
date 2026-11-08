/*
 * WebAssembly entry points for the browser demo (web/index.html).
 *
 * The same closed loop as the unit tests (tests/unit/src/test_ctl.c) and the
 * firmware's simulation backend (app/src/hal/act_sim.c): the controller core
 * at the cueing rate (500 Hz), then two steps of the six leg servos and six
 * actuator models at 1 kHz. Only this file is web-specific; mcue/ is compiled
 * unchanged (scripts/build-web.sh).
 */
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "mcue/ctl.h"
#include "mcue/kin.h"
#include "mcue/plant.h"

#define WEB_EXPORT(name) __attribute__((export_name(name)))

/* Vehicle samples at 100 Hz, as the host sends them: every 5th cueing step. */
#define INPUT_EVERY 5

static struct {
	mc_geometry g;
	mc_ctl c;
	mc_servo_bank b;
	mc_plant pl[MC_LEGS];
	mc_servo_cmd cmd;
	mc_pose true_pose;
	double cur[MC_LEGS];
	bool estop;
	bool streaming;
	double f[3], w[3];
	uint32_t step_no;
	uint16_t seq; /* vehicle sample sequence number, as the host sends it */
	double t;
} S;

/*
 * Snapshot for the page, read as a Float64Array (layout in web/app.js, SNAP):
 * t, state, phase, fault, fault_leg, ws_saturated, ws_scale, comms_lost,
 * legs[6] (true), sp[6] (setpoint), pose[6] (true), cue[6] (washout output),
 * cmd[6] (after workspace limiting), current[6], platform joints[6][3] (base
 * frame, true pose), f[3], w[3].
 */
#define SNAP_LEN (8 + 6 * 6 + 18 + 6)
static double snap[SNAP_LEN];

/* Geometry: base joints[6][3], platform joints[6][3] (platform frame), h0, l0,
 * l_min, l_max, soft_lo, soft_hi, cue_lo, cue_hi.
 */
#define GEO_LEN (36 + 8)
static double geo[GEO_LEN];

static void update_loads(double load[MC_LEGS])
{
	double l[MC_LEGS];
	mc_fk_info info;

	for (int i = 0; i < MC_LEGS; i++) {
		l[i] = mc_plant_length(&S.pl[i]);
	}
	(void)mc_fk(&S.g, l, &S.true_pose, 30, 1e-12, &info);
	if (!info.converged || !mc_statics(&S.g, &S.true_pose, S.c.p.mass, &S.c.p.com, load)) {
		(void)memset(&S.true_pose, 0, sizeof(S.true_pose));
		(void)memset(load, 0, sizeof(double) * MC_LEGS);
	}
}

WEB_EXPORT("web_init")
int web_init(void)
{
	mc_geometry_params gp;
	mc_ctl_params cp;
	mc_washout_params wp;
	mc_plant_params pp;
	mc_servo_params sp;

	(void)memset(&S, 0, sizeof(S));
	mc_geometry_default_params(&gp);
	if (!mc_geometry_build(&S.g, &gp)) {
		return 0;
	}
	mc_ctl_default_params(&cp, &S.g);
	mc_washout_default_params(&wp);
	if (!mc_ctl_init(&S.c, &S.g, &cp, &wp)) {
		return 0;
	}
	mc_plant_default_params(&pp, &S.g);
	mc_servo_default_params(&sp, &pp);
	mc_bank_init(&S.b, &sp, &cp);
	for (int i = 0; i < MC_LEGS; i++) {
		mc_plant_init(&S.pl[i], &pp, S.g.l_min); /* power-up: on the lower stops */
		S.b.fb.pos[i] = mc_plant_encoder(&S.pl[i]);
	}
	S.f[2] = MC_GRAVITY;
	S.streaming = true;

	for (int i = 0; i < MC_LEGS; i++) {
		geo[3 * i + 0] = S.g.base[i].x;
		geo[3 * i + 1] = S.g.base[i].y;
		geo[3 * i + 2] = S.g.base[i].z;
		geo[18 + 3 * i + 0] = S.g.plat[i].x;
		geo[18 + 3 * i + 1] = S.g.plat[i].y;
		geo[18 + 3 * i + 2] = S.g.plat[i].z;
	}
	geo[36] = S.g.h0;
	geo[37] = S.g.l0;
	geo[38] = S.g.l_min;
	geo[39] = S.g.l_max;
	geo[40] = S.g.soft_lo;
	geo[41] = S.g.soft_hi;
	geo[42] = S.c.p.cue_lo;
	geo[43] = S.c.p.cue_hi;
	return 1;
}

WEB_EXPORT("web_geometry")
const double *web_geometry(void)
{
	return geo;
}

WEB_EXPORT("web_command")
int web_command(int cmd)
{
	return mc_ctl_command(&S.c, (mc_cmd)cmd);
}

WEB_EXPORT("web_set_param")
int web_set_param(int key, int value)
{
	return mc_ctl_set_param(&S.c, key, value);
}

WEB_EXPORT("web_get_param")
int web_get_param(int key)
{
	int32_t v = 0;

	return (mc_ctl_get_param(&S.c, key, &v) == MC_CST_OK) ? v : -1;
}

/* Vehicle state at the driver's head: specific force (m/s^2, +g on z at rest), body rates (rad/s). */
WEB_EXPORT("web_vehicle")
void web_vehicle(double fx, double fy, double fz, double wx, double wy, double wz)
{
	S.f[0] = fx;
	S.f[1] = fy;
	S.f[2] = fz;
	S.w[0] = wx;
	S.w[1] = wy;
	S.w[2] = wz;
}

/* Off: the vehicle samples stop, as if the CAN link to the host were cut. */
WEB_EXPORT("web_stream")
void web_stream(int on)
{
	S.streaming = (on != 0);
}

WEB_EXPORT("web_estop")
void web_estop(int on)
{
	S.estop = (on != 0);
}

/* n cueing periods: controller, then two servo periods (see test_ctl.c, sim_step). */
WEB_EXPORT("web_step")
void web_step(int n)
{
	for (int k = 0; k < n; k++) {
		double load[MC_LEGS];

		if (S.streaming && ((S.step_no % INPUT_EVERY) == 0U)) {
			S.seq++;
			(void)mc_ctl_vehicle(&S.c, S.f, S.w, S.seq);
		}
		S.step_no++;
		mc_ctl_step(&S.c, &S.b.fb, S.estop, false, &S.cmd);
		update_loads(load);
		for (int s = 0; s < 2; s++) {
			double pos[MC_LEGS];

			for (int i = 0; i < MC_LEGS; i++) {
				pos[i] = mc_plant_encoder(&S.pl[i]);
			}
			mc_bank_step(&S.b, &S.cmd, pos, S.cur);
			for (int i = 0; i < MC_LEGS; i++) {
				mc_plant_step(&S.pl[i], S.cur[i], load[i]);
			}
		}
		S.t += S.c.dt;
	}
}

static void put_pose(double *d, const mc_pose *q)
{
	d[0] = q->x;
	d[1] = q->y;
	d[2] = q->z;
	d[3] = q->roll;
	d[4] = q->pitch;
	d[5] = q->yaw;
}

WEB_EXPORT("web_snapshot")
const double *web_snapshot(void)
{
	double R[3][3];
	double *d = snap;

	d[0] = S.t;
	d[1] = S.c.state;
	d[2] = S.c.phase;
	d[3] = S.c.fault;
	d[4] = S.c.fault_leg;
	d[5] = S.c.ws_saturated ? 1.0 : 0.0;
	d[6] = S.c.ws_scale;
	d[7] = S.c.comms_lost ? 1.0 : 0.0;
	d += 8;
	for (int i = 0; i < MC_LEGS; i++) {
		d[i] = mc_plant_length(&S.pl[i]);
		d[6 + i] = S.c.sp[i];
	}
	d += 12;
	put_pose(d, &S.true_pose);
	put_pose(d + 6, &S.c.cue_pose);
	put_pose(d + 12, &S.c.cmd_pose);
	d += 18;
	for (int i = 0; i < MC_LEGS; i++) {
		d[i] = S.cur[i];
	}
	d += 6;
	mc_rotation(&S.true_pose, R);
	for (int i = 0; i < MC_LEGS; i++) {
		const mc_vec3 *b = &S.g.plat[i];

		d[3 * i + 0] = S.true_pose.x + R[0][0] * b->x + R[0][1] * b->y + R[0][2] * b->z;
		d[3 * i + 1] = S.true_pose.y + R[1][0] * b->x + R[1][1] * b->y + R[1][2] * b->z;
		d[3 * i + 2] = S.true_pose.z + S.g.h0 + R[2][0] * b->x + R[2][1] * b->y +
			       R[2][2] * b->z;
	}
	d += 18;
	for (int i = 0; i < 3; i++) {
		d[i] = S.f[i];
		d[3 + i] = S.w[i];
	}
	return snap;
}

WEB_EXPORT("web_state_name")
const char *web_state_name(int s)
{
	return mc_state_name((mc_state)s);
}

WEB_EXPORT("web_fault_name")
const char *web_fault_name(int f)
{
	return mc_fault_name((mc_fault)f);
}
