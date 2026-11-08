/*
 * Diagnostic shell, SPEC F14:
 *   motion status|legs|pose|params|fault|plant|timing|track|can|version
 *   motion cmd enable|engage|park|reset     (same as the CAN commands)
 *   motion set|get <key> [value]             (SET_PARAM / GET_PARAM)
 *   motion seize <leg 1-6> <0|1>             (fault injection)
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/app_version.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "act.h"
#include "canlink.h"
#include "motion.h"
#include "wdt.h"

LOG_MODULE_REGISTER(shell_cmds, LOG_LEVEL_INF);

#define DEG (180.0 / 3.14159265358979323846)

static const char *const cmd_status_names[] = {"OK", "BAD_STATE", "ESTOP", "BAD_KEY",
					       "BAD_VALUE"};

static const char *cst(int st)
{
	return ((unsigned)st < ARRAY_SIZE(cmd_status_names)) ? cmd_status_names[st] : "?";
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	struct motion_status s;

	motion_get_status(&s);
	shell_print(sh, "state=%s phase=%d fault=%s leg=%d estop=%d comms_lost=%d saturated=%d "
			"scale=%.3f fk=%d descended=%d steps=%u",
		    mc_state_name(s.state), s.phase, mc_fault_name(s.fault), s.fault_leg + 1,
		    s.estop, s.comms_lost, s.ws_saturated, s.ws_scale, s.fk_ok, s.descended,
		    s.steps);
	shell_print(sh, "pose x=%.2f y=%.2f z=%.2f mm roll=%.3f pitch=%.3f yaw=%.3f deg",
		    s.pose.x * 1e3, s.pose.y * 1e3, s.pose.z * 1e3, s.pose.roll * DEG,
		    s.pose.pitch * DEG, s.pose.yaw * DEG);
	return 0;
}

static int cmd_legs(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	struct motion_status s;

	motion_get_status(&s);
	for (int i = 0; i < MC_LEGS; i++) {
		shell_print(sh, "leg%d length=%.3f mm setpoint=%.3f mm error=%.4f mm current=%.2f A "
				"load_ff=%.0f N",
			    i + 1, s.meas[i] * 1e3, s.sp[i] * 1e3, s.ferr[i] * 1e3, s.current[i],
			    s.force[i]);
	}
	return 0;
}

static void print_pose(const struct shell *sh, const char *name, const mc_pose *q)
{
	shell_print(sh, "%s x=%.2f y=%.2f z=%.2f mm roll=%.3f pitch=%.3f yaw=%.3f deg", name,
		    q->x * 1e3, q->y * 1e3, q->z * 1e3, q->roll * DEG, q->pitch * DEG, q->yaw * DEG);
}

static int cmd_pose(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	struct motion_status s;

	motion_get_status(&s);
	print_pose(sh, "fk ", &s.pose);
	print_pose(sh, "cue", &s.cue_pose);
	print_pose(sh, "cmd", &s.cmd_pose);
	shell_print(sh, "input f=%.3f %.3f %.3f m/s^2 w=%.4f %.4f %.4f rad/s scale=%.3f",
		    s.in_now[0], s.in_now[1], s.in_now[2], s.in_now[3], s.in_now[4], s.in_now[5],
		    s.ws_scale);
	return 0;
}

static int cmd_params(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	mc_washout_params w;

	motion_get_washout(&w);
	shell_print(sh, "force_gain=%.3f force_limit=%.2f surge_sway wn=%.3f zeta=%.3f wb=%.3f",
		    w.force_gain, w.force_limit, w.surge_sway_wn, w.surge_sway_zeta,
		    w.surge_sway_wb);
	shell_print(sh, "heave wn=%.3f zeta=%.3f wb=%.3f tilt wn=%.3f zeta=%.3f rate=%.3f deg/s "
			"accel=%.3f deg/s^2",
		    w.heave_wn, w.heave_zeta, w.heave_wb, w.tilt_wn, w.tilt_zeta,
		    w.tilt_rate_limit * DEG, w.tilt_accel_limit * DEG);
	shell_print(sh, "rot gain=%.3f wn=%.3f zeta=%.3f", w.rot_gain, w.rot_wn, w.rot_zeta);
	return 0;
}

static int cmd_fault(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	struct motion_status s;

	motion_get_status(&s);
	shell_print(sh, "fault=%s leg=%d value=%.6f last_event=%s events=%u state=%s descended=%d",
		    mc_fault_name(s.fault), s.fault_leg + 1, s.fault_value,
		    mc_fault_name(s.event), s.event_count, mc_state_name(s.state), s.descended);
	return 0;
}

static int cmd_plant(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	struct act_sim_state p;

	act_sim_get(&p);
	for (int i = 0; i < MC_LEGS; i++) {
		shell_print(sh, "leg%d true_length=%.4f mm speed=%.2f mm/s load=%.0f N current=%.2f A%s",
			    i + 1, p.length[i] * 1e3, p.speed[i] * 1e3, p.load[i], p.current[i],
			    p.seized[i] ? " SEIZED" : "");
	}
	return 0;
}

static int cmd_timing(const struct shell *sh, size_t argc, char **argv)
{
	struct motion_timing t;

	if ((argc > 1) && (strcmp(argv[1], "reset") == 0)) {
		motion_reset_timing();
		shell_print(sh, "timing reset");
		return 0;
	}
	motion_get_timing(&t);
	shell_print(sh, "TIMING servo cycles=%u min=%uus max=%uus mean=%.1fus overruns=%u",
		    t.servo.cycles, t.servo.min_us, t.servo.max_us, t.servo.mean_us,
		    t.servo.overruns);
	shell_print(sh, "TIMING cue cycles=%u min=%uus max=%uus mean=%.1fus overruns=%u",
		    t.cue.cycles, t.cue.min_us, t.cue.max_us, t.cue.mean_us, t.cue.overruns);
	shell_print(sh, "vehicle samples=%u dropped=%u stale=%u", t.vehicle_samples,
		    t.vehicle_dropped, t.vehicle_stale);
	return 0;
}

static int cmd_track(const struct shell *sh, size_t argc, char **argv)
{
	double rms[MC_LEGS], peak[MC_LEGS], w_rms = 0.0, w_peak = 0.0;
	uint32_t n;

	if ((argc > 1) && (strcmp(argv[1], "reset") == 0)) {
		motion_reset_tracking();
		shell_print(sh, "tracking reset");
		return 0;
	}
	motion_get_tracking(rms, peak, &n);
	for (int i = 0; i < MC_LEGS; i++) {
		w_rms = fmax(w_rms, rms[i]);
		w_peak = fmax(w_peak, peak[i]);
	}
	shell_print(sh, "TRACK samples=%u rms_um=%.1f peak_um=%.1f", n, w_rms * 1e6,
		    w_peak * 1e6);
	return 0;
}

static int cmd_cmd(const struct shell *sh, size_t argc, char **argv)
{
	static const struct {
		const char *name;
		mc_cmd cmd;
	} map[] = {{"enable", MC_CMD_ENABLE},
		   {"engage", MC_CMD_ENGAGE},
		   {"park", MC_CMD_PARK},
		   {"reset", MC_CMD_RESET_FAULT}};

	ARG_UNUSED(argc);
	for (size_t i = 0; i < ARRAY_SIZE(map); i++) {
		if (strcmp(argv[1], map[i].name) == 0) {
			int st = motion_command(map[i].cmd);

			shell_print(sh, "CMD %s %s", map[i].name, cst(st));
			return 0;
		}
	}
	shell_error(sh, "unknown command %s", argv[1]);
	return -EINVAL;
}

static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	int st = motion_set_param(atoi(argv[1]), (int32_t)strtol(argv[2], NULL, 0));

	shell_print(sh, "SET %s %s %s", argv[1], argv[2], cst(st));
	return 0;
}

static int cmd_get(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	int32_t v = 0;
	int st = motion_get_param(atoi(argv[1]), &v);

	shell_print(sh, "GET %s = %d %s", argv[1], v, cst(st));
	return 0;
}

static int cmd_can(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	struct canlink_stats c;

	canlink_get_stats(&c);
	shell_print(sh, "can up=%d state=%d tx_ok=%u tx_err=%u tx_overflow=%u bus_off=%u "
			"rx_vehicle=%u rx_cmds=%u rx_dropped=%u",
		    c.up, c.state, c.tx_ok, c.tx_errors, c.tx_overflows, c.bus_off_count,
		    c.rx_vehicle, c.rx_cmds, c.rx_dropped);
	return 0;
}

static int cmd_version(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "motion_fw %s reset=%s", APP_VERSION_STRING,
		    wdt_reset_reason_name(wdt_reset_reason()));
	return 0;
}

#ifdef CONFIG_MOTION_DEBUG_CMDS
static int cmd_seize(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	int leg = atoi(argv[1]);
	bool on = atoi(argv[2]) != 0;

	if (act_sim_seize(leg - 1, on) != 0) {
		shell_error(sh, "leg must be 1..6");
		return -EINVAL;
	}
	LOG_WRN("SEIZE leg=%d %s", leg, on ? "on" : "off"); /* timestamped */
	return 0;
}
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(
	motion_cmds, SHELL_CMD(status, NULL, "State, fault, flags, pose", cmd_status),
	SHELL_CMD(legs, NULL, "Leg lengths, setpoints, errors, currents", cmd_legs),
	SHELL_CMD(pose, NULL, "FK, washout and limited poses; input", cmd_pose),
	SHELL_CMD(params, NULL, "Washout parameters", cmd_params),
	SHELL_CMD(fault, NULL, "Latched fault and last event", cmd_fault),
	SHELL_CMD(plant, NULL, "Simulated actuators (true state)", cmd_plant),
	SHELL_CMD_ARG(timing, NULL, "Loop timing [reset]", cmd_timing, 1, 1),
	SHELL_CMD_ARG(track, NULL, "Tracking error while cueing [reset]", cmd_track, 1, 1),
	SHELL_CMD_ARG(cmd, NULL, "enable|engage|park|reset", cmd_cmd, 2, 0),
	SHELL_CMD_ARG(set, NULL, "<key> <value>: SET_PARAM", cmd_set, 3, 0),
	SHELL_CMD_ARG(get, NULL, "<key>: GET_PARAM", cmd_get, 2, 0),
	SHELL_CMD(can, NULL, "CAN statistics", cmd_can),
	SHELL_CMD(version, NULL, "Firmware version", cmd_version),
#ifdef CONFIG_MOTION_DEBUG_CMDS
	SHELL_CMD_ARG(seize, NULL, "<leg 1-6> <0|1>: seize a simulated leg", cmd_seize, 3, 0),
#endif
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(motion, &motion_cmds, "Motion platform controller", NULL);
