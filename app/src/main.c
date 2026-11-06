/*
 * motion_fw: controller for a 6-DOF motion platform (SPEC.md).
 *
 * Start order:
 *  1. reset reason + task watchdog
 *  2. motion loops (servos off: the controller starts in OFF)
 *  3. CAN and the host reports
 *
 * E-stop: SW1. A real E-stop circuit is normally closed and opens when
 * pressed; the board button is normally open, so "pressed" stands for
 * "circuit open" here. The ESTOP state latches; RESET needs the button
 * released and the platform parked.
 */
#include <zephyr/app_version.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "canlink.h"
#include "host.h"
#include "motion.h"
#include "wdt.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static void on_input(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if ((evt->type == INPUT_EV_KEY) && (evt->code == INPUT_KEY_1)) {
		motion_set_estop(evt->value != 0);
	}
}

INPUT_CALLBACK_DEFINE(NULL, on_input, NULL);

int main(void)
{
	(void)wdt_init();
	LOG_INF("motion_fw %s reset=%s%s%s", APP_VERSION_STRING,
		wdt_reset_reason_name(wdt_reset_reason()),
		(wdt_last_stalled_thread()[0] != '\0') ? " thread=" : "", wdt_last_stalled_thread());

	if (motion_start() != 0) {
		LOG_ERR("motion start failed");
		return 0;
	}
	if (canlink_init() == 0) {
		(void)host_start();
	}
	LOG_INF("READY state=OFF");
	return 0;
}
