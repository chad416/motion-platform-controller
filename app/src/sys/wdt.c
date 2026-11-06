#include "wdt.h"

#include <string.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/task_wdt/task_wdt.h>

LOG_MODULE_REGISTER(wdt, LOG_LEVEL_INF);

#define NOTE_MAGIC   0x57444754U /* "WDGT" */
#define NAME_MAX_LEN 12

/* Survives a warm reset: the task-watchdog callback writes it before rebooting. */
struct reset_note {
	uint32_t magic;
	char thread[NAME_MAX_LEN];
};

static __noinit struct reset_note note;
static char stalled[NAME_MAX_LEN];
static enum reset_reason reason = RESET_REASON_POWER_ON;
static bool available;

static void on_timeout(int channel_id, void *user_data)
{
	const char *name = user_data;

	ARG_UNUSED(channel_id);
	note.magic = NOTE_MAGIC;
	(void)strncpy(note.thread, name, sizeof(note.thread) - 1U);
	note.thread[sizeof(note.thread) - 1U] = '\0';

	LOG_ERR("WATCHDOG thread=%s stalled, rebooting", name);
	LOG_PANIC(); /* flush the log before the reset */
	sys_reboot(SYS_REBOOT_COLD);
}

static enum reset_reason read_reset_reason(void)
{
	uint32_t cause = 0U;

	if (note.magic == NOTE_MAGIC) {
		(void)strncpy(stalled, note.thread, sizeof(stalled) - 1U);
		note.magic = 0U;
		return RESET_REASON_TASK_WATCHDOG;
	}
	if (hwinfo_get_reset_cause(&cause) != 0) {
		return RESET_REASON_POWER_ON;
	}
	(void)hwinfo_clear_reset_cause();

	if (cause & RESET_WATCHDOG) {
		return RESET_REASON_HW_WATCHDOG;
	}
	if (cause & RESET_SOFTWARE) {
		return RESET_REASON_SOFTWARE;
	}
	if (cause & RESET_PIN) {
		return RESET_REASON_PIN;
	}
	return RESET_REASON_POWER_ON;
}

int wdt_init(void)
{
	reason = read_reset_reason();

	const struct device *hw = DEVICE_DT_GET_OR_NULL(DT_ALIAS(watchdog0));

	if ((hw != NULL) && !device_is_ready(hw)) {
		hw = NULL;
	}

	int rc = task_wdt_init(hw);

	available = (rc == 0);
	if (!available) {
		LOG_ERR("task watchdog init failed (%d)", rc);
	}
	return rc;
}

int wdt_register(const char *name, uint32_t timeout_ms)
{
	if (!available) {
		return -1;
	}

	int ch = task_wdt_add(timeout_ms, on_timeout, (void *)name);

	if (ch < 0) {
		LOG_ERR("no watchdog channel for %s (%d)", name, ch);
		return -1;
	}
	return ch;
}

void wdt_feed(int channel)
{
	if (channel >= 0) {
		(void)task_wdt_feed(channel);
	}
}

bool wdt_available(void)
{
	return available;
}

enum reset_reason wdt_reset_reason(void)
{
	return reason;
}

const char *wdt_reset_reason_name(enum reset_reason r)
{
	switch (r) {
	case RESET_REASON_POWER_ON:
		return "POWER_ON";
	case RESET_REASON_PIN:
		return "PIN";
	case RESET_REASON_SOFTWARE:
		return "SOFTWARE";
	case RESET_REASON_TASK_WATCHDOG:
		return "TASK_WATCHDOG";
	case RESET_REASON_HW_WATCHDOG:
		return "HW_WATCHDOG";
	default:
		return "?";
	}
}

const char *wdt_last_stalled_thread(void)
{
	return stalled;
}
