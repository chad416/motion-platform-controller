/*
 * Task watchdog (one channel per thread, hardware IWDG as fallback) and the
 * reset reason of this boot. SPEC R01.
 */
#ifndef MOTION_WDT_H_
#define MOTION_WDT_H_

#include <stdbool.h>
#include <stdint.h>

enum reset_reason {
	RESET_REASON_POWER_ON = 0,
	RESET_REASON_PIN = 1,
	RESET_REASON_SOFTWARE = 2,
	RESET_REASON_TASK_WATCHDOG = 3,
	RESET_REASON_HW_WATCHDOG = 4,
};

/* Read and clear the reset cause, then start the task watchdog. Call first in main(). */
int wdt_init(void);

/* Register the calling thread. Returns a channel, or -1 if the watchdog is unavailable. */
int wdt_register(const char *name, uint32_t timeout_ms);

/* Feed a channel. A channel of -1 is ignored. */
void wdt_feed(int channel);

bool wdt_available(void);
enum reset_reason wdt_reset_reason(void);
const char *wdt_reset_reason_name(enum reset_reason r);

/* Name of the thread that starved the task watchdog before this boot, or "". */
const char *wdt_last_stalled_thread(void);

#endif /* MOTION_WDT_H_ */
