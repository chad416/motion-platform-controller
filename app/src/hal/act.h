/*
 * Actuator HAL: six legs, current in, encoder position out. SPEC §5.
 *
 * The control code only calls act_read() and act_write(). This build links
 * the simulation backend (act_sim.c: the mcue plant model with the payload's
 * gravity load); on hardware the same calls become the drive interface.
 */
#ifndef MOTION_ACT_H_
#define MOTION_ACT_H_

#include <stdbool.h>

#include "mcue/geometry.h"

/* Power-up state: legs resting on their lower end stops. */
int act_init(const mc_geometry *g);

/* Encoder positions as leg lengths (m). Servo thread. */
void act_read(double pos[MC_LEGS]);

/*
 * Current commands (A), held for one servo period. In simulation this also
 * advances the plant by one period. Servo thread.
 */
void act_write(const double current[MC_LEGS]);

/* ---- simulation backend only ---------------------------------------------- */

/* Recompute the payload's gravity load on each leg at the true pose (cueing rate). */
void act_sim_update_load(void);

/* Fault injection: seize (or free) leg 0..5. */
int act_sim_seize(int leg, bool seized);

struct act_sim_state {
	double length[MC_LEGS]; /* true, m */
	double speed[MC_LEGS];  /* true, m/s */
	double load[MC_LEGS];   /* N, compression + */
	double current[MC_LEGS];/* applied, A */
	bool seized[MC_LEGS];
};

void act_sim_get(struct act_sim_state *out);

#endif /* MOTION_ACT_H_ */
