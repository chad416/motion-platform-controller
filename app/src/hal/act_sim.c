/*
 * Simulation backend of the actuator HAL: six mcue plant models. The servo
 * thread steps them (act_write); the cueing thread updates their gravity
 * load from the statics at the true pose (act_sim_update_load). A spinlock
 * guards the plants between the two.
 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>

#include "act.h"
#include "mcue/kin.h"
#include "mcue/plant.h"
#include "mcue/platform_config.h"

static struct k_spinlock lock;
static mc_geometry geo;
static mc_plant plant[MC_LEGS];
static double load[MC_LEGS];
static mc_pose true_pose; /* FK warm start, cueing thread only */

int act_init(const mc_geometry *g)
{
	mc_plant_params p;

	geo = *g;
	mc_plant_default_params(&p, g);
	for (int i = 0; i < MC_LEGS; i++) {
		mc_plant_init(&plant[i], &p, g->l_min);
		load[i] = 0.0;
	}
	(void)memset(&true_pose, 0, sizeof(true_pose));
	act_sim_update_load();
	return 0;
}

void act_read(double pos[MC_LEGS])
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	for (int i = 0; i < MC_LEGS; i++) {
		pos[i] = mc_plant_encoder(&plant[i]);
	}
	k_spin_unlock(&lock, key);
}

void act_write(const double current[MC_LEGS])
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	for (int i = 0; i < MC_LEGS; i++) {
		mc_plant_step(&plant[i], current[i], load[i]);
	}
	k_spin_unlock(&lock, key);
}

void act_sim_update_load(void)
{
	double l[MC_LEGS], f[MC_LEGS];
	const mc_vec3 com = {MC_CFG_PAYLOAD_COM_X_M, MC_CFG_PAYLOAD_COM_Y_M,
			     MC_CFG_PAYLOAD_COM_Z_M};
	mc_fk_info info;
	k_spinlock_key_t key = k_spin_lock(&lock);

	for (int i = 0; i < MC_LEGS; i++) {
		l[i] = mc_plant_length(&plant[i]);
	}
	k_spin_unlock(&lock, key);

	/* the physics, outside the lock: FK of the true legs, then the statics */
	(void)mc_fk(&geo, l, &true_pose, 30, 1e-12, &info);
	if (!info.converged || !mc_statics(&geo, &true_pose, MC_CFG_PAYLOAD_KG, &com, f)) {
		(void)memset(&true_pose, 0, sizeof(true_pose));
		return; /* keep the last load */
	}

	key = k_spin_lock(&lock);
	(void)memcpy(load, f, sizeof(load));
	k_spin_unlock(&lock, key);
}

int act_sim_seize(int leg, bool seized)
{
	if ((leg < 0) || (leg >= MC_LEGS)) {
		return -EINVAL;
	}
	k_spinlock_key_t key = k_spin_lock(&lock);

	plant[leg].stuck = seized;
	k_spin_unlock(&lock, key);
	return 0;
}

void act_sim_get(struct act_sim_state *out)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	for (int i = 0; i < MC_LEGS; i++) {
		out->length[i] = mc_plant_length(&plant[i]);
		out->speed[i] = mc_plant_speed(&plant[i]);
		out->load[i] = load[i];
		out->current[i] = plant[i].current;
		out->seized[i] = plant[i].stuck;
	}
	k_spin_unlock(&lock, key);
}
