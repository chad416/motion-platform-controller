#include "mcue/plant.h"

#include <math.h>
#include <string.h>

#include "mcue/platform_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void mc_plant_default_params(mc_plant_params *p, const mc_geometry *g)
{
	/* leg angle from vertical at neutral: cos a = h0 / l0 */
	double c = g->h0 / g->l0;

	p->lead = MC_CFG_ACT_SCREW_LEAD_M;
	p->efficiency = MC_CFG_ACT_SCREW_EFFICIENCY;
	p->kt = MC_CFG_ACT_MOTOR_KT_NM_A;
	p->j_motor = MC_CFG_ACT_MOTOR_INERTIA_KG_M2;
	p->viscous = MC_CFG_ACT_VISCOUS_NM_S_RAD;
	p->coulomb = MC_CFG_ACT_COULOMB_NM;
	p->coulomb_eps = MC_CFG_ACT_COULOMB_SMOOTHING_RAD_S;
	p->i_max = MC_CFG_ACT_CURRENT_LIMIT_A;
	p->counts_per_rev = MC_CFG_ACT_ENCODER_COUNTS_REV;
	p->load_mass = MC_CFG_PAYLOAD_KG / (6.0 * c * c);
	p->stop_lo = g->l_min;
	p->stop_hi = g->l_max;
	p->dt = 1.0 / MC_CFG_SRV_RATE_HZ;
	p->substeps = (int)MC_CFG_ACT_PLANT_SUBSTEPS;
}

static double radius(const mc_plant_params *p)
{
	return p->lead / (2.0 * M_PI);
}

double mc_plant_inertia(const mc_plant_params *p)
{
	double r = radius(p);

	return p->j_motor + ((p->load_mass * r * r) / p->efficiency);
}

void mc_plant_init(mc_plant *pl, const mc_plant_params *p, double length)
{
	(void)memset(pl, 0, sizeof(*pl));
	pl->p = *p;
	pl->theta = (length - p->stop_lo) / radius(p);
}

void mc_plant_step(mc_plant *pl, double current, double load)
{
	const mc_plant_params *p = &pl->p;
	const double r = radius(p);
	const double j = mc_plant_inertia(p);
	const double h = p->dt / p->substeps;
	const double theta_hi = (p->stop_hi - p->stop_lo) / r;
	double i = fmax(-p->i_max, fmin(p->i_max, current));

	pl->current = i;
	if (pl->stuck) {
		pl->omega = 0.0;
		return;
	}
	for (int k = 0; k < p->substeps; k++) {
		double torque = (p->kt * i) - (p->viscous * pl->omega) -
				(p->coulomb * tanh(pl->omega / p->coulomb_eps)) -
				((r * load) / p->efficiency);

		pl->omega += h * (torque / j);
		pl->theta += h * pl->omega;
		/* inelastic end stops */
		if ((pl->theta <= 0.0) && (pl->omega <= 0.0)) {
			pl->theta = 0.0;
			pl->omega = 0.0;
		} else if ((pl->theta >= theta_hi) && (pl->omega >= 0.0)) {
			pl->theta = theta_hi;
			pl->omega = 0.0;
		}
	}
}

double mc_plant_length(const mc_plant *pl)
{
	return pl->p.stop_lo + (pl->theta * radius(&pl->p));
}

double mc_plant_speed(const mc_plant *pl)
{
	return pl->omega * radius(&pl->p);
}

double mc_plant_encoder(const mc_plant *pl)
{
	double counts = floor((pl->theta / (2.0 * M_PI)) * pl->p.counts_per_rev);

	return pl->p.stop_lo + ((counts / pl->p.counts_per_rev) * pl->p.lead);
}
