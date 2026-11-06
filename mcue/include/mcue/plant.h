/*
 * Electric ball-screw actuator model: the simulation backend of the actuator
 * HAL. SPEC F07, §6.5.
 *
 * Per leg, in motor angle theta (0 at the lower end stop):
 *
 *   J w' = Kt i - b w - Tc tanh(w / w_eps) - r F / eta,   r = lead / 2 pi
 *   J    = J_motor + m_load r^2 / eta
 *
 * F is the axial load (N, compression positive): the payload weight on this
 * leg from the statics at the current pose, set from outside at the cueing
 * rate. m_load is the payload mass reflected to one leg, M / (6 cos^2 a) with
 * a the leg angle from vertical at neutral; the legs are modelled decoupled
 * (SPEC §9.4). The drive limits the current. The mechanical end stops are
 * inelastic. The encoder reports whole counts.
 *
 * Integration: semi-implicit Euler, `substeps` steps per call.
 */
#ifndef MCUE_PLANT_H_
#define MCUE_PLANT_H_

#include <stdbool.h>

#include "mcue/geometry.h"

typedef struct {
	double lead;           /* m per motor revolution */
	double efficiency;     /* ball screw */
	double kt;             /* Nm/A */
	double j_motor;        /* rotor + screw, kg m^2 */
	double viscous;        /* Nm/(rad/s) */
	double coulomb;        /* Nm */
	double coulomb_eps;    /* rad/s, tanh smoothing */
	double i_max;          /* drive current limit, A */
	double counts_per_rev; /* encoder */
	double load_mass;      /* payload mass reflected to one leg, kg */
	double stop_lo, stop_hi; /* mechanical end stops, leg length, m */
	double dt;             /* one call (servo period), s */
	int substeps;
} mc_plant_params;

typedef struct {
	mc_plant_params p;
	double theta;   /* rad from the lower stop */
	double omega;   /* rad/s */
	double current; /* applied, after the drive limit */
	bool stuck;     /* fault injection: seized (motor or screw) */
} mc_plant;

/* From platform.json; load_mass and the stops from the geometry and payload. */
void mc_plant_default_params(mc_plant_params *p, const mc_geometry *g);

void mc_plant_init(mc_plant *pl, const mc_plant_params *p, double length);

/* One servo period with current command i and axial load F (N, compression +). */
void mc_plant_step(mc_plant *pl, double current, double load);

double mc_plant_length(const mc_plant *pl); /* true leg length, m */
double mc_plant_speed(const mc_plant *pl);  /* true leg speed, m/s */

/* Encoder reading as a leg length: whole counts from the lower stop. */
double mc_plant_encoder(const mc_plant *pl);

/* Total inertia at the motor, J_motor + m_load r^2 / eta (kg m^2). */
double mc_plant_inertia(const mc_plant_params *p);

#endif /* MCUE_PLANT_H_ */
