/*
 * Platform geometry and pose types. SPEC §3.
 *
 * Frames: x forward, y left, z up. Pose = position relative to neutral plus
 * ZYX Euler angles: R = Rz(yaw) * Ry(pitch) * Rx(roll).
 */
#ifndef MCUE_GEOMETRY_H_
#define MCUE_GEOMETRY_H_

#include <stdbool.h>

#define MC_LEGS 6

/* pi for every mcue file: C11 <math.h> does not have to define M_PI */
#define MC_PI 3.14159265358979323846

typedef struct {
	double x, y, z;
} mc_vec3;

/* Pose relative to neutral: metres and radians. */
typedef struct {
	double x, y, z;
	double roll, pitch, yaw;
} mc_pose;

typedef struct {
	double base_radius;      /* m */
	double platform_radius;  /* m */
	double base_half_deg;    /* base joint pair half-angle */
	double platform_half_deg;/* platform half-angle h_p (SPEC §3) */
	double neutral_height;   /* base plane to platform plane, m */
	double stroke;           /* m */
	double soft_margin;      /* m, each end */
} mc_geometry_params;

typedef struct {
	mc_vec3 base[MC_LEGS]; /* a_i, base frame */
	mc_vec3 plat[MC_LEGS]; /* b_i, platform frame */
	double h0;
	double l0;    /* leg length at neutral */
	double l_min; /* physical stroke ends */
	double l_max;
	double soft_lo; /* l_min + margin */
	double soft_hi; /* l_max - margin */
} mc_geometry;

/* Build joint positions and leg limits. Returns false for invalid parameters. */
bool mc_geometry_build(mc_geometry *g, const mc_geometry_params *p);

/* Parameters from config/platform.json (generated header). */
void mc_geometry_default_params(mc_geometry_params *p);

#endif /* MCUE_GEOMETRY_H_ */
