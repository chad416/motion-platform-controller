/*
 * Stewart platform kinematics: inverse, Jacobian, forward (Newton-Raphson),
 * statics. SPEC F01, F02, F07 (gravity load).
 */
#ifndef MCUE_KIN_H_
#define MCUE_KIN_H_

#include <stdbool.h>

#include "mcue/geometry.h"

/* Rotation matrix R = Rz(yaw) Ry(pitch) Rx(roll). */
void mc_rotation(const mc_pose *q, double R[3][3]);

/* Leg lengths for a pose. */
void mc_ik(const mc_geometry *g, const mc_pose *q, double legs[MC_LEGS]);

/* d legs / d (x, y, z, roll, pitch, yaw), row i = leg i. */
void mc_jacobian(const mc_geometry *g, const mc_pose *q, double J[MC_LEGS][6]);

typedef struct {
	int iterations;
	double residual; /* max |leg error| after the last iteration, m (HUGE_VAL if not finite) */
	bool converged;
} mc_fk_info;

/*
 * Pose from leg lengths by Newton-Raphson, starting from *q (the last pose).
 * Iterates until the step is below tol (rad / m), max_iter iterations, or a
 * singular Jacobian. Converged (the return value) means the leg residual is
 * below tol too, so a small step at a wrong pose does not count. On a singular
 * Jacobian *q stays at the iterate reached. A NaN or infinite leg length or
 * iterate restores *q to the starting pose and reports not converged.
 */
bool mc_fk(const mc_geometry *g, const double legs[MC_LEGS], mc_pose *q, int max_iter, double tol,
	   mc_fk_info *info);

/*
 * Leg forces (N, positive = compression) that hold a payload of `mass` kg with
 * its centre of mass at `com` (platform frame) in pose q. Returns false if singular.
 */
bool mc_statics(const mc_geometry *g, const mc_pose *q, double mass, const mc_vec3 *com,
		double force[MC_LEGS]);

#define MC_GRAVITY 9.80665

#endif /* MCUE_KIN_H_ */
