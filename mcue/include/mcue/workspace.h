/*
 * Workspace limiting by scaling the whole pose towards neutral. SPEC F03, P03.
 * Legs are never clamped individually, so the platform never moves in a shape
 * that was not commanded.
 */
#ifndef MCUE_WORKSPACE_H_
#define MCUE_WORKSPACE_H_

#include <stdbool.h>

#include "mcue/geometry.h"

#define MC_WS_ITERATIONS 12 /* scale resolution 1/4096 */

/* True if every leg of the pose is inside the soft limits. legs may be NULL. */
bool mc_pose_valid(const mc_geometry *g, const mc_pose *q, double legs[MC_LEGS]);

typedef enum {
	MC_WS_OK = 0,     /* command reachable, unchanged */
	MC_WS_SCALED = 1, /* scaled towards neutral */
} mc_ws_result;

/*
 * Limit a command: out = scale * cmd with the largest scale (bisection,
 * MC_WS_ITERATIONS steps) that keeps every leg inside the soft limits.
 * Neutral must be valid (it is, by construction of the geometry).
 * scale may be NULL.
 */
mc_ws_result mc_ws_limit(const mc_geometry *g, const mc_pose *cmd, mc_pose *out, double *scale);

#endif /* MCUE_WORKSPACE_H_ */
