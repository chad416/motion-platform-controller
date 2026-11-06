#include "mcue/workspace.h"

#include <stddef.h>

#include "mcue/kin.h"

bool mc_pose_valid(const mc_geometry *g, const mc_pose *q, double legs[MC_LEGS])
{
	double l[MC_LEGS];
	bool ok = true;

	mc_ik(g, q, l);
	for (int i = 0; i < MC_LEGS; i++) {
		if ((l[i] < g->soft_lo) || (l[i] > g->soft_hi)) {
			ok = false;
		}
		if (legs != NULL) {
			legs[i] = l[i];
		}
	}
	return ok;
}

static mc_pose scaled(const mc_pose *q, double s)
{
	return (mc_pose){s * q->x, s * q->y, s * q->z, s * q->roll, s * q->pitch, s * q->yaw};
}

mc_ws_result mc_ws_limit(const mc_geometry *g, const mc_pose *cmd, mc_pose *out, double *scale)
{
	if (mc_pose_valid(g, cmd, NULL)) {
		*out = *cmd;
		if (scale != NULL) {
			*scale = 1.0;
		}
		return MC_WS_OK;
	}

	/* Invariant: lo is always a valid scale (0 = neutral), hi always invalid. */
	double lo = 0.0;
	double hi = 1.0;

	for (int k = 0; k < MC_WS_ITERATIONS; k++) {
		double mid = 0.5 * (lo + hi);
		mc_pose t = scaled(cmd, mid);

		if (mc_pose_valid(g, &t, NULL)) {
			lo = mid;
		} else {
			hi = mid;
		}
	}

	*out = scaled(cmd, lo);
	if (scale != NULL) {
		*scale = lo;
	}
	return MC_WS_SCALED;
}
