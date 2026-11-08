#include "mcue/geometry.h"

#include <math.h>

#include "mcue/platform_config.h" /* generated from config/platform.json */

static double deg2rad(double d)
{
	return d * (MC_PI / 180.0);
}

void mc_geometry_default_params(mc_geometry_params *p)
{
	p->base_radius = MC_CFG_BASE_RADIUS_M;
	p->platform_radius = MC_CFG_PLATFORM_RADIUS_M;
	p->base_half_deg = MC_CFG_BASE_HALF_ANGLE_DEG;
	p->platform_half_deg = MC_CFG_PLATFORM_HALF_ANGLE_DEG;
	p->neutral_height = MC_CFG_NEUTRAL_HEIGHT_M;
	p->stroke = MC_CFG_STROKE_M;
	p->soft_margin = MC_CFG_SOFT_MARGIN_M;
}

bool mc_geometry_build(mc_geometry *g, const mc_geometry_params *p)
{
	if ((p->base_radius <= 0.0) || (p->platform_radius <= 0.0) || (p->neutral_height <= 0.0) ||
	    (p->stroke <= 0.0) || (p->soft_margin < 0.0) || ((2.0 * p->soft_margin) >= p->stroke)) {
		return false;
	}

	/*
	 * Leg 2k joins base joint 120k - h_b to platform joint 120k - (60 - h_p);
	 * leg 2k+1 mirrors it (SPEC §3).
	 */
	for (int k = 0; k < 3; k++) {
		double c = 120.0 * k;

		for (int s = 0; s < 2; s++) {
			double sign = (s == 0) ? -1.0 : 1.0;
			double ab = deg2rad(c + (sign * p->base_half_deg));
			double ap = deg2rad(c + (sign * (60.0 - p->platform_half_deg)));
			int i = (2 * k) + s;

			g->base[i] = (mc_vec3){p->base_radius * cos(ab), p->base_radius * sin(ab), 0.0};
			g->plat[i] = (mc_vec3){p->platform_radius * cos(ap), p->platform_radius * sin(ap),
					       0.0};
		}
	}

	double dx = g->plat[0].x - g->base[0].x;
	double dy = g->plat[0].y - g->base[0].y;
	double dz = p->neutral_height;

	g->h0 = p->neutral_height;
	g->l0 = sqrt((dx * dx) + (dy * dy) + (dz * dz));
	g->l_min = g->l0 - (p->stroke / 2.0);
	g->l_max = g->l0 + (p->stroke / 2.0);
	g->soft_lo = g->l_min + p->soft_margin;
	g->soft_hi = g->l_max - p->soft_margin;
	return true;
}
