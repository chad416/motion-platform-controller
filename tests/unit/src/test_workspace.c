/* SPEC F03, P03: workspace limiting by scaling, and the 10 000-command property test. */
#include <math.h>
#include <zephyr/ztest.h>

#include "kin_vectors.h"
#include "mcue/kin.h"
#include "mcue/workspace.h"

static mc_geometry g;

static void *setup(void)
{
	mc_geometry_params p;

	mc_geometry_default_params(&p);
	zassert_true(mc_geometry_build(&g, &p));
	return NULL;
}

ZTEST_SUITE(workspace, NULL, setup, NULL, NULL, NULL);

static uint32_t rng_state = 987654321U;

static double urand(double lo, double hi)
{
	rng_state ^= rng_state << 13;
	rng_state ^= rng_state >> 17;
	rng_state ^= rng_state << 5;
	return lo + ((hi - lo) * ((double)rng_state / 4294967295.0));
}

static const double *as_array(const mc_pose *q)
{
	return &q->x; /* mc_pose is six consecutive doubles */
}

BUILD_ASSERT(sizeof(mc_pose) == 6 * sizeof(double), "mc_pose must be six packed doubles");

ZTEST(workspace, test_reachable_commands_are_untouched)
{
	for (size_t k = 0; k < ARRAY_SIZE(kin_vectors); k++) {
		const double *v = kin_vectors[k].pose;
		mc_pose cmd = {v[0], v[1], v[2], v[3], v[4], v[5]};
		mc_pose out;
		double s;

		zassert_equal(mc_ws_limit(&g, &cmd, &out, &s), MC_WS_OK, "%s", kin_vectors[k].name);
		zassert_equal(s, 1.0);
		zassert_mem_equal(&out, &cmd, sizeof(cmd));
	}
}

ZTEST(workspace, test_excessive_commands_match_reference_scale)
{
	for (size_t k = 0; k < ARRAY_SIZE(ws_vectors); k++) {
		const double *v = ws_vectors[k].cmd;
		mc_pose cmd = {v[0], v[1], v[2], v[3], v[4], v[5]};
		mc_pose out;
		double s;

		zassert_equal(mc_ws_limit(&g, &cmd, &out, &s), MC_WS_SCALED, "%s", ws_vectors[k].name);
		/* Same bisection as the reference; allow one step for boundary rounding. */
		zassert_within(s, ws_vectors[k].scale, 1.0 / 4096.0, "%s: %f vs %f", ws_vectors[k].name,
			       s, ws_vectors[k].scale);
		zassert_true(mc_pose_valid(&g, &out, NULL));
	}
}

ZTEST(workspace, test_property_10000_random_commands)
{
	/*
	 * Half the commands up to about twice the single-axis workspace on every axis
	 * at once (SPEC P03), half small enough to be mostly reachable, so both the
	 * scaling path and the "leave it untouched" path are exercised.
	 */
	const double lim[6] = {0.5, 0.45, 0.28, 0.5, 0.5, 0.95};
	int scaled = 0;

	for (int n = 0; n < 10000; n++) {
		mc_pose cmd;
		double *c = &cmd.x;
		double k = ((n % 2) == 0) ? 1.0 : 0.15;

		for (int j = 0; j < 6; j++) {
			c[j] = urand(-k * lim[j], k * lim[j]);
		}

		mc_pose out;
		double s;
		double legs[MC_LEGS];
		mc_ws_result r = mc_ws_limit(&g, &cmd, &out, &s);

		/* 1. Every leg inside the soft limits, always. */
		zassert_true(mc_pose_valid(&g, &out, legs), "command %d left the workspace", n);
		for (int i = 0; i < MC_LEGS; i++) {
			zassert_true((legs[i] >= g.soft_lo) && (legs[i] <= g.soft_hi));
		}
		/* 2. The output is the command scaled towards neutral: same shape. */
		zassert_true((s >= 0.0) && (s <= 1.0));
		for (int j = 0; j < 6; j++) {
			zassert_within(as_array(&out)[j], s * c[j], 1e-15);
		}
		/* 3. A reachable command is never changed. */
		if (mc_pose_valid(&g, &cmd, NULL)) {
			zassert_equal(r, MC_WS_OK);
		} else {
			scaled++;
		}
	}
	TC_PRINT("property test: %d of 10000 commands needed scaling\n", scaled);
	zassert_true((scaled > 3000) && (scaled < 9000), "both paths must be exercised (%d)", scaled);
}
