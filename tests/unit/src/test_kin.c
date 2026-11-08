/* SPEC F01, F02, F07 (statics): kinematics against the NumPy reference and first principles. */
#include <math.h>
#include <zephyr/ztest.h>

#include "kin_vectors.h"
#include "mcue/kin.h"
#include "mcue/platform_config.h"

static mc_geometry g;

static void *setup(void)
{
	mc_geometry_params p;

	mc_geometry_default_params(&p);
	zassert_true(mc_geometry_build(&g, &p));
	return NULL;
}

ZTEST_SUITE(kin, NULL, setup, NULL, NULL, NULL);

static mc_pose pose_of(const double v[6])
{
	return (mc_pose){v[0], v[1], v[2], v[3], v[4], v[5]};
}

static uint32_t rng_state = 12345U;

static double urand(double lo, double hi)
{
	rng_state ^= rng_state << 13;
	rng_state ^= rng_state >> 17;
	rng_state ^= rng_state << 5;
	return lo + ((hi - lo) * ((double)rng_state / 4294967295.0));
}

ZTEST(kin, test_geometry_matches_reference)
{
	zassert_within(g.l0, KIN_VEC_L0, 1e-12);
	zassert_within(g.l_max - g.l_min, MC_CFG_STROKE_M, 1e-12);
	zassert_within(g.soft_hi - g.soft_lo, MC_CFG_STROKE_M - (2.0 * MC_CFG_SOFT_MARGIN_M), 1e-12);
}

ZTEST(kin, test_invalid_geometry_rejected)
{
	mc_geometry_params p;
	mc_geometry tmp;

	mc_geometry_default_params(&p);
	p.soft_margin = p.stroke; /* margins larger than the stroke */
	zassert_false(mc_geometry_build(&tmp, &p));
	mc_geometry_default_params(&p);
	p.neutral_height = 0.0;
	zassert_false(mc_geometry_build(&tmp, &p));
}

ZTEST(kin, test_neutral_and_pure_heave_closed_form)
{
	double l[MC_LEGS];
	mc_pose q = {0};

	mc_ik(&g, &q, l);
	for (int i = 0; i < MC_LEGS; i++) {
		zassert_within(l[i], g.l0, 1e-12, "leg %d", i);
	}

	/* Pure heave: horizontal offset is unchanged, vertical becomes h0 + dz. */
	double dx = g.plat[0].x - g.base[0].x;
	double dy = g.plat[0].y - g.base[0].y;

	q.z = 0.05;
	mc_ik(&g, &q, l);
	for (int i = 0; i < MC_LEGS; i++) {
		zassert_within(l[i], sqrt((dx * dx) + (dy * dy) + pow(g.h0 + 0.05, 2)), 1e-12);
	}
}

ZTEST(kin, test_ik_matches_numpy_reference)
{
	for (size_t k = 0; k < ARRAY_SIZE(kin_vectors); k++) {
		mc_pose q = pose_of(kin_vectors[k].pose);
		double l[MC_LEGS];

		mc_ik(&g, &q, l);
		for (int i = 0; i < MC_LEGS; i++) {
			zassert_within(l[i], kin_vectors[k].legs[i], 1e-12, "%s leg %d",
				       kin_vectors[k].name, i);
		}
	}
}

ZTEST(kin, test_statics_matches_numpy_reference)
{
	mc_vec3 com = {MC_CFG_PAYLOAD_COM_X_M, MC_CFG_PAYLOAD_COM_Y_M, MC_CFG_PAYLOAD_COM_Z_M};

	for (size_t k = 0; k < ARRAY_SIZE(kin_vectors); k++) {
		mc_pose q = pose_of(kin_vectors[k].pose);
		double f[MC_LEGS];

		zassert_true(mc_statics(&g, &q, MC_CFG_PAYLOAD_KG, &com, f));
		for (int i = 0; i < MC_LEGS; i++) {
			zassert_within(f[i], kin_vectors[k].force[i], 1e-6, "%s leg %d",
				       kin_vectors[k].name, i);
		}
	}
}

ZTEST(kin, test_statics_neutral_by_hand)
{
	/* Symmetric: each leg carries m g / 6, divided by the cosine of its angle to vertical. */
	mc_vec3 com = {0.0, 0.0, 0.3};
	mc_pose q = {0};
	double f[MC_LEGS];

	zassert_true(mc_statics(&g, &q, 500.0, &com, f));
	for (int i = 0; i < MC_LEGS; i++) {
		zassert_within(f[i], (500.0 * MC_GRAVITY / 6.0) / (g.h0 / g.l0), 1e-9);
	}
}

ZTEST(kin, test_jacobian_matches_finite_differences)
{
	const double h = 1e-7;

	for (size_t k = 0; k < ARRAY_SIZE(kin_vectors); k++) {
		mc_pose q = pose_of(kin_vectors[k].pose);
		double J[MC_LEGS][6];

		mc_jacobian(&g, &q, J);
		for (int c = 0; c < 6; c++) {
			double qp[6], qm[6], lp[MC_LEGS], lm[MC_LEGS];

			for (int j = 0; j < 6; j++) {
				qp[j] = kin_vectors[k].pose[j];
				qm[j] = kin_vectors[k].pose[j];
			}
			qp[c] += h;
			qm[c] -= h;
			mc_pose a = pose_of(qp), b = pose_of(qm);

			mc_ik(&g, &a, lp);
			mc_ik(&g, &b, lm);
			for (int i = 0; i < MC_LEGS; i++) {
				zassert_within(J[i][c], (lp[i] - lm[i]) / (2.0 * h), 1e-6,
					       "%s J[%d][%d]", kin_vectors[k].name, i, c);
			}
		}
	}
}

ZTEST(kin, test_fk_round_trip_1000_random_poses)
{
	int worst_iter = 0;

	for (int n = 0; n < 1000; n++) {
		mc_pose truth = {urand(-0.15, 0.15), urand(-0.15, 0.15), urand(-0.10, 0.10),
				 urand(-0.17, 0.17), urand(-0.17, 0.17), urand(-0.35, 0.35)};
		double l[MC_LEGS];
		mc_pose q = {0}; /* cold start from neutral */
		mc_fk_info info;

		mc_ik(&g, &truth, l);
		zassert_true(mc_fk(&g, l, &q, 10, 1e-12, &info), "pose %d did not converge", n);
		zassert_within(q.x, truth.x, 1e-9);
		zassert_within(q.y, truth.y, 1e-9);
		zassert_within(q.z, truth.z, 1e-9);
		zassert_within(q.roll, truth.roll, 1e-9);
		zassert_within(q.pitch, truth.pitch, 1e-9);
		zassert_within(q.yaw, truth.yaw, 1e-9);
		worst_iter = MAX(worst_iter, info.iterations);
	}
	TC_PRINT("FK cold start from neutral: worst case %d iterations\n", worst_iter);
}

ZTEST(kin, test_fk_warm_start_meets_spec)
{
	/* SPEC F02: from the last pose (one 2 ms cueing step away) within 6 iterations to 1e-6. */
	for (size_t k = 0; k < ARRAY_SIZE(kin_vectors); k++) {
		mc_pose truth = pose_of(kin_vectors[k].pose);
		mc_pose q = truth;
		double l[MC_LEGS];
		mc_fk_info info;

		/* last pose: 1 mm and 0.1 deg away on every axis (more than one cycle at full speed) */
		q.x -= 0.001;
		q.y += 0.001;
		q.z -= 0.001;
		q.roll += 0.00175;
		q.pitch -= 0.00175;
		q.yaw += 0.00175;
		mc_ik(&g, &truth, l);
		zassert_true(mc_fk(&g, l, &q, 6, 1e-6, &info), "%s", kin_vectors[k].name);
		zassert_true(info.iterations <= 3, "%s took %d iterations", kin_vectors[k].name,
			     info.iterations);
		zassert_true(info.residual < 1e-9);
	}
}

/* fmax() ignores NaN: a NaN leg length must not come back as a converged pose. */
ZTEST(kin, test_fk_rejects_non_finite_legs)
{
	const mc_pose start = {0.01, -0.02, 0.03, 0.01, 0.02, -0.03};
	double l[MC_LEGS];
	mc_pose q = start;
	mc_fk_info info;

	mc_ik(&g, &start, l);
	l[3] = NAN;
	zassert_false(mc_fk(&g, l, &q, 30, 1e-9, &info));
	zassert_false(info.converged);
	zassert_false(isfinite(info.residual));
	zassert_true(isfinite(q.x) && isfinite(q.roll) && isfinite(q.yaw), "NaN pose returned");
	zassert_equal(q.x, start.x, "pose restored to the start");
	zassert_equal(q.yaw, start.yaw);
}

/* Leg lengths no pose produces: not converged, whatever the last step was. */
ZTEST(kin, test_fk_unreachable_legs_not_converged)
{
	double l[MC_LEGS];
	mc_pose q = {0};
	mc_fk_info info;

	/*
	 * Every leg 50 mm: the joints of legs 1 and 2 are 0.333 m apart on the base
	 * and 0.5 m apart on the platform, so 2 x 50 mm of leg cannot bridge them.
	 */
	for (int i = 0; i < MC_LEGS; i++) {
		l[i] = 0.05;
	}
	bool ok = mc_fk(&g, l, &q, 30, 1e-9, &info);

	TC_PRINT("unreachable legs: converged %d after %d iterations, residual %.3g m\n", ok,
		 info.iterations, info.residual);
	zassert_false(ok);
	zassert_true(info.residual > 1e-9);
}
