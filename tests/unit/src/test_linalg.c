/* 6x6 solver used by forward kinematics and statics. */
#include <zephyr/ztest.h>

#include "mcue/linalg.h"

ZTEST_SUITE(linalg, NULL, NULL, NULL, NULL, NULL);

ZTEST(linalg, test_solves_a_system_needing_pivoting)
{
	/* Zero on the first diagonal element forces a row swap. */
	const double A[6][6] = {
		{0, 2, 1, 0, 0, 1}, {3, 1, 0, 2, 0, 0}, {1, 0, 4, 1, 1, 0},
		{0, 1, 1, 5, 0, 2}, {2, 0, 0, 1, 6, 1}, {1, 1, 0, 0, 1, 7},
	};
	const double x_true[6] = {1, -2, 3, -4, 5, -6};
	double b[6] = {0};
	double x[6];

	for (int r = 0; r < 6; r++) {
		for (int c = 0; c < 6; c++) {
			b[r] += A[r][c] * x_true[c];
		}
	}
	zassert_true(mc_solve6(A, b, x));
	for (int i = 0; i < 6; i++) {
		zassert_within(x[i], x_true[i], 1e-12);
	}
}

ZTEST(linalg, test_singular_matrix_is_reported)
{
	double A[6][6] = {{0}};
	const double b[6] = {1, 1, 1, 1, 1, 1};
	double x[6];

	for (int i = 0; i < 6; i++) {
		A[i][0] = 1.0; /* rank 1 */
	}
	zassert_false(mc_solve6(A, b, x));
}
