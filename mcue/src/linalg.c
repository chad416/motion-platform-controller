#include "mcue/linalg.h"

#include <math.h>
#include <string.h>

bool mc_solve6(const double A[6][6], const double b[6], double x[6])
{
	double M[6][6];
	double v[6];

	(void)memcpy(M, A, sizeof(M));
	(void)memcpy(v, b, sizeof(v));

	/* Forward elimination with partial pivoting. */
	for (int k = 0; k < 6; k++) {
		int piv = k;
		double best = fabs(M[k][k]);

		for (int r = k + 1; r < 6; r++) {
			if (fabs(M[r][k]) > best) {
				best = fabs(M[r][k]);
				piv = r;
			}
		}
		if (best < MC_SINGULAR_PIVOT) {
			return false;
		}
		if (piv != k) {
			for (int c = 0; c < 6; c++) {
				double t = M[k][c];

				M[k][c] = M[piv][c];
				M[piv][c] = t;
			}
			double t = v[k];

			v[k] = v[piv];
			v[piv] = t;
		}
		for (int r = k + 1; r < 6; r++) {
			double f = M[r][k] / M[k][k];

			M[r][k] = 0.0;
			for (int c = k + 1; c < 6; c++) {
				M[r][c] -= f * M[k][c];
			}
			v[r] -= f * v[k];
		}
	}

	/* Back substitution. */
	for (int r = 5; r >= 0; r--) {
		double s = v[r];

		for (int c = r + 1; c < 6; c++) {
			s -= M[r][c] * x[c];
		}
		x[r] = s / M[r][r];
	}
	return true;
}
