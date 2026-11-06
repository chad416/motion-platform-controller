/* Small dense linear algebra for the 6x6 kinematics problems. */
#ifndef MCUE_LINALG_H_
#define MCUE_LINALG_H_

#include <stdbool.h>

#define MC_SINGULAR_PIVOT 1e-12

/*
 * Solve A x = b for a 6x6 A by LU decomposition with partial pivoting.
 * A and b are not modified. Returns false if a pivot is below MC_SINGULAR_PIVOT.
 */
bool mc_solve6(const double A[6][6], const double b[6], double x[6]);

#endif /* MCUE_LINALG_H_ */
