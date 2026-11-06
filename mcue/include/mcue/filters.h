/*
 * Second-order sections (biquads) designed from continuous prototypes with the
 * bilinear (Tustin) transform. Direct form II transposed, double precision.
 */
#ifndef MCUE_FILTERS_H_
#define MCUE_FILTERS_H_

#include <stdbool.h>

typedef struct {
	double b0, b1, b2; /* numerator, normalised by a0 */
	double a1, a2;     /* denominator, a0 = 1 */
	double z1, z2;     /* state */
} mc_biquad;

/*
 * Continuous prototype H(s) = (n2 s^2 + n1 s + n0) / (d2 s^2 + d1 s + d0),
 * discretised at sample rate fs. Returns false if the result is not stable.
 */
bool mc_biquad_tustin(mc_biquad *f, double n2, double n1, double n0, double d2, double d1, double d0,
		      double fs);

/* Prototypes used by the washout. */
bool mc_biquad_hp2(mc_biquad *f, double wn, double zeta, double fs); /* s^2/(s^2+2zwn s+wn^2) */
bool mc_biquad_lp2(mc_biquad *f, double wn, double zeta, double fs); /* wn^2/(s^2+2zwn s+wn^2) */
bool mc_biquad_hp1(mc_biquad *f, double wb, double fs);              /* s/(s+wb) */

double mc_biquad_step(mc_biquad *f, double x);
void mc_biquad_reset(mc_biquad *f);

/* Poles of the discrete section are inside the unit circle. */
bool mc_biquad_stable(const mc_biquad *f);

#endif /* MCUE_FILTERS_H_ */
