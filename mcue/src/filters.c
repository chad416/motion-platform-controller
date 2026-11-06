#include "mcue/filters.h"

#include <math.h>

bool mc_biquad_tustin(mc_biquad *f, double n2, double n1, double n0, double d2, double d1, double d0,
		      double fs)
{
	/* s = K (1 - z^-1) / (1 + z^-1), K = 2 fs */
	double K = 2.0 * fs;
	double K2 = K * K;
	double B0 = (n2 * K2) + (n1 * K) + n0;
	double B1 = (2.0 * n0) - (2.0 * n2 * K2);
	double B2 = (n2 * K2) - (n1 * K) + n0;
	double A0 = (d2 * K2) + (d1 * K) + d0;
	double A1 = (2.0 * d0) - (2.0 * d2 * K2);
	double A2 = (d2 * K2) - (d1 * K) + d0;

	if (A0 == 0.0) {
		return false;
	}
	f->b0 = B0 / A0;
	f->b1 = B1 / A0;
	f->b2 = B2 / A0;
	f->a1 = A1 / A0;
	f->a2 = A2 / A0;
	mc_biquad_reset(f);
	return mc_biquad_stable(f);
}

bool mc_biquad_hp2(mc_biquad *f, double wn, double zeta, double fs)
{
	return mc_biquad_tustin(f, 1.0, 0.0, 0.0, 1.0, 2.0 * zeta * wn, wn * wn, fs);
}

bool mc_biquad_lp2(mc_biquad *f, double wn, double zeta, double fs)
{
	return mc_biquad_tustin(f, 0.0, 0.0, wn * wn, 1.0, 2.0 * zeta * wn, wn * wn, fs);
}

bool mc_biquad_hp1(mc_biquad *f, double wb, double fs)
{
	/*
	 * First order, transformed directly: the 2nd-order formula with d2 = 0
	 * would leave a pole at z = -1 (cancelled by a zero), which is marginal.
	 * s / (s + wb) -> K (1 - z^-1) / ((K + wb) + (wb - K) z^-1)
	 */
	double K = 2.0 * fs;
	double A0 = K + wb;

	f->b0 = K / A0;
	f->b1 = -K / A0;
	f->b2 = 0.0;
	f->a1 = (wb - K) / A0;
	f->a2 = 0.0;
	mc_biquad_reset(f);
	return mc_biquad_stable(f);
}

double mc_biquad_step(mc_biquad *f, double x)
{
	double y = (f->b0 * x) + f->z1;

	f->z1 = (f->b1 * x) - (f->a1 * y) + f->z2;
	f->z2 = (f->b2 * x) - (f->a2 * y);
	return y;
}

void mc_biquad_reset(mc_biquad *f)
{
	f->z1 = 0.0;
	f->z2 = 0.0;
}

bool mc_biquad_stable(const mc_biquad *f)
{
	/* z^2 + a1 z + a2: both roots inside the unit circle (Jury conditions). */
	return (fabs(f->a2) < 1.0) && (fabs(f->a1) < (1.0 + f->a2));
}
