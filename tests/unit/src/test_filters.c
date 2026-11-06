/*
 * SPEC F04: Tustin biquads. The discrete response at e^{jW} must equal the
 * continuous prototype at the pre-warped frequency 2 fs tan(W / 2) (that is
 * what the bilinear transform is), and the poles must be inside the unit circle.
 */
#include <math.h>
#include <zephyr/ztest.h>

#include "mcue/filters.h"

#define FS 500.0

ZTEST_SUITE(filters, NULL, NULL, NULL, NULL, NULL);

typedef struct {
	double re, im;
} cplx;

static cplx cdiv(cplx a, cplx b)
{
	double d = (b.re * b.re) + (b.im * b.im);

	return (cplx){((a.re * b.re) + (a.im * b.im)) / d, ((a.im * b.re) - (a.re * b.im)) / d};
}

/* H(e^{jW}) of the section */
static cplx discrete(const mc_biquad *f, double W)
{
	cplx n = {f->b0 + (f->b1 * cos(W)) + (f->b2 * cos(2.0 * W)),
		  -(f->b1 * sin(W)) - (f->b2 * sin(2.0 * W))};
	cplx d = {1.0 + (f->a1 * cos(W)) + (f->a2 * cos(2.0 * W)),
		  -(f->a1 * sin(W)) - (f->a2 * sin(2.0 * W))};

	return cdiv(n, d);
}

/* (n2 s^2 + n1 s + n0) / (d2 s^2 + d1 s + d0) at s = j w */
static cplx continuous(const double n[3], const double d[3], double w)
{
	cplx num = {n[2] - (n[0] * w * w), n[1] * w};
	cplx den = {d[2] - (d[0] * w * w), d[1] * w};

	return cdiv(num, den);
}

static void check_response(const mc_biquad *f, const double n[3], const double d[3])
{
	static const double freqs[] = {0.3, 1.0, 10.0}; /* rad/s */

	for (size_t k = 0; k < ARRAY_SIZE(freqs); k++) {
		double W = freqs[k] / FS;
		double warped = 2.0 * FS * tan(W / 2.0);
		cplx hd = discrete(f, W);
		cplx hc = continuous(n, d, warped);
		double mag = sqrt((hc.re * hc.re) + (hc.im * hc.im));

		zassert_within(hd.re, hc.re, 1e-9 * (1.0 + mag), "re at %.1f rad/s", freqs[k]);
		zassert_within(hd.im, hc.im, 1e-9 * (1.0 + mag), "im at %.1f rad/s", freqs[k]);
	}
}

ZTEST(filters, test_hp2_matches_prototype)
{
	mc_biquad f;
	double wn = 1.0, z = 1.0;

	zassert_true(mc_biquad_hp2(&f, wn, z, FS));
	check_response(&f, (const double[]){1, 0, 0}, (const double[]){1, 2 * z * wn, wn * wn});
	/* DC blocked, unity at Nyquist */
	zassert_within((f.b0 + f.b1 + f.b2) / (1 + f.a1 + f.a2), 0.0, 1e-12);
	zassert_within((f.b0 - f.b1 + f.b2) / (1 - f.a1 + f.a2), 1.0, 1e-12);
}

ZTEST(filters, test_lp2_matches_prototype)
{
	mc_biquad f;
	double wn = 4.0, z = 1.0;

	zassert_true(mc_biquad_lp2(&f, wn, z, FS));
	check_response(&f, (const double[]){0, 0, wn * wn}, (const double[]){1, 2 * z * wn, wn * wn});
	zassert_within((f.b0 + f.b1 + f.b2) / (1 + f.a1 + f.a2), 1.0, 1e-12);
}

ZTEST(filters, test_hp1_matches_prototype)
{
	mc_biquad f;
	double wb = 1.0;

	/* a first-order section must be strictly stable, not a cancelled pole at z = -1 */
	zassert_true(mc_biquad_hp1(&f, wb, FS));
	zassert_equal(f.a2, 0.0);
	check_response(&f, (const double[]){0, 1, 0}, (const double[]){0, 1, wb});
}

ZTEST(filters, test_resonator_matches_prototype)
{
	mc_biquad f;

	zassert_true(mc_biquad_tustin(&f, 0, 0, 1, 1, 6, 9, FS));
	check_response(&f, (const double[]){0, 0, 1}, (const double[]){1, 6, 9});
}

ZTEST(filters, test_unstable_prototype_rejected)
{
	mc_biquad f;

	/* s^2 - 2 s + 4: poles in the right half plane map outside the unit circle */
	zassert_false(mc_biquad_tustin(&f, 0, 0, 4, 1, -2, 4, FS));
	zassert_false(mc_biquad_tustin(&f, 0, 0, 0, 0, 0, 0, FS)); /* degenerate */
}

ZTEST(filters, test_step_and_reset)
{
	mc_biquad f;

	zassert_true(mc_biquad_lp2(&f, 4.0, 1.0, FS));
	double y = 0.0;

	for (int k = 0; k < 5 * (int)FS; k++) {
		y = mc_biquad_step(&f, 1.0);
	}
	zassert_within(y, 1.0, 1e-6, "LP2 settles to its DC gain");
	mc_biquad_reset(&f);
	zassert_within(mc_biquad_step(&f, 0.0), 0.0, 0.0);
}
