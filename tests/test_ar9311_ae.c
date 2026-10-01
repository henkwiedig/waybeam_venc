#include "ar9311_ae.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int expect(const char *name, int ok)
{
	printf("  %s  %s\n", ok ? "PASS" : "FAIL", name);
	return ok ? 0 : 1;
}

static void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

/* A grid whose every zone has channel mean @p level, except @p bright zones
 * (from the top-left, column-major) at 255. */
static void make_grid(uint8_t *rro, unsigned level, unsigned bright)
{
	const uint32_t count = 800;
	unsigned col, row, ch, n = 0;

	memset(rro, 0, AR9311_AE_RRO_SIZE);
	for (col = 0; col < AR9311_AE_COLS; col++) {
		uint8_t *column = rro + col * AR9311_AE_COL_STRIDE;

		for (row = 0; row < AR9311_AE_ROWS * AR9311_AE_CHANNELS; row++)
			put_le32(column + row * 4, count);
		for (row = 0; row < AR9311_AE_ROWS; row++, n++) {
			unsigned v = n < bright ? 255 : level;

			for (ch = 0; ch < AR9311_AE_CHANNELS; ch++)
				put_le32(column + AR9311_AE_BLOCK +
					(row * AR9311_AE_CHANNELS + ch) * 4, v * count);
		}
	}
}

static Ar9311AeLimits default_limits(void)
{
	Ar9311AeLimits lim = { 1, AR9311_AE_LINES_MAX, AR9311_AE_GAIN_ONE,
		64u * AR9311_AE_GAIN_ONE, AR9311_AE_TARGET };

	return lim;
}

static int test_gain_and_meter(void)
{
	static uint8_t rro[AR9311_AE_RRO_SIZE];
	Ar9311AeMeter m;
	int f = 0;

	f += expect("gain code 0x00 is 1x", ar9311_ae_gain_q10(0x00) == 1024);
	f += expect("gain code 0x10 is 2x", ar9311_ae_gain_q10(0x10) == 2048);
	f += expect("gain code 0x18 is 3x", ar9311_ae_gain_q10(0x18) == 3072);
	f += expect("gain code 0x60 is 64x", ar9311_ae_gain_q10(0x60) == 65536);
	f += expect("gain code clamps above 0x60",
		ar9311_ae_gain_q10(0x7f) == 65536);

	make_grid(rro, 80, 0);
	f += expect("meter flat grid", ar9311_ae_meter(rro, &m) == 0 &&
		fabsf(m.luma - 80.0f) < 0.01f && m.saturated == 0);
	make_grid(rro, 40, 64);
	f += expect("meter counts clipped zones",
		ar9311_ae_meter(rro, &m) == 0 && m.saturated == 64);
	memset(rro, 0, sizeof(rro));
	f += expect("meter rejects an empty grid", ar9311_ae_meter(rro, &m) != 0);
	return f;
}

static int test_split(void)
{
	Ar9311AeLimits lim = default_limits();
	Ar9311AePoint p;
	int f = 0;

	p = ar9311_ae_split(500.0, &lim);
	f += expect("short exposure uses time only", p.gain_code == 0 &&
		p.lines == 500);
	p = ar9311_ae_split(AR9311_AE_LINES_MAX * 3.0, &lim);
	f += expect("long exposure fills the frame, then gains",
		p.lines == AR9311_AE_LINES_MAX && p.gain_code == 0x18);
	p = ar9311_ae_split(AR9311_AE_LINES_MAX * 2.9, &lim);
	/* Lines are at their ceiling, so only the code can match: within half
	 * a gain step (1/16 = 6.25% at the bottom of an octave). */
	f += expect("lines at the ceiling: nearest gain code",
		fabs(ar9311_ae_exposure_of(&p) - AR9311_AE_LINES_MAX * 2.9) <
			AR9311_AE_LINES_MAX * 2.9 * 0.032);
	p = ar9311_ae_split(800.0 * 1.3, &lim);
	f += expect("lines below the ceiling absorb the rounding",
		fabs(ar9311_ae_exposure_of(&p) - 800.0 * 1.3) < 1.5);
	lim.gain_max_q10 = 4u * AR9311_AE_GAIN_ONE;
	p = ar9311_ae_split(AR9311_AE_LINES_MAX * 30.0, &lim);
	f += expect("gain ceiling holds", ar9311_ae_gain_q10(p.gain_code) <=
		lim.gain_max_q10 && p.lines == AR9311_AE_LINES_MAX);
	lim = default_limits();
	lim.lines_max = 500;
	p = ar9311_ae_split(1000.0, &lim);
	f += expect("shutter ceiling moves exposure into gain",
		p.lines <= 500 && p.gain_code == 0x10);
	return f;
}

/* Linear scene: luma = exposure * k, clipped. */
static int converge(double k, double *exposure, int *steps)
{
	Ar9311AeLimits lim = default_limits();
	static uint8_t rro[AR9311_AE_RRO_SIZE];
	Ar9311AeMeter m;
	int i, quiet = 0;

	for (i = 0; i < 60 && quiet < 3; i++) {
		double luma = *exposure * k;
		Ar9311AePoint p;

		make_grid(rro, luma > 255 ? 255u : (unsigned)luma, 0);
		ar9311_ae_meter(rro, &m);
		quiet = ar9311_ae_step(exposure, &m, &lim) ? 0 : quiet + 1;
		p = ar9311_ae_split(*exposure, &lim);
		*exposure = ar9311_ae_exposure_of(&p);
	}
	*steps = i;
	return quiet >= 3;
}

static int test_step(void)
{
	Ar9311AeLimits lim = default_limits();
	static uint8_t rro[AR9311_AE_RRO_SIZE];
	Ar9311AeMeter m;
	double e = 1000.0;
	int steps, f = 0;

	make_grid(rro, AR9311_AE_TARGET + 2, 0);
	ar9311_ae_meter(rro, &m);
	f += expect("deadband holds near target",
		ar9311_ae_step(&e, &m, &lim) == 0 && e == 1000.0);

	e = 20000.0;   /* far too bright for k below */
	f += expect("bright scene converges", converge(0.01, &e, &steps) &&
		fabs(e * 0.01 - AR9311_AE_TARGET) < AR9311_AE_TARGET * 0.1);
	e = 50.0;      /* dark scene: target needs ~40000 = 1123 lines x 36x */
	f += expect("dark scene converges into gain",
		converge(AR9311_AE_TARGET / 40000.0, &e, &steps) &&
		fabs(e - 40000.0) < 40000.0 * 0.1);
	e = 1000.0;    /* darker than the ceiling can fix: saturates there */
	converge(0.0000001, &e, &steps);
	f += expect("pitch dark pins the ceiling",
		fabs(e - AR9311_AE_LINES_MAX * 64.0) < 1.0);

	make_grid(rro, 30, 200);   /* a third of the frame clipped */
	ar9311_ae_meter(rro, &m);
	e = 5000.0;
	ar9311_ae_step(&e, &m, &lim);
	f += expect("clipped highlights pull exposure down", e < 5000.0);
	return f;
}

int main(void)
{
	int failures = test_gain_and_meter() + test_split() + test_step();

	printf("ar9311 ae: %s\n", failures ? "FAILED" : "ok");
	return failures ? 1 : 0;
}
