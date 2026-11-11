#include "ar9311_ae.h"
#include "ar9311_awb.h"

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

static uint8_t g_rro[AR9311_AE_RRO_SIZE];

/* Every zone a surface of green @p g with the given R/G and B/G, in the
 * SC231AI's channel layout (0 and 3 green, 1 red, 2 blue). */
static void fill(double g, double rg, double bg)
{
	const uint32_t count = 800;
	unsigned blk, i;

	memset(g_rro, 0, sizeof(g_rro));
	for (blk = 0; blk < AR9311_AE_COLS; blk++) {
		uint8_t *col = g_rro + blk * AR9311_AE_COL_STRIDE;

		for (i = 0; i < AR9311_AE_ROWS * AR9311_AE_CHANNELS; i++)
			put_le32(col + i * 4, count);
	}
	for (i = 0; i < AR9311_AE_COLS * AR9311_AE_ROWS; i++) {
		uint8_t *zone = g_rro + (i / AR9311_AE_ROWS) * AR9311_AE_COL_STRIDE +
			AR9311_AE_BLOCK + (i % AR9311_AE_ROWS) * AR9311_AE_CHANNELS * 4;

		put_le32(zone + 0, (uint32_t)(g * count));
		put_le32(zone + 4, (uint32_t)(g * rg * count));
		put_le32(zone + 8, (uint32_t)(g * bg * count));
		put_le32(zone + 12, (uint32_t)(g * count));
	}
}

/* Overwrite the first @p n zones with another surface. */
static void paint(unsigned n, double g, double rg, double bg)
{
	const uint32_t count = 800;
	unsigned i;

	for (i = 0; i < n; i++) {
		uint8_t *zone = g_rro + (i / AR9311_AE_ROWS) * AR9311_AE_COL_STRIDE +
			AR9311_AE_BLOCK + (i % AR9311_AE_ROWS) * AR9311_AE_CHANNELS * 4;

		put_le32(zone + 0, (uint32_t)(g * count));
		put_le32(zone + 4, (uint32_t)(g * rg * count));
		put_le32(zone + 8, (uint32_t)(g * bg * count));
		put_le32(zone + 12, (uint32_t)(g * count));
	}
}

static int near(uint32_t v, uint32_t want, uint32_t tol)
{
	return v + tol >= want && v <= want + tol;
}

static int test_estimate(const Ar9311AwbSensor *s)
{
	Ar9311AwbEstimate e;
	Ar9311AwbGains g;
	int f = 0;

	fill(100, s->day_rg, s->day_bg);
	f += expect("daylight grey is found",
		ar9311_awb_estimate(g_rro, s, &e) == 0 && fabs(e.t - 1.0) < 0.01);
	g = ar9311_awb_gains_for(&e);
	f += expect("daylight grey gets the daylight gains",
		near(g.red, 461, 2) && near(g.blue, 743, 3));

	fill(100, s->warm_rg, s->warm_bg);
	f += expect("lamp-lit grey is found",
		ar9311_awb_estimate(g_rro, s, &e) == 0 && fabs(e.t) < 0.01);
	g = ar9311_awb_gains_for(&e);
	f += expect("lamp-lit grey gets the warm gains",
		near(g.red, 310, 2) && near(g.blue, 962, 3));

	/* Half the frame a red object: off the line, so it does not count. */
	fill(100, s->day_rg, s->day_bg);
	paint(288, 100, 2.0, s->day_bg);
	f += expect("a red object does not pull the balance",
		ar9311_awb_estimate(g_rro, s, &e) == 0 && fabs(e.t - 1.0) < 0.01 &&
		e.zones == 288);

	/* Clipped and dark zones are ignored, whatever their colour. */
	fill(100, s->day_rg, s->day_bg);
	paint(200, 250, s->warm_rg, s->warm_bg);
	paint(100, 4, s->warm_rg, s->warm_bg);
	f += expect("clipped and dark zones are ignored",
		ar9311_awb_estimate(g_rro, s, &e) == 0 && fabs(e.t - 1.0) < 0.01);

	/* A frame of saturated colour has nothing neutral to go by. */
	fill(100, 2.0, 0.1);
	f += expect("no neutral zones: no estimate",
		ar9311_awb_estimate(g_rro, s, &e) != 0);

	memset(g_rro, 0, sizeof(g_rro));
	f += expect("an empty grid is rejected",
		ar9311_awb_estimate(g_rro, s, &e) != 0);
	return f;
}

static int test_step(void)
{
	Ar9311AwbGains g = { 461, 743 }, target = { 310, 962 };
	int i, moved = 0, f = 0;

	for (i = 0; i < 60; i++)
		moved += ar9311_awb_step(&g, &target);
	f += expect("steps converge on the target",
		near(g.red, 310, 3) && near(g.blue, 962, 9) && moved > 5);
	g = target;
	f += expect("deadband holds at the target",
		ar9311_awb_step(&g, &target) == 0 &&
		g.red == target.red && g.blue == target.blue);
	g.red = 300;
	target.red = 304;   /* 1.3 %: outside the deadband, below one step */
	f += expect("a small error still makes progress",
		ar9311_awb_step(&g, &target) == 1 && g.red == 301);
	return f;
}

static int test_ct(const Ar9311AwbSensor *s)
{
	Ar9311AwbEstimate e;
	Ar9311AwbGains g;
	int f = 0;

	e = ar9311_awb_at_ct(s, 2800);
	f += expect("2800 K is the warm point", fabs(e.t) < 1e-6);
	e = ar9311_awb_at_ct(s, 6500);
	g = ar9311_awb_gains_for(&e);
	f += expect("6500 K is the daylight point",
		fabs(e.t - 1.0) < 1e-6 && near(g.red, 461, 2) && near(g.blue, 743, 3));
	e = ar9311_awb_at_ct(s, 4500);
	f += expect("colour temperature round-trips",
		near(ar9311_awb_ct_of(&e), 4500, 2));
	e = ar9311_awb_at_ct(s, 100000);
	f += expect("extreme colour temperatures clamp",
		e.t <= 1.8 + 1e-9 && e.t > 1.0);
	return f;
}

int main(void)
{
	const Ar9311AwbSensor *s = ar9311_awb_sensor("sc231ai");
	int f = 0;

	f += expect("sc231ai has a calibration", s != NULL);
	f += expect("nt99235 has none yet", ar9311_awb_sensor("nt99235") == NULL);
	f += expect("no sensor, no calibration", ar9311_awb_sensor(NULL) == NULL);
	if (!s)
		return 1;
	f += test_estimate(s);
	f += test_step();
	f += test_ct(s);
	printf("ar9311 awb: %s\n", f ? "FAILED" : "ok");
	return f ? 1 : 0;
}
