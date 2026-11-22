#include "ar9311_ae.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define AE_STATS_PATH "/sys/kernel/debug/ar-isp/stats_raw"
#define AE_LADDER_ARM "/sys/kernel/debug/ar-isp/ladders"
#define AE_TUNING_PARAM "/sys/module/ar_isp/parameters/tuning"
#define AE_TONE_SCALAR "/sys/module/ar_isp/parameters/tone_scalar"
#define AE_TONE_REBUILD "/sys/kernel/debug/ar-isp/tone"
/* cm and cm2, the ISP's exposure-keyed saturation stages, read the same
 * scalar; their re-latch is the colour node, absent from drivers without
 * it.  cm2 runs on the NT99235 and is gated off on the SC231AI. */
#define AE_CM_TRIGGER "/sys/module/ar_isp/parameters/cm_trigger"
#define AE_CM2_TRIGGER "/sys/module/ar_isp/parameters/cm2_trigger"
#define AE_COLOUR_RELATCH "/sys/kernel/debug/ar-isp/colour"
/* The vendor AE's exposure table in a tuning file (ar9311_ae.h): entry
 * count, then {gain Q8, lines} records. */
#define AE_TONE_COUNT_OFF 0x0b6508u
#define AE_TONE_TABLE_OFF 0x0b6524u
/* The P1 is built with an NT99235 or an SC231AI camera.  Both drivers take
 * exposure (lines) and gain (the same code scale) as writable module
 * parameters; the AE drives whichever one bound the camera node, within
 * that driver's integration clamp and gain ceiling, metering the channels
 * that track its brightness.
 *
 * The SC231AI goes to 640x: in the dark the vendor's AE runs its analogue
 * gain to 116.5x and adds the sensor's digital gain up to 5.5x (641x), at
 * the same 1/60 s; stopping at the NT99235's 64x left a dark room 2.5-3.5
 * stops under the target.
 *
 * The target scales with the sensor's tone curve: the SC231AI's tuning
 * gamma is bright (its test pattern measured raw 14 % -> 148, 69 % -> 253,
 * mid-grey near raw 10 %), so the same metered target as the NT99235
 * overexposed it by a stop with the brightest curve pinned.  With the tone
 * pages following the exposure as stock's do, its gamma and DRC pages
 * matched stock's at the same light, and stock exposed 0.53 EV more than a
 * half target in sunshine (39 against 27 lines), 0.35 EV more at dusk: the
 * SC231AI's 0 EV is +0.5 EV on the half.  The NT99235, side by side with
 * a stock P1 in the same sunshine, came out the same: stock 86 lines where
 * our target of 56 took 61, its tone pages likewise following the exposure
 * (its gamma page matched stock's at the exposure's table position).  Its
 * 0 EV is +0.5 EV on 56.  Exposure compensation stays relative to either. */
typedef struct {
	const char *name;
	uint32_t lines_max;
	unsigned gain_code_max;
	unsigned channels;      /* AR9311_AE_METER_* */
	unsigned target_q8;     /* scale on the metered target, 256 = 1x */
	int tone;               /* drive the ISP's tone pages (ar9311_ae.h) */
} AeSensor;

static const AeSensor g_ae_sensors[] = {
	{ "nt99235", AR9311_AE_LINES_MAX, 0x60u, AR9311_AE_METER_ALL, 362u, 1 },
	/* 1125 - 11 lines; 640x */
	{ "sc231ai", 1114u, AR9311_AE_GAIN_CODE_MAX, AR9311_AE_METER_GREEN_03,
		181u, 1 },
};
static const AeSensor *g_ae_sensor;
static char g_ae_exposure_path[64];
static char g_ae_gain_path[64];
#define AE_SENSOR_EXPOSURE g_ae_exposure_path
#define AE_SENSOR_GAIN g_ae_gain_path

/* The config's limits for the bound sensor: within its integration clamp
 * and gain ceiling, so the AE never counts on lines or gain the driver would
 * cut, and with the target scaled to its tone curve. */
static Ar9311AeLimits sensor_limits(const Ar9311AeLimits *limits)
{
	Ar9311AeLimits lim = *limits;
	uint32_t gain_max;

	if (!g_ae_sensor)
		return lim;
	if (lim.lines_max > g_ae_sensor->lines_max)
		lim.lines_max = g_ae_sensor->lines_max;
	if (lim.lines_min > lim.lines_max)
		lim.lines_min = lim.lines_max;
	gain_max = ar9311_ae_gain_q10(g_ae_sensor->gain_code_max);
	if (lim.gain_max_q10 > gain_max)
		lim.gain_max_q10 = gain_max;
	if (lim.gain_min_q10 > lim.gain_max_q10)
		lim.gain_min_q10 = lim.gain_max_q10;
	lim.target = (lim.target * g_ae_sensor->target_q8 + 128u) / 256u;
	if (lim.target < 1)
		lim.target = 1;
	return lim;
}

/* ISP stages whose strength the driver looks up by sensor gain (Q8,
 * 256 = 1x): raw and luma noise reduction, 3D denoise, demosaic and chroma
 * noise filtering.  Left at 1x they denoise a 30x picture like a 1x one. */
static const char *const g_ae_ladders[] = {
	"/sys/module/ar_isp/parameters/rnr_gain",
	"/sys/module/ar_isp/parameters/lnr_gain",
	"/sys/module/ar_isp/parameters/de3d_gain",
	"/sys/module/ar_isp/parameters/cfa_gain",
	"/sys/module/ar_isp/parameters/cnf_gain",
};

/* Control law.  A zone counts as clipped from this mean up.  Above the
 * clipped share, the target is lowered in proportion so a bright sky does
 * not wash out the rest; never below half of it.  Inside the deadband the
 * exposure holds; outside, each decision corrects half the error in the log
 * domain, which converges without overshoot across the sensor's 1-2 frame
 * latency at the decision rates used here. */
#define AE_CLIP_LUMA 240.0f
#define AE_CLIP_SHARE 0.04f
#define AE_DEADBAND 0.08
#define AE_GAIN_STEP 0.5
#define AE_SPLIT_SLACK 0.02
/* Anti-flicker: a whole number of half periods may overrun the line limit
 * by this much (60 Hz: 2 half periods are 1125 lines against 1123). */
#define AE_FLICKER_SLACK 1.01

/* ── Pure part ───────────────────────────────────────────────────────── */

uint32_t ar9311_ae_gain_q10(unsigned code)
{
	if (code > AR9311_AE_GAIN_CODE_MAX)
		code = AR9311_AE_GAIN_CODE_MAX;
	return (64u * (16u + (code & 15u))) << (code >> 4);
}

static uint32_t get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
		(uint32_t)p[3] << 24;
}

/* Centre-weighted: the middle half of the frame in each direction counts
 * double, as an FPV pilot looks there and not at the sky or the frame. */
static int zone_weight(unsigned col, unsigned row)
{
	return (col >= AR9311_AE_COLS / 4 && col < AR9311_AE_COLS * 3 / 4 &&
		row >= AR9311_AE_ROWS / 4 && row < AR9311_AE_ROWS * 3 / 4) ? 2 : 1;
}

int ar9311_ae_meter(const uint8_t *rro, unsigned channels, Ar9311AeMeter *out)
{
	double sum = 0.0;
	unsigned weights = 0, metered = 0;
	unsigned col, row, ch;

	channels &= AR9311_AE_METER_ALL;
	if (channels == 0)
		channels = AR9311_AE_METER_ALL;
	for (ch = 0; ch < AR9311_AE_CHANNELS; ch++)
		metered += (channels >> ch) & 1u;
	out->luma = 0.0f;
	out->saturated = 0;
	for (col = 0; col < AR9311_AE_COLS; col++) {
		const uint8_t *column = rro + col * AR9311_AE_COL_STRIDE;
		uint32_t count = get_le32(column);

		if (count == 0)
			return -1;
		for (row = 0; row < AR9311_AE_ROWS; row++) {
			const uint8_t *zone = column + AR9311_AE_BLOCK +
				row * AR9311_AE_CHANNELS * 4;
			double zone_sum = 0.0;
			float mean;
			int w = zone_weight(col, row);

			for (ch = 0; ch < AR9311_AE_CHANNELS; ch++)
				if (channels & (1u << ch))
					zone_sum += get_le32(zone + ch * 4);
			mean = (float)(zone_sum / metered / count);
			if (mean >= AE_CLIP_LUMA)
				out->saturated++;
			sum += (double)mean * w;
			weights += (unsigned)w;
		}
	}
	out->luma = (float)(sum / weights);
	return 0;
}

unsigned ar9311_ae_tone_table(const uint8_t *blob, size_t len, uint64_t *ev)
{
	uint32_t count, i;

	if (len < AE_TONE_TABLE_OFF)
		return 0;
	count = get_le32(blob + AE_TONE_COUNT_OFF);
	if (count < 2 || count > AR9311_AE_TONE_TABLE_MAX ||
	    AE_TONE_TABLE_OFF + (size_t)count * 8u > len)
		return 0;
	for (i = 0; i < count; i++) {
		const uint8_t *rec = blob + AE_TONE_TABLE_OFF + i * 8u;

		ev[i] = (uint64_t)get_le32(rec) * get_le32(rec + 4);
		if (ev[i] == 0 || (i > 0 && ev[i] < ev[i - 1]))
			return 0;   /* not an exposure table */
	}
	return count;
}

uint32_t ar9311_ae_tone_scalar(const uint64_t *ev, unsigned n, double exposure)
{
	double e = exposure * 256.0;   /* the table's gain is Q8 */
	unsigned i;

	if (n < 2 || !(e > (double)ev[0]))
		return 0;
	for (i = 1; i < n; i++)
		if ((double)ev[i] >= e) {
			double span = (double)(ev[i] - ev[i - 1]);
			double frac = span > 0.0 ? (e - (double)ev[i - 1]) / span : 0.0;

			return (uint32_t)(((double)(i - 1) + frac) * 256.0 + 0.5);
		}
	return (n - 1) * 256u;
}

double ar9311_ae_exposure_of(const Ar9311AePoint *pt)
{
	return (double)pt->lines * ar9311_ae_gain_q10(pt->gain_code) /
		AR9311_AE_GAIN_ONE;
}

static unsigned code_at_or_below(uint32_t gain_q10, unsigned min_code,
	unsigned max_code)
{
	unsigned code = min_code;

	while (code < max_code && ar9311_ae_gain_q10(code + 1) <= gain_q10)
		code++;
	return code;
}

static unsigned code_at_or_above(uint32_t gain_q10)
{
	unsigned code = 0;

	while (code < AR9311_AE_GAIN_CODE_MAX && ar9311_ae_gain_q10(code) < gain_q10)
		code++;
	return code;
}

static double clamp_d(double v, double lo, double hi)
{
	return v < lo ? lo : v > hi ? hi : v;
}

/* The point for one gain code: lines carry whatever the code leaves. */
static Ar9311AePoint point_for_code(double exposure, unsigned code,
	const Ar9311AeLimits *lim)
{
	Ar9311AePoint pt;
	double lines = exposure * AR9311_AE_GAIN_ONE / ar9311_ae_gain_q10(code);

	pt.gain_code = code;
	pt.lines = (uint32_t)clamp_d(lines + 0.5, lim->lines_min, lim->lines_max);
	return pt;
}

/* Gain code nearest to @p gain_q10 within the limits. */
static unsigned nearest_code(double gain_q10, unsigned min_code,
	unsigned max_code)
{
	unsigned lo = code_at_or_below(gain_q10 > 0 ? (uint32_t)gain_q10 : 0,
		min_code, max_code);

	if (lo < max_code &&
	    fabs(ar9311_ae_gain_q10(lo + 1) - gain_q10) <
		fabs(ar9311_ae_gain_q10(lo) - gain_q10))
		return lo + 1;
	return lo;
}

/* Mains light ripples at twice its frequency, so an integration time that is
 * a whole number of half periods sees the same light on every row and the
 * picture carries no bands.  Lines go down to such a multiple (never up:
 * that would overexpose) and the gain makes up the rest.  Returns 0 when the
 * constraint cannot apply: off, an exposure below one half period (bright,
 * and short exposures cannot be made band-free by timing), or a line limit
 * shorter than one half period. */
static int split_flicker(double exposure, const Ar9311AeLimits *lim,
	unsigned min_code, unsigned max_code, Ar9311AePoint *pt)
{
	double half = 1e9 / (2.0 * lim->flicker_hz) / AR9311_AE_LINE_NS;
	double min_gain = ar9311_ae_gain_q10(min_code) / (double)AR9311_AE_GAIN_ONE;
	double want_lines;
	unsigned n, n_max;

	if (lim->flicker_hz != 50 && lim->flicker_hz != 60)
		return 0;
	n_max = (unsigned)(lim->lines_max * AE_FLICKER_SLACK / half);
	if (n_max == 0 || exposure < half * min_gain)
		return 0;
	want_lines = exposure / min_gain;
	n = (unsigned)(want_lines / half);
	if (n < 1)
		n = 1;
	if (n > n_max)
		n = n_max;
	pt->lines = (uint32_t)clamp_d(n * half + 0.5, lim->lines_min,
		lim->lines_max);
	pt->gain_code = nearest_code(exposure / pt->lines * AR9311_AE_GAIN_ONE,
		min_code, max_code);
	return 1;
}

Ar9311AePoint ar9311_ae_split(double exposure, const Ar9311AeLimits *lim)
{
	unsigned min_code = code_at_or_above(lim->gain_min_q10);
	unsigned max_code = code_at_or_below(lim->gain_max_q10, 0,
		AR9311_AE_GAIN_CODE_MAX);
	Ar9311AePoint lo, hi;
	double lines, need;

	if (max_code < min_code)
		max_code = min_code;
	if (split_flicker(exposure, lim, min_code, max_code, &lo))
		return lo;
	lines = clamp_d(exposure, lim->lines_min, lim->lines_max);
	need = exposure / lines * AR9311_AE_GAIN_ONE;
	lo = point_for_code(exposure, code_at_or_below(need > 0 ? (uint32_t)need : 0,
		min_code, max_code), lim);
	if (lo.gain_code >= max_code)
		return lo;
	/* With the lines at a limit they cannot absorb the code's rounding, and
	 * the lower code can fall short by up to a gain step (3-6%).  Take the
	 * next code up only when that shortfall matters: below 2% it is well
	 * inside the control deadband, and the lower gain is the less noisy
	 * picture. */
	if (exposure - ar9311_ae_exposure_of(&lo) <= exposure * AE_SPLIT_SLACK)
		return lo;
	hi = point_for_code(exposure, lo.gain_code + 1, lim);
	return fabs(ar9311_ae_exposure_of(&hi) - exposure) <
		fabs(ar9311_ae_exposure_of(&lo) - exposure) ? hi : lo;
}

static float effective_target(const Ar9311AeMeter *meter, uint32_t target)
{
	float share = (float)meter->saturated /
		(AR9311_AE_COLS * AR9311_AE_ROWS);
	float scale = 1.0f;

	if (share > AE_CLIP_SHARE) {
		scale = AE_CLIP_SHARE / share;
		if (scale < 0.5f)
			scale = 0.5f;
	}
	return (float)target * scale;
}

int ar9311_ae_step(double *exposure, const Ar9311AeMeter *meter,
	const Ar9311AeLimits *lim)
{
	double ratio, next;
	double lo = (double)lim->lines_min *
		ar9311_ae_gain_q10(code_at_or_above(lim->gain_min_q10)) /
		AR9311_AE_GAIN_ONE;
	double hi = (double)lim->lines_max * lim->gain_max_q10 / AR9311_AE_GAIN_ONE;

	ratio = effective_target(meter, lim->target) /
		(meter->luma > 1.0f ? meter->luma : 1.0f);
	if (fabs(ratio - 1.0) < AE_DEADBAND)
		return 0;
	next = clamp_d(*exposure * pow(ratio, AE_GAIN_STEP), lo, hi > lo ? hi : lo);
	if (fabs(next - *exposure) < 0.5)
		return 0;
	*exposure = next;
	return 1;
}

/* ── Runtime ─────────────────────────────────────────────────────────── */

struct Ar9311Ae {
	pthread_t thread;
	pthread_mutex_t lock;
	int stop;
	uint32_t period_us;
	unsigned channels;   /* the bound sensor's AR9311_AE_METER_* */
	Ar9311AeLimits limits;
	/* Status, under lock. */
	Ar9311AeMeter meter;
	Ar9311AePoint point;
	double exposure;
	unsigned long decisions, moves;
	int have_stats;
	/* The tone pages (ar9311_ae.h): the tuning file's exposure table, and
	 * the scalar last given to the ISP (-1 = none yet). */
	uint64_t *tone_ev;
	unsigned tone_n;
	long tone_scalar;
	int colour;   /* the driver re-latches cm on the scalar (colour node) */
};

/* The scalar for the AWB's daylight test (ar9311_ae_tone_now). */
static long g_ae_tone_now = -1;

long ar9311_ae_tone_now(void)
{
	return __atomic_load_n(&g_ae_tone_now, __ATOMIC_RELAXED);
}

uint32_t ar9311_ae_target_for_ev(double ev)
{
	double t;

	if (!(ev >= -AR9311_AE_EV_MAX))   /* also catches NaN */
		ev = -AR9311_AE_EV_MAX;
	if (ev > AR9311_AE_EV_MAX)
		ev = AR9311_AE_EV_MAX;
	t = AR9311_AE_TARGET * pow(2.0, ev) + 0.5;
	if (t < AR9311_AE_TARGET_MIN)
		return AR9311_AE_TARGET_MIN;
	if (t > AR9311_AE_TARGET_MAX)
		return AR9311_AE_TARGET_MAX;
	return (uint32_t)t;
}

static int write_int(const char *path, long value)
{
	char buf[24];
	int fd, len, ok;

	fd = open(path, O_WRONLY | O_CLOEXEC);
	if (fd < 0)
		return -1;
	len = snprintf(buf, sizeof(buf), "%ld\n", value);
	ok = write(fd, buf, (size_t)len) == len;
	close(fd);
	return ok ? 0 : -1;
}

static int read_int(const char *path, long *value)
{
	FILE *f = fopen(path, "r");
	int ok;

	if (!f)
		return -1;
	ok = fscanf(f, "%ld", value) == 1;
	fclose(f);
	return ok ? 0 : -1;
}

/* One coherent snapshot: the driver refuses (EAGAIN) until statistics exist
 * and a flip can land mid-copy, which the two sequence words expose. */
int ar9311_stats_read(uint8_t *buf, uint32_t *seq)
{
	int fd = open(AE_STATS_PATH, O_RDONLY | O_CLOEXEC);
	size_t got = 0;
	ssize_t n;

	if (fd < 0)
		return -1;
	while (got < AR9311_AE_STATS_SIZE &&
	       (n = read(fd, buf + got, AR9311_AE_STATS_SIZE - got)) > 0)
		got += (size_t)n;
	close(fd);
	if (got != AR9311_AE_STATS_SIZE ||
	    get_le32(buf) != get_le32(buf + AR9311_AE_STATS_SIZE - 4))
		return -1;
	*seq = get_le32(buf);
	return 0;
}

static void (*g_ae_gain_hook)(uint32_t gain_q10);

void ar9311_ae_set_gain_hook(void (*hook)(uint32_t gain_q10))
{
	g_ae_gain_hook = hook;
}

/* Sensor first, then the ISP ladders when the gain moved, re-armed so the
 * driver re-reads them. */
static int apply_point(const Ar9311AePoint *pt, const Ar9311AePoint *prev)
{
	size_t i;
	long q8;

	if (pt->lines != prev->lines &&
	    write_int(AE_SENSOR_EXPOSURE, (long)pt->lines) != 0)
		return -1;
	if (pt->gain_code == prev->gain_code)
		return 0;
	if (write_int(AE_SENSOR_GAIN, (long)pt->gain_code) != 0)
		return -1;
	q8 = (long)(ar9311_ae_gain_q10(pt->gain_code) / 4u);
	for (i = 0; i < sizeof(g_ae_ladders) / sizeof(g_ae_ladders[0]); i++)
		if (write_int(g_ae_ladders[i], q8) != 0)
			return -1;
	if (g_ae_gain_hook)
		g_ae_gain_hook(ar9311_ae_gain_q10(pt->gain_code));
	return write_int(AE_LADDER_ARM, 1);
}

static void sleep_us(uint32_t us)
{
	struct timespec ts = { us / 1000000u, (long)(us % 1000000u) * 1000 };

	nanosleep(&ts, NULL);
}

/* The tone pages follow the exposure the scene needs, rebuilt when it has
 * moved by a whole table entry (~3 %). */
static void apply_tone(Ar9311Ae *ae, double exposure)
{
	long scalar = (long)ar9311_ae_tone_scalar(ae->tone_ev, ae->tone_n,
		exposure);

	if (ae->tone_scalar >= 0 && labs(scalar - ae->tone_scalar) < 256)
		return;
	if (write_int(AE_TONE_SCALAR, scalar) != 0 ||
	    write_int(AE_TONE_REBUILD, 1) != 0) {
		fprintf(stderr, "WARN: [ar9311] AE: tone pages not updated: %s\n",
			strerror(errno));
		return;
	}
	if (ae->colour && (write_int(AE_CM_TRIGGER, scalar) != 0 ||
	    write_int(AE_CM2_TRIGGER, scalar) != 0 ||
	    write_int(AE_COLOUR_RELATCH, 1) != 0))
		fprintf(stderr, "WARN: [ar9311] AE: cm not updated: %s\n",
			strerror(errno));
	pthread_mutex_lock(&ae->lock);
	ae->tone_scalar = scalar;
	pthread_mutex_unlock(&ae->lock);
	__atomic_store_n(&g_ae_tone_now, scalar, __ATOMIC_RELAXED);
}

/* The exposure table of the tuning file the ISP runs (ar_isp's tuning
 * parameter, a firmware path), or none. */
static void load_tone(Ar9311Ae *ae)
{
	char name[128], path[192];
	uint8_t *blob = NULL;
	long len;
	FILE *f;

	f = fopen(AE_TUNING_PARAM, "r");
	if (!f)
		return;
	if (!fgets(name, sizeof(name), f))
		name[0] = '\0';
	fclose(f);
	name[strcspn(name, "\n")] = '\0';
	if (!name[0] || access(AE_TONE_SCALAR, W_OK) != 0)
		return;
	snprintf(path, sizeof(path), "/lib/firmware/%s", name);
	f = fopen(path, "rb");
	if (!f)
		return;
	if (fseek(f, 0, SEEK_END) == 0 && (len = ftell(f)) > 0 &&
	    fseek(f, 0, SEEK_SET) == 0 && (blob = malloc((size_t)len)) &&
	    fread(blob, 1, (size_t)len, f) == (size_t)len &&
	    (ae->tone_ev = malloc(AR9311_AE_TONE_TABLE_MAX * sizeof(uint64_t))))
		ae->tone_n = ar9311_ae_tone_table(blob, (size_t)len, ae->tone_ev);
	fclose(f);
	free(blob);
	ae->colour = ae->tone_n && access(AE_CM_TRIGGER, W_OK) == 0 &&
		access(AE_CM2_TRIGGER, W_OK) == 0 &&
		access(AE_COLOUR_RELATCH, W_OK) == 0;
	if (ae->tone_n)
		printf("> [ar9311] AE drives the tone pages%s (%s, %u exposures)\n",
			ae->colour ? " and cm" : "", name, ae->tone_n);
	else
		fprintf(stderr, "WARN: [ar9311] AE: no exposure table in %s, "
			"tone pages stay pinned\n", path);
}

static void decide(Ar9311Ae *ae, const uint8_t *stats)
{
	Ar9311AeMeter meter;
	Ar9311AeLimits lim;
	Ar9311AePoint next, prev;
	double exposure;
	int moved;

	if (ar9311_ae_meter(stats + 4, ae->channels, &meter) != 0)
		return;
	pthread_mutex_lock(&ae->lock);
	lim = sensor_limits(&ae->limits);
	exposure = ae->exposure;
	prev = ae->point;
	pthread_mutex_unlock(&ae->lock);

	moved = ar9311_ae_step(&exposure, &meter, &lim);
	/* Limits can change underneath a settled scene; re-split every time so
	 * a tightened ceiling takes effect without waiting for a move. */
	next = ar9311_ae_split(exposure, &lim);
	if ((next.lines != prev.lines || next.gain_code != prev.gain_code) &&
	    apply_point(&next, &prev) != 0)
		fprintf(stderr, "WARN: [ar9311] AE: writing the operating point "
			"failed: %s\n", strerror(errno));
	if (ae->tone_n)
		apply_tone(ae, exposure);

	pthread_mutex_lock(&ae->lock);
	ae->meter = meter;
	ae->point = next;
	ae->exposure = ar9311_ae_exposure_of(&next);
	ae->decisions++;
	ae->moves += (unsigned long)moved;
	ae->have_stats = 1;
	pthread_mutex_unlock(&ae->lock);
}

static void *ae_thread(void *opaque)
{
	Ar9311Ae *ae = opaque;
	uint8_t *buf = malloc(AR9311_AE_STATS_SIZE);
	uint32_t seq, last_seq = 0;
	int have_seq = 0;

	if (!buf)
		return NULL;
	while (!__atomic_load_n(&ae->stop, __ATOMIC_ACQUIRE)) {
		sleep_us(ae->period_us);
		/* No new frame since the last decision: the sensor has not
		 * shown the last change yet, so deciding again would double
		 * it.  Missing statistics (before the first frame, after an
		 * ISP reconfigure) are simply waited out. */
		if (ar9311_stats_read(buf, &seq) != 0 ||
		    (have_seq && seq == last_seq))
			continue;
		last_seq = seq;
		have_seq = 1;
		decide(ae, buf);
	}
	free(buf);
	return NULL;
}

/* Has the i2c driver @p name bound a device?  Its sysfs directory then holds
 * a link named after the client ("0-001a"). */
static int sensor_bound(const char *name)
{
	char dir[64];
	struct dirent *de;
	DIR *d;
	int bound = 0;

	snprintf(dir, sizeof(dir), "/sys/bus/i2c/drivers/%s", name);
	d = opendir(dir);
	if (!d)
		return 0;
	while (!bound && (de = readdir(d)) != NULL)
		bound = de->d_name[0] >= '0' && de->d_name[0] <= '9' &&
			strchr(de->d_name, '-') != NULL;
	closedir(d);
	return bound;
}

/* Point the sensor paths at the bound sensor's parameters.  0, or -1 when
 * no supported sensor driver has bound. */
static int find_sensor(void)
{
	size_t i;

	for (i = 0; i < sizeof(g_ae_sensors) / sizeof(g_ae_sensors[0]); i++) {
		const AeSensor *s = &g_ae_sensors[i];

		if (!sensor_bound(s->name))
			continue;
		snprintf(g_ae_exposure_path, sizeof(g_ae_exposure_path),
			"/sys/module/%s/parameters/exposure", s->name);
		snprintf(g_ae_gain_path, sizeof(g_ae_gain_path),
			"/sys/module/%s/parameters/gain", s->name);
		g_ae_sensor = s;
		printf("> [ar9311] AE drives the %s\n", s->name);
		return 0;
	}
	return -1;
}

const char *ar9311_sensor_name(void)
{
	size_t i;

	for (i = 0; i < sizeof(g_ae_sensors) / sizeof(g_ae_sensors[0]); i++)
		if (sensor_bound(g_ae_sensors[i].name))
			return g_ae_sensors[i].name;
	return NULL;
}

Ar9311Ae *ar9311_ae_start(const Ar9311AeLimits *limits, uint32_t rate_hz)
{
	Ar9311Ae *ae;
	long lines, code;

	if (find_sensor() != 0) {
		fprintf(stderr, "WARN: [ar9311] AE disabled: no bound camera "
			"sensor (nt99235, sc231ai)\n");
		return NULL;
	}
	if (read_int(AE_SENSOR_EXPOSURE, &lines) != 0 ||
	    read_int(AE_SENSOR_GAIN, &code) != 0 ||
	    access(AE_STATS_PATH, R_OK) != 0) {
		fprintf(stderr, "WARN: [ar9311] AE disabled: sensor parameters or "
			"ISP statistics (%s) not available\n", AE_STATS_PATH);
		return NULL;
	}
	ae = calloc(1, sizeof(*ae));
	if (!ae)
		return NULL;
	pthread_mutex_init(&ae->lock, NULL);
	ae->limits = *limits;
	ae->channels = g_ae_sensor->channels;
	ae->period_us = 1000000u / (rate_hz < 1 ? 1 : rate_hz > 30 ? 30 : rate_hz);
	ae->point.lines = (uint32_t)lines;
	ae->point.gain_code = (unsigned)code;
	if (g_ae_gain_hook)
		g_ae_gain_hook(ar9311_ae_gain_q10((unsigned)code));
	ae->exposure = ar9311_ae_exposure_of(&ae->point);
	ae->tone_scalar = -1;
	if (g_ae_sensor->tone)
		load_tone(ae);
	if (pthread_create(&ae->thread, NULL, ae_thread, ae) != 0) {
		pthread_mutex_destroy(&ae->lock);
		free(ae->tone_ev);
		free(ae);
		return NULL;
	}
	printf("> [ar9311] AE running at %u Hz, target luma %u\n",
		1000000u / ae->period_us, sensor_limits(limits).target);
	return ae;
}

void ar9311_ae_set_limits(Ar9311Ae *ae, const Ar9311AeLimits *limits)
{
	if (!ae)
		return;
	pthread_mutex_lock(&ae->lock);
	ae->limits = *limits;
	pthread_mutex_unlock(&ae->lock);
}

void ar9311_ae_get_limits(Ar9311Ae *ae, Ar9311AeLimits *limits)
{
	pthread_mutex_lock(&ae->lock);
	*limits = ae->limits;
	pthread_mutex_unlock(&ae->lock);
}

char *ar9311_ae_status_json(Ar9311Ae *ae)
{
	char buf[512];
	int pos;

	Ar9311AeLimits lim;

	if (!ae)
		return NULL;
	pthread_mutex_lock(&ae->lock);
	lim = sensor_limits(&ae->limits);   /* what the AE actually uses */
	pos = snprintf(buf, sizeof(buf), "{\"ok\":true,\"data\":{"
		"\"engine\":\"ar9311\",\"statsLive\":%s,\"luma\":%.1f,"
		"\"target\":%u,\"saturatedZones\":%u,\"lines\":%u,"
		"\"exposureUs\":%u,\"gainCode\":%u,\"gain\":%u,"
		"\"decisions\":%lu,\"moves\":%lu,\"antiFlickerHz\":%u,"
		"\"toneScalar\":%.1f,\"limits\":{\"linesMin\":%u,\"linesMax\":%u,"
		"\"gainMin\":%u,\"gainMax\":%u}}}",
		ae->have_stats ? "true" : "false", (double)ae->meter.luma,
		lim.target, ae->meter.saturated, ae->point.lines,
		(unsigned)((uint64_t)ae->point.lines * AR9311_AE_LINE_NS / 1000u),
		ae->point.gain_code, ar9311_ae_gain_q10(ae->point.gain_code),
		ae->decisions, ae->moves, lim.flicker_hz,
		ae->tone_scalar < 0 ? -1.0 : ae->tone_scalar / 256.0, lim.lines_min,
		lim.lines_max, lim.gain_min_q10, lim.gain_max_q10);
	pthread_mutex_unlock(&ae->lock);
	if (pos < 0 || pos >= (int)sizeof(buf))
		return NULL;
	return strdup(buf);
}

int ar9311_ae_drives_cm(const Ar9311Ae *ae)
{
	return ae && ae->colour;
}

void ar9311_ae_stop(Ar9311Ae *ae)
{
	if (!ae)
		return;
	__atomic_store_n(&ae->stop, 1, __ATOMIC_RELEASE);
	pthread_join(ae->thread, NULL);
	__atomic_store_n(&g_ae_tone_now, -1, __ATOMIC_RELAXED);
	pthread_mutex_destroy(&ae->lock);
	free(ae->tone_ev);
	free(ae);
}
