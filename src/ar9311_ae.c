#include "ar9311_ae.h"

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
#define AE_SENSOR_EXPOSURE "/sys/module/nt99235/parameters/exposure"
#define AE_SENSOR_GAIN "/sys/module/nt99235/parameters/gain"

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

int ar9311_ae_meter(const uint8_t *rro, Ar9311AeMeter *out)
{
	double sum = 0.0;
	unsigned weights = 0;
	unsigned col, row, ch;

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
				zone_sum += get_le32(zone + ch * 4);
			mean = (float)(zone_sum / AR9311_AE_CHANNELS / count);
			if (mean >= AE_CLIP_LUMA)
				out->saturated++;
			sum += (double)mean * w;
			weights += (unsigned)w;
		}
	}
	out->luma = (float)(sum / weights);
	return 0;
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
	Ar9311AeLimits limits;
	/* Status, under lock. */
	Ar9311AeMeter meter;
	Ar9311AePoint point;
	double exposure;
	unsigned long decisions, moves;
	int have_stats;
};

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
static int read_stats(uint8_t *buf, uint32_t *seq)
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
	return write_int(AE_LADDER_ARM, 1);
}

static void sleep_us(uint32_t us)
{
	struct timespec ts = { us / 1000000u, (long)(us % 1000000u) * 1000 };

	nanosleep(&ts, NULL);
}

static void decide(Ar9311Ae *ae, const uint8_t *stats)
{
	Ar9311AeMeter meter;
	Ar9311AeLimits lim;
	Ar9311AePoint next, prev;
	double exposure;
	int moved;

	if (ar9311_ae_meter(stats + 4, &meter) != 0)
		return;
	pthread_mutex_lock(&ae->lock);
	lim = ae->limits;
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
		if (read_stats(buf, &seq) != 0 || (have_seq && seq == last_seq))
			continue;
		last_seq = seq;
		have_seq = 1;
		decide(ae, buf);
	}
	free(buf);
	return NULL;
}

Ar9311Ae *ar9311_ae_start(const Ar9311AeLimits *limits, uint32_t rate_hz)
{
	Ar9311Ae *ae;
	long lines, code;

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
	ae->period_us = 1000000u / (rate_hz < 1 ? 1 : rate_hz > 30 ? 30 : rate_hz);
	ae->point.lines = (uint32_t)lines;
	ae->point.gain_code = (unsigned)code;
	ae->exposure = ar9311_ae_exposure_of(&ae->point);
	if (pthread_create(&ae->thread, NULL, ae_thread, ae) != 0) {
		pthread_mutex_destroy(&ae->lock);
		free(ae);
		return NULL;
	}
	printf("> [ar9311] AE running at %u Hz, target luma %u\n",
		1000000u / ae->period_us, limits->target);
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

	if (!ae)
		return NULL;
	pthread_mutex_lock(&ae->lock);
	pos = snprintf(buf, sizeof(buf), "{\"ok\":true,\"data\":{"
		"\"engine\":\"ar9311\",\"statsLive\":%s,\"luma\":%.1f,"
		"\"target\":%u,\"saturatedZones\":%u,\"lines\":%u,"
		"\"exposureUs\":%u,\"gainCode\":%u,\"gain\":%u,"
		"\"decisions\":%lu,\"moves\":%lu,\"antiFlickerHz\":%u,"
		"\"limits\":{\"linesMin\":%u,\"linesMax\":%u,"
		"\"gainMin\":%u,\"gainMax\":%u}}}",
		ae->have_stats ? "true" : "false", (double)ae->meter.luma,
		ae->limits.target, ae->meter.saturated, ae->point.lines,
		(unsigned)((uint64_t)ae->point.lines * AR9311_AE_LINE_NS / 1000u),
		ae->point.gain_code, ar9311_ae_gain_q10(ae->point.gain_code),
		ae->decisions, ae->moves, ae->limits.flicker_hz, ae->limits.lines_min,
		ae->limits.lines_max, ae->limits.gain_min_q10,
		ae->limits.gain_max_q10);
	pthread_mutex_unlock(&ae->lock);
	if (pos < 0 || pos >= (int)sizeof(buf))
		return NULL;
	return strdup(buf);
}

void ar9311_ae_stop(Ar9311Ae *ae)
{
	if (!ae)
		return;
	__atomic_store_n(&ae->stop, 1, __ATOMIC_RELEASE);
	pthread_join(ae->thread, NULL);
	pthread_mutex_destroy(&ae->lock);
	free(ae);
}
