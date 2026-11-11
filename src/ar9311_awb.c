#include "ar9311_awb.h"

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

#define AWB_WB_PATH "/sys/kernel/debug/ar-isp/white_balance"

/* Calibrations.  SC231AI: zone channels 0 and 3 are green, 1 red, 2 blue;
 * the neutral points are the gains that rendered grey grey on a P1 under a
 * warm lamp (310/962) and on a D65 monitor (461/743).  The NT99235 has no
 * calibration yet, so the AWB leaves it alone. */
static const Ar9311AwbSensor g_awb_sensors[] = {
	{ "sc231ai", 1, 2, 256.0 / 310, 256.0 / 962, 256.0 / 461, 256.0 / 743 },
};

/* Colour temperatures of the two calibration points, for awbCt. */
#define AWB_WARM_K 2800.0
#define AWB_DAY_K 6500.0

/* Zone selection.  Statistics are 8-bit means: a zone counts only when no
 * channel is near clipping and green is clear of the noise floor.  A neutral
 * zone lies within AWB_WIDTH (natural log of R/G and B/G) of the illuminant
 * line, between somewhat warmer than the warm point and well past daylight
 * (shade); everything else is a coloured object.  Too few neutral zones and
 * the estimate is not trusted. */
#define AWB_CLIP 235.0
#define AWB_DARK 8.0
#define AWB_WIDTH 0.18
#define AWB_T_MIN (-0.5)
#define AWB_T_MAX 1.8
#define AWB_MIN_ZONES 24u

/* Control: each decision corrects this share of the log error; inside the
 * deadband the gains hold.  At 15 Hz the balance settles in about a second
 * without visible steps. */
#define AWB_STEP 0.15
#define AWB_DEADBAND 0.01
#define AWB_GAIN_MIN 64u
#define AWB_GAIN_MAX 4095u

/* ── Pure part ───────────────────────────────────────────────────────── */

const Ar9311AwbSensor *ar9311_awb_sensor(const char *name)
{
	size_t i;

	if (!name)
		return NULL;
	for (i = 0; i < sizeof(g_awb_sensors) / sizeof(g_awb_sensors[0]); i++)
		if (strcmp(g_awb_sensors[i].name, name) == 0)
			return &g_awb_sensors[i];
	return NULL;
}

static uint32_t get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
		(uint32_t)p[3] << 24;
}

/* The illuminant line in (ln R/G, ln B/G): the warm point plus t times the
 * way to the daylight point. */
static Ar9311AwbEstimate on_line(const Ar9311AwbSensor *s, double t)
{
	Ar9311AwbEstimate e;

	e.zones = 0;
	e.t = t;
	e.rg = exp(log(s->warm_rg) + t * (log(s->day_rg) - log(s->warm_rg)));
	e.bg = exp(log(s->warm_bg) + t * (log(s->day_bg) - log(s->warm_bg)));
	return e;
}

int ar9311_awb_estimate(const uint8_t *rro, const Ar9311AwbSensor *s,
	Ar9311AwbEstimate *out)
{
	double wx = log(s->warm_rg), wy = log(s->warm_bg);
	double vx = log(s->day_rg) - wx, vy = log(s->day_bg) - wy;
	double len2 = vx * vx + vy * vy;
	double sum_t = 0.0, sum_w = 0.0;
	unsigned zones = 0, blk, entry, ch;

	for (blk = 0; blk < AR9311_AE_COLS; blk++) {
		const uint8_t *col = rro + blk * AR9311_AE_COL_STRIDE;
		uint32_t count = get_le32(col);

		if (count == 0)
			return -1;
		for (entry = 0; entry < AR9311_AE_ROWS; entry++) {
			const uint8_t *zone = col + AR9311_AE_BLOCK +
				entry * AR9311_AE_CHANNELS * 4;
			double m[AR9311_AE_CHANNELS], g = 0.0, peak = 0.0;
			double px, py, t, dx, dy;

			for (ch = 0; ch < AR9311_AE_CHANNELS; ch++) {
				m[ch] = (double)get_le32(zone + ch * 4) / count;
				if (m[ch] > peak)
					peak = m[ch];
				if (ch != s->ch_red && ch != s->ch_blue)
					g += m[ch] / 2.0;
			}
			if (peak >= AWB_CLIP || g < AWB_DARK ||
			    m[s->ch_red] <= 0.0 || m[s->ch_blue] <= 0.0)
				continue;
			px = log(m[s->ch_red] / g) - wx;
			py = log(m[s->ch_blue] / g) - wy;
			t = (px * vx + py * vy) / len2;
			dx = px - t * vx;
			dy = py - t * vy;
			if (t < AWB_T_MIN || t > AWB_T_MAX ||
			    dx * dx + dy * dy > AWB_WIDTH * AWB_WIDTH)
				continue;
			/* Brighter neutral zones are the better white reference. */
			sum_t += g * t;
			sum_w += g;
			zones++;
		}
	}
	if (zones < AWB_MIN_ZONES)
		return -1;
	*out = on_line(s, sum_t / sum_w);
	out->zones = zones;
	return 0;
}

static uint32_t clamp_gain(double q8)
{
	if (!(q8 >= AWB_GAIN_MIN))   /* also catches NaN */
		return AWB_GAIN_MIN;
	if (q8 > AWB_GAIN_MAX)
		return AWB_GAIN_MAX;
	return (uint32_t)(q8 + 0.5);
}

Ar9311AwbGains ar9311_awb_gains_for(const Ar9311AwbEstimate *e)
{
	Ar9311AwbGains g = { clamp_gain(256.0 / e->rg), clamp_gain(256.0 / e->bg) };

	return g;
}

static int step_one(uint32_t *gain, uint32_t target)
{
	double err = log((double)target / *gain);
	uint32_t next;

	if (fabs(err) < AWB_DEADBAND)
		return 0;
	next = clamp_gain(*gain * exp(AWB_STEP * err));
	if (next == *gain)   /* always make progress outside the deadband */
		next = target > *gain ? *gain + 1 : *gain - 1;
	*gain = next;
	return 1;
}

int ar9311_awb_step(Ar9311AwbGains *g, const Ar9311AwbGains *target)
{
	int moved = step_one(&g->red, target->red);

	moved |= step_one(&g->blue, target->blue);
	return moved;
}

/* Colour temperature runs along the line linearly in mired. */
Ar9311AwbEstimate ar9311_awb_at_ct(const Ar9311AwbSensor *s, uint32_t kelvin)
{
	double warm = 1e6 / AWB_WARM_K, day = 1e6 / AWB_DAY_K;
	double t = (warm - 1e6 / (kelvin ? kelvin : 1)) / (warm - day);

	if (t < AWB_T_MIN)
		t = AWB_T_MIN;
	if (t > AWB_T_MAX)
		t = AWB_T_MAX;
	return on_line(s, t);
}

uint32_t ar9311_awb_ct_of(const Ar9311AwbEstimate *e)
{
	double warm = 1e6 / AWB_WARM_K, day = 1e6 / AWB_DAY_K;
	double mired = warm - e->t * (warm - day);

	return mired > 1.0 ? (uint32_t)(1e6 / mired + 0.5) : 0;
}

/* ── Runtime ─────────────────────────────────────────────────────────── */

struct Ar9311Awb {
	pthread_t thread;
	pthread_mutex_t lock;
	int stop;
	uint32_t period_us;
	const Ar9311AwbSensor *sensor;
	/* Under lock. */
	uint32_t manual_ct;   /* 0 = auto */
	Ar9311AwbGains gains;
	Ar9311AwbEstimate est;
	int have_est;
	unsigned long decisions, moves;
};

static int read_gains(Ar9311AwbGains *g)
{
	unsigned r, gr, b;
	FILE *f = fopen(AWB_WB_PATH, "r");
	int ok;

	if (!f)
		return -1;
	ok = fscanf(f, "%u %u %u", &r, &gr, &b) == 3;
	fclose(f);
	if (!ok)
		return -1;
	g->red = clamp_gain(r);
	g->blue = clamp_gain(b);
	return 0;
}

static int write_gains(const Ar9311AwbGains *g)
{
	char buf[48];
	int fd = open(AWB_WB_PATH, O_WRONLY | O_CLOEXEC);
	int len, ok;

	if (fd < 0)
		return -1;
	len = snprintf(buf, sizeof(buf), "%u 256 %u\n", g->red, g->blue);
	ok = write(fd, buf, (size_t)len) == len;
	close(fd);
	return ok ? 0 : -1;
}

static void sleep_us(uint32_t us)
{
	struct timespec ts = { us / 1000000u, (long)(us % 1000000u) * 1000 };

	nanosleep(&ts, NULL);
}

static void decide(Ar9311Awb *awb, const uint8_t *stats)
{
	Ar9311AwbEstimate est;
	Ar9311AwbGains gains, target;
	uint32_t manual_ct;
	int have, moved;

	pthread_mutex_lock(&awb->lock);
	manual_ct = awb->manual_ct;
	gains = awb->gains;
	pthread_mutex_unlock(&awb->lock);

	/* A fixed setting applies without statistics; auto waits for a frame. */
	if (!manual_ct && !stats)
		return;
	if (manual_ct) {
		est = ar9311_awb_at_ct(awb->sensor, manual_ct);
		have = 1;
	} else {
		have = ar9311_awb_estimate(stats + 4, awb->sensor, &est) == 0;
	}
	if (!have) {
		pthread_mutex_lock(&awb->lock);
		awb->have_est = 0;
		awb->decisions++;
		pthread_mutex_unlock(&awb->lock);
		return;
	}
	target = ar9311_awb_gains_for(&est);
	if (manual_ct) {   /* a chosen setting applies at once */
		moved = gains.red != target.red || gains.blue != target.blue;
		gains = target;
	} else {
		moved = ar9311_awb_step(&gains, &target);
	}
	if (moved && write_gains(&gains) != 0)
		fprintf(stderr, "WARN: [ar9311] AWB: writing %s failed: %s\n",
			AWB_WB_PATH, strerror(errno));

	pthread_mutex_lock(&awb->lock);
	awb->gains = gains;
	awb->est = est;
	awb->have_est = 1;
	awb->decisions++;
	awb->moves += (unsigned long)moved;
	pthread_mutex_unlock(&awb->lock);
}

static void *awb_thread(void *opaque)
{
	Ar9311Awb *awb = opaque;
	uint8_t *buf = malloc(AR9311_AE_STATS_SIZE);
	uint32_t seq, last_seq = 0;
	int have_seq = 0;

	if (!buf)
		return NULL;
	while (!__atomic_load_n(&awb->stop, __ATOMIC_ACQUIRE)) {
		int fresh;

		sleep_us(awb->period_us);
		/* Deciding twice on one frame would double the step. */
		fresh = ar9311_stats_read(buf, &seq) == 0 &&
			!(have_seq && seq == last_seq);
		if (fresh) {
			last_seq = seq;
			have_seq = 1;
		}
		decide(awb, fresh ? buf : NULL);
	}
	free(buf);
	return NULL;
}

Ar9311Awb *ar9311_awb_start(uint32_t rate_hz, uint32_t manual_ct)
{
	const char *name = ar9311_sensor_name();
	const Ar9311AwbSensor *s = ar9311_awb_sensor(name);
	Ar9311Awb *awb;
	Ar9311AwbGains gains;

	if (!s) {
		fprintf(stderr, "WARN: [ar9311] AWB disabled: no colour "
			"calibration for the %s; white balance stays as loaded\n",
			name ? name : "camera");
		return NULL;
	}
	if (read_gains(&gains) != 0) {
		fprintf(stderr, "WARN: [ar9311] AWB disabled: %s not available\n",
			AWB_WB_PATH);
		return NULL;
	}
	awb = calloc(1, sizeof(*awb));
	if (!awb)
		return NULL;
	pthread_mutex_init(&awb->lock, NULL);
	awb->sensor = s;
	awb->manual_ct = manual_ct;
	awb->gains = gains;
	awb->period_us = 1000000u / (rate_hz < 1 ? 1 : rate_hz > 30 ? 30 : rate_hz);
	if (pthread_create(&awb->thread, NULL, awb_thread, awb) != 0) {
		pthread_mutex_destroy(&awb->lock);
		free(awb);
		return NULL;
	}
	if (manual_ct)
		printf("> [ar9311] AWB fixed at %u K for the %s\n", manual_ct, s->name);
	else
		printf("> [ar9311] AWB running at %u Hz for the %s\n",
			1000000u / awb->period_us, s->name);
	return awb;
}

void ar9311_awb_set_mode(Ar9311Awb *awb, uint32_t manual_ct)
{
	if (!awb)
		return;
	pthread_mutex_lock(&awb->lock);
	awb->manual_ct = manual_ct;
	pthread_mutex_unlock(&awb->lock);
}

char *ar9311_awb_status_json(Ar9311Awb *awb)
{
	char buf[512];
	int pos;

	if (!awb)
		return NULL;
	pthread_mutex_lock(&awb->lock);
	pos = snprintf(buf, sizeof(buf), "{\"ok\":true,\"data\":{"
		"\"engine\":\"ar9311\",\"sensor\":\"%s\",\"mode\":\"%s\","
		"\"gains\":{\"red\":%u,\"green\":256,\"blue\":%u},"
		"\"estimate\":{\"live\":%s,\"zones\":%u,\"rg\":%.3f,\"bg\":%.3f,"
		"\"ct\":%u},\"decisions\":%lu,\"moves\":%lu}}",
		awb->sensor->name, awb->manual_ct ? "ct_manual" : "auto",
		awb->gains.red, awb->gains.blue, awb->have_est ? "true" : "false",
		awb->est.zones, awb->est.rg, awb->est.bg,
		awb->manual_ct ? awb->manual_ct : ar9311_awb_ct_of(&awb->est),
		awb->decisions, awb->moves);
	pthread_mutex_unlock(&awb->lock);
	if (pos < 0 || pos >= (int)sizeof(buf))
		return NULL;
	return strdup(buf);
}

void ar9311_awb_stop(Ar9311Awb *awb)
{
	if (!awb)
		return;
	__atomic_store_n(&awb->stop, 1, __ATOMIC_RELEASE);
	pthread_join(awb->thread, NULL);
	pthread_mutex_destroy(&awb->lock);
	free(awb);
}
