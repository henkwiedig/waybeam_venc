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
/* The ISP driver's colour stages (CCM bank blend, cm column) follow this;
 * absent from drivers without them. */
#define AWB_CT_PARAM "/sys/module/ar_isp/parameters/colour_temp"
#define AWB_CT_RELATCH "/sys/kernel/debug/ar-isp/colour"
/* Re-sent when the applied balance has moved this far: about 85 K at
 * 6500 K, a twentieth of a CCM band gap. */
#define AWB_CT_STEP_MIRED 2.0

/* Calibrations.  SC231AI: zone channels 0 and 3 are green, 1 red, 2 blue.
 * Gains (R/B, Q8) that render grey grey, at each locus point:
 *   warm 269/834: a warm lamp (310/962 as measured; the monitor-and-lamp
 *     calibration left grey 13 % short of green, see mid);
 *   mid 400/644: stock's AWB on the same P1 at dusk wrote 399/646, where
 *     its colour matrix was tuning bank 1 (band 3700-4300 K);
 *   daylight 537/474: stock in sunshine, matrix bank 3 (6200-6700 K).  The
 *     sunlit white walls of that scene measured 504-552/465-495 in our own
 *     statistics; the old straight line through mid reached their R gain
 *     with 10 % too much blue, as the locus bends.
 * The NT99235 has no calibration yet, so the AWB leaves it alone. */
static const Ar9311AwbSensor g_awb_sensors[] = {
	{ "sc231ai", 1, 2,
		{ 256.0 / 269, 256.0 / 400, 256.0 / 537 },
		{ 256.0 / 834, 256.0 / 644, 256.0 / 474 } },
};

/* Colour temperatures of the locus points: their positions on the tuning
 * file's CCM bands (above).  Linear in mired along each segment. */
static const double g_awb_kelvin[AR9311_AWB_POINTS] = { 2800.0, 4000.0, 6500.0 };

/* Zone selection.  Statistics are 8-bit means: a zone counts only when no
 * channel is near clipping and green is clear of the noise floor.  A neutral
 * zone lies within AWB_WIDTH (natural log of R/G and B/G) of the locus,
 * between AR9311_AWB_T_MIN and AR9311_AWB_T_MAX; everything else is a
 * coloured object.  Too few neutral zones and the estimate is not trusted. */
#define AWB_CLIP 235.0
#define AWB_DARK 8.0
#define AWB_WIDTH 0.18
#define AWB_MIN_ZONES 24u

/* Daylight.  Orange, brown and yellow surfaces (autumn leaves, a hedge,
 * brick) lie along the warm end of the locus and count as greys under a
 * warm light: on a sunny garden 35 of 45 "neutral" zones were such objects
 * and pulled the balance to 458/591 where stock and the white walls said
 * 537/474.  At an exposure only daylight gives, neutral zones start at
 * AWB_DAY_FLOOR (~4300 K); the floor drops to the whole locus between tone
 * scalars AWB_DAY_SCALAR and AWB_DIM_SCALAR (ar9311_ae.h: 180 is 272 lines
 * at 1x, 260 is 1114 lines at 2.8x; the sun sits at 70-115, a lamp-lit room
 * near 390). */
#define AWB_DAY_FLOOR 1.2
#define AWB_DAY_SCALAR 180.0
#define AWB_DIM_SCALAR 260.0

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

/* The locus in (ln R/G, ln B/G). */
typedef struct {
	double x[AR9311_AWB_POINTS], y[AR9311_AWB_POINTS];
} Locus;

static void locus_of(const Ar9311AwbSensor *s, Locus *l)
{
	unsigned i;

	for (i = 0; i < AR9311_AWB_POINTS; i++) {
		l->x[i] = log(s->rg[i]);
		l->y[i] = log(s->bg[i]);
	}
}

/* Position of (x, y) on the locus, and its squared distance from it.  The
 * first segment runs on below warm, the last on past daylight. */
static double locus_t(const Locus *l, double x, double y, double *dist2)
{
	double best_t = 0.0, best_d2 = HUGE_VAL;
	unsigned k;

	for (k = 0; k + 1 < AR9311_AWB_POINTS; k++) {
		double vx = l->x[k + 1] - l->x[k], vy = l->y[k + 1] - l->y[k];
		double u = ((x - l->x[k]) * vx + (y - l->y[k]) * vy) /
			(vx * vx + vy * vy);
		double dx, dy, d2;

		if (k > 0 && u < 0.0)
			u = 0.0;
		if (k + 2 < AR9311_AWB_POINTS && u > 1.0)
			u = 1.0;
		dx = x - (l->x[k] + u * vx);
		dy = y - (l->y[k] + u * vy);
		d2 = dx * dx + dy * dy;
		if (d2 < best_d2) {
			best_d2 = d2;
			best_t = k + u;
		}
	}
	*dist2 = best_d2;
	return best_t;
}

/* The segment a locus position lies on (the first below mid, the second
 * from it), and how far along it. */
static unsigned segment_of(double t, double *u)
{
	unsigned k = t < 1.0 ? 0u : 1u;

	*u = t - k;
	return k;
}

/* The light at locus position t. */
static Ar9311AwbEstimate on_locus(const Ar9311AwbSensor *s, double t)
{
	Ar9311AwbEstimate e;
	Locus l;
	double u;
	unsigned k = segment_of(t, &u);

	locus_of(s, &l);
	e.zones = 0;
	e.t = t;
	e.rg = exp(l.x[k] + u * (l.x[k + 1] - l.x[k]));
	e.bg = exp(l.y[k] + u * (l.y[k + 1] - l.y[k]));
	return e;
}

/* Mean locus position of the neutral zones from @p floor up; the zone count,
 * or 0 for none. */
static unsigned neutral_mean(const uint8_t *rro, const Ar9311AwbSensor *s,
	const Locus *l, double floor, double *mean)
{
	double sum_t = 0.0;
	unsigned zones = 0, blk, entry, ch;

	for (blk = 0; blk < AR9311_AE_COLS; blk++) {
		const uint8_t *col = rro + blk * AR9311_AE_COL_STRIDE;
		uint32_t count = get_le32(col);

		for (entry = 0; entry < AR9311_AE_ROWS; entry++) {
			const uint8_t *zone = col + AR9311_AE_BLOCK +
				entry * AR9311_AE_CHANNELS * 4;
			double m[AR9311_AE_CHANNELS], g = 0.0, peak = 0.0;
			double t, d2;

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
			t = locus_t(l, log(m[s->ch_red] / g),
				log(m[s->ch_blue] / g), &d2);
			if (t < floor || t > AR9311_AWB_T_MAX ||
			    d2 > AWB_WIDTH * AWB_WIDTH)
				continue;
			/* Every neutral zone counts once: weighted by brightness,
			 * a lit lamp shade or window frame outvoted the grey
			 * the light falls on. */
			sum_t += t;
			zones++;
		}
	}
	if (zones)
		*mean = sum_t / zones;
	return zones;
}

int ar9311_awb_estimate(const uint8_t *rro, const Ar9311AwbSensor *s,
	double t_floor, Ar9311AwbEstimate *out)
{
	Locus l;
	double mean = 0.0;
	unsigned blk, zones;

	for (blk = 0; blk < AR9311_AE_COLS; blk++)
		if (get_le32(rro + blk * AR9311_AE_COL_STRIDE) == 0)
			return -1;
	if (!(t_floor > AR9311_AWB_T_MIN))   /* also catches NaN */
		t_floor = AR9311_AWB_T_MIN;
	locus_of(s, &l);
	zones = neutral_mean(rro, s, &l, t_floor, &mean);
	/* A sunlit scene with nothing neutral in daylight colours (a sunset)
	 * still balances on what there is. */
	if (zones < AWB_MIN_ZONES && t_floor > AR9311_AWB_T_MIN)
		zones = neutral_mean(rro, s, &l, AR9311_AWB_T_MIN, &mean);
	if (zones < AWB_MIN_ZONES)
		return -1;
	*out = on_locus(s, mean);
	out->zones = zones;
	return 0;
}

double ar9311_awb_floor_for(long tone_q8)
{
	double scalar = tone_q8 / 256.0, f;

	if (tone_q8 < 0 || scalar >= AWB_DIM_SCALAR)
		return AR9311_AWB_T_MIN;
	if (scalar <= AWB_DAY_SCALAR)
		return AWB_DAY_FLOOR;
	f = (AWB_DIM_SCALAR - scalar) / (AWB_DIM_SCALAR - AWB_DAY_SCALAR);
	return AR9311_AWB_T_MIN + f * (AWB_DAY_FLOOR - AR9311_AWB_T_MIN);
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

/* Colour temperature runs along each locus segment linearly in mired. */
static double mired_at(double t)
{
	double u;
	unsigned k = segment_of(t, &u);
	double m0 = 1e6 / g_awb_kelvin[k], m1 = 1e6 / g_awb_kelvin[k + 1];

	return m0 + u * (m1 - m0);
}

Ar9311AwbEstimate ar9311_awb_at_ct(const Ar9311AwbSensor *s, uint32_t kelvin)
{
	double mired = 1e6 / (kelvin ? kelvin : 1), t = AR9311_AWB_T_MAX;
	unsigned k;

	/* Mired falls along the locus: find the segment that holds it, the end
	 * segments running on past their points. */
	for (k = 0; k + 1 < AR9311_AWB_POINTS; k++) {
		double m0 = 1e6 / g_awb_kelvin[k], m1 = 1e6 / g_awb_kelvin[k + 1];

		if (mired >= m1 || k + 2 == AR9311_AWB_POINTS) {
			t = k + (m0 - mired) / (m0 - m1);
			break;
		}
	}
	if (t < AR9311_AWB_T_MIN)
		t = AR9311_AWB_T_MIN;
	if (t > AR9311_AWB_T_MAX)
		t = AR9311_AWB_T_MAX;
	return on_locus(s, t);
}

uint32_t ar9311_awb_ct_of(const Ar9311AwbEstimate *e)
{
	double mired = mired_at(e->t);

	return mired > 1.0 ? (uint32_t)(1e6 / mired + 0.5) : 0;
}

uint32_t ar9311_awb_ct_of_gains(const Ar9311AwbSensor *s,
	const Ar9311AwbGains *g)
{
	Ar9311AwbEstimate e;
	Locus l;
	double d2;

	if (!g->red || !g->blue)
		return 0;
	locus_of(s, &l);
	e.t = locus_t(&l, log(256.0 / g->red), log(256.0 / g->blue), &d2);
	if (e.t < AR9311_AWB_T_MIN)
		e.t = AR9311_AWB_T_MIN;
	if (e.t > AR9311_AWB_T_MAX)
		e.t = AR9311_AWB_T_MAX;
	return ar9311_awb_ct_of(&e);
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
	int colour;          /* the driver takes colour_temp */
	uint32_t ccm_ct;     /* last colour temperature sent to it, 0 none */
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

/* Tell the ISP's colour stages the light the applied gains balance. */
static void send_ct(Ar9311Awb *awb, const Ar9311AwbGains *g)
{
	uint32_t ct = ar9311_awb_ct_of_gains(awb->sensor, g), last;
	char buf[16];
	int fd, len, ok;

	pthread_mutex_lock(&awb->lock);
	last = awb->ccm_ct;
	pthread_mutex_unlock(&awb->lock);
	if (!ct || (last && fabs(1e6 / ct - 1e6 / last) < AWB_CT_STEP_MIRED))
		return;
	fd = open(AWB_CT_PARAM, O_WRONLY | O_CLOEXEC);
	if (fd < 0)
		return;
	len = snprintf(buf, sizeof(buf), "%u\n", ct);
	ok = write(fd, buf, (size_t)len) == len;
	close(fd);
	if (ok) {
		fd = open(AWB_CT_RELATCH, O_WRONLY | O_CLOEXEC);
		ok = fd >= 0 && write(fd, "1\n", 2) == 2;
		if (fd >= 0)
			close(fd);
	}
	if (!ok) {
		fprintf(stderr, "WARN: [ar9311] AWB: colour stages not updated: "
			"%s\n", strerror(errno));
		return;
	}
	pthread_mutex_lock(&awb->lock);
	awb->ccm_ct = ct;
	pthread_mutex_unlock(&awb->lock);
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
		have = ar9311_awb_estimate(stats + 4, awb->sensor,
			ar9311_awb_floor_for(ar9311_ae_tone_now()), &est) == 0;
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
	if (awb->colour)
		send_ct(awb, &gains);

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
	awb->colour = access(AWB_CT_PARAM, W_OK) == 0 &&
		access(AWB_CT_RELATCH, W_OK) == 0;
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
	if (!awb->colour)
		printf("> [ar9311] AWB: the ISP driver has no colour_temp; its "
			"colour matrix stays at the loaded bank\n");
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
		"\"ct\":%u},\"ccmCt\":%u,\"decisions\":%lu,\"moves\":%lu}}",
		awb->sensor->name, awb->manual_ct ? "ct_manual" : "auto",
		awb->gains.red, awb->gains.blue, awb->have_est ? "true" : "false",
		awb->est.zones, awb->est.rg, awb->est.bg,
		awb->manual_ct ? awb->manual_ct : ar9311_awb_ct_of(&awb->est),
		awb->ccm_ct, awb->decisions, awb->moves);
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
