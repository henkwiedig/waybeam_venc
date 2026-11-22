#ifndef AR9311_AWB_H
#define AR9311_AWB_H

#include <stdint.h>

/* Auto white balance for the AR9311 camera.  The ISP's zone statistics are
 * raw Bayer means taken before its white balance stage, and everything after
 * that stage is colour-neutral, so the gains that render a neutral surface
 * grey are simply green/red and green/blue of that surface.  The AWB finds
 * the neutral surfaces of a frame as the zones whose colour lies on the
 * sensor's illuminant locus (the raw colour of grey under warm to daylight
 * light), averages them, and moves the ISP's red and blue gains towards the
 * result.  Coloured objects sit off the locus and do not pull the balance.
 * The colour temperature of the gains it applies goes to the ISP driver,
 * whose colour matrix and saturation follow the light as stock's do.
 *
 * The pure part (estimate, step, colour temperature) has no I/O so the host
 * tests can drive it; ar9311_awb_start() wraps it in a thread. */

/* The illuminant locus: three calibration points, warm (2800 K), mid
 * (4000 K) and daylight (6500 K), joined by straight lines in (ln R/G,
 * ln B/G).  A position t on it runs 0 at warm, 1 at mid, 2 at daylight, and
 * past either end along the end segment. */
#define AR9311_AWB_POINTS 3
#define AR9311_AWB_T_MIN (-0.5)   /* ~2400 K */
#define AR9311_AWB_T_MAX 2.4      /* ~8700 K, shade */

/* A sensor's colour calibration: which zone channels are red and blue (the
 * other two are green), and the neutral R/G and B/G at each locus point,
 * measured as the gains that rendered a grey surface grey. */
typedef struct {
	const char *name;
	unsigned ch_red, ch_blue;
	double rg[AR9311_AWB_POINTS], bg[AR9311_AWB_POINTS];
} Ar9311AwbSensor;

/* The calibrated sensor named @p name, or NULL (no AWB for it). */
const Ar9311AwbSensor *ar9311_awb_sensor(const char *name);

/* Red and blue gains in Q8 (256 = 1x); green stays 1x. */
typedef struct {
	uint32_t red, blue;
} Ar9311AwbGains;

typedef struct {
	unsigned zones;   /* zones that counted as neutral */
	double rg, bg;    /* the light's neutral R/G and B/G */
	double t;         /* position on the locus: 0 warm, 1 mid, 2 daylight */
} Ar9311AwbEstimate;

/** Estimate the light from one RRO grid (the stats snapshot without its
 *  leading sequence word), counting neutral zones from locus position
 *  @p t_floor up (AR9311_AWB_T_MIN for all of it; see
 *  ar9311_awb_floor_for()).  A floor that leaves too few zones falls back to
 *  the whole locus.  0, or -1 when too few zones look neutral to trust (then
 *  the gains should hold). */
int ar9311_awb_estimate(const uint8_t *rro, const Ar9311AwbSensor *s,
	double t_floor, Ar9311AwbEstimate *out);

/** The lowest locus position a neutral zone may have at an exposure whose
 *  tone scalar (Q8, ar9311_ae_tone_now(); negative when unknown) is
 *  @p tone_q8: in daylight the light is daylight, and the warm end of the
 *  locus holds orange, brown and yellow surfaces, not greys. */
double ar9311_awb_floor_for(long tone_q8);

/** The gains that render the estimated light neutral. */
Ar9311AwbGains ar9311_awb_gains_for(const Ar9311AwbEstimate *e);

/** Move @p g part of the way towards @p target (log domain, with a small
 *  deadband).  Returns 1 if a gain changed. */
int ar9311_awb_step(Ar9311AwbGains *g, const Ar9311AwbGains *target);

/** The locus estimate for a colour temperature in kelvin (isp.awbMode
 *  ct_manual), and the inverse for status reporting. */
Ar9311AwbEstimate ar9311_awb_at_ct(const Ar9311AwbSensor *s, uint32_t kelvin);
uint32_t ar9311_awb_ct_of(const Ar9311AwbEstimate *e);

/** Colour temperature of the light gains @p g balance, on the locus: what
 *  the ISP's colour stages are told while those gains are applied. */
uint32_t ar9311_awb_ct_of_gains(const Ar9311AwbSensor *s,
	const Ar9311AwbGains *g);

/* ── Runtime ─────────────────────────────────────────────────────────── */

typedef struct Ar9311Awb Ar9311Awb;

/** Start the AWB thread at @p rate_hz decisions per second (1..30), in auto
 *  mode or, with @p manual_ct non-zero, fixed at that colour temperature.
 *  NULL (reason on stderr) when the bound sensor has no calibration or the
 *  ISP has no white_balance interface; the gains then stay as loaded. */
Ar9311Awb *ar9311_awb_start(uint32_t rate_hz, uint32_t manual_ct);

/** Switch between auto (@p manual_ct 0) and a fixed colour temperature. */
void ar9311_awb_set_mode(Ar9311Awb *awb, uint32_t manual_ct);

/** Status as a malloc'd JSON API reply, or NULL. */
char *ar9311_awb_status_json(Ar9311Awb *awb);

/** Stop and join the thread, free @p awb.  NULL is fine.  The gains stay. */
void ar9311_awb_stop(Ar9311Awb *awb);

#endif /* AR9311_AWB_H */
