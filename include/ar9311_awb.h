#ifndef AR9311_AWB_H
#define AR9311_AWB_H

#include <stdint.h>

/* Auto white balance for the AR9311 camera.  The ISP's zone statistics are
 * raw Bayer means taken before its white balance stage, and everything after
 * that stage is colour-neutral, so the gains that render a neutral surface
 * grey are simply green/red and green/blue of that surface.  The AWB finds
 * the neutral surfaces of a frame as the zones whose colour lies on the
 * sensor's illuminant line (the raw colour of grey under warm to daylight
 * light), averages them, and moves the ISP's red and blue gains towards the
 * result.  Coloured objects sit off the line and do not pull the balance.
 *
 * The pure part (estimate, step, colour temperature) has no I/O so the host
 * tests can drive it; ar9311_awb_start() wraps it in a thread. */

/* A sensor's colour calibration: which zone channels are red and blue (the
 * other two are green), and the neutral R/G and B/G at a warm and a daylight
 * point, measured as the gains that rendered a grey surface grey. */
typedef struct {
	const char *name;
	unsigned ch_red, ch_blue;
	double warm_rg, warm_bg;   /* ~2800 K */
	double day_rg, day_bg;     /* ~6500 K */
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
	double t;         /* position on the illuminant line: 0 warm, 1 daylight */
} Ar9311AwbEstimate;

/** Estimate the light from one RRO grid (the stats snapshot without its
 *  leading sequence word).  0, or -1 when too few zones look neutral to
 *  trust (then the gains should hold). */
int ar9311_awb_estimate(const uint8_t *rro, const Ar9311AwbSensor *s,
	Ar9311AwbEstimate *out);

/** The gains that render the estimated light neutral. */
Ar9311AwbGains ar9311_awb_gains_for(const Ar9311AwbEstimate *e);

/** Move @p g part of the way towards @p target (log domain, with a small
 *  deadband).  Returns 1 if a gain changed. */
int ar9311_awb_step(Ar9311AwbGains *g, const Ar9311AwbGains *target);

/** The illuminant-line estimate for a colour temperature in kelvin
 *  (isp.awbMode ct_manual), and the inverse for status reporting. */
Ar9311AwbEstimate ar9311_awb_at_ct(const Ar9311AwbSensor *s, uint32_t kelvin);
uint32_t ar9311_awb_ct_of(const Ar9311AwbEstimate *e);

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
