#ifndef AR9311_AE_H
#define AR9311_AE_H

#include <stdint.h>

/* Auto-exposure for the AR9311 camera (NT99235 or SC231AI + Artosyn ISP on
 * the MissingLynk drivers).  The ISP meters the frame into a zone grid; a thread
 * here turns that into sensor integration time and analogue gain, written
 * through the sensor driver's module parameters, and keeps the ISP's
 * gain-keyed denoise stages in step with the applied gain.
 *
 * The pure part (metering, control step, exposure/gain split) has no I/O so
 * the host tests can drive it; ar9311_ae_start() wraps it in the thread. */

/* Statistics snapshot (/sys/kernel/debug/ar-isp/stats_raw): flip sequence
 * u32, the RRO zone grid, the raw histogram, the sequence again.  The grid
 * is 36 columns x 16 rows; each column is a block of per-zone pixel counts
 * followed by a block of per-zone sums, 4 channels of u32 each, 8-bit
 * samples (ml-kernel ar-isp-stats.h). */
#define AR9311_AE_COLS 36
#define AR9311_AE_ROWS 16
#define AR9311_AE_CHANNELS 4
#define AR9311_AE_COL_STRIDE 0x200
#define AR9311_AE_BLOCK 0x100
#define AR9311_AE_RRO_SIZE (AR9311_AE_COLS * AR9311_AE_COL_STRIDE)
#define AR9311_AE_HIST_SIZE 0x1000
#define AR9311_AE_STATS_SIZE (4 + AR9311_AE_RRO_SIZE + AR9311_AE_HIST_SIZE + 4)

/* Gain in Q10, 1024 = 1x: the scale of isp.gainMax/gainMin. */
#define AR9311_AE_GAIN_ONE 1024u
/* Sensor gain code range, the nt99235 driver's scale (gain = 2^(code>>4) *
 * (16 + (code&15)) / 16, 1x at 0x00): up to 640x at 0x94, the SC231AI's top
 * (analogue to 116.5x, digital above).  Each sensor has its own ceiling in
 * ar9311_ae.c; the NT99235 stops at 64x (0x60). */
#define AR9311_AE_GAIN_CODE_MAX 0x94
/* 1080p60 frame: 1125 lines, integration up to 1123 (NT99235 driver clamp;
 * the SC231AI's is lower, see ar9311_ae.c). */
#define AR9311_AE_LINES_MAX 1123u
#define AR9311_AE_LINE_NS 14815u   /* (1 s / 60) / 1125 lines */
/* Default metered-luma target, 0..255 in the linear sensor domain. */
#define AR9311_AE_TARGET 56u

typedef struct {
	float luma;          /* centre-weighted mean zone luma, 0..255 */
	unsigned saturated;  /* zones whose mean is at or near clipping */
} Ar9311AeMeter;

typedef struct {
	uint32_t lines_min, lines_max;
	uint32_t gain_min_q10, gain_max_q10;
	uint32_t target;
	uint32_t flicker_hz;   /* mains anti-banding: 0 off, 50 or 60 */
} Ar9311AeLimits;

typedef struct {
	uint32_t lines;
	unsigned gain_code;
} Ar9311AePoint;

/* Exposure compensation range, stops, and the target it may reach: +2 EV
 * from 56 is 224, still under the clip threshold the meter counts at. */
#define AR9311_AE_EV_MAX 2.0
#define AR9311_AE_TARGET_MIN 8u
#define AR9311_AE_TARGET_MAX 224u

/** Metered-luma target for an exposure compensation of @p ev stops:
 *  AR9311_AE_TARGET * 2^ev, clamped to [TARGET_MIN, TARGET_MAX]. */
uint32_t ar9311_ae_target_for_ev(double ev);

/** Gain of a sensor gain code, Q10. */
uint32_t ar9311_ae_gain_q10(unsigned code);

/* The Bayer channels of a zone the meter averages, bit n for channel n.
 * The NT99235 meters all four.  The SC231AI's greens are channels 0 and 3;
 * its red and blue sit well below green until the ISP's white balance
 * lifts them, so an average over all four would overexpose it. */
#define AR9311_AE_METER_ALL 0xfu
#define AR9311_AE_METER_GREEN_03 0x9u

/** Meter one RRO grid (the stats snapshot without its leading sequence
 *  word) over @p channels (AR9311_AE_METER_*; 0 means all four).  Returns
 *  0, or -1 for a grid with no pixel counts. */
int ar9311_ae_meter(const uint8_t *rro, unsigned channels, Ar9311AeMeter *out);

/** Operating point for a total exposure in line-gain units (lines x gain).
 *  Integration time first, then the largest gain code that does not
 *  overshoot, then lines again to absorb the code's rounding.  With
 *  anti-flicker on and an exposure of at least one half mains period, the
 *  lines are a whole number of half periods and the gain takes the rest. */
Ar9311AePoint ar9311_ae_split(double exposure, const Ar9311AeLimits *lim);

/** One control decision.  @p exposure is the current total (line-gain
 *  units) and is updated.  Returns 1 when it moved, 0 inside the deadband. */
int ar9311_ae_step(double *exposure, const Ar9311AeMeter *meter,
	const Ar9311AeLimits *lim);

/** Exposure (line-gain units) of an operating point. */
double ar9311_ae_exposure_of(const Ar9311AePoint *pt);

/* ── Runtime ─────────────────────────────────────────────────────────── */

typedef struct Ar9311Ae Ar9311Ae;

/** Start the AE thread at @p rate_hz decisions per second (1..30).  Starts
 *  from the sensor's current setting.  NULL (with the reason on stderr) when
 *  the statistics or sensor interfaces are missing; the camera then simply
 *  keeps its fixed exposure. */
Ar9311Ae *ar9311_ae_start(const Ar9311AeLimits *limits, uint32_t rate_hz);

/** Called with the new gain (Q10) each time the AE moves it, just before it
 *  re-arms the ISP's gain ladders: a stage that follows the gain writes its
 *  parameter here and rides on that re-arm.  NULL for none. */
void ar9311_ae_set_gain_hook(void (*hook)(uint32_t gain_q10));

/** Replace the limits (live config changes).  Takes effect next decision. */
void ar9311_ae_set_limits(Ar9311Ae *ae, const Ar9311AeLimits *limits);

/** Current limits, for a caller that changes one of them. */
void ar9311_ae_get_limits(Ar9311Ae *ae, Ar9311AeLimits *limits);

/** Status as a malloc'd JSON API reply, or NULL. */
char *ar9311_ae_status_json(Ar9311Ae *ae);

/** Stop and join the thread, free @p ae.  NULL is fine. */
void ar9311_ae_stop(Ar9311Ae *ae);

/* ── Shared with the AWB ─────────────────────────────────────────────── */

/** Read one coherent statistics snapshot (AR9311_AE_STATS_SIZE bytes) into
 *  @p buf and its flip sequence into @p seq.  0, or -1 when the driver has
 *  none yet or a flip landed mid-copy. */
int ar9311_stats_read(uint8_t *buf, uint32_t *seq);

/** Name of the camera sensor driver that bound the camera ("nt99235",
 *  "sc231ai"), or NULL. */
const char *ar9311_sensor_name(void);

#endif /* AR9311_AE_H */
