#ifndef AR9311_PICTURE_H
#define AR9311_PICTURE_H

#include <math.h>
#include <stdint.h>

/* Picture controls of the AR9311 ISP: saturation, contrast and sharpness in
 * percent,
 * 100 = the tuning's own picture.  They are module parameters of the
 * MissingLynk ar_isp driver as patched by the OpenIPC builder (P1 kernel
 * patch 0002), re-latched live through its debugfs ladders and tone files.
 * Without the patch the parameters are missing and the apply fails. */

#define AR9311_PICTURE_NEUTRAL 100u
#define AR9311_PICTURE_MAX 200u

/* Saturation falls with gain above 64x: there the picture is mostly chroma
 * noise on lifted shadows (purple on an SC231AI at 364x, where the vendor's
 * picture stays near grey).  Full saturation to 64x, which is all the
 * NT99235 reaches, then down linearly in log2(gain) to the floor at 640x,
 * the SC231AI's top.  This ramp stands in for the ISP's cm stage, which
 * the vendor lowers with the exposure; when the AE drives cm itself
 * (ar9311_picture_set_gain_ramp) the ramp is off, as stock's colour matrix
 * stays unscaled at night: both together left the picture grey (chroma
 * 2.5 against stock's 8.0 at 500x). */
#define AR9311_PICTURE_SAT_KNEE_Q10 (64u * 1024u)
#define AR9311_PICTURE_SAT_TOP_Q10 (640u * 1024u)
#define AR9311_PICTURE_SAT_FLOOR_PCT 35.0

/** @p saturation as it applies at @p gain_q10 (Q10, 1024 = 1x). */
static inline uint32_t ar9311_picture_saturation_at(uint32_t saturation,
	uint32_t gain_q10)
{
	double t, pct;

	if (gain_q10 <= AR9311_PICTURE_SAT_KNEE_Q10)
		return saturation;
	t = log2((double)gain_q10 / AR9311_PICTURE_SAT_KNEE_Q10) /
		log2((double)AR9311_PICTURE_SAT_TOP_Q10 / AR9311_PICTURE_SAT_KNEE_Q10);
	if (t > 1.0)
		t = 1.0;
	pct = 100.0 - t * (100.0 - AR9311_PICTURE_SAT_FLOOR_PCT);
	return (uint32_t)(saturation * pct / 100.0 + 0.5);
}

/** Write the controls and re-latch the ISP.  Only a control that differs
 *  from the last successful apply is written.  Saturation is applied as
 *  ar9311_picture_saturation_at() the current gain.  Returns 0 or -1. */
int ar9311_picture_apply(uint32_t saturation, uint32_t contrast,
	uint32_t sharpness);

/** The AE's gain hook: re-scale saturation for @p gain_q10.  Writes the
 *  parameter only; the AE's ladder re-arm that follows latches it. */
void ar9311_picture_set_gain(uint32_t gain_q10);

/** Whether saturation follows the gain ramp above (default on).  Off when
 *  the ISP's cm stage follows the exposure instead; takes effect at the
 *  next apply or gain move. */
void ar9311_picture_set_gain_ramp(int on);

#endif /* AR9311_PICTURE_H */
