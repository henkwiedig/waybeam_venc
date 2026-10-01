#ifndef AR9311_IQ_H
#define AR9311_IQ_H

#include <stddef.h>
#include <stdint.h>

/* /api/v1/iq on AR9311: the CV610 response shape ("_schema", groups with
 * "fields"), carrying only the fields a ground-station menu written for
 * CV610 drives, each one a view of one picture control:
 *
 *   exposure.auto.compensation   AE target 14..224     ev (target 56 * 2^ev)
 *   saturation.op_type           0 auto / 1 manual     saturation 100 %
 *   saturation.manual.saturation 0..255, 128 = tuned   saturation
 *   csc.contr                    0..100, 50 = tuned    contrast
 *   sharpen.op_type              0 auto / 1 manual     sharpness 100 %
 *   sharpen.manual.texture_strength / .edge_strength
 *                                32 x 0..4095, 2048 = tuned  sharpness
 *
 * Live only, like CV610's ISP state: nothing is saved, and a waybeam
 * restart starts from the tuned picture.  This file is the pure mapping;
 * ar9311_runtime.c holds the state and applies it. */

/* The controls: saturation, contrast and sharpness in percent of the tuned
 * picture (0..200), ev in stops (-2..+2). */
typedef struct {
	uint32_t saturation;
	uint32_t contrast;
	uint32_t sharpness;
	double ev;
} Ar9311Picture;

#define AR9311_IQ_SAT_NEUTRAL 128
#define AR9311_IQ_SHARP_NEUTRAL 2048
#define AR9311_IQ_SHARP_ENTRIES 32

/** The tuned picture: 100 % everywhere, 0 EV. */
void ar9311_iq_neutral(Ar9311Picture *pic);

/** Render the whole IQ surface for @p pic as {"ok":true,"data":{...}}.
 *  Returns the length, or -1 when @p buf is too small. */
int ar9311_iq_render(const Ar9311Picture *pic, char *buf, size_t size);

/** Apply one IQ write to @p pic.  Out-of-range values and unknown fields are
 *  rejected (-1) and leave @p pic alone, as on CV610.  op_type 1 (manual)
 *  changes nothing until a manual value is written. */
int ar9311_iq_set(Ar9311Picture *pic, const char *param, const char *value);

#endif /* AR9311_IQ_H */
