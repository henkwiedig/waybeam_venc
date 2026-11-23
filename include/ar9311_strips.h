#ifndef AR9311_STRIPS_H
#define AR9311_STRIPS_H

#include <math.h>
#include <stdint.h>

/* video0.strips: the frame encoded as N horizontal strips, one wave5
 * instance each, joined into one picture of N slices (ar9311_merge.h).
 *
 * The AR9311 has a single H.26x core, so the strips are encoded one after
 * the other, not in parallel; what they buy is time.  A strip is queued as
 * soon as the camera has written its last line, and is encoded while the
 * camera is still writing the strips below it; its slice goes onto the
 * frame-shm ring as a part of the access unit at once, so the link carries
 * the top of the picture while the rest is being encoded.  The last strip
 * follows the camera band by band (sub-frame sync, which wave5 allows one
 * instance at a time).  Measured on a P1 -> VRX Pro at ~22 Mbit/s,
 * capture -> ground receiver, 1/2/3/4 strips: 28.7 / 23.4 / 20.4 / 18.9 ms;
 * the vsync-locked display target 47.8 / 37.5 / 38.0 / 35.6 ms, as every
 * part costs the radio's write path ~1.5 ms.
 *
 * Strips are whole 64-line CTU rows, so the seams fall on the picture's CTU
 * grid; the spare rows go to the top strips, leaving the last strip, the
 * one that decides when the picture is complete, the smallest: 9/8, 6/6/5,
 * 5/4/4/4 rows at 1080p.
 *
 * Overlap (video0.stripOverlap): a strip's motion vectors may reach past its
 * edge, where its own encoder saw padding and the decoder sees the
 * neighbouring strip, so the picture drifts from the seams until the intra
 * refresh passes -- little in a still scene, badly when a tilt moves the
 * picture across a seam (P1, hand tilt: up to 40 % of a frame's pixels,
 * worst frames 29 dB from what the encoders meant).  With overlap_rows each
 * strip also encodes that many rows beyond each seam, cut off as slices of
 * their own, which the merge drops: past the seam the encoder then refers
 * to the same picture content the decoder has, only coded by the other
 * encoder (1 row: 10-90x fewer pixels off by 8 levels or more, worst
 * frames 36-43 dB; what is left comes from moves of more than 64 lines a
 * frame).  A strip with rows above its seam has to start a slice there, and
 * wave5's slices are all of one size, so it is coded in slices of
 * overlap_rows rows.  With the overlap the loop filter runs across slice
 * edges: each encoder deblocks its own, and the decoder deblocks the seam
 * with pixels both encoders had close copies of.  Cost at 1 row: the first
 * part +1.4 ms, the whole picture +1.8 ms, ~12 % more lines encoded. */

#define AR9311_STRIPS_MAX 4
#define AR9311_STRIPS_OVERLAP_MAX 2
#define AR9311_STRIPS_ROW_LINES 64

typedef struct {
	int count;
	uint32_t overlap_rows;
	uint32_t y0[AR9311_STRIPS_MAX];     /* first line sent */
	uint32_t height[AR9311_STRIPS_MAX]; /* lines sent */
	uint32_t enc_y0[AR9311_STRIPS_MAX];     /* first line encoded */
	uint32_t enc_height[AR9311_STRIPS_MAX]; /* lines encoded */
	uint32_t slice_rows[AR9311_STRIPS_MAX]; /* CTU rows a slice, 0 = one */
	/* Strips 0..count-2: queued this long after the camera timestamp, the
	 * sub-frame-sync schedule's release time for their last band.  The
	 * last strip is queued right behind the one before it (0 here) and
	 * follows the camera from sfs_start_us, its first band's release. */
	uint32_t due_us[AR9311_STRIPS_MAX];
	int32_t sfs_start_us;
} Ar9311StripPlan;

/** Lay out @p count strips (2..AR9311_STRIPS_MAX) over a picture
 *  @p height lines tall, @p overlap_rows CTU rows past each seam
 *  (0..AR9311_STRIPS_OVERLAP_MAX), on the sub-frame-sync schedule
 *  @p sfs_start_us / @p sfs_row_us of the whole frame
 *  (ar9311_encoder_sfs_schedule()).  Returns 0, or -1 for a layout the
 *  picture cannot take. */
static inline int ar9311_strips_plan(int count, uint32_t height,
	uint32_t overlap_rows, int32_t sfs_start_us, uint32_t sfs_row_us,
	Ar9311StripPlan *plan)
{
	uint32_t rows_total = (height + AR9311_STRIPS_ROW_LINES - 1) /
		AR9311_STRIPS_ROW_LINES;
	uint32_t base, extra, y = 0, o = overlap_rows;
	int k;

	if (count < 2 || count > AR9311_STRIPS_MAX ||
	    o > AR9311_STRIPS_OVERLAP_MAX ||
	    rows_total < (uint32_t)count)
		return -1;
	base = rows_total / (uint32_t)count;
	extra = rows_total % (uint32_t)count;
	plan->count = count;
	plan->overlap_rows = o;
	for (k = 0; k < count; k++) {
		uint32_t rows = base + ((uint32_t)k < extra ? 1u : 0u);
		uint32_t first_row = y / AR9311_STRIPS_ROW_LINES;
		uint32_t top = k > 0 ? o : 0, bottom = k < count - 1 ? o : 0;

		/* a middle strip's bottom overlap has to start a slice too */
		if (top && bottom && rows % o)
			return -1;
		plan->y0[k] = y;
		plan->height[k] = k == count - 1 ? height - y :
			rows * AR9311_STRIPS_ROW_LINES;
		plan->enc_y0[k] = y - top * AR9311_STRIPS_ROW_LINES;
		plan->enc_height[k] = plan->height[k] +
			(top + bottom) * AR9311_STRIPS_ROW_LINES;
		plan->slice_rows[k] = !o ? 0 : k == 0 ? rows : o;
		y += plan->height[k];
		if (k < count - 1) {
			plan->due_us[k] = (uint32_t)(sfs_start_us +
				(int32_t)(sfs_row_us *
					(first_row + rows - 1 + bottom)));
		} else {
			plan->due_us[k] = 0;
			plan->sfs_start_us = sfs_start_us +
				(int32_t)(sfs_row_us * (first_row - top));
		}
	}
	return 0;
}

/* Bitrate.  Every strip has an encoder and a rate control of its own, so the
 * way the bitrate is shared decides how the strips' qualities compare.
 * Shared by lines, the strip with the busier picture (the ground below the
 * sky) is quantised harder, and the seam shows as a step in sharpness: in a
 * P1 flight (11-23, 2 strips, ~12 Mbit/s) the slice QP was 29 above the seam
 * and 37 below, detail across the seam a third down.  So a strip's share is
 * its coded lines times 2^(bias/6) -- 6 QP is about twice the bits -- and the
 * biases follow the strips' QPs, closing a set fraction of the gap
 * (video0.stripBalance).
 *
 * Closing it costs latency.  The last strip is sent after the camera has
 * finished, the others while it is still writing, so a bit moved into the
 * last strip delays the picture by its time on the radio.  P1 bench, 22
 * Mbit/s, plain top and busy bottom, capture -> goggles: 22.4 ms at 0 %
 * (QP 19.9/24.2), 23.6 ms at 50 % (20.4/23.6), 26.2 ms at 100 % (22.0/22.5,
 * 22 % / 78 % of the bitrate). */
#define AR9311_STRIPS_BIAS_MAX 6.0f       /* QP: half to twice by lines */
#define AR9311_STRIPS_BIAS_DEADBAND 0.5f  /* QP a step would move */

/** Share @p kbps out to the strips of @p plan by coded lines weighted by
 *  @p bias, so that what is left once the overlap rows are dropped comes to
 *  @p kbps; the shares add up exactly.  All biases 0: by lines alone. */
static inline void ar9311_strips_split(const Ar9311StripPlan *plan,
	const float *bias, uint32_t kbps, uint32_t *share)
{
	double w[AR9311_STRIPS_MAX], wsum = 0.0;
	uint64_t coded = 0;
	uint32_t total, left;
	int k;

	for (k = 0; k < plan->count; k++) {
		coded += plan->enc_height[k];
		w[k] = plan->enc_height[k] * exp2(bias[k] / 6.0);
		wsum += w[k];
	}
	total = left = (uint32_t)((uint64_t)kbps * coded /
		(plan->y0[plan->count - 1] + plan->height[plan->count - 1]));
	for (k = 0; k < plan->count; k++) {
		share[k] = k == plan->count - 1 ? left :
			(uint32_t)((double)total * w[k] / wsum);
		left -= share[k];
	}
}

/** Move @p bias toward closing the fraction @p strength (0..1) of the gap
 *  between the strips' qualities, from each strip's mean slice QP @p qp over
 *  the last stretch.  A strip's QP above the mean (weighted by coded lines)
 *  asks for that much more bias -- a QP of bias is about a QP of gap -- so
 *  the bias it should have is strength x (bias + distance): at 1 the QPs
 *  meet, at 0.5 the gap that is left equals the bias, so about half closes,
 *  at 0 the biases go back to 0 (by lines).  Each step goes half the way,
 *  then the biases are centred and held within +-AR9311_STRIPS_BIAS_MAX;
 *  a step that would move no bias by AR9311_STRIPS_BIAS_DEADBAND is not
 *  taken.  Returns 1 when a bias moved (the shares need setting again). */
static inline int ar9311_strips_balance(const Ar9311StripPlan *plan,
	const float *qp, float strength, float *bias)
{
	double lines = 0.0, mean = 0.0, centre = 0.0, far = 0.0;
	float was[AR9311_STRIPS_MAX], move[AR9311_STRIPS_MAX];
	int k, moved = 0;

	for (k = 0; k < plan->count; k++) {
		lines += plan->enc_height[k];
		mean += plan->enc_height[k] * (double)qp[k];
	}
	mean /= lines;
	for (k = 0; k < plan->count; k++) {
		move[k] = strength * (bias[k] + (float)(qp[k] - mean)) - bias[k];
		if (fabsf(move[k]) > far)
			far = fabsf(move[k]);
	}
	if (far < AR9311_STRIPS_BIAS_DEADBAND)
		return 0;
	for (k = 0; k < plan->count; k++) {
		was[k] = bias[k];
		bias[k] += 0.5f * move[k];
		centre += plan->enc_height[k] * (double)bias[k];
	}
	centre /= lines;
	for (k = 0; k < plan->count; k++) {
		bias[k] -= (float)centre;
		if (bias[k] > AR9311_STRIPS_BIAS_MAX)
			bias[k] = AR9311_STRIPS_BIAS_MAX;
		if (bias[k] < -AR9311_STRIPS_BIAS_MAX)
			bias[k] = -AR9311_STRIPS_BIAS_MAX;
		if (fabsf(bias[k] - was[k]) > 0.01f)
			moved = 1;
	}
	return moved;
}

#endif /* AR9311_STRIPS_H */
