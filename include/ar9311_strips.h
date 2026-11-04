#ifndef AR9311_STRIPS_H
#define AR9311_STRIPS_H

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
 * 5/4/4/4 rows at 1080p. */

#define AR9311_STRIPS_MAX 4
#define AR9311_STRIPS_ROW_LINES 64

typedef struct {
	int count;
	uint32_t y0[AR9311_STRIPS_MAX];     /* first line */
	uint32_t height[AR9311_STRIPS_MAX]; /* lines */
	/* Strips 0..count-2: queued this long after the camera timestamp, the
	 * sub-frame-sync schedule's release time for their last band.  The
	 * last strip is queued right behind the one before it (0 here) and
	 * follows the camera from sfs_start_us, its first band's release. */
	uint32_t due_us[AR9311_STRIPS_MAX];
	int32_t sfs_start_us;
} Ar9311StripPlan;

/** Lay out @p count strips (2..AR9311_STRIPS_MAX) over a picture
 *  @p height lines tall, on the sub-frame-sync schedule @p sfs_start_us /
 *  @p sfs_row_us of the whole frame (ar9311_encoder_sfs_schedule()).
 *  Returns 0, or -1 for a count the picture cannot take. */
static inline int ar9311_strips_plan(int count, uint32_t height,
	int32_t sfs_start_us, uint32_t sfs_row_us, Ar9311StripPlan *plan)
{
	uint32_t rows_total = (height + AR9311_STRIPS_ROW_LINES - 1) /
		AR9311_STRIPS_ROW_LINES;
	uint32_t base, extra, y = 0;
	int k;

	if (count < 2 || count > AR9311_STRIPS_MAX ||
	    rows_total < (uint32_t)count)
		return -1;
	base = rows_total / (uint32_t)count;
	extra = rows_total % (uint32_t)count;
	plan->count = count;
	for (k = 0; k < count; k++) {
		uint32_t rows = base + ((uint32_t)k < extra ? 1u : 0u);
		uint32_t first_row = y / AR9311_STRIPS_ROW_LINES;

		plan->y0[k] = y;
		plan->height[k] = k == count - 1 ? height - y :
			rows * AR9311_STRIPS_ROW_LINES;
		y += plan->height[k];
		if (k < count - 1) {
			plan->due_us[k] = (uint32_t)(sfs_start_us +
				(int32_t)(sfs_row_us * (first_row + rows - 1)));
		} else {
			plan->due_us[k] = 0;
			plan->sfs_start_us = sfs_start_us +
				(int32_t)(sfs_row_us * first_row);
		}
	}
	return 0;
}

#endif /* AR9311_STRIPS_H */
