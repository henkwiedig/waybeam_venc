#ifndef AR9311_MERGE_H
#define AR9311_MERGE_H

#include "ar9311_strips.h"

#include <stddef.h>
#include <stdint.h>

/* video0.strips (ar9311_strips.h): N wave5 instances encode horizontal
 * strips of a frame as pictures of their own; this rewrites their access
 * units into one H.265 picture with N slices, so the receiver sees an
 * ordinary 1080p stream and needs nothing new.
 *
 *   strip 0:  its SPS is rewritten to the full height (+ the last strip's
 *             conformance window); VPS/PPS/slice pass as they are.
 *   strip k:  its VPS/SPS/PPS are dropped (they must match strip 0's apart
 *             from the height), each slice header gets
 *             first_slice_segment_in_pic_flag 0 and its address moved down
 *             by the CTUs of the strips above.
 * Every strip but the last is whole CTU rows with no cropping.  Rows a strip
 * codes past a seam (overlap_rows) are slices of their own, which are
 * dropped; the addresses count only the rows that are sent.
 *
 * All encoders must run in lock step (same pictures, IDRs together).
 * Without overlap rows the loop filter must be off across slice boundaries
 * (pps_loop_filter_across_slices_enabled_flag 0), so the decoder filters
 * each seam exactly as the encoders treated their picture edges; with them
 * it may run across, as both sides of a seam are rows both encoders coded.
 * What cannot be matched: a motion vector that reads past a strip's edge
 * (its own padding at encode time, the neighbour's pixels at decode time,
 * or with overlap rows the other encoder's coding of the same rows), so the
 * picture drifts from the seams until the intra refresh passes; see
 * ar9311_strips.h for what that measured. */

#define AR9311_MERGE_MAX_STRIPS AR9311_STRIPS_MAX
#define AR9311_MERGE_MAX_PS 512

typedef struct {
	int valid;
	uint32_t width, height;
	uint32_t conf_flag, conf[4];
	uint32_t chroma_format_idc, separate_colour_plane;
	uint32_t log2_max_poc_lsb;
	uint32_t num_st_rps;
	uint32_t num_delta_pocs[65];
	uint32_t num_used_curr[65];   /* used_by_curr_pic entries per set */
	uint32_t long_term_present, num_lt_sps;
	uint32_t temporal_mvp, sao;
	uint32_t ctb_log2;
	size_t width_bit;        /* RBSP bit offset of pic_width_in_luma_samples */
	size_t after_conf_bit;   /* first bit after the conformance window */
	size_t stop_bit;         /* the rbsp_stop_one_bit */
} Ar9311HevcSps;

typedef struct {
	int valid;
	uint32_t pps_id;
	uint32_t dependent_slices, output_flag_present, num_extra_bits;
	uint32_t cabac_init_present, weighted_pred, weighted_bipred;
	uint32_t num_ref_l0_def, num_ref_l1_def;
	uint32_t lists_mod_present, slice_chroma_qp_offsets_present;
	uint32_t tiles, wpp, lf_across_slices;
	uint32_t deblock_override_enabled, deblock_disabled;
	uint32_t header_ext_present, chroma_qp_offset_list;
	int32_t init_qp_minus26;
} Ar9311HevcPps;

typedef struct {
	int n;                   /* strips */
	Ar9311HevcSps sps[AR9311_MERGE_MAX_STRIPS];
	Ar9311HevcPps pps[AR9311_MERGE_MAX_STRIPS];
	uint8_t top_pps_rbsp[AR9311_MERGE_MAX_PS];
	size_t top_pps_len;
	int warned;
	/* coded heights: as configured until each strip's SPS confirms them
	 * (strip 0's SPS is rewritten before the others' first access units
	 * exist) */
	uint32_t coded_h[AR9311_MERGE_MAX_STRIPS];
	uint32_t conf_bottom;    /* last strip's conformance window, chroma rows */
	/* CTU rows each strip codes past each of its seams, sliced off and
	 * dropped here (ar9311_strips.h) */
	uint32_t overlap_rows;
	/* last picture of each strip, for the lock-step check */
	int idr[AR9311_MERGE_MAX_STRIPS];
	uint32_t poc[AR9311_MERGE_MAX_STRIPS];
	/* and the QP its rate control chose for it: the slice QP of its first
	 * slice sent (ar9311_strips_balance()) */
	int qp[AR9311_MERGE_MAX_STRIPS];
	/* scratch */
	uint8_t *rbsp, *out_rbsp;
	size_t scratch_len;
	const char *error;       /* last failure, for the log */
} Ar9311Merge;

/** @p n strips (2..AR9311_MERGE_MAX_STRIPS), @p heights their coded
 *  picture heights in lines, top to bottom, @p overlap_rows of them past
 *  each seam. */
void ar9311_merge_init(Ar9311Merge *m, int n, const uint32_t *heights,
	uint32_t overlap_rows);
void ar9311_merge_free(Ar9311Merge *m);

/** Rewrite strip @p k's access unit into @p out (capacity @p cap); strips
 *  of one frame come in order 0..n-1.  Returns the output length, or 0 with
 *  m->error set (m->error may also be set as a note on success). */
size_t ar9311_merge_strip(Ar9311Merge *m, int k, const uint8_t *au,
	size_t len, uint8_t *out, size_t cap);

/** 1 when the last pictures of all strips were the same picture (same POC
 *  LSB and IDR-ness). */
int ar9311_merge_in_step(const Ar9311Merge *m);

#endif /* AR9311_MERGE_H */
