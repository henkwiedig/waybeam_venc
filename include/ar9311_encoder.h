#ifndef AR9311_ENCODER_H
#define AR9311_ENCODER_H

#include "ar9311_capture.h"

#include <stddef.h>
#include <stdint.h>

/* The AR9311's Chips&Media Wave521C H.265 encoder, as the mainline wave5
 * stateful V4L2 mem2mem driver (with the MissingLynk patches: 2048-byte
 * source stride, live bitrate, forced keyframes).
 *
 * The OUTPUT (source) queue imports the camera's exported planes, one V4L2
 * buffer per camera buffer, same index.  The CAPTURE queue holds mmapped
 * bitstream buffers, one access unit each.
 *
 * The codec firmware keeps hidden state across instance destroy, so a later
 * instance of the same boot can encode noise (MissingLynk ml-kernel
 * docs/wave5-encoder-instance-reuse.md).  ar9311_encoder_open() therefore
 * resets the VPU and reloads its firmware first, through the MissingLynk
 * wave5 debugfs recover hook, whenever no other codec instance is live. */

#define AR9311_ENCODER_STREAM_BUFS 4
#define AR9311_ENCODER_GOP_MAX 2047  /* V4L2_CID_MPEG_VIDEO_GOP_SIZE range */

/* wave5 codes H.265 in 64x64 CTUs, so its cyclic intra refresh walks rows
 * of 64 picture lines: 17 at 1080p.  The driver turns a refresh period P
 * into rows / P rows per picture and refreshes every row of every picture
 * when P exceeds the row count.  The shared intra_refresh_compute() counts
 * 32-line rows, the CV610 and SigmaStar CTU. */
#define AR9311_ENCODER_REFRESH_ROW_LINES 64

typedef struct {
	uint32_t total_rows;  /* refresh rows in the picture */
	uint32_t period;      /* the V4L2 refresh period, 1..total_rows */
	uint32_t rows;        /* rows refreshed per picture */
	uint32_t cycle;       /* pictures per full refresh */
} Ar9311RefreshPlan;

/** Plan wave5's refresh for a picture @p height (> 0) lines tall that
 *  should refresh at least @p shared_rows of intra_refresh_compute()'s
 *  32-line rows per picture. */
static inline void ar9311_encoder_refresh_plan(uint32_t height,
	uint32_t shared_rows, Ar9311RefreshPlan *plan)
{
	uint32_t want = (shared_rows + 1) / 2;

	plan->total_rows = (height + AR9311_ENCODER_REFRESH_ROW_LINES - 1) /
		AR9311_ENCODER_REFRESH_ROW_LINES;
	if (want < 1)
		want = 1;
	if (want > plan->total_rows)
		want = plan->total_rows;
	plan->period = plan->total_rows / want;
	plan->rows = plan->total_rows / plan->period;
	plan->cycle = (plan->total_rows + plan->rows - 1) / plan->rows;
}

/* Sub-frame sync (the builder's 0004-media-wave5-sub-frame-sync kernel patch):
 * the encoder takes a camera buffer while the CVISP is still writing it and
 * works through it 64 lines at a time, each band released on a schedule
 * from the buffer's timestamp.  At CVISP depth 1 the timestamp marks the
 * start of the write and line L is in memory (L + 1) line times later;
 * every further unit of depth hands the buffer back one frame later.
 * Measured on a P1 (NT99235 1080p60, 1125 lines a frame): band n complete at
 * ts + 925 + 947 n us, chroma 60-110 us after its luma, spread <= ~100 us. */
#define AR9311_SFS_LINE_NS 14815   /* 1 s / (60 fps * 1125 lines) */
#define AR9311_SFS_BAND_LINES 64
#define AR9311_SFS_CHROMA_US 110
#define AR9311_SFS_MARGIN_US 400

/** Band 0 release (@p start_us after the timestamp) and band period
 *  (@p row_us) for a CVISP hold depth (>= 1) at @p fps. */
static inline void ar9311_encoder_sfs_schedule(uint32_t depth, uint32_t fps,
	int32_t *start_us, uint32_t *row_us)
{
	uint32_t band_us = (AR9311_SFS_BAND_LINES * AR9311_SFS_LINE_NS + 500u) /
		1000u;

	*row_us = band_us;
	*start_us = (int32_t)(band_us + AR9311_SFS_CHROMA_US +
		AR9311_SFS_MARGIN_US) -
		(int32_t)((depth - 1u) * (1000000u / fps));
}

typedef struct {
	uint32_t fps;
	uint32_t bitrate_kbps;
	uint32_t gop_frames;   /* frames between IDRs, 1..AR9311_ENCODER_GOP_MAX;
	                        * ignored with refresh_period set */
	uint32_t refresh_period; /* cyclic intra refresh period, see
	                          * Ar9311RefreshPlan; no periodic IDR; 0 = off
	                          * (needs the builder's wave5 intra-refresh
	                          * kernel patch) */
	uint32_t min_qp;       /* 0 = driver default */
	uint32_t max_qp;       /* 0 = driver default */
	int mirror;            /* left-right, done by the encoder */
	int flip;              /* upside down; both = rotated 180 degrees */
	int32_t sfs_start_us;  /* sub-frame sync schedule (see above) */
	uint32_t sfs_row_us;   /* 0 = sub-frame sync off */
	int no_lf_across_slices; /* deblocking/SAO stop at slice boundaries
	                          * (video0.strips: each seam is one) */
} Ar9311EncoderParams;

typedef struct {
	int initialized;  /* set by open; close is a no-op without it */
	int fd;
	Ar9311Geometry src;
	int src_count;
	void *stream_map[AR9311_ENCODER_STREAM_BUFS];
	uint32_t stream_len[AR9311_ENCODER_STREAM_BUFS];
	uint8_t *stream_copy;  /* cached copy of the last access unit taken */
	int streaming;
} Ar9311Encoder;

/** 1 when the encoder has the sub-frame sync controls, else 0.  Opens and
 *  closes the node without streaming, which leaves the firmware alone. */
int ar9311_encoder_has_sfs(void);

/** Reset the VPU (see above), find the encoder node, program it for the
 *  camera geometry and @p params,
 *  import @p src_count source buffers and queue the bitstream buffers.
 *  Returns 0 or -1. */
int ar9311_encoder_open(Ar9311Encoder *enc, const Ar9311Geometry *src,
	int src_count, const Ar9311EncoderParams *params);

/** STREAMON on both queues.  Returns 0 or -1. */
int ar9311_encoder_start(Ar9311Encoder *enc);

/** Queue camera buffer @p index (its planes as dma-buf @p fds) for encode.
 *  @p timestamp_us travels with the frame to its access unit. */
int ar9311_encoder_queue_source(Ar9311Encoder *enc, int index, const int *fds,
	uint64_t timestamp_us);

/** The same, starting @p first_row lines into the camera buffer, for a
 *  strip encoder whose own height is the strip's (video0.strips). */
int ar9311_encoder_queue_source_rows(Ar9311Encoder *enc, int index,
	const int *fds, uint64_t timestamp_us, uint32_t first_row);

/** Take back a source buffer the encoder has finished reading.  Returns 0
 *  with *index set, 1 when none is done, -1 on error. */
int ar9311_encoder_reclaim_source(Ar9311Encoder *enc, int *index);

/** Take a finished access unit.  Returns 0 with *slot, *data, *len and
 *  *timestamp_us (the source frame's) set -- Annex-B in a cached copy,
 *  valid until the next call, 0 bytes for a frame the driver flagged
 *  failed -- 1 when none is ready, -1 on error. */
int ar9311_encoder_dequeue_stream(Ar9311Encoder *enc, int *slot,
	const uint8_t **data, size_t *len, uint64_t *timestamp_us);

/** Give bitstream buffer @p slot back to the encoder. */
int ar9311_encoder_release_stream(Ar9311Encoder *enc, int slot);

/** Live target bitrate; takes effect on the next frame. */
int ar9311_encoder_set_bitrate(Ar9311Encoder *enc, uint32_t kbps);

/** Encode the next frame as an IDR. */
int ar9311_encoder_request_idr(Ar9311Encoder *enc);

/** STREAMOFF, unmap and close.  Safe on a zeroed or partly opened encoder. */
void ar9311_encoder_close(Ar9311Encoder *enc);

#endif /* AR9311_ENCODER_H */
