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

typedef struct {
	uint32_t fps;
	uint32_t bitrate_kbps;
	uint32_t gop_frames;   /* frames between IDRs, 1..AR9311_ENCODER_GOP_MAX */
	uint32_t min_qp;       /* 0 = driver default */
	uint32_t max_qp;       /* 0 = driver default */
} Ar9311EncoderParams;

typedef struct {
	int initialized;  /* set by open; close is a no-op without it */
	int fd;
	Ar9311Geometry src;
	int src_count;
	void *stream_map[AR9311_ENCODER_STREAM_BUFS];
	uint32_t stream_len[AR9311_ENCODER_STREAM_BUFS];
	int streaming;
} Ar9311Encoder;

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

/** Take back a source buffer the encoder has finished reading.  Returns 0
 *  with *index set, 1 when none is done, -1 on error. */
int ar9311_encoder_reclaim_source(Ar9311Encoder *enc, int *index);

/** Take a finished access unit.  Returns 0 with *slot, *data, *len and
 *  *timestamp_us (the source frame's) set -- Annex-B, valid until
 *  ar9311_encoder_release_stream, 0 bytes for a frame the driver flagged
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
