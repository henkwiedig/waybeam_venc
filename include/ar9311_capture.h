#ifndef AR9311_CAPTURE_H
#define AR9311_CAPTURE_H

#include <stdint.h>

/* The AR9311 (Artosyn PROXIMA 9311) camera output: the ar-cvisp V4L2 node of
 * the MissingLynk open camera stack (NT99235 -> CSI-2 -> VIF -> ISP -> CVISP).
 * Planar YUV 4:2:0 in three planes, luma stride wider than the picture.
 *
 * Buffers are MMAP on the node and every plane is exported as a dma-buf once,
 * so the encoder imports them without a copy (see ar9311_encoder.h). */

#define AR9311_CAPTURE_MAX_PLANES 3
#define AR9311_CAPTURE_MAX_BUFS 8

typedef struct {
	uint32_t width;
	uint32_t height;
	uint32_t planes;
	uint32_t bytesperline[AR9311_CAPTURE_MAX_PLANES];
	uint32_t sizeimage[AR9311_CAPTURE_MAX_PLANES];
} Ar9311Geometry;

typedef struct {
	int initialized;  /* set by open; close is a no-op without it */
	int fd;
	Ar9311Geometry geo;
	int buf_count;
	int dmabuf[AR9311_CAPTURE_MAX_BUFS][AR9311_CAPTURE_MAX_PLANES];
	/* Buffers a consumer may hold at once.  The node completes a buffer
	 * only while depth + 1 others stay queued under it, so everything
	 * above that is what can be out with the encoder. */
	int max_held;
	int streaming;
} Ar9311Capture;

/** Find the ar-cvisp node, check it delivers width x height, allocate and
 *  export the buffers and queue them all.  Returns 0 or -1. */
int ar9311_capture_open(Ar9311Capture *cap, uint32_t width, uint32_t height);

/** STREAMON.  Brings the whole camera chain up (sensor included). */
int ar9311_capture_start(Ar9311Capture *cap);

/** Take a filled buffer.  Returns 0 with *index and *timestamp_us (the
 *  driver's CLOCK_MONOTONIC capture time) set, 1 when none is ready, -1 on
 *  error.  *corrupt is set for a buffer the driver flagged bad. */
int ar9311_capture_dequeue(Ar9311Capture *cap, int *index, int *corrupt,
	uint64_t *timestamp_us);

/** Give a buffer back to the camera.  Returns 0 or -1. */
int ar9311_capture_requeue(Ar9311Capture *cap, int index);

/** STREAMOFF, close the exported dma-bufs and the node.  Safe on a
 *  zeroed or partly opened capture. */
void ar9311_capture_close(Ar9311Capture *cap);

#endif /* AR9311_CAPTURE_H */
