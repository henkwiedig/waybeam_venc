#include "ar9311_encoder.h"

#include "ar9311_v4l2.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>

/* wave5 registers a decoder and an encoder node; this is the encoder. */
#define AR9311_ENCODER_NODE_NAME "C&M Wave5 VPU encoder"
/* One access unit per buffer.  A 1080p IDR at the top of the bitrate range
 * stays well below this; the driver rounds the request up itself. */
#define AR9311_ENCODER_STREAM_BYTES (1024u * 1024u)
/* Rate-control buffer window.  Without one the CBR loop overshot its target
 * about 2x on the bench; ~6 frames at 60 fps keeps the rate tight without
 * starving IDRs. */
#define AR9311_ENCODER_VBV_MS 100
/* QP of the instance's first IDR, coded before rate control has history:
 * the vendor encoder's own value (MissingLynk docs/air-video-benchmark.md). */
#define AR9311_ENCODER_FIRST_IDR_QP 35
/* MissingLynk wave5 debugfs: state lists live instances ("inst N ..."),
 * recover resets the codec block and re-downloads its firmware. */
#define AR9311_WAVE5_STATE "/sys/kernel/debug/wave5/state"
#define AR9311_WAVE5_RECOVER "/sys/kernel/debug/wave5/recover"

static int set_ctrl(int fd, uint32_t id, int32_t value, const char *name)
{
	struct v4l2_control ctrl;

	memset(&ctrl, 0, sizeof(ctrl));
	ctrl.id = id;
	ctrl.value = value;
	if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) != 0) {
		fprintf(stderr, "ERROR: [ar9311] encoder %s=%d: %s\n", name,
			(int)value, strerror(errno));
		return -1;
	}
	return 0;
}

/* Source format at the camera's own strides.  The encoder must take them
 * unchanged, or a plane lands at the wrong offset inside the imported
 * dma-buf; a demand for more bytes than the camera buffer holds would read
 * past it. */
static int set_source_format(int fd, const Ar9311Geometry *src)
{
	struct v4l2_format fmt;
	uint32_t i;

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420M;
	fmt.fmt.pix_mp.width = src->width;
	fmt.fmt.pix_mp.height = src->height;
	fmt.fmt.pix_mp.num_planes = (uint8_t)src->planes;
	for (i = 0; i < src->planes; i++) {
		fmt.fmt.pix_mp.plane_fmt[i].bytesperline = src->bytesperline[i];
		fmt.fmt.pix_mp.plane_fmt[i].sizeimage = src->sizeimage[i];
	}
	if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
		perror("[ar9311] S_FMT encoder source");
		return -1;
	}
	if (fmt.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_YUV420M ||
	    fmt.fmt.pix_mp.num_planes != src->planes) {
		fprintf(stderr, "ERROR: [ar9311] encoder substituted the source "
			"format\n");
		return -1;
	}
	for (i = 0; i < src->planes; i++) {
		if (fmt.fmt.pix_mp.plane_fmt[i].bytesperline != src->bytesperline[i] ||
		    fmt.fmt.pix_mp.plane_fmt[i].sizeimage > src->sizeimage[i]) {
			fprintf(stderr, "ERROR: [ar9311] encoder will not take plane "
				"%u at stride %u\n", i, src->bytesperline[i]);
			return -1;
		}
	}
	return 0;
}

static int set_stream_format(int fd, const Ar9311Geometry *src)
{
	struct v4l2_format fmt;

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_HEVC;
	fmt.fmt.pix_mp.width = src->width;
	fmt.fmt.pix_mp.height = src->height;
	fmt.fmt.pix_mp.num_planes = 1;
	fmt.fmt.pix_mp.plane_fmt[0].sizeimage = AR9311_ENCODER_STREAM_BYTES;
	if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
		perror("[ar9311] S_FMT encoder HEVC");
		return -1;
	}
	return 0;
}

/* Rate control.  The driver defaults to fixed QP with no bitrate target and
 * to 30 fps, so the frame rate has to be set or the CBR budget is computed
 * for the wrong number of frames. */
static int set_rate_control(int fd, const Ar9311EncoderParams *params)
{
	struct v4l2_streamparm parm;

	memset(&parm, 0, sizeof(parm));
	parm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	parm.parm.output.timeperframe.numerator = 1;
	parm.parm.output.timeperframe.denominator = params->fps;
	if (ioctl(fd, VIDIOC_S_PARM, &parm) != 0) {
		perror("[ar9311] S_PARM encoder frame rate");
		return -1;
	}
	if (set_ctrl(fd, V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, 1, "rc") != 0 ||
	    set_ctrl(fd, V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE, 1, "mb_rc") != 0 ||
	    set_ctrl(fd, V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
		V4L2_MPEG_VIDEO_BITRATE_MODE_CBR, "cbr") != 0 ||
	    set_ctrl(fd, V4L2_CID_MPEG_VIDEO_VBV_SIZE, AR9311_ENCODER_VBV_MS,
		"vbv") != 0 ||
	    set_ctrl(fd, V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP,
		AR9311_ENCODER_FIRST_IDR_QP, "i_frame_qp") != 0 ||
	    set_ctrl(fd, V4L2_CID_MPEG_VIDEO_BITRATE,
		(int32_t)(params->bitrate_kbps * 1000u), "bitrate") != 0 ||
	    set_ctrl(fd, V4L2_CID_MPEG_VIDEO_GOP_SIZE,
		params->refresh_period ? 0 : (int32_t)params->gop_frames,
		"gop") != 0)
		return -1;
	/* Gradual refresh instead of keyframes: a band of intra CTU rows in
	 * every picture, the opening IDR the only one.  A keyframe is ~13x a
	 * P-frame at 1080p and stalls the radio link for tens of ms; the band
	 * spreads that cost evenly. */
	if (params->refresh_period &&
	    (set_ctrl(fd, V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD_TYPE,
		V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD_TYPE_CYCLIC,
		"refresh_type") != 0 ||
	     set_ctrl(fd, V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD,
		(int32_t)params->refresh_period, "refresh_period") != 0)) {
		fprintf(stderr, "ERROR: [ar9311] this kernel's wave5 has no intra "
			"refresh (builder patch 0001-media-wave5-expose-cyclic-"
			"intra-refresh); use video0.resilience=off\n");
		return -1;
	}
	/* Orientation in the encoder's pre-processor rather than at the
	 * sensor: a sensor mirror or flip moves the Bayer phase the ISP is
	 * tuned for.  wave5 latches these at instance open and only ever sets
	 * the bits, so 0 is left unwritten. */
	if ((params->mirror && set_ctrl(fd, V4L2_CID_HFLIP, 1, "hflip") != 0) ||
	    (params->flip && set_ctrl(fd, V4L2_CID_VFLIP, 1, "vflip") != 0))
		return -1;
	if (params->min_qp && set_ctrl(fd, V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP,
		(int32_t)params->min_qp, "min_qp") != 0)
		return -1;
	if (params->max_qp && set_ctrl(fd, V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP,
		(int32_t)params->max_qp, "max_qp") != 0)
		return -1;
	/* Every IDR carries VPS/SPS/PPS in-band, so a receiver (or the
	 * frame-shm consumer) can start at any IDR.  No access-unit
	 * delimiters: they cost a NAL per frame and nothing downstream uses
	 * them.  Level 4.1 is the lowest that admits 1080p60; unset, the VPS
	 * carries general_level_idc 10, which is not a defined level. */
	if (set_ctrl(fd, V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR, 1,
		"prepend_param_sets") != 0 ||
	    set_ctrl(fd, V4L2_CID_MPEG_VIDEO_AU_DELIMITER, 0, "aud") != 0 ||
	    set_ctrl(fd, V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
		V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1, "level") != 0)
		return -1;
	return 0;
}

static int request_buffers(int fd, uint32_t type, uint32_t memory,
	uint32_t count)
{
	struct v4l2_requestbuffers req;

	memset(&req, 0, sizeof(req));
	req.type = type;
	req.memory = memory;
	req.count = count;
	if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
		perror("[ar9311] REQBUFS encoder");
		return -1;
	}
	if (req.count < count) {
		fprintf(stderr, "ERROR: [ar9311] encoder gave %u of %u buffers\n",
			req.count, count);
		return -1;
	}
	return 0;
}

static void init_stream_buffer(struct v4l2_buffer *buf,
	struct v4l2_plane *plane)
{
	memset(buf, 0, sizeof(*buf));
	memset(plane, 0, sizeof(*plane));
	buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	buf->memory = V4L2_MEMORY_MMAP;
	buf->m.planes = plane;
	buf->length = 1;
}

static int map_stream_buffers(Ar9311Encoder *enc)
{
	int i;

	if (request_buffers(enc->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		V4L2_MEMORY_MMAP, AR9311_ENCODER_STREAM_BUFS) != 0)
		return -1;
	for (i = 0; i < AR9311_ENCODER_STREAM_BUFS; i++) {
		struct v4l2_buffer buf;
		struct v4l2_plane plane;
		void *map;

		init_stream_buffer(&buf, &plane);
		buf.index = (uint32_t)i;
		if (ioctl(enc->fd, VIDIOC_QUERYBUF, &buf) != 0) {
			perror("[ar9311] QUERYBUF encoder");
			return -1;
		}
		map = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, enc->fd,
			plane.m.mem_offset);
		if (map == MAP_FAILED) {
			perror("[ar9311] mmap encoder bitstream");
			return -1;
		}
		enc->stream_map[i] = map;
		enc->stream_len[i] = plane.length;
		if (ar9311_encoder_release_stream(enc, i) != 0)
			return -1;
	}
	return 0;
}

/* Return the VPU to its boot state before this instance is created.
 *
 * The wave5 firmware keeps hidden state across instances: after the first
 * encoder instance of a boot, a new one can come up emitting a valid stream
 * of pure noise at an inflated bitrate, or watchdogging, whatever its input
 * (MissingLynk ml-kernel docs/wave5-encoder-instance-reuse.md; reproduced
 * on a P1).  A block reset with a firmware re-download clears it -- verified
 * on a P1 whose VPU was already poisoned: three restarts in a row, all clean.
 *
 * The reset destroys every live instance, so it is skipped when any exists
 * (nothing else uses the codec on an air unit, but it is not ours to kill). */
static void reset_vpu(void)
{
	char line[128];
	int live = 0, ok = 0;
	FILE *f = fopen(AR9311_WAVE5_STATE, "r");

	if (!f) {
		fprintf(stderr, "WARN: [ar9311] %s missing (debugfs not mounted?): "
			"VPU not reset, a restart within this boot may encode "
			"noise\n", AR9311_WAVE5_STATE);
		return;
	}
	while (fgets(line, sizeof(line), f))
		if (strncmp(line, "inst ", 5) == 0)
			live++;
	fclose(f);
	if (live) {
		fprintf(stderr, "WARN: [ar9311] %d other wave5 instance(s) live: "
			"VPU not reset\n", live);
		return;
	}
	f = fopen(AR9311_WAVE5_RECOVER, "w");
	if (f) {
		/* The reset runs on the write, which fclose flushes. */
		int put_ok = fputs("1", f) != EOF;

		ok = fclose(f) == 0 && put_ok;
	}
	if (!ok) {
		fprintf(stderr, "WARN: [ar9311] VPU reset failed: %s\n",
			strerror(errno));
		return;
	}
	printf("> [ar9311] VPU reset (block reset + firmware reload)\n");
}

int ar9311_encoder_open(Ar9311Encoder *enc, const Ar9311Geometry *src,
	int src_count, const Ar9311EncoderParams *params)
{
	memset(enc, 0, sizeof(*enc));
	enc->initialized = 1;
	enc->src = *src;
	enc->src_count = src_count;
	reset_vpu();
	enc->fd = ar9311_v4l2_open_by_name(AR9311_ENCODER_NODE_NAME);
	if (enc->fd < 0)
		return -1;
	if (set_source_format(enc->fd, src) != 0 ||
	    set_stream_format(enc->fd, src) != 0 ||
	    set_rate_control(enc->fd, params) != 0)
		return -1;
	if (request_buffers(enc->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		V4L2_MEMORY_DMABUF, (uint32_t)src_count) != 0)
		return -1;
	if (map_stream_buffers(enc) != 0)
		return -1;
	if (params->refresh_period)
		printf("> [ar9311] encoder H.265 %ux%u@%u, %u kbps CBR, intra "
			"refresh over %u frames, no periodic IDR\n", src->width,
			src->height, params->fps, params->bitrate_kbps,
			params->refresh_period);
	else
		printf("> [ar9311] encoder H.265 %ux%u@%u, %u kbps CBR, IDR every "
			"%u frames\n", src->width, src->height, params->fps,
			params->bitrate_kbps, params->gop_frames);
	return 0;
}

static int stream_on(int fd, uint32_t type)
{
	int t = (int)type;

	if (ioctl(fd, VIDIOC_STREAMON, &t) != 0) {
		perror("[ar9311] STREAMON encoder");
		return -1;
	}
	return 0;
}

int ar9311_encoder_start(Ar9311Encoder *enc)
{
	if (stream_on(enc->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) != 0 ||
	    stream_on(enc->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) != 0)
		return -1;
	enc->streaming = 1;
	return 0;
}

int ar9311_encoder_queue_source(Ar9311Encoder *enc, int index, const int *fds,
	uint64_t timestamp_us)
{
	struct v4l2_buffer buf;
	struct v4l2_plane planes[AR9311_CAPTURE_MAX_PLANES];
	uint32_t i;

	memset(&buf, 0, sizeof(buf));
	memset(planes, 0, sizeof(planes));
	buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	buf.memory = V4L2_MEMORY_DMABUF;
	buf.index = (uint32_t)index;
	buf.m.planes = planes;
	buf.length = enc->src.planes;
	/* mem2mem copies the source timestamp onto the access unit. */
	buf.timestamp.tv_sec = (time_t)(timestamp_us / 1000000u);
	buf.timestamp.tv_usec = (suseconds_t)(timestamp_us % 1000000u);
	for (i = 0; i < enc->src.planes; i++) {
		planes[i].m.fd = fds[i];
		planes[i].length = enc->src.sizeimage[i];
		planes[i].bytesused = enc->src.sizeimage[i];
	}
	if (ioctl(enc->fd, VIDIOC_QBUF, &buf) != 0) {
		perror("[ar9311] QBUF encoder source");
		return -1;
	}
	return 0;
}

int ar9311_encoder_reclaim_source(Ar9311Encoder *enc, int *index)
{
	struct v4l2_buffer buf;
	struct v4l2_plane planes[AR9311_CAPTURE_MAX_PLANES];

	memset(&buf, 0, sizeof(buf));
	memset(planes, 0, sizeof(planes));
	buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	buf.memory = V4L2_MEMORY_DMABUF;
	buf.m.planes = planes;
	buf.length = enc->src.planes;
	if (ioctl(enc->fd, VIDIOC_DQBUF, &buf) != 0) {
		if (errno == EAGAIN)
			return 1;
		perror("[ar9311] DQBUF encoder source");
		return -1;
	}
	*index = (int)buf.index;
	return 0;
}

int ar9311_encoder_dequeue_stream(Ar9311Encoder *enc, int *slot,
	const uint8_t **data, size_t *len, uint64_t *timestamp_us)
{
	struct v4l2_buffer buf;
	struct v4l2_plane plane;

	init_stream_buffer(&buf, &plane);
	if (ioctl(enc->fd, VIDIOC_DQBUF, &buf) != 0) {
		if (errno == EAGAIN)
			return 1;
		perror("[ar9311] DQBUF encoder bitstream");
		return -1;
	}
	if (buf.index >= AR9311_ENCODER_STREAM_BUFS ||
	    plane.bytesused > enc->stream_len[buf.index]) {
		fprintf(stderr, "ERROR: [ar9311] encoder returned bad buffer %u\n",
			buf.index);
		return -1;
	}
	*slot = (int)buf.index;
	*data = (const uint8_t *)enc->stream_map[buf.index] + plane.data_offset;
	*len = plane.bytesused > plane.data_offset
		? plane.bytesused - plane.data_offset : 0;
	if (buf.flags & V4L2_BUF_FLAG_ERROR)
		*len = 0;
	*timestamp_us = (uint64_t)buf.timestamp.tv_sec * 1000000u +
		(uint64_t)buf.timestamp.tv_usec;
	return 0;
}

int ar9311_encoder_release_stream(Ar9311Encoder *enc, int slot)
{
	struct v4l2_buffer buf;
	struct v4l2_plane plane;

	init_stream_buffer(&buf, &plane);
	buf.index = (uint32_t)slot;
	if (ioctl(enc->fd, VIDIOC_QBUF, &buf) != 0) {
		perror("[ar9311] QBUF encoder bitstream");
		return -1;
	}
	return 0;
}

int ar9311_encoder_set_bitrate(Ar9311Encoder *enc, uint32_t kbps)
{
	return set_ctrl(enc->fd, V4L2_CID_MPEG_VIDEO_BITRATE,
		(int32_t)(kbps * 1000u), "bitrate");
}

int ar9311_encoder_request_idr(Ar9311Encoder *enc)
{
	return set_ctrl(enc->fd, V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 0,
		"force_key_frame");
}

void ar9311_encoder_close(Ar9311Encoder *enc)
{
	int i;

	if (!enc->initialized)
		return;
	if (enc->fd >= 0 && enc->streaming) {
		int type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;

		(void)ioctl(enc->fd, VIDIOC_STREAMOFF, &type);
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		(void)ioctl(enc->fd, VIDIOC_STREAMOFF, &type);
	}
	enc->streaming = 0;
	for (i = 0; i < AR9311_ENCODER_STREAM_BUFS; i++) {
		if (enc->stream_map[i])
			munmap(enc->stream_map[i], enc->stream_len[i]);
		enc->stream_map[i] = NULL;
	}
	if (enc->fd >= 0)
		close(enc->fd);
	enc->fd = -1;
	enc->initialized = 0;
}
