#include "ar9311_capture.h"

#include "ar9311_v4l2.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/videodev2.h>

#define AR9311_CAPTURE_NODE_NAME "ar-cvisp"
#define AR9311_CAPTURE_DEPTH_PARAM "/sys/module/ar_cvisp/parameters/depth"
/* Buffers requested from the node (MissingLynk ml-air-video's default).  The
 * block rotates through a five-slot hardware ring; extra V4L2 buffers are
 * what lets the encoder hold frames without starving it. */
#define AR9311_CAPTURE_REQ_BUFS 8
/* Frames the encoder may hold at once: a credit bound and, since every held
 * frame is one queued ahead of encode, a latency bound. */
#define AR9311_CAPTURE_HELD_MAX 3

/* The CVISP hold depth, read rather than assumed: running with the wrong
 * number starves the camera silently. */
static int read_depth(void)
{
	FILE *f = fopen(AR9311_CAPTURE_DEPTH_PARAM, "r");
	int depth = 1;

	if (f) {
		if (fscanf(f, "%d", &depth) != 1 || depth < 1)
			depth = 1;
		fclose(f);
	}
	return depth;
}

static int read_geometry(int fd, Ar9311Geometry *geo)
{
	struct v4l2_format fmt;
	uint32_t i;

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
		perror("[ar9311] G_FMT camera");
		return -1;
	}
	if (fmt.fmt.pix_mp.num_planes != AR9311_CAPTURE_MAX_PLANES ||
	    fmt.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_YUV420M) {
		fprintf(stderr, "ERROR: [ar9311] camera format is not 3-plane "
			"YUV420M (%u planes)\n", fmt.fmt.pix_mp.num_planes);
		return -1;
	}
	geo->width = fmt.fmt.pix_mp.width;
	geo->height = fmt.fmt.pix_mp.height;
	geo->planes = fmt.fmt.pix_mp.num_planes;
	for (i = 0; i < geo->planes; i++) {
		geo->bytesperline[i] = fmt.fmt.pix_mp.plane_fmt[i].bytesperline;
		geo->sizeimage[i] = fmt.fmt.pix_mp.plane_fmt[i].sizeimage;
	}
	return 0;
}

static void init_buffer(struct v4l2_buffer *buf, struct v4l2_plane *planes,
	uint32_t count)
{
	memset(buf, 0, sizeof(*buf));
	memset(planes, 0, sizeof(*planes) * count);
	buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	buf->memory = V4L2_MEMORY_MMAP;
	buf->m.planes = planes;
	buf->length = count;
}

static int export_buffers(Ar9311Capture *cap)
{
	struct v4l2_requestbuffers req;
	int i;
	uint32_t p;

	memset(&req, 0, sizeof(req));
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	req.memory = V4L2_MEMORY_MMAP;
	req.count = AR9311_CAPTURE_REQ_BUFS;
	if (ioctl(cap->fd, VIDIOC_REQBUFS, &req) != 0) {
		perror("[ar9311] REQBUFS camera");
		return -1;
	}
	if (req.count < 2 || req.count > AR9311_CAPTURE_MAX_BUFS) {
		fprintf(stderr, "ERROR: [ar9311] camera gave %u buffers\n",
			req.count);
		return -1;
	}
	cap->buf_count = (int)req.count;
	for (i = 0; i < cap->buf_count; i++) {
		for (p = 0; p < cap->geo.planes; p++) {
			struct v4l2_exportbuffer exp;

			memset(&exp, 0, sizeof(exp));
			exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
			exp.index = (uint32_t)i;
			exp.plane = p;
			exp.flags = O_RDWR | O_CLOEXEC;
			if (ioctl(cap->fd, VIDIOC_EXPBUF, &exp) != 0) {
				perror("[ar9311] EXPBUF camera");
				return -1;
			}
			cap->dmabuf[i][p] = exp.fd;
		}
	}
	return 0;
}

int ar9311_capture_open(Ar9311Capture *cap, uint32_t width, uint32_t height)
{
	int i, p;

	memset(cap, 0, sizeof(*cap));
	cap->initialized = 1;
	for (i = 0; i < AR9311_CAPTURE_MAX_BUFS; i++)
		for (p = 0; p < AR9311_CAPTURE_MAX_PLANES; p++)
			cap->dmabuf[i][p] = -1;
	cap->fd = ar9311_v4l2_open_by_name(AR9311_CAPTURE_NODE_NAME);
	if (cap->fd < 0)
		return -1;
	if (read_geometry(cap->fd, &cap->geo) != 0)
		return -1;
	if (cap->geo.width != width || cap->geo.height != height) {
		fprintf(stderr, "ERROR: [ar9311] camera delivers %ux%u, config "
			"asks for %ux%u\n", cap->geo.width, cap->geo.height,
			width, height);
		return -1;
	}
	if (export_buffers(cap) != 0)
		return -1;
	cap->max_held = cap->buf_count - (read_depth() + 1);
	if (cap->max_held > AR9311_CAPTURE_HELD_MAX)
		cap->max_held = AR9311_CAPTURE_HELD_MAX;
	if (cap->max_held < 1)
		cap->max_held = 1;
	for (i = 0; i < cap->buf_count; i++) {
		if (ar9311_capture_requeue(cap, i) != 0)
			return -1;
	}
	printf("> [ar9311] camera %ux%u, %d buffers, %d may be held\n",
		cap->geo.width, cap->geo.height, cap->buf_count, cap->max_held);
	return 0;
}

int ar9311_capture_start(Ar9311Capture *cap)
{
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	if (ioctl(cap->fd, VIDIOC_STREAMON, &type) != 0) {
		perror("[ar9311] STREAMON camera");
		return -1;
	}
	cap->streaming = 1;
	return 0;
}

int ar9311_capture_dequeue(Ar9311Capture *cap, int *index, int *corrupt,
	uint64_t *timestamp_us)
{
	struct v4l2_buffer buf;
	struct v4l2_plane planes[AR9311_CAPTURE_MAX_PLANES];

	init_buffer(&buf, planes, cap->geo.planes);
	if (ioctl(cap->fd, VIDIOC_DQBUF, &buf) != 0) {
		if (errno == EAGAIN)
			return 1;
		perror("[ar9311] DQBUF camera");
		return -1;
	}
	*index = (int)buf.index;
	*corrupt = (buf.flags & V4L2_BUF_FLAG_ERROR) ? 1 : 0;
	*timestamp_us = (uint64_t)buf.timestamp.tv_sec * 1000000u +
		(uint64_t)buf.timestamp.tv_usec;
	return 0;
}

int ar9311_capture_requeue(Ar9311Capture *cap, int index)
{
	struct v4l2_buffer buf;
	struct v4l2_plane planes[AR9311_CAPTURE_MAX_PLANES];

	init_buffer(&buf, planes, cap->geo.planes);
	buf.index = (uint32_t)index;
	if (ioctl(cap->fd, VIDIOC_QBUF, &buf) != 0) {
		perror("[ar9311] QBUF camera");
		return -1;
	}
	return 0;
}

void ar9311_capture_close(Ar9311Capture *cap)
{
	int i, p;

	if (!cap->initialized)
		return;
	if (cap->fd >= 0 && cap->streaming) {
		int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

		(void)ioctl(cap->fd, VIDIOC_STREAMOFF, &type);
	}
	cap->streaming = 0;
	for (i = 0; i < AR9311_CAPTURE_MAX_BUFS; i++) {
		for (p = 0; p < AR9311_CAPTURE_MAX_PLANES; p++) {
			if (cap->dmabuf[i][p] >= 0)
				close(cap->dmabuf[i][p]);
			cap->dmabuf[i][p] = -1;
		}
	}
	if (cap->fd >= 0)
		close(cap->fd);
	cap->fd = -1;
	cap->initialized = 0;
}
