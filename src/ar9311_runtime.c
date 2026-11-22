#include "ar9311_runtime.h"

#include "ar9311_ae.h"
#include "ar9311_awb.h"
#include "ar9311_capture.h"
#include "ar9311_encoder.h"
#include "ar9311_iq.h"
#include "ar9311_merge.h"
#include "ar9311_strips.h"
#include "ar9311_picture.h"
#include "ar9311_validation.h"
#include "h26x_param_sets.h"
#include "h26x_util.h"
#include "hevc_rtp.h"
#include "idr_rate_limit.h"
#include "intra_refresh.h"
#include "output_socket.h"
#include "pipeline_common.h"
#include "rtp_session.h"
#include "timing.h"
#include "venc_api.h"
#include "venc_frame_ring.h"
#include "venc_httpd.h"
#include "venc_respawn.h"
#include "venc_ring.h"

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/timerfd.h>
#include <time.h>
#include <unistd.h>

#define AR9311_VENC_CHN 0
#define AR9311_FRAME_RING_SLOTS 8
#define AR9311_FRAME_RING_BYTES (512u * 1024u)
/* A 60 fps camera that delivers nothing for this long has stalled. */
#define AR9311_STALL_MS 2000

typedef struct {
	VencConfig config;
	VencOutputUri output_uri;
	Ar9311Capture capture;
	Ar9311Encoder encoder;
	int held;                 /* camera buffers out with the encoder */
	Ar9311Ae *ae;             /* NULL when AE could not start */
	Ar9311Awb *awb;           /* NULL when off or the sensor has no calibration */

	int socket_handle;
	struct sockaddr_storage destination;
	socklen_t destination_len;
	VencOutputUriType transport;
	int connected_udp;
	/* Live retarget, seqlock-protected: the fields above are written by
	 * ar9311_apply_server() on the httpd thread and read once per frame
	 * by the encode loop.  Odd generation = a write is in progress.  Same
	 * pattern, and the same known gaps under connectedUdp, as the CV610
	 * and SigmaStar outputs (see the note in cv610_runtime.c). */
	unsigned transport_gen;
	struct {
		int socket_handle;
		struct sockaddr_storage destination;
		socklen_t destination_len;
		int connected_udp;
		int output_enabled;
	} tx;
	int output_enabled;
	/* The destination actually programmed, compared against instead of the
	 * config: venc_api commits a new value before dispatching the apply. */
	char applied_server[256];
	OutputSocketQueue send_queue;
	venc_frame_ring_t *frame_ring;
	VencRingLowWater low_water;
	int low_water_ready;
	int pressure_state;
	uint32_t pressure_frames;

	RtpPacketizerState rtp;
	H26xParamSets param_sets;
	HevcRtpStats rtp_stats;
	uint32_t frame_ticks;
	uint16_t max_payload_size;
	uint32_t live_bitrate;
	int verbose;

	uint64_t frames;
	uint64_t bytes;
	uint64_t camera_skips;
	uint64_t output_drops;
	uint64_t packets_sent;
	uint64_t last_frame_us;
	PipelineRateWatch rate_watch;

	/* Cyclic intra refresh (resilience presets with an intra mode). */
	uint8_t gdr_active;
	uint8_t gdr_cycle_len;    /* frames per refresh wave */
	uint8_t gdr_counter;      /* position of the next frame in the wave */
	uint8_t *wave_buf;        /* param sets + access unit, frame-shm only */
	size_t wave_buf_len;

	/* Added to the camera timestamp before it is published: at CVISP depth
	 * 1 it marks the start of the write, so one frame period keeps the
	 * published capture time where depth 2 puts it (just after the write)
	 * and latency figures comparable. */
	uint64_t pts_offset_us;

	/* video0.strips (ar9311_strips.h): strip k of the frame on encoder
	 * se[k] (se[0] is `encoder`), joined into one picture of N slices
	 * (ar9311_merge.h) and published as N parts. */
	int strips;                   /* 0, or the number of strips */
	Ar9311Encoder strip_enc[AR9311_STRIPS_MAX - 1];
	Ar9311Encoder *se[AR9311_STRIPS_MAX];
	Ar9311Merge merge;
	Ar9311StripPlan plan;
	struct {
		int used, index, queued, reclaimed;
		uint64_t ts;
	} strip_frame[AR9311_CAPTURE_MAX_BUFS];
	struct {
		int have, slot;
		const uint8_t *data;
		size_t len;
		uint64_t ts;
	} strip_au[AR9311_STRIPS_MAX];
	uint8_t *strip_buf;
	size_t strip_cap;
	int strip_idr_pending;
	int strip_next;               /* next strip of the open frame, 0 = none */
	uint64_t strip_cur_ts;
	VencFrameMeta strip_meta;
	uint64_t strip_merge_errors, strip_out_of_step, strip_abandoned;
	uint64_t strip_t_first_sum, strip_t_whole_sum, strip_t_n;
	uint64_t strip_last_report_us;
} Ar9311RunnerContext;

/* The shared VencApplyCallbacks ABI carries no opaque pointer, so like every
 * other backend the live-control wrappers refer to the one active runner.
 * Published before HTTP starts, cleared after HTTP stops. */
static Ar9311RunnerContext *g_ar9311_runner;
static volatile sig_atomic_t g_ar9311_stop;

static Ar9311IntraRefreshStatus g_ar9311_intra_status;
static pthread_mutex_t g_ar9311_status_mutex = PTHREAD_MUTEX_INITIALIZER;

void ar9311_runtime_intra_refresh_status(Ar9311IntraRefreshStatus *out)
{
	if (!out)
		return;
	pthread_mutex_lock(&g_ar9311_status_mutex);
	*out = g_ar9311_intra_status;
	pthread_mutex_unlock(&g_ar9311_status_mutex);
}

static void ar9311_signal_handler(int signo)
{
	(void)signo;
	g_ar9311_stop = 1;
}

/* ── Live controls ───────────────────────────────────────────────────── */

/* The strips' shares of @p kbps, by the lines each encodes, so that what is
 * left once the overlap rows are dropped comes to @p kbps. */
static void ar9311_strips_split(const Ar9311StripPlan *plan, uint32_t kbps,
	uint32_t *share)
{
	uint64_t coded = 0;
	uint32_t total, left;
	int k;

	for (k = 0; k < plan->count; k++)
		coded += plan->enc_height[k];
	total = left = (uint32_t)((uint64_t)kbps * coded / AR9311_HEIGHT);
	for (k = 0; k < plan->count; k++) {
		share[k] = k == plan->count - 1 ? left :
			(uint32_t)((uint64_t)total * plan->enc_height[k] / coded);
		left -= share[k];
	}
}

static int ar9311_apply_bitrate(uint32_t kbps)
{
	Ar9311RunnerContext *ctx = g_ar9311_runner;

	if (!ctx)
		return -1;
	if (ctx->strips) {
		uint32_t share[AR9311_STRIPS_MAX];
		int k;

		ar9311_strips_split(&ctx->plan, kbps, share);
		for (k = 0; k < ctx->strips; k++)
			if (ar9311_encoder_set_bitrate(ctx->se[k], share[k]) != 0)
				return -1;
	} else if (ar9311_encoder_set_bitrate(&ctx->encoder, kbps) != 0) {
		return -1;
	}
	__atomic_store_n(&ctx->live_bitrate, kbps, __ATOMIC_RELEASE);
	return 0;
}

static int ar9311_apply_verbose(bool on)
{
	if (!g_ar9311_runner)
		return -1;
	__atomic_store_n(&g_ar9311_runner->verbose, on ? 1 : 0, __ATOMIC_RELEASE);
	return 0;
}

static int ar9311_apply_max_payload_size(uint16_t size)
{
	if (!g_ar9311_runner)
		return -1;
	__atomic_store_n(&g_ar9311_runner->max_payload_size, size,
		__ATOMIC_RELEASE);
	return 0;
}

static int ar9311_request_idr(void)
{
	if (!g_ar9311_runner)
		return -1;
	if (!idr_rate_limit_allow(AR9311_VENC_CHN))
		return 0;
	if (g_ar9311_runner->strips) {
		/* both strips must turn IDR on the same frame: the loop asks
		 * both encoders right before it queues a top strip */
		__atomic_store_n(&g_ar9311_runner->strip_idr_pending, 1,
			__ATOMIC_RELEASE);
		return 0;
	}
	return ar9311_encoder_request_idr(&g_ar9311_runner->encoder);
}

static uint32_t ar9311_query_live_fps(void)
{
	return g_ar9311_runner ? AR9311_FPS : 0;
}

static int ar9311_transport_fill(Ar9311RunnerContext *ctx,
	venc_frame_ring_fill_t *ring_fill, uint8_t *fill_pct)
{
	if (ctx->frame_ring) {
		if (venc_frame_ring_get_fill(ctx->frame_ring, ring_fill) != 0)
			return -1;
		*fill_pct = ring_fill->fill_pct;
		return 0;
	}
	if (ctx->socket_handle >= 0)
		return output_socket_get_fill_pct(ctx->socket_handle,
			&ctx->send_queue, fill_pct);
	return -1;
}

static char *ar9311_query_transport_status(void)
{
	Ar9311RunnerContext *ctx = g_ar9311_runner;
	venc_frame_ring_fill_t ring;
	uint8_t fill = 0;
	int have_fill;
	char buf[512];
	int pos;

	if (!ctx)
		return NULL;
	memset(&ring, 0, sizeof(ring));
	have_fill = ar9311_transport_fill(ctx, &ring, &fill) == 0;
	if (!ctx->frame_ring && ctx->socket_handle < 0)
		pos = snprintf(buf, sizeof(buf), "{\"ok\":true,\"data\":{"
			"\"active\":false,\"transport\":\"none\"}}");
	else
		pos = snprintf(buf, sizeof(buf), "{\"ok\":true,\"data\":{"
			"\"active\":true,\"transport\":\"%s\",\"fillPct\":%u,"
			"\"inPressure\":%s,\"pressureDrops\":%u,"
			"\"transportDrops\":%llu,\"packetsSent\":%llu,"
			"\"cameraSkips\":%llu}}",
			ctx->frame_ring ? "frame-shm" :
				ctx->transport == VENC_OUTPUT_URI_UNIX ? "unix" : "udp",
			have_fill ? (unsigned)fill : 0u,
			__atomic_load_n(&ctx->pressure_state, __ATOMIC_RELAXED)
				? "true" : "false",
			(unsigned)__atomic_load_n(&ctx->pressure_frames,
				__ATOMIC_RELAXED),
			ctx->frame_ring ? (unsigned long long)ring.full_drops :
				(unsigned long long)__atomic_load_n(&ctx->output_drops,
					__ATOMIC_RELAXED),
			ctx->frame_ring ? (unsigned long long)ring.writes :
				(unsigned long long)__atomic_load_n(&ctx->packets_sent,
					__ATOMIC_RELAXED),
			(unsigned long long)__atomic_load_n(&ctx->camera_skips,
				__ATOMIC_RELAXED));
	if (pos < 0 || pos >= (int)sizeof(buf))
		return NULL;
	return strdup(buf);
}

/* Ring transports are created once, so a switch to or from one is
 * restart-class: commit, respawn, report success (the CV610 behaviour and
 * its reasoning).  A socket destination is validated before anything is
 * touched, then retargeted under the seqlock. */
static int ar9311_apply_server(const char *uri)
{
	Ar9311RunnerContext *ctx = g_ar9311_runner;
	VencOutputUri parsed;

	if (!ctx || !uri || venc_config_parse_output_uri(uri, &parsed) != 0)
		return -1;
	if (ctx->applied_server[0] && strcmp(ctx->applied_server, uri) == 0)
		return 0;
	if ((parsed.type == VENC_OUTPUT_URI_UDP ||
	     parsed.type == VENC_OUTPUT_URI_UNIX) &&
	    !output_socket_destination_is_usable(&parsed)) {
		fprintf(stderr, "ERROR: outgoing.server %s has no usable "
			"destination; transport left unchanged\n", uri);
		return -1;
	}
	if (ctx->frame_ring || parsed.type == VENC_OUTPUT_URI_SHM ||
	    parsed.type == VENC_OUTPUT_URI_FRAME_SHM) {
		printf("> Destination %s requires a restart (ring transports are "
			"created once); committed, respawning\n", uri);
		snprintf(ctx->applied_server, sizeof(ctx->applied_server), "%s",
			uri);
		venc_api_request_reinit();
		return 0;
	}
	__atomic_fetch_add(&ctx->transport_gen, 1, __ATOMIC_RELEASE);
	if (output_socket_configure(&ctx->socket_handle, &ctx->destination,
		&ctx->destination_len, &ctx->transport, &parsed,
		ctx->config.outgoing.connected_udp,
		ctx->config.outgoing.allow_unix_encoder_stall,
		&ctx->connected_udp) != 0) {
		ctx->applied_server[0] = '\0';
		__atomic_fetch_add(&ctx->transport_gen, 1, __ATOMIC_RELEASE);
		return -1;
	}
	(void)output_socket_capture_capacity(ctx->socket_handle,
		&ctx->send_queue);
	ctx->output_uri = parsed;
	__atomic_fetch_add(&ctx->transport_gen, 1, __ATOMIC_RELEASE);
	snprintf(ctx->applied_server, sizeof(ctx->applied_server), "%s", uri);
	if (ar9311_request_idr() != 0)
		fprintf(stderr, "WARN: destination changed but the bootstrap IDR "
			"request failed\n");
	printf("> Destination changed to %s\n", uri);
	return 0;
}

/* Gates both transports; the encoder keeps running (fps is restart-only). */
static int ar9311_apply_output_enabled(bool on)
{
	Ar9311RunnerContext *ctx = g_ar9311_runner;

	if (!ctx)
		return -1;
	if (ctx->output_enabled == (on ? 1 : 0))
		return 0;
	if (on && ctx->socket_handle < 0 && !ctx->frame_ring) {
		if (!ctx->config.outgoing.server[0] ||
		    ar9311_apply_server(ctx->config.outgoing.server) != 0)
			return -1;
	}
	__atomic_store_n(&ctx->output_enabled, on ? 1 : 0, __ATOMIC_RELEASE);
	if (on && ar9311_request_idr() != 0)
		fprintf(stderr, "WARN: output enabled but the bootstrap IDR "
			"request failed\n");
	printf("> Output %s\n", on ? "enabled" : "disabled");
	return 0;
}

/* The picture controls behind /api/v1/iq.  Live only: they start from the
 * tuned picture on every daemon start, as CV610's ISP state does. */
static pthread_mutex_t g_ar9311_picture_mutex = PTHREAD_MUTEX_INITIALIZER;
static Ar9311Picture g_ar9311_picture = { 100, 100, 100, 0.0 };

static double ar9311_picture_ev(void)
{
	double ev;

	pthread_mutex_lock(&g_ar9311_picture_mutex);
	ev = g_ar9311_picture.ev;
	pthread_mutex_unlock(&g_ar9311_picture_mutex);
	return ev;
}

/* ── Auto-exposure limits ────────────────────────────────────────────── */

static uint32_t ar9311_us_to_lines(uint32_t us, uint32_t fallback)
{
	uint64_t lines;

	if (us == 0)
		return fallback;
	lines = (uint64_t)us * 1000u / AR9311_AE_LINE_NS;
	if (lines < 1)
		lines = 1;
	return lines > AR9311_AE_LINES_MAX ? AR9311_AE_LINES_MAX : (uint32_t)lines;
}

/* Config limits in the AE's units.  0 means "no limit": the sensor's own
 * range, 1 line to one frame period and 1x to its gain ceiling (64x on the
 * NT99235, 640x on the SC231AI; the AE caps it per sensor). */
static void ar9311_ae_limits_from(const VencConfigIsp *isp, Ar9311AeLimits *lim)
{
	lim->lines_min = ar9311_us_to_lines(isp->shutter_min_us, 1);
	lim->lines_max = ar9311_us_to_lines(isp->shutter_max_us,
		AR9311_AE_LINES_MAX);
	lim->gain_min_q10 = isp->gain_min ? isp->gain_min : AR9311_AE_GAIN_ONE;
	lim->gain_max_q10 = isp->gain_max ? isp->gain_max :
		ar9311_ae_gain_q10(AR9311_AE_GAIN_CODE_MAX);
	if (lim->lines_min > lim->lines_max)
		lim->lines_min = lim->lines_max;
	if (lim->gain_min_q10 > lim->gain_max_q10)
		lim->gain_min_q10 = lim->gain_max_q10;
	lim->target = ar9311_ae_target_for_ev(ar9311_picture_ev());
	lim->flicker_hz = isp->anti_flicker_hz;
}

/* One live AE setting changed (a limit or anti-flicker): the shared API has
 * already committed it to the config, so rebuild them all from there. */
static int ar9311_apply_ae_limit(uint32_t value)
{
	Ar9311RunnerContext *ctx = g_ar9311_runner;
	Ar9311AeLimits lim;

	(void)value;
	if (!ctx || !ctx->ae)
		return -1;
	ar9311_ae_limits_from(&ctx->config.isp, &lim);
	ar9311_ae_set_limits(ctx->ae, &lim);
	return 0;
}

static char *ar9311_query_iq_info(void)
{
	Ar9311Picture pic;
	char *buf = malloc(4096);

	if (!buf)
		return NULL;
	pthread_mutex_lock(&g_ar9311_picture_mutex);
	pic = g_ar9311_picture;
	pthread_mutex_unlock(&g_ar9311_picture_mutex);
	if (ar9311_iq_render(&pic, buf, 4096) < 0) {
		free(buf);
		return NULL;
	}
	return buf;
}

/* The ISP takes saturation, contrast and sharpness; EV moves the AE target.
 * The state is committed only once the ISP has taken it. */
static int ar9311_apply_iq_param(const char *param, const char *value)
{
	Ar9311Picture pic;
	int rc = 0;

	pthread_mutex_lock(&g_ar9311_picture_mutex);
	pic = g_ar9311_picture;
	if (ar9311_iq_set(&pic, param, value) != 0) {
		pthread_mutex_unlock(&g_ar9311_picture_mutex);
		fprintf(stderr, "[ar9311] iq: %s=%s rejected\n", param, value);
		return -1;
	}
	if (ar9311_picture_apply(pic.saturation, pic.contrast,
		pic.sharpness) != 0)
		rc = -1;
	else
		g_ar9311_picture = pic;
	pthread_mutex_unlock(&g_ar9311_picture_mutex);
	/* The AE limits read the committed EV; without AE it has no effect. */
	if (rc == 0 && g_ar9311_runner && g_ar9311_runner->ae)
		rc = ar9311_apply_ae_limit(0);
	return rc;
}

static char *ar9311_query_ae_info(void)
{
	return g_ar9311_runner ? ar9311_ae_status_json(g_ar9311_runner->ae) : NULL;
}

/* ── White balance ───────────────────────────────────────────────────── */

/* isp.awbMode ct_manual fixes the balance at isp.awbCt; auto follows the
 * light.  isp.awbFps 0 leaves the gains the camera loader set. */
static uint32_t ar9311_awb_manual_ct(const VencConfigIsp *isp)
{
	return strcmp(isp->awb_mode, "ct_manual") == 0 ? isp->awb_ct : 0;
}

static void ar9311_awb_begin(Ar9311RunnerContext *ctx)
{
	if (ctx->config.isp.awb_fps)
		ctx->awb = ar9311_awb_start(ctx->config.isp.awb_fps,
			ar9311_awb_manual_ct(&ctx->config.isp));
}

static int ar9311_apply_awb_rate(uint32_t hz)
{
	Ar9311RunnerContext *ctx = g_ar9311_runner;

	(void)hz;   /* already committed to the config */
	if (!ctx)
		return -1;
	ar9311_awb_stop(ctx->awb);
	ctx->awb = NULL;
	ar9311_awb_begin(ctx);
	return ctx->config.isp.awb_fps && !ctx->awb ? -1 : 0;
}

static int ar9311_apply_awb_mode(int mode, uint32_t ct)
{
	Ar9311RunnerContext *ctx = g_ar9311_runner;

	if (!ctx || (!ctx->awb && ctx->config.isp.awb_fps))
		return -1;
	ar9311_awb_set_mode(ctx->awb, mode == 1 ? ct : 0);
	return 0;
}

static char *ar9311_query_awb_info(void)
{
	if (!g_ar9311_runner)
		return NULL;
	/* Off (awbFps 0, or no calibration for this sensor) is a state, not
	 * a failure: the gains are the ones the camera loader set. */
	if (!g_ar9311_runner->awb)
		return strdup("{\"ok\":true,\"data\":{\"engine\":\"ar9311\","
			"\"mode\":\"off\"}}");
	return ar9311_awb_status_json(g_ar9311_runner->awb);
}

static const VencApplyCallbacks g_ar9311_apply_callbacks = {
	.apply_bitrate = ar9311_apply_bitrate,
	.apply_verbose = ar9311_apply_verbose,
	.request_idr = ar9311_request_idr,
	.query_live_fps = ar9311_query_live_fps,
	.apply_max_payload_size = ar9311_apply_max_payload_size,
	.query_transport_status = ar9311_query_transport_status,
	.apply_server = ar9311_apply_server,
	.apply_output_enabled = ar9311_apply_output_enabled,
	.apply_gain_max = ar9311_apply_ae_limit,
	.apply_gain_min = ar9311_apply_ae_limit,
	.apply_shutter_max = ar9311_apply_ae_limit,
	.apply_shutter_min = ar9311_apply_ae_limit,
	.apply_anti_flicker = ar9311_apply_ae_limit,
	.query_iq_info = ar9311_query_iq_info,
	.apply_iq_param = ar9311_apply_iq_param,
	.query_ae_info = ar9311_query_ae_info,
	.query_awb_info = ar9311_query_awb_info,
	.apply_awb_mode = ar9311_apply_awb_mode,
	.apply_awb_rate = ar9311_apply_awb_rate,
};

/* ── Output ──────────────────────────────────────────────────────────── */

static void ar9311_transport_begin_frame(Ar9311RunnerContext *ctx)
{
	unsigned gen_before, gen_after;

	for (;;) {
		gen_before = __atomic_load_n(&ctx->transport_gen, __ATOMIC_ACQUIRE);
		if (gen_before & 1u) {
			sched_yield();
			continue;
		}
		ctx->tx.socket_handle = ctx->socket_handle;
		ctx->tx.destination = ctx->destination;
		ctx->tx.destination_len = ctx->destination_len;
		ctx->tx.connected_udp = ctx->connected_udp;
		ctx->tx.output_enabled =
			__atomic_load_n(&ctx->output_enabled, __ATOMIC_ACQUIRE);
		gen_after = __atomic_load_n(&ctx->transport_gen, __ATOMIC_ACQUIRE);
		if (gen_before == gen_after)
			break;
	}
}

static int ar9311_output_write(const uint8_t *header, size_t header_len,
	const uint8_t *payload1, size_t payload1_len,
	const uint8_t *payload2, size_t payload2_len, void *opaque)
{
	Ar9311RunnerContext *ctx = opaque;
	int ret;

	ret = output_socket_send_parts(ctx->tx.socket_handle,
		&ctx->tx.destination, ctx->tx.destination_len,
		ctx->tx.connected_udp, header, header_len,
		payload1, payload1_len, payload2, payload2_len);
	if (ret != 0) {
		__atomic_add_fetch(&ctx->output_drops, 1, __ATOMIC_RELAXED);
		if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS)
			return 0;
	} else {
		__atomic_add_fetch(&ctx->packets_sent, 1, __ATOMIC_RELAXED);
	}
	return ret;
}

/* One access unit as RTP.  VPS/SPS/PPS are tracked and re-sent ahead of
 * every IDR, so a receiver can join at any IDR. */
static void ar9311_send_rtp_frame(Ar9311RunnerContext *ctx,
	const uint8_t *frame, size_t len, int wave_head)
{
	size_t cursor = 0;
	const uint8_t *nal;
	size_t nal_len;
	uint16_t max_payload;

	max_payload = __atomic_load_n(&ctx->max_payload_size, __ATOMIC_ACQUIRE);
	/* A refresh wave is where a receiver can join, so it gets the
	 * parameter sets an IDR would carry (19 = IDR_W_RADL selects them). */
	if (wave_head)
		(void)hevc_rtp_prepend_param_sets(&ctx->param_sets, 19, &ctx->rtp,
			ar9311_output_write, ctx, max_payload, &ctx->rtp_stats);
	while (h26x_util_annexb_next(frame, len, &cursor, &nal, &nal_len)) {
		uint8_t nal_type = h26x_util_hevc_nalu_type(nal, nal_len);

		if (nal_type == 19 || nal_type == 20)
			(void)hevc_rtp_prepend_param_sets(&ctx->param_sets, nal_type,
				&ctx->rtp, ar9311_output_write, ctx, max_payload,
				&ctx->rtp_stats);
		if (hevc_rtp_send_nal(nal, nal_len, &ctx->rtp, ar9311_output_write,
			ctx, cursor == len, max_payload, &ctx->rtp_stats) != nal_len) {
			/* Leave a sequence gap so receivers see the loss, and do
			 * not append later NALs to the incomplete access unit. */
			ctx->rtp.seq++;
			break;
		}
	}
	ctx->rtp.timestamp += ctx->frame_ticks;
}

/* One pass over the access unit: cache its parameter sets (wave5 emits them
 * only on an IDR) and report whether it is one. */
static int ar9311_scan_frame(Ar9311RunnerContext *ctx, const uint8_t *frame,
	size_t len)
{
	size_t cursor = 0;
	const uint8_t *nal;
	size_t nal_len;
	int idr = 0;

	while (h26x_util_annexb_next(frame, len, &cursor, &nal, &nal_len)) {
		uint8_t type = h26x_util_hevc_nalu_type(nal, nal_len);

		h26x_param_sets_update(&ctx->param_sets, PT_H265, type, nal,
			nal_len);
		if (type == 19 || type == 20)
			idr = 1;
	}
	return idr;
}

/* The access unit with VPS/SPS/PPS in front, for the frame-shm ring, whose
 * consumer forwards whole units.  NULL when the sets are not known yet or
 * the buffer cannot grow; the caller then writes the unit as it is. */
static const uint8_t *ar9311_with_param_sets(Ar9311RunnerContext *ctx,
	const uint8_t *frame, size_t len, size_t *out_len)
{
	static const uint8_t start[4] = { 0, 0, 0, 1 };
	H26xParamSetRef refs[3];
	size_t n, i, need = len;
	uint8_t *p;

	n = h26x_param_sets_get_prepend(&ctx->param_sets, PT_H265, 19, refs, 3);
	if (n == 0)
		return NULL;
	for (i = 0; i < n; i++)
		need += sizeof(start) + refs[i].len;
	if (need > ctx->wave_buf_len) {
		p = realloc(ctx->wave_buf, need);
		if (!p)
			return NULL;
		ctx->wave_buf = p;
		ctx->wave_buf_len = need;
	}
	p = ctx->wave_buf;
	for (i = 0; i < n; i++) {
		memcpy(p, start, sizeof(start));
		memcpy(p + sizeof(start), refs[i].data, refs[i].len);
		p += sizeof(start) + refs[i].len;
	}
	memcpy(p, frame, len);
	*out_len = need;
	return ctx->wave_buf;
}

/* Frame metadata and wave position.  The wave restarts at an IDR; every
 * other frame is one band further.  Returns 1 for the first frame of a wave
 * that is not itself an IDR. */
static int ar9311_gdr_step(Ar9311RunnerContext *ctx, int idr,
	VencFrameMeta *meta)
{
	int head;

	meta->flags = idr ? VENC_FRAME_FLAG_IDR : 0;
	if (idr || !ctx->gdr_active || ctx->gdr_cycle_len == 0) {
		ctx->gdr_counter = 0;
		return 0;
	}
	head = ctx->gdr_counter == 0;
	meta->flags |= VENC_FRAME_FLAG_GDR;
	meta->gdr_pos = ctx->gdr_counter;
	meta->gdr_len = ctx->gdr_cycle_len;
	if (++ctx->gdr_counter >= ctx->gdr_cycle_len)
		ctx->gdr_counter = 0;
	return head;
}

/* Every frame-shm producer measures the ring's low water: the ring
 * publishes the gauge unconditionally, so an unmeasured one would read as
 * a perfectly drained consumer. */
static void ar9311_observe_transport(Ar9311RunnerContext *ctx)
{
	venc_frame_ring_fill_t ring;
	uint8_t fill = 0;
	int p_state, now_us_ok;
	uint32_t p_frames;
	uint64_t now_us;

	now_us_ok = ar9311_transport_fill(ctx, &ring, &fill) == 0;
	if (ctx->frame_ring && now_us_ok) {
		now_us = wb_monotonic_us();
		if (!ctx->low_water_ready) {
			venc_ring_low_water_reset(&ctx->low_water, now_us);
			ctx->low_water_ready = 1;
		}
		venc_ring_low_water_observe(&ctx->low_water, ring.used_slots,
			ring.slot_count);
		if (venc_ring_low_water_tick(&ctx->low_water, now_us))
			venc_frame_ring_set_low_water(ctx->frame_ring,
				venc_ring_low_water_slots(&ctx->low_water));
	}
	if (!now_us_ok)
		return;
	p_state = __atomic_load_n(&ctx->pressure_state, __ATOMIC_RELAXED);
	p_frames = __atomic_load_n(&ctx->pressure_frames, __ATOMIC_RELAXED);
	venc_observe_pressure(fill, &p_state, &p_frames);
	__atomic_store_n(&ctx->pressure_state, p_state, __ATOMIC_RELAXED);
	__atomic_store_n(&ctx->pressure_frames, p_frames, __ATOMIC_RELAXED);
}

static void ar9311_publish_frame(Ar9311RunnerContext *ctx,
	const uint8_t *frame, size_t len, uint64_t capture_us)
{
	VencFrameMeta meta;
	uint64_t frames;
	int head;

	memset(&meta, 0, sizeof(meta));
	meta.pts = (uint32_t)capture_us;
	meta.codec = VENC_FRAME_CODEC_H265;
	/* Wave bookkeeping runs every frame, sent or not, so the position stays
	 * true to the encoder across an output disable. */
	head = ar9311_gdr_step(ctx, ar9311_scan_frame(ctx, frame, len), &meta);
	ar9311_transport_begin_frame(ctx);
	if (ctx->tx.output_enabled && ctx->frame_ring) {
		const uint8_t *out = frame;
		size_t out_len = len;

		if (head) {
			const uint8_t *with = ar9311_with_param_sets(ctx, frame, len,
				&out_len);

			if (with)
				out = with;
			else
				out_len = len;
		}
		if (venc_frame_ring_write(ctx->frame_ring, &meta, out,
			(uint32_t)out_len) != 0)
			__atomic_add_fetch(&ctx->output_drops, 1, __ATOMIC_RELAXED);
	} else if (ctx->tx.output_enabled && ctx->tx.socket_handle >= 0) {
		ar9311_send_rtp_frame(ctx, frame, len, head);
	}
	ar9311_observe_transport(ctx);
	frames = __atomic_add_fetch(&ctx->frames, 1, __ATOMIC_RELAXED);
	__atomic_add_fetch(&ctx->bytes, len, __ATOMIC_RELAXED);
	pipeline_common_rate_watch(&ctx->rate_watch, &ctx->config,
		(uint32_t)len, wb_monotonic_us());
	if (__atomic_load_n(&ctx->verbose, __ATOMIC_ACQUIRE) &&
	    frames % AR9311_FPS == 0)
		printf("> AR9311 frames=%llu bytes=%llu camera_skips=%llu "
			"output_drops=%llu\n", (unsigned long long)frames,
			(unsigned long long)ctx->bytes,
			(unsigned long long)ctx->camera_skips,
			(unsigned long long)ctx->output_drops);
}

static int ar9311_output_start(Ar9311RunnerContext *ctx)
{
	RtpSessionState session;

	ctx->output_enabled = ctx->config.outgoing.enabled ? 1 : 0;
	/* Seeded for every transport and before the disabled bail, so an
	 * output enabled later over HTTP does not send ssrc 0 with a frozen
	 * timestamp. */
	rtp_session_init(&session, rtp_session_payload_type(PT_H265), AR9311_FPS);
	ctx->rtp.seq = session.seq;
	ctx->rtp.timestamp = session.timestamp;
	ctx->rtp.ssrc = session.ssrc;
	ctx->rtp.payload_type = session.payload_type;
	ctx->frame_ticks = session.frame_ticks;
	if (!ctx->config.outgoing.enabled)
		return 0;
	snprintf(ctx->applied_server, sizeof(ctx->applied_server), "%s",
		ctx->config.outgoing.server);
	if (ctx->output_uri.type == VENC_OUTPUT_URI_FRAME_SHM) {
		ctx->frame_ring = venc_frame_ring_create(ctx->output_uri.endpoint,
			AR9311_FRAME_RING_SLOTS, AR9311_FRAME_RING_BYTES);
		return ctx->frame_ring ? 0 : -1;
	}
	/* Non-fatal at start: an output that cannot come up leaves the craft
	 * reachable over HTTP, where outgoing.server is live-settable. */
	if (output_socket_configure(&ctx->socket_handle, &ctx->destination,
		&ctx->destination_len, &ctx->transport, &ctx->output_uri,
		ctx->config.outgoing.connected_udp,
		ctx->config.outgoing.allow_unix_encoder_stall,
		&ctx->connected_udp) != 0) {
		fprintf(stderr, "ERROR: outgoing.server %s could not be brought up; "
			"starting with NO video output -- set outgoing.server over "
			"the API to recover\n", ctx->config.outgoing.server);
		ctx->applied_server[0] = '\0';
		return 0;
	}
	(void)output_socket_capture_capacity(ctx->socket_handle,
		&ctx->send_queue);
	return 0;
}

static void ar9311_output_stop(Ar9311RunnerContext *ctx)
{
	if (ctx->frame_ring) {
		venc_frame_ring_destroy(ctx->frame_ring);
		ctx->frame_ring = NULL;
	}
	if (ctx->socket_handle >= 0) {
		close(ctx->socket_handle);
		ctx->socket_handle = -1;
	}
}

/* ── Lifecycle ───────────────────────────────────────────────────────── */

static VencConfig *ar9311_config(void *opaque)
{
	return &((Ar9311RunnerContext *)opaque)->config;
}

static int ar9311_prepare(void *opaque)
{
	Ar9311RunnerContext *ctx = opaque;
	const char *err;

	setvbuf(stdout, NULL, _IONBF, 0);
	ctx->socket_handle = -1;
	err = ar9311_validate_config(&ctx->config);
	if (err) {
		fprintf(stderr, "ERROR: %s\n", err);
		return 1;
	}
	if (ctx->config.outgoing.enabled &&
	    venc_config_parse_output_uri(ctx->config.outgoing.server,
		&ctx->output_uri) != 0) {
		fprintf(stderr, "ERROR: invalid AR9311 output URI: %s\n",
			ctx->config.outgoing.server);
		return 1;
	}
	venc_api_set_sensor_info(0, 0, ctx->config.sensor.index);
	ctx->live_bitrate = ctx->config.video0.bitrate;
	ctx->max_payload_size = ctx->config.outgoing.max_payload_size;
	ctx->verbose = ctx->config.system.verbose ? 1 : 0;
	printf("> AR9311 backend selected (V4L2 camera + wave5, first slice)\n");
	/* 0 and 1 are both one picture */
	ctx->strips = ctx->config.video0.strips >= 2 ?
		(int)ctx->config.video0.strips : 0;
	return 0;
}

/* Map the resilience preset's intra mode onto wave5's cyclic refresh.  The
 * shared intra_refresh_compute() gives the rows per picture for the mode's
 * heal window, in 32-line rows; wave5 refreshes 64-line rows and takes a
 * period instead (ar9311_encoder_refresh_plan()).  No stripe QP: wave5 has
 * no such control, so intraRefreshQp is not read. */
static void ar9311_plan_refresh(Ar9311RunnerContext *ctx,
	Ar9311EncoderParams *params)
{
	Ar9311IntraRefreshStatus st;
	Ar9311RefreshPlan plan;
	IntraRefreshDerived d;
	IntraRefreshMode mode;

	memset(&plan, 0, sizeof(plan));
	mode = intra_refresh_parse_mode(ctx->config.video0.intra_refresh_mode);
	intra_refresh_compute(mode, AR9311_HEIGHT, AR9311_FPS,
		ctx->config.video0.intra_refresh_lines, 0,
		ctx->config.video0.gop_size, &d);
	if (mode != INTRA_MODE_OFF && d.lines > 0 && d.total_rows > 0) {
		ar9311_encoder_refresh_plan(AR9311_HEIGHT, d.lines, &plan);
		ctx->gdr_active = 1;
		ctx->gdr_cycle_len = (uint8_t)plan.cycle;
	}
	params->refresh_period = plan.period;

	memset(&st, 0, sizeof(st));
	snprintf(st.mode_name, sizeof(st.mode_name), "%s",
		intra_refresh_mode_name(mode));
	st.active = plan.period > 0;
	st.mi_supported = 1;
	st.apply_ok = 1;
	st.target_ms = d.target_ms;
	st.total_rows = plan.total_rows;
	st.requested_lines = ctx->config.video0.intra_refresh_lines;
	st.effective_lines_per_p = plan.rows;
	st.lines_clamped = d.lines_clamped;
	st.explicit_gop_sec = ctx->config.video0.gop_size;
	/* With refresh on there is no periodic IDR at all. */
	st.effective_gop_sec = plan.period ? 0.0 : ctx->config.video0.gop_size;
	pthread_mutex_lock(&g_ar9311_status_mutex);
	g_ar9311_intra_status = st;
	pthread_mutex_unlock(&g_ar9311_status_mutex);
}

/* Sub-frame sync, decided before the camera opens: it reads the CVISP depth
 * at REQBUFS.  On needs both builder kernel patches (wave5 sub-frame sync,
 * CVISP depth per stream) and runs the camera at depth 1, where a buffer
 * comes back as the write starts; off runs depth 2, where it comes back
 * written.  Depth 1 without sub-frame sync would encode torn frames. */
/* image.mirror / image.flip.  A sensor driver with an orientation parameter
 * (bit 0 mirror, bit 1 flip; both P1 sensors) turns the readout itself, and
 * the frame still arrives top-down, so strips and sub-frame sync keep
 * working.  Where that moves the Bayer phase (the NT99235) the driver
 * reports the new order and the ISP is told to re-read it; the ISP takes it
 * at chain start anyway, so -EPIPE before then is fine.  Written every start,
 * so a cleared setting also undoes an earlier one.  Without the parameter the
 * encoder turns the picture, and a flip there turns sub-frame sync off
 * (ar9311_plan_sfs). */
#define AR9311_ISP_BAYER_PATH "/sys/kernel/debug/ar-isp/bayer"

static void ar9311_plan_orientation(Ar9311RunnerContext *ctx,
	Ar9311EncoderParams *params)
{
	int mirror = ctx->config.image.mirror ? 1 : 0;
	int flip = ctx->config.image.flip ? 1 : 0;
	const char *sensor = ar9311_sensor_name();
	char path[96];
	FILE *f;
	int ok;

	params->mirror = mirror;
	params->flip = flip;
	if (!sensor)
		return;
	snprintf(path, sizeof(path), "/sys/module/%s/parameters/orientation",
		sensor);
	f = fopen(path, "w");
	if (!f)
		return;   /* no such parameter: the encoder turns it */
	ok = fprintf(f, "%d\n", mirror | flip << 1) > 0;
	if (fclose(f) != 0)
		ok = 0;
	if (ok) {
		params->mirror = 0;
		params->flip = 0;
		f = fopen(AR9311_ISP_BAYER_PATH, "w");
		if (f) {
			fputs("1\n", f);
			fclose(f);
		}
		if (mirror || flip)
			printf("> [ar9311] picture %s by the %s\n",
				mirror && flip ? "turned 180 degrees" :
				mirror ? "mirrored" : "flipped", sensor);
		return;
	}
	fprintf(stderr, "WARNING: [ar9311] %s: %s; turning the picture in the "
		"encoder\n", path, strerror(errno));
}

static void ar9311_plan_sfs(Ar9311RunnerContext *ctx,
	Ar9311EncoderParams *params)
{
	int on = ctx->config.video0.sub_frame_sync;

	ctx->pts_offset_us = 0;
	if (on && params->flip) {
		fprintf(stderr, "WARNING: [ar9311] image.flip turns the picture in "
			"the encoder, which reads the frame bottom-up; sub-frame "
			"sync and strips cannot follow that, running without\n");
		on = 0;
	}
	if (on && !ar9311_encoder_has_sfs()) {
		fprintf(stderr, "WARNING: [ar9311] video0.subFrameSync: this "
			"kernel's wave5 has no sub-frame sync (builder patch "
			"0004-media-wave5-sub-frame-sync); running without\n");
		on = 0;
	}
	if (ar9311_capture_set_depth(on ? 1 : 2) != 0 && on) {
		fprintf(stderr, "WARNING: [ar9311] video0.subFrameSync: the CVISP "
			"depth cannot be set (builder patch 0005-media-artosyn-"
			"cvisp-depth-per-stream); running without\n");
		on = 0;
	}
	if (!on)
		return;
	ar9311_encoder_sfs_schedule((uint32_t)ar9311_capture_depth(), AR9311_FPS,
		&params->sfs_start_us, &params->sfs_row_us);
	ctx->pts_offset_us = 1000000u / AR9311_FPS;
	printf("> [ar9311] sub-frame sync: CVISP depth %d, band 0 at +%d us, "
		"%u us per %u lines\n", ar9311_capture_depth(),
		(int)params->sfs_start_us, params->sfs_row_us,
		AR9311_SFS_BAND_LINES);
}


/* ── Strips (video0.strips, ar9311_strips.h) ──────────────────────────── */

/* Where the camera's last line lands after its timestamp at depth 1
 * (measured, see AR9311_SFS_*), to report part times against. */
#define AR9311_STRIP_LAST_LINE_US 15962u

static int ar9311_strips_start(Ar9311RunnerContext *ctx,
	const Ar9311EncoderParams *params)
{
	uint32_t overlap = ctx->config.video0.strip_overlap;
	uint32_t share[AR9311_STRIPS_MAX];
	int n = ctx->strips, k;

	/* Without sub-frame sync the camera hands a frame back only once it
	 * is written, so nothing would overlap; the strips' parts also need
	 * the frame-shm ring (an RTP sender publishes whole units). */
	if (!params->sfs_row_us || ar9311_capture_depth() != 1 ||
	    ctx->output_uri.type != VENC_OUTPUT_URI_FRAME_SHM ||
	    ar9311_strips_plan(n, AR9311_HEIGHT, overlap, params->sfs_start_us,
		params->sfs_row_us, &ctx->plan) != 0) {
		fprintf(stderr, "WARNING: [ar9311] video0.strips needs sub-frame "
			"sync (CVISP depth 1), a frame-shm output and a "
			"video0.stripOverlap the strips can take; encoding one "
			"picture\n");
		ctx->strips = 0;
		if (ar9311_capture_open(&ctx->capture, AR9311_WIDTH,
			AR9311_HEIGHT) != 0 ||
		    ar9311_encoder_open(&ctx->encoder, &ctx->capture.geo,
			ctx->capture.buf_count, params) != 0 ||
		    ar9311_encoder_start(&ctx->encoder) != 0 ||
		    ar9311_capture_start(&ctx->capture) != 0)
			return -1;
		return 0;
	}
	if (ar9311_capture_open(&ctx->capture, AR9311_WIDTH, AR9311_HEIGHT) != 0)
		return -1;
	ctx->se[0] = &ctx->encoder;
	for (k = 1; k < n; k++)
		ctx->se[k] = &ctx->strip_enc[k - 1];
	ar9311_strips_split(&ctx->plan, params->bitrate_kbps, share);
	if (overlap)
		printf("> [ar9311] strips: %u row(s) of 64 lines past each seam, "
			"sliced off and dropped; loop filter across slice edges\n",
			overlap);
	for (k = 0; k < n; k++) {
		Ar9311EncoderParams p = *params;
		Ar9311Geometry g = ctx->capture.geo;

		g.height = ctx->plan.enc_height[k];
		/* Without overlap each seam is a slice boundary the decoder
		 * must not filter across: each encoder saw it as its
		 * picture's edge.  With it the encoders filter their own
		 * slice edges and the decoder the seam, with pixels both
		 * encoders had close copies of. */
		p.no_lf_across_slices = !overlap;
		p.slice_ctus = ctx->plan.slice_rows[k] *
			((AR9311_WIDTH + AR9311_STRIPS_ROW_LINES - 1) /
				AR9311_STRIPS_ROW_LINES);
		p.bitrate_kbps = share[k];
		if (k < n - 1) {
			p.sfs_row_us = 0;
			p.sfs_start_us = 0;
		} else {
			p.sfs_start_us = ctx->plan.sfs_start_us;
		}
		if (ar9311_encoder_open(ctx->se[k], &g, ctx->capture.buf_count,
			&p) != 0)
			return -1;
		printf("> [ar9311] strip %d: lines %u..%u (encodes %u..%u), %u "
			"kbps, %s %+d us\n", k,
			ctx->plan.y0[k], ctx->plan.y0[k] + ctx->plan.height[k] - 1,
			ctx->plan.enc_y0[k],
			ctx->plan.enc_y0[k] + ctx->plan.enc_height[k] - 1,
			p.bitrate_kbps, k < n - 1 ? "queued at" : "sub-frame sync from",
			k < n - 1 ? (int)ctx->plan.due_us[k] : (int)p.sfs_start_us);
	}
	ar9311_merge_init(&ctx->merge, n, ctx->plan.enc_height,
		ctx->plan.overlap_rows);
	ctx->strip_cap = 1u << 20;
	ctx->strip_buf = malloc(ctx->strip_cap);
	if (!ctx->strip_buf)
		return -1;
	for (k = 0; k < n; k++)
		if (ar9311_encoder_start(ctx->se[k]) != 0)
			return -1;
	return ar9311_capture_start(&ctx->capture) != 0 ? -1 : 0;
}

/* Part @p k of the open frame onto the ring: all but the last flagged
 * partial, all with the first part's metadata. */
static void ar9311_strips_publish(Ar9311RunnerContext *ctx, int k,
	const uint8_t *data, size_t len, uint64_t capture_us)
{
	int last = k == ctx->strips - 1;
	VencFrameMeta meta;
	const uint8_t *out = data;
	size_t out_len = len;

	if (k == 0) {
		int head;

		memset(&meta, 0, sizeof(meta));
		meta.pts = (uint32_t)capture_us;
		meta.codec = VENC_FRAME_CODEC_H265;
		head = ar9311_gdr_step(ctx, ar9311_scan_frame(ctx, data, len),
			&meta);
		ctx->strip_meta = meta;
		if (head) {
			const uint8_t *with = ar9311_with_param_sets(ctx, data, len,
				&out_len);

			if (with)
				out = with;
			else
				out_len = len;
		}
	} else {
		meta = ctx->strip_meta;
	}
	if (!last)
		meta.flags |= VENC_FRAME_FLAG_PARTIAL;
	ar9311_transport_begin_frame(ctx);
	if (ctx->tx.output_enabled && ctx->frame_ring &&
	    venc_frame_ring_write(ctx->frame_ring, &meta, out,
		(uint32_t)out_len) != 0)
		__atomic_add_fetch(&ctx->output_drops, 1, __ATOMIC_RELAXED);
	__atomic_add_fetch(&ctx->bytes, len, __ATOMIC_RELAXED);
	if (!last)
		return;
	ar9311_observe_transport(ctx);
	__atomic_add_fetch(&ctx->frames, 1, __ATOMIC_RELAXED);
	pipeline_common_rate_watch(&ctx->rate_watch, &ctx->config,
		(uint32_t)len, wb_monotonic_us());
}

static int ar9311_strips_grow(Ar9311RunnerContext *ctx, size_t len)
{
	uint8_t *p;

	if (len + 4096 <= ctx->strip_cap)
		return 0;
	p = realloc(ctx->strip_buf, len + 4096);
	if (!p)
		return -1;
	ctx->strip_buf = p;
	ctx->strip_cap = len + 4096;
	return 0;
}

/* Give up the open frame (its published parts end incomplete: the
 * consumer drops them when the next pts arrives) and ask for an IDR. */
static void ar9311_strips_fail(Ar9311RunnerContext *ctx, uint64_t *counter,
	const char *what)
{
	if ((*counter)++ < 10 || *counter % 600 == 0)
		fprintf(stderr, "WARN: [ar9311] strips: %s (#%llu)\n", what,
			(unsigned long long)*counter);
	ctx->strip_next = 0;
	__atomic_store_n(&ctx->strip_idr_pending, 1, __ATOMIC_RELEASE);
}

static int ar9311_strips_take(Ar9311RunnerContext *ctx, int k,
	const uint8_t *data, size_t len, uint64_t ts)
{
	int last = k == ctx->strips - 1;
	uint64_t now;
	size_t n;

	if (k == 0) {
		if (ctx->strip_next != 0)
			ctx->strip_abandoned++;	/* the open frame lost its end */
		ctx->strip_next = 0;
	} else if (ctx->strip_next != k || ts != ctx->strip_cur_ts) {
		if (ctx->strip_next != 0)
			ctx->strip_abandoned++;
		ctx->strip_next = 0;	/* the rest of a given-up frame */
		return 0;
	}
	if (!len) {
		ar9311_strips_fail(ctx, &ctx->strip_merge_errors,
			"encoder returned an empty strip");
		return 0;
	}
	if (ar9311_strips_grow(ctx, len) != 0)
		return -1;
	n = ar9311_merge_strip(&ctx->merge, k, data, len, ctx->strip_buf,
		ctx->strip_cap);
	if (!n) {
		ar9311_strips_fail(ctx, &ctx->strip_merge_errors, ctx->merge.error);
		return 0;
	}
	if (last && !ar9311_merge_in_step(&ctx->merge)) {
		ar9311_strips_fail(ctx, &ctx->strip_out_of_step,
			"strips out of step");
		return 0;
	}
	ar9311_strips_publish(ctx, k, ctx->strip_buf, n,
		ts + ctx->pts_offset_us);
	now = wb_monotonic_us();
	if (k == 0) {
		ctx->strip_cur_ts = ts;
		ctx->strip_t_first_sum += now - ts;
	}
	ctx->strip_next = k + 1;
	if (last) {
		ctx->strip_t_whole_sum += now - ts;
		ctx->strip_t_n++;
		ctx->last_frame_us = now;
		ctx->strip_next = 0;
	}
	return 0;
}

/* Every finished strip, in frame order: one access unit is held per
 * encoder and the oldest (frame, then strip) goes first. */
static int ar9311_strips_drain(Ar9311RunnerContext *ctx)
{
	for (;;) {
		int k, pick = -1;

		for (k = 0; k < ctx->strips; k++) {
			if (!ctx->strip_au[k].have) {
				int r = ar9311_encoder_dequeue_stream(ctx->se[k],
					&ctx->strip_au[k].slot, &ctx->strip_au[k].data,
					&ctx->strip_au[k].len, &ctx->strip_au[k].ts);

				if (r < 0)
					return -1;
				ctx->strip_au[k].have = r == 0;
			}
			if (ctx->strip_au[k].have && (pick < 0 ||
			    ctx->strip_au[k].ts < ctx->strip_au[pick].ts))
				pick = k;
		}
		if (pick < 0)
			return 0;
		if (ar9311_strips_take(ctx, pick, ctx->strip_au[pick].data,
			ctx->strip_au[pick].len, ctx->strip_au[pick].ts) != 0)
			return -1;
		ctx->strip_au[pick].have = 0;
		if (ar9311_encoder_release_stream(ctx->se[pick],
			ctx->strip_au[pick].slot) != 0)
			return -1;
	}
}

static int ar9311_strips_reclaim(Ar9311RunnerContext *ctx)
{
	int k, index, ret;

	for (k = 0; k < ctx->strips; k++) {
		while ((ret = ar9311_encoder_reclaim_source(ctx->se[k], &index))
			== 0) {
			if (index < 0 || index >= AR9311_CAPTURE_MAX_BUFS)
				continue;
			if (++ctx->strip_frame[index].reclaimed < ctx->strips)
				continue;
			ctx->strip_frame[index].used = 0;
			ctx->held--;
			if (ar9311_capture_requeue(&ctx->capture, index) != 0)
				return -1;
		}
		if (ret < 0)
			return -1;
	}
	return 0;
}

static int ar9311_strips_feed(Ar9311RunnerContext *ctx)
{
	uint64_t ts;
	int index, corrupt, ret;

	while ((ret = ar9311_capture_dequeue(&ctx->capture, &index, &corrupt,
		&ts)) == 0) {
		if (corrupt || ctx->held >= ctx->capture.max_held ||
		    index >= AR9311_CAPTURE_MAX_BUFS) {
			__atomic_add_fetch(&ctx->camera_skips, 1, __ATOMIC_RELAXED);
			if (ar9311_capture_requeue(&ctx->capture, index) != 0)
				return -1;
			continue;
		}
		memset(&ctx->strip_frame[index], 0, sizeof(ctx->strip_frame[0]));
		ctx->strip_frame[index].used = 1;
		ctx->strip_frame[index].index = index;
		ctx->strip_frame[index].ts = ts;
		ctx->held++;
	}
	return ret < 0 ? -1 : 0;
}

/* Queue every strip that is due, oldest frame first; *next gets the next
 * due time (0 = nothing waiting). */
static int ar9311_strips_issue(Ar9311RunnerContext *ctx, uint64_t now,
	uint64_t *next)
{
	*next = 0;
	for (;;) {
		int i, k, pick = -1;

		for (i = 0; i < AR9311_CAPTURE_MAX_BUFS; i++)
			if (ctx->strip_frame[i].used &&
			    ctx->strip_frame[i].queued < ctx->strips &&
			    (pick < 0 || ctx->strip_frame[i].ts <
				ctx->strip_frame[pick].ts))
				pick = i;
		if (pick < 0)
			return 0;
		while ((k = ctx->strip_frame[pick].queued) < ctx->strips) {
			uint64_t due = ctx->plan.due_us[k] ?
				ctx->strip_frame[pick].ts + ctx->plan.due_us[k] : 0;

			if (due && now < due) {
				*next = due;
				return 0;
			}
			/* every strip turns IDR on the same frame */
			if (k == 0 && __atomic_exchange_n(&ctx->strip_idr_pending, 0,
				__ATOMIC_ACQ_REL)) {
				for (i = 0; i < ctx->strips; i++)
					if (ar9311_encoder_request_idr(ctx->se[i]) != 0)
						fprintf(stderr, "WARN: [ar9311] strips: IDR "
							"request failed\n");
			}
			if (ar9311_encoder_queue_source_rows(ctx->se[k], pick,
				ctx->capture.dmabuf[pick], ctx->strip_frame[pick].ts,
				ctx->plan.enc_y0[k]) != 0)
				return -1;
			ctx->strip_frame[pick].queued++;
		}
	}
}

static void ar9311_strips_report(Ar9311RunnerContext *ctx, uint64_t now)
{
	if (now - ctx->strip_last_report_us < 10000000ull)
		return;
	if (ctx->strip_last_report_us && ctx->strip_t_n)
		printf("> [ar9311] strips: %llu pictures in 10 s, first part ready "
			"%+.2f ms, whole picture %+.2f ms after the camera's last "
			"line (avg); merge errors %llu, out of step %llu, "
			"abandoned %llu\n",
			(unsigned long long)ctx->strip_t_n,
			(double)ctx->strip_t_first_sum / (double)ctx->strip_t_n /
				1000.0 - AR9311_STRIP_LAST_LINE_US / 1000.0,
			(double)ctx->strip_t_whole_sum / (double)ctx->strip_t_n /
				1000.0 - AR9311_STRIP_LAST_LINE_US / 1000.0,
			(unsigned long long)ctx->strip_merge_errors,
			(unsigned long long)ctx->strip_out_of_step,
			(unsigned long long)ctx->strip_abandoned);
	ctx->strip_t_first_sum = ctx->strip_t_whole_sum = ctx->strip_t_n = 0;
	ctx->strip_last_report_us = now;
}

static int ar9311_run_strips(Ar9311RunnerContext *ctx)
{
	struct pollfd pfd[2 + AR9311_STRIPS_MAX];
	int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
	int rc = 0, k, nfd;

	if (tfd < 0) {
		perror("[ar9311] timerfd");
		return -1;
	}
	ctx->last_frame_us = wb_monotonic_us();
	while (!g_ar9311_stop) {
		struct itimerspec its;
		uint64_t now = wb_monotonic_us(), next;
		uint64_t expirations;

		if (venc_api_get_reinit()) {
			venc_api_clear_reinit();
			venc_respawn_request();
			break;
		}
		if (ar9311_strips_issue(ctx, now, &next) != 0) {
			rc = -1;
			break;
		}
		memset(&its, 0, sizeof(its));
		if (next) {
			its.it_value.tv_sec = (time_t)(next / 1000000u);
			its.it_value.tv_nsec = (long)(next % 1000000u) * 1000;
		}
		(void)timerfd_settime(tfd, TFD_TIMER_ABSTIME, &its, NULL);
		nfd = 0;
		pfd[nfd].fd = ctx->capture.fd;
		pfd[nfd++].events = POLLIN;
		pfd[nfd].fd = tfd;
		pfd[nfd++].events = POLLIN;
		for (k = 0; k < ctx->strips; k++) {
			pfd[nfd].fd = ctx->se[k]->fd;
			pfd[nfd++].events = POLLIN | POLLOUT;
		}
		if (poll(pfd, (nfds_t)nfd, 1000) < 0) {
			if (errno == EINTR)
				continue;
			rc = -1;
			break;
		}
		(void)!read(tfd, &expirations, sizeof(expirations));
		if (ar9311_strips_drain(ctx) != 0 ||
		    ar9311_strips_reclaim(ctx) != 0 ||
		    ar9311_strips_feed(ctx) != 0) {
			rc = -1;
			break;
		}
		now = wb_monotonic_us();
		ar9311_strips_report(ctx, now);
		if (now - ctx->last_frame_us > AR9311_STALL_MS * 1000ull) {
			fprintf(stderr, "ERROR: [ar9311] no encoded frame for %d ms: "
				"camera or encoder stalled\n", AR9311_STALL_MS);
			rc = -1;
			break;
		}
	}
	close(tfd);
	return rc;
}

static void ar9311_strips_teardown(Ar9311RunnerContext *ctx)
{
	int i, k;

	for (i = 0; i < 10 && ctx->held > 0; i++) {
		struct pollfd pfd[AR9311_STRIPS_MAX];

		for (k = 0; k < ctx->strips; k++) {
			pfd[k].fd = ctx->se[k]->fd;
			pfd[k].events = POLLIN | POLLOUT;
		}
		if (poll(pfd, (nfds_t)ctx->strips, 50) <= 0)
			continue;
		if (ar9311_strips_drain(ctx) != 0 ||
		    ar9311_strips_reclaim(ctx) != 0)
			break;
	}
	if (ctx->held > 0)
		fprintf(stderr, "WARN: [ar9311] strips: %d frame(s) still held at "
			"shutdown\n", ctx->held);
	for (k = 0; k < ctx->strips; k++)
		if (ctx->strip_au[k].have)
			(void)ar9311_encoder_release_stream(ctx->se[k],
				ctx->strip_au[k].slot);
}

static int ar9311_pipeline_start(Ar9311RunnerContext *ctx)
{
	Ar9311EncoderParams params;

	memset(&params, 0, sizeof(params));
	params.fps = AR9311_FPS;
	params.bitrate_kbps = ctx->config.video0.bitrate;
	params.gop_frames = pipeline_common_gop_frames(
		ctx->config.video0.gop_size, AR9311_FPS);
	params.min_qp = ctx->config.video0.min_qp;
	params.max_qp = ctx->config.video0.max_qp;
	ar9311_plan_orientation(ctx, &params);
	ar9311_plan_refresh(ctx, &params);
	ar9311_plan_sfs(ctx, &params);
	if (ctx->strips)
		return ar9311_strips_start(ctx, &params);
	if (ar9311_capture_open(&ctx->capture, AR9311_WIDTH, AR9311_HEIGHT) != 0)
		return -1;
	if (ar9311_encoder_open(&ctx->encoder, &ctx->capture.geo,
		ctx->capture.buf_count, &params) != 0)
		return -1;
	/* Encoder first: the camera starts filling buffers the moment it
	 * streams, and each one goes straight to the encoder. */
	if (ar9311_encoder_start(&ctx->encoder) != 0 ||
	    ar9311_capture_start(&ctx->capture) != 0)
		return -1;
	return 0;
}

static int ar9311_init(void *opaque)
{
	Ar9311RunnerContext *ctx = opaque;
	struct sigaction action;

	memset(&action, 0, sizeof(action));
	action.sa_handler = ar9311_signal_handler;
	sigemptyset(&action.sa_mask);
	(void)sigaction(SIGINT, &action, NULL);
	(void)sigaction(SIGTERM, &action, NULL);
	if (ar9311_pipeline_start(ctx) != 0)
		return -1;
	{
		Ar9311AeLimits lim;

		/* Non-fatal: without AE the camera keeps its fixed exposure. */
		ar9311_ae_limits_from(&ctx->config.isp, &lim);
		ar9311_ae_set_gain_hook(ar9311_picture_set_gain);
		ctx->ae = ar9311_ae_start(&lim, ctx->config.isp.ae_fps);
		/* The cm stage lowers colour with the exposure itself; the
		 * picture's own gain ramp would take it twice. */
		ar9311_picture_set_gain_ramp(!ar9311_ae_drives_cm(ctx->ae));
	}
	/* Non-fatal: without AWB the gains stay as the camera loader set them. */
	ar9311_awb_begin(ctx);
	/* Non-fatal too: the tuned picture, which also undoes what an earlier
	 * daemon left in the ISP driver's parameters. */
	{
		Ar9311Picture pic;

		pthread_mutex_lock(&g_ar9311_picture_mutex);
		ar9311_iq_neutral(&g_ar9311_picture);
		pic = g_ar9311_picture;
		pthread_mutex_unlock(&g_ar9311_picture_mutex);
		(void)ar9311_picture_apply(pic.saturation, pic.contrast,
			pic.sharpness);
	}
	if (ar9311_output_start(ctx) != 0)
		return -1;
	g_ar9311_runner = ctx;
	if (venc_api_register(&ctx->config, "ar9311", &g_ar9311_apply_callbacks,
		NULL) != 0)
		return -1;
	if (venc_httpd_start(ctx->config.system.web_port) != 0)
		return -1;
	return 0;
}

/* Take back every source buffer the encoder has finished reading. */
static int ar9311_reclaim_sources(Ar9311RunnerContext *ctx)
{
	int index, ret;

	while ((ret = ar9311_encoder_reclaim_source(&ctx->encoder, &index)) == 0) {
		ctx->held--;
		if (ar9311_capture_requeue(&ctx->capture, index) != 0)
			return -1;
	}
	return ret < 0 ? -1 : 0;
}

/* Hand every filled camera buffer to the encoder, as long as the camera
 * keeps enough queued under it; a frame beyond that is skipped (requeued
 * unencoded) so the camera never starves. */
static int ar9311_feed_encoder(Ar9311RunnerContext *ctx)
{
	uint64_t capture_us;
	int index, corrupt, ret;

	while ((ret = ar9311_capture_dequeue(&ctx->capture, &index,
		&corrupt, &capture_us)) == 0) {
		if (corrupt || ctx->held >= ctx->capture.max_held) {
			__atomic_add_fetch(&ctx->camera_skips, 1, __ATOMIC_RELAXED);
			if (ar9311_capture_requeue(&ctx->capture, index) != 0)
				return -1;
			continue;
		}
		if (ar9311_encoder_queue_source(&ctx->encoder, index,
			ctx->capture.dmabuf[index], capture_us) != 0)
			return -1;
		ctx->held++;
	}
	return ret < 0 ? -1 : 0;
}

static int ar9311_drain_stream(Ar9311RunnerContext *ctx)
{
	const uint8_t *data;
	uint64_t capture_us;
	size_t len;
	int slot, ret;

	while ((ret = ar9311_encoder_dequeue_stream(&ctx->encoder, &slot, &data,
		&len, &capture_us)) == 0) {
		if (len > 0) {
			ar9311_publish_frame(ctx, data, len,
				capture_us + ctx->pts_offset_us);
			ctx->last_frame_us = wb_monotonic_us();
		}
		if (ar9311_encoder_release_stream(&ctx->encoder, slot) != 0)
			return -1;
	}
	return ret < 0 ? -1 : 0;
}

static int ar9311_run(void *opaque)
{
	Ar9311RunnerContext *ctx = opaque;
	struct pollfd pfd[2];

	if (ctx->strips)
		return ar9311_run_strips(ctx);
	ctx->last_frame_us = wb_monotonic_us();
	while (!g_ar9311_stop) {
		int ready;

		if (venc_api_get_reinit()) {
			venc_api_clear_reinit();
			venc_respawn_request();
			printf("> AR9311 reinit requested: cold restart via fork+exec "
				"(the next wave5 instance of this boot may encode "
				"garbage; reboot if the video breaks)\n");
			break;
		}
		pfd[0].fd = ctx->capture.fd;
		pfd[0].events = POLLIN;
		pfd[1].fd = ctx->encoder.fd;
		pfd[1].events = POLLIN | POLLOUT;
		ready = poll(pfd, 2, 1000);
		if (ready < 0 && errno == EINTR)
			continue;
		if (ready < 0)
			return -1;
		if (ar9311_drain_stream(ctx) != 0 ||
		    ar9311_reclaim_sources(ctx) != 0 ||
		    ar9311_feed_encoder(ctx) != 0)
			return -1;
		if (wb_monotonic_us() - ctx->last_frame_us >
		    AR9311_STALL_MS * 1000ull) {
			fprintf(stderr, "ERROR: [ar9311] no encoded frame for %d ms: "
				"camera or encoder stalled\n", AR9311_STALL_MS);
			return -1;
		}
	}
	return 0;
}

/* Let the encoder finish every frame it holds before anything is stopped.
 * A wave5 job is completed only from its IRQ, and a close landing while one
 * is outstanding waits uninterruptibly under the lock both codec nodes
 * share -- the teardown shape that wedges the VPU for the rest of the boot
 * (MissingLynk ml-cam2enc -A).  Bounded: a job that never completes is
 * reported, not waited on forever. */
static void ar9311_drain_encoder(Ar9311RunnerContext *ctx)
{
	int i;

	for (i = 0; i < 10 && ctx->held > 0; i++) {
		struct pollfd pfd = { .fd = ctx->encoder.fd,
			.events = POLLIN | POLLOUT };

		if (poll(&pfd, 1, 50) <= 0)
			continue;
		if (ar9311_drain_stream(ctx) != 0 ||
		    ar9311_reclaim_sources(ctx) != 0)
			break;
	}
	if (ctx->held > 0)
		fprintf(stderr, "WARN: [ar9311] encoder still holds %d frame(s) at "
			"shutdown\n", ctx->held);
}

static void ar9311_teardown(void *opaque)
{
	Ar9311RunnerContext *ctx = opaque;

	venc_httpd_pause();
	venc_httpd_stop();
	g_ar9311_runner = NULL;
	/* Before the camera stops: the threads read its statistics. */
	ar9311_ae_stop(ctx->ae);
	ctx->ae = NULL;
	ar9311_awb_stop(ctx->awb);
	ctx->awb = NULL;
	if (ctx->strips && ctx->encoder.streaming)
		ar9311_strips_teardown(ctx);
	else if (ctx->encoder.streaming)
		ar9311_drain_encoder(ctx);
	ar9311_output_stop(ctx);
	/* Camera first: it stops writing the buffers the encoder imported. */
	ar9311_capture_close(&ctx->capture);
	ar9311_encoder_close(&ctx->encoder);
	{
		int k;

		for (k = 0; k < AR9311_STRIPS_MAX - 1; k++)
			ar9311_encoder_close(&ctx->strip_enc[k]);
	}
	ar9311_merge_free(&ctx->merge);
	free(ctx->strip_buf);
	ctx->strip_buf = NULL;
	free(ctx->wave_buf);
	ctx->wave_buf = NULL;
	ctx->wave_buf_len = 0;
}

static int ar9311_map_result(int result)
{
	return result == 0 ? 0 : 2;
}

static const BackendOps g_ar9311_ops = {
	.name = "ar9311",
	.context_size = sizeof(Ar9311RunnerContext),
	.config = ar9311_config,
	.prepare = ar9311_prepare,
	.init = ar9311_init,
	.run = ar9311_run,
	.teardown = ar9311_teardown,
	.map_pipeline_result = ar9311_map_result,
};

const BackendOps *ar9311_runtime_backend_ops(void)
{
	return &g_ar9311_ops;
}
