#include "ar9311_runtime.h"

#include "ar9311_ae.h"
#include "ar9311_capture.h"
#include "ar9311_encoder.h"
#include "ar9311_iq.h"
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

static int ar9311_apply_bitrate(uint32_t kbps)
{
	Ar9311RunnerContext *ctx = g_ar9311_runner;

	if (!ctx || ar9311_encoder_set_bitrate(&ctx->encoder, kbps) != 0)
		return -1;
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
 * range, 1 line to one frame period and 1x to 64x. */
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
	return 0;
}

/* Map the resilience preset's intra mode onto wave5's cyclic refresh.  The
 * shared intra_refresh_compute() gives the rows per picture for the mode's
 * heal window; wave5 takes a period instead (it refreshes total/period rows
 * per picture), so pick the period that gives at least those rows.  No
 * stripe QP: wave5 has no such control, so intraRefreshQp is not read. */
static void ar9311_plan_refresh(Ar9311RunnerContext *ctx,
	Ar9311EncoderParams *params)
{
	Ar9311IntraRefreshStatus st;
	IntraRefreshDerived d;
	IntraRefreshMode mode;
	uint32_t period = 0, rows = 0;

	mode = intra_refresh_parse_mode(ctx->config.video0.intra_refresh_mode);
	intra_refresh_compute(mode, AR9311_HEIGHT, AR9311_FPS,
		ctx->config.video0.intra_refresh_lines, 0,
		ctx->config.video0.gop_size, &d);
	if (mode != INTRA_MODE_OFF && d.lines > 0 && d.total_rows > 0) {
		period = d.total_rows / d.lines;
		if (period < 1)
			period = 1;
		rows = d.total_rows / period;
		ctx->gdr_active = 1;
		ctx->gdr_cycle_len = (uint8_t)((d.total_rows + rows - 1) / rows);
	}
	params->refresh_period = period;

	memset(&st, 0, sizeof(st));
	snprintf(st.mode_name, sizeof(st.mode_name), "%s",
		intra_refresh_mode_name(mode));
	st.active = period > 0;
	st.mi_supported = 1;
	st.apply_ok = 1;
	st.target_ms = d.target_ms;
	st.total_rows = d.total_rows;
	st.requested_lines = ctx->config.video0.intra_refresh_lines;
	st.effective_lines_per_p = rows;
	st.lines_clamped = d.lines_clamped;
	st.explicit_gop_sec = ctx->config.video0.gop_size;
	/* With refresh on there is no periodic IDR at all. */
	st.effective_gop_sec = period ? 0.0 : ctx->config.video0.gop_size;
	pthread_mutex_lock(&g_ar9311_status_mutex);
	g_ar9311_intra_status = st;
	pthread_mutex_unlock(&g_ar9311_status_mutex);
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
	params.mirror = ctx->config.image.mirror;
	params.flip = ctx->config.image.flip;
	ar9311_plan_refresh(ctx, &params);
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
		ctx->ae = ar9311_ae_start(&lim, ctx->config.isp.ae_fps);
	}
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
			ar9311_publish_frame(ctx, data, len, capture_us);
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
	/* Before the camera stops: the thread reads its statistics. */
	ar9311_ae_stop(ctx->ae);
	ctx->ae = NULL;
	if (ctx->encoder.streaming)
		ar9311_drain_encoder(ctx);
	ar9311_output_stop(ctx);
	/* Camera first: it stops writing the buffers the encoder imported. */
	ar9311_capture_close(&ctx->capture);
	ar9311_encoder_close(&ctx->encoder);
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
