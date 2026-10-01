#include "ar9311_validation.h"

#include "ar9311_encoder.h"
#include "pipeline_common.h"

#include <math.h>
#include <string.h>

static const char *validate_video(const VencConfig *cfg)
{
	const VencConfigVideo *v = &cfg->video0;

	if (v->width != AR9311_WIDTH || v->height != AR9311_HEIGHT)
		return "AR9311 phase one encodes video0.size=1920x1080 only";
	if (v->fps != AR9311_FPS)
		return "AR9311 phase one runs the camera at video0.fps=60 only";
	if (v->bitrate < VENC_BITRATE_MIN_KBPS ||
	    v->bitrate > VENC_BITRATE_MAX_KBPS)
		return "video0.bitrate is outside the supported range";
	if (!isfinite(v->gop_size) || v->gop_size < 0.0 ||
	    pipeline_common_gop_frames(v->gop_size, AR9311_FPS) >
		AR9311_ENCODER_GOP_MAX)
		return "AR9311 video0.gop_size exceeds the encoder's "
			"2047-frame IDR period";
	if (v->max_qp && v->min_qp > v->max_qp)
		return "video0.min_qp is above video0.max_qp";
	if (v->min_qp > 51 || v->max_qp > 51)
		return "video0.min_qp/max_qp must be in range 0..51";
	if (strcmp(v->rc_mode, "cbr") != 0)
		return "AR9311 phase one supports video0.rc_mode=cbr only";
	/* The intra-refresh presets map onto wave5's cyclic refresh; the
	 * SVC-T ones (ref_base set: rally, range, fpv, ltr) need reference
	 * structures wave5 does not offer. */
	if (v->ref_base != 0)
		return "AR9311 supports video0.resilience off, rescue, quality, "
			"sprint, racing, endurance and patrol (no SVC-T presets)";
	if (strcmp(v->framing, "off") != 0)
		return "AR9311 phase one supports video0.framing=off only";
	if (v->slice_count > 1)
		return "AR9311 phase one supports video0.slice_count=1 only";
	return NULL;
}

static const char *validate_output(const VencConfig *cfg)
{
	VencOutputUri uri;

	if (cfg->outgoing.max_payload_size < VENC_OUTPUT_PAYLOAD_MIN_BYTES ||
	    cfg->outgoing.max_payload_size > VENC_OUTPUT_PAYLOAD_CEILING_BYTES)
		return "outgoing.max_payload_size is outside the supported range";
	if (!cfg->outgoing.enabled)
		return NULL;
	if (strcmp(cfg->outgoing.stream_mode, "rtp") != 0)
		return "AR9311 phase one supports outgoing.stream_mode=rtp only";
	if (venc_config_parse_output_uri(cfg->outgoing.server, &uri) != 0)
		return "invalid outgoing.server URI";
	if (uri.type == VENC_OUTPUT_URI_SHM)
		return "AR9311 phase one supports udp://, unix://, or "
			"frame-shm:// output";
	return NULL;
}

const char *ar9311_validate_config(const VencConfig *cfg)
{
	const char *err;

	if (!cfg)
		return "missing config";
	if (cfg->sensor.index > 0)
		return "AR9311 has a single camera: sensor.index must be -1 or 0";
	if (cfg->sensor.mode > 0)
		return "AR9311 has a single sensor mode: sensor.mode must be -1 or 0";
	err = validate_video(cfg);
	if (err)
		return err;
	if (cfg->audio.enabled)
		return "AR9311 phase one has no audio";
	if (cfg->record.enabled)
		return "AR9311 phase one has no recording";
	return validate_output(cfg);
}

/* Everything the backend supports is read once at start except these: the
 * encoder takes a new bitrate and a keyframe request while running, the AE
 * limits are read every decision, and the output side is plain userspace
 * state. */
int ar9311_field_is_restart_only(const char *field_key)
{
	static const char *const live[] = {
		"system.verbose",
		"video0.bitrate",
		"isp.gain_max", "isp.gain_min",
		"isp.shutter_max_us", "isp.shutter_min_us",
		"isp.anti_flicker_hz",
		"outgoing.enabled", "outgoing.server",
		"outgoing.max_payload_size",
	};
	size_t i;

	if (!field_key)
		return 1;
	for (i = 0; i < sizeof(live) / sizeof(live[0]); i++) {
		if (strcmp(field_key, live[i]) == 0)
			return 0;
	}
	return 1;
}
