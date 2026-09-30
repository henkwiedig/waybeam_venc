#include "ar9311_validation.h"
#include "venc_api.h"
#include "venc_config.h"

#include <stdio.h>
#include <string.h>

/* venc_config.c's loader calls this hook; its real home is venc_api.c, which
 * this test does not link. */
const char *venc_api_validate_loaded_config(const VencConfig *cfg)
{
	return ar9311_validate_config(cfg);
}

static int expect(const char *name, int ok, const char *detail)
{
	printf("  %s  %s%s%s\n", ok ? "PASS" : "FAIL", name,
		detail ? ": " : "", detail ? detail : "");
	return ok ? 0 : 1;
}

static void valid_config(VencConfig *cfg)
{
	venc_config_defaults(cfg);
	cfg->video0.width = AR9311_WIDTH;
	cfg->video0.height = AR9311_HEIGHT;
	cfg->video0.fps = AR9311_FPS;
	cfg->video0.bitrate = 8000;
	cfg->video0.gop_size = 1.0;
	snprintf(cfg->video0.rc_mode, sizeof(cfg->video0.rc_mode), "cbr");
	snprintf(cfg->video0.resilience, sizeof(cfg->video0.resilience), "off");
	snprintf(cfg->video0.framing, sizeof(cfg->video0.framing), "off");
	cfg->video0.slice_count = 1;
	cfg->audio.enabled = false;
	cfg->record.enabled = false;
	cfg->outgoing.enabled = true;
	snprintf(cfg->outgoing.server, sizeof(cfg->outgoing.server),
		"frame-shm://venc_frames");
	snprintf(cfg->outgoing.stream_mode, sizeof(cfg->outgoing.stream_mode),
		"rtp");
	cfg->outgoing.max_payload_size = 1400;
}

static int expect_config(const char *name, const VencConfig *cfg, int valid)
{
	const char *err = ar9311_validate_config(cfg);

	return expect(name, valid ? err == NULL : err != NULL, err);
}

static int test_validation(void)
{
	VencConfig cfg;
	int failures = 0;

	valid_config(&cfg);
	failures += expect_config("ar9311_accept_default_shape", &cfg, 1);

	valid_config(&cfg);
	snprintf(cfg.outgoing.server, sizeof(cfg.outgoing.server),
		"udp://192.168.3.204:5600");
	failures += expect_config("ar9311_accept_udp", &cfg, 1);

	valid_config(&cfg);
	cfg.video0.width = 1280;
	cfg.video0.height = 720;
	failures += expect_config("ar9311_reject_720p", &cfg, 0);

	valid_config(&cfg);
	cfg.video0.fps = 30;
	failures += expect_config("ar9311_reject_30fps", &cfg, 0);

	valid_config(&cfg);
	cfg.video0.gop_size = 0.0;
	failures += expect_config("ar9311_accept_all_intra", &cfg, 1);

	valid_config(&cfg);
	cfg.video0.gop_size = 34.0;   /* 2040 frames */
	failures += expect_config("ar9311_accept_gop_2040", &cfg, 1);

	valid_config(&cfg);
	cfg.video0.gop_size = 35.0;   /* 2100 frames > 2047 */
	failures += expect_config("ar9311_reject_gop_over_2047", &cfg, 0);

	valid_config(&cfg);
	cfg.video0.min_qp = 40;
	cfg.video0.max_qp = 30;
	failures += expect_config("ar9311_reject_inverted_qp", &cfg, 0);

	valid_config(&cfg);
	snprintf(cfg.video0.resilience, sizeof(cfg.video0.resilience), "racing");
	failures += expect_config("ar9311_reject_resilience", &cfg, 0);

	valid_config(&cfg);
	cfg.audio.enabled = true;
	failures += expect_config("ar9311_reject_audio", &cfg, 0);

	valid_config(&cfg);
	snprintf(cfg.outgoing.server, sizeof(cfg.outgoing.server),
		"shm://local_shm");
	failures += expect_config("ar9311_reject_packet_shm", &cfg, 0);

	valid_config(&cfg);
	cfg.sensor.mode = 1;
	failures += expect_config("ar9311_reject_sensor_mode_1", &cfg, 0);
	return failures;
}

static int test_mutability(void)
{
	int failures = 0;

	failures += expect("ar9311_bitrate_live",
		!ar9311_field_is_restart_only("video0.bitrate"), NULL);
	failures += expect("ar9311_server_live",
		!ar9311_field_is_restart_only("outgoing.server"), NULL);
	failures += expect("ar9311_gop_restart",
		ar9311_field_is_restart_only("video0.gop_size"), NULL);
	failures += expect("ar9311_fps_restart",
		ar9311_field_is_restart_only("video0.fps"), NULL);
	return failures;
}

int main(void)
{
	int failures = test_validation() + test_mutability();

	printf("ar9311 validation: %s\n", failures ? "FAILED" : "ok");
	return failures ? 1 : 0;
}
