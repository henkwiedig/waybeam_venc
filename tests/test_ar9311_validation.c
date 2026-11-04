#include "ar9311_encoder.h"
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
	failures += expect_config("ar9311_accept_racing",
		&cfg, venc_config_apply_resilience_preset("racing",
			&cfg.video0) == 0);
	failures += expect_config("ar9311_accept_racing_preset", &cfg, 1);

	valid_config(&cfg);
	(void)venc_config_apply_resilience_preset("range", &cfg.video0);
	failures += expect_config("ar9311_reject_svc_preset", &cfg, 0);

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

static int expect_plan(const char *name, uint32_t shared_rows,
	uint32_t period, uint32_t rows, uint32_t cycle)
{
	Ar9311RefreshPlan p;
	char detail[96];

	ar9311_encoder_refresh_plan(AR9311_HEIGHT, shared_rows, &p);
	snprintf(detail, sizeof(detail), "total %u period %u rows %u cycle %u",
		p.total_rows, p.period, p.rows, p.cycle);
	return expect(name, p.total_rows == 17 && p.period == period &&
		p.rows == rows && p.cycle == cycle, detail);
}

/* 1080 lines are 17 of wave5's 64-line rows; the shared helper asks in
 * 32-line rows (fast 4, balanced 2, robust 1 at 1080p60). */
static int test_refresh_plan(void)
{
	int failures = 0;

	failures += expect_plan("ar9311_refresh_fast", 4, 8, 2, 9);
	failures += expect_plan("ar9311_refresh_balanced", 2, 17, 1, 17);
	/* Was period 34: above the row count, wave5 refreshed every row of
	 * every picture. */
	failures += expect_plan("ar9311_refresh_robust", 1, 17, 1, 17);
	failures += expect_plan("ar9311_refresh_3_shared_rows", 3, 8, 2, 9);
	failures += expect_plan("ar9311_refresh_whole_picture", 34, 1, 17, 1);
	return failures;
}

int main(void)
{
	int failures = test_validation() + test_mutability() +
		test_refresh_plan();

	printf("ar9311 validation: %s\n", failures ? "FAILED" : "ok");
	return failures ? 1 : 0;
}
