#include "ar9311_encoder.h"
#include "ar9311_strips.h"
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

	/* What /api/v1/capabilities offers for video0.resilience on AR9311:
	 * the presets that pass once expanded, less the ones that run as
	 * another preset here. */
	{
		char got[160] = "";
		const char *name;

		for (size_t i = 0;
		     (name = venc_config_resilience_preset_name(i)); i++) {
			valid_config(&cfg);
			(void)venc_config_apply_resilience_preset(name,
				&cfg.video0);
			if (ar9311_validate_config(&cfg) ||
			    !ar9311_resilience_offered(name))
				continue;
			strncat(got, " ", sizeof(got) - strlen(got) - 1);
			strncat(got, name, sizeof(got) - strlen(got) - 1);
		}
		failures += expect("ar9311_resilience_presets",
			strcmp(got, " off rescue quality racing endurance") == 0,
			got);
	}
	/* sprint and patrol stay accepted: a saved one must still start. */
	valid_config(&cfg);
	(void)venc_config_apply_resilience_preset("patrol", &cfg.video0);
	failures += expect_config("ar9311_accept_unlisted_patrol", &cfg, 1);

	valid_config(&cfg);
	cfg.video0.strips = 4;
	failures += expect_config("ar9311_accept_4_strips", &cfg, 1);

	valid_config(&cfg);
	cfg.video0.strips = 5;
	failures += expect_config("ar9311_reject_5_strips", &cfg, 0);

	valid_config(&cfg);
	cfg.video0.strip_overlap = 2;
	failures += expect_config("ar9311_accept_strip_overlap_2", &cfg, 1);

	valid_config(&cfg);
	cfg.video0.strip_overlap = 3;
	failures += expect_config("ar9311_reject_strip_overlap_3", &cfg, 0);

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

/* Depth 1: band 0 (lines 0-63) is in memory 948 us after the timestamp,
 * plus 110 us of chroma and a 400 us margin; depth 2 hands the buffer back a
 * frame later, written, so every band is already due. */
static int test_sfs_schedule(void)
{
	int32_t start;
	uint32_t row;
	int failures = 0;

	ar9311_encoder_sfs_schedule(1, 60, &start, &row);
	failures += expect("ar9311_sfs_depth1", start == 1458 && row == 948, NULL);
	ar9311_encoder_sfs_schedule(2, 60, &start, &row);
	failures += expect("ar9311_sfs_depth2", start == 1458 - 16666 &&
		row == 948, NULL);
	return failures;
}

/* Strips on the depth-1 SFS schedule (band n released at 1458 + 948 n us):
 * whole 64-line rows, the spare rows to the top, every strip but the last
 * queued at its last band's release, the last one following the camera from
 * its first band. */
static int test_strips_plan(void)
{
	Ar9311StripPlan p;
	int failures = 0;

	failures += expect("ar9311_strips_2",
		ar9311_strips_plan(2, 1080, 0, 1458, 948, &p) == 0 &&
		p.y0[0] == 0 && p.height[0] == 576 &&
		p.y0[1] == 576 && p.height[1] == 504 &&
		p.due_us[0] == 1458 + 948 * 8 && p.due_us[1] == 0 &&
		p.sfs_start_us == 1458 + 948 * 9, NULL);
	failures += expect("ar9311_strips_3",
		ar9311_strips_plan(3, 1080, 0, 1458, 948, &p) == 0 &&
		p.height[0] == 384 && p.height[1] == 384 && p.height[2] == 312 &&
		p.y0[2] == 768 && p.due_us[0] == 1458 + 948 * 5 &&
		p.due_us[1] == 1458 + 948 * 11 &&
		p.sfs_start_us == 1458 + 948 * 12, NULL);
	failures += expect("ar9311_strips_4",
		ar9311_strips_plan(4, 1080, 0, 1458, 948, &p) == 0 &&
		p.height[0] == 320 && p.height[1] == 256 && p.height[2] == 256 &&
		p.height[3] == 248 && p.y0[3] == 832 &&
		p.due_us[0] == 5250 && p.due_us[1] == 9042 &&
		p.due_us[2] == 12834 && p.sfs_start_us == 13782, NULL);
	failures += expect("ar9311_strips_reject_1",
		ar9311_strips_plan(1, 1080, 0, 1458, 948, &p) != 0, NULL);
	failures += expect("ar9311_strips_reject_5",
		ar9311_strips_plan(5, 1080, 0, 1458, 948, &p) != 0, NULL);
	failures += expect("ar9311_strips_reject_too_few_rows",
		ar9311_strips_plan(4, 128, 0, 1458, 948, &p) != 0, NULL);
	/* overlap: a row past each seam, strip 0 two slices (9 + 1 rows), the
	 * others one row a slice; strip 0 waits for its extra row, the last
	 * strip follows the camera from its own */
	failures += expect("ar9311_strips_2_overlap",
		ar9311_strips_plan(2, 1080, 1, 1458, 948, &p) == 0 &&
		p.y0[0] == 0 && p.height[0] == 576 &&
		p.enc_y0[0] == 0 && p.enc_height[0] == 640 &&
		p.slice_rows[0] == 9 &&
		p.y0[1] == 576 && p.height[1] == 504 &&
		p.enc_y0[1] == 512 && p.enc_height[1] == 568 &&
		p.slice_rows[1] == 1 &&
		p.due_us[0] == 1458 + 948 * 9 &&
		p.sfs_start_us == 1458 + 948 * 8, NULL);
	failures += expect("ar9311_strips_3_overlap",
		ar9311_strips_plan(3, 1080, 1, 1458, 948, &p) == 0 &&
		p.enc_height[0] == 448 && p.slice_rows[0] == 6 &&
		p.enc_y0[1] == 320 && p.enc_height[1] == 512 &&
		p.slice_rows[1] == 1 &&
		p.enc_y0[2] == 704 && p.enc_height[2] == 376 &&
		p.due_us[0] == 1458 + 948 * 6 && p.due_us[1] == 1458 + 948 * 12 &&
		p.sfs_start_us == 1458 + 948 * 11, NULL);
	failures += expect("ar9311_strips_reject_overlap_3",
		ar9311_strips_plan(2, 1080, 3, 1458, 948, &p) != 0, NULL);
	failures += expect("ar9311_strips_reject_overlap_off_slice_grid",
		ar9311_strips_plan(3, 576, 2, 1458, 948, &p) != 0, NULL);
	return failures;
}

int main(void)
{
	int failures = test_validation() + test_mutability() +
		test_refresh_plan() + test_sfs_schedule() + test_strips_plan();

	printf("ar9311 validation: %s\n", failures ? "FAILED" : "ok");
	return failures ? 1 : 0;
}
