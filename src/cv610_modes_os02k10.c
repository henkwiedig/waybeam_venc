#include "cv610_modes.h"

/* All three entries are reverse-engineered from the Caddx Ascent's stock
 * firmware (`ar_ldyhs_sky`), not hardware-verified on a bring-up board the
 * way IMX662's table is — see
 * documentation/os02k10/OS02K10_ASCENT_REGISTER_RE.md.
 *
 * sensor_clock_hz is 0 (unknown) for every mode: sensor_clock_select() in
 * cv610_pipeline.c writes MCLK through a sysfs knob
 * (CV610_SNS0_CLK_HZ_PATH) that belongs to a different reference bring-up's
 * kernel module set than the Ascent's own (ko_cv610_20s/). Whether OS02K10's
 * MCLK on this board is even software-configurable through that path is
 * unconfirmed, and hz==0 makes sensor_clock_select() a documented no-op
 * (leave the clock as the loader set it) rather than fail — the safe
 * default until this is checked on the bench. 60 fps matches the Ascent's
 * actual boot configuration (confirmed live, see the RE doc §9). */
static const Cv610SensorMode g_cv610_modes[] = {
	{ 1920, 1080,  50, 10, 0u, "OS02K10 1080p50 RAW10" },
	{ 1920, 1080,  60, 10, 0u, "OS02K10 1080p60 RAW10" },
	{ 1920, 1080, 100, 10, 0u, "OS02K10 1080p100 RAW10" },
};

#define CV610_MODE_COUNT \
	(sizeof(g_cv610_modes) / sizeof(g_cv610_modes[0]))

const Cv610SensorMode *cv610_mode_table(size_t *count)
{
	if (count)
		*count = CV610_MODE_COUNT;
	return g_cv610_modes;
}

/* VPSS scales, it does not invent detail: upscaling past the captured size
 * costs encoder bandwidth for no information.  Refuse it rather than let a
 * config quietly waste the link. */
#define CV610_OUT_ALIGN 8u
#define CV610_OUT_MIN   128u

const Cv610SensorMode *cv610_mode_for_fps(uint32_t fps)
{
	size_t i;

	for (i = 0; i < CV610_MODE_COUNT; ++i) {
		if (g_cv610_modes[i].fps == fps)
			return &g_cv610_modes[i];
	}
	return NULL;
}

const Cv610SensorMode *cv610_mode_select(int forced_mode, uint32_t target_fps,
	int *out_index)
{
	const Cv610SensorMode *pick;
	size_t pick_i = 0;
	size_t i;

	if (out_index)
		*out_index = -1;

	/* A forced index is the whole answer: it exists or the config is wrong.
	 * Parity with find_best_mode(), which fails bring-up rather than
	 * quietly running a mode the operator did not ask for. */
	if (forced_mode >= 0) {
		if ((size_t)forced_mode >= CV610_MODE_COUNT)
			return NULL;
		if (out_index)
			*out_index = forced_mode;
		return &g_cv610_modes[forced_mode];
	}
	/* Nothing to select against.  Rejecting keeps the behaviour this
	 * function replaced — a bare cv610_mode_for_fps(0) returned NULL too —
	 * rather than inventing a rate for a config that names none. */
	if (target_fps == 0)
		return NULL;

	pick = cv610_mode_for_fps(target_fps);
	if (pick != NULL) {
		if (out_index)
			*out_index = (int)(pick - g_cv610_modes);
		return pick;
	}

	/* Slowest mode still faster than the target: never deliver fewer frames
	 * than asked for while a mode that can keep up exists. */
	for (i = 0; i < CV610_MODE_COUNT; ++i) {
		const Cv610SensorMode *m = &g_cv610_modes[i];

		if (m->fps > target_fps && (pick == NULL || m->fps < pick->fps)) {
			pick = m;
			pick_i = i;
		}
	}
	/* Target above every mode: clamp to the fastest. */
	if (pick == NULL) {
		for (i = 0; i < CV610_MODE_COUNT; ++i) {
			if (pick == NULL || g_cv610_modes[i].fps > pick->fps) {
				pick = &g_cv610_modes[i];
				pick_i = i;
			}
		}
	}
	if (out_index)
		*out_index = (int)pick_i;
	return pick;
}

const char *cv610_mode_check_output(const Cv610SensorMode *mode,
	uint32_t width, uint32_t height)
{
	if (mode == NULL)
		return "no sensor mode selected";
	/* Only a fully unset geometry means "the mode's own size".  A half-set
	 * one (1920x0) is a typo, not a request, and must not be completed into
	 * something the caller did not ask for. */
	if (width == 0 && height == 0)
		return NULL;
	if (width == 0 || height == 0)
		return "CV610 video0.size needs both a width and a height";
	if (width % CV610_OUT_ALIGN != 0 || height % CV610_OUT_ALIGN != 0)
		return "CV610 video0.size must be a multiple of 8";
	if (width < CV610_OUT_MIN || height < CV610_OUT_MIN)
		return "CV610 video0.size must be at least 128x128";
	if (width > mode->width || height > mode->height)
		return "CV610 video0.size cannot exceed the sensor mode's capture size";
	return NULL;
}

void cv610_mode_resolve_output(const Cv610SensorMode *mode,
	uint32_t req_width, uint32_t req_height,
	uint32_t *out_width, uint32_t *out_height)
{
	uint32_t w = 0;
	uint32_t h = 0;

	if (mode != NULL) {
		int given = (req_width != 0 && req_height != 0);

		w = given ? req_width : mode->width;
		h = given ? req_height : mode->height;
	}
	if (out_width)
		*out_width = w;
	if (out_height)
		*out_height = h;
}
