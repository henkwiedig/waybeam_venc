#include "ar9311_picture.h"

#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#define PICTURE_SATURATION "/sys/module/ar_isp/parameters/saturation"
#define PICTURE_CONTRAST "/sys/module/ar_isp/parameters/contrast"
#define PICTURE_SHARPNESS "/sys/module/ar_isp/parameters/sharpness"
/* Re-latches: the ladders write rewrites the colour-space bank, which
 * carries saturation, and the lnr bank, which carries sharpness; tone 2
 * forces a gamma page rebuild, which carries contrast. */
#define PICTURE_LADDERS "/sys/kernel/debug/ar-isp/ladders"
#define PICTURE_TONE "/sys/kernel/debug/ar-isp/tone"

/* What the ISP holds, unknown until a write succeeds: the parameters
 * outlive a daemon restart, so the first apply always writes. */
#define PICTURE_UNKNOWN UINT32_MAX
static uint32_t g_saturation = PICTURE_UNKNOWN;   /* as applied, gain-scaled */
static uint32_t g_contrast = PICTURE_UNKNOWN;
static uint32_t g_sharpness = PICTURE_UNKNOWN;
/* The configured saturation and the AE's gain it is scaled by.  The AE
 * thread and the API both land here. */
static uint32_t g_saturation_set = AR9311_PICTURE_NEUTRAL;
static uint32_t g_gain_q10 = 1024u;
static int g_gain_ramp = 1;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

static int write_uint(const char *path, uint32_t value)
{
	char buf[16];
	int fd, len, ok;

	fd = open(path, O_WRONLY | O_CLOEXEC);
	if (fd < 0)
		return -1;
	len = snprintf(buf, sizeof(buf), "%u\n", value);
	ok = write(fd, buf, (size_t)len) == len;
	close(fd);
	return ok ? 0 : -1;
}

/* Write one control and its re-latch.  A neutral value that cannot be
 * written is not an error: a kernel without the parameter shows the tuned
 * picture, which is what neutral asks for. */
static int apply_one(const char *name, const char *param, const char *relatch,
	uint32_t relatch_value, uint32_t value, uint32_t *held)
{
	if (value == *held)
		return 0;
	if (write_uint(param, value) == 0 &&
	    write_uint(relatch, relatch_value) == 0) {
		*held = value;
		return 0;
	}
	if (value == AR9311_PICTURE_NEUTRAL)
		return 0;
	fprintf(stderr, "ERROR: [ar9311] %s %u not applied (needs the "
		"builder's ar_isp picture patch and debugfs)\n", name, value);
	return -1;
}

/* Saturation as it applies now: on the ramp, or as set. */
static uint32_t saturation_now(void)
{
	return g_gain_ramp ?
		ar9311_picture_saturation_at(g_saturation_set, g_gain_q10) :
		g_saturation_set;
}

int ar9311_picture_apply(uint32_t saturation, uint32_t contrast,
	uint32_t sharpness)
{
	int rc = 0;

	if (saturation > AR9311_PICTURE_MAX)
		saturation = AR9311_PICTURE_MAX;
	if (contrast > AR9311_PICTURE_MAX)
		contrast = AR9311_PICTURE_MAX;
	if (sharpness > AR9311_PICTURE_MAX)
		sharpness = AR9311_PICTURE_MAX;
	pthread_mutex_lock(&g_lock);
	g_saturation_set = saturation;
	rc |= apply_one("saturation", PICTURE_SATURATION, PICTURE_LADDERS, 1,
		saturation_now(), &g_saturation);
	rc |= apply_one("contrast", PICTURE_CONTRAST, PICTURE_TONE, 2,
		contrast, &g_contrast);
	rc |= apply_one("sharpness", PICTURE_SHARPNESS, PICTURE_LADDERS, 1,
		sharpness, &g_sharpness);
	pthread_mutex_unlock(&g_lock);
	return rc ? -1 : 0;
}

void ar9311_picture_set_gain(uint32_t gain_q10)
{
	uint32_t value;

	pthread_mutex_lock(&g_lock);
	g_gain_q10 = gain_q10;
	value = saturation_now();
	if (value != g_saturation && write_uint(PICTURE_SATURATION, value) == 0)
		g_saturation = value;
	pthread_mutex_unlock(&g_lock);
}

void ar9311_picture_set_gain_ramp(int on)
{
	pthread_mutex_lock(&g_lock);
	g_gain_ramp = on;
	pthread_mutex_unlock(&g_lock);
}
