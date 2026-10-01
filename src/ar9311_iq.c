#include "ar9311_iq.h"

#include "ar9311_ae.h"

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Ranges as served in "_schema"; writes outside them are rejected. */
#define EV_COMP_MIN 14   /* ar9311_ae_target_for_ev(-2) */
#define EV_COMP_MAX 224  /* ar9311_ae_target_for_ev(+2) */
#define SAT_MAX 255
#define CONTR_MAX 100
#define SHARP_MAX 4095
#define PCT_MAX 200

typedef struct {
	char *buf;
	size_t size;
	size_t pos;
	int overflow;
} Out;

static void put(Out *o, const char *fmt, ...)
{
	va_list ap;
	int n;

	if (o->overflow)
		return;
	va_start(ap, fmt);
	n = vsnprintf(o->buf + o->pos, o->size - o->pos, fmt, ap);
	va_end(ap);
	if (n < 0 || (size_t)n >= o->size - o->pos) {
		o->overflow = 1;
		return;
	}
	o->pos += (size_t)n;
}

static uint32_t pct_clamp(uint32_t v)
{
	return v > PCT_MAX ? PCT_MAX : v;
}

static unsigned sat_of_pct(uint32_t pct)
{
	unsigned v = (pct_clamp(pct) * AR9311_IQ_SAT_NEUTRAL + 50) / 100;

	return v > SAT_MAX ? SAT_MAX : v;
}

static unsigned sharp_of_pct(uint32_t pct)
{
	unsigned v = (pct_clamp(pct) * AR9311_IQ_SHARP_NEUTRAL + 50) / 100;

	return v > SHARP_MAX ? SHARP_MAX : v;
}

void ar9311_iq_neutral(Ar9311Picture *pic)
{
	pic->saturation = 100;
	pic->contrast = 100;
	pic->sharpness = 100;
	pic->ev = 0.0;
}

int ar9311_iq_render(const Ar9311Picture *pic, char *buf, size_t size)
{
	Out o = { buf, size, 0, 0 };
	unsigned sharp = sharp_of_pct(pic->sharpness);
	int i;

	if (!buf || size == 0)
		return -1;
	put(&o, "{\"ok\":true,\"data\":{\"_schema\":["
		"{\"name\":\"exposure\",\"fields\":["
		"{\"name\":\"auto.compensation\",\"count\":1,\"min\":%d,"
		"\"max\":%d,\"domain\":\"auto\"}]},"
		"{\"name\":\"saturation\",\"fields\":["
		"{\"name\":\"op_type\",\"count\":1,\"min\":0,\"max\":1,"
		"\"domain\":\"direct\"},"
		"{\"name\":\"manual.saturation\",\"count\":1,\"min\":0,"
		"\"max\":%d,\"domain\":\"manual\"}]},"
		"{\"name\":\"csc\",\"fields\":["
		"{\"name\":\"contr\",\"count\":1,\"min\":0,\"max\":%d,"
		"\"domain\":\"direct\"}]},"
		"{\"name\":\"sharpen\",\"fields\":["
		"{\"name\":\"op_type\",\"count\":1,\"min\":0,\"max\":1,"
		"\"domain\":\"direct\"},"
		"{\"name\":\"manual.texture_strength\",\"count\":%d,\"min\":0,"
		"\"max\":%d,\"domain\":\"manual\"},"
		"{\"name\":\"manual.edge_strength\",\"count\":%d,\"min\":0,"
		"\"max\":%d,\"domain\":\"manual\"}]}]",
		EV_COMP_MIN, EV_COMP_MAX, SAT_MAX, CONTR_MAX,
		AR9311_IQ_SHARP_ENTRIES, SHARP_MAX,
		AR9311_IQ_SHARP_ENTRIES, SHARP_MAX);
	put(&o, ",\"exposure\":{\"ret\":0,\"fields\":"
		"{\"auto.compensation\":%u}}",
		ar9311_ae_target_for_ev(pic->ev));
	/* The neutral setting is what "auto" (op_type 0) selects, so that is
	 * how it reads back. */
	put(&o, ",\"saturation\":{\"ret\":0,\"fields\":{\"op_type\":%d,"
		"\"manual.saturation\":%u}}",
		pic->saturation == 100 ? 0 : 1, sat_of_pct(pic->saturation));
	put(&o, ",\"csc\":{\"ret\":0,\"fields\":{\"contr\":%u}}",
		pct_clamp(pic->contrast) / 2);
	put(&o, ",\"sharpen\":{\"ret\":0,\"fields\":{\"op_type\":%d",
		pic->sharpness == 100 ? 0 : 1);
	put(&o, ",\"manual.texture_strength\":[");
	for (i = 0; i < AR9311_IQ_SHARP_ENTRIES; i++)
		put(&o, "%s%u", i ? "," : "", sharp);
	put(&o, "],\"manual.edge_strength\":[");
	for (i = 0; i < AR9311_IQ_SHARP_ENTRIES; i++)
		put(&o, "%s%u", i ? "," : "", sharp);
	put(&o, "]}}}}");
	return o.overflow ? -1 : (int)o.pos;
}

static int parse_long(const char **p, long *out)
{
	char *end;
	long v;

	errno = 0;
	v = strtol(*p, &end, 10);
	if (end == *p || errno)
		return -1;
	*out = v;
	*p = end;
	return 0;
}

/* One integer, nothing after it, inside [min, max]. */
static int parse_one(const char *value, long min, long max, long *out)
{
	const char *p = value;

	if (parse_long(&p, out) != 0 || *p != '\0')
		return -1;
	return *out < min || *out > max ? -1 : 0;
}

/* op_type: 0 is auto, the tuned picture; 1 (manual) changes nothing until
 * a manual value is written, as on CV610 where it only selects which half
 * of the struct the ISP reads. */
static int op_type(const char *value, uint32_t *pct)
{
	long v;

	if (parse_one(value, 0, 1, &v) != 0)
		return -1;
	if (v == 0)
		*pct = 100;
	return 0;
}

int ar9311_iq_set(Ar9311Picture *pic, const char *param, const char *value)
{
	long v;

	if (!pic || !param || !value)
		return -1;

	if (strcmp(param, "exposure.auto.compensation") == 0) {
		double ev;

		if (parse_one(value, EV_COMP_MIN, EV_COMP_MAX, &v) != 0)
			return -1;
		ev = log2((double)v / AR9311_AE_TARGET);
		if (ev > AR9311_AE_EV_MAX)
			ev = AR9311_AE_EV_MAX;
		if (ev < -AR9311_AE_EV_MAX)
			ev = -AR9311_AE_EV_MAX;
		pic->ev = ev;
		return 0;
	}
	if (strcmp(param, "saturation.op_type") == 0)
		return op_type(value, &pic->saturation);
	if (strcmp(param, "saturation.manual.saturation") == 0) {
		if (parse_one(value, 0, SAT_MAX, &v) != 0)
			return -1;
		pic->saturation = (uint32_t)((v * 100 + AR9311_IQ_SAT_NEUTRAL / 2) /
			AR9311_IQ_SAT_NEUTRAL);
		return 0;
	}
	if (strcmp(param, "csc.contr") == 0) {
		if (parse_one(value, 0, CONTR_MAX, &v) != 0)
			return -1;
		pic->contrast = (uint32_t)(v * 2);
		return 0;
	}
	if (strcmp(param, "sharpen.op_type") == 0)
		return op_type(value, &pic->sharpness);
	if (strcmp(param, "sharpen.manual.texture_strength") == 0 ||
	    strcmp(param, "sharpen.manual.edge_strength") == 0) {
		/* One control behind both curves: their mean, relative to the
		 * neutral curve, is the sharpness. */
		const char *p = value;
		long sum = 0, pct;
		int n = 0;

		for (;;) {
			if (parse_long(&p, &v) != 0 || v < 0 || v > SHARP_MAX)
				return -1;
			sum += v;
			n++;
			if (*p == '\0')
				break;
			if (*p != ',' || n == AR9311_IQ_SHARP_ENTRIES)
				return -1;
			p++;
		}
		pct = (sum * 100 + (long)n * AR9311_IQ_SHARP_NEUTRAL / 2) /
			((long)n * AR9311_IQ_SHARP_NEUTRAL);
		pic->sharpness = (uint32_t)(pct > PCT_MAX ? PCT_MAX : pct);
		return 0;
	}
	return -1;
}
