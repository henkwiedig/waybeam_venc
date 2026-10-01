#include "ar9311_iq.h"

#include "cJSON.h"

#include <stdio.h>
#include <string.h>

static int expect(const char *name, int ok)
{
	printf("  %s  %s\n", ok ? "PASS" : "FAIL", name);
	return ok ? 0 : 1;
}

/* One write on a picture at @p start_pct (all three controls), 0 EV. */
static Ar9311Picture after(uint32_t start_pct, const char *param,
	const char *value, int *rc)
{
	Ar9311Picture pic = { start_pct, start_pct, start_pct, 0.0 };

	*rc = ar9311_iq_set(&pic, param, value);
	return pic;
}

static int rejected(const char *param, const char *value)
{
	Ar9311Picture pic;
	int rc;

	pic = after(150, param, value, &rc);
	/* A rejected write leaves the picture alone. */
	return rc != 0 && pic.saturation == 150 && pic.contrast == 150 &&
		pic.sharpness == 150 && pic.ev == 0.0;
}

static const cJSON *field(const cJSON *data, const char *group,
	const char *name)
{
	const cJSON *g = cJSON_GetObjectItemCaseSensitive(data, group);

	return cJSON_GetObjectItemCaseSensitive(
		cJSON_GetObjectItemCaseSensitive(g, "fields"), name);
}

static int test_render(void)
{
	Ar9311Picture isp;
	char buf[4096];
	cJSON *root;
	const cJSON *data, *arr;
	int f = 0;

	ar9311_iq_neutral(&isp);
	f += expect("render fits", ar9311_iq_render(&isp, buf, sizeof(buf)) > 0);
	root = cJSON_Parse(buf);
	f += expect("render is JSON", root != NULL);
	if (!root)
		return f + 1;
	data = cJSON_GetObjectItemCaseSensitive(root, "data");
	f += expect("schema present",
		cJSON_IsArray(cJSON_GetObjectItemCaseSensitive(data, "_schema")));
	f += expect("neutral ev reads the default AE target",
		field(data, "exposure", "auto.compensation")->valueint == 56);
	f += expect("neutral saturation reads auto, 128",
		field(data, "saturation", "op_type")->valueint == 0 &&
		field(data, "saturation", "manual.saturation")->valueint == 128);
	f += expect("neutral contrast reads 50",
		field(data, "csc", "contr")->valueint == 50);
	arr = field(data, "sharpen", "manual.edge_strength");
	f += expect("sharpen curves are 32 entries at 2048",
		cJSON_GetArraySize(arr) == AR9311_IQ_SHARP_ENTRIES &&
		cJSON_GetArrayItem(arr, 31)->valueint == 2048 &&
		field(data, "sharpen", "op_type")->valueint == 0);
	cJSON_Delete(root);

	isp.saturation = 200;
	isp.contrast = 30;
	isp.sharpness = 150;
	isp.ev = 1.0;
	ar9311_iq_render(&isp, buf, sizeof(buf));
	root = cJSON_Parse(buf);
	data = cJSON_GetObjectItemCaseSensitive(root, "data");
	f += expect("changed values read back",
		field(data, "exposure", "auto.compensation")->valueint == 112 &&
		field(data, "saturation", "op_type")->valueint == 1 &&
		field(data, "saturation", "manual.saturation")->valueint == 255 &&
		field(data, "csc", "contr")->valueint == 15 &&
		cJSON_GetArrayItem(field(data, "sharpen",
			"manual.texture_strength"), 0)->valueint == 3072);
	cJSON_Delete(root);
	f += expect("small buffer refused",
		ar9311_iq_render(&isp, buf, 64) < 0);
	return f;
}

static int test_set(void)
{
	Ar9311Picture p;
	int rc, f = 0;

	p = after(100, "exposure.auto.compensation", "112", &rc);
	f += expect("ev: doubled target is +1", rc == 0 && p.ev == 1.0);
	p = after(100, "exposure.auto.compensation", "28", &rc);
	f += expect("ev: 28 is -1", rc == 0 && p.ev == -1.0);
	f += expect("ev: outside 14..224 rejected",
		rejected("exposure.auto.compensation", "255") &&
		rejected("exposure.auto.compensation", "0"));
	/* gsmenu's 0-10 levels write int(n * 25.5 + 0.5). */
	p = after(150, "saturation.manual.saturation", "128", &rc);
	f += expect("saturation: level 5 (128) is neutral",
		rc == 0 && p.saturation == 100 && p.contrast == 150);
	p = after(100, "saturation.manual.saturation", "255", &rc);
	f += expect("saturation: level 10 (255) is 199 %",
		rc == 0 && p.saturation == 199);
	p = after(100, "saturation.manual.saturation", "0", &rc);
	f += expect("saturation: 0 is grey", rc == 0 && p.saturation == 0);
	p = after(150, "saturation.op_type", "0", &rc);
	f += expect("saturation: auto restores neutral",
		rc == 0 && p.saturation == 100);
	p = after(150, "saturation.op_type", "1", &rc);
	f += expect("saturation: manual alone changes nothing",
		rc == 0 && p.saturation == 150);
	p = after(100, "csc.contr", "100", &rc);
	f += expect("contrast: 100 is double", rc == 0 && p.contrast == 200);
	p = after(150, "csc.contr", "50", &rc);
	f += expect("contrast: 50 is neutral", rc == 0 && p.contrast == 100);
	f += expect("contrast: 101 rejected", rejected("csc.contr", "101"));
	p = after(100, "sharpen.manual.edge_strength", "3072,3072,3072,3072",
		&rc);
	f += expect("sharpness: curve mean against 2048",
		rc == 0 && p.sharpness == 150);
	p = after(100, "sharpen.manual.texture_strength", "0", &rc);
	f += expect("sharpness: 0 is off", rc == 0 && p.sharpness == 0);
	p = after(100, "sharpen.manual.edge_strength", "4095,4095", &rc);
	f += expect("sharpness: capped at 200 %", rc == 0 && p.sharpness == 200);
	p = after(150, "sharpen.op_type", "0", &rc);
	f += expect("sharpness: auto restores neutral",
		rc == 0 && p.sharpness == 100);
	f += expect("sharpness: entry over 4095 or junk rejected",
		rejected("sharpen.manual.edge_strength", "4096") &&
		rejected("sharpen.manual.edge_strength", "1,,2") &&
		rejected("sharpen.manual.edge_strength", "1,"));
	f += expect("unknown field rejected",
		rejected("sharpen.manual.overshoot", "1") &&
		rejected("saturation.manual.saturation", "12x"));
	return f;
}

int main(void)
{
	int failures = test_render() + test_set();

	printf("ar9311 iq: %s\n", failures ? "FAILED" : "ok");
	return failures ? 1 : 0;
}
