/* ar9311_merge: N strip pictures joined into one picture of N slices.
 * The module's own parsers are used to read the result back, hence the
 * source include. */
#include "../src/ar9311_merge.c"

#include "ar9311_merge_fixture.h"

#include <stdio.h>

static int expect(const char *name, int ok, const char *detail)
{
	printf("  %s  %s%s%s\n", ok ? "PASS" : "FAIL", name,
		detail ? ": " : "", detail ? detail : "");
	return ok ? 0 : 1;
}

/* The first NAL of @p type in an Annex-B buffer, as RBSP. */
static size_t find_rbsp(const uint8_t *buf, size_t len, uint32_t type,
	uint8_t *rbsp, int skip)
{
	size_t cursor = 0, nal_len;
	const uint8_t *nal;

	while (next_nal(buf, len, &cursor, &nal, &nal_len))
		if (((nal[0] >> 1) & 0x3f) == type && skip-- == 0)
			return unescape(nal, nal_len, rbsp);
	return 0;
}

static int count_nals(const uint8_t *buf, size_t len)
{
	size_t cursor = 0, nal_len;
	const uint8_t *nal;
	int n = 0;

	while (next_nal(buf, len, &cursor, &nal, &nal_len))
		n++;
	return n;
}

static uint32_t slice_type_of(const uint8_t *au, size_t len)
{
	size_t cursor = 0, nal_len;
	const uint8_t *nal;

	while (next_nal(au, len, &cursor, &nal, &nal_len))
		if (((nal[0] >> 1) & 0x3f) < 32)
			return (nal[0] >> 1) & 0x3f;
	return 64;
}

typedef struct {
	const uint8_t *data;
	size_t len;
} Au;

/* Join one frame of @p n strips and check the result: strip 0 carries the
 * parameter sets with the full height, every other strip only its slice,
 * moved to its CTU offset with the slice data untouched, every slice with
 * the same POC (@p want_poc, or strip 0's when it is -1). */
static int check_frame(const char *tag, Ar9311Merge *m, int n,
	const uint32_t *heights, const Au *au, uint32_t want_poc)
{
	static uint8_t out[AR9311_MERGE_MAX_STRIPS][4096];
	static uint8_t a[4096], b[4096];
	size_t len[AR9311_MERGE_MAX_STRIPS];
	Ar9311HevcSps sps;
	Ar9311HevcPps pps;
	char name[128];
	uint32_t full = 0, off = 0;
	int k, failures = 0;

	for (k = 0; k < n; k++) {
		len[k] = ar9311_merge_strip(m, k, au[k].data, au[k].len, out[k],
			sizeof(out[k]));
		snprintf(name, sizeof(name), "%s strip %d joined", tag, k);
		failures += expect(name, len[k] > 0, m->error);
		if (!len[k])
			return failures;
		full += heights[k];
	}
	snprintf(name, sizeof(name), "%s in step", tag);
	failures += expect(name, ar9311_merge_in_step(m), NULL);
	if (want_poc == (uint32_t)-1)
		want_poc = m->poc[0];

	/* the picture's SPS/PPS, as the receiver sees them */
	if (find_rbsp(out[0], len[0], 33, a, 0)) {
		snprintf(name, sizeof(name), "%s SPS full height", tag);
		failures += expect(name, parse_sps(a, find_rbsp(out[0], len[0],
			33, a, 0), &sps) && sps.width == 1920 &&
			sps.height == full && !sps.conf_flag, NULL);
		snprintf(name, sizeof(name), "%s SPS otherwise strip 0's", tag);
		failures += expect(name, sps.log2_max_poc_lsb ==
			m->sps[0].log2_max_poc_lsb && sps.ctb_log2 ==
			m->sps[0].ctb_log2 && sps.num_st_rps == m->sps[0].num_st_rps &&
			sps.sao == m->sps[0].sao, NULL);
		size_t plen = find_rbsp(out[0], len[0], 34, b, 0);

		snprintf(name, sizeof(name), "%s PPS passed through", tag);
		failures += expect(name,
			plen && parse_pps(b, plen, &pps) && !pps.lf_across_slices,
			NULL);
	} else {
		sps = m->sps[0];
		sps.height = full;
		pps = m->pps[0];
	}

	for (k = 0; k < n; k++) {
		SliceHdr h, h0;
		const char *why = NULL;
		size_t mlen = find_rbsp(out[k], len[k], slice_type_of(out[k], len[k]),
			a, 0);
		size_t olen = find_rbsp(au[k].data, au[k].len,
			slice_type_of(au[k].data, au[k].len), b, 0);
		int ok;

		snprintf(name, sizeof(name), "%s strip %d: slice only", tag, k);
		if (k > 0)
			failures += expect(name, count_nals(out[k], len[k]) == 1, NULL);
		ok = parse_slice(a, mlen, &sps, &pps, slice_type_of(out[k], len[k]),
			30 * ctbs(full, sps.ctb_log2), &h, &why);
		snprintf(name, sizeof(name), "%s strip %d: address %u", tag, k,
			off * 30 / 64);
		failures += expect(name, ok && h.first == (k == 0) &&
			h.addr == off * 30 / 64 && h.poc_lsb == want_poc, why);
		ok = parse_slice(b, olen, &m->sps[k], &m->pps[k],
			slice_type_of(au[k].data, au[k].len),
			30 * ctbs(heights[k], sps.ctb_log2), &h0, &why);
		snprintf(name, sizeof(name), "%s strip %d: slice data untouched", tag,
			k);
		failures += expect(name, ok && mlen - h.data_byte ==
			olen - h0.data_byte && memcmp(a + h.data_byte,
			b + h0.data_byte, olen - h0.data_byte) == 0, NULL);
		off += heights[k];
	}
	return failures;
}

static int test_two(void)
{
	static const uint32_t heights[2] = { 576, 504 };
	Au f0[2] = { { fx_two_s0_f0, sizeof(fx_two_s0_f0) },
		{ fx_two_s1_f0, sizeof(fx_two_s1_f0) } };
	Au f1[2] = { { fx_two_s0_f1, sizeof(fx_two_s0_f1) },
		{ fx_two_s1_f1, sizeof(fx_two_s1_f1) } };
	Ar9311Merge m;
	int failures;

	ar9311_merge_init(&m, 2, heights, 0);
	failures = check_frame("2 strips IDR", &m, 2, heights, f0, 0);
	failures += check_frame("2 strips P", &m, 2, heights, f1, (uint32_t)-1);
	ar9311_merge_free(&m);
	return failures;
}

static int test_three(void)
{
	static const uint32_t heights[3] = { 384, 384, 312 };
	Au f0[3] = { { fx_three_s0_f0, sizeof(fx_three_s0_f0) },
		{ fx_three_s1_f0, sizeof(fx_three_s1_f0) },
		{ fx_three_s2_f0, sizeof(fx_three_s2_f0) } };
	Au f1[3] = { { fx_three_s0_f1, sizeof(fx_three_s0_f1) },
		{ fx_three_s1_f1, sizeof(fx_three_s1_f1) },
		{ fx_three_s2_f1, sizeof(fx_three_s2_f1) } };
	Ar9311Merge m;
	int failures;

	ar9311_merge_init(&m, 3, heights, 0);
	failures = check_frame("3 strips IDR", &m, 3, heights, f0, 0);
	failures += check_frame("3 strips P", &m, 3, heights, f1, (uint32_t)-1);
	ar9311_merge_free(&m);
	return failures;
}

/* A strip whose encoder codes another height than planned is reported
 * (the caller asks for an IDR) and the plan follows it. */
static int test_learned_geometry(void)
{
	static const uint32_t wrong[2] = { 576, 512 };
	Ar9311Merge m;
	uint8_t out[4096];
	int failures = 0;

	ar9311_merge_init(&m, 2, wrong, 0);
	failures += expect("geometry: strip 0 joins",
		ar9311_merge_strip(&m, 0, fx_two_s0_f0, sizeof(fx_two_s0_f0), out,
			sizeof(out)) > 0, m.error);
	failures += expect("geometry: wrong height reported",
		ar9311_merge_strip(&m, 1, fx_two_s1_f0, sizeof(fx_two_s1_f0), out,
			sizeof(out)) == 0 && m.error && strstr(m.error, "learned"),
		m.error);
	failures += expect("geometry: learned", m.coded_h[1] == 504, NULL);
	ar9311_merge_free(&m);
	return failures;
}

/* Strips from different pictures are out of step: an IDR top over a P
 * bottom must not reach the receiver as one picture. */
static int test_out_of_step(void)
{
	static const uint32_t heights[2] = { 576, 504 };
	Ar9311Merge m;
	uint8_t out[4096];
	int failures = 0;

	ar9311_merge_init(&m, 2, heights, 0);
	(void)ar9311_merge_strip(&m, 0, fx_two_s0_f0, sizeof(fx_two_s0_f0), out,
		sizeof(out));
	(void)ar9311_merge_strip(&m, 1, fx_two_s1_f0, sizeof(fx_two_s1_f0), out,
		sizeof(out));
	(void)ar9311_merge_strip(&m, 0, fx_two_s0_f1, sizeof(fx_two_s0_f1), out,
		sizeof(out));
	(void)ar9311_merge_strip(&m, 1, fx_two_s1_f0, sizeof(fx_two_s1_f0), out,
		sizeof(out));
	failures += expect("out of step detected", !ar9311_merge_in_step(&m),
		NULL);
	ar9311_merge_free(&m);
	return failures;
}

/* Without overlap rows, a PPS that filters across slice boundaries would
 * blur every seam the encoders never filtered: refused. */
static int test_lf_across_refused(void)
{
	static const uint32_t heights[2] = { 576, 504 };
	uint8_t au[sizeof(fx_two_s0_f0)], out[4096], rbsp[512], pps_rbsp[512];
	uint8_t built[sizeof(fx_two_s0_f0) + 64];
	size_t cursor = 0, nal_len, pos = 0, rlen;
	const uint8_t *nal;
	Ar9311HevcPps pps;
	Ar9311Merge m;
	int failures = 0;

	memcpy(au, fx_two_s0_f0, sizeof(au));
	/* rebuild the access unit with the PPS's loop filter flag set */
	while (next_nal(au, sizeof(au), &cursor, &nal, &nal_len)) {
		if (((nal[0] >> 1) & 0x3f) == 34) {
			Br r;
			Bw w = { pps_rbsp, sizeof(pps_rbsp), 0, 0 };

			rlen = unescape(nal, nal_len, rbsp);
			if (!parse_pps(rbsp, rlen, &pps))
				return expect("lf: fixture PPS parses", 0, NULL);
			/* walk to pps_loop_filter_across_slices_enabled_flag */
			r.p = rbsp;
			r.bits = rlen * 8;
			r.pos = 16;
			r.err = 0;
			br_ue(&r); br_ue(&r); br_u(&r, 1); br_u(&r, 1); br_u(&r, 3);
			br_u(&r, 1); br_u(&r, 1); br_ue(&r); br_ue(&r); br_se(&r);
			br_u(&r, 1); br_u(&r, 1);
			if (br_u(&r, 1))
				br_ue(&r);
			br_se(&r); br_se(&r); br_u(&r, 1); br_u(&r, 1); br_u(&r, 1);
			br_u(&r, 1); br_u(&r, 1); br_u(&r, 1);
			bw_copy(&w, rbsp, 0, r.pos);
			bw_u(&w, 1, 1);
			bw_copy(&w, rbsp, r.pos + 1, rlen * 8);
			(void)put_nal(pps_rbsp, w.pos >> 3, built, sizeof(built), &pos);
			continue;
		}
		(void)put_raw(nal, nal_len, built, sizeof(built), &pos);
	}
	ar9311_merge_init(&m, 2, heights, 0);
	failures += expect("lf across slices refused",
		ar9311_merge_strip(&m, 0, built, pos, out, sizeof(out)) == 0 &&
		m.error && strstr(m.error, "loop filter"), m.error);
	ar9311_merge_free(&m);
	return failures;
}

/* The slice NALs of an Annex-B buffer, in order. */
static int slices_of(const uint8_t *buf, size_t len, const uint8_t **nal,
	size_t *nal_len, int max)
{
	size_t cursor = 0, l;
	const uint8_t *p;
	int n = 0;

	while (next_nal(buf, len, &cursor, &p, &l))
		if (((p[0] >> 1) & 0x3f) < 32 && n < max) {
			nal[n] = p;
			nal_len[n++] = l;
		}
	return n;
}

/* video0.stripOverlap 1 on two strips: strip 0 (9 + 1 rows) keeps its first
 * slice as it is and loses the second, strip 1 (1-row slices from line 512)
 * loses its first and the other 8 move to CTU rows 9..16, slice data
 * untouched; the PPS filters across slices and passes. */
static int test_overlap(void)
{
	static const uint32_t heights[2] = { 640, 568 };
	static const Au fr[2][2] = {
		{ { fx_ovl_s0_f0, sizeof(fx_ovl_s0_f0) },
		  { fx_ovl_s1_f0, sizeof(fx_ovl_s1_f0) } },
		{ { fx_ovl_s0_f1, sizeof(fx_ovl_s0_f1) },
		  { fx_ovl_s1_f1, sizeof(fx_ovl_s1_f1) } } };
	static uint8_t out[4096], a[512], b[512];
	Ar9311HevcSps sps;
	Ar9311HevcPps pps;
	Ar9311Merge m;
	char name[128];
	int failures = 0, f, k, i;

	memset(&sps, 0, sizeof(sps));
	memset(&pps, 0, sizeof(pps));
	ar9311_merge_init(&m, 2, heights, 1);
	for (f = 0; f < 2; f++) {
		for (k = 0; k < 2; k++) {
			const uint8_t *in_s[16], *out_s[16];
			size_t in_l[16], out_l[16];
			size_t len = ar9311_merge_strip(&m, k, fr[f][k].data,
				fr[f][k].len, out, sizeof(out));
			const char *why = NULL;
			int nin, nout, ok = 1;

			snprintf(name, sizeof(name), "overlap frame %d strip %d joined",
				f, k);
			failures += expect(name, len > 0, m.error);
			if (!len)
				continue;
			if (f == 0 && k == 0) {
				size_t sl = find_rbsp(out, len, 33, a, 0);
				size_t pl = find_rbsp(out, len, 34, b, 0);

				failures += expect("overlap SPS 1080 lines",
					sl && parse_sps(a, sl, &sps) && sps.height == 1080,
					NULL);
				failures += expect("overlap PPS filters across slices",
					pl && parse_pps(b, pl, &pps) && pps.lf_across_slices,
					NULL);
			}
			nin = slices_of(fr[f][k].data, fr[f][k].len, in_s, in_l, 16);
			nout = slices_of(out, len, out_s, out_l, 16);
			snprintf(name, sizeof(name), "overlap frame %d strip %d: %d of "
				"%d slices kept", f, k, nout, nin);
			failures += expect(name, nin == (k ? 9 : 2) &&
				nout == (k ? 8 : 1), NULL);
			for (i = 0; i < nout && i < nin - 1 && ok; i++) {
				const uint8_t *src = in_s[k ? i + 1 : i];
				size_t src_len = in_l[k ? i + 1 : i];
				uint32_t type = (src[0] >> 1) & 0x3f;
				size_t mlen = unescape(out_s[i], out_l[i], a);
				size_t olen = unescape(src, src_len, b);
				SliceHdr h, h0;

				ok = parse_slice(a, mlen, &sps, &pps, type, 30 * 17, &h,
					&why) &&
				     parse_slice(b, olen, &m.sps[k], &m.pps[k], type,
					30 * ctbs(m.sps[k].height, 6), &h0, &why) &&
				     h.first == (k == 0 && i == 0) &&
				     h.addr == (k ? 270u + 30u * (uint32_t)i : 0u) &&
				     h.poc_lsb == m.poc[0] &&
				     mlen - h.data_byte == olen - h0.data_byte &&
				     memcmp(a + h.data_byte, b + h0.data_byte,
					olen - h0.data_byte) == 0;
			}
			snprintf(name, sizeof(name), "overlap frame %d strip %d: "
				"addresses, POC, slice data", f, k);
			failures += expect(name, ok, why);
		}
		snprintf(name, sizeof(name), "overlap frame %d in step", f);
		failures += expect(name, ar9311_merge_in_step(&m), NULL);
	}
	ar9311_merge_free(&m);
	return failures;
}

/* With overlap rows, a strip whose slice runs across its seam would carry
 * them into the picture: refused. */
static int test_overlap_needs_slices(void)
{
	static const uint32_t heights[2] = { 576, 504 };
	uint8_t out[4096];
	Ar9311Merge m;
	int failures;

	ar9311_merge_init(&m, 2, heights, 1);
	failures = expect("overlap rows inside a kept slice refused",
		ar9311_merge_strip(&m, 0, fx_two_s0_f0, sizeof(fx_two_s0_f0), out,
			sizeof(out)) == 0 && m.error && strstr(m.error, "overlap"),
		m.error);
	ar9311_merge_free(&m);
	return failures;
}

int main(void)
{
	int failures = test_two() + test_three() + test_learned_geometry() +
		test_out_of_step() + test_lf_across_refused() + test_overlap() +
		test_overlap_needs_slices();

	printf("ar9311 merge: %s\n", failures ? "FAILED" : "ok");
	return failures ? 1 : 0;
}
