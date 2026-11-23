#include "ar9311_merge.h"

#include <stdlib.h>
#include <string.h>

/* ── Bit I/O over RBSP (emulation prevention removed) ─────────────────── */

typedef struct {
	const uint8_t *p;
	size_t bits;
	size_t pos;
	int err;
} Br;

static uint32_t br_u(Br *b, unsigned n)
{
	uint32_t v = 0;

	while (n--) {
		if (b->pos >= b->bits) {
			b->err = 1;
			return 0;
		}
		v = (v << 1) | ((b->p[b->pos >> 3] >> (7 - (b->pos & 7))) & 1u);
		b->pos++;
	}
	return v;
}

static uint32_t br_ue(Br *b)
{
	unsigned lz = 0;

	while (!br_u(b, 1)) {
		if (b->err || ++lz > 31) {
			b->err = 1;
			return 0;
		}
	}
	return lz ? ((1u << lz) - 1u) + br_u(b, lz) : 0;
}

static int32_t br_se(Br *b)
{
	uint32_t k = br_ue(b);

	return (k & 1) ? (int32_t)((k + 1) / 2) : -(int32_t)(k / 2);
}

typedef struct {
	uint8_t *p;
	size_t cap;     /* bytes */
	size_t pos;     /* bits */
	int err;
} Bw;

static void bw_bit(Bw *w, unsigned bit)
{
	size_t byte = w->pos >> 3;

	if (byte >= w->cap) {
		w->err = 1;
		return;
	}
	if ((w->pos & 7) == 0)
		w->p[byte] = 0;
	if (bit)
		w->p[byte] |= (uint8_t)(0x80u >> (w->pos & 7));
	w->pos++;
}

static void bw_u(Bw *w, uint32_t v, unsigned n)
{
	while (n--)
		bw_bit(w, (v >> n) & 1u);
}

static void bw_ue(Bw *w, uint32_t v)
{
	uint64_t x = (uint64_t)v + 1, t = x;
	unsigned n = 0;

	while (t) {
		n++;
		t >>= 1;
	}
	bw_u(w, 0, n - 1);
	while (n--)
		bw_bit(w, (unsigned)((x >> n) & 1u));
}

static void bw_copy(Bw *w, const uint8_t *src, size_t from, size_t to)
{
	while (from < to) {
		if ((from & 7) == 0 && (w->pos & 7) == 0 && to - from >= 8) {
			size_t n = (to - from) >> 3;

			if ((w->pos >> 3) + n > w->cap) {
				w->err = 1;
				return;
			}
			memcpy(w->p + (w->pos >> 3), src + (from >> 3), n);
			w->pos += n * 8;
			from += n * 8;
			continue;
		}
		bw_bit(w, (src[from >> 3] >> (7 - (from & 7))) & 1u);
		from++;
	}
}

static void bw_trailing(Bw *w)
{
	bw_bit(w, 1);
	while (w->pos & 7)
		bw_bit(w, 0);
}

static unsigned ceil_log2(uint32_t x)
{
	unsigned n = 0;

	while ((1ull << n) < x)
		n++;
	return n;
}

/* ── NAL <-> RBSP ─────────────────────────────────────────────────────── */

static size_t unescape(const uint8_t *src, size_t len, uint8_t *dst)
{
	size_t i, o = 0;
	unsigned zeros = 0;

	for (i = 0; i < len; i++) {
		if (zeros >= 2 && src[i] == 3) {
			zeros = 0;
			continue;
		}
		dst[o++] = src[i];
		zeros = src[i] ? 0 : zeros + 1;
	}
	return o;
}

/* Escaped NAL with a 4-byte start code appended at out[*pos]. */
static int put_nal(const uint8_t *rbsp, size_t len, uint8_t *out, size_t cap,
	size_t *pos)
{
	size_t i, o = *pos;
	unsigned zeros = 0;

	if (o + 4 > cap)
		return -1;
	out[o++] = 0;
	out[o++] = 0;
	out[o++] = 0;
	out[o++] = 1;
	for (i = 0; i < len; i++) {
		if (zeros >= 2 && rbsp[i] <= 3) {
			if (o >= cap)
				return -1;
			out[o++] = 3;
			zeros = 0;
		}
		if (o >= cap)
			return -1;
		out[o++] = rbsp[i];
		zeros = rbsp[i] ? 0 : zeros + 1;
	}
	if (out[o - 1] == 0) {
		if (o >= cap)
			return -1;
		out[o++] = 3;
	}
	*pos = o;
	return 0;
}

static int put_raw(const uint8_t *nal, size_t len, uint8_t *out, size_t cap,
	size_t *pos)
{
	if (*pos + 4 + len > cap)
		return -1;
	memcpy(out + *pos, "\0\0\0\1", 4);
	memcpy(out + *pos + 4, nal, len);
	*pos += 4 + len;
	return 0;
}

/* Annex-B: the next NAL after *cursor, without its start code or trailing
 * zero bytes. */
static int next_nal(const uint8_t *buf, size_t len, size_t *cursor,
	const uint8_t **nal, size_t *nal_len)
{
	size_t i = *cursor, start;

	while (i + 3 <= len && !(buf[i] == 0 && buf[i + 1] == 0 &&
		buf[i + 2] == 1))
		i++;
	if (i + 3 > len)
		return 0;
	start = i + 3;
	i = start;
	while (i + 3 <= len && !(buf[i] == 0 && buf[i + 1] == 0 &&
		buf[i + 2] == 1))
		i++;
	if (i + 3 > len)
		i = len;
	*nal = buf + start;
	*nal_len = i - start;
	while (*nal_len && (*nal)[*nal_len - 1] == 0)
		(*nal_len)--;
	*cursor = i;
	return *nal_len >= 2;
}

/* ── Parameter sets ───────────────────────────────────────────────────── */

static int parse_ptl(Br *b, uint32_t msl)
{
	uint32_t i, prof[8] = { 0 }, lvl[8] = { 0 };

	br_u(b, 32);
	br_u(b, 32);
	br_u(b, 24);	/* 88 bits of general profile */
	br_u(b, 8);	/* general_level_idc */
	for (i = 0; i < msl; i++) {
		prof[i] = br_u(b, 1);
		lvl[i] = br_u(b, 1);
	}
	if (msl > 0)
		for (i = msl; i < 8; i++)
			br_u(b, 2);
	for (i = 0; i < msl; i++) {
		if (prof[i]) {
			br_u(b, 32);
			br_u(b, 32);
			br_u(b, 24);
		}
		if (lvl[i])
			br_u(b, 8);
	}
	return !b->err;
}

static int parse_st_rps(Br *b, uint32_t idx, uint32_t num,
	const uint32_t *ndp, uint32_t *out_ndp, uint32_t *out_used)
{
	uint32_t used_n = 0;

	uint32_t inter = 0, i;

	if (idx != 0)
		inter = br_u(b, 1);
	if (inter) {
		uint32_t delta_idx_minus1 = 0, ref, n = 0;

		if (idx == num)
			delta_idx_minus1 = br_ue(b);
		if (delta_idx_minus1 + 1 > idx)
			return 0;
		ref = idx - (delta_idx_minus1 + 1);
		br_u(b, 1);
		br_ue(b);
		for (i = 0; i <= ndp[ref]; i++) {
			uint32_t used = br_u(b, 1), use_delta = 1;

			if (!used)
				use_delta = br_u(b, 1);
			if (used || use_delta)
				n++;
			used_n += used;
		}
		*out_ndp = n;
		*out_used = used_n;
	} else {
		uint32_t neg = br_ue(b), pos = br_ue(b);

		if (neg > 16 || pos > 16)
			return 0;
		for (i = 0; i < neg + pos; i++) {
			br_ue(b);
			used_n += br_u(b, 1);
		}
		*out_ndp = neg + pos;
		*out_used = used_n;
	}
	return !b->err;
}

static size_t stop_bit_of(const uint8_t *rbsp, size_t len)
{
	size_t n = len;
	unsigned k = 0;

	while (n && rbsp[n - 1] == 0)
		n--;
	if (!n)
		return 0;
	while (!(rbsp[n - 1] & (1u << k)))
		k++;
	return (n - 1) * 8 + (7 - k);
}

static int parse_sps(const uint8_t *rbsp, size_t len, Ar9311HevcSps *s)
{
	Br b = { rbsp, len * 8, 16, 0 };
	uint32_t msl, i, present;

	memset(s, 0, sizeof(*s));
	br_u(&b, 4);
	msl = br_u(&b, 3);
	br_u(&b, 1);
	if (!parse_ptl(&b, msl))
		return 0;
	br_ue(&b);
	s->chroma_format_idc = br_ue(&b);
	if (s->chroma_format_idc == 3)
		s->separate_colour_plane = br_u(&b, 1);
	s->width_bit = b.pos;
	s->width = br_ue(&b);
	s->height = br_ue(&b);
	s->conf_flag = br_u(&b, 1);
	if (s->conf_flag)
		for (i = 0; i < 4; i++)
			s->conf[i] = br_ue(&b);
	s->after_conf_bit = b.pos;
	br_ue(&b);
	br_ue(&b);
	s->log2_max_poc_lsb = br_ue(&b) + 4;
	present = br_u(&b, 1);
	for (i = present ? 0 : msl; i <= msl; i++) {
		br_ue(&b);
		br_ue(&b);
		br_ue(&b);
	}
	s->ctb_log2 = br_ue(&b) + 3;
	s->ctb_log2 += br_ue(&b);
	br_ue(&b);
	br_ue(&b);
	br_ue(&b);
	br_ue(&b);
	if (br_u(&b, 1) && br_u(&b, 1))
		return 0;	/* scaling list data: not handled */
	br_u(&b, 1);
	s->sao = br_u(&b, 1);
	if (br_u(&b, 1)) {
		br_u(&b, 4);
		br_u(&b, 4);
		br_ue(&b);
		br_ue(&b);
		br_u(&b, 1);
	}
	s->num_st_rps = br_ue(&b);
	if (s->num_st_rps > 64)
		return 0;
	for (i = 0; i < s->num_st_rps; i++)
		if (!parse_st_rps(&b, i, s->num_st_rps, s->num_delta_pocs,
			&s->num_delta_pocs[i], &s->num_used_curr[i]))
			return 0;
	s->long_term_present = br_u(&b, 1);
	if (s->long_term_present) {
		s->num_lt_sps = br_ue(&b);
		if (s->num_lt_sps > 32)
			return 0;
		for (i = 0; i < s->num_lt_sps; i++) {
			br_u(&b, s->log2_max_poc_lsb);
			br_u(&b, 1);
		}
	}
	s->temporal_mvp = br_u(&b, 1);
	s->stop_bit = stop_bit_of(rbsp, len);
	s->valid = !b.err && s->ctb_log2 >= 4 && s->ctb_log2 <= 6 &&
		s->stop_bit > b.pos;
	return s->valid;
}

static int parse_pps(const uint8_t *rbsp, size_t len, Ar9311HevcPps *p)
{
	Br b = { rbsp, len * 8, 16, 0 };

	memset(p, 0, sizeof(*p));
	p->pps_id = br_ue(&b);
	br_ue(&b);
	p->dependent_slices = br_u(&b, 1);
	p->output_flag_present = br_u(&b, 1);
	p->num_extra_bits = br_u(&b, 3);
	br_u(&b, 1);
	p->cabac_init_present = br_u(&b, 1);
	p->num_ref_l0_def = br_ue(&b);
	p->num_ref_l1_def = br_ue(&b);
	p->init_qp_minus26 = br_se(&b);
	br_u(&b, 1);
	br_u(&b, 1);
	if (br_u(&b, 1))
		br_ue(&b);
	br_se(&b);
	br_se(&b);
	p->slice_chroma_qp_offsets_present = br_u(&b, 1);
	p->weighted_pred = br_u(&b, 1);
	p->weighted_bipred = br_u(&b, 1);
	br_u(&b, 1);
	p->tiles = br_u(&b, 1);
	p->wpp = br_u(&b, 1);
	if (p->tiles)
		return 0;	/* tiles: not handled */
	p->lf_across_slices = br_u(&b, 1);
	if (br_u(&b, 1)) {
		p->deblock_override_enabled = br_u(&b, 1);
		p->deblock_disabled = br_u(&b, 1);
		if (!p->deblock_disabled) {
			br_se(&b);
			br_se(&b);
		}
	}
	if (br_u(&b, 1))
		return 0;	/* scaling list data: not handled */
	p->lists_mod_present = br_u(&b, 1);
	br_ue(&b);
	p->header_ext_present = br_u(&b, 1);
	if (br_u(&b, 1))
		return 0;	/* PPS extensions: not handled */
	p->valid = !b.err;
	return p->valid;
}

/* ── Slice segment header ─────────────────────────────────────────────── */

typedef struct {
	uint32_t first, no_output, pps_id, addr, slice_type, poc_lsb, idr;
	uint32_t dependent;
	int32_t qp_delta;        /* slice_qp_delta */
	size_t after_addr_bit;   /* first bit after slice_segment_address */
	size_t end_bit;          /* first bit of byte_alignment() */
	size_t data_byte;        /* slice data start */
} SliceHdr;

static int parse_slice(const uint8_t *rbsp, size_t len,
	const Ar9311HevcSps *s, const Ar9311HevcPps *p, uint32_t nal_type,
	uint32_t pic_ctbs, SliceHdr *h, const char **why)
{
	Br b = { rbsp, len * 8, 16, 0 };
	uint32_t dependent = 0, i, sao_l = 0, sao_c = 0, tmvp = 0, curr = 0;
	uint32_t dbk_off = p->deblock_disabled;

	memset(h, 0, sizeof(*h));
	h->idr = nal_type == 19 || nal_type == 20;
	h->first = br_u(&b, 1);
	if (nal_type >= 16 && nal_type <= 23)
		h->no_output = br_u(&b, 1);
	h->pps_id = br_ue(&b);
	if (!h->first) {
		if (p->dependent_slices)
			dependent = br_u(&b, 1);
		h->addr = br_u(&b, ceil_log2(pic_ctbs));
	}
	h->dependent = dependent;
	h->after_addr_bit = b.pos;
	if (!dependent) {
		br_u(&b, p->num_extra_bits);
		h->slice_type = br_ue(&b);
		if (p->output_flag_present)
			br_u(&b, 1);
		if (s->separate_colour_plane)
			br_u(&b, 2);
		if (!h->idr) {
			h->poc_lsb = br_u(&b, s->log2_max_poc_lsb);
			if (!br_u(&b, 1)) {
				uint32_t ndp;

				if (!parse_st_rps(&b, s->num_st_rps, s->num_st_rps,
					s->num_delta_pocs, &ndp, &curr)) {
					*why = "slice RPS";
					return 0;
				}
			} else {
				uint32_t idx = 0;

				if (s->num_st_rps > 1)
					idx = br_u(&b, ceil_log2(s->num_st_rps));
				if (idx >= s->num_st_rps) {
					*why = "slice RPS index";
					return 0;
				}
				curr = s->num_used_curr[idx];
			}
			if (s->long_term_present) {
				uint32_t nsps = 0, npics;

				if (s->num_lt_sps > 0)
					nsps = br_ue(&b);
				npics = br_ue(&b);
				if (nsps + npics > 32) {
					*why = "long-term refs";
					return 0;
				}
				for (i = 0; i < nsps + npics; i++) {
					if (i < nsps) {
						/* used flag lives in the SPS: not tracked */
						if (s->num_lt_sps > 1)
							br_u(&b, ceil_log2(s->num_lt_sps));
					} else {
						br_u(&b, s->log2_max_poc_lsb);
						curr += br_u(&b, 1);
					}
					if (br_u(&b, 1))
						br_ue(&b);
				}
			}
			if (s->temporal_mvp)
				tmvp = br_u(&b, 1);
		}
		if (s->sao) {
			sao_l = br_u(&b, 1);
			if (s->chroma_format_idc != 0 && !s->separate_colour_plane)
				sao_c = br_u(&b, 1);
		}
		if (h->slice_type == 0 || h->slice_type == 1) {
			uint32_t l0 = p->num_ref_l0_def, l1 = p->num_ref_l1_def;

			if (br_u(&b, 1)) {
				l0 = br_ue(&b);
				if (h->slice_type == 0)
					l1 = br_ue(&b);
			}
			if (p->lists_mod_present && curr > 1) {
				unsigned v = ceil_log2(curr);

				if (br_u(&b, 1))
					for (i = 0; i <= l0; i++)
						br_u(&b, v);
				if (h->slice_type == 0 && br_u(&b, 1))
					for (i = 0; i <= l1; i++)
						br_u(&b, v);
			}
			if (h->slice_type == 0)
				br_u(&b, 1);
			if (p->cabac_init_present)
				br_u(&b, 1);
			if (tmvp) {
				uint32_t from_l0 = 1;

				if (h->slice_type == 0)
					from_l0 = br_u(&b, 1);
				if ((from_l0 && l0 > 0) || (!from_l0 && l1 > 0))
					br_ue(&b);
			}
			if ((p->weighted_pred && h->slice_type == 1) ||
			    (p->weighted_bipred && h->slice_type == 0)) {
				*why = "weighted prediction";
				return 0;
			}
			br_ue(&b);
		}
		h->qp_delta = br_se(&b);
		if (p->slice_chroma_qp_offsets_present) {
			br_se(&b);
			br_se(&b);
		}
		if (p->deblock_override_enabled && br_u(&b, 1)) {
			dbk_off = br_u(&b, 1);
			if (!dbk_off) {
				br_se(&b);
				br_se(&b);
			}
		}
		if (p->lf_across_slices && (sao_l || sao_c || !dbk_off))
			br_u(&b, 1);
	}
	if (p->tiles || p->wpp) {
		uint32_t n = br_ue(&b);

		if (n > 0) {
			uint32_t bits = br_ue(&b) + 1;

			if (bits > 32) {
				*why = "entry points";
				return 0;
			}
			for (i = 0; i < n; i++)
				br_u(&b, bits);
		}
	}
	if (p->header_ext_present) {
		uint32_t n = br_ue(&b);

		for (i = 0; i < n; i++)
			br_u(&b, 8);
	}
	h->end_bit = b.pos;
	if (br_u(&b, 1) != 1) {
		*why = "slice header misparsed (no alignment bit)";
		return 0;
	}
	while (b.pos & 7)
		if (br_u(&b, 1) != 0) {
			*why = "slice header misparsed (alignment)";
			return 0;
		}
	h->data_byte = b.pos >> 3;
	if (b.err) {
		*why = "slice header truncated";
		return 0;
	}
	return 1;
}

/* ── Merge ────────────────────────────────────────────────────────────── */

void ar9311_merge_init(Ar9311Merge *m, int n, const uint32_t *heights,
	uint32_t overlap_rows)
{
	int k;

	memset(m, 0, sizeof(*m));
	m->n = n;
	m->overlap_rows = overlap_rows;
	for (k = 0; k < n && k < AR9311_MERGE_MAX_STRIPS; k++) {
		m->coded_h[k] = heights[k];
		m->poc[k] = 0xffffffffu;
	}
}

void ar9311_merge_free(Ar9311Merge *m)
{
	free(m->rbsp);
	free(m->out_rbsp);
	m->rbsp = m->out_rbsp = NULL;
	m->scratch_len = 0;
}

static int scratch(Ar9311Merge *m, size_t len)
{
	uint8_t *a, *b;

	if (len + 64 <= m->scratch_len)
		return 0;
	a = realloc(m->rbsp, len + 64);
	if (!a)
		return -1;
	m->rbsp = a;
	b = realloc(m->out_rbsp, len + 64);
	if (!b)
		return -1;
	m->out_rbsp = b;
	m->scratch_len = len + 64;
	return 0;
}

static uint32_t ctbs(uint32_t px, uint32_t ctb_log2)
{
	return (px + (1u << ctb_log2) - 1) >> ctb_log2;
}

/* CTU rows strip @p k codes past its seams, both together. */
static uint32_t overlap_of(const Ar9311Merge *m, int k)
{
	return (k > 0 ? m->overlap_rows : 0) +
		(k < m->n - 1 ? m->overlap_rows : 0);
}

/* The picture's height: the lines the strips send. */
static uint32_t full_height(const Ar9311Merge *m)
{
	uint32_t h = 0;
	int k;

	for (k = 0; k < m->n; k++)
		h += m->coded_h[k] - (overlap_of(m, k) << m->sps[0].ctb_log2);
	return h;
}

/* Strip 0's SPS with the whole picture's height. */
static int rewrite_sps(Ar9311Merge *m, size_t rlen, uint8_t *out, size_t cap,
	size_t *pos)
{
	const Ar9311HevcSps *s = &m->sps[0];
	Bw w = { m->out_rbsp, m->scratch_len, 0, 0 };

	(void)rlen;
	bw_copy(&w, m->rbsp, 0, s->width_bit);
	bw_ue(&w, s->width);
	bw_ue(&w, full_height(m));
	bw_u(&w, m->conf_bottom ? 1 : 0, 1);
	if (m->conf_bottom) {
		bw_ue(&w, 0);
		bw_ue(&w, 0);
		bw_ue(&w, 0);
		bw_ue(&w, m->conf_bottom);
	}
	bw_copy(&w, m->rbsp, s->after_conf_bit, s->stop_bit);
	bw_trailing(&w);
	return w.err ? -1 : put_nal(m->out_rbsp, w.pos >> 3, out, cap, pos);
}

size_t ar9311_merge_strip(Ar9311Merge *m, int k, const uint8_t *au,
	size_t len, uint8_t *out, size_t cap)
{
	size_t cursor = 0, pos = 0, nal_len, rlen;
	const uint8_t *nal;
	int last = k == m->n - 1, kept = 0, cut = 0;

	m->error = NULL;
	if (k < 0 || k >= m->n)
		return m->error = "no such strip", 0;
	while (next_nal(au, len, &cursor, &nal, &nal_len)) {
		uint32_t type = (nal[0] >> 1) & 0x3f;

		if (scratch(m, nal_len) != 0)
			return m->error = "no memory", 0;
		if (type == 33) {
			Ar9311HevcSps *s = &m->sps[k];
			uint32_t cb;

			rlen = unescape(nal, nal_len, m->rbsp);
			if (!parse_sps(m->rbsp, rlen, s))
				return m->error = "SPS not understood", 0;
			cb = s->conf_flag ? s->conf[3] : 0;
			if ((s->conf_flag && (s->conf[0] | s->conf[1] | s->conf[2])) ||
			    (!last && (cb || (s->height & ((1u << s->ctb_log2) - 1)))))
				return m->error = "strip is not whole CTU rows", 0;
			if (s->height <= overlap_of(m, k) << s->ctb_log2)
				return m->error = "strip no taller than its overlap", 0;
			if (k > 0 && (s->width != m->sps[0].width ||
			    s->ctb_log2 != m->sps[0].ctb_log2 ||
			    s->log2_max_poc_lsb != m->sps[0].log2_max_poc_lsb ||
			    s->num_st_rps != m->sps[0].num_st_rps ||
			    s->temporal_mvp != m->sps[0].temporal_mvp ||
			    s->sao != m->sps[0].sao))
				return m->error = "SPS differs from strip 0's", 0;
			if (s->height != m->coded_h[k] ||
			    (last && cb != m->conf_bottom)) {
				/* strip 0's SPS said otherwise: wrong until the next
				 * IDR, which the caller asks for */
				m->coded_h[k] = s->height;
				if (last)
					m->conf_bottom = cb;
				return m->error = "strip geometry learned, need an IDR", 0;
			}
			if (k == 0) {
				if (rewrite_sps(m, rlen, out, cap, &pos) != 0)
					return m->error = "output full", 0;
			}
			continue;
		}
		if (type == 34) {
			rlen = unescape(nal, nal_len, m->rbsp);
			if (!parse_pps(m->rbsp, rlen, &m->pps[k]))
				return m->error = "PPS not understood", 0;
			/* with overlap rows the seam may be filtered: the rows
			 * either side are what both encoders filtered with */
			if (m->pps[k].lf_across_slices && !m->overlap_rows)
				return m->error = "loop filter crosses slices", 0;
			if (k == 0) {
				m->top_pps_len = rlen < AR9311_MERGE_MAX_PS ? rlen : 0;
				memcpy(m->top_pps_rbsp, m->rbsp, m->top_pps_len);
			} else {
				if (rlen != m->top_pps_len ||
				    memcmp(m->rbsp, m->top_pps_rbsp, rlen) != 0) {
					if (m->pps[k].init_qp_minus26 !=
					    m->pps[0].init_qp_minus26 ||
					    m->pps[k].pps_id != m->pps[0].pps_id ||
					    m->pps[k].dependent_slices !=
					    m->pps[0].dependent_slices)
						return m->error = "PPS differs from strip 0's", 0;
					if (!m->warned++)
						m->error = "PPS bytes differ (fields used match)";
				}
				continue;
			}
		}
		if (k > 0 && (type == 32 || type == 35))
			continue;	/* VPS, AUD: strip 0's stand */
		if (type < 32) {
			const Ar9311HevcSps *s = &m->sps[k];
			SliceHdr h;
			const char *why = "slice";
			uint32_t w_ctb, rows, keep0, keep1, addr, full_ctbs;
			int j;
			Bw w;

			if (!s->valid || !m->pps[k].valid || !m->sps[0].valid)
				return m->error = "slice before its SPS/PPS", 0;
			rlen = unescape(nal, nal_len, m->rbsp);
			w_ctb = ctbs(s->width, s->ctb_log2);
			rows = ctbs(s->height, s->ctb_log2);
			if (!parse_slice(m->rbsp, rlen, s, &m->pps[k], type,
				w_ctb * rows, &h, &why))
				return m->error = why, 0;
			if (h.dependent)
				return m->error = "dependent slice segment", 0;
			m->idr[k] = (int)h.idr;
			m->poc[k] = h.poc_lsb;
			/* The rows past the seams go.  The ones kept start and
			 * end on slice boundaries, or a kept slice would carry
			 * overlap rows into the picture. */
			keep0 = k > 0 ? w_ctb * m->overlap_rows : 0;
			keep1 = w_ctb * (rows - (last ? 0 : m->overlap_rows));
			if (h.addr < keep0)
				continue;
			if (h.addr >= keep1) {
				if (!cut++ && h.addr != keep1)
					return m->error = "a slice runs into the overlap", 0;
				continue;
			}
			if (!kept++) {
				if (h.addr != keep0)
					return m->error = "a slice runs into the overlap", 0;
				m->qp[k] = 26 + m->pps[k].init_qp_minus26 + h.qp_delta;
			}
			addr = h.addr - keep0;
			for (j = 0; j < k; j++)
				addr += w_ctb * (ctbs(m->coded_h[j], s->ctb_log2) -
					overlap_of(m, j));
			if (!addr) {
				/* the picture's first slice, as it is */
				if (put_raw(nal, nal_len, out, cap, &pos))
					return m->error = "output full", 0;
				continue;
			}
			full_ctbs = w_ctb * ctbs(full_height(m), s->ctb_log2);
			w.p = m->out_rbsp;
			w.cap = m->scratch_len;
			w.pos = 0;
			w.err = 0;
			bw_copy(&w, m->rbsp, 0, 16);	/* NAL header */
			bw_u(&w, 0, 1);			/* not the picture's first */
			if (type >= 16 && type <= 23)
				bw_u(&w, h.no_output, 1);
			bw_ue(&w, h.pps_id);
			if (m->pps[k].dependent_slices)
				bw_u(&w, 0, 1);		/* an independent slice */
			bw_u(&w, addr, ceil_log2(full_ctbs));
			bw_copy(&w, m->rbsp, h.after_addr_bit, h.end_bit);
			bw_trailing(&w);		/* byte_alignment() */
			bw_copy(&w, m->rbsp, h.data_byte * 8, rlen * 8);
			if (w.err || put_nal(m->out_rbsp, w.pos >> 3, out, cap, &pos))
				return m->error = "output full", 0;
			continue;
		}
		if (put_raw(nal, nal_len, out, cap, &pos))
			return m->error = "output full", 0;
	}
	if (!kept)
		return m->error = "strip without a slice", 0;
	if (!last && m->overlap_rows && !cut)
		return m->error = "a slice runs into the overlap", 0;
	return pos;
}

int ar9311_merge_in_step(const Ar9311Merge *m)
{
	int k;

	for (k = 1; k < m->n; k++)
		if (m->poc[k] != m->poc[0] || m->idr[k] != m->idr[0])
			return 0;
	return 1;
}
