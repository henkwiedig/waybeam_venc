#ifndef AR9311_RUNTIME_H
#define AR9311_RUNTIME_H

#include "backend.h"

#include <stdint.h>

/** The AR9311 backend: MissingLynk V4L2 camera -> wave5 H.265 -> the shared
 *  RTP / frame-shm outputs.  See docs/AR9311_BACKEND.md. */
const BackendOps *ar9311_runtime_backend_ops(void);

/* Intra refresh as applied, for /api/v1/intra/status and
 * /api/v1/resilience/status (same fields as the other backends). */
typedef struct {
	char mode_name[16];
	int active;
	int mi_supported;
	int apply_ok;
	uint32_t target_ms;
	uint32_t total_rows;
	uint32_t requested_lines;
	uint32_t effective_lines_per_p;
	int lines_clamped;
	uint32_t requested_qp;
	uint32_t effective_qp;
	double explicit_gop_sec;
	double effective_gop_sec;
	int gop_auto;
} Ar9311IntraRefreshStatus;

void ar9311_runtime_intra_refresh_status(Ar9311IntraRefreshStatus *out);

#endif /* AR9311_RUNTIME_H */
