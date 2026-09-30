#ifndef AR9311_RUNTIME_H
#define AR9311_RUNTIME_H

#include "backend.h"

/** The AR9311 backend: MissingLynk V4L2 camera -> wave5 H.265 -> the shared
 *  RTP / frame-shm outputs.  See docs/AR9311_BACKEND.md. */
const BackendOps *ar9311_runtime_backend_ops(void);

#endif /* AR9311_RUNTIME_H */
