#ifndef AR9311_VALIDATION_H
#define AR9311_VALIDATION_H

#include "venc_config.h"

/* The one mode the AR9311 camera chain delivers today: the NT99235's
 * 2-lane 1920x1080p60, the only mode the open CVISP output is validated
 * for (MissingLynk ml-kernel docs/camera-stack.md). */
#define AR9311_WIDTH 1920
#define AR9311_HEIGHT 1080
#define AR9311_FPS 60

/** NULL when @p cfg is something this backend runs exactly as written,
 *  otherwise the reason it is not. */
const char *ar9311_validate_config(const VencConfig *cfg);

/** 1 for a field the backend reads only at start (a set needs a restart),
 *  0 for a field it applies live. */
int ar9311_field_is_restart_only(const char *field_key);

#endif /* AR9311_VALIDATION_H */
