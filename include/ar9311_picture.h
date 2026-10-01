#ifndef AR9311_PICTURE_H
#define AR9311_PICTURE_H

#include <stdint.h>

/* Picture controls of the AR9311 ISP: saturation, contrast and sharpness in
 * percent,
 * 100 = the tuning's own picture.  They are module parameters of the
 * MissingLynk ar_isp driver as patched by the OpenIPC builder (P1 kernel
 * patch 0002), re-latched live through its debugfs ladders and tone files.
 * Without the patch the parameters are missing and the apply fails. */

#define AR9311_PICTURE_NEUTRAL 100u
#define AR9311_PICTURE_MAX 200u

/** Write the controls and re-latch the ISP.  Only a control that differs
 *  from the last successful apply is written.  Returns 0 or -1. */
int ar9311_picture_apply(uint32_t saturation, uint32_t contrast,
	uint32_t sharpness);

#endif /* AR9311_PICTURE_H */
