/*
 * os02k10_cmos.h — OmniVision OS02K10 driver for HiSilicon Hi3516CV610
 *                  (hi3516cv6xx, V5 "OT" SDK, ss_* / ot_* MPI API).
 *
 * Ported from a full register-level reverse-engineering pass of the Caddx
 * Ascent air unit's stock firmware (`ar_ldyhs_sky`), not from a vendor SDK
 * sample — the vendor SDK checkout this build targets ships no OS02K10
 * driver at all. Full methodology, findings, and what's confirmed vs.
 * assumed: documentation/os02k10/OS02K10_ASCENT_REGISTER_RE.md.
 *
 * Confirmed against real hardware (2026-09-05): the static init sequence
 * (os02k10_cfg.h) was captured live via LD_PRELOAD on a booted, streaming
 * Ascent and matches byte-for-byte; the exposure/gain registers below were
 * confirmed the same way (they're the only ones observed to change while
 * streaming). Register addresses NOT listed here (analog/digital/DPC/LENC
 * blocks) are written verbatim from the recovered table but their bit-level
 * meaning is unknown — see os02k10_cfg.h and the RE doc.
 *
 * Build: compile in-tree under
 *   sensors/cv610/os02k10/ (this directory), see Makefile.
 */

#ifndef OS02K10_CMOS_H
#define OS02K10_CMOS_H

#include "ot_common.h"
#include "ot_common_isp.h"
#include "ot_common_video.h"
#include "ot_sns_ctrl.h"
#include "ot_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- I2C wiring -----------------------------------------------------------
 * Confirmed via os02k10_i2c_init() in ar_ldyhs_sky: ioctl(fd,
 * OT_I2C_SLAVE_FORCE, 0x36) with no shift — 0x36 is already the 7-bit
 * SCCB address (unlike IMX662's 8-bit-then->>1 convention).
 */
#define OS02K10_I2C_ADDR   0x36
#define OS02K10_ADDR_BYTE  2      /* 16-bit register address */
#define OS02K10_DATA_BYTE  1      /* 8-bit register data     */

/* ---- OS02K10 register map — only the credibly-decoded subset -------------
 * Everything else the init sequence writes (analog/PLL/MIPI/DPC/LENC
 * blocks) stays as raw {addr,data} pairs in os02k10_cfg.h; no bitfield
 * meaning is invented for those. See the RE doc §6 for the full reasoning.
 */
#define OS02K10_REG_SOFTWARE_RESET  0x0103  /* 0x01 = reset (self-clearing) */
#define OS02K10_REG_MODE_SELECT     0x0100  /* 0x00 standby, 0x01 streaming */
#define OS02K10_REG_EXPOSURE_HI     0x3501  /* coarse integration time, hi  */
#define OS02K10_REG_EXPOSURE_LO     0x3502  /* coarse integration time, lo  */
/* Gain register pair. CONFIRMED to be the AE-adjusted analog gain by a live
 * bench diff (only 0x3501/0x3502/0x3508 moved while streaming vs. the
 * recovered init table) — but the register's internal fixed-point format
 * was NOT independently confirmed (only one live sample: init 0x01/0x00 ->
 * streaming 0x04/0x00). cmos_again_calc_table() below assumes a Q8.8
 * (whole.fraction) split as the simplest reasonable guess; treat this as
 * unverified until measured against two or more known gain points. */
#define OS02K10_REG_GAIN_HI         0x3508
#define OS02K10_REG_GAIN_LO         0x3509
#define OS02K10_REG_HTS_HI          0x380c  /* line_length_pclk, hi (const) */
#define OS02K10_REG_HTS_LO          0x380d  /* line_length_pclk, lo (const) */
#define OS02K10_REG_VTS_HI          0x380e  /* frame_length_lines, hi       */
#define OS02K10_REG_VTS_LO          0x380f  /* frame_length_lines, lo       */
/* Mirror/flip: both values confirmed against the vendor's own decompiled
 * init table (ar_ldyhs_sky, the angle==0/angle==180 select right before
 * the FORMAT1 write). FORMAT1=0x02/FORMAT2=0x00 is normal (angle=0);
 * FORMAT1=0x0c/FORMAT2=0x00 is the vendor's only other state, a combined
 * 180-degree mirror+flip -- there is no independent single-axis mirror-
 * only or flip-only value anywhere in that table, so os02k10_mirror_flip()
 * below only implements those two states.
 *
 * Confirmed correct on hardware (both geometry and colour) with the
 * camera correctly mounted: an earlier bench session saw a ~90-degree-
 * looking result from this exact write and wrongly concluded the write
 * corrupted the MIPI RX capture path (see git history on this file for
 * the abandoned VPSS-level workaround) -- the real cause was the camera
 * having physically tipped over during testing, not this register. */
#define OS02K10_REG_FORMAT1         0x3820
#define OS02K10_REG_FORMAT2         0x3821

/* ---- exposure / gain model -------------------------------------------------
 * OmniVision SCCB convention: exposure is written directly as integration
 * time in lines (no VMAX-SHR0 subtraction the way Sony sensors do it).
 * Max integration time = VTS - 6: this is not a guess, it is the exact
 * relationship the vendor's own g_os02k10_mode_tbl encodes for all three
 * modes (VTS 2598/2165/1298 each paired with a "usable lines" field of
 * 2592/2159/1292 — see the RE doc §7).
 */
#define OS02K10_HTS_LINEAR          831u     /* 0x033f, constant across modes */
#define OS02K10_VTS_50FPS_LINEAR    2598u    /* 0x0a26 */
#define OS02K10_VTS_60FPS_LINEAR    2165u    /* 0x0875 — Ascent boot default */
#define OS02K10_VTS_100FPS_LINEAR   1298u    /* 0x0512 */
#define OS02K10_MIN_INT_TIME_MARGIN 6u       /* max_int_time = VTS - this   */
#define OS02K10_FULL_LINES_MAX      0xffffu  /* VTS is a 16-bit field       */

/* Gain bounds: UNVERIFIED placeholder (see OS02K10_REG_GAIN_HI comment
 * above). Assumes a Q8.8 linear code (256 == 1x) and a conservative 1x-16x
 * useful range pending real calibration. */
#define OS02K10_AGAIN_MIN           256u                 /* 1x  (assumed)   */
#define OS02K10_AGAIN_MAX           (256u * 16u)          /* 16x (assumed)  */

/* ---- resolution modes -----------------------------------------------------
 * All three linear-mode FPS profiles the vendor firmware compiles; no WDR
 * mode exists for this sensor in this firmware (cmos_set_wdr_mode() below
 * rejects everything but OT_WDR_MODE_NONE, same restriction IMX662 has).
 */
typedef enum {
	OS02K10_SENSOR_2M_50FPS_10BIT_LINEAR_MODE = 0,
	OS02K10_SENSOR_2M_60FPS_10BIT_LINEAR_MODE,
	OS02K10_SENSOR_2M_100FPS_10BIT_LINEAR_MODE,
	OS02K10_MODE_BUTT
} os02k10_res_mode;

typedef struct {
	td_u32      ver_lines;      /* this mode's real VTS                */
	td_u32      max_ver_lines;  /* FULL_LINES_MAX                      */
	td_float    max_fps;
	td_float    min_fps;
	td_u32      width;
	td_u32      height;
	td_u8       sns_mode;
	ot_wdr_mode wdr_mode;
	const char *mode_name;
} os02k10_video_mode_tbl;

/* ---- context accessors + transport (os02k10_sensor_ctl.c) ----------------- */
ot_isp_sns_state   *os02k10_get_ctx(ot_vi_pipe vi_pipe);
ot_isp_sns_commbus *os02k10_get_bus_info(ot_vi_pipe vi_pipe);

td_void os02k10_init(ot_vi_pipe vi_pipe);
td_void os02k10_exit(ot_vi_pipe vi_pipe);
td_void os02k10_standby(ot_vi_pipe vi_pipe);
td_void os02k10_restart(ot_vi_pipe vi_pipe);
td_s32  os02k10_write_register(ot_vi_pipe vi_pipe, td_u32 addr, td_u32 data);
td_s32  os02k10_read_register(ot_vi_pipe vi_pipe, td_u32 addr);
td_void os02k10_mirror_flip(ot_vi_pipe vi_pipe, ot_isp_sns_mirrorflip_type sns_mirror_flip);
td_s32  os02k10_i2c_init(ot_vi_pipe vi_pipe);
td_s32  os02k10_i2c_exit(ot_vi_pipe vi_pipe);

/* exported registration object (os02k10_cmos.c) */
extern ot_isp_sns_obj g_sns_os02k10_obj;

#ifdef __cplusplus
}
#endif
#endif /* OS02K10_CMOS_H */
