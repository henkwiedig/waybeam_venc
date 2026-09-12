/*
 * os02k10_cmos.c — OS02K10 ISP-facing logic + registration for Hi3516CV610.
 *
 * Implements the public HiSilicon sensor-driver contract (ot_isp_sns_obj +
 * pfn_cmos_*), modeled file-for-file on sensors/cv610/imx662/imx662_cmos.c
 * but using OS02K10's own recovered register model: exposure is written
 * directly as integration time in lines (no Sony-style VMAX-SHR0
 * subtraction), and there is no confirmed dual-conversion-gain (HCG/LCG)
 * switch, so none is implemented.
 *
 * AE and gain calibration status: see os02k10_cmos.h and
 * documentation/os02k10/OS02K10_ASCENT_REGISTER_RE.md. The fast-update
 * register set (exposure hi/lo, gain hi/lo, VTS hi/lo) is confirmed by a
 * live bench diff; the gain register's internal fixed-point format is not.
 */

#include <math.h>
#include <stdio.h>
#include "securec.h"
#include "sensor_common.h"
#include "ot_mpi_isp.h"
#include "ot_mpi_ae.h"
#include "ot_mpi_awb.h"
#include "os02k10_cmos.h"
#include "os02k10_cmos_param.h"

#ifndef high_8bits
#define high_8bits(x)    (((x) >> 8) & 0xff)
#endif
#ifndef low_8bits
#define low_8bits(x)     ((x) & 0xff)
#endif

/* ---- per-pipe context ----------------------------------------------------- */
static ot_isp_sns_state   g_os02k10_state[OT_ISP_MAX_PIPE_NUM];
/* Static zero initialization selects I2C bus 0 for every pipe, matching the
 * recovered per-pipe bus table (pipe 0 -> bus 0). */
static ot_isp_sns_commbus g_os02k10_bus_info[OT_ISP_MAX_PIPE_NUM];
static td_u32 g_init_exposure[OT_ISP_MAX_PIPE_NUM] = {0};

ot_isp_sns_state *os02k10_get_ctx(ot_vi_pipe vi_pipe)
{
	return &g_os02k10_state[vi_pipe];
}
ot_isp_sns_commbus *os02k10_get_bus_info(ot_vi_pipe vi_pipe)
{
	return &g_os02k10_bus_info[vi_pipe];
}

/* ---- mode table -----------------------------------------------------------
 * All three recovered FPS profiles, all 1920x1080 linear. ver_lines is each
 * mode's own real VTS (not a shared placeholder) — see os02k10_cmos.h.
 * 60 FPS is listed default-first (index matches the Ascent's actual boot
 * configuration); order otherwise doesn't matter since selection is by fps
 * threshold in cmos_set_image_mode(), not by table position.
 */
static const os02k10_video_mode_tbl g_os02k10_mode_tbl[OS02K10_MODE_BUTT] = {
	{ OS02K10_VTS_50FPS_LINEAR, OS02K10_FULL_LINES_MAX, 50.0f, 5.0f,
	  1920, 1080, 0, OT_WDR_MODE_NONE, "OS02K10_2M_50FPS_10BIT_LINEAR" },
	{ OS02K10_VTS_60FPS_LINEAR, OS02K10_FULL_LINES_MAX, 60.0f, 5.0f,
	  1920, 1080, 1, OT_WDR_MODE_NONE, "OS02K10_2M_60FPS_10BIT_LINEAR" },
	{ OS02K10_VTS_100FPS_LINEAR, OS02K10_FULL_LINES_MAX, 100.0f, 5.0f,
	  1920, 1080, 2, OT_WDR_MODE_NONE, "OS02K10_2M_100FPS_10BIT_LINEAR" },
};

/* ==========================================================================
 *  AE (exposure / gain)
 * ========================================================================== */
static td_void cmos_get_ae_linear_default(ot_vi_pipe vi_pipe, ot_isp_ae_sensor_default *ae_sns_dft,
										   const ot_isp_sns_state *sns_state)
{
	ae_sns_dft->max_again        = OS02K10_AGAIN_MAX;
	ae_sns_dft->min_again        = OS02K10_AGAIN_MIN;
	ae_sns_dft->max_again_target = ae_sns_dft->max_again;
	ae_sns_dft->min_again_target = ae_sns_dft->min_again;

	/* No separate digital-gain register confirmed on OS02K10; keep unity. */
	ae_sns_dft->max_dgain        = 1024;
	ae_sns_dft->min_dgain        = 1024;
	ae_sns_dft->max_dgain_target = ae_sns_dft->max_dgain;
	ae_sns_dft->min_dgain_target = ae_sns_dft->min_dgain;

	ae_sns_dft->ae_compensation  = 0x38;
	ae_sns_dft->init_exposure    = g_init_exposure[vi_pipe] ? g_init_exposure[vi_pipe] : 40000;

	/* Direct integration-time model: max = VTS - margin (confirmed
	 * relationship, see OS02K10_MIN_INT_TIME_MARGIN). */
	ae_sns_dft->max_int_time        = sns_state->fl_std - OS02K10_MIN_INT_TIME_MARGIN;
	ae_sns_dft->min_int_time        = 1;
	ae_sns_dft->max_int_time_target = 65535;
	ae_sns_dft->min_int_time_target = 1;
}

static td_s32 cmos_get_ae_default(ot_vi_pipe vi_pipe, ot_isp_ae_sensor_default *ae_sns_dft)
{
	ot_isp_sns_state *sns_state = os02k10_get_ctx(vi_pipe);
	sns_check_pointer_return(ae_sns_dft);
	sns_check_pointer_return(sns_state);

	ae_sns_dft->fps            = g_os02k10_mode_tbl[sns_state->img_mode].max_fps;
	ae_sns_dft->full_lines_std = sns_state->fl_std;
	ae_sns_dft->full_lines_max = g_os02k10_mode_tbl[sns_state->img_mode].max_ver_lines;
	ae_sns_dft->flicker_freq   = 0;
	ae_sns_dft->hmax_times     = (td_u32)(1000000000.0f /
		(sns_state->fl_std * ae_sns_dft->fps));
	ae_sns_dft->lines_per500ms = (td_u32)(sns_state->fl_std * ae_sns_dft->fps / 2);
	ae_sns_dft->int_time_accu.accu_type = OT_ISP_AE_ACCURACY_LINEAR;
	ae_sns_dft->int_time_accu.accuracy  = 1;
	ae_sns_dft->int_time_accu.offset    = 0;
	ae_sns_dft->again_accu.accu_type    = OT_ISP_AE_ACCURACY_TABLE;
	ae_sns_dft->again_accu.accuracy     = 1;
	ae_sns_dft->dgain_accu.accu_type    = OT_ISP_AE_ACCURACY_TABLE;
	ae_sns_dft->dgain_accu.accuracy     = 1;
	ae_sns_dft->isp_dgain_shift          = 8;
	ae_sns_dft->min_isp_dgain_target     = 1u << ae_sns_dft->isp_dgain_shift;
	/* Same conservative ceiling rationale as imx662_cmos.c: keep AE
	 * exhausting analog gain before reaching for ISP digital gain. Live
	 * over /api/v1/iq/set?exposure.auto.ispd_gain_max=<n>. */
	ae_sns_dft->max_isp_dgain_target     = 4u << ae_sns_dft->isp_dgain_shift;

	switch (sns_state->wdr_mode) {
		case OT_WDR_MODE_NONE:
		default:
			cmos_get_ae_linear_default(vi_pipe, ae_sns_dft, sns_state);
			break;
	}
	return TD_SUCCESS;
}

/* VTS lives at fast-update i2c_data[4..5]. */
static td_void cmos_config_vts(ot_isp_sns_state *sns_state, td_u32 vts)
{
	sns_state->regs_info[0].i2c_data[4].data = high_8bits(vts);
	sns_state->regs_info[0].i2c_data[5].data = low_8bits(vts);
}

static td_void cmos_fps_set(ot_vi_pipe vi_pipe, td_float fps, ot_isp_ae_sensor_default *ae_sns_dft)
{
	ot_isp_sns_state *sns_state = os02k10_get_ctx(vi_pipe);
	td_float max_fps = g_os02k10_mode_tbl[sns_state->img_mode].max_fps;
	td_u32   vts;

	sns_check_pointer_void_return(ae_sns_dft);
	if (fps > max_fps || fps < g_os02k10_mode_tbl[sns_state->img_mode].min_fps) {
		isp_err_trace("Unsupported OS02K10 fps: %f\n", fps);
		return;
	}
	vts = (td_u32)((td_float)g_os02k10_mode_tbl[sns_state->img_mode].ver_lines * max_fps / fps);
	vts = (vts > OS02K10_FULL_LINES_MAX) ? OS02K10_FULL_LINES_MAX : vts;

	sns_state->fl_std = vts;
	ae_sns_dft->fps            = fps;
	ae_sns_dft->full_lines_std = sns_state->fl_std;
	ae_sns_dft->max_int_time   = sns_state->fl_std - OS02K10_MIN_INT_TIME_MARGIN;
	cmos_config_vts(sns_state, vts);
}

static td_void cmos_slow_framerate_set(ot_vi_pipe vi_pipe, td_u32 full_lines,
									   ot_isp_ae_sensor_default *ae_sns_dft)
{
	ot_isp_sns_state *sns_state = os02k10_get_ctx(vi_pipe);
	sns_check_pointer_void_return(ae_sns_dft);

	full_lines = (full_lines > OS02K10_FULL_LINES_MAX) ? OS02K10_FULL_LINES_MAX : full_lines;
	sns_state->fl[0] = full_lines;
	ae_sns_dft->full_lines = full_lines;
	ae_sns_dft->max_int_time = full_lines - OS02K10_MIN_INT_TIME_MARGIN;
	cmos_config_vts(sns_state, full_lines);
}

/* Direct integration time, written straight to EXPOSURE_HI/LO — no
 * frame-length subtraction (that's the Sony SHR0 model, not this one). */
static td_void cmos_inttime_update(ot_vi_pipe vi_pipe, td_u32 int_time)
{
	ot_isp_sns_state *sns_state = os02k10_get_ctx(vi_pipe);
	td_u32 fl = sns_state->fl[0] ? sns_state->fl[0] : sns_state->fl_std;

	if (int_time > fl - OS02K10_MIN_INT_TIME_MARGIN) {
		int_time = fl - OS02K10_MIN_INT_TIME_MARGIN;
	}
	if (int_time < 1) {
		int_time = 1;
	}
	sns_state->regs_info[0].i2c_data[0].data = high_8bits(int_time);
	sns_state->regs_info[0].i2c_data[1].data = low_8bits(int_time);
}

/* UNVERIFIED: assumes a Q8.8 fixed-point gain code (256 == 1x). HiSilicon
 * 'again' linear units are 1024 == 1x, so the register code is again/4 and
 * the round-trip is exact for every value AE can produce. See
 * os02k10_cmos.h's OS02K10_REG_GAIN_HI comment for what would confirm or
 * correct this. */
static td_u32 os02k10_gain_lin_to_reg(td_u32 again_lin)
{
	td_u32 reg;
	if (again_lin < OS02K10_AGAIN_MIN * 4u) {
		again_lin = OS02K10_AGAIN_MIN * 4u;
	}
	reg = again_lin / 4u;
	if (reg > (OS02K10_AGAIN_MAX)) {
		reg = OS02K10_AGAIN_MAX;
	}
	return reg;
}

static td_void cmos_again_calc_table(ot_vi_pipe vi_pipe, td_u32 *again_lin, td_u32 *again_db)
{
	td_u32 reg;
	ot_unused(vi_pipe);
	sns_check_pointer_void_return(again_lin);
	sns_check_pointer_void_return(again_db);

	reg = os02k10_gain_lin_to_reg(*again_lin);
	*again_db  = reg;                 /* register value */
	*again_lin = reg * 4u;            /* snapped back    */
}

static td_void cmos_dgain_calc_table(ot_vi_pipe vi_pipe, td_u32 *dgain_lin, td_u32 *dgain_db)
{
	ot_unused(vi_pipe);
	sns_check_pointer_void_return(dgain_lin);
	sns_check_pointer_void_return(dgain_db);
	*dgain_lin = 1024;   /* no separate digital gain register confirmed */
	*dgain_db  = 0;
}

/* Gain register at fast-update i2c_data[2..3]. */
static td_void cmos_gains_update(ot_vi_pipe vi_pipe, td_u32 again, td_u32 dgain)
{
	ot_isp_sns_state *sns_state = os02k10_get_ctx(vi_pipe);
	td_u32 reg = again;
	ot_unused(dgain);

	if (reg > OS02K10_AGAIN_MAX) {
		reg = OS02K10_AGAIN_MAX;
	}
	sns_state->regs_info[0].i2c_data[2].data = high_8bits(reg);
	sns_state->regs_info[0].i2c_data[3].data = low_8bits(reg);
}

static td_void cmos_get_inttime_max(ot_vi_pipe vi_pipe, td_u16 man_ratio_enable, td_u32 *ratio,
									ot_isp_ae_int_time_range *int_time, td_u32 *lf_max_int_time)
{
	/* Linear-only: no WDR/long-short split exists for this sensor here. */
	ot_unused(vi_pipe); ot_unused(man_ratio_enable); ot_unused(ratio);
	ot_unused(int_time); ot_unused(lf_max_int_time);
}

static td_void cmos_ae_fswdr_attr_set(ot_vi_pipe vi_pipe, ot_isp_ae_fswdr_attr *ae_fswdr_attr)
{
	ot_unused(vi_pipe); ot_unused(ae_fswdr_attr);
}

static td_s32 cmos_init_ae_exp_function(ot_isp_ae_sensor_exp_func *exp_func)
{
	sns_check_pointer_return(exp_func);
	(td_void)memset_s(exp_func, sizeof(ot_isp_ae_sensor_exp_func), 0, sizeof(ot_isp_ae_sensor_exp_func));

	exp_func->pfn_cmos_get_ae_default     = cmos_get_ae_default;
	exp_func->pfn_cmos_fps_set            = cmos_fps_set;
	exp_func->pfn_cmos_slow_framerate_set = cmos_slow_framerate_set;
	exp_func->pfn_cmos_inttime_update     = cmos_inttime_update;
	exp_func->pfn_cmos_gains_update       = cmos_gains_update;
	exp_func->pfn_cmos_again_calc_table   = cmos_again_calc_table;
	exp_func->pfn_cmos_dgain_calc_table   = cmos_dgain_calc_table;
	exp_func->pfn_cmos_get_inttime_max    = cmos_get_inttime_max;
	exp_func->pfn_cmos_ae_fswdr_attr_set  = cmos_ae_fswdr_attr_set;
	return TD_SUCCESS;
}

/* ==========================================================================
 *  AWB — placeholder, borrowed via os02k10_cmos_param.h (see its header
 *  comment: double-borrowed from IMX662, itself borrowed from sc450ai).
 * ========================================================================== */
static const ot_isp_awb_ccm g_os02k10_awb_ccm = {
	4,
	{
		{ 7500, { 0x0114, 0x801a, 0x0006, 0x8009, 0x010c, 0x8003, 0x0005, 0x801e, 0x0119 } },
		{ 6500, { 0x0119, 0x801a, 0x0002, 0x800b, 0x010e, 0x8003, 0x0005, 0x801e, 0x0119 } },
		{ 4800, { 0x0119, 0x801b, 0x0001, 0x800c, 0x0115, 0x8008, 0x0009, 0x8023, 0x011b } },
		{ 2600, { 0x011f, 0x801c, 0x8004, 0x8010, 0x0110, 0x0000, 0x0012, 0x803c, 0x012a } },
	},
};

static td_s32 cmos_get_awb_default(ot_vi_pipe vi_pipe, ot_isp_awb_sensor_default *awb_sns_dft)
{
	ot_unused(vi_pipe);
	sns_check_pointer_return(awb_sns_dft);
	(td_void)memset_s(awb_sns_dft, sizeof(ot_isp_awb_sensor_default), 0,
					  sizeof(ot_isp_awb_sensor_default));

	awb_sns_dft->wb_ref_temp    = OS02K10_AWB_STATIC_TEMP;
	awb_sns_dft->gain_offset[0] = OS02K10_AWB_STATIC_WB_R;
	awb_sns_dft->gain_offset[1] = OS02K10_AWB_STATIC_WB_GR;
	awb_sns_dft->gain_offset[2] = OS02K10_AWB_STATIC_WB_GB;
	awb_sns_dft->gain_offset[3] = OS02K10_AWB_STATIC_WB_B;
	awb_sns_dft->wb_para[0] = OS02K10_AWB_P1;
	awb_sns_dft->wb_para[1] = OS02K10_AWB_P2;
	awb_sns_dft->wb_para[2] = OS02K10_AWB_Q1;
	awb_sns_dft->wb_para[3] = OS02K10_AWB_A1;
	awb_sns_dft->wb_para[4] = OS02K10_AWB_B1;
	awb_sns_dft->wb_para[5] = OS02K10_AWB_C1;
	(td_void)memcpy_s(&awb_sns_dft->ccm, sizeof(awb_sns_dft->ccm),
					  &g_os02k10_awb_ccm, sizeof(g_os02k10_awb_ccm));
	(td_void)memcpy_s(&awb_sns_dft->agc_tbl, sizeof(awb_sns_dft->agc_tbl),
					  &g_os02k10_awb_agc_table, sizeof(g_os02k10_awb_agc_table));
	(td_void)memcpy_s(&awb_sns_dft->sector, sizeof(awb_sns_dft->sector),
					  &g_os02k10_color_sector, sizeof(g_os02k10_color_sector));
	return TD_SUCCESS;
}

static td_s32 cmos_init_awb_exp_function(ot_isp_awb_sensor_exp_func *exp_func)
{
	sns_check_pointer_return(exp_func);
	(td_void)memset_s(exp_func, sizeof(ot_isp_awb_sensor_exp_func), 0,
					  sizeof(ot_isp_awb_sensor_exp_func));
	exp_func->pfn_cmos_get_awb_default = cmos_get_awb_default;
	return TD_SUCCESS;
}

/* ==========================================================================
 *  ISP defaults
 * ========================================================================== */
static td_s32 cmos_get_isp_default(ot_vi_pipe vi_pipe, ot_isp_cmos_default *isp_def)
{
	ot_isp_sns_state *sns_state = os02k10_get_ctx(vi_pipe);
	sns_check_pointer_return(isp_def);
	sns_check_pointer_return(sns_state);
	(td_void)memset_s(isp_def, sizeof(ot_isp_cmos_default), 0, sizeof(ot_isp_cmos_default));

	/* Same block selection as imx662_cmos.c: generic ISP-behaviour blocks
	 * on, lsc/dpc off (no per-module shading capture / not measured). See
	 * os02k10_cmos_param.h for the borrow provenance of every table here. */
	isp_def->key.bit1_demosaic         = 1;
	isp_def->demosaic                  = &g_cmos_demosaic;
	isp_def->key.bit1_gamma            = 1;
	isp_def->gamma                     = &g_cmos_gamma;
	isp_def->key.bit1_clut             = 1;
	isp_def->clut                      = &g_cmos_clut;
	isp_def->key.bit1_anti_false_color = 1;
	isp_def->anti_false_color          = &g_cmos_anti_false_color;
	isp_def->key.bit1_cac              = 1;
	isp_def->cac                       = &g_cmos_cac;
	isp_def->key.bit1_ldci             = 1;
	isp_def->ldci                      = &g_cmos_ldci;
	isp_def->key.bit1_dehaze           = 1;
	isp_def->dehaze                    = &g_cmos_dehaze;
	isp_def->key.bit1_ca               = 1;
	isp_def->ca                        = &g_cmos_ca;
	isp_def->key.bit1_bayer_nr         = 1;
	isp_def->bayer_nr                  = &g_cmos_bayer_nr;
	isp_def->key.bit1_sharpen          = 1;
	isp_def->sharpen                   = &g_cmos_yuv_sharpen;
	isp_def->key.bit1_drc              = 1;
	isp_def->drc                       = &g_cmos_drc;
	(td_void)memcpy_s(&isp_def->noise_calibration, sizeof(ot_isp_noise_calibration),
					  &g_cmos_noise_calibration, sizeof(ot_isp_noise_calibration));

	isp_def->sns_mode.sns_id        = OS02K10_ID;
	isp_def->sns_mode.sns_mode      = sns_state->img_mode;
	return TD_SUCCESS;
}

static td_s32 cmos_get_isp_black_level(ot_vi_pipe vi_pipe, ot_isp_cmos_black_level *black_level)
{
	td_s32 i;
	ot_unused(vi_pipe);
	sns_check_pointer_return(black_level);
	(td_void)memset_s(black_level, sizeof(ot_isp_cmos_black_level), 0,
					  sizeof(ot_isp_cmos_black_level));

	black_level->auto_attr.update = TD_TRUE;
	for (i = 0; i < OT_ISP_BAYER_CHN_NUM; i++) {
		black_level->auto_attr.black_level[0][i] = OS02K10_BLACK_LEVEL;
	}
	return TD_SUCCESS;
}

static td_void cmos_set_pixel_detect(ot_vi_pipe vi_pipe, td_bool enable)
{
	ot_unused(vi_pipe); ot_unused(enable);
}

static td_s32 cmos_set_wdr_mode(ot_vi_pipe vi_pipe, td_u8 mode)
{
	ot_isp_sns_state *sns_state = os02k10_get_ctx(vi_pipe);
	sns_check_pointer_return(sns_state);

	switch (mode) {
		case OT_WDR_MODE_NONE:
			sns_state->wdr_mode = OT_WDR_MODE_NONE;
			sns_state->fl_std   = OS02K10_VTS_60FPS_LINEAR;
			break;
		default:
			isp_err_trace("OS02K10: unsupported WDR mode %d (linear only, no HDR mode compiled)\n", mode);
			return TD_FAILURE;
	}
	sns_state->sync_init = TD_FALSE;
	return TD_SUCCESS;
}

/* ==========================================================================
 *  Fast-update register table — 6 registers the ISP writes per frame,
 *  confirmed by a live bench diff (§10/§12 of the RE doc): exposure and
 *  gain are the only registers that move while streaming; VTS is included
 *  here (as IMX662's VMAX is) for slow-shutter / dynamic-fps support even
 *  though it wasn't observed to change in the one capture taken.
 * ========================================================================== */
static td_void cmos_comm_sns_reg_info_init(ot_vi_pipe vi_pipe, ot_isp_sns_state *sns_state)
{
	td_u32 i;
	const td_u16 reg_addr[] = {
		OS02K10_REG_EXPOSURE_HI, OS02K10_REG_EXPOSURE_LO,  /* 0..1 exposure */
		OS02K10_REG_GAIN_HI, OS02K10_REG_GAIN_LO,          /* 2..3 gain     */
		OS02K10_REG_VTS_HI, OS02K10_REG_VTS_LO,            /* 4..5 vts      */
	};

	sns_state->regs_info[0].sns_type         = OT_ISP_SNS_TYPE_I2C;
	sns_state->regs_info[0].com_bus.i2c_dev  = g_os02k10_bus_info[vi_pipe].i2c_dev;
	sns_state->regs_info[0].cfg2_valid_delay_max = 2;
	sns_state->regs_info[0].reg_num =
		(td_u32)(sizeof(reg_addr) / sizeof(reg_addr[0]));

	for (i = 0; i < sns_state->regs_info[0].reg_num; i++) {
		sns_state->regs_info[0].i2c_data[i].update        = TD_TRUE;
		sns_state->regs_info[0].i2c_data[i].delay_frame_num = 0;
		/* The kernel's fast-update I2C path (ot_sensor_i2c_write() in
		 * sensor_i2c.ko) right-shifts dev_addr by 1 to get the 7-bit
		 * slave address, so it needs the 8-bit form here -- unlike the
		 * userspace OT_I2C_SLAVE_FORCE ioctl in os02k10_sensor_ctl.c,
		 * which takes OS02K10_I2C_ADDR (7-bit) raw. Passing the 7-bit
		 * form here made every per-frame AE/gain write target the wrong
		 * address (0x1B instead of 0x36): silently dropped exposure/gain
		 * updates plus a "wait idle abort" flood in dmesg. Matches
		 * IMX662_I2C_ADDR's convention (defined 8-bit, shifted down only
		 * at the ioctl call site). */
		sns_state->regs_info[0].i2c_data[i].dev_addr      = (OS02K10_I2C_ADDR << 1);
		sns_state->regs_info[0].i2c_data[i].addr_byte_num = OS02K10_ADDR_BYTE;
		sns_state->regs_info[0].i2c_data[i].data_byte_num = OS02K10_DATA_BYTE;
		sns_state->regs_info[0].i2c_data[i].reg_addr      = reg_addr[i];
	}
}

static td_void cmos_sns_reg_info_update(ot_vi_pipe vi_pipe, ot_isp_sns_state *sns_state)
{
	td_u32 i;
	ot_unused(vi_pipe);
	for (i = 0; i < sns_state->regs_info[0].reg_num; i++) {
		if (sns_state->regs_info[0].i2c_data[i].data == sns_state->regs_info[1].i2c_data[i].data) {
			sns_state->regs_info[0].i2c_data[i].update = TD_FALSE;
		} else {
			sns_state->regs_info[0].i2c_data[i].update = TD_TRUE;
		}
	}
}

static td_s32 cmos_get_sns_regs_info(ot_vi_pipe vi_pipe, ot_isp_sns_regs_info *sns_regs_info)
{
	ot_isp_sns_state *sns_state = os02k10_get_ctx(vi_pipe);
	sns_check_pointer_return(sns_regs_info);
	sns_check_pointer_return(sns_state);

	if ((sns_state->sync_init == TD_FALSE) || (sns_regs_info->config == TD_FALSE)) {
		cmos_comm_sns_reg_info_init(vi_pipe, sns_state);
		sns_state->sync_init = TD_TRUE;
	} else {
		cmos_sns_reg_info_update(vi_pipe, sns_state);
	}
	sns_regs_info->config = TD_FALSE;
	(td_void)memcpy_s(sns_regs_info, sizeof(ot_isp_sns_regs_info),
					  &sns_state->regs_info[0], sizeof(ot_isp_sns_regs_info));
	(td_void)memcpy_s(&sns_state->regs_info[1], sizeof(ot_isp_sns_regs_info),
					  &sns_state->regs_info[0], sizeof(ot_isp_sns_regs_info));
	sns_state->fl[1] = sns_state->fl[0];
	return TD_SUCCESS;
}

static td_s32 cmos_set_image_mode(ot_vi_pipe vi_pipe, const ot_isp_cmos_sns_image_mode *sns_image_mode)
{
	ot_isp_sns_state *sns_state = os02k10_get_ctx(vi_pipe);
	sns_check_pointer_return(sns_image_mode);
	sns_check_pointer_return(sns_state);

	if (sns_image_mode->width > 1920 || sns_image_mode->height > 1080 ||
		sns_image_mode->fps > 100.0f) {
		isp_err_trace("OS02K10: unsupported mode %ux%u@%0.2f\n",
					  sns_image_mode->width, sns_image_mode->height,
					  sns_image_mode->fps);
		return TD_FAILURE;
	}
	if (sns_image_mode->fps > 75.0f) {
		sns_state->img_mode = OS02K10_SENSOR_2M_100FPS_10BIT_LINEAR_MODE;
		sns_state->fl_std   = OS02K10_VTS_100FPS_LINEAR;
	} else if (sns_image_mode->fps > 55.0f) {
		sns_state->img_mode = OS02K10_SENSOR_2M_60FPS_10BIT_LINEAR_MODE;
		sns_state->fl_std   = OS02K10_VTS_60FPS_LINEAR;
	} else {
		sns_state->img_mode = OS02K10_SENSOR_2M_50FPS_10BIT_LINEAR_MODE;
		sns_state->fl_std   = OS02K10_VTS_50FPS_LINEAR;
	}
	sns_state->sync_init = TD_FALSE;
	sns_state->fl[0]     = sns_state->fl_std;
	sns_state->fl[1]     = sns_state->fl[0];
	return TD_SUCCESS;
}

static td_void sensor_global_init(ot_vi_pipe vi_pipe)
{
	ot_isp_sns_state *sns_state = os02k10_get_ctx(vi_pipe);

	sns_state->init      = TD_FALSE;
	sns_state->sync_init = TD_FALSE;
	/* Default mode matches the Ascent's actual boot configuration
	 * (60 fps, angle=0), confirmed in the boot log capture. */
	sns_state->img_mode  = OS02K10_SENSOR_2M_60FPS_10BIT_LINEAR_MODE;
	sns_state->wdr_mode  = OT_WDR_MODE_NONE;
	sns_state->fl_std    = OS02K10_VTS_60FPS_LINEAR;
	sns_state->fl[0]     = OS02K10_VTS_60FPS_LINEAR;
	sns_state->fl[1]     = OS02K10_VTS_60FPS_LINEAR;
	(td_void)memset_s(&sns_state->regs_info[0], sizeof(ot_isp_sns_regs_info), 0,
					  sizeof(ot_isp_sns_regs_info));
	(td_void)memset_s(&sns_state->regs_info[1], sizeof(ot_isp_sns_regs_info), 0,
					  sizeof(ot_isp_sns_regs_info));
}

static td_s32 cmos_init_sensor_exp_function(ot_isp_sns_exp_func *sensor_exp_func)
{
	sns_check_pointer_return(sensor_exp_func);
	(td_void)memset_s(sensor_exp_func, sizeof(ot_isp_sns_exp_func), 0, sizeof(ot_isp_sns_exp_func));

	sensor_exp_func->pfn_cmos_sns_init          = os02k10_init;
	sensor_exp_func->pfn_cmos_sns_exit          = os02k10_exit;
	sensor_exp_func->pfn_cmos_sns_global_init   = sensor_global_init;
	sensor_exp_func->pfn_cmos_set_image_mode    = cmos_set_image_mode;
	sensor_exp_func->pfn_cmos_set_wdr_mode      = cmos_set_wdr_mode;
	sensor_exp_func->pfn_cmos_get_isp_default   = cmos_get_isp_default;
	sensor_exp_func->pfn_cmos_get_isp_black_level = cmos_get_isp_black_level;
	sensor_exp_func->pfn_cmos_set_pixel_detect  = cmos_set_pixel_detect;
	sensor_exp_func->pfn_cmos_get_sns_reg_info  = cmos_get_sns_regs_info;
	return TD_SUCCESS;
}

/* ==========================================================================
 *  Registration
 * ========================================================================== */
static td_s32 sensor_register_callback(ot_vi_pipe vi_pipe, ot_isp_3a_alg_lib *ae_lib,
									   ot_isp_3a_alg_lib *awb_lib)
{
	ot_isp_sns_register sns_register;
	ot_isp_sns_exp_func *sns_exp_func = &sns_register.sns_exp;
	ot_isp_ae_sensor_register ae_register;
	ot_isp_ae_sensor_exp_func *ae_exp_func = &ae_register.sns_exp;
	ot_isp_awb_sensor_register awb_register;
	ot_isp_awb_sensor_exp_func *awb_exp_func = &awb_register.sns_exp;
	ot_isp_sns_attr_info sns_attr_info = { .sns_id = OS02K10_ID };
	td_s32 ret;

	sns_check_pointer_return(ae_lib);
	sns_check_pointer_return(awb_lib);

	cmos_init_sensor_exp_function(sns_exp_func);
	ret = ot_mpi_isp_sensor_reg_callback(vi_pipe, &sns_attr_info, &sns_register);
	if (ret != TD_SUCCESS) {
		isp_err_trace("OS02K10: isp sensor reg callback failed!\n");
		return ret;
	}

	cmos_init_ae_exp_function(ae_exp_func);
	ret = ot_mpi_ae_sensor_reg_callback(vi_pipe, ae_lib, &sns_attr_info, &ae_register);
	if (ret != TD_SUCCESS) {
		isp_err_trace("OS02K10: ae sensor reg callback failed!\n");
		return ret;
	}

	cmos_init_awb_exp_function(awb_exp_func);
	ret = ot_mpi_awb_sensor_reg_callback(vi_pipe, awb_lib, &sns_attr_info, &awb_register);
	if (ret != TD_SUCCESS) {
		isp_err_trace("OS02K10: awb sensor reg callback failed!\n");
		return ret;
	}
	return TD_SUCCESS;
}

static td_s32 sensor_unregister_callback(ot_vi_pipe vi_pipe, ot_isp_3a_alg_lib *ae_lib,
										 ot_isp_3a_alg_lib *awb_lib)
{
	td_s32 ret;
	sns_check_pointer_return(ae_lib);
	sns_check_pointer_return(awb_lib);

	ret  = ot_mpi_isp_sensor_unreg_callback(vi_pipe, OS02K10_ID);
	ret += ot_mpi_ae_sensor_unreg_callback(vi_pipe, ae_lib, OS02K10_ID);
	ret += ot_mpi_awb_sensor_unreg_callback(vi_pipe, awb_lib, OS02K10_ID);
	return (ret != TD_SUCCESS) ? TD_FAILURE : TD_SUCCESS;
}

static td_s32 sensor_set_init(ot_vi_pipe vi_pipe, ot_isp_init_attr *init_attr)
{
	sns_check_pointer_return(init_attr);
	g_init_exposure[vi_pipe] = init_attr->exp_time;
	return TD_SUCCESS;
}

static td_s32 sensor_set_bus_info(ot_vi_pipe vi_pipe, ot_isp_sns_commbus bus_info)
{
	g_os02k10_bus_info[vi_pipe].i2c_dev = bus_info.i2c_dev;
	return TD_SUCCESS;
}

ot_isp_sns_obj g_sns_os02k10_obj = {
	.pfn_register_callback    = sensor_register_callback,
	.pfn_un_register_callback = sensor_unregister_callback,
	.pfn_standby              = os02k10_standby,
	.pfn_restart              = os02k10_restart,
	.pfn_mirror_flip          = os02k10_mirror_flip,
	.pfn_write_reg            = os02k10_write_register,
	.pfn_read_reg             = os02k10_read_register,
	.pfn_set_bus_info         = sensor_set_bus_info,
	.pfn_set_init             = sensor_set_init
};
