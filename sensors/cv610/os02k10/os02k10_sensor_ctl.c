/*
 * os02k10_sensor_ctl.c — OS02K10 low-level transport + power-on register init.
 *
 * Implements the standard HiSilicon sensor transport (Linux /dev/i2c-N +
 * OT_I2C_SLAVE_FORCE) and the per-mode register init the ISP calls at
 * stream start. Recovered write order, verbatim:
 *   1. software reset (0x0103=0x01), then a 10 ms delay
 *   2. the common power-on block (os02k10_cfg.h, 313 registers)
 *   3. per-mode HTS/VTS override (HTS is constant; only VTS changes)
 *   4. default exposure + mode-select (streaming on)
 * — matching the vendor's own os02k10_init()/FUN_000fb890 exactly (see
 * documentation/os02k10/OS02K10_ASCENT_REGISTER_RE.md §5, cross-validated
 * against a real hardware LD_PRELOAD capture).
 */

#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#ifdef OT_GPIO_I2C
#include "gpioi2c_ex.h"
#else
#include "ot_i2c.h"
#endif
#include "securec.h"

#include "os02k10_cmos.h"
#include "os02k10_cfg.h"

#define I2C_DEV_FILE_NUM     16
#define I2C_BUF_NUM          8

/* Zero-initialized storage keeps this C99-portable without baking the SDK's
 * pipe count into an explicit {-1, ...} initializer. Store fd + 1 so zero
 * remains the closed sentinel (open() may legally return descriptor 0). */
static int g_fd_plus_one[OT_ISP_MAX_PIPE_NUM];

static int os02k10_i2c_fd(ot_vi_pipe vi_pipe)
{
	return g_fd_plus_one[vi_pipe] - 1;
}

td_s32 os02k10_i2c_init(ot_vi_pipe vi_pipe)
{
	int fd;

	if (g_fd_plus_one[vi_pipe] != 0) {
		return TD_SUCCESS;
	}
#ifdef OT_GPIO_I2C
	fd = open("/dev/gpioi2c_ex", O_RDONLY, S_IRUSR);
	if (fd < 0) {
		isp_err_trace("Open gpioi2c_ex error!\n");
		return TD_FAILURE;
	}
	g_fd_plus_one[vi_pipe] = fd + 1;
#else
	td_s32 ret;
	char dev_file[I2C_DEV_FILE_NUM] = {0};
	td_u8 dev_num;
	ot_isp_sns_commbus *bus = os02k10_get_bus_info(vi_pipe);

	dev_num = bus->i2c_dev;
	(td_void)snprintf_s(dev_file, sizeof(dev_file), sizeof(dev_file) - 1, "/dev/i2c-%u", dev_num);

	fd = open(dev_file, O_RDWR, S_IRUSR | S_IWUSR);
	if (fd < 0) {
		isp_err_trace("Open /dev/i2c-%u error!\n", dev_num);
		return TD_FAILURE;
	}

	/* Confirmed via ar_ldyhs_sky: passed as-is, no >>1 — 0x36 is already
	 * the 7-bit SCCB address on this sensor. */
	ret = ioctl(fd, OT_I2C_SLAVE_FORCE, OS02K10_I2C_ADDR);
	if (ret < 0) {
		isp_err_trace("I2C_SLAVE_FORCE error!\n");
		close(fd);
		return ret;
	}
	g_fd_plus_one[vi_pipe] = fd + 1;
#endif
	return TD_SUCCESS;
}

td_s32 os02k10_i2c_exit(ot_vi_pipe vi_pipe)
{
	if (g_fd_plus_one[vi_pipe] != 0) {
		close(os02k10_i2c_fd(vi_pipe));
		g_fd_plus_one[vi_pipe] = 0;
		return TD_SUCCESS;
	}
	return TD_FAILURE;
}

td_s32 os02k10_read_register(ot_vi_pipe vi_pipe, td_u32 addr)
{
	/* The vendor driver's own read path is a hardwired stub that always
	 * returns 0 (confirmed: os02k10_read_register in ar_ldyhs_sky is
	 * `return 0;` unconditionally) — the ISP never reads back from this
	 * sensor. Match that behavior; use tools/os02k10_i2c_dump.c for real
	 * readback from outside the process. */
	ot_unused(vi_pipe);
	ot_unused(addr);
	return TD_SUCCESS;
}

td_s32 os02k10_write_register(ot_vi_pipe vi_pipe, td_u32 addr, td_u32 data)
{
	int fd;

	if (g_fd_plus_one[vi_pipe] == 0) {
		return TD_SUCCESS;
	}
	fd = os02k10_i2c_fd(vi_pipe);
#ifdef OT_GPIO_I2C
	i2c_data.dev_addr      = OS02K10_I2C_ADDR;
	i2c_data.reg_addr      = addr;
	i2c_data.addr_byte_num = OS02K10_ADDR_BYTE;
	i2c_data.data          = data;
	i2c_data.data_byte_num = OS02K10_DATA_BYTE;
	if (ioctl(fd, GPIO_I2C_WRITE, &i2c_data)) {
		isp_err_trace("GPIO-I2C write failed!\n");
		return TD_FAILURE;
	}
#else
	td_u32 idx = 0;
	td_u8 buf[I2C_BUF_NUM];

	/* Confirmed 3-byte transaction: {addr_hi, addr_lo, data}. */
	buf[idx++] = (addr >> 8) & 0xff;
	buf[idx++] = addr & 0xff;
	buf[idx++] = data & 0xff;

	if (write(fd, buf, OS02K10_ADDR_BYTE + OS02K10_DATA_BYTE) < 0) {
		isp_err_trace("I2C_WRITE error!\n");
		return TD_FAILURE;
	}
#endif
	return TD_SUCCESS;
}

static void delay_ms(int ms)
{
	usleep(ms * 1000);
}

td_void os02k10_standby(ot_vi_pipe vi_pipe)
{
	(td_void)os02k10_write_register(vi_pipe, OS02K10_REG_MODE_SELECT, 0x00);
}

td_void os02k10_restart(ot_vi_pipe vi_pipe)
{
	(td_void)os02k10_write_register(vi_pipe, OS02K10_REG_MODE_SELECT, 0x01);
}

td_void os02k10_mirror_flip(ot_vi_pipe vi_pipe, ot_isp_sns_mirrorflip_type sns_mirror_flip)
{
	/* Confirmed against the vendor's own decompiled init table
	 * (cmos_2or4lane_10bit_linear_init-equivalent in ar_ldyhs_sky): it
	 * writes FORMAT1 as exactly 0x02 (angle==0) or 0x0c (angle==180,
	 * i.e. mirror+flip together), FORMAT2 always 0x00 either way. The
	 * vendor firmware itself has no third or fourth value anywhere in
	 * that table -- ISP_SNS_MIRROR-only and ISP_SNS_FLIP-only aren't a
	 * gap in this port, this sensor's stock firmware doesn't support
	 * single-axis orientation either, only the combined 180 degrees.
	 * Confirmed correct on hardware (geometry and colour) with the
	 * camera correctly mounted.
	 *
	 * Other bit patterns in this register are unexplored by the vendor
	 * table and confirmed unsafe on the bench: 0x03 dropped the image
	 * entirely until the sensor's full init sequence was replayed. Do
	 * not add new values here without a vendor-table cross-reference. */
	switch (sns_mirror_flip) {
		case ISP_SNS_NORMAL:
			(td_void)os02k10_write_register(vi_pipe, OS02K10_REG_FORMAT1, 0x02);
			(td_void)os02k10_write_register(vi_pipe, OS02K10_REG_FORMAT2, 0x00);
			break;
		case ISP_SNS_MIRROR_FLIP:
			(td_void)os02k10_write_register(vi_pipe, OS02K10_REG_FORMAT1, 0x0c);
			(td_void)os02k10_write_register(vi_pipe, OS02K10_REG_FORMAT2, 0x00);
			break;
		default:
			isp_err_trace("OS02K10: mirror/flip mode %d not supported by this sensor's "
						  "firmware (only NORMAL and MIRROR_FLIP exist), ignoring\n",
						  sns_mirror_flip);
			break;
	}
}

/* ---- power-on register block ----------------------------------------------
 * The common block lives in os02k10_cfg.h and runs first; the leading
 * software reset + delay and the per-mode tail below are issued here to
 * match the exact recovered order (see file header).
 */
static td_s32 os02k10_write_init_seq(ot_vi_pipe vi_pipe)
{
	td_s32 ret = 0;
	td_u32 i;

	for (i = 0; i < OS02K10_INIT_SEQ_LEN; i++) {
		ret += os02k10_write_register(vi_pipe, g_os02k10_init_seq[i].addr,
									  g_os02k10_init_seq[i].data);
	}
	return ret;
}

static td_s32 os02k10_linear_1080p_init(ot_vi_pipe vi_pipe, td_u32 vts)
{
	td_s32 ret = 0;

	ret += os02k10_write_register(vi_pipe, OS02K10_REG_SOFTWARE_RESET, 0x01);
	delay_ms(10);

	ret += os02k10_write_init_seq(vi_pipe);

	/* HTS is constant across every mode; only VTS (frame length) changes
	 * the frame rate at this fixed pixel clock. */
	ret += os02k10_write_register(vi_pipe, OS02K10_REG_HTS_HI, (OS02K10_HTS_LINEAR >> 8) & 0xff);
	ret += os02k10_write_register(vi_pipe, OS02K10_REG_HTS_LO, OS02K10_HTS_LINEAR & 0xff);
	ret += os02k10_write_register(vi_pipe, OS02K10_REG_VTS_HI, (vts >> 8) & 0xff);
	ret += os02k10_write_register(vi_pipe, OS02K10_REG_VTS_LO, vts & 0xff);

	/* Default exposure (recovered constant, 0x04da = 1242 lines) then
	 * mode-select streaming-on — exact final two writes of every capture. */
	ret += os02k10_write_register(vi_pipe, OS02K10_REG_EXPOSURE_HI, 0x04);
	ret += os02k10_write_register(vi_pipe, OS02K10_REG_EXPOSURE_LO, 0xda);
	ret += os02k10_write_register(vi_pipe, OS02K10_REG_MODE_SELECT, 0x01);

	return ret;
}

td_void os02k10_default_reg_init(ot_vi_pipe vi_pipe)
{
	ot_isp_sns_state *sns_state = os02k10_get_ctx(vi_pipe);

	switch (sns_state->img_mode) {
		case OS02K10_SENSOR_2M_50FPS_10BIT_LINEAR_MODE:
			(td_void)os02k10_linear_1080p_init(vi_pipe, OS02K10_VTS_50FPS_LINEAR);
			break;
		case OS02K10_SENSOR_2M_100FPS_10BIT_LINEAR_MODE:
			(td_void)os02k10_linear_1080p_init(vi_pipe, OS02K10_VTS_100FPS_LINEAR);
			break;
		case OS02K10_SENSOR_2M_60FPS_10BIT_LINEAR_MODE:
		default:
			(td_void)os02k10_linear_1080p_init(vi_pipe, OS02K10_VTS_60FPS_LINEAR);
			break;
	}
}

td_void os02k10_init(ot_vi_pipe vi_pipe)
{
	ot_isp_sns_state *sns_state;

	(td_void)os02k10_i2c_init(vi_pipe);
	os02k10_default_reg_init(vi_pipe);
	delay_ms(2);
	sns_state = os02k10_get_ctx(vi_pipe);
	sns_state->init = TD_TRUE;
}

td_void os02k10_exit(ot_vi_pipe vi_pipe)
{
	(td_void)os02k10_i2c_exit(vi_pipe);
}
