/*
 * os02k10_i2c_dump — read OmniVision OS02K10 registers over /dev/i2c on the
 * Caddx Ascent (Hi3516CV610) air unit.
 *
 * The sensor plugin's os02k10_read_register() is a stub that always returns
 * 0 (the ISP only ever writes), so there is no way to confirm from the app
 * itself that an init sequence actually landed. This does the 16-bit
 * address + 8-bit value read the sensor needs, from a second process.
 *
 * Adapted from tools/cv610_i2c_dump.c (same pattern, IMX662). Register set
 * and I2C parameters (bus 0, 7-bit slave 0x36, 3-byte write transaction)
 * were recovered from ar_ldyhs_sky in the Ascent_H_Sky_18_21_10 firmware —
 * see documentation/os02k10/OS02K10_ASCENT_REGISTER_RE.md.
 *
 * DEV-ONLY. Not installed into any image; push it to /tmp and delete it.
 *
 * Build:
 *   toolchain/toolchain.hisilicon-hi3516cv6xx/bin/arm-openipc-linux-musleabi-gcc \
 *       -O2 -s -Wall -Wextra -std=c99 -o out/os02k10_i2c_dump tools/os02k10_i2c_dump.c
 *
 * Run:
 *   scp -O out/os02k10_i2c_dump root@<ascent-ip>:/tmp/
 *   ssh root@<ascent-ip> /tmp/os02k10_i2c_dump             # the recovered register set
 *   ssh root@<ascent-ip> /tmp/os02k10_i2c_dump 0x3501 0x3502
 *   ssh root@<ascent-ip> /tmp/os02k10_i2c_dump 0x3800:0x380f
 *
 * Output is one register per line, "0xADDR 0xVV" or "0xADDR ERR", so a diff
 * of two dumps (or tools/os02k10_init_vs_live_diff.sh against the recovered
 * init table) is the whole comparison.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* Linux I2C_SLAVE_FORCE; os02k10_i2c_init() binds the same address, so the
 * non-FORCE ioctl would be refused as busy while the streamer is running. */
#define I2C_SLAVE_FORCE 0x0706

#define DEFAULT_DEV  "/dev/i2c-0"
#define DEFAULT_ADDR 0x36 /* 7-bit SCCB address forced by os02k10_i2c_init() */

/* Every register address os02k10's recovered init table (2-lane, 1920x1080,
 * 10-bit linear, 50/60/100 fps) writes, grouped by address range. Registers
 * tagged "core" in os02k10_registers.csv are understood (mode/exposure/
 * timing/crop); the rest are marked unknown there and are read here purely
 * for init-vs-live diffing, not because their bits are decoded. */
static const unsigned short g_default_ranges[][2] = {
	/* core */
	{ 0x0100, 0x0100 },
	/* unknown */
	{ 0x0102, 0x0104 },
	{ 0x0109, 0x0109 },
	{ 0x0303, 0x0303 },
	{ 0x0305, 0x0307 },
	{ 0x030a, 0x030a },
	{ 0x0317, 0x0317 },
	{ 0x0323, 0x0325 },
	{ 0x0327, 0x0327 },
	{ 0x032c, 0x032e },
	/* PLL/clock configuration (unknown detail) */
	{ 0x300f, 0x300f },
	{ 0x3012, 0x3012 },
	{ 0x3026, 0x3027 },
	{ 0x302d, 0x302d },
	{ 0x3103, 0x3103 },
	{ 0x3106, 0x3106 },
	/* reserved/analog trim (unknown detail) */
	{ 0x3400, 0x3400 },
	{ 0x3406, 0x3406 },
	{ 0x3408, 0x3408 },
	{ 0x340c, 0x340c },
	{ 0x3425, 0x342b },
	/* core */
	{ 0x3501, 0x3502 },
	/* unknown */
	{ 0x3504, 0x3504 },
	{ 0x3508, 0x3509 },
	{ 0x3544, 0x3544 },
	{ 0x3548, 0x3549 },
	{ 0x3584, 0x3584 },
	{ 0x3588, 0x3589 },
	/* analog/timing block (unknown detail) */
	{ 0x3601, 0x3601 },
	{ 0x3604, 0x3606 },
	{ 0x3608, 0x3608 },
	{ 0x360a, 0x360b },
	{ 0x360e, 0x3613 },
	{ 0x362a, 0x3631 },
	{ 0x3638, 0x3638 },
	{ 0x3643, 0x364a },
	{ 0x364c, 0x3651 },
	{ 0x3661, 0x3663 },
	{ 0x3665, 0x3665 },
	{ 0x3667, 0x3668 },
	{ 0x366f, 0x3671 },
	{ 0x3673, 0x3673 },
	{ 0x3681, 0x3681 },
	{ 0x3700, 0x3703 },
	{ 0x3706, 0x370b },
	{ 0x3714, 0x3714 },
	{ 0x371b, 0x371d },
	{ 0x3756, 0x3757 },
	{ 0x3762, 0x3762 },
	{ 0x376c, 0x376c },
	{ 0x3776, 0x3777 },
	{ 0x3779, 0x3779 },
	{ 0x377c, 0x377c },
	{ 0x3783, 0x3785 },
	{ 0x3790, 0x3790 },
	{ 0x3793, 0x3794 },
	{ 0x3796, 0x3797 },
	{ 0x379c, 0x379c },
	{ 0x37a1, 0x37a1 },
	{ 0x37bb, 0x37bb },
	{ 0x37bd, 0x37c0 },
	{ 0x37c7, 0x37c7 },
	{ 0x37ca, 0x37ca },
	{ 0x37cc, 0x37cd },
	{ 0x37cf, 0x37cf },
	{ 0x37d1, 0x37d3 },
	{ 0x37d5, 0x37d8 },
	{ 0x37da, 0x37dd },
	/* core: crop window, output size, HTS, VTS */
	{ 0x3800, 0x380f },
	/* unknown */
	{ 0x3811, 0x3811 },
	{ 0x3813, 0x3817 },
	{ 0x381c, 0x381c },
	/* core: mirror/flip (bit meaning not confirmed against a live camera) */
	{ 0x3820, 0x3822 },
	/* unknown */
	{ 0x382a, 0x382a },
	{ 0x3833, 0x3833 },
	{ 0x384c, 0x384d },
	{ 0x3858, 0x3858 },
	{ 0x3865, 0x3868 },
	/* digital/BLC block (unknown detail) */
	{ 0x3900, 0x3900 },
	{ 0x390c, 0x3911 },
	{ 0x3940, 0x3940 },
	{ 0x394c, 0x3951 },
	{ 0x3980, 0x3980 },
	{ 0x398c, 0x3991 },
	/* DPC/black-level block (unknown detail) */
	{ 0x3c01, 0x3c01 },
	{ 0x3c05, 0x3c05 },
	{ 0x3c0f, 0x3c0f },
	{ 0x3c12, 0x3c12 },
	{ 0x3c14, 0x3c14 },
	{ 0x3c19, 0x3c19 },
	{ 0x3c21, 0x3c21 },
	{ 0x3c3b, 0x3c3b },
	{ 0x3c3d, 0x3c3d },
	{ 0x3c55, 0x3c55 },
	{ 0x3c5d, 0x3c5e },
	{ 0x3ce0, 0x3ce3 },
	/* reserved (unknown detail) */
	{ 0x3d8c, 0x3d8d },
	/* digital gain/BLC block (unknown detail) */
	{ 0x4001, 0x4001 },
	{ 0x4004, 0x4005 },
	{ 0x4008, 0x400b },
	{ 0x400e, 0x400e },
	{ 0x4011, 0x4011 },
	{ 0x4028, 0x402b },
	{ 0x402e, 0x4033 },
	{ 0x4050, 0x4051 },
	/* reserved/timing (unknown detail) */
	{ 0x410f, 0x410f },
	{ 0x4288, 0x428a },
	/* DPC (unknown detail) */
	{ 0x430b, 0x430e },
	{ 0x4314, 0x4314 },
	/* BLC/LSC (unknown detail) */
	{ 0x4500, 0x4501 },
	{ 0x4504, 0x4504 },
	{ 0x4507, 0x4508 },
	{ 0x4603, 0x4603 },
	{ 0x4640, 0x4640 },
	{ 0x4646, 0x4649 },
	{ 0x464d, 0x464d },
	{ 0x4654, 0x4655 },
	/* MIPI TX PHY/timing (unknown detail) */
	{ 0x4800, 0x4800 },
	{ 0x480e, 0x480e },
	{ 0x4810, 0x4811 },
	{ 0x4813, 0x4813 },
	{ 0x4837, 0x4837 },
	{ 0x484b, 0x484b },
	/* OTP/PDAF-related (unknown detail) */
	{ 0x4d00, 0x4d05 },
	{ 0x4d09, 0x4d09 },
	/* DPC/LENC/digital (unknown detail) */
	{ 0x5000, 0x5000 },
	{ 0x5080, 0x5080 },
	{ 0x50c0, 0x50c0 },
	{ 0x5100, 0x5100 },
	{ 0x5200, 0x5203 },
	{ 0x5392, 0x5392 },
	{ 0x5395, 0x539b },
	/* LENC (unknown detail) */
	{ 0x5412, 0x5412 },
	{ 0x5415, 0x541b },
	{ 0x5492, 0x5492 },
	{ 0x5495, 0x549b },
	/* AWB/digital gain related (unknown detail) */
	{ 0x5780, 0x5780 },
	{ 0x5786, 0x5786 },
};

static int g_fd = -1;

static int reg_read(unsigned short reg, unsigned char *val)
{
	unsigned char buf[2] = { (unsigned char)(reg >> 8), (unsigned char)(reg & 0xff) };

	if (write(g_fd, buf, sizeof(buf)) != (ssize_t)sizeof(buf))
		return -1;
	if (read(g_fd, val, 1) != 1)
		return -1;
	return 0;
}

static void dump_range(unsigned short start, unsigned short end)
{
	unsigned int reg; /* wider than u16 so end == 0xffff terminates */

	for (reg = start; reg <= end; reg++) {
		unsigned char val;

		if (reg_read((unsigned short)reg, &val) == 0)
			printf("0x%04X 0x%02X\n", reg, val);
		else
			printf("0x%04X ERR\n", reg);
	}
}

/* "0x3501" or "0x3800:0x380f" */
static int parse_range(const char *arg, unsigned short *start, unsigned short *end)
{
	char *sep;
	long a, b;

	a = strtol(arg, &sep, 0);
	if (sep == arg || a < 0 || a > 0xffff)
		return -1;

	if (*sep == '\0') {
		b = a;
	} else if (*sep == ':') {
		const char *tail = sep + 1;
		b = strtol(tail, &sep, 0);
		if (sep == tail || *sep != '\0' || b < a || b > 0xffff)
			return -1;
	} else {
		return -1;
	}

	*start = (unsigned short)a;
	*end   = (unsigned short)b;
	return 0;
}

int main(int argc, char *argv[])
{
	const char *dev = getenv("I2C_DEV");
	const char *addr_env = getenv("I2C_ADDR");
	long addr = DEFAULT_ADDR;
	int i;

	if (dev == NULL)
		dev = DEFAULT_DEV;
	if (addr_env != NULL)
		addr = strtol(addr_env, NULL, 0);

	g_fd = open(dev, O_RDWR);
	if (g_fd < 0) {
		fprintf(stderr, "open %s: %s\n", dev, strerror(errno));
		return 1;
	}
	if (ioctl(g_fd, I2C_SLAVE_FORCE, addr) < 0) {
		fprintf(stderr, "I2C_SLAVE_FORCE 0x%02lx: %s\n", addr, strerror(errno));
		close(g_fd);
		return 1;
	}

	if (argc > 1) {
		for (i = 1; i < argc; i++) {
			unsigned short start, end;

			if (parse_range(argv[i], &start, &end) != 0) {
				fprintf(stderr, "bad range '%s' (want 0xNNNN or 0xNNNN:0xNNNN)\n",
						argv[i]);
				close(g_fd);
				return 2;
			}
			dump_range(start, end);
		}
	} else {
		for (i = 0; i < (int)(sizeof(g_default_ranges) / sizeof(g_default_ranges[0])); i++)
			dump_range(g_default_ranges[i][0], g_default_ranges[i][1]);
	}

	close(g_fd);
	return 0;
}
