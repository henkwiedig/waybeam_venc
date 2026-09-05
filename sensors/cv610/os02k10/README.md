# OmniVision OS02K10 plugin for Hi3516CV610

This is the source-built HiSilicon V5 sensor plugin used by the CV610
backend for the Caddx Ascent air unit. Unlike `sensors/cv610/imx662/`
(a scaffold written against the public sensor interface plus datasheet/
V4L2-driver references), this plugin is a **direct port of a reverse-
engineered register table**: the vendor SDK checkout this build targets
ships no OS02K10 driver at all, and OS02K10 doesn't have the kind of public
Linux/V4L2 reference driver IMX662 does. Everything here comes from a full
register-level RE pass of the Ascent's own stock firmware
(`ar_ldyhs_sky`) — see
`documentation/os02k10/OS02K10_ASCENT_REGISTER_RE.md` for the complete
methodology and findings, and `documentation/os02k10/os02k10_init.txt` /
`os02k10_registers.csv` for the recovered data this plugin is built from.

Build it with the same public OpenHisilicon headers and OpenIPC ARMv7 musl
toolchain as the daemon:

```sh
make sensor-cv610 SOC_BUILD=cv610 CV610_SENSOR_PLUGIN=os02k10 \
	CV610_CC=/path/to/arm-openipc-linux-musleabi-gcc \
	CV610_SDK_INC=/path/to/openhisilicon
```

Install the result as `/usr/lib/sensors/libsns_os02k10.so`.

Linear mode only, 1920x1080, three FPS profiles (50/60/100 — the Ascent
boots at 60). No WDR/HDR mode exists in the recovered firmware; this
plugin's `cmos_set_wdr_mode()` rejects everything but `OT_WDR_MODE_NONE`.

## What's confirmed vs. what's a placeholder

**Confirmed against real, actively-streaming Ascent hardware (2026-09-05):**
- The entire static power-on/mode-select register sequence
  (`os02k10_cfg.h` plus the reset/HTS-VTS/exposure/stream-on writes in
  `os02k10_sensor_ctl.c`) — an `LD_PRELOAD` trace of a real boot matched it
  byte-for-byte, same order.
- The I2C transport: bus 0, 7-bit slave address `0x36`, 3-byte write
  transaction, read path is a stub (matches the vendor driver exactly —
  it never reads back either).
- The fast-update register identities: exposure (`0x3501`/`0x3502`) and
  gain (`0x3508`, `0x3509` presumed same field) are the only registers a
  live capture showed changing while the camera was streaming — everything
  else in the table held its init value throughout.
- Mirror/flip for the default orientation (`angle=0` → `0x3820=0x02`,
  `0x3821=0x00`) — matches a correctly-oriented real image.

**Not confirmed — treat as placeholders needing real calibration:**
- The gain register's internal fixed-point format
  (`os02k10_gain_lin_to_reg()` assumes Q8.8; only one live gain sample
  exists, not enough to derive the real scale).
- Every IQ tuning table in `os02k10_cmos_param.h` — copied verbatim from
  `sensors/cv610/imx662/imx662_cmos_param.h`, itself already a borrow from
  `smart_sc450ai`. This is a double-borrow with no OS02K10-specific
  evidence behind it at all; it compiles and produces a non-flat image,
  nothing more. The vendor SDK's `omnivision_os04d10` (same vendor family,
  same SDK generation) would be a better donor than IMX662/sc450ai — that
  substitution was not done in this pass, only identified as the right
  follow-up.
- Mirror/flip for the non-default orientation: the alternate `0x3820=0x0c`
  value the vendor runtime can select was never captured live, so
  `os02k10_mirror_flip()` only reproduces the confirmed default and logs
  a warning for anything else rather than guessing.
- Black level (`OS02K10_BLACK_LEVEL`) — no register in the recovered table
  was identified as a black-level pedestal; the value here is IMX662's
  borrowed constant, unchanged.

## Reproducing or extending the register-level RE

`documentation/os02k10/OS02K10_ASCENT_REGISTER_RE.md` documents the exact
method (Ghidra decompile cross-checked against `arm-linux-gnueabi-objdump
-M force-thumb` disassembly parsing) in case this firmware is revised or a
future gap (like the gain-register format) needs to be re-derived the same
way. `tools/os02k10_i2c_dump.c` (read-only live register dump) and
`tools/os02k10_i2c_write_trace.c` (`LD_PRELOAD` write tracer) are both
confirmed working against real Ascent hardware and are the fastest way to
validate a change to this plugin against the vendor's own behavior —
compare `tools/os02k10_i2c_dump` output from this plugin's build against
`documentation/os02k10/os02k10_live.txt` (a real capture from the stock
firmware) with `tools/os02k10_init_vs_live_diff.sh`.
