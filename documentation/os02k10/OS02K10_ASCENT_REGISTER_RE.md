# OS02K10 sensor register recovery — Caddx Ascent (Hi3516CV610)

Reverse-engineering workflow and findings for the OmniVision OS02K10 sensor
driver embedded in the Caddx Ascent "Sky" (air unit) firmware. Feeds
[[hi3516cv610-ascent-backend]]: the Ascent's actual shipped sensor is
OS02K10, not SC850SL/SC4336P/IMX307 as earlier firmware dumps had it — this
is a newer firmware (`Ascent_V18.21.10`) with a different default sensor.

## 1. Where the driver lives

Firmware image: `Ascent_H_Sky_18_21_10.img`, extracted to
`.../Ascent_V18.21.10/extractions/Ascent_H_Sky_18_21_10.img.extracted/`.
Two UBIFS volumes matter:

- `3AB3A1/.../img-1024149406_vol-ubifs.../ubifs-root` — generic busybox
  Linux rootfs (management OS side, not sensor-specific).
- `3AB3A1/.../img-1891784038_vol-ubifs.../ubifs-root/fpv/` — the actual FPV
  air-unit payload: `ar_ldyhs_sky` (2.4 MB), kernel modules
  (`ko_cv610_20s/`), AR8030 wireless-link firmware (`boot_ar8030/`), and
  per-mode/per-scene ISP tuning blobs (`tunning/cam_os02k10_*fps_xg*.bin`).

**`fpv/ar_ldyhs_sky` is the target binary.** It is the streamer/MPP
application itself — ELF32 ARM EABI5, soft-float, dynamically linked against
only `libc.so`/`libgcc_s.so.1` (musl), entry `0x5a82d`, **stripped of
`.symtab` but not of `.dynsym`** (4913 dynamic symbols survive because the
sensor driver object files were compiled `-fPIC` and linked in statically —
their exported symbols stayed in the dynamic symbol table even though the
final binary is non-PIE `EXEC`). That dynsym leak is what makes this
recovery tractable without blind pattern matching:

```
$ nm -D fpv/ar_ldyhs_sky | grep -i os02k10
002a24ac B g_os02k10_mirr_flip_want_angle
00262580 D g_os02k10_mode_tbl
0026b484 D g_sns_os02k10_obj
000fcc4d T os02k10_exit
000fa0a1 T os02k10_get_bus_info
000fa0b5 T os02k10_get_ctx
000fb7a1 T os02k10_i2c_exit
000fb691 T os02k10_i2c_init
000fcb29 T os02k10_init
000fcb21 T os02k10_prog
000fb7c9 T os02k10_read_register
000fcb25 T os02k10_restart
000fcb23 T os02k10_standby
000fb7cd T os02k10_write_register
```

There is no separate `libsns_os02k10.so` — unlike this repo's own CV610
sensor plugins (`sensors/cv610/imx662/`, built as a standalone `.so` per the
public OpenHisilicon SDK convention), Ascent's vendor build linked the
per-sensor object statically into the single `ar_ldyhs_sky` executable.

No other file in the firmware references `os02k10` (checked scripts,
configs, and every other extracted binary) — `ar_ldyhs_sky` is the sole
implementation.

## 2. Method

1. Confirmed the binary is dynamically linked EXEC, ARM Thumb-2
   (`file`, `readelf -h`, `readelf -d`).
2. Pulled global symbol addresses from `.dynsym` (`nm -D`, `readelf -sW`) —
   gave exact entry points for all 11 exported `os02k10_*` functions plus the
   two data objects (`g_os02k10_mode_tbl`, `g_sns_os02k10_obj`).
3. Opened the binary in Ghidra (auto-analysis, no manual signatures needed —
   Thumb-2 code, EABI calling convention) and decompiled each exported
   function. The one call from `os02k10_init()` into an *unnamed* local
   function (`FUN_000fb890`, body `0x000fb890`–`0x000fcb05`) turned out to be
   the actual per-mode register-programming routine — 327 calls to
   `os02k10_write_register()` in a single function body.
4. **Cross-checked the decompiler's transcription independently**: dumped
   the same address range with
   `arm-linux-gnueabi-objdump -d -M force-thumb --start-address=0xfb890 --stop-address=0xfcb06 fpv/ar_ldyhs_sky`
   and wrote a small parser (walks `movs/movw r2,#val` → `movs/movw r1,#reg`
   → `bl <os02k10_write_register>` triples) that reconstructs the write list
   straight from the machine code, independent of the decompiler's C
   rendering. **Both methods produced an identical 327-write sequence.**
   This is the same technique the live dumper/tracer below assume you can
   redo if the firmware is revised.
5. Resolved the two PC-relative data lookups (`os02k10_get_bus_info`'s
   per-pipe I2C bus table, `g_os02k10_mode_tbl`'s mode array) by hand from
   the Thumb `ldr rX,[pc,#n]; add rX,pc` idiom, then read the raw bytes at
   the computed address with Ghidra's memory reader to confirm.

No kernel-side artifact was needed: `ot_sensor_i2c.ko` /
`ot_sensor_spi.ko` exist in `ko_cv610_20s/extdrv/` but the OS02K10 transport
turned out to be plain userspace `/dev/i2c-N` (see below), so the kernel
module wasn't part of the recovered path and wasn't examined further.

## 3. I2C transport

```c
// os02k10_i2c_init(vi_pipe):
bus = os02k10_get_bus_info(vi_pipe);      // per-pipe I2C bus number, see below
snprintf(dev_file, "/dev/i2c-%u", bus);
fd = open(dev_file, O_RDWR, 0600);
ioctl(fd, I2C_SLAVE_FORCE /* 0x0706 */, 0x36);   // 7-bit slave address
```

- **I2C slave address: `0x36`** (7-bit) — the standard OmniVision/OS-series
  SCCB default (`0x6C`/`0x6D` in 8-bit read/write form).
- **Per-pipe bus table** (`g_os02k10_get_bus_info`, 4-byte array at
  `0x0026aa3e`): `{0x00, 0x01, 0x00, 0xFF}` — pipe 0 → `/dev/i2c-0`, pipe 1 →
  `/dev/i2c-1`, pipe 2 → `/dev/i2c-0`, pipe 3 → invalid/unused. The Ascent
  air unit is single-sensor, so only **pipe 0 / `/dev/i2c-0`** is live.
- **Register write = one 3-byte I2C transaction**:
  `write(fd, {reg>>8, reg&0xff, val}, 3)` — i.e. **16-bit big-endian register
  address + 8-bit value**, single `write(2)` call (no repeated-start
  read-modify-write). Same convention as this repo's IMX335/IMX415/IMX662
  sensor tooling (`tools/regscan/regscan.c`, `tools/cv610_i2c_dump.c`).
- **Register read is a stub**: `os02k10_read_register()` is
  `return 0;` — unconditionally. The ISP never reads back from the sensor;
  there is no way to confirm from the daemon itself that an init write
  landed. Same situation this repo already hit with IMX662
  (`tools/cv610_i2c_dump.c`'s header comment) — the live dumper below talks
  to `/dev/i2c-0` directly, bypassing the app entirely.

## 4. Mode selection

`os02k10_init(vi_pipe)` reads a small per-pipe context struct (returned by
`os02k10_get_ctx()`) and picks an FPS profile before calling the register
table function:

| ctx flags (WDR mode / 2nd flag / angle byte) | FPS profile | passed as |
|---|---|---|
| `ctx[0]==0 && ctx[3]==0 && angle==0` | 50 | `0x32` |
| `ctx[0]==0 && ctx[3]==0 && angle==1`, or `ctx[0]!=0 && ctx[3]==0 && angle==1` | 60 | `0x3c` |
| `ctx[0]==0 && ctx[3]==0 && angle==2`, or `ctx[0]!=0 && ctx[3]==0 && angle==2`, or `ctx[0]!=0 && ctx[3]!=0` | 100 | `0x64` |

The ctx struct's exact field semantics (what sets `ctx[0]`/`ctx[3]`/angle at
board-config time) were **not traced further** — out of scope once the
downstream register effect (which FPS tail runs) was confirmed. The boot log
in the target context (`OS02K10_2Lane_60FPS_10BIT_LINEAR_MODE`) confirms the
Ascent ships configured for the 60 FPS profile, which is what
`os02k10_init.txt` uses as its main (fully numbered) sequence.

The lane count is **not selected here** — the final `printf` in the register
function hard-codes `"OS02K10_%dLane_..."` with a literal `2`. This build
only contains a 2-lane linear-mode table; there is no compiled 4-lane
variant (`os02k10_write_register` has exactly one caller, confirmed via
Ghidra `get_xrefs_to`).

## 5. The recovered register table

**Primary deliverable: [`os02k10_init.txt`](os02k10_init.txt).** Full
327-write sequence, in exact program order, split into:

- a 314-write **common prefix** shared by every FPS profile (soft reset,
  10 ms delay, PLL/MIPI/analog/timing/crop/BLC/LENC/AWB blocks, ending with
  a provisional HTS/VTS and a first exposure/gain write that the tail then
  overrides),
- the **60 FPS tail** (7 writes: HTS/VTS override, exposure, streaming-on) —
  inlined as entries `0316`–`0322` since that's the profile this firmware
  boots into,
- an appendix with the **50 FPS and 100 FPS tails** (only one tail executes
  at runtime; which one is chosen per §4).

Frame-timing cross-check (why this is very likely correct, not a
transcription artifact): HTS is `0x033f` = 831 in all three profiles; VTS is
`0x0a26`=2598 (50fps), `0x0875`=2165 (60fps), `0x0512`=1298 (100fps).
2598/2165 = 1.200 and 2598/1298 = 2.002 — matching the 50:60:100 ratio
exactly, which is what you'd expect if frame rate is set purely by scaling
VTS at a fixed pixel clock (`fps = pclk / (HTS * VTS)`).

**[`os02k10_registers.csv`](os02k10_registers.csv)** — the same data
deduplicated to 313 unique addresses with their final (post-override) 60 FPS
value, tagged by register-block group where identifiable.

Duplicate writes are real (not a decompiler artifact) — e.g. `0x0305`,
`0x3012` and the exposure/HTS/VTS registers are written once in the common
block with a provisional value and again in the tail with the final one.
Both copies are preserved in `os02k10_init.txt`; only the CSV collapses to
final value.

## 6. Register groups — what's actually known vs. inferred

Only the following are asserted with confidence, because they match the
well-documented OmniVision/OS-series SCCB register map used across the
whole product line (not invented for this sensor specifically):

| register | meaning |
|---|---|
| `0x0100` | mode select — `0`=standby, `1`=streaming (last write in every profile) |
| `0x0103` | software reset, self-clearing (first write, followed by a 10 ms delay) |
| `0x3501`/`0x3502` | 16-bit coarse exposure (high/low byte) |
| `0x380c`/`0x380d` | HTS — line length in pixel clocks (high/low byte) |
| `0x380e`/`0x380f` | VTS — frame length in lines (high/low byte) |
| `0x3800`-`0x3807` | analog crop window (x/y start, x/y end) |
| `0x3808`/`0x3809` | output width (`0x0780` = 1920) |
| `0x380a`/`0x380b` | output height (`0x0438` = 1080) |

Everything else in the table (PLL/clock trims `0x30xx`/`0x31xx`, analog
front-end `0x34xx`/`0x36xx`/`0x37xx`, digital/BLC `0x39xx`/`0x40xx`, DPC
`0x3cxx`/`0x43xx`, LENC `0x54xx`, MIPI TX/PHY `0x48xx`, AWB/gain-adjacent
`0x50xx`/`0x57xx`) is **marked `unknown` in the CSV deliberately** — these
are reserved/analog-trim addresses whose per-bit meaning is not published
for OS02K10 and was not independently verified here. Do not treat the CSV
`group` column as authoritative register documentation; it is a coarse
address-range bucket to help a future diff, not a decoded bitfield map.

**Mirror/flip** (`0x3820`/`0x3821`) is the one exception worth flagging as
*partially* understood: the recovered table writes `0x3821 = 0`
unconditionally, but `0x3820` is computed at runtime —
`g_os02k10_mirr_flip_want_angle` (a global byte, `0x002a24ac`) gates a
`?2:0xc` select before the write. `0x0002` and `0x000c` differ in bits 2–3,
consistent with the OV/OS convention where `0x3820`/`0x3821` pack
mirror+flip+binning-related bits together, but which physical orientation
each value corresponds to was **not verified against a live camera** — flag
as inferred, not confirmed.

## 7. Mode table (`g_os02k10_mode_tbl`)

108 bytes at `0x00262580`, three 36-byte entries (one per FPS profile), 9
`u32`/`float` words each:

| field | 50fps entry | 60fps entry | 100fps entry | meaning |
|---|---|---|---|---|
| w0 | 2592 | 2159 | 1292 | `VTS - 6` in every case (2598-6, 2165-6, 1298-6) — very likely a usable-integration-time-lines ceiling for AE, not independently confirmed |
| w1 | 9744 | 9744 | 9744 | constant across modes — **unknown** |
| w2 | `50.0f` | `60.0f` | `100.0f` | FPS, IEEE-754 float |
| w3 | `30.0f` | `30.0f` | `30.0f` | constant across modes — **unknown** (possibly an AE floor unrelated to sensor mode) |
| w4 | 1920 | 1920 | 1920 | output width |
| w5 | 1080 | 1080 | 1080 | output height |
| w6, w7 | 0 | 0 | 0 | reserved/unused |
| w8 | `0x1aabff` | `0x1aaa94` | `0x1aac22` | address-shaped but inconsistent thumb/ARM parity across entries — **not** a verified function pointer; likely a per-mode label/descriptor reference, unresolved |

This table is exposed for completeness (task priority "any alternate mode
tables") — it corroborates the three FPS profiles and the fixed 1920x1080
output, but w1/w3/w8 are explicitly unresolved; don't build logic around
them without further RE.

## 8. Live register dumper

**[`tools/os02k10_i2c_dump.c`](../../tools/os02k10_i2c_dump.c)** — read-only,
modeled directly on this repo's existing `tools/cv610_i2c_dump.c` (same
CLI, same env-var overrides, same output format). Talks straight to
`/dev/i2c-0` at slave `0x36`; never touches the sensor driver (which can't
read back registers anyway, per §3).

```sh
# Build (same cross toolchain as the rest of the CV610 backend):
toolchain/toolchain.hisilicon-hi3516cv6xx/bin/arm-openipc-linux-musleabi-gcc \
    -O2 -s -Wall -Wextra -std=c99 -o out/os02k10_i2c_dump tools/os02k10_i2c_dump.c

# Run (push to the Ascent air unit, e.g. over the AR8030 link's debug shell
# or serial console — see boot_ar8030/ for the wireless-link boot config):
scp -O out/os02k10_i2c_dump root@<ascent-ip>:/tmp/
ssh root@<ascent-ip> /tmp/os02k10_i2c_dump                 # default register set (recovered table)
ssh root@<ascent-ip> /tmp/os02k10_i2c_dump 0x3501 0x3502   # specific registers
ssh root@<ascent-ip> /tmp/os02k10_i2c_dump 0x3800:0x380f   # a range
```

Output: one register per line, `0xADDR 0xVAL` or `0xADDR ERR`, sorted by
address — directly diffable against `os02k10_registers.csv`.

`DEV-ONLY`, like its imx662 counterpart: push to `/tmp`, don't install it
into any image.

## 9. Runtime write logging (if a future firmware revision needs re-recovery)

`ar_ldyhs_sky`'s internal `os02k10_write_register()` calls are **not**
PLT-indirected (direct `bl` to a statically-linked local function — nothing
to `LD_PRELOAD`-intercept there), but the I2C bytes still cross the libc
boundary via `open()`/`ioctl()`/`write()`, which **are** dynamically bound.
**[`tools/os02k10_i2c_write_trace.c`](../../tools/os02k10_i2c_write_trace.c)**
interposes those three (style matches this repo's existing
`tools/mi_snr_preload_trace.c`): it watches for `open("/dev/i2c-0", ...)`,
confirms the `I2C_SLAVE_FORCE`/`I2C_SLAVE` target is `0x36`, and logs every
3-byte `write()` on that fd as `timestamp reg value` before passing the
call through to real libc.

```sh
toolchain/toolchain.hisilicon-hi3516cv6xx/bin/arm-openipc-linux-musleabi-gcc -O2 -fPIC -shared -o out/os02k10_i2c_write_trace.so \
    tools/os02k10_i2c_write_trace.c -ldl

scp -O out/os02k10_i2c_write_trace.so root@<ascent-ip>:/tmp/
ssh root@<ascent-ip> 'LD_PRELOAD=/tmp/os02k10_i2c_write_trace.so /fpv/ar_ldyhs_sky' 2>/tmp/os02k10_boot_writes.log
```

This is temporary/reversible by construction (env-var interposition, no
binary patched, nothing installed) — exactly the "prefer LD_PRELOAD /
wrapper" guidance for capturing a real boot sequence if this document ever
needs to be regenerated against a different firmware build.

**Update — run against real hardware (2026-09-05):** the user built and ran
both `tools/os02k10_i2c_dump.c` and `tools/os02k10_i2c_write_trace.c` on a
fully booted, actively streaming stock Ascent unit. The `LD_PRELOAD` trace
captured the real boot's I2C writes end to end, and it is **byte-for-byte
identical, in the same order**, to the 322-entry 60 FPS sequence in
`os02k10_init.txt` (entries 0001–0322, the common prefix plus the 60 FPS
tail) — including the exact `usleep`-timed gap after the first write
(`0x0103=0x01`) and the exact final four writes
(`0x380f=0x75`, `0x3501=0x04`, `0x3502=0xda`, `0x0100=0x01`). This is
independent, on-device confirmation of the static register-table recovery
in §5 — not just of the decompiler/objdump cross-check in §2. The boot log
also confirms context this document only inferred: `sensor_cfg,433:
==========sensor0: cv2004==========` and `find sensor=os02k10, ok!`
selecting the OS02K10 path, `init sensor:1920,1080,60 ... angle=0`
confirming the 60 FPS / 2-lane / angle-0 profile matches §4's table exactly,
and `vi_pipe:0, (MIPI) OS02K10_2Lane_60FPS_10BIT_LINEAR_MODE angle=0, init
success!` as the final confirmation string.

## 10. Init-vs-live comparison

**[`tools/os02k10_init_vs_live_diff.sh`](../../tools/os02k10_init_vs_live_diff.sh)**
takes the recovered reference table and a live dump (in
`os02k10_i2c_dump`'s `0xADDR 0xVAL` output format) and prints
`register | init value | live value | changed`:

```sh
ssh root@<ascent-ip> /tmp/os02k10_i2c_dump > /tmp/os02k10_live.txt
tools/os02k10_init_vs_live_diff.sh documentation/os02k10/os02k10_registers.csv /tmp/os02k10_live.txt
```

**Update — run for real (2026-09-05):** `documentation/os02k10/os02k10_live.txt`
now holds a real `os02k10_i2c_dump` capture from a fully booted, actively
streaming stock Ascent unit (`/usrdata # ./os02k10_i2c_dump`), replacing the
earlier fabricated placeholder. Running the diff against it shows **exactly
four registers differ, and every other one of the ~309 read registers
matches the recovered init table exactly**:

| register | init value | live value | changed |
|---|---|---|---|
| `0x0103` | `0x01` | `0x00` | yes — expected: software reset is self-clearing |
| `0x3501` | `0x04` | `0x00` | yes — exposure, AE-adjusted while streaming |
| `0x3502` | `0xda` | `0x86` | yes — exposure, paired with `0x3501` |
| `0x3508` | `0x01` | `0x04` | yes — analog gain, AE-adjusted while streaming |

No other register moved from its documented init value while the camera was
live-streaming — meaning the ISP's per-frame update path touches only
`0x3501`/`0x3502` (exposure) and `0x3508` (gain) on this sensor, and nothing
else in the table is runtime-managed. This resolves §12's open question
empirically. `0x3509` stayed at its init value `0x00` in this capture (the
scene's gain apparently didn't need the low/fine byte to move); treat it as
"exists, part of the same 16-bit-ish gain field, not yet observed to
change" rather than "confirmed inert."

## 11. Unresolved / left for hardware validation

- `os02k10_get_ctx()`'s context struct fields (what sets FPS-selection
  flags and the mirror/flip angle at board-config time) — not traced past
  their observed effect.
- Mode-table fields w1 (9744, constant) and w3 (30.0f, constant) — unknown.
- Mode-table w8 ("pointer-shaped" but thumb/ARM-parity-inconsistent across
  entries) — not resolved.
- `0x3820`/`0x3821` mirror/flip bit meaning — the bench capture confirms
  `angle=0` (normal, no mirror/flip) writes `0x3820=0x02`/`0x3821=0x00`
  correctly and matches a real, correctly-oriented streaming image, but the
  `0x0c` alternate value (from the runtime `?2:0xc` select in §6) was never
  exercised — no mirrored/flipped capture was taken. Confirmed for the
  default orientation only.
- The bulk of the analog/digital/DPC/LENC register blocks (see §6) — no
  attempt was made to invent bitfield meanings without a datasheet or
  reference driver; they're preserved verbatim in the table but tagged
  `unknown`. The live-vs-init diff in §10 confirms none of them are
  runtime-managed (they held their init value throughout streaming), which
  at least rules out "secretly an AE/AWB-adjusted register."
- §8's write-trace tool and §9's live-dumper are now both **confirmed
  working against real Ascent hardware** (2026-09-05) — no longer flagged
  as unvalidated tooling.

## 12. Per-frame AE/gain register path (RE attempt inconclusive, resolved empirically instead)

`os02k10_init.txt` covers the *static* power-on/mode-select sequence only.
It does not cover how the running ISP updates exposure/gain each frame — on
IMX662 (this repo's other CV610 sensor plugin) that's a separate
fast-update path (`cmos_comm_sns_reg_info_init`/`cmos_get_sns_regs_info`)
wired through a distinct `ot_isp_sns_obj` slot, not through the sensor's own
`write_register`.

Attempted to locate the OS02K10 equivalent in the same Ghidra project by:
1. Reading `g_sns_os02k10_obj`'s raw 12-word struct — 4 slots
   (`pfn_register_callback`@`0xfa1e0`, `pfn_un_register_callback`@`0xfa0cc`,
   plus the already-known standby/restart/write_reg/read_reg) resolved to
   real, well-formed functions; the other 4 non-null slots
   (`0xf9d48`/`0xf9c14`/`0xf9db4`/`0xf9dd4`) did not decompile cleanly —
   Ghidra had never auto-created functions there (no direct call-graph
   xrefs reach them, only reachable through the struct itself), and forcing
   a function boundary at the exact computed address produced garbage or
   1-byte stubs.
2. Decompiling `pfn_register_callback` (`0xfa1e0`) far enough to find where
   it builds the `ot_isp_sensor_exp_func` table passed to
   `ot_mpi_isp_sensor_reg_callback()` (the IMX662-equivalent of
   `cmos_init_sensor_exp_function()`), then resolving one more PC-relative
   literal (the field in `pfn_cmos_get_sns_reg_info`'s struct position) to
   `0xfa474`. Raw bytes at that address *do* look like a sane Thumb
   prologue (`push {r3-r7,lr}; mov r5,r0; ldr r7,[pc,#0x1ac]; ...`) when
   read manually, but the MCP Ghidra tools available in this session had no
   way to force Thumb-mode disassembly at a fresh address the way the
   `arm-linux-gnueabi-objdump -M force-thumb` CLI approach did for the
   original static-table recovery (§2) — every attempt to materialize it as
   a decompilable function produced the same "created but garbage/1-byte"
   result as the other three slots.

**Conclusion at the time: not resolved via static RE.** The gain-register
identity (`0x3508`/`0x3509`) was left as an *assumed* standard OV/OS SCCB
convention, not something confirmed from the binary — the static table
never writes it a meaningful non-zero value (`0x3508=1, 0x3509=0` is a flat
default).

**Superseded — resolved empirically instead (2026-09-05):** §10's real
init-vs-live diff, captured on an actively streaming Ascent unit, shows
`0x3501`/`0x3502` (exposure) and `0x3508` (gain) as the *only* three
registers that moved from their init values while the camera was live, and
nothing else did. This confirms the assumed gain register directly, without
needing to locate the vendor's fast-update table function in the binary —
the bench data answers the question the static/dynamic RE couldn't. `0x3509`
is presumed part of the same field (16-bit-wide gain, mirroring the
exposure register's L/H split) but wasn't observed to move in this capture;
a scene requiring more gain range would confirm it. The
`pfn_cmos_get_sns_reg_info`-equivalent function is still unidentified in the
binary (the Ghidra-side investigation above is left as-is for reference),
but is no longer needed — `sensors/cv610/os02k10/os02k10_cmos.h` can treat
`0x3501`/`0x3502`/`0x3508` as confirmed, not assumed.
