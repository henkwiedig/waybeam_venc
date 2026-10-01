# AR9311 backend (BetaFPV P1)

The Artosyn PROXIMA 9311 air unit of the BetaFPV P1, running the mainline
Linux kernel with MissingLynk's open drivers
([ml-kernel](https://github.com/Missing-Lynk/ml-kernel)). Unlike the other
backends there is no vendor SDK: the camera and the encoder are plain V4L2
devices, so the backend is ioctl code and nothing is dlopen'd.

## Pipeline

```
NT99235 --CSI-2--> VIF --> ISP --> CVISP  ==dma-buf==>  wave5 H.265  --> RTP / frame-shm
          (MissingLynk media/artosyn drivers,                (mainline wave5 +
           one V4L2 capture node "ar-cvisp")                  MissingLynk patches)
```

- **Capture** (`src/ar9311_capture.c`): the `ar-cvisp` node, 1920x1080
  YUV420M in three planes, luma stride 2048. Eight MMAP buffers; every plane
  is exported once as a dma-buf. The node completes a buffer only while
  `depth + 1` others (`/sys/module/ar_cvisp/parameters/depth`) stay queued
  under it, so at most four frames are out with the encoder -- enough for it
  to fall behind during a keyframe and catch up; a frame beyond that is
  requeued unencoded (`cameraSkips` in `/api/v1/transport/status`).
- **Encode** (`src/ar9311_encoder.c`): the wave5 mem2mem encoder imports the
  camera planes at their own strides (zero copy) and returns one Annex-B
  access unit per bitstream buffer. H.265 Main, level 4.1, CBR with
  frame- and CU-level rate control, a 100 ms VBV window, no AUDs. Either
  cyclic intra refresh with no periodic IDR (`video0.resilience`, see
  below) or an IDR period from `video0.gopSize` with VPS/SPS/PPS on every
  IDR. The camera
  buffer's capture timestamp travels through the encoder to the frame-shm
  `pts`. `image.mirror` / `image.flip` are done here too (V4L2 HFLIP/VFLIP,
  set at open): mirroring at the sensor would move the Bayer phase the ISP
  is tuned for.
- **Auto-exposure** (`src/ar9311_ae.c`): see below.
- **Output** (`src/ar9311_runtime.c`): the shared HEVC RTP packetizer over
  `udp://` / `unix://`, or the `frame-shm://` whole-frame ring (the transport
  `ar8030-transport-tx` consumes on OpenIPC's AR8030 targets).
- One thread: `poll()` over the camera and the encoder, then drain finished
  access units, return source buffers to the camera, feed new frames.

Validated on a P1 (RAM-booted, flashless): 1080p60 sustained with no
skipped frames, clean decode, CBR tracking its target across 2/8/30 Mbit/s
including live changes.

## Intra refresh (GDR)

A 1080p60 IDR is about 270 KB, 13 times a P-frame. Over the AR8030 link it
took 70-99 ms to write, the RTP stream gapped, and the ground's decoder asked
for yet another IDR: a visible stall every second or two. With a
`video0.resilience` refresh preset the encoder never sends a periodic IDR.
Instead every P-frame intra-codes a band of CTU rows, and the band walks down
the picture (wave5 cyclic intra refresh, `intra_refresh_mode` CTU rows).

- **Presets:** `off`, `rescue`, `quality`, `sprint`, `racing`, `endurance`,
  `patrol`. The shared `intra_refresh_compute()` gives the rows per frame,
  and the backend rounds them so that the period divides the 34 CTU rows of
  1080p. For example `racing` is 4 rows per frame and a full refresh every
  9 frames (150 ms). The presets with a base/enhance reference structure
  (SVC-T) are rejected because wave5 has no such control. Changing the
  preset needs a restart. `video0.gopSize` is ignored while refresh is on.
- **Stream:** one IDR at start, and then on `GET /request/idr`. Frame-shm
  frames carry the GDR position and cycle length in their meta. Each refresh
  wave's head gets VPS/SPS/PPS prepended (RTP and frame-shm), so a receiver
  that joins mid-stream can decode within one cycle.
- **Status:** `GET /api/v1/intra/status` and `/api/v1/resilience/status`.
  `gop.effective_sec` reads 0 when there is no periodic IDR.
- **Kernel:** mainline wave5 has the refresh plumbing but no V4L2 control for
  it. The OpenIPC builder adds
  `V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD(_TYPE)` (cyclic) through a P1
  kernel patch. On a kernel without it, a refresh preset fails the encoder
  open and the message names the missing patch.

Measured on a P1 with `racing` at 14 Mbit/s: P-frames average 28 KB and
peak at 39 KB, and the only I-frame is 51 KB. Over the radio for 60 s:
0 RTP gaps, 0 IDR requests from the ground, frame writes 6-10 ms at most
(70-99 ms with IDRs), capture to write 12 ms (25-31 ms), and 0 camera skips.

## Auto-exposure

waybeam's own AE loop, written against the kernel interfaces of the
MissingLynk drivers (it shares no code with MissingLynk's GPL `ml-aed`):

- **Metering:** `/sys/kernel/debug/ar-isp/stats_raw`, the ISP's 36x16 zone
  grid of per-channel 8-bit sums (linear, before gamma).  Zone luma is the
  mean of the four channel means; the centre half of the frame counts
  double.  A zone at a mean of 240 or more counts as clipped.
- **Control:** one decision per new statistics frame at `isp.aeFps` (default
  15 Hz, so the sensor's 1-2 frame latency always has passed).  Exposure is
  the product lines x gain; each decision corrects half the log error
  towards the target, holding inside a +-8 % deadband.  The target is a
  metered luma of 56; when more than 4 % of zones clip it is lowered in
  proportion (to at most half), so a bright sky does not wash out the rest.
- **Actuation:** integration time first (up to one frame, 1123 lines at
  1080p60), then analogue gain through the sensor's code formula
  (1x-64x, 1/16 steps per octave); lines absorb the code's rounding, and at
  the line limit the next code is taken only when the lower one falls more
  than 2 % short.  Written to `/sys/module/nt99235/parameters/{exposure,
  gain}`.  The ISP's gain-keyed denoise stages (`rnr/lnr/de3d/cfa/cnf_gain`)
  follow the applied gain and are re-armed through debugfs.
- **Anti-flicker (live):** `isp.antiFlickerHz` `0` (off), `50` or `60`.
  Mains light ripples at twice its frequency, so with it on, an exposure of
  at least one half period gets an integration time that is a whole number
  of half periods (snapped down, never up) and the gain makes up the rest,
  rounded to the nearest code.  At 50 Hz that caps a dim scene at 10 ms
  (675 lines) and costs about 1.7x gain against a full frame; at 60 Hz two
  half periods are 1125 lines, taken as the 1123 the sensor allows.
  Shorter exposures (bright scenes) and a shutter cap below one half period
  leave it inactive.  Measured on a P1: 50 Hz -> 675 lines (10.000 ms) at
  11.5x, 60 Hz -> 1123 lines at 7x, luma held throughout.
- **Limits (live):** `isp.shutterMaxUs` / `shutterMinUs` (converted to
  lines, 14.815 us each) and `isp.gainMax` / `gainMin` (1024 = 1x); 0 means
  the sensor's own range.  `isp.aeFps` is read at start.
- **Status:** `GET /api/v1/ae` (luma, target, clipped zones, lines,
  exposure in us, gain, decision and move counters, limits).

Measured on a P1 indoors: a covered lens reaches 64x within about 1 s and
recovers as fast when uncovered; a steady scene holds with no moves.  Not
yet in this loop: tone-curve selection (the ISP stays on its default), AWB.  Without the statistics or sensor interfaces AE
stays off and the camera keeps the sensor driver's fixed exposure.

## Picture controls (`/api/v1/iq`)

AR9311 serves the IQ routes in the CV610 shape (`_schema` plus groups), with
the subset a ground-station menu written for CV610 uses, so the same
PixelPilot/gsmenu drives both air units; the web UI's IQ tab renders from
the schema. Live only, as on CV610: a restart returns to the tuned picture.
The field table is in the contract ("AR9311 subset"); behind it:

- **Saturation** (`saturation.manual.saturation`, 128 = tuned): the six
  chroma coefficients of the ISP's colour-space matrix (bank 0x3c00), scaled.
  The vendor's matrix writer packs the Cb/Cr offsets (the 128 mid-point)
  into the same words; those stay, or a desaturated picture turns green.
- **Contrast** (`csc.contr`, 50 = tuned): the gamma curve's output, scaled
  about mid-grey.
- **Sharpness** (`sharpen.manual.*`, 2048 = tuned): lnr register 0x3d14, the
  two fields the vendor's acutance control drives, scaled the vendor's way
  (to zero below 100 %, towards the field maximum above) from the value the
  driver replays. The vendor's own setting sits low, so most of the visible
  range is above 100 % (edge energy 2.9 / 3.3 / 6.9 at 0 / 100 / 200 %).
- **Exposure compensation** (`exposure.auto.compensation`): the AE target,
  56 x 2^ev for +-2 EV.

The first three are `ar_isp` module parameters (`saturation`, `contrast`,
`sharpness`, percent) added by the OpenIPC builder's P1 kernel patch
`0002-media-artosyn-isp-picture-controls.patch`, re-latched through the
driver's debugfs `ladders` and `tone` files. Without the patch a write fails
and the picture stays the tuned one.

## Runtime prerequisites

The kernel modules `nt99235 ar-csi2 ar-vif ar-isp ar-cvisp wave5`, the
Wave521C firmware (`/lib/firmware/cnm/wave521c_k3_codec_fw.bin`) and the ISP
tuning (`/lib/firmware/artosyn/nt99235-tuning-preview-fpv.bin`).
`init.d/load-ar9311-camera start` loads the modules;
`init.d/S95waybeam.ar9311` runs it before starting the daemon.

## Configuration surface (first slice)

Exactly one mode, which is all the open camera chain delivers today:
`video0.size=1920x1080`, `video0.fps=60`, `sensor.mode` 0.

| Field | Class |
|---|---|
| `video0.bitrate` | live (wave5 re-targets the rate controller) |
| `outgoing.enabled`, `outgoing.server`, `outgoing.maxPayloadSize`, `system.verbose` | live |
| `isp.gainMax`, `isp.gainMin`, `isp.shutterMaxUs`, `isp.shutterMinUs` (AE limits), `isp.antiFlickerHz` | live |
| `isp.aeFps` (AE decision rate) | restart |
| `image.mirror`, `image.flip` (encoder-side, both = 180 degrees), `video0.resilience` (intra-refresh presets, see above), `video0.gopSize` (IDR period, max 2047 frames; only with resilience `off`), `video0.minQp`/`maxQp`, `outgoing.connectedUdp`, `system.webPort` | restart |

`GET /request/idr` works (forced keyframe on the next frame, rate limited).
Picture controls are on `/api/v1/iq` (above), not config fields.
Everything else — ISP tuning, AWB, audio, recording, snapshot, ROI,
SVC-T resilience presets, slicing, debug OSD — is advertised unsupported and
rejected: `ar9311_validate_config()` (`src/ar9311_validation.c`) and the
field list in `src/venc_api.c` say exactly what runs.

## Known limits

- **The VPU is reset before every encoder open.** The wave5 firmware keeps
  hidden state across encoder instances: within one boot, a later H.265
  instance can come up in a "garbage mode" that emits a *valid* bitstream of
  pure noise at about 2.5x the target bitrate, or watchdog (MissingLynk
  `docs/wave5-encoder-instance-reuse.md`; reproduced on a P1). Probing an
  instance for it does not work reliably: with a static input the garbage
  mode can code small frames too. Instead `ar9311_encoder_open()` writes
  `1` to `/sys/kernel/debug/wave5/recover` (MissingLynk's wave5 debug hook:
  codec block reset + firmware re-download) whenever
  `/sys/kernel/debug/wave5/state` lists no live instance. Verified on a P1
  whose VPU was already poisoned: service restarts and an API respawn all
  streamed clean video. The hook is documented upstream as an operator tool
  because it destroys live instances; the backend only uses it when there
  are none. Without debugfs (`load-ar9311-camera` mounts it) the reset is
  skipped with a warning, and a restart within the boot may encode noise.
- Camera restarts within a boot have worked on the P1 (ml-kernel
  `docs/camera-stack.md` reports CVISP writing only on the first bring-up
  after a RAM boot from the vendor system, which this flashless board never
  does).

## Build

```sh
make build SOC_BUILD=ar9311 AR9311_CC=/path/to/aarch64-openipc-linux-musl-gcc
make stage SOC_BUILD=ar9311   # + S95waybeam, load-ar9311-camera, waybeam.json
```

`AR9311_CC` defaults to the OpenIPC builder's `ar9311_fpv_artlynk-p1` build
tree next to this checkout.
