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
  under it, so at most three frames are out with the encoder; a frame beyond
  that is requeued unencoded (`cameraSkips` in `/api/v1/transport/status`).
- **Encode** (`src/ar9311_encoder.c`): the wave5 mem2mem encoder imports the
  camera planes at their own strides (zero copy) and returns one Annex-B
  access unit per bitstream buffer. H.265 Main, level 4.1, CBR with
  frame- and CU-level rate control, a 100 ms VBV window, IDR period from
  `video0.gopSize`, VPS/SPS/PPS repeated on every IDR, no AUDs. The camera
  buffer's capture timestamp travels through the encoder to the frame-shm
  `pts`.
- **Auto-exposure** (`src/ar9311_ae.c`): see below.
- **Output** (`src/ar9311_runtime.c`): the shared HEVC RTP packetizer over
  `udp://` / `unix://`, or the `frame-shm://` whole-frame ring (the transport
  `ar8030-transport-tx` consumes on OpenIPC's AR8030 targets).
- One thread: `poll()` over the camera and the encoder, then drain finished
  access units, return source buffers to the camera, feed new frames.

Validated on a P1 (RAM-booted, flashless): 1080p60 sustained with no
skipped frames, clean decode, CBR tracking its target across 2/8/30 Mbit/s
including live changes.

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
- **Limits (live):** `isp.shutterMaxUs` / `shutterMinUs` (converted to
  lines, 14.815 us each) and `isp.gainMax` / `gainMin` (1024 = 1x); 0 means
  the sensor's own range.  `isp.aeFps` is read at start.
- **Status:** `GET /api/v1/ae` (luma, target, clipped zones, lines,
  exposure in us, gain, decision and move counters, limits).

Measured on a P1 indoors: a covered lens reaches 64x within about 1 s and
recovers as fast when uncovered; a steady scene holds with no moves.  Not
yet in this loop: tone-curve selection (the ISP stays on its default),
mains anti-flicker, AWB.  Without the statistics or sensor interfaces AE
stays off and the camera keeps the sensor driver's fixed exposure.

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
| `isp.gainMax`, `isp.gainMin`, `isp.shutterMaxUs`, `isp.shutterMinUs` (AE limits) | live |
| `isp.aeFps` (AE decision rate) | restart |
| `video0.gopSize` (IDR period, max 2047 frames), `video0.minQp`/`maxQp`, `outgoing.connectedUdp`, `system.webPort` | restart |

`GET /request/idr` works (forced keyframe on the next frame, rate limited).
Everything else — ISP tuning, AWB, audio, recording, snapshot, ROI,
resilience presets, slicing, debug OSD — is advertised unsupported and
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
