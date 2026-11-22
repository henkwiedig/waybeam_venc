# AR9311 backend (BetaFPV P1)

The Artosyn PROXIMA 9311 air unit of the BetaFPV P1, running the mainline
Linux kernel with MissingLynk's open drivers
([ml-kernel](https://github.com/Missing-Lynk/ml-kernel)). Unlike the other
backends there is no vendor SDK: the camera and the encoder are plain V4L2
devices, so the backend is ioctl code and nothing is dlopen'd.

## Pipeline

```
NT99235 --CSI-2--> VIF --> ISP --> CVISP  ==dma-buf==>  wave5 H.265  --> RTP / frame-shm
 or SC231AI
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
  The block writes a buffer during the frame *after* the tick that armed
  it, so depth 2 hands it back about 1 ms after its last row; the driver's
  default 3 adds a whole frame (16.7 ms at 60 fps) and 1 hands it back
  as the write starts (torn frames, unless the encoder follows the write:
  sub-frame sync, below).  waybeam sets the depth before it opens the
  camera: 1 with `video0.subFrameSync`, 2 without.  The buffer timestamp
  is the hand-back time, so this delay never shows in a capture-to-display
  figure built on it.
- **Encode** (`src/ar9311_encoder.c`): the wave5 mem2mem encoder imports the
  camera planes at their own strides (zero copy) and returns one Annex-B
  access unit per bitstream buffer. H.265 Main, level 4.1, CBR with
  frame- and CU-level rate control, a 100 ms VBV window, no AUDs. Either
  cyclic intra refresh with no periodic IDR (`video0.resilience`, see
  below) or an IDR period from `video0.gopSize` with VPS/SPS/PPS on every
  IDR. The camera
  buffer's capture timestamp travels through the encoder to the frame-shm
  `pts`. `image.mirror` / `image.flip` turn the readout at the sensor when
  its driver has an `orientation` parameter (both P1 sensors, with the
  builder's kernel patches 0007/0008): the frame still arrives top-down, so
  strips and sub-frame sync keep working.  The SC231AI keeps its Bayer
  phase; the NT99235's moves, its driver reports the new order and waybeam
  has the ISP re-read it (`/sys/kernel/debug/ar-isp/bayer`).  Without the
  parameter the encoder turns the picture (V4L2 HFLIP/VFLIP, set at open);
  an encoder flip reads the frame bottom-up, so it turns sub-frame sync and
  with it strips off, with a warning.
- **Auto-exposure** (`src/ar9311_ae.c`) and **white balance**
  (`src/ar9311_awb.c`): see below.
- **Output** (`src/ar9311_runtime.c`): the shared HEVC RTP packetizer over
  `udp://` / `unix://`, or the `frame-shm://` whole-frame ring (the transport
  `ar8030-transport-tx` consumes on OpenIPC's AR8030 targets).
- The bitstream buffers are uncached to the CPU; each access unit is copied
  out once before it is parsed or sent.  Read in place (the Annex-B scan
  goes a byte at a time) it cost the loop ~10 ms per 1080p60 frame and
  starved the encoder.
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
  `patrol`. The shared `intra_refresh_compute()` gives the rows per frame
  in 32-line rows (the CV610 and SigmaStar CTU); wave5 codes 64x64 CTUs, so
  the backend halves them, rounding up, and picks a period for the 17
  64-line rows of 1080p (`ar9311_encoder_refresh_plan()`). The driver
  refreshes rows / period rows per picture and every row of every picture
  for a period above the row count, so the period never exceeds 17.
  `racing` and `sprint` refresh 2 rows (128 lines) per frame, a full
  refresh every 9 frames (150 ms); `endurance` and `patrol` 1 row, every
  17 frames (283 ms, the slowest wave5 refresh at 1080p). The `intra`
  status counts these 64-line rows. The presets with a base/enhance
  reference structure (SVC-T) are rejected because wave5 has no such
  control. Changing the preset needs a restart. `video0.gopSize` is ignored
  while refresh is on.
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

## Sub-frame sync

`video0.subFrameSync` (restart; on in the AR9311 default config) overlaps
the encode with the camera's write.  The CVISP hands each buffer back as it
starts writing it (depth 1) and the wave5 encoder takes it at once,
working through it 64 lines (one CTU row) at a time: before each band it
waits for a toggle in its sub-frame sync register (register mode,
Chips&Media's `ENC_SET_SUB_FRAME_SYNC`).  Nothing in the SoC tells software
how far the camera has got, so the kernel driver releases the bands on a
schedule from the buffer's timestamp, which waybeam passes in at open:

- **Timing**, measured on a P1 by marking the last line of every 64-line
  band before QBUF and polling for the overwrite (240 frames): at depth 1,
  band n is in memory at timestamp + 925 + 947 n us, chroma 60-110 us
  after its luma, spread <= ~100 us.  947 us / 64 lines is the sensor's
  line time (1/60 s over 1125 lines).  waybeam releases band n at
  +1458 + 948 n us: the write, 110 us for chroma and a 400 us margin
  (`ar9311_encoder_sfs_schedule()`).  A band released early would be encoded
  from the previous frame's memory; a late one only costs latency.
- **Effect**: the access unit is ready ~1.7 ms after the frame's last line
  is in memory instead of ~8 ms.  `ar8030-transport-tx`'s capture -> write
  start on a P1 at ~22 Mbit/s: 8.8-9.4 ms off, 2.5-3.0 ms on; 60 fps,
  0 camera skips either way.
- **Timestamps**: at depth 1 the camera timestamp marks the start of the
  write, so waybeam adds one frame period before it publishes it: the
  frame-shm `pts` stays the moment the frame is complete (+0.9 ms), as at
  depth 2, and latency figures keep their meaning.
- **Kernel**: the OpenIPC builder's P1 patches `0004-media-wave5-sub-frame-sync`
  (the `V4L2_CID_WAVE5_SFS_START_US` / `V4L2_CID_WAVE5_SFS_ROW_US` encoder
  controls, the firmware's five one-hot source slots) and
  `0005-media-artosyn-cvisp-depth-per-stream` (a writable `depth`, read at
  REQBUFS).  Without either, waybeam warns and runs without, at depth 2.
  The firmware runs one sub-frame sync instance at a time.

## Strips

`video0.strips` (restart; 2 in the AR9311 default config, 0 or 1 = one
picture, at most 4) encodes the frame as that many horizontal strips, one
wave5 instance each, and joins them into one H.265 picture with as many
slices (`ar9311_merge.c`).  The receiver sees an ordinary 1080p stream.

- **Why**: the AR9311 has one H.26x core (the vendor device tree has one
  `h26x` node), so strips are not encoded in parallel: two strips queued
  together take 8.1 ms against 7.1 ms for the whole frame, and letting both
  into the firmware's command queue at once only saves the host's 0.17 ms
  turnaround.  wave5 also has no early slice output on this firmware (the
  bitstream is packed after the last CTU row).  What strips buy is the
  overlap the vendor's low-delay "pattern" mode uses too
  (`libldrt_pipeline.so` `AR_LDRT_PIPELINE_GetPattern`, 2-4 strips with
  overlaps, decoded per stream and stitched on its ground): a strip is
  queued as soon as the camera has written it and encoded while the camera
  writes the strips below, and its slice goes onto the frame-shm ring at
  once (`VENC_FRAME_FLAG_PARTIAL`), so the link carries the top of the
  picture while the rest is being encoded.
- **Layout** (`ar9311_strips_plan()`, host-tested): whole 64-line CTU rows,
  the spare rows to the top strips, so the last strip, the one that decides
  when the picture is complete, is the smallest: 9/8, 6/6/5, 5/4/4/4 rows.
  Every strip but the last is queued at its last band's release time in the
  sub-frame sync schedule (2 strips: the top at +9042 us); the last one
  follows the camera from its first band with sub-frame sync (2 strips:
  +9990 us).  The bitrate is split by the lines each strip encodes; IDR
  requests go to every
  encoder right before a frame's first strip is queued, so all strips turn
  IDR together.  Needs sub-frame sync and a frame-shm output; without either
  waybeam warns and encodes one picture.
- **Join**: strip 0's SPS gets the full height; the other strips' VPS/SPS/PPS
  are dropped after a check that they match strip 0's, and each slice
  header is rewritten to first_slice_segment_in_pic_flag 0 and its CTU
  address in the whole picture.  A frame whose strips disagree (POC, IDR)
  or fail to join is given up and both encoders are asked for an IDR.
- **Seams are not bit-exact**: a motion vector at a strip's edge (and its
  interpolation taps) reads that encoder's padding while encoding but the
  neighbouring strip's pixels while decoding, and the error spreads until
  the intra refresh wave passes (racing: every 9-10 frames).  In a still
  scene that is invisible (up to ~0.1 % of the pixels, ~80 dB from what
  the encoders meant); a tilt that moves the picture across a seam spreads
  it over the whole picture (hand-tilted P1, 26-44 lines a frame: up to
  40 % of a frame's pixels, ~28k a frame off by 8 levels or more, worst
  frames 29 dB).
- **Overlap** (`video0.stripOverlap`, restart, 0..2, default 1): each strip
  also encodes that many 64-line rows past each of its seams and the join
  drops them, so past the seam a motion vector reads the content the
  decoder has, only coded by the other encoder.  The rows go as slices of
  their own: strip 0 is sliced after its sent rows (9 + 1), a strip with
  rows above its seam in slices of `stripOverlap` rows (wave5 cuts slices
  of one size only, `V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB`); the join
  checks that the cut falls on slice boundaries.  With the overlap the loop
  filter runs across slice edges (`..._LOOP_FILTER_MODE_ENABLED`): each
  encoder deblocks its own slice edges, the decoder the seam, with pixels
  both encoders had close copies of.  Without it the encoders stop the
  loop filter at slice boundaries (`..._DISABLED_AT_SLICE_BOUNDARY`), so
  the decoder treats each seam as the encoders treated their picture
  edges.  Measured on the hand-tilted P1, 1 row against none: pixels off
  by >= 8 levels ~28k -> 0.4-2.9k a frame, the median frame 46 -> 57 dB
  from what the encoders meant, the worst 29 -> 36-43 dB; what is left
  comes in bursts at moves of more than 64 lines a frame (2 rows would
  cover those).  The CTU-row edges stay as smooth as inside a strip (edge
  step 1.3-1.6 against 1.2; 2.7-2.9 with 1-row slices and no filtering
  across them).  Cost: the first part +1.4 ms, the whole picture +1.8 ms
  after the camera's last line, ~12 % more lines encoded (the bitrate
  split makes up for the dropped bits).  A still scene gains nothing.
- **Measured** on a P1 -> VRX Pro at ~22 Mbit/s (PI rate control, 2 x 60 s
  per setting; capture -> ground receiver, PixelPilot latency p50 / p99,
  vsync-lock target, drops per 10 s, tx write time per frame):

  | strips | capture->rx | p50 / p99 | lock target | drops | tx writes |
  | --- | --- | --- | --- | --- | --- |
  | 1 | 28.7 ms | 35.0 / 43.8 | 47.8 | 3.2 | 8.6 ms |
  | 2 | 23.4 | 26.0 / 33.0 | 37.5 | 2.2 | 10.4 |
  | 3 | 20.4 | 23.3 / 34.3 | 38.0 | 4.4 | 11.9 |
  | 4 | 18.9 | 20.7 / 33.7 | 35.6 | 5.2 | 13.1 |

  Every part costs the radio's write path ~1.5 ms, so beyond 2 strips the
  median keeps falling while the tail, and with it the displayed latency,
  hardly moves and drops grow.  2 is the default.  The VRX Pro decodes the
  picture slice by slice; the single-slice stream never used that.
- The transmitter's capture -> write start figure is meaningless with
  strips: the first part is written before the published capture time
  (the depth-2 convention, frame complete + 0.9 ms).

## Auto-exposure

waybeam's own AE loop, written against the kernel interfaces of the
MissingLynk drivers (it shares no code with MissingLynk's GPL `ml-aed`):

- **Metering:** `/sys/kernel/debug/ar-isp/stats_raw`, the ISP's zone grid
  of per-channel 8-bit sums (linear, before gamma and before the white
  balance).  The grid is 36 blocks down the frame, each 16 zones across
  (the driver header calls them columns and rows the other way round; the
  SC231AI's test pattern, a horizontal ramp, shows it).  Zone luma is the
  mean of the channels the bound sensor meters on: all four for the
  NT99235, the two greens (channels 0 and 3) for the SC231AI, whose red and
  blue sit well below green until the white balance lifts them.  The centre
  half of the frame counts double.  A zone at a mean of 240 or more counts
  as clipped.
- **Control:** one decision per new statistics frame at `isp.aeFps` (default
  15 Hz, so the sensor's 1-2 frame latency always has passed).  Exposure is
  the product lines x gain; each decision corrects half the log error
  towards the target, holding inside a +-8 % deadband.  The target is a
  metered luma of 79 for the NT99235 and 40 for the SC231AI.  The
  NT99235's is 56 + 0.5 EV: side by side with a stock P1 in sunshine, stock
  took 86 lines where 56 took 61 (+0.5 EV).  The SC231AI's tuning
  gamma is brighter (its test pattern: raw 14 % -> 148, 69 % -> 253,
  mid-grey near raw 10 %): half the target with the brightest curve pinned,
  +0.5 EV on that since the tone pages follow the exposure, measured
  against a stock P1 at the same light (sunshine: stock 39 lines where a
  target of 28 took 27, +0.53 EV; dusk +0.35 EV; at 40 the two were within
  0.1 EV).  Exposure compensation scales either.  When more
  than 4 % of zones clip it is lowered in proportion (to at most half), so
  a bright sky does not wash out the rest.
- **Actuation:** integration time first (up to one frame: 1123 lines at
  1080p60 for the NT99235, 1114 for the SC231AI), then gain through the sensor's code formula
  (1/16 steps per octave): 1x-64x on the NT99235, 1x-640x on the SC231AI,
  whose driver (builder P1 kernel patch 0009) uses its analogue gain to
  116.5x and the sensor's digital gain above, as the vendor's AE does in the
  dark (measured on a stock P1: 116.5x x 5.5x at 1/60 s); lines absorb the code's rounding, and at
  the line limit the next code is taken only when the lower one falls more
  than 2 % short.  Written to `/sys/module/<sensor>/parameters/{exposure,
  gain}` of whichever sensor driver bound the camera (both use the same
  gain code scale).  The ISP's gain-keyed denoise stages (`rnr/lnr/de3d/cfa/cnf_gain`)
  follow the applied gain and are re-armed through debugfs, and so does
  saturation above 64x (see "Saturation" below).
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
  the sensor's own range, and either is capped at the sensor's ceiling.  `isp.aeFps` is read at start.
- **Tone pages:** the ISP's gamma curve and DRC profile follow the
  exposure, as on stock.  The sensor's tuning file carries the vendor AE's
  exposure table (rising {gain Q8, lines} entries, ~3 % apart, from
  0x0b6524, count at 0x0b6508: 478 for the SC231AI, 366 for the NT99235); the position of the decided exposure on it,
  Q8, goes to the ISP driver's `tone_scalar` parameter and a write of `1` to
  debugfs `tone` rebuilds both pages from the tuning file's band tables
  whenever it has moved by a whole entry.  A stock P1 at dusk (1114 lines x
  31x) sat at ~341, between DRC profiles 3 and 4, where stock's page was; in
  sunshine the position drops below 110 (DRC profile 0, the strongest shadow
  lift), at night it climbs towards profile 5 (shadows held against noise).
  At the same light as a stock P1 (sunshine, same exposure) both pages
  came out within a few counts of stock's, for either camera.  The
  NT99235's file leaves DRC profiles 4 and 5 empty (its dark end); there
  the driver turns DRC off, as a stock P1 with that camera does at night
  (kernel patch 0012; at 1123 lines x 64x both then sit within a code or
  two of each other).  The ISP's cm and cm2 stages (exposure-keyed
  saturation) read the same scalar: the AE writes it to `cm_trigger` and
  `cm2_trigger` too and re-latches them through debugfs `colour` (builder
  P1 kernel patches 0010/0012; without that node they stay at their loaded
  rows).  cm2 runs on the NT99235 and is gated off on the SC231AI.
- **Status:** `GET /api/v1/ae` (luma, target, clipped zones, lines,
  exposure in us, gain, decision and move counters, the tone scalar as
  `toneScalar` (-1 while pinned), limits).

Measured on a P1 indoors: a covered lens reaches 64x within about 1 s and
recovers as fast when uncovered; a steady scene holds with no moves.  An
SC231AI in a dark room (lamp off, daylight behind blinds) settles near 544x
on target, where the 64x ceiling left it 3.6 stops under (luma 2.3 of 28).  Without the statistics or sensor interfaces AE
stays off and the camera keeps the sensor driver's fixed exposure.

## White balance

The ISP's zone statistics are taken before its white balance stage, and
everything after that stage is colour-neutral (the SC231AI's test pattern,
a ramp with all four Bayer channels equal, comes out grey at every level
with unity gains).  So the gains that render a neutral surface grey are
simply green/red and green/blue of that surface, and the AWB only has to
find the neutral surfaces of a frame:

- **Calibration:** per sensor, the raw colour of grey at three points of
  its illuminant locus, as gains (Q8): SC231AI warm 269/834 (2800 K), mid
  400/644 (4000 K) and daylight 537/474 (6500 K), joined by straight lines
  in (ln R/G, ln B/G).  A position t on the locus runs 0 (warm), 1 (mid),
  2 (daylight), on past either end along the end segment.  The mid and
  daylight points are a stock P1's own balance on the same unit: 399/646 at
  dusk, where its colour matrix was tuning bank 1 (3700-4300 K), and 537/474
  in sunshine, bank 3 (6200-6700 K); the sunlit white walls of that scene
  measured 504-552/465-495 in our statistics.  A straight line through warm
  and mid reached daylight's red gain with 10 % too much blue: the locus
  bends.  The warm point is a lamp (310/962 as first measured, scaled by the
  0.867 a monitor-and-lamp calibration had left grey short of green).  The
  NT99235 has no calibration yet; for it the AWB stays off and the gains
  stay as loaded.
- **Estimate:** one per new statistics frame at `isp.awbFps` (default 15).
  Zones with a channel at 235 or more (clipped) or green under 8 (noise)
  are skipped.  A zone counts as neutral when its colour lies within 0.18
  (ln units) of the locus, between t -0.5 (~2400 K) and 2.4 (~8700 K,
  shade); coloured objects sit off the locus and do not pull the balance.
  The neutral zones, each counted once (weighted by brightness, a lit lamp
  shade outvoted the grey the light fell on), give a position on the locus,
  and that point's G/R and G/B are the gains.  Fewer than 24 neutral zones:
  no estimate, the gains hold.
- **Daylight:** orange, brown and yellow surfaces lie along the warm end of
  the locus and pass for greys under a warm light.  On a sunny autumn
  garden 35 of 45 "neutral" zones were such objects (tree, hedge, brick)
  and pulled the balance to 458/591 where stock and the white walls said
  537/474.  At an exposure only daylight gives, neutral zones therefore
  start at t 1.2 (~4300 K): up to a tone scalar of 180 (272 lines at 1x;
  the sun sits at 70-115), relaxing linearly to the whole locus at 260
  (1114 lines at 2.8x; a lamp-lit room is near 390).  When that window
  holds fewer than 24 zones (a sunset) the whole locus counts.  Same garden:
  527/483, stock 537/474.
- **Actuation:** each decision moves the red and blue gains 15 % of the
  way (log domain), holding inside a 1 % deadband, so the balance settles
  in about a second without visible steps.  Written to the ISP driver's
  `/sys/kernel/debug/ar-isp/white_balance` (`"red green blue"`, Q8, green
  stays 256).
- **Colour stages:** the colour temperature of the applied gains (their
  position on the locus, linear in mired between the points) goes to the
  ISP driver's `colour_temp` parameter, re-latched by a write to debugfs
  `colour`, whenever it has moved by 2 mired (builder P1 kernel patch 0010).
  The driver picks ccm1's matrix from the tuning file's four banks by their
  kelvin bands, blending in the gaps, and cm's CT column the same way, as
  stock's AWB does: in sunshine bank 3 and cm 1.0, word for word stock's
  registers, where the pinned bank 0 and column 0 (0.9) left foliage and
  sky 12 % less saturated.  Without the parameter the colour stages stay
  as loaded.
- **Modes (live):** `isp.awbMode` `auto`, or `ct_manual` to fix the balance
  at `isp.awbCt` kelvin (on the locus, linear in mired between the
  calibration points; applies at once).  `isp.awbFps` `0` stops the
  loop and leaves the gains where they are.
- **Status:** `GET /api/v1/awb` (mode, gains, the estimate's neutral zones,
  R/G, B/G and colour temperature, the colour temperature last given to the
  ISP's colour stages as `ccmCt` (0 none), decision and move counters;
  `"mode": "off"` when the loop is not running).

Measured on a P1 (SC231AI), with the first calibration: from the loader's
daylight gains under a warm lamp the loop settled at 309/960 (2755 K) within
about a second; a hand in the frame does not move it.  With the current one
(2026-11-21, dusk with a street lamp, the scene stock had balanced at
399/646): 351/700 (4437 K), frame mean U/V -0.1/-0.5, where the first
calibration gave 330/922 (3119 K) and a blue picture.  With the
three-point locus (2026-11-22, sunny garden, stock captured 72 s apart):
528/481 (6255 K; stock 546/466), ccm1 bank 3 and cm 32 as stock's, frame
mean U/V +1.6/-4.0 against stock's +0.3/-3.9, gamma and DRC pages within a
few counts.  6500 K manual gives 537/474, 4000 K 400/644.

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
  Above 64x the AE scales it down with the gain, linearly in log2(gain) to
  35 % at 640x: there an SC231AI's shadows came out purple (green clipped,
  red and blue lifted) where the vendor's picture stays near grey.  That
  ramp stands in for the ISP's cm stage, which the vendor lowers with the
  exposure; when the AE drives cm itself (tone pages following the
  exposure, kernel patch 0010) the ramp is off, as stock's colour-space
  matrix stays unscaled at night: both together left the picture grey
  (chroma 2.5 against stock's 8.0 at 640x; without the ramp 9.4).
- **Contrast** (`csc.contr`, 50 = tuned): the gamma curve's output, scaled
  about mid-grey.
- **Sharpness** (`sharpen.manual.*`, 2048 = tuned): lnr register 0x3d14, the
  two fields the vendor's acutance control drives, scaled the vendor's way
  (to zero below 100 %, towards the field maximum above).  100 % is, for
  the NT99235, the value the driver replays (the vendor's own setting sits
  low, so most of the visible range is above 100 %: edge energy 2.9 / 3.3 /
  6.9 at 0 / 100 / 200 %); for the SC231AI, the tuning file's own
  sharpening ladder at the current gain, as on a stock P1 (60 at 1x easing
  to 18 at 640x, five times the replayed NT99235 value at 1x; kernel patch
  0011, `sharpness_tuned=1` from the loader).  With it, its defect-pixel
  thresholds (the tuning file's gain ladder, `dpc_tuned=1`), raw noise
  reduction sub-stage bits (`rnr_mode=0x0c`, bit 4 by the ladder) and line length
  (`line_length=2240`) are stock's too: on a sunny garden, stock captured
  3 minutes apart, edge energy 18.4 against stock's 18.8 where it was 8.7,
  mid-tone texture 30.3 against 30.2, leaf highlights without the white
  sparkle the replayed defect thresholds let through.
- **Exposure compensation** (`exposure.auto.compensation`): the AE target,
  56 x 2^ev for +-2 EV.

The first three are `ar_isp` module parameters (`saturation`, `contrast`,
`sharpness`, percent) added by the OpenIPC builder's P1 kernel patch
`0002-media-artosyn-isp-picture-controls.patch`, re-latched through the
driver's debugfs `ladders` and `tone` files. Without the patch a write fails
and the picture stays the tuned one.

## Runtime prerequisites

The kernel modules `nt99235` or `sc231ai`, `ar-csi2 ar-vif ar-isp ar-cvisp
wave5`, the Wave521C firmware (`/lib/firmware/cnm/wave521c_k3_codec_fw.bin`)
and the sensor's ISP tuning (`/lib/firmware/artosyn/nt99235-tuning-preview-fpv.bin`
or `sc231ai-tuning-preview-fpv.bin`).  `init.d/load-ar9311-camera start`
loads the sensor drivers in turn until one binds (one device-tree node names
both; each driver reads the chip ID at probe), then the chain with the bound
sensor's tuning file (named explicitly for the NT99235 too, so the AE can
read its exposure table), for the NT99235 `drc_on=1 sharpness_tuned=1`
(DRC on by its own gate and both table lengths, kernel patch 0012; never
`gates=1` on this camera: its file's other gates start statistics engines
the driver gives no buffer, which hung a P1), and for the SC231AI daylight
white balance gains to start from and `gates=1` (the ISP takes the tuning
file's stage gates: the
replayed NT99235 trace had cm2 on and DRC plus three lens-shading enables
off, where this sensor's tuning and a stock P1 have them the other way), plus
`sharpness_tuned=1 dpc_tuned=1 rnr_mode=0x0c line_length=2240` (sharpening,
defect-pixel thresholds, raw noise reduction and line length as a stock
SC231AI P1 runs them, see "Sharpness"; drivers without kernel patch 0011
ignore them); `init.d/S95waybeam.ar9311` runs it before starting the daemon.

## Configuration surface (first slice)

Exactly one mode, which is all the open camera chain delivers today:
`video0.size=1920x1080`, `video0.fps=60`, `sensor.mode` 0.

| Field | Class |
|---|---|
| `video0.bitrate` | live (wave5 re-targets the rate controller) |
| `outgoing.enabled`, `outgoing.server`, `outgoing.maxPayloadSize`, `system.verbose` | live |
| `isp.gainMax`, `isp.gainMin`, `isp.shutterMaxUs`, `isp.shutterMinUs` (AE limits), `isp.antiFlickerHz` | live |
| `isp.awbMode`, `isp.awbCt`, `isp.awbFps` (white balance) | live |
| `isp.aeFps` (AE decision rate) | restart |
| `image.mirror`, `image.flip` (both = 180 degrees; at the sensor, encoder-side on a kernel without the orientation parameter), `video0.resilience` (intra-refresh presets, see above), `video0.subFrameSync`, `video0.strips`, `video0.stripOverlap` (see above), `video0.gopSize` (IDR period, max 2047 frames; only with resilience `off`), `video0.minQp`/`maxQp`, `outgoing.connectedUdp`, `system.webPort` | restart |

`GET /request/idr` works (forced keyframe on the next frame, rate limited).
Picture controls are on `/api/v1/iq` (above), not config fields.
Everything else — ISP tuning, audio, recording, snapshot, ROI,
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
