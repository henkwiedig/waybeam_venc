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
- **Output** (`src/ar9311_runtime.c`): the shared HEVC RTP packetizer over
  `udp://` / `unix://`, or the `frame-shm://` whole-frame ring (the transport
  `ar8030-transport-tx` consumes on OpenIPC's AR8030 targets).
- One thread: `poll()` over the camera and the encoder, then drain finished
  access units, return source buffers to the camera, feed new frames.

Validated on a P1 (RAM-booted, flashless): 1080p60 sustained with no
skipped frames, clean decode, CBR tracking its target across 2/8/30 Mbit/s
including live changes.

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
| `video0.gopSize` (IDR period, max 2047 frames), `video0.minQp`/`maxQp`, `outgoing.connectedUdp`, `system.webPort` | restart |

`GET /request/idr` works (forced keyframe on the next frame, rate limited).
Everything else — ISP/AE controls, audio, recording, snapshot, ROI,
resilience presets, slicing, debug OSD — is advertised unsupported and
rejected: `ar9311_validate_config()` (`src/ar9311_validation.c`) and the
field list in `src/venc_api.c` say exactly what runs.

## Known limits

- **Restart needs a reboot.** The wave5 firmware keeps hidden state across
  encoder instances: within one boot, a second H.265 instance can come up
  in a "garbage mode" that emits a *valid* bitstream of pure noise at about
  2.5x the target bitrate (seen on the bench; MissingLynk
  `docs/wave5-encoder-instance-reuse.md`). So anything that re-creates the
  encoder — a restart-class API change (respawn), `S95waybeam restart`, a
  crash — can leave the stream broken until the next boot. Change
  restart-class fields in `/etc/waybeam.json` and reboot.
- **No auto-exposure.** The ISP runs at the sensor driver's fixed exposure
  and gain (`nt99235` module parameters `exposure`, `gain`; see
  `load-ar9311-camera`). MissingLynk's `ml-aed` closes the AE loop through
  the same parameters; porting it is the next step for image quality.
- The camera chain writes frames reliably on its first bring-up of a boot
  (ml-kernel `docs/camera-stack.md`), which matches the one-instance rule
  above.

## Build

```sh
make build SOC_BUILD=ar9311 AR9311_CC=/path/to/aarch64-openipc-linux-musl-gcc
make stage SOC_BUILD=ar9311   # + S95waybeam, load-ar9311-camera, waybeam.json
```

`AR9311_CC` defaults to the OpenIPC builder's `ar9311_fpv_artlynk-p1` build
tree next to this checkout.
