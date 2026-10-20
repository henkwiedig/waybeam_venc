# ROI (Region of Interest) QP — Horizontal Bands

Status: **Implemented**. Signed center-emphasis ROI with stepped horizontal
bands, verified on SSC30KQ (Infinity6E) at 1080p.

## Implementation

Full-height horizontal bands centered on the frame, with signed delta QP
tapering from center (strongest) to edges (weakest). Higher-index ROI
regions override lower ones in overlap zones, so the innermost (narrowest)
band gets the full QP delta.

```
┌─────┬─────┬──────────┬─────┬─────┐
│     │     │          │     │     │
│ +qp │+qp/2│    0     │+qp/2│ +qp │   full height
│     │     │ (center) │     │     │   bands
│     │     │          │     │     │
└─────┴─────┴──────────┴─────┴─────┘
  edge  mid    center    mid   edge     (2 steps, negative roiQp,
                                         |roiQp| shown as qp)
```

- Negative `roiQp`: higher quality in center. **Programmed as coarser edges,
  not a sharper centre (from 0.84.2)**: every delta is lifted by `|roiQp|`,
  so the innermost band is at 0, the outer bands at `|roiQp|` minus their
  taper, and a full-frame region at index 0 (below every band) puts the rest
  of the frame at `+|roiQp|`. That is `roiSteps + 1` regions.
- Positive `roiQp`: lower quality in center (softer, higher QP); the bands
  are programmed as-is, `roiSteps` regions.
- `0`: ROI disabled (all regions cleared)

Why the lift: ROI deltas are relative to the frame QP the rate controller
picks. A discount on the centre made CBR raise the frame QP to pay for it,
and once `frame QP + |roiQp|` passed the QP ceiling the frame QP pinned there
and the bitrate target was missed by multiples. Raising the edges instead
lets CBR lower the frame QP, so the centre gets the bits the edges gave up
and the target holds. CV610 bench, OS02K10 1080p100, `roiQp -20`,
`roiSteps 2`, `roiCenter 0.4`, static scene, frame QP from the sidecar:

| Target | ROI off | ROI on, before 0.84.2 | ROI on, from 0.84.2 |
|---|---|---|---|
| 1000 kbps | 990 kbps, QP 45.5 | **25000 kbps, QP 51** | 995 kbps, QP 42.7 |
| 4000 kbps | 3930 kbps, QP 37.2 | | 3955 kbps, QP 35.3 |
| 10000 kbps | 9770 kbps, QP 34.9 | | 9895 kbps, QP 32.5 |
| 20000 kbps | 19600 kbps, QP 33.0 | | 19720 kbps, QP 30.1 |

At 1000 kbps the frame QP is already close to 51, so the edges hit the
H.265 ceiling and have little left to give: the centre gains about 3 QP
there. `roiSteps 4` behaved the same (1000 kbps, QP 42.5-43.2).

Same bench with the `cam_os02k10_100fps_xg1_day_colortrans.bin` sensor bin,
which codes far cheaper (QP 37 instead of 45.5 at 1000 kbps):

| Target | ROI off | ROI on, before 0.84.2 | ROI on, from 0.84.2 |
|---|---|---|---|
| 1000 kbps | 985 kbps, QP 37.2 | **4990 kbps, QP 51** | 985 kbps, QP 35.1 |
| 4000 kbps | 3920 kbps, QP 32.1 | | 3955 kbps, QP 30.8 |
| 10000 kbps | 9780 kbps, QP 30.0 | | 9895 kbps, QP 28.1 |
| 20000 kbps | 19655 kbps, QP 28.3 | | 19775 kbps, QP 25.9 |

After the old ROI was switched off, CBR undershot to ~220 kbps at QP 48-50
for several seconds; with the new ROI it went straight back to target.
The frame QP falls by only 1.3-2.4 here even though the edges have plenty of
room below 51, so the frame QP understates how far the bits moved, or the
rate controller only partly hands the edge savings to the centre. Not
measured in the bitstream.

All rect coordinates are aligned to 32 pixels for H.265 CTU compatibility.
The full-frame region is aligned the same way, so on a 1080-row frame the
bottom 24 rows are outside every region and stay at delta 0.

## Config

```json
"fpv": {
    "roiEnabled": true,
    "roiQp": -18,
    "roiSteps": 2,
    "roiCenter": 0.25
}
// An EXAMPLE of ROI switched on, not the shipped defaults.
// Those are roiEnabled:false, roiQp:-20, roiSteps:2, roiCenter:0.4.
```

- `roiEnabled`: bool, MUT_LIVE — enable/disable ROI bands. Ships **`false`**
  since 0.76.0, paired with a non-zero `roiQp` so switching it on does
  something. It previously shipped `true` alongside `roiQp: 0`, i.e. on with
  nothing to apply.
- `roiQp`: int, MUT_LIVE, range `-20..20` (was `-30..30` before 0.79.0)
- `roiSteps`: uint16, MUT_LIVE, range `1..4`
- `roiCenter`: double, MUT_LIVE, range `0.1..0.9`

## Example

With `roiCenter=0.33`, `roiQp=-18`, `roiSteps=2`:

| Region | Width | QP delta |
|------|-------|----------|
| Full frame (index 0) | 100% of frame | +18 |
| Outer band (index 1) | ~67% of frame | +9 |
| Center band (index 2) | ~33% of frame | 0 |

The outer band covers the middle two thirds, the center band overlaps the
middle third. Since higher-index regions override, the center gets 0, the
ring around it +9 and the edges +18.

## Live API

```text
GET /api/v1/set?fpv.roi_enabled=true
GET /api/v1/set?fpv.roi_qp=-18
GET /api/v1/set?fpv.roi_steps=2
GET /api/v1/set?fpv.roi_center=0.25
GET /api/v1/set?fpv.roi_qp=0          # disable
```

## ABI Findings

### MI_VENC_RoiCfg_t

```c
typedef struct {
    MI_U32  u32Index;
    MI_BOOL bEnable;
    MI_BOOL bAbsQp;
    MI_S32  s32Qp;
    MI_VENC_Rect_t stRect;
} MI_VENC_RoiCfg_t;
```

- Up to 5 ROI regions used (index 0-4)
- `MI_VENC_Rect_t` uses `MI_U32` fields
- Unused regions are explicitly disabled

### Functions

```c
MI_S32 MI_VENC_SetRoiCfg(MI_VENC_CHN chn, MI_VENC_RoiCfg_t *cfg);
MI_S32 MI_VENC_GetRoiCfg(MI_VENC_CHN chn, MI_U32 idx, MI_VENC_RoiCfg_t *cfg);
```

On Maruko, both functions take an additional `MI_VENC_DEV dev` parameter.

## Constraints

- 32-pixel alignment required for ROI rects
- Signed delta range: `-30..30` (SDK rejects values outside this)
- Full-frame height must be 32-pixel aligned (1080 → 1056 used)

## Previous Implementation

The original implementation used concentric centered rectangles (centerbox
mode) with unsigned QP and up to 8 regions. This was replaced with the
simpler horizontal band layout which better matches the FPV use case
(center of frame is always the most important regardless of vertical
position).
