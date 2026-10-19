# Guided camera calibration

Open **Calibration** in the browser console, or **Camera → Lens calibration**
in Android. The standalone page is `http://10.42.0.1:8080/ui/calibration.html`.

The suite corrects camera geometry. It does not change the approved ISP colour
profile, exposure settings, robot tuning or motor state.

## Workflow

1. **Prepare the target.** ChArUco is the default; checkerboard is also supported.
   Download the PDF, print at 100%/actual size, and measure the 50 mm reference
   line and one square. Enter the measured square size if the print differs.
2. **Start automatic capture.** Keep the bot idle and the camera mechanically
   fixed. Show the target in the highlighted region and hold it still. The suite
   detects corners, checks sharpness/brightness/stability, and captures useful
   views without a capture button.
3. **Follow the guidance.** Move the board across the image, change its distance,
   and tilt an edge towards the camera. The coverage grid, accepted count and
   live guidance show what is still missing. Duplicate views are rejected.
4. **Automatic analysis.** At the requested view count (default 32), analysis
   starts once at least six image regions, two scales and three tilted views
   have been captured. Pinhole and fisheye models are compared using held-out
   views, then the selected model is refitted using the accepted set.
5. **Review results.** The GUI shows validation errors and raw/corrected previews.
   Export session JSON, images/data ZIP and a PDF report. Failed quality gates
   request better views rather than claiming a successful calibration.

The physical target still has to be placed in front of the camera. No coefficients
are invented when there are no real target observations. Passed reprojection
checks do not establish camera/ToF extrinsics or guarantee every real-world edge
is straight; compare independent objects near the image borders.

## Persistence and preview

Sessions live under `/var/lib/2pybot/vision-calibration/<session-id>/`, with accepted
JPEGs, corner coordinates, metadata and results. A restart restores an unfinished
session paused; use Resume when the camera/target are ready. New sessions retain
previous sessions for export. Capture is bounded to 60 views.

A passing solution becomes the active intrinsic record. Corrected preview uses
the owner's snapshots, currently about 1 Hz, on a separate worker. The normal
WebRTC stream remains raw; this first implementation does not insert a geometric
warp into the hardware encoder pipeline.

## Depth health and timing

The GUI also shows raw 8×8 depth, useful/near-field/inconsistent counts and diagnostic
guidance. A green zone is usable, amber is inside the supported 50 mm minimum,
and grey is rejected. Near-window/self returns require a physical optical/mounting
check; applying a guessed large range offset does not solve them.

Camera JPEG sidecars carry V4L2 monotonic capture timestamps, publisher/sequence,
size and CRC32. The reader verifies the sidecar against the actual JPEG and its
publication time before trusting the capture stamp. ToF samples use host-read
midpoint timestamps and an explicit uncertainty estimate. A bounded history
provides the nearest camera/depth pair; **this is not hardware synchronization**.
Accepted calibration samples retain that pair and the raw readings for replay.

## Installation

The Debian system OpenCV/ArUco installation on the Cubie is supported. On a new
machine, install `software/radxa/console/requirements-vision.txt` in the console
environment. OpenCV 4.5–4.x, numpy and ReportLab are used; OpenCV 5 is excluded
because its fisheye API changed. Calibration modules load lazily so a missing
optional vision dependency does not take down the main console.

The legacy `2pybot-calibration.service` and `/home/radxa/calibration` app were
archived before removal. Their live ToF SDK dependency was moved to
`/var/lib/2pybot/tof-sdk/vendor`; `40-tof.conf` now points there. Old calibration
results are not imported into the new suite. Camera profiles and saved robot
parameters were retained.

## APIs

- `GET /api/calibration/status`
- `POST /api/calibration/start`: `{ "board": { "kind": "charuco", "columns": 8,
  "rows": 6, "square_mm": 20 }, "goal": 32 }`
- `POST /api/calibration/action/{pause,resume,analyze,cancel}`
- `GET /api/calibration/board.pdf` or `board.svg`
- `GET /api/calibration/preview.jpg?corrected=true|false`
- `GET /api/calibration/session/<id>.json`
- `GET /api/calibration/session/<id>/report.pdf` or `export.zip`
- `GET /api/calibration/active`
- `GET /api/vision/diagnostics`

The next physical milestones are intrinsic captures/independent validation,
ToF optical-quality checks, then camera/base and camera/ToF mounting calibration.
See the [vision roadmap](VISION_ROADMAP.md).
