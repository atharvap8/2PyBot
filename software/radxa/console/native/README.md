# Cubie A7A hardware camera publisher

`2pybot_hwstream` captures MJPEG from the USB camera and uses Allwinner's
**Cedar VE2 H.264 encoder** directly. JPEG decoding and pixel conversion run
on the CPU; H.264 encoding does not use libx264 or the GPU/NPU.

The console selects this backend for MJPEG modes when the executable exists.
The process publishes `rtsp://127.0.0.1:8554/cam` and atomically writes original
JPEG snapshots to `/dev/shm/2pybot_snap.jpg` for calibration.

## Build

The Radxa image needs `libcedarc-dev-2.0.0-arm64` and the compiler/development
packages `build-essential`, `pkg-config`, `libavcodec-dev`, `libavformat-dev`,
`libavutil-dev`, and `libswscale-dev`.

```sh
make -C /home/radxa/2pybot-console/native
sudo systemctl restart 2pybot-console.service
```

Install `99-2pybot-encoder.rules` in `/etc/udev/rules.d/` and ensure the console
user belongs to `video`. The rules grant access to the two Cedar devices and
the system DMA heap independently of desktop login ACLs.

## Buffering and timestamps

- Request four V4L2 capture buffers and drain completed buffers to the latest frame.
- Reject frames older than 200 ms, corrupted frames, and non-increasing timestamps.
- Use monotonic kernel capture time for RTP; wall-clock/NTP changes cannot affect it.
- Hardware Baseline encoding has no B-frame reordering and an IDR every half second.
- Publish directly through the RTSP muxer, without an intermediate encoder FIFO.
- Bound a blocked RTSP write to 500 ms; the console supervisor restarts failed publishers.
- A five-second frame-processing watchdog terminates a stuck publisher so the
  supervisor can restart it; start/stop operations are serialized in the console.
- Convert full-range JPEG to limited-range NV12 explicitly and signal the BT.601
  matrix/range in the H.264 stream.

`GET /api/stream` reports `encoder: cedar_h264`, `hardware: true`, and progress
metrics. `capture_to_publish_ms` measures kernel capture timestamp to completion
of the local RTSP write; it does not include the phone's network/decoder/display.
`drop_frames` counts intentionally discarded stale/queued/invalid frames.
`timestamp_fallbacks` counts camera timestamps that could not be used as monotonic
capture times (normally zero). CPU-decoder errors and hardware failures are
available in `journalctl -u 2pybot-console.service`.
