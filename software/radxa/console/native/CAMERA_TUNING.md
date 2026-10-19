# IMX708 / Allwinner ISP camera tuning

The output-only preset described below has been superseded by the
[Raspberry Pi-derived sensor profile](IMX708_RPI_PROFILE.md), which installs
IMX708 CCM/white-point/gamma calibration in hardware. Manual controls, IPC,
persistence and exposure-unit fixes remain in use. Default output settings are
now neutral rather than the earlier contrast/saturation compensation.

## Fixes

The CSI publisher owns `/dev/2pybot-imx708` exclusively. The previous Python
`v4l2-ctl` queries opened a second capture handle and received `EBUSY`; the empty
control list incorrectly became “camera exposes no manual exposure control”.
The sensor does support exposure/gain, and the VIN/ISP capture descriptor supports
their control events. The publisher now provides bounded Unix-datagram IPC at
`/dev/shm/2pybot-camera.sock`. Only the capture owner performs ISP writes.

Important units on this board:

- VIN `V4L2_CID_EXPOSURE_ABSOLUTE` is **microseconds**, unlike UVC's 100-us units.
- Sensor exposure is Q4 line count; the 2304×1296 mode has a 13.3607-us line time.
- Exposure is capped to VTS minus 48 lines: approximately **17.209 ms at 56 fps**.
- User gain is Q4: **16 = 1×, 32 = 2×, 64 = 4×, 256 = 16×**.
- The ISP manual gain setting is Q8; the publisher converts Q4 to Q8 on writes.
- Reported exposure/gain readback comes from actual IMX708 registers 0x0202/0x0204;
  the sensor driver's cached V4L2 values do not track ISP updates reliably.

Manual readback was verified with a 6000-us request returning 5986 us and a
2× gain request returning 2× while H.264 publication continued.

## Screenshot-based colour preset

The supplied lit-room screenshot showed lifted/hazy blacks, weak contrast and
colour cast. The current room was dark, so no appearance calibration or
brightness optimization was performed on current frames.

The fixed output transform was replaced with live controls:

| Natural preset | Value |
| :--- | ---: |
| Contrast | 125% |
| Saturation | 150% |
| Output black point | 16 |
| Gamma | 100% (no shadow lift) |
| Exposure | Auto |
| White balance | Neutral-reference auto |

Neutral chroma U=V=128 now stays exactly 128. The previous fixed offsets shifted
neutral U/V. Tone processing has a highlight shoulder, respects limited-range
NV12 (Y 16–235, chroma 16–240), and is shared by video and JPEG. The JPEG converter
explicitly expands limited-range BT.601 to full-range JPEG; H.264 signals
limited-range BT.601. The optical-black ISP offset remains -64; output black-point
adjustment is a separate reversible control, not a guessed sensor calibration.

White balance no longer falls back to averaging furniture/coloured objects when
there are too few bright neutral samples. In darkness or without a neutral
reference it freezes instead of chasing scene colours/noise. Manual red/blue
Bayer gains are also available.

This is a screenshot-based starting preset, not a measured colour-chart profile.
It cannot recover optical flare or clipped scene detail. Lit-room appearance
cannot be visually verified while the room is dark.

## Operation and persistence

The Camera tab exposes exposure, gain, contrast, saturation, output black point,
gamma and white balance while video is running. Controls persist atomically in
`/var/lib/2pybot/camera_tuning.json` and are restored when the publisher starts.
Profiles use CSI-specific units/ranges; the specific profile-save route precedes
the generic profile route. The saved `natural_imx708` profile matches Natural.

Auto-calibrate is an explicit button action. For CSI it uses microseconds and
frame-safe exposure bounds, then analogue gain. It is not run automatically on
startup or against the current dark-room scene.

Validation includes neutral-grey preservation, legal luma/chroma range, monotonic
tone curves, actual AArch64 SIMD/scalar parity, in-place transforms and padded
rows. The native source was built on the Cubie with its installed Cedar/ISP SDK.

Recovery checkpoints:

- Git stash: `bff3ae922b8c4e88bac66bfffebae157fe4e2fc0`.
- Code archive: `/home/radxa/archives/pre-boot-coordination-20261011-035043.tar.gz`.
- Previous publisher binary: `/home/radxa/archives/camera-before-live-tuning-20261011-041429/2pybot_hwstream`.

## Research references

- Linux V4L2 camera-sensor controls and timing:
  https://www.kernel.org/doc/html/latest/userspace-api/media/drivers/camera-sensor.html
- IMX708 upstream exposure/gain registers and mode timings:
  https://github.com/raspberrypi/linux/blob/rpi-6.6.y/drivers/media/i2c/imx708.c
- Allwinner ISP control events, manual exposure units and gain handling:
  https://github.com/Jackey-Huo/libAWIspApi/blob/master/libisp/isp.c
- Allwinner AE types and Q8 gains:
  https://github.com/Jackey-Huo/libAWIspApi/blob/master/libisp/include/isp_3a_ae.h
- NV12 layout and chroma ordering:
  https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/pixfmt-yuv-planar.html

The installed A733 VIN/sensor source and ISP602 ABI were inspected as well;
their units and control IDs, rather than assumptions from a USB webcam, govern
the implementation.
