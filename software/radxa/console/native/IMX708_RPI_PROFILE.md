# Raspberry Pi-derived IMX708 sensor colour profile on ISP602

## Why output contrast/saturation was insufficient

The Raspberry Pi image quality comes from libcamera's sensor-specific ISP tuning,
not the raw kernel sensor driver alone. Our former fallback profile came from
OV13850; its inappropriate colour matrices were disabled, leaving no sensor-to-
display RGB colour correction. The earlier output-only preset could change
contrast and colour strength but could not correct spectral cross-channel errors.

## Installed hardware pipeline

`imx708_profile.h` ports numerical calibration from Raspberry Pi libcamera commit
`9f4754cbc6f9ae2276c9e2f42db9302e64c723b5`:

- IMX708 colour matrices at 2964, 3610, 4640, 5910 and 7590 K.
- The calibrated five-point PiSP/NN-AWB R/G, B/G temperature locus.
- The VC4 IMX708 33-point gamma curve.

The installed A733 ISP602 ABI was checked against headers and exported functions:
colour records are packed **26 bytes** (temperature plus twelve signed 16-bit
values), matrix coefficients use **Q8**, and the gamma configuration is **30736
bytes**, containing five RGB tables with 1024 entries per channel. Curves are
resampled to the ISP's 12-bit output. Quantized CCM rows sum exactly to 256, so
neutral RGB stays neutral.

The sensor matrix is applied in the **linear-RGB ISP stage before gamma/YUV**.
All three hardware matrix slots receive the selected/interpolated sensor matrix
to prevent the fallback ISP's unrelated colour-temperature estimate choosing an
OV13850 matrix. Temperature changes update every slot, then ISP configuration is
updated and read back. Raw demosaic, CCM and gamma are enabled. Uncalibrated
foreign LSC/MSC, colour-enhancement tables and HDR/local tone-mapping remain off.

The capture driver ignored the requested partial-range format and returned JPEG
colour space. The installed ISP's RGB2YUV table confirmed that space index 0 uses
full-range coefficients. The publisher now converts full-range NV12 to limited
range mathematically before H.264, rather than clipping levels or incorrectly
labelling the data. White-balance sampling uses the corresponding full-range
BT.601 coefficients. JPEG conversion then expands the corrected limited-range
pixels once. Neutral chroma remains 128 in both conversions.

## White balance and controls

The `rpi_imx708` preset uses a **5000 K starting white point** and calibrated Bayer
gains (approximately R=516, G=256, B=453 in Q8). Auto white balance adapts using
neutral scene candidates, constrained to the IMX708 temperature locus and slewed
by at most 125 K per update. It freezes in high-gain darkness or without a usable
neutral reference. This is a simplified adaptation, not a port of Raspberry Pi's
complete Bayesian/raw-statistics AWB algorithm.

The Kelvin slider controls both Bayer white balance and the hardware CCM. Manual
red/blue controls remain available. Auto-mode restoration ignores stale manual
gains; they previously overrode the sensor profile during startup.

Output controls default to **contrast 100%, saturation 100%, black point 0,
gamma 100%**. There is no artificial saturation/black-point compensation on top
of the sensor correction. User-selected adjustments still work and persist.

## Verified

- Actual ISP readback confirms calibrated matrices, gamma values and enabled
  demosaic/CCM/gamma modules.
- Switching 3610 K to 5910 K changes the hardware matrix.
- All matrix rows preserve neutral RGB after Q8 rounding.
- Gamma endpoints are 0/4095 and the 1024-entry curve is monotonic.
- Sensor exposure/gain controls and streaming remain available.

The current room was dark. No current-frame appearance calibration was used and
no lit-scene visual improvement is claimed as measured. The stock Raspberry Pi
ALSC/lens-shading mesh was not copied: this module has a different fixed-focus
M12 lens. Full Raspberry Pi equivalence, lens shading, flare and colour-chart
accuracy require lit-scene optical validation; matching the sensor CCM/gamma is
a substantial pipeline correction, not a complete replacement ISP.

References:

- https://github.com/raspberrypi/libcamera/blob/9f4754cbc6f9ae2276c9e2f42db9302e64c723b5/src/ipa/rpi/vc4/data/imx708.json
- https://github.com/raspberrypi/libcamera/blob/main/src/ipa/rpi/pisp/data/imx708.json
- Installed BSP `isp_comm.h` / `isp_tuning_priv.h`, ISP602 tuning getters/setters,
  and A733 `sunxi_camera_v2.h` colour-space extensions.

Recovery binary and prior settings:
`/home/radxa/archives/camera-before-rpi-profile-20261011-044100/`.
