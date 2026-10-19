# Android cockpit refresh — 3.30.0

## Research and design direction

- [DJI Fly](https://www.dji.com/global/mobile/dji-fly): camera-first composition,
  readable live status and lightweight controls adjacent to the primary view.
- [Android Reply adaptive sample](https://github.com/android/compose-samples/tree/main/Reply):
  at most five compact navigation destinations, rail navigation on larger windows,
  and list/detail panes sized to the available window.
- Automotive cockpit inspiration: restrained charcoal surfaces, a large velocity
  readout, clear status hierarchy and a central robot scene. This is an original
  layout; no company artwork or branding is reused.

## Implemented

- Five primary destinations: Drive, Live, Routes, Map, More. More exposes Tune,
  Camera, Sessions and Setup. Landscape uses a 72 dp rail; its middle section
  scrolls on short windows while STOP remains anchored.
- System-bar/cutout insets are applied once. Primary panels size from available
  content width; tool panes are bounded rather than occupying a fixed percentage
  of arbitrarily wide/short windows.
- Drive: separate velocity/instrument cluster and unobstructed 3D workspace in
  landscape; stacked scene and compact telemetry ribbon in portrait.
- Live: correct 16:9 camera framing, bounded instrument panel and expandable
  secondary controls. Existing video scaling preserves image aspect ratio.
- Routes: direct entry to replay, compact embedded toolbar, independently
  scrolling teaching/status sidebar and a flexible route canvas. Portrait stacks
  the map above the controls. Original replay behavior is retained.
- Camera, Setup and Sessions: independently scrolling columns when space permits.
- Tune: searchable parameters, lazily composed rows and persistent save actions.
- Unified neutral surfaces, softer accent, clearer type hierarchy and consistent
  card geometry. Removed unnecessary bottom spacers reserved by the old shell.

## Verification scope

Build the installable APK with `:app:assembleDebug`. Browser layout inspections
cover the embedded route page at narrow portrait and short landscape sizes.
Native rotation/touch/video behavior needs a connected Android device; browser
captures are not native Android screenshots.

The Android revision is 3.30.0; controller firmware may independently remain
the installed 3.20.0 original-command replay build.
