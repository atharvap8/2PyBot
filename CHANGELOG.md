# Platform revisions

## 3.1.1 — robot-log PDF export

- Setup exports a paginated PDF of all buffered robot-log rows with host,
  revision, timestamp and page numbers through Android's document picker.
- Load recent console logs on connect and preserve decoded multiline content.

## 3.1.0 — robot checkpoint

- Captured a complete 107-parameter dump after firmware confirmed loading NVS.
- Captured settings are compiled defaults, including user-adjusted trim, drive
  speed, steering limit and LED front offset.
- Checkpoints current balancing firmware, camera/ISP profile, host readiness,
  ToF integration and CAD assets before Android feature work.
