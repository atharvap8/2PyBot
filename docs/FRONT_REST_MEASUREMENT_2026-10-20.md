# Front-rest posture measurement — 2026-10-20

Observed at **16:35:39 UTC**, with the user identifying the robot as resting on
its front support, previously described as approximately 45 degrees.

Ten fresh samples over about two seconds:

- Forward/control pitch `pF`: **+62.956° mean**, +62.95° to +62.97°.
- Raw IMU pitch: **−61.416° mean**.
- `ISGN=-1`, `TRIM=1.54042°`; control pitch is `ISGN * (raw_pitch - TRIM)`.
- The observed body lean is approximately **63° from upright**, approximately
  18° beyond the earlier 45° estimate. This measures body posture, not a CAD
  panel's cut angle.
- Controller READY, IDLE/not balancing; both motor current commands 0 A.
- Both motor links/modes healthy, both fault bytes zero, both RPMs zero.
- Installed firmware 4.1.0, source `d8d0c8df1510b7d0b4b8a5ca4cf2758bf8ae1762`,
  build `de28982b8e0ccf0bac7fc3f61a4d4c84668fdbd209b17c84d6523c11676d89b0`.

## Stand-up capability at this posture

The installed program has no `SELFRIGHT` capability or `SR*` parameters. Its
normal balancing entry requires `|pF| < ARMANG=5°`; its normal fall cutoff is
`MAXTILT=55°`. A request to arm normally cannot stand it up from this 63° rest.

The separately supplied `BaseLinkHub/self_righting.h` contains a bounded
self-righting state machine, but it is not integrated into the installed program.
Its supplied `SR_MAX_REST_DEG` is 50° and it explicitly refuses a 63° rest before
applying current. Its traction-limit claims are model predictions, not a physical
stand-up result for this measured posture.

No stand-up motor pulse, arming-limit override or self-righting firmware change
was performed as part of this measurement.
