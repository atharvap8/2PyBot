# Selectable BLDC velocity balancing

This is an experimental actuator-path comparison. Software compilation does not
establish stable balancing/climbing on the hardware; existing body gains have
not been retuned for the motor driver's internal speed-loop response.

## Selection

| `DRVMODE` | Driver mode | Balance arming |
| --- | --- | --- |
| `0` | Velocity | Bench only; blocked |
| `1` | Current/torque | Enabled; retained fallback |
| `2` | Velocity | New compiled default; requires both drivers to confirm velocity mode |

The compiled default is `2` for fresh/missing NVS settings. Existing saved values
and the firmware worker's previous live selection are preserved. To switch a bot
whose old selection is still `1`, stop with local SELECT, then use:

```sh
bot params set DRVMODE 2 --save
bot params get DRVMODE
```

Wait for healthy motor mode/link readback, then arm using local START. Check wheel
direction with the wheels clear before a floor trial. If velocity direction is
wrong, SELECT first; `VSIGN` independently controls velocity-command wire sign
and accepts only `-1` or `+1`. It defaults to `-1` to map the existing signed RPM
convention back to robot-forward wheel motion. Do not change current-mode
`ISGN` or encoder directions to compensate for a velocity-only sign mismatch.

Return to the previous controller while stopped:

```sh
bot params set DRVMODE 1
```

`bot params save` persists the selected mode and other live parameters. Without
saving, a cold boot loads the prior NVS selection. Neither mode selection nor
firmware deployment arms motors. Mode/sign changes are refused while motor
authority or pending arming is active.

Without the CLI, use `POST /api/params/set` with
`{"key":"DRVMODE","value":2}` on the local console API.

## Control path

The normal/stiff/climb body controllers, sensing and position feedback remain.
Their wheel acceleration is integrated into common wheel velocity. In mode `2`,
the result is sent as signed RPM to the drivers' internal velocity loops. The
ESP32 current PI and acceleration-to-current feed-forward are bypassed.
The current-mode virtual-state clamp `CURRENT/HUBKP` is therefore not applied
to velocity balancing. Velocity state is seeded from measured motion on reset.

The velocity protocol accepts **+/-330 RPM**. The integrated command is bounded
by that representable speed and any explicit nonzero `MAXSPD`. Common balance
speed is allocated first; steering uses the remaining RPM range. There is no
`SPDHEAD` or voltage-estimated free-speed derating. A `DRIVEV` above the protocol's
representable velocity is bounded accordingly in mode `2`; `VDRIVE` reports it.

Balancing sends the velocity acceleration byte as `0`, avoiding an additional
configured bench ramp. `HUBACC` continues to control bench-only commands.

**Current limits:** `CURRENT`, `HUBKP`, `HUBKI`, `HUBKFF` and yaw-current gains
belong to the ESP32 current-control path. They do not enforce a hard current
ceiling in velocity mode. The motor driver chooses current and uses its own
limits/protection. Actual current and driver faults remain in `HM` feedback.

## Stop and telemetry

SELECT/E-stop disables the balancer and sends **zero RPM** in velocity mode.
The internal speed loop may actively resist motion at zero speed; this is not
the same as current-mode zero torque. The protocol brake bit is still false.
No automatic brake or position-mode feature is introduced here.

Mode mismatch, stale feedback, motor faults, tilt/rate cutoffs, maintenance and
manual arming checks remain. An old-mode reply immediately fails the mode check,
including the interval directly after changing `DRVMODE`.

`PR,VELBAL,1` identifies this firmware capability. Debug telemetry appends
`drvMode`, `rpmCmdL`, `rpmCmdR`; the RPM fields are actual signed UART targets.
The old `HM.current_cmd_*` fields are zero in velocity mode because no current
setpoint is sent. Use measured current fields when evaluating load response.

## Local workspace recovery

Pre-existing stash-merge conflict alternatives were preserved in
`build/pre-velocity-conflicts.patch`. Conflicts were resolved using the published
4.1.0/no-free-speed-limiter side. Existing Git stashes and the handoff document
were retained.
