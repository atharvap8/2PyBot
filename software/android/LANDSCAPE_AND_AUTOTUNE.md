# Landscape Layout and Auto-Tuning

Reference: `MainActivity.kt`, `TuneScreen.kt`, `AutoScreen.kt`, and `BotRepository.kt`.

## Landscape behavior

- Navigation changes from a bottom bar to a rail.
- The top app bar is hidden; stop control moves to the rail header.
- Live uses camera and status panes.
- Tune places parameter groups in a 132 dp left column and values on the right.
- Map uses separate trail and statistics areas.
- Auto places routine controls beside the progress log.

Portrait retains the bottom navigation and top stop action. Activity orientation is `fullSensor`, with configuration changes handled by the activity.

## Auto-tune transport

The app posts `AT,<routine>` through `/api/command`. The imported [Radxa server](../radxa/console/console/server.py) already includes `AT,` in its command-prefix whitelist.

Firmware `A,...` progress lines are forwarded by the console as log events. `BotRepository` filters them into the Auto log. Completed tuning values remain in RAM until Save sends `PS` through the console.

| Action | Firmware command | Current behavior |
| :--- | :--- | :--- |
| Wobble | `AT,wobble` | Measure pitch RMS; reduce K3/K4 in bounded steps while balancing |
| Trim | `AT,trim` | Average corrected pitch and adjust TRIM while balancing |
| Radius | `AT,radius` | Start a two-metre radius measurement |
| Done | `AT,done` | Currently intercepted by the firmware busy guard during radius measurement |
| Stall | `AT,stall` | Search raw motor speed from idle; UI asks for confirmation |
| Abort | `AT,stop` | Abort the active routine through its dedicated firmware path |
| Keep result | Parameter save action | Persist firmware settings to NVS |
| Discard | Parameter load action | Reload saved firmware values when present |

Wobble/trim require balancing. Stall/radius use different preconditions. The firmware timeout is 45 seconds.

The app stop control sends `X`, not `AT,stop`; it does not cancel stall tuning in the current firmware. App controls cannot correct this firmware limitation.

## Current parameter coverage

The Tune screen derives groups from received parameter metadata. It can render all **106** current fields, including the eight TERRAIN fields. The hint text remains defined for the original nine groups; TERRAIN uses the same generic row renderer.

Gains and TRIM now share the firmware `P` store used by serial tuning. The imported console still parses only the original 15 debug fields, so the five newer terrain telemetry fields are not yet available to this app.

See [the firmware tuning guide](../../docs/BaseLink/Config_and_Tuning_Guide.md) for limits and [the USB protocol](../../firmware/BaseLink/PROTOCOL.md) for commands.
