# Route autopilot — 3.20.0

The v4 BLDC branch retains this host protocol and recording format. The new
motor firmware applies its configured differential steering limit (default
1.5 rad/s). Re-teach routes against the new radius/track and steering behaviour.

Open Android **Auto → Autopilot · routes / waypoints**, or
`http://10.42.0.1:8080/ui/autonomy.html` while connected to the robot.
Android download: `/ui/2PyBot-v3.20.0-debug.apk`.

## Try a route

1. Connect the gamepad. Camera position, torch and payload controls are independent.
2. Choose **Create 2 m straight route**, or **Teach manually**, drive with the
   gamepad, then **Save taught route**. Teaching also supports rolling the disarmed
   robot on its wheels; saving and previewing do not move it.
3. Place the robot at the route's physical start, facing the taught heading.
4. Press local gamepad **START** and wait for stable balancing.
5. Press **Start replay**. New taught routes replay recorded speed/turn commands
   with their original timing, including reverse and pauses.
6. Keep the page/app visible. Drive-stick input takes over immediately;
   use **Resume explicitly** to hand back. **Pause / hold** stops route requests
   while preserving balancing; gamepad **SELECT** or **E-STOP motors** disables motors.

New recordings capture drive commands and odometry at 20 Hz. Replay has no 0.6 m/s
ceiling, forward-only restriction, host reference ramp or artificial corner/goal
slowdown. Existing waypoint-only routes never recorded timing/speed: re-teach for
original-command replay; their legacy geometric follower uses configured DRIVEV.
Hardware wheel/acceleration limits, balance control and operator stop remain.
Actual motion depends on load/traction and can drift from the taught odometry.

## Current scope

Routes use start-relative encoder odometry without the former 10 m/100-point cap. They are
supervised, forward-only indoor routes on clear floor. There is no SLAM, automatic
relocalisation, rear coverage or cliff detection. Align the physical start manually.

**ToF is advisory only.** Missing, invalid, close or stale depth does not block
Start/Resume, pause replay or cap route speed. Reported depth estimates are only
displayed as advisory metrics; route replay performs no automatic obstacle stop.
The optional manual-drive ToF guard does not gate a granted navigation lease.
Camera/payload position and camera/torch inputs do not grant/revoke drive authority.

The visible page sends a keyed supervision heartbeat; three seconds without it
pauses replay. MCU commands also have an independent 500 ms timeout. Odometry
resets, disarming, firmware maintenance and loss of the native lease
pause/abort the mission. Airborne/recovery diagnostics are advisory at the host;
the MCU retains its own balance/fall handling. Resuming requires an explicit action.

Routes persist under `/var/lib/2pybot/autonomy/routes/`. Control/status endpoints
live under `/api/autonomy/`; native protocol is documented in
[`firmware/BaseLink/PROTOCOL.md`](../firmware/BaseLink/PROTOCOL.md).
