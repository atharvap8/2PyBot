# Map-based autonomy

This page covers the plan and the current state for lidar autonomy on the Radxa
console. Everything runs in the console process. The firmware only receives the
navigation lease (`AN`/`AU`) and the safety-bubble caps (`BG`).

The host can stop the robot but can never arm it. Local START and an open
supervising page are required to start. Any stick movement takes over immediately.
Since 5.2.0 a mission keeps driving if the gamepad drops out. Since 5.3.0 a lidar
mission (go-to, roam, explore) also keeps driving if the supervising page is lost
(Wi-Fi out of range), and after a gamepad takeover it takes the drive back once
the sticks have rested for 1.5 s. STOP on the console, SELECT on the gamepad, the
safety bubble and the controller's 0.5 s command timeout still end or hold it.

## Plan

| # | Feature | What it does | Where |
|---|---------|--------------|-------|
| 1 | Map correction | Load a saved map, relocalize, then keep mapping on top of it (*Extend*). Erase a disc back to unknown so it is redrawn. Delete maps. | `slam/engine.py` `extend`, `erase`, `delete_map`; Maps step 6 |
| 2 | Safety bubble | Every scan gives forward/reverse speed caps and left/right turn clearance around the measured body. Sent to the controller as `BG` (all drive sources) and applied again by the host planners. | `slam/bubble.py`, `bubble_guard.py`, firmware `bubble_guard.h` |
| 3 | Area labels | Tap the map and type a name. Labels are stored in the map JSON and usable as destinations. | `engine.add_label`; Maps step 7 |
| 4 | Roaming | Vacuum-style: straight runs, stop at the bubble, turn toward open and less-visited space, back out when boxed in. Detects a stalled wheel. | `missions.RoamPlanner` |
| 5 | Reality view | *Drive* tab: chase camera showing map walls, live lidar posts, bubble, planned path ribbon, goal beacon, floating labels and a HUD. | `ui/slam/drive.js` |
| 6 | Map autonomy | *Go to* a label or a tapped point: A* on the inflated map, pure pursuit on the SLAM pose, re-planning around new lidar obstacles. *Explore & map*: drive to the nearest frontier repeatedly while mapping. | `slam/planner.py`, `missions.GotoPlanner`, `missions.ExplorePlanner` |

Teach-and-replay stays available unchanged (`/ui/autonomy.html`). The new map
missions use the SLAM pose instead of start-relative odometry, so they do not need
the robot to start at a marked spot. They survive odometry resets and correct drift
continuously.

## Safety bubble geometry

The footprint was measured from the C1 lidar centre: right 10 cm, left 17 cm,
front 6 cm and back 7 cm. The lidar is 31 mm right of the centreline on the axle
line, so in the base frame the body is the rectangle x ∈ [−0.07, 0.06] m,
y ∈ [−0.131, 0.139] m. The diagonals (corners) from the lidar centre are:

| Corner | Distance |
|--------|----------|
| Front-left | 0.180 m |
| Front-right | 0.117 m |
| Back-left | 0.184 m |
| Back-right | 0.122 m |

- Returns inside the body plus 2.5 cm are treated as the robot itself. The bubble
  uses every return beyond 3 cm, not the 12 cm SLAM self radius. Until 5.4.0 the
  body box itself was drawn 1 cm larger on every side; it is now the measured body.
- Beams are floor-clipped for the current lean, as in SLAM.
- **Forward and reverse caps:** the corridor is the body width plus 3 cm on each
  side. The cap is the braking speed for (free distance − 6 cm), with a = 0.6 m/s²
  and a reaction time of 0.25 s.
- **Turning:** the body is rotated 0.12–0.60 rad each way. A direction is blocked
  if a corner would come within 2.5 cm of a return.
- **Stale scans:** after 0.4 s without a usable scan (lidar off, large lean) the
  caps are 0. The firmware also caps to 0 after 500 ms without `BG`.
- **Enable switch:** Maps step 8, persisted in `/var/lib/2pybot/bubble.json`,
  default on. It engages only after the lidar has produced scans in the current
  session.
- **Below the scan plane:** the lidar plane is 0.265 m above the floor, so lower
  objects are not seen. Missions also respect the ToF corridor cap when the ToF
  guard is enabled, and roaming backs off when the wheels stall.

## API

| Method | Path | Body |
|--------|------|------|
| POST | `/api/autonomy/mission/start` | `{kind: roam\|goto\|explore, speed_mps?, duration_s?, goal?: {label} \| {x, y, name?}, goals?: [goal, ...]}` → `{mission_id, lease_key}`. `goals` (up to 12) are visited in the order given. |
| GET | `/api/autonomy/mission` | Mission, per-kind blocking reasons, bubble status |
| POST | `/api/autonomy/heartbeat` | Unchanged: required every < 3 s |
| POST | `/api/autonomy/pause`, `/resume`, `/stop` | Unchanged |
| GET/POST | `/api/bubble` | `{enabled}` |
| POST | `/slam/api/maps/extend` | `{name}` |
| POST | `/slam/api/maps/delete` | `{name}` |
| POST | `/slam/api/map/erase` | `{x, y, r}` |
| GET | `/slam/api/labels` | — |
| POST | `/slam/api/labels` | `{name, x, y}` |
| POST | `/slam/api/labels/delete` | `{name}` |

## Not yet verified on hardware

None of this has been driven yet. Before relying on it:

1. Check the bubble state, caps and footprint drawing with the robot balancing in
   place next to a wall.
2. Run *Roam* at 0.15 m/s in an open room with a hand on the gamepad.
3. Load a map, check it localizes, then *Go to* a label.
