# Platform revisions

For the main milestones and estimated dates for future work, see the
[Revision Table](docs/REVISION_HISTORY.md).

## 5.4.0: routes with several stops, autopilot on the gamepad, a flashlight that looks after itself

Firmware changes compile but have not been driven yet.

- **Routes with several stops.** In Maps, add places from the list or tap the map
  for each stop, reorder them with the arrows, and Start drives them in that
  order. One stop is a plain go-to. The map shows the numbered stops and the order;
  the mission shows "Stop 2 of 4". A stop that cannot be reached ends the route and
  names it. API: `goals: [...]` (up to 12) on `/api/autonomy/mission/start`.
- **Autopilot on the gamepad.** A = go (resume a paused mission, or repeat the last
  one), B = pause, X = end, Y = roam. RT is an analog precision trigger: the
  further it is pulled, the slower and finer the left stick, down to a quarter.
  The gestures moved to LT held + Y / B / X / A. D-pad, sticks, LB, RB, START and
  SELECT are unchanged. The buttons only report a key to the host (`PK,GO` ...);
  they are not a takeover and they never arm.
- **The flashlight comes on by itself in the dark.** While the robot is balancing:
  a fully dark picture (mean luma under 25) for 1.5 s switches it on at full
  brightness, with the head centred and the beam at its widest, and it is then
  left exactly like that. When the darkness is gone it switches off and the
  flashlight is the gamepad's again. A first version also regulated brightness
  and aimed the beam; on the robot that was restless, so it was removed. Use the
  flashlight from the gamepad and the bot takes what you set and waits 1.5 s after
  your last touch. Parked or powered down, the light goes off. The camera's
  automatic re-exposure steps back while darkness is the light's job. Switch in
  the Camera app; firmware command `TL,<on>,<percent>` (same 90 % cap).
- **Safety bubble 1 cm smaller on every side.** The body box is now the measured
  body (it was drawn 1 cm larger). What counts as the robot's own body in the scan
  has not moved.
- **A gamepad that connects but does nothing.** The robot's log shows the cause is
  in the controller's Bluetooth stack: after some reconnects it delivers no input
  ("connected", then not one button), and every later attempt is refused with
  `Failed to set device information client: 0xc` until the controller restarts.
  That stack is a prebuilt library, so this is contained rather than cured: the
  firmware now counts the input of each connection and drops a link that delivers
  nothing (`[PAD] DEAD LINK`), and when the stack is provably stuck and the robot
  is idle, the doctor restarts the controller (`BT,RESTART`, about 10 s, pairings
  kept). While the robot is balancing it is left alone: press STOP and it recovers.
- A re-exposure that could not run (camera control channel not ready) now says so
  and is tried again after 30 s, instead of reading as "re-exposed" and resting
  five minutes.

## 5.3.0: gamepad reconnect no longer drops the robot, missions survive Wi-Fi loss, no more room-hopping on the map

The firmware changes compile but have not been driven yet. The causes below were
read from the robot's own log of the 5.2.0 test drive.

- **Connecting the gamepad no longer switches the motors off.** While a gamepad
  reconnects, the balance loop runs late in a burst: 20 ms and 22 ms about 50 ms
  apart, roughly 2 s before the pad reports connected (logged twice on 5.2.0, both
  times followed by "CONTROL LOOP OVERRUN"). 5.2.0 rode through one late loop but
  stopped for a second one within a second. Late time is now budgeted: each late
  loop banks its length, the bank drains 100 ms per second, and the motors stop
  only for one loop over 100 ms or more than 250 ms banked. The stall guard is
  unchanged.
- **Losing Wi-Fi no longer stops a lidar mission.** Go-to, roam and explore carry on
  when the supervising page stops answering (the log shows two "supervisor
  disconnected" pauses on one short route). They still stop for STOP, SELECT, the
  safety bubble and the controller's command timeout. A recorded replay drives
  blind, so it still pauses without its supervisor.
- **After you take over with the gamepad, the autopilot takes the drive back.** A
  takeover no longer pauses a lidar mission. It waits until the sticks have rested
  for 1.5 s, asks the controller for the drive again and carries on, replanning if
  you moved it off the route. The controller reports how long the pad has been idle
  (new last field of `NR`); SELECT still ends the mission.
- **"Lost contact with the controller" root cause fixed.** The doctor ran four
  `systemctl`/`ip` commands on the console's event loop every 5 s. Starting a
  process from the 370 MB console froze everything for 0.3 to 0.5 s each time
  (38 "Console was busy" lines in about three minutes of missions), which is what starved
  the serial reader. The checks now run off the event loop, start through
  `posix_spawn` (no copy of the console), and drop to every 30 s while balancing.
- **The map position no longer hops between rooms.** Two faults did it. (1) A
  confident scan match that the pose filter refused was counted as "lost", and
  after a second of that the whole map was searched. It now loosens the filter and
  takes the matcher's word after three scans. (2) The whole-map search moved the
  robot to its best score at once, and rooms look alike to a lidar. It now looks
  within 0.6 m first, and a far-away place must clearly beat what is nearby and
  win a second search, at the spot odometry says it should then be, before the
  robot is moved. Until then the pose carries on by odometry. Find me and loading a
  map still move at once.
- **Drive view without the glitches.** The robot sends its live pose 20 times a
  second: the last matched pose carried forward by the odometry measured since.
  The browser used to combine a 5 Hz map pose with newer odometry itself, which
  jittered in turns and snapped on corrections. The view now cuts only when the
  robot says it moved the pose in one step.
- **The 3D robot is the full CAD assembly.** `scripts/generate_viz_glb.py` turns the
  Onshape export (`models/2pybot.glb`: 45 MB, 686,000 triangles in 17,000 pieces)
  into `ui/viz/robot2.json` + `robot2.bin` (1.4 MB, 111,000 triangles): rotated into
  the robot frame, one welded mesh per part and colour, the 1 mm capacitors and the
  three decorative wires left out, screws and LEDs as hulls. The view now shows the
  real shell with its vents and logo, the hub motors and their mounts, the lidar
  where it really sits (31 mm right of centre), the LED ring, decks, battery and
  boards. In the system view the controller, IMU, Radxa and camera light up as the
  real parts in their true places; the depth sensor, compass and payload are not in
  the CAD and keep stand-ins. The printed-parts model stays as the fallback. The
  45 MB export itself is not committed.
- **Unload a map.** Maps has an Unload button (`POST /slam/api/maps/unload`): back
  to idle with an empty map, saved files untouched. Loading one map straight
  after another now refreshes the 2D picture and the 3D walls (both were keyed on a
  version number that was always 1).

## 5.2.0: 3D console, autopilot that goes round things, missions without the gamepad

Firmware changes in this revision compile but have not been driven on the robot
yet. Test them with a hand on the robot before trusting them: see the last two
points of "Autopilot and controller".

### Autopilot and controller

- **Goes round obstacles instead of waiting at them.** A go-to mission checks the
  next 1.6 m of its route against the live lidar scan four times a second and
  plans a new route as soon as something sits on it, while still driving. Routes
  are planned on the map plus what the lidar and the depth sensor see now. If the
  robot is stopped by something the lidar cannot see (too low), it marks the spot,
  reverses 0.2 m and plans round it. If there is no way round it waits clear of
  the obstacle, keeps looking, carries on when the way opens, and ends the mission
  only after 40 s stuck in one place. The old behaviour was to drive up to the
  obstacle, wait 2 s, and give up after six tries.
- **"Lost contact with the controller" no longer ends a drive for a blip.** That
  pause fired when the console had not processed an odometry line for 0.3 s. The
  console reads the serial port on the same event loop as everything else, so a
  busy moment looked like a lost controller. A mission now holds still and rides
  out gaps of up to 2 s (the controller itself stops 0.5 s after the last
  command, which is the real safety net), renews a drive lease the controller
  timed out (at most three times per mission), and logs how long each gap was. A
  gamepad takeover or STOP is never renewed.
- **Missions do not need the gamepad.** The console no longer asks for a connected
  gamepad to start, and the firmware no longer revokes the drive lease when the
  gamepad drops out (before, that paused the mission mid-drive). Arming is still
  local START, moving a stick still takes over at once, and STOP, the supervising
  Maps page and the 0.5 s command timeout still end a drive.
- **One late control loop no longer switches the motors off.** After a gamepad
  dropped out, the balance loop once ran late and the firmware cut the motors for
  it ("CONTROL LOOP OVERRUN"), which is a certain fall. An isolated late loop of up
  to 100 ms is now logged (`[MAIN] Late loop N ms ridden through`) and ridden
  through. A longer one, or a second within a second, still stops the motors, and
  the stall guard still zeroes them after 150 ms. Why the loop ran late is not yet
  known; the new log line and the `loopNms` field in the stop reason are there to
  find out.
- Pause reasons are told apart: "gamepad takeover" and "native lease lost" were one
  message that always blamed the gamepad.

### Camera

- **Re-exposes by itself.** When the picture stays too dark or too bright (mean
  luma below 55 or above 185 for 15 s) the camera runs the same exposure walk as
  the Auto-calibrate button, then rests for a minute (five when the sensor has run
  out of range, so a dark room does not make it hunt). Switch in the Camera app;
  on by default. Each action is written to the console log.

### Console

- **System view on the robot itself.** The System button turns the 3D robot to face
  you and puts every part around it as a block with its live reading, wired to
  where the part sits. Hover (or tap) a block and its wire and its part light up
  while the rest fades. The full diagram and logs are one link away.
- **Live labels on the parked robot.** Camera, depth sensor, controller, lidar,
  Radxa and both motors show their state and reading (frame rate, nearest object,
  loop time, scan rate, CPU temperature and load, motor temperatures), from the
  same health data as the system map. Left and right are the robot's own.
- **Smooth driving view.** The robot is drawn from wheel odometry at telemetry rate
  and only corrected by the SLAM pose, instead of stepping five times a second.
  Map walls are clean outlines traced on the robot (`/slam/api/scene_vector`,
  about 13 kB for a house instead of 317 kB of cells) and fade in when the map
  changes. A live obstacle has to be seen in three scans before it is drawn and
  lingers for a moment after, so nothing flickers. The speed readout is steadied.
- **View follows the robot's state.** Disarming always returns to the parked view
  and balancing always switches to the driving view, even if a view was held.
  The wheels do not turn in the parked view. View changes are a little slower and
  travel less; the app pane fades in instead of sliding across the screen.
- **No log lines over the view.** Controller log lines go to the Console log only.
  The result of something you did shows in the status pill for a few seconds. A
  paused mission is announced for a few seconds and then stays on its card.
- **Maps.** Inside the console the map sits on top and the settings get the room
  below in two columns. The 3D/Map switch floats over the map with no bar of its
  own. The Radar view is gone.
- **Phones.** The dock fits a narrow screen, the parked robot is framed to fit, and
  the system view stacks its blocks under the robot.

### 3D console (new in this revision)

- **New console home.** The console opens on a live 3D view of the robot with an
  app pane beside it and a dock below, replacing the tab rail. The view chooses
  its own mode from live state: a studio shot when disarmed, a chase camera over
  the map while balancing or on a mission, and a top-down view with proximity arcs
  when it is slow and close to something. Mode changes are one continuous camera
  move, not a cut.
- **The robot is the real CAD.** `ui/viz/robot.json` is generated from the printed
  STLs by `scripts/generate_viz_model.py` as a true rotation. The older driving and
  world assets negate one axis only, which mirrors the robot; they are untouched.
  Wheels, tyre tread, the lidar and the LED ring are parametric.
- **Live layers.** Map walls and floor, the planned route and goal, live lidar
  returns the map does not explain, depth-sensor obstacles and drop-offs, and
  proximity arcs computed from the scan.
- **Apps in the pane.** Drive (map, camera, mission), Maps, Camera, ML, Controls
  (telemetry, calibration, log and every parameter group), Terminal and Deploy. Existing pages are hosted with their own header hidden.
- **Safety unchanged.** The new page can stop things (STOP, pause or end a mission,
  power down) but never starts motion; missions still start from Maps, which keeps
  its own supervision lease.
- **Runs in the browser only.** The robot serves static files; three.js r160 is
  vendored, nothing is fetched from the internet. Quality scales down on slow
  devices. `/?demo` runs the console on synthetic data, and
  `scripts/preview_console.py` serves it on a laptop without the robot.
- **Fallback.** The previous console is kept at `/ui/classic.html`.

## 5.1.0: nicer maps robot, app wordmark and splash, detector acceleration, doctor

- **Maps 3D robot** is a proper little model now: round wheels (real cylinders),
  a body, a camera head with a lens and a lidar puck on top, placed by its pose,
  instead of the stack of axis-aligned cubes.
- **Maps panel** tidied: even button grids (no more ragged rows), consistent
  section headers with an accent tick and divider, and a shorter "Export STL".
- **App branding**: the top-left dot-and-ring is replaced by the 2PyBot wordmark,
  and a launch screen shows the wordmark, the real front photo of the robot and a
  loading bar on the black and orange theme.
- **On-board detector acceleration.** Detection now selects the fastest ONNX
  Runtime execution provider that is available (NPU first, then other
  accelerators, then XNNPACK, then CPU) and always falls back to CPU. The
  provider in use is reported in status and on the camera overlay. See
  docs/NPU_ACCELERATION.md for the Radxa A733 VeriSilicon NPU plan.
- **Doctor self-healer** merged: a rule engine that ranks likely faults and
  auto-applies safe, idempotent recoveries, with `/api/doctor` and
  `2pybot doctor`.

## 5.0.0: SLAM console refresh, in-browser terminal, visual re-pair

- **Maps page** rebuilt as a single clean panel (Mapping, Saved maps, Edit, Autopilot,
  Status). The guided check-lidar / check-wheels steps, tips, glossary and advanced
  panels are gone. The 3D view is the default and shows a simple assembled model of
  the robot instead of two cubes. Drive view removed.
- **Maps download, import and 3D export.** Download a map as a portable zip, import a
  map zip, and export the map as a binary STL for viewing or printing.
- **In-browser terminal.** A Terminal page runs the full 2pybot TUI over a PTY
  websocket with a vendored xterm.js.
- **Deploy page** reworked from a read-only dead end into /deploy/ with live status, a
  working firmware reflash control and a copyable operator deploy command.
- **Console navigation** is consistent across Console, System, Maps, Deploy, Terminal
  and App; the dead-end Autopilot page was dropped.
- **System map**: the lidar block no longer sits under the inspector; the centre image
  is the background-removed front photo, tilt corrected.
- **Visual re-pair (gamepad detection).** Showing the gamepad to the camera for a few
  seconds, while disarmed and with no pad connected, clears the Bluetooth pairings and
  restarts into pairing mode. Uses the on-board detector; adds onnxruntime.
- **Sleep state LEDs** breathe brighter: the status LED and the front ring LED use
  higher minimum and maximum brightness while powered down.
- Manual sources moved to docs/manual/src/; the built PDFs stay in docs/manual/.

## 4.7.8: Gamepad re-pair actually clears the pairing

- **Re-pair fixed.** 4.7.7 tried to create a new NVS namespace (`2pb_bt`) to hold a
  restart marker, and the create call was refused on the robot even with NVS space
  free, so `BT,FORGET` reported `could not store the re-pair request` and did nothing.
  Re-pair now erases the two existing namespaces that hold pairing state, `BTstack`
  (BR/EDR link keys, BLE bonds, BLE root keys) and `bp32` (Bluepad32 settings),
  directly, then restarts so the stale in-RAM Bluetooth state is dropped. No new
  namespace is created. Parameters and calibration are untouched.

## 4.7.7: Gamepad re-pair that works; power-down mode

- **Gamepad re-pair fixed.** In 4.7.6 `BT,FORGET` deleted the pairing keys while
  Bluetooth was running. The controller log then showed
  `SM_EVENT_PAIRING_COMPLETE: Invalid device` and
  `uni_bt_le_on_encryption_change: Device not found`, and the pad never paired again;
  only a full flash erase helped. Re-pair now does the Bluetooth part of that erase
  properly: the controller stores a marker, restarts, and before Bluetooth starts it
  erases the NVS namespaces `BTstack` (link keys, BLE bonds) and `bp32` (Bluepad32
  settings). Parameters and calibration are kept. Capability `PR,PADREPAIR,2`.
- **Pairing button is HOME + A** (EVOFOX One S Android mode, reported as `Android`), not
  HOME + B. Fixed in the firmware messages, docs, CLI, TUI, API and system map.
- **Power-down mode** to stop the robot heating while unused. *Power down* in the
  console header, `2pybot power sleep|wake`, or TUI Robot page; `POST /api/power`.
  - Host: camera pipeline stopped, lidar scanning stopped and its motor spun down,
    depth sensor ranging stopped, object detection and autonomy off.
  - Controller (`SL,1`/`SL,0`, capability `PR,SLEEP,1`): arming refused, LED ring
    reduced to one faint breathing front LED, slow dim status LED, camera servos
    released, torch off, gamepad disconnected and new connections refused.
  - The Radxa keeps the console and Wi-Fi running, because that is how the robot is
    woken; Linux suspend would take Wi-Fi down and the board has no wake-on-LAN.
  - Not persisted: a console restart, deployment or reboot comes back awake.
  - The system map shows powered-down blocks as ASLEEP instead of errors.
- `NR` last field (always 0 before) now reports the controller power-down state.

## 4.7.6 — Stops are short again; gamepad re-pair

- **Stops after a stick release were too long.** The 4.7.5 DRVDEC ramp let the drive
  target fall at only 1.5 m/s², so the robot kept going about 0.35 m from 1 m/s and
  appeared to shoot ahead or back before stopping.
  - DRVDEC is off again (0).
  - The fall it addressed is handled differently. New VERRMAX (0.7 m/s) clamps the
    speed error in drive mode: braking starts at once and stays strong, but the
    velocity term can no longer take all of the acceleration and motor current, so
    the balance terms keep authority.
- **Bubble** back to the measured braking model (BUBDEC 0.5 m/s², BUBLAT 0.30 s). The
  stop margin stays at 8 cm, which covers the 3 cm overshoot seen in the recording.
- **Migration.** New firmware applies these defaults once, replacing values saved by
  4.7.5 (NVS marker `_brake2`; capability `PR,BRAKEPROFILE,2`). The update worker
  no longer replays the old values after a flash.
- **Gamepad re-pair.** New `BT,FORGET` command (disarmed only) clears all bonded pads
  and accepts a new one. Available as `2pybot gamepad repair` and as *Re-pair gamepad*
  on the TUI Robot page. Capability `PR,PADREPAIR,1`.

## 4.7.5 — Fixes from the bubble/vibration recording

- **Fall when releasing the stick in fast reverse.**
  - Recorded: −1.17 m/s, the speed target jumped to 0 in one step, the acceleration
    command and the 2.7 A current limit saturated for 0.6 s, and the robot tipped backwards.
  - New DRVDEC (1.5 m/s²) limits how fast the drive target may fall toward zero.
    Speeding up stays immediate, and the bubble cap still applies at once.
- **22 Hz vibration, worst in climb mode.** Climb mode showed 6× the normal acceleration
  noise and hit the limit 17 % of the time. The speed fed to the controller came from
  differencing averaged position every loop; reply jitter and missed replies made
  noise that grew with speed (0.02 → 0.07 m/s rms).
  - New VELSRC=1 uses each motor's sample-timed velocity instead.
  - VELSRC=0 restores the old method.
- **Bubble stopped too close.** It stopped 0.6–2.4 cm short of obstacles instead of
  5 cm, and twice overshot slightly. The model is now more conservative:
  BUBDEC 0.35 m/s², BUBLAT 0.40 s, BUBMRG 0.08 m.

## Unreleased — 2pybot control centre

- `2pybot` opens a full-screen terminal control centre (curses, Python 3.9 standard
  library). Pages: dashboard, robot actions, parameters, logs, services, updates,
  network, system and help. `X` stops the motors from any page.
- `2pybot <command>` / `bot <command>` print readable coloured tables instead of
  raw JSON (`--json` keeps the raw data). New commands `services` and
  `logs [-f]` (controller log by default).
- `doctor` prints a summary and always saves the full report to a file.

## 4.7.4 — Start-up loader, new fall pattern

- **Start-up loader:** the boot rainbow is gone. From power-on, a white comet spins
  clockwise while a dim cyan bar fills from the front, until the Radxa reports ready
  (`BR,1`) and the controller has calibrated. A green fill from the front then
  confirms ready. The loader returns whenever the Radxa drops out (`BR,0`). It
  replaces error code 5.
- **Fall:** one short red flash, then a slow red glow on the side that hit the
  ground. The 8-LED code with the white marker is gone.
- **Error codes:** no white front marker. The n red LEDs (counted clockwise from
  the front) flash on their own.

## 4.7.3 — Cleaner ring motion

- While the comets run, every other LED is off: no base glow, no centre marker.
- Climb mode: a single violet LED hops one LED at a time toward the travel
  direction, two hoppers into the front for forward and into the back for reverse,
  one across the front half for turning. Faster with speed; all other LEDs off.
  Stopped in climb: the front LED alone, slow pulse.

## 4.7.2 — Motion-based LED ring

- **Colour = state:**
  - red = errors only (n red LEDs = code n; whole ring = fatal);
  - violet + white heartbeat = gamepad connected, waiting for START;
  - green = manual, blue = autopilot;
  - violet marquee = climb mode;
  - magenta = safety bubble limiting;
  - amber blink = pushed.
- **Motion = direction, faster with the stick:**
  - forward: comets from both sides run into the front LED;
  - reverse: comets run into the rear LED;
  - turning: a comet sweeps the front half (left to right for a right turn).
- **Climb:** every other LED lit, marching toward the travel direction.
- **Holding:** breathing ring, a blinking front "eye", and a glow toward the tilt.
- **Bubble diagnostics:** returns near the body are reported (`near_diag`) even while
  scans are gated out, to find self-detection.

## 4.7.1 — Settings finally persist

- Every boot logged `PL!,no NVS, using defaults` and every save failed. One NVS
  entry per parameter (~150 blobs) filled the 20 KB partition together with the
  Bluetooth bonds.
- All parameters are now saved as one blob of (key hash, value) records. A save
  clears the old per-key entries first. Legacy per-key data is still read once.
- `PN` reports NVS usage per namespace. `PX,<ns>` erases an obsolete namespace;
  it refuses Bluetooth, PHY and Wi-Fi namespaces.
- Ring orientation confirmed by the owner: front LED 9, numbered clockwise.
  This is now the compiled default.

## 4.7.0 — Bubble v2, readable LED ring, servo and wheel-speed fixes

- **Bubble v2.**
  - The Radxa now sends free distances, `BG,en,front,back,flags,age_ms`, on every
    scan (~20 Hz) instead of precomputed speed caps.
  - The controller computes the braking limit every cycle and subtracts its own
    wheel travel since the scan, so a fast approach is limited by the distance
    actually left.
  - Tunable: BUBDEC (0.5 m/s²), BUBLAT (0.30 s), BUBMRG (0.05 m), BUBSTALE (300 ms).
  - Footprint +1 cm on every side. The corridor follows the current arc while turning.
  - Isolated single returns are ignored. Commanded turns into an obstacle are blocked.
  - New `BB` telemetry shows the controller-side clearance and caps.
- **LED ring.** Each state has its own colour and motion:
  - waiting to arm: white front LED, blue breathing;
  - holding: steady green with a bright front;
  - driving: green arc; autopilot: blue arc;
  - pushed (external disturbance): amber fast blink;
  - bubble limiting: red arc front or back;
  - self-uprighting: orange chase.
  - **Error codes:** the number of red LEDs that flash. 1 motor link, 2 motor fault,
    3 motor mode, 4 IMU, 5 waiting for Radxa, 6 maintenance, 7 no gamepad,
    8 fell, 9 self-uprighting aborted.
  - Fatal: the whole ring flashes red continuously.
  - `LT,1` and `LT,2` orientation tests; the new LEDDIR parameter sets the ring direction.
- **Servo.** Both servo lines are driven low from the first line of `setup()`. They
  used to float for seconds until the payload started. The camera stick now has to
  read centred after the gamepad connects before it can move the servos.
- **Wheel speed.** Measured against when the motor was asked (its sample time)
  rather than when the reply arrived. The arrival jitter, scaled by wheel speed,
  made velocity noisy at high speed.
- **Diagnostics.** `HF,<s>` starts a 100 Hz HV wheel/current stream.
  `POST /api/debug/record` records raw controller lines. Boot-ROM noise is no
  longer written to the controller log.

## 4.6.1 — No more false motor faults from corrupt RS485 frames

- The "LEFT fault 0xD1 OVERHEAT" trip was a corrupt reply, not a hot motor. 0xD1
  sets undefined bits 0x80/0x40, it cleared on the next reply, and the left link
  had 2094 timeouts and 14 CRC errors. A misaligned 10-byte window passes the 8-bit
  CRC about 1 time in 256.
- Replies that pass the CRC but are impossible (fault bits outside 0x1F, |rpm| > 900)
  are now rejected and resynchronised.
- A fault disarms only after 3 consecutive replies agree (~7.5 ms). A mode change
  needs 2 consecutive replies (~2.5 ms).
- Ignored transient faults are logged, rate-limited. Confirmed faults are logged with
  temperature, rpm, current and the raw frame.
- Console: after a controller USB drop, the serial port is found again under
  `/dev/2pybot`, by-id or any `ttyUSB*`, and the retry message is logged once.

## 4.6.0 — Lidar autonomy: bubble, labels, roaming, map navigation

- Safety bubble from the measured body footprint (R10/L17/F6/B7 cm from the lidar,
  diagonals triangulated). Forward/reverse caps and turn clearance on every scan.
  New firmware `BG` command caps all drive sources, including the navigation lease;
  fails closed. Switch on the Maps page (default on).
- Map correction: *Extend* a saved map (relocalize, then keep mapping on top), an
  eraser tool, and map deletion.
- Area labels: tap the map and name a spot; stored with the map.
- Missions through the navigation lease: vacuum-style *Roam*, *Go to* a label or a
  tapped point (A* on the map, re-planning around new obstacles) and *Explore & map*
  (frontier exploration).
- *Drive* view: chase-camera reality view with walls, live lidar, bubble, path and labels.
- See [docs/AUTONOMY.md](docs/AUTONOMY.md). Not yet driven on hardware.

## 4.5.0 — Safety fixes, fusion SLAM merged to main

Merges all work since 4.1.1: self-uprighting, lidar SLAM with
pose fusion, ToF map layer, single web port, tilted map view, the redesigned Android
app (portrait and landscape) and the user manual and technical Q&A PDFs.

Safety fixes from the incident review (hang, one wheel at full speed, link loss, falls):

- The M0601 drivers keep their last current setpoint while no frames arrive. The
  controller now sends zero-current frames from the first milliseconds after any
  reset (power-on, brownout, panic, watchdog) and throughout the host boot wait.
- Independent loop stall guard (esp_timer task): zero both motors after 150 ms of loop
  stall, restart the ESP32 after 800 ms. Blocking maintenance commands, allowed only
  with motors off, suppress the restart for up to 5 s.
- Reset reason printed at every boot (`[BOOT] Reset reason: ...`).
- Controller log persisted on the Radxa (`/var/lib/2pybot/logs/controller.log`,
  rotating), including telemetry LOST/restored edges, so incidents survive reboots.

## 4.4.0 — Sensing fusion and single web port

- Lidar at 20 Hz (C1M1R2, 1200 rpm); per-beam floor clipping from the measured 0.265 m
  scan plane; scans de-skewed to the mid-sweep pose; whole scans dropped only above 12° / 60°/s.
- EKF pose fusion: wheel + gyro prediction, gated lidar scan-match correction with a
  score-surface covariance, wheel-slip detection.
- VL53L5CX 3D layer at 0.21 m: low obstacles and drop-offs in the map; mount tilt is
  estimated from floor returns. Worker retries the first transfer and backs off when absent.
- Both camera servos detach after their own last command (SVIDLE, default 2 s).
- One web port (8080): console, /sysmap/ and /slam/ linked, one orange system-map theme.

## Unreleased — Standalone lidar mapping (no ROS)

- RPLIDAR C1 driver on the 40-pin UART4 (`console/lidar_c1.py`).
- Numpy 2D SLAM in the console: wheel + gyro odometry (auto-detected gyro axis),
  pitch-aware scan gating, occupancy grid, scan matching, save/load, global
  relocalization (`console/slam/`).
- "2PyBot Maps" beginner page on port 6970 and `/slam/`, with a guided
  five-step wizard, live map/radar, glossary and a no-hardware demo.
- Firmware `O` record carries sensor-frame gyro X/Y/Z (append-compatible).
- `SerialLink.add_line_listener()` gives read-only access to raw `O`/`D` lines.
- Integration fixes: lidar/SLAM threads start in the console startup hook (the
  deployer's import check no longer opens `/dev/ttyAS4`) and stop the C1 on
  shutdown; `/slam/api/estop` passes the firmware-maintenance gate like the
  other E-STOP routes; map export/save copy the grid under the lock and encode
  outside it so the ESP32 serial reader never waits on PNG/disk work; odometry
  and attitude histories are snapshotted before iteration.
- Relocalization searches an even 0.25 m lattice of free cells (the old
  sampling left ~1.5 m gaps) and re-searches automatically after ~2 s lost.
  Candidate scoring runs in 2048-pose batches so memory no longer grows with
  map area, and "Find me" and the automatic search are serialized.
- Firmware: Bluetooth TX power raised to BR/EDR +6..+9 dBm and BLE +9 dBm
  (ESP32 maximum; defaults were 0..+3 / +3 dBm), logged as `[PAD] BT TX power`.

## 4.3.0 — Experimental front-bumper self-uprighting

- Branch from owner-selected stable BLDC `ed91341` (`stable-bldc`). Preserve the
  separate lifted-wheel work in a stash; it is not part of this experiment.
- Local gamepad START at stationary +60 +/-10 degrees fires one bounded
  current-mode recovery. SELECT/E-STOP aborts; upright START still arms balance.
- Use slew-limited attitude current PD, then seed the stable balancer from actual
  wheel velocity near upright. Keep normal balancing fall limits separate.
- Add SELFRIGHT tuning, SR phase/reason telemetry and explicit app mode labels.

## 4.2.0 — Fixed input values and verified saves

- Enforce raw TRIM=-1.2603 degrees and MAXSTR=4000 encoder counts/s in firmware,
  including NVS loading, recomputation, legacy tuning and update snapshot replay.
- Report NVS success only after successful writes and storage readback. The host
  waits for MCU acknowledgements; disconnected/busy/rejected saves return errors.
- Preserve unsaved state across parameter dumps and serialize writes before save.
  The app waits for verified persistence and warns when controller firmware is old.
- Remove stored yaw integral during turn braking/heading hold, and clear it on
  commanded turn reversal, avoiding an old opposing torque on direction changes.
- Retain two-second orientation hold, persistent video, current-only 18 V control,
  direct steering and the owner's existing additions. Tests are opt-in only.

## 4.1.1 — Direct steering and persistent video

- Set the confirmed raw neutral TRIM to -1.4 degrees with a targeted input-profile
  migration, retaining the current-mode 18 V / 2.6 kg setup and existing additions.
- Remove the predictive fall-prevention governor from production firmware. Keep
  tilt/rate/fault/current stops and balance-priority torque allocation.
- Default steering to 3 rad/s (about 2915 differential encoder counts/s), direct
  command/current response, linear steer expo and full steering in LOW/HIGH.
  Remove drive-stick cardinal snap and clear stale yaw integral on release.
- Require an orientation pose to remain stable for two seconds. Retain the video
  session across layout/configuration changes and reattach renderers without
  closing playback; retry network failures until explicit Stop.
- Display trimmed control pitch separately from raw IMU angle. Publish a signed,
  installable release APK using the stable project development signing key.

## 4.1.0 — One-pull deployment and status ring

- Return to current-only control for the owner's 5S / 18 V, 2.6 kg bot. Remove
  velocity balancing/bench command paths and migrate old velocity snapshots to
  the targeted current baseline while retaining trim, calibration and bot tools.
- Move GPIO15 ring transmission to I2S1 DMA, clear pixels before boot and skip
  busy frames to address colour corruption reported when arming under load.
  Preserve every animation branch from pre-BLDC stepper revision `a8427f0`.
- Add an active-high GPIO32 blue button LED: boot/wait blink, balancing breathe,
  ready steady-on, arming blink, stop/maintenance pulses and fast error blink.
  Independent hardware PWM/timer keeps the indication running during calibration.
- Keep attitude feedback at full strength: recorded full-stick frames showed
  terrain scaling nearly erased or reversed the catch command near six degrees.
- Remove free-speed/SPDHEAD derating of DRIVEV and bypass the predictive
  drive-reference governor for the requested unrestricted full-stick test.
  Preserve existing tilt/rate, current, link/fault and operator-stop protections.
- Manage the existing hotspot's 2.4 GHz/channel 6 setting through one-pull,
  verify the live radio and restore the previous band on activation failure.
- Add managed fast-forward pulls, commit-bound CI firmware/APK bundles, local
  firmware build support and input/toolchain fingerprint caching.
- Stage versioned host releases, hold controller maintenance across restarts,
  queue firmware through its existing worker and report verified/failure state.
- Restore the original firmware-only ring animations and remove Radxa ESP32 LED
  ownership, including deployment overrides. A separately wired SPI backend is
  opt-in; the normal deployment LED service is stopped/disabled.
- Add a deployment dashboard and persist Android downloads
  separately from application release snapshots.

## 4.0.3 — RS485 transport regression and full-stick headroom

- Revert the 4.0.1/4.0.2 one-outstanding-request transport. Each motor again
  receives exactly one frame per 5 ms cycle (supplied BaseLinkHub behaviour).
  4.0.2 skipped drive frames while a reply was pending and then resynchronised
  for >=10 ms, so a single late/lost reply froze that wheel's torque for
  ~25 ms and loop jitter halved the update rate. Mode safety is unchanged:
  every reply carries the motor mode byte, so a velocity-mode reboot disarms
  on the next reply. Late replies with an unknown DRIVE/QUERY layout update
  only the common fields.
- Full stick now maps to min(DRIVEV, SPDHEAD x free speed, MAXSPD). New
  SPDHEAD (default 0.70) keeps the catch margin the balance loop needs; free
  speed is FREERPM when measured (AT,stall), else SUPPLYV x Kv. At 12 V that is
  ~0.51 m/s; 1.00 m/s needs a measured free speed >= 1.43 m/s (higher supply).
  Navigation velocity obeys the same limit. New PR,VDRIVE readback.
- Log motor fault-byte transitions with their meaning.
- Control law (4/4/0.5 common loop, direct acceleration feed-forward) unchanged.

## 4.0.2 — RS485 arming dropout

- Keep the last confirmed motor mode/position through an individual missing
  reply; retain the 60 ms sustained-link-loss cutoff and immediate real-fault stop.
- Service each UART independently and confirm mode after resynchronisation
  before sending more drive current. Log the exact mode, fault, reply ages,
  timeout counts and CRC counts when a motor-health disarm occurs.

## 4.0.1 — Full-stick balance recovery

- Remove the supply/Kv estimate from speed/reference/steering clamps. MAXSPD=0
  is uncapped; full-stick HIGH defaults to 1.00 m/s. Explicit positive MAXSPD
  values are respected independently of the estimate.
- Pass LQI acceleration directly to current feed-forward, preserve catch torque
  during overspeed recovery, and back-calculate virtual speed state against the
  existing current budget rather than an absolute model speed.
- Retune the delayed common velocity loop to 4/4/0.5 and retain full motor-current
  authority. Accept queued motor replies before declaring timeout on CPU jitter.
- Record measured free RPM without setting a speed ceiling. Parse both original
  and v4 hub telemetry, and add regression coverage for the captured fall states.

## 4.0.0 — M0601 BLDC hardware

- Replace STEP/DIR, TMC2226 and PCNT wheel drivers with independent M0601
  RS485 links using the supplied hub firmware pin map and encoder units.
- Set confirmed ISGN/RPMSGN to -1 and default HUBVSRC to 1. Retain supplied
  straight-line/balance gains and limits.
- Separate balance/turn current, reserve balance current first, remove steering
  acceleration from common feed-forward, and slew heading hold/turn release.
- Default full-stick yaw to 1.5 rad/s instead of the supplied ~7.7 rad/s.
- Track one request per UART, resynchronise late replies, and keep temperature
  queries out of armed position feedback. Stream parameter dumps without
  blocking control; keep NVS and drive-mode changes to stopped motors.
- Add a separate BLDC NVS namespace and update-image hardware schema, a MOTOR
  tuning group, hub telemetry parsing and offline C++ regression checks.

## 3.40.0 — Routes landscape rendering fix

- Preserve the active workspace when switching between the bottom bar and rail,
  including landscape windows narrower than 600 dp.
- Bind WebView lifecycle handling to its actual instance and release it only
  when AndroidView releases that instance; use explicit bounded view sizing.
- Give the embedded route page definite viewport height, a compact landscape
  layout and scrollable fallback for very short windows. Add native Reload/error state.

## 3.30.0 — adaptive Android cockpit

- Rebuild navigation around five primary destinations and a compact landscape rail.
- Redesign Drive/Live/Map around a dominant scene and bounded instrument panes;
  correct live camera framing to 16:9.
- Add adaptive Camera/Setup/Sessions columns and searchable lazy parameter rows.
- Open Routes directly, with a responsive embedded map/teaching workspace.
- Unify surfaces, typography, spacing and insets; preserve the installed controller
  and original-command replay behavior. See [design notes](docs/ANDROID_COCKPIT_DESIGN.md).

## 3.20.0 — post-3.10 development revision

Numbering follows **3.10.0 → 3.20.0 → 3.30.0**. The prototype features below
are consolidated into 3.20.0. Existing local artifacts and recorder metadata
retain their original identifiers for hash verification and rollback.

### Original-command replay and more responsive reference governor

- Record timestamped 20 Hz speed/yaw/odometry samples, preserving reverse,
  pauses and original timing. Remove the replay-only speed/yaw ceiling, host
  ramp, duration/length caps and predictive-recovery mission interruptions.
- Remove the speed selector for recorded replay; old waypoint-only routes need
  re-teaching to obtain timing that was never stored.
- Double reference acceleration allowance, triple jerk allowance, reduce
  prediction horizon and recovery latch, and make mild hazard attenuation quadratic.

### Supervised routes / waypoints

- Teach start-relative wheel-odometry routes, preview them, and explicitly replay
  them from the marked physical start; include a ready-to-create 2 m straight route.
- Default cruise target is 0.6 m/s, including climb assistance; dense taught
  waypoints do not impose a hidden 0.25 m/s limit. Slow for turns and goal approach.
- Keep ToF advisory only: no depth prerequisite, obstacle hold or distance speed
  cap for routes. Remove camera/payload-position and payload-input dependencies.
- Add visible-page supervision, controller watchdog and latched drive-stick takeover.
- Add native navigation leases and separate 10 Hz navigation feedback; retain
  all saved parameters, startup gating and local-only motor arming.
- Expose the route page from Android Auto and the browser console.

### Android latest-build reflash and independent bot worker

- Add Setup firmware controls with compiled version/hash, progress and result.
- Register validated successful builds and publish over SSH; pin each request
  to the displayed build ID.
- Keep camera/API online while an independent unprivileged service flashes
  only the app, verifies rollback and boot/NVS, restores tuning and checks calibration.
- Add maintenance arming/raw-drive lock, durable recovery, duplicate/mutation
  rejection and serial reconnect baseline reset. Updates never auto-arm.
- Document pitch-gimbal and vibration-isolated camera/ToF pod recommendations.

### Reliable hand-back and controlled descent

- Separate uphill deficit from direction-aware downhill/braking cues, including
  sustained uncommanded rolling. Add DOWNCLIMB, EXITING and COOLDOWN states.
- Replace the perfect-sample exit timer with accumulated evidence and monitored
  normal-controller hand-back. Permit quiet flat-ground neutral release while
  retaining short pauses on a slope, and reject failed hand-backs.
- Add a descent-reference governor using saved CLMBV, measured overspeed and
  gradual recovery. Preserve reverse braking and full wheel balance authority.
- Append ten exit/descent diagnostic fields; retain all 107 saved parameters.

## 3.10.0 — automatic load-aware climb and retained slope compensation

- Detect sustained speed-reference error and pitch-baseline bias with 0.4 s
  filters, request confirmation, per-wheel following checks and hysteresis.
- Blend normal/climb control and speed/steering authority over 0.75 s; preserve
  compensation during brief neutral stops and confirm flat recovery for 5 s.
- Give D-pad/H selections a persistent manual override, with AC,1 / H,3 to
  resume auto and AC,0 to disable it. These commands never arm the robot.
- Fix the pg-to-home integral regression: mild 25% recovery no longer drains
  climb compensation; high immediate hazards and velocity saturation still
  bleed state, with conditional acceleration anti-windup.
- Add 14 diagnostic fields, an auto-climb control page and recorder/monitor
  support. Preserve all 107 saved parameters and the startup/calibration gate.

## 3.9.0 — predictive fall-prevention reference governor

- Shape operator speed requests with bounded reference acceleration/jerk and
  predict near-term tilt and wheel-speed margin before the existing fall cutoff.
- Prioritize full selected pitch/rate feedback during manoeuvres/recovery,
  attenuate velocity-tracking error and yaw, and retain stronger stiff-hold gains.
- Allocate steering from per-wheel speed headroom without reducing common balance
  acceleration or changing the 107 saved parameters, fall cutoffs or startup gate.
- Append override/reference/wheel-target debug fields and expose them in the
  read-only drive monitor and recorder. Add production-controller constraint tests
  and recorded-state replay; physical fall reduction requires a repeat drive test.

## 3.8.0 — ONNX objects viewer

- Add a bounded snapshot-based SSD-MobileNetV1 INT8 detector, COCO labels,
  confidence/person filters and temporary class-aware track IDs.
- Add web/Android Objects viewers and inference/frame-age metrics; saved lens
  correction can be used without guessing camera–ToF object distances.
- Provide a fixed checksum-verified model installer and optional ARM64 CPU runtime.

## 3.7.0 — hold firmware initialization until Radxa ACK

- Move IMU/driver/gamepad/payload/LED startup and gyro calibration behind BR,1.
- Keep serial, safe disabled outputs and read-only parameter discovery available
  during the hold; host readiness is independent of IMU telemetry.
- Report WAIT_HOST, INITIALIZING, CALIBRATING, READY and ERROR startup phases.
- ACK does not arm the bot; calibration must finish before a fresh local START.

## 3.6.0 — frame timing and passive depth diagnostics

- Add checksum-bound camera snapshot metadata with V4L2 monotonic capture time,
  publisher/sequence identity and publication timing.
- Record ToF host-read timestamps/uncertainty, sequence, signal/ambient/reflectance
  and bounded depth history; pair snapshots with nearest depth frames.
- Show near-field, inconsistent and rejected-zone reasons plus raw depth grid in
  the calibration GUI. Pairing is time-aware, not claimed hardware synchronization.

## 3.5.0 — guided camera calibration suite

- Add a progressive web/Android wizard with printable ChArUco/checkerboard targets,
  target detection, sharpness/stability checks, coverage guidance and automatic capture.
- Automatically compare pinhole/fisheye fits using held-out views, retain sessions,
  generate raw/corrected previews, and export geometry, captures and PDF reports.
- Use snapshots from the existing camera owner; no competing capture device or
  colour/exposure changes. Actual coefficients require physical target captures.
- Move the live ToF SDK to a managed location before removing the old calibration app.

## 3.4.0 — remove fixed creep limits and prepare vision roadmap

- Remove fixed 0.20/0.45 m/s unknown/partial-depth caps. Fresh unconfirmed depth
  leaves the normal DRIVEV limit; confirmed obstacles still apply braking/stop
  limits and stale/offline depth still holds forward requests.
- Keep unknown depth explicitly labelled as unobserved rather than clear.
- Document camera distortion/intrinsics, camera/base and camera/ToF extrinsics,
  time-aligned distance overlays and a later measured ONNX perception pipeline.

## 3.3.0 — Drive alignment and richer chase world

- Use one forward-first world frame for CAD, encoder movement, trail and depth.
- Read the existing PITCHSGN calibration for display; raw negative forward tilt
  now leans the CAD forward rather than backward.
- Keep grounded CAD yaw aligned with odometry; idle compass changes cannot leave
  a sideways heading offset when forward driving resumes.
- Add a centered rear chase view and richer dark-world reference guidance.
- Expand the world to the full viewport with floating speed/status and collapsible
  map/details; add a fading direction ribbon, range arcs and ground parallax cues.
- Camera follow/explore changes preserve the view; all scene guidance is labelled
  as a reference rather than fabricated sensor-detected terrain or roads.

## 3.2.0 — refined Android Drive world

### Less intrusive collision guard and reference calibration

- Use a 0.20 m close-obstacle clearance, 0.18 s reaction model and narrower wheel
  corridor; partial coverage permits up to 0.45 m/s, fresh unknown depth creep
  up to 0.20 m/s. Confirmed collisions stop requests; stale/offline depth holds.
- Add measured-reference range-bias calibration to Drive, persisted separately
  from robot NVS. Require stable central returns and a bias within ±0.10 m.
- Preserve raw ranges alongside corrected ranges; never fabricate calibration
  from invalid/self-reflection returns. Balance/reverse/steering are unchanged.

### Live world motion and lift evidence

- Move CAD in the same metre-based frame as the encoder trail, with interpolated
  heading/pitch, encoder wheel rotation, camera following and exploration mode.
- Show compass-observed hand-held yaw when wheel odometry cannot observe rotation.
- Show existing firmware airborne/landing events, low-g and handled/tilted cues.
  Lift animation is symbolic; steady hand-held height/contact remain unmeasured.
- Freeze stale pose and preserve short-angle interpolation across 0/360 degrees.

### Dark touch-navigable Drive world

- Replace Android's light painter-style scene with a hardware depth-buffered 3D
  world, plain dark floor, fog, contact shadow and smoothly shaded CAD surfaces.
- One finger orbits, two fingers pan, pinch zooms; Recenter restores the view.
- Add dark instrument cards and a toggleable trail panel. No CAD edge/wireframe
  pass is drawn. Camera gestures change the view only.
- Use full-resolution printed CAD surfaces for the Android world, preserving
  vents/cut-outs rather than decimation artefacts; retain smooth tyre shading.

### Prepared assembled robot launcher icon

- Render a front three-quarter app icon from all seven full-resolution current
  robot STL components, using matching parametric tyres/hubs and app-theme colours.
- Generate 1024px artwork, density-specific launcher/adaptive layers, round icons
  and Android 13 themed monochrome icons with adaptive-mask safe padding.
- Keep the render reproducible with `python scripts/generate_launcher_icon.py`.

## 3.1.7 — measured-depth driving scene and projected guard

- Replace the web ToF grid with a grey driving scene: trail left, current assembled
  STL robot and accepted depth returns centrally, speed/status right. Add matching
  Android Drive destination, preserving existing camera and Live views.
- Use seven current printed CAD components plus parametric tyres/hubs; ship meshes
  locally, without a CDN or the obsolete simplified STEP model.
- Confirm isolated depth returns over successive frames; immediate close returns
  stop forward requests. Apply reaction/braking limits inside a swept corridor.
- A missing zone no longer blocks every request: partial coverage reduces speed;
  stale/wholly unknown depth stays blocked. Reverse, steering and balance recovery
  retain their existing authority. Guard remains opt-in after restart.
- Expose coverage, confidence, measured points and stopping distance; mounting and
  optical projection are explicitly approximate, with a reference floor.

## 3.1.6 — labelled terrain sessions

- Add a Data destination with start/stop, terrain markers and outcome notes.
- Capture ordered raw WebSocket/HTTP events to disk before app decoding, including
  ToF, unknown message types, disconnects, parameter changes and app commands.
- Export persisted sessions as ZIP: JSONL, state/action CSV, live parameter/version
  metadata and pitch/speed/distance/staleness/saturation diagnostics.
- Preserve unassigned reward/terminal fields for offline RL preparation; camera
  video and unobserved sensor internals are not represented as captured data.

## 3.1.5 — controlled three-way rotation

- Support portrait, landscape and reverse landscape with sensor hysteresis.
- Ignore reverse portrait and unknown/flat sensor readings; rotation tests cover
  all permitted windows and the excluded upside-down state.

## 3.1.4 — foreground screen keep-awake

- Keep the display awake while the robot app is visible; clear the window flag
  when leaving the app without requiring wake-lock or background permissions.

## 3.1.3 — complete landscape navigation

- Landscape navigation adapts item height to available screen height and keeps
  all destinations accessible, with scrolling only on unusually short displays.
- Stop remains fixed and accessible above the destination list.

## 3.1.2 — trail reports

- Map exports a rendered trail/robot-position image as PNG or PDF.
- Optional telemetry includes the current pose, controller debug values and
  connection validity; disabling it exports the trail image without the data panel.

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
