# Lidar mapping (RPLIDAR C1) — standalone, no ROS

Open **http://10.42.0.1:6970/** (or the console at `/slam/`). The page walks you
through five steps: check the lidar, check the wheels, make a map, save it, use it.

## Wiring (40-pin header, UART4)
| C1 | A7A pin | Signal |
|---|---|---|
| 5 V | 2 or 4 | +5 V |
| GND | 6 | GND |
| TX | 18 | PJ25 / UART4-RX |
| RX | 16 | PJ24 / UART4-TX |

Avoid pins 3/5 (I2C bus 7, ToF) and 8/10 (debug console). Enable with
`rsetup` → Overlay → Manage overlays → **Enable UART4**, reboot → `/dev/ttyAS4`.

## How it works
- `console/lidar_c1.py` — C1 driver (pyserial). Owns only the lidar UART.
- `console/slam/odom_core.py` — wheel + gyro odometry. Finds the vertical gyro
  axis automatically while you spin the robot. The compass is not used.
- `console/slam/grid.py` — 5 cm occupancy grid and a correlative scan matcher (numpy).
- `console/slam/engine.py` — skips scans while the robot leans >3° or rocks
  fast, clips the range where the tilted laser would hit the floor, maps, saves,
  loads and localizes (global search on load and on **Find me**).
- `console/slam_app.py` + `ui/slam/` — the page and API (`/api/state`, `/ws`,
  `/api/mapping/*`, `/api/maps/*`, `/api/relocalize`, `/api/test/*`, `/api/estop`).

It is **listen-only**: it reads raw `O`/`D` lines through
`SerialLink.add_line_listener()` and never sends drive commands. The STOP button
uses the console's normal E-STOP (allowed during firmware maintenance, like the
console and system-map STOP routes). The lidar and SLAM threads start with the
console service and the C1 is sent STOP when the service shuts down.

## Settings (environment)
`BOT_SLAM_PORT` (6970, 0 = only `/slam/`), `BOT_LIDAR_PORT` (`/dev/ttyAS4`),
`BOT_LIDAR_BAUD` (460800), `BOT_MAPS_DIR` (`~/.2pybot/maps`), `BOT_LIDAR=0` to
leave the lidar off. Mount numbers and thresholds: `DEFAULTS` in `slam/engine.py`.

## Try it without the robot
`cd software/radxa/console && python -m console.slam_demo` → http://localhost:6970/

## Not verified on hardware yet
1. The A7A UART clock at 460800 (`sudo grep -i uart /sys/kernel/debug/clk/clk_summary`).
2. That the C1 answers legacy `SCAN 0x20` with 5-byte nodes (the driver falls
   back to sending the motor-speed command and says so on the page).
3. `lidar_height` (0.26 m) and `lidar_y` (−0.031 m) come from the CAD assembly;
   the optical window height on the C1 head is not measured.
