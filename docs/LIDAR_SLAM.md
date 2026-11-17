# Lidar mapping (RPLIDAR C1) — standalone, no ROS

Open **http://10.42.0.1:6970/** (or the console at `/slam/`). The page walks you
through five steps: check the lidar, check the wheels, make a map, save it, use it.

![Live mapping](images/slam/3_mapping_live.png)

More screenshots (simulator): [lidar check](images/slam/1_radar_lidar_check.png) ·
[wheel tests](images/slam/2_wheel_tests.png) · [saved + glossary](images/slam/4_saved_with_glossary.png) ·
[localizing](images/slam/5_localizing_on_saved_map.png) · [phone](images/slam/6_phone_landscape.png)

## Wiring (40-pin header, UART4)
| C1 | A7A pin | Signal |
|---|---|---|
| 5 V | 2 or 4 | +5 V |
| GND | 6 | GND |
| TX | 18 | PJ25 / UART4-RX |
| RX | 16 | PJ24 / UART4-TX |

TX/RX cross over. Logic is 3.3 V on both sides; only the C1 supply is 5 V.
Avoid pins 3/5 (I2C bus 7, ToF) and 8/10 (debug console).

### Bot setup (once, both needed)
1. **Enable UART4:** `sudo rsetup enable_overlays sun60iw2p1-uart4.dtbo`
   (or `rsetup` → Overlay → Manage overlays → Enable UART4).
2. **UART clock 96 MHz:** all A733 UARTs run from a 24 MHz crystal by default,
   which cannot make 460800 (the C1's fixed rate; you get ~500000). The overlay
   [`scripts/radxa/overlays/2pybot-uart-pll96m.dts`](../scripts/radxa/overlays/2pybot-uart-pll96m.dts)
   moves them to PLL-PERI0-480M / 5 = 96 MHz (460800 → +0.16 %, 115200 unchanged):
   ```bash
   dtc -@ -q -I dts -O dtb -o /tmp/2pybot-uart-pll96m.dtbo scripts/radxa/overlays/2pybot-uart-pll96m.dts
   sudo install -m 0644 /tmp/2pybot-uart-pll96m.dtbo /boot/dtbo/
   sudo u-boot-update && sudo reboot
   ```
   Undo: `sudo rm /boot/dtbo/2pybot-uart-pll96m.dtbo && sudo u-boot-update && sudo reboot`.
3. Check after reboot: `/dev/ttyAS4` exists and
   `sudo cat /sys/kernel/debug/clk/apb-uart/clk_rate` prints `96000000`.

Verified on the bot (2026-11-17): C1 model 65, firmware 1.02, hardware 18,
health good, 10 Hz, ~400 points per rotation.

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

## Hardware findings
- 460800 on UART4 needs the 96 MHz clock overlay above (verified).
- The C1 answers legacy `SCAN 0x20` with 5-byte nodes, but only after the HQ
  motor-speed command (`0xA8`); the driver sends it automatically after the first
  empty SCAN (verified, ~1.5 s delay at connect).

## Not verified on hardware yet
1. `lidar_height` (0.26 m) and `lidar_y` (−0.031 m) come from the CAD assembly;
   the optical window height on the C1 head is not measured.
