# 2PyBot — Radxa Ops Runbook

Everything needed to operate the robot's onboard Radxa Cubie A7A
(Debian 11, `ssh radxa@192.168.10.132`). Scripts live in `~/2pybot/scripts/radxa/`.

## Architecture recap

```
EvoFox One S ──BT──> Radxa (Docker: ROS 2 Humble)
                      ├─ joy_node + pybot_teleop ──> /cmd_vel
                      ├─ micro-ROS agent <──USB serial──> ESP32 (client)
                      └─ foxglove bridge ──ws:8765──> your PC (viz)
ESP32: balance PID @200Hz, steppers @20kHz — always standalone-safe
```

## Quick reference

| Task | Command (on Radxa) |
|---|---|
| Start ROS stack | `~/2pybot/scripts/radxa/start_stack.sh up` |
| Stop stack | `start_stack.sh down` |
| Container status | `start_stack.sh status` |
| Follow logs | `start_stack.sh logs [container]` |
| Flash ESP32 | `flash_esp32.sh` |
| ROS shell in container | `ros_shell.sh` |
| Any ros2 cmd | `ros_shell.sh ros2 topic list` |
| Joint states live | `joint_states.sh` |
| EvoFox stream (joy) | `evofox_monitor.sh joy` |
| EvoFox raw (no ROS) | `evofox_monitor.sh raw` |
| Pair EvoFox | `pair_evofox.sh scan` then `pair_evofox.sh pair <MAC>` |
| Raw ESP32 serial | `monitor_esp32.sh` (auto stops/restarts agent) |
| Remote viz bridge | `foxglove.sh` -> connect ws://radxa:8765 |

## 1. Full startup, from cold

```bash
ssh radxa@192.168.10.132
cd ~/2pybot/scripts/radxa

./pair_evofox.sh status          # controller connected? js0 present?
./pair_evofox.sh connect <MAC>   # if not (pad must be on)
./start_stack.sh up              # teleop + micro-ROS agent
./start_stack.sh logs 2pybot_microros_agent   # wait for "session established"
```

Then press **A** on the pad to enable motors; **B** is kill.
Left stick Y = drive, right stick X = steer.

## 2. Verify each link of the chain

```bash
./evofox_monitor.sh raw    # gamepad -> kernel OK? (axes move)
./evofox_monitor.sh joy    # joy_node -> ROS OK?
./evofox_monitor.sh cmd    # teleop shaping -> /cmd_vel OK? (press A first)
./ros_shell.sh ros2 node list          # /pybot_esp32 = micro-ROS up
./ros_shell.sh ros2 topic echo /pybot/pitch    # ESP32 -> ROS telemetry
./joint_states.sh          # wheel angles live
./ros_shell.sh ros2 topic hz /pybot/state      # telemetry rate check
```

## 3. Flashing the ESP32 (from the Radxa)

```bash
# sync latest firmware from your PC first:
#   scp -r firmware radxa@192.168.10.132:~/2pybot/
./flash_esp32.sh                 # compiles + uploads BaseLink
./flash_esp32.sh ~/2pybot/firmware/BaseLink /dev/ttyUSB0   # explicit
```

The script stops the micro-ROS agent (it owns the USB port), flashes,
and restarts it. First compile takes a few minutes on the A7A.

Firmware config lives in `firmware/BaseLink/config.h`:
- `USE_MICROROS 1` — ROS mode (current). `0` = legacy ESP-NOW joystick
- `MR_TRANSPORT 0` — USB serial to agent. `1` = WiFi UDP
- In serial mode, USB debug prints and the USB tuner are disabled;
  Bluetooth telemetry (GUI) still works.

## 4. Visualization on your PC (Arch Linux)

See `scripts/radxa/rqt_remote.md` for full copies.

```bash
# RViz (Arch, docker):
xhost +local:docker
docker run -it --rm --net=host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix osrf/ros:humble-desktop rviz2

# rqt graph / plots:
docker run -it --rm --net=host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix osrf/ros:humble-desktop rqt
```

RViz displays: Fixed Frame `base_link`; add RobotModel
(`/robot_description`), TF, Odometry (`/pybot/odom`).

rqt: Node Graph = Plugins > Introspection; Plot `/pybot/pitch/data`.

Fallback (always works): `./foxglove.sh` on the Radxa, then Foxglove
Studio -> `ws://192.168.10.132:8765`.

## 5. Rebuilding the ROS image (after ros2_ws changes)

```bash
# from PC: scp -r ros2_ws radxa@192.168.10.132:~/2pybot/
cd ~/2pybot/ros2_ws
docker build -t 2pybot:latest -f docker/Dockerfile .
cd ~/2pybot/scripts/radxa && ./start_stack.sh restart
```

## 6. Troubleshooting

| Symptom | Fix |
|---|---|
| `/joy` silent | `pair_evofox.sh status`; re-pair; check `evofox_monitor.sh raw` |
| No `/pybot_esp32` node | agent logs; replug USB; `flash_esp32.sh` stops/starts agent correctly? |
| Agent "bind error" | another process owns /dev/ttyUSB0 — `monitor_esp32.sh` left open? |
| Robot ignores /cmd_vel | motors enabled? (press A; check `ros2 topic echo /pybot/enable`) |
| Container restart-loop | `start_stack.sh logs`; usually a launch file error after ws change |
| No topics on PC | firewall/multicast; use Foxglove fallback |
| GUI telemetry needed | Bluetooth serial still streams; run gui/robot_controller_ui.py on PC |

## 7. Upload environment facts (Radxa)

- `arduino-cli` 1.5.1 at `~/.local/bin` (on PATH in interactive shells)
- ESP32 core `esp32:esp32@3.3.10`; FQBN `esp32:esp32:esp32:PartitionScheme=huge_app`
- Libraries in `~/Arduino/libraries`: TMCStepper, STM32duino ISM6HG256X,
  QMC5883LCompass, FastLED, micro_ros_arduino (humble)
- ESP32 on `/dev/ttyUSB0`; user in `dialout` + `docker` groups
- `tio` for raw serial (`Ctrl-t q` to quit)
