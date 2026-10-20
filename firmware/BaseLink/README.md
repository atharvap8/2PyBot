# BaseLink Firmware

ESP32 v4.0.1 firmware for two M0601 BLDC hubs, with LQR/LQI balancing,
direct Bluepad32 gamepad input, runtime parameters, terrain adaptation, a
WS2812 ring, and camera payload control. [BLDC integration details](../../docs/BLDC_V4.md)
cover the control changes and offline checks.

## Build

Open `BaseLink.ino` from this folder. The sketch filename must match the `BaseLink` directory name.

Add both board-manager URLs in Arduino IDE preferences:

```text
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
```

Install **ESP32 + Bluepad32 Arduino**, select **ESP32 Dev Module**, and choose **Huge APP (3 MB)**. Firmware CI uses `esp32-bluepad32:esp32@4.1.0` with this FQBN:

```text
esp32-bluepad32:esp32:esp32:PartitionScheme=huge_app
```

Install these libraries:

- STM32duino ISM6HG256X
- QMC5883LCompass
- NeoPixelBus by Makuna

`Preferences` comes with the ESP32 core. The stock ESP32 board package does not supply `Bluepad32.h`. The separate legacy Controller sketch uses the stock ESP32 3.x core.

## Pin map

Values below come from `config.h`.

| Signal | GPIO |
| :--- | :--- |
| Right RS485 TX / RX / DE | 19 / 18 / 23 (Serial2, motor ID 2) |
| Left RS485 TX / RX / DE | 17 / 16 / 4 (Serial1, motor ID 1) |
| I2C SDA / SCL | 21 / 22 |
| WS2812 DIN | 15 |
| Camera pan / zoom | 26 / 0 |
| Torch MOSFET gate | 2 |

Each motor has its own RS485 transceiver and UART at 115200 8N1; DE and /RE
are tied and driven by UART RTS. The supplied harness is ESP32-WROOM (16/17
would conflict with PSRAM on WROVER). Wheel feedback comes from the hubs.
`ISGN=-1`, `RPMSGN=-1`, `LDIR=-1`, `RDIR=+1`, and `HUBVSRC=1` are the v4 defaults.

Power the ring and servos from the 5 V rail with common ground. The ring has 16 LEDs, a compiled brightness setting of 200/255, and a 50 Hz refresh setting. GPIO 0 and GPIO 2 are boot-strapping pins; external wiring must preserve their required reset levels.

## Boot and serial connection

Use USB serial at **460800 baud**, 8N1, with newline termination.

Boot loads the `2pybot-bldc4` parameter namespace and waits for host readiness
when BOOTWAIT is enabled. It then initializes sensors and hubs, settles for
3000 ms, calibrates the gyro and starts the gamepad/payload. The state is IDLE.

Check for:

```text
[PARAM] ... tunables ...
[IMU] Initialisation OK
[HUB] RIGHT id2 OK; LEFT id1 OK
[PAD] Bluepad32 ready ...
```

Missing motor replies block normal arming. Mode loss, faults or link timeout
while balancing disarm. IMU initialization failure halts setup.

`P?` streams settings incrementally. `PL`, `PD` and `PS` require stopped motors,
as do mode/direction/feedback-source changes. The old stepper namespaces and
checkpoint are historical data and are not applied to BLDC motor settings.

## Gamepad pairing

Put the EVOFOX One S in **Home+B** pairing mode. Keep both sticks centered until the center-calibration log appears. Firmware adopts held buttons on the first processed connection frame; release START before pressing it to arm.

If pairing keys need clearing, enable the existing `BP32.forgetBluetoothKeys()` call for one flash, pair again, then disable that call.

## Controls

| Input | Action |
| :--- | :--- |
| Left stick Y | Forward/backward drive |
| Left stick X | Steering |
| Right stick X | Camera pan rate |
| Right stick Y | Camera zoom rate |
| START | Request arming |
| SELECT | Stop balancing |
| LB | Toggle LOW/HIGH speed |
| RB | Toggle torch |
| D-pad UP | Toggle stiff hold |
| D-pad DOWN | Toggle climb mode |
| D-pad LEFT / RIGHT | Dim / brighten torch; repeat while held |
| Y / B / X / A | Nod yes / nod no / spin / dance |

LOW mode defaults to half drive and steering authority. Boot defaults to LOW. Speed mode remains selected across arm/disarm and reconnection.

`PAD_STEER_ON_LEFT_STICK=0` moves steering to right-stick X and disables camera pan input. This is a compile-time mapping choice.

## Balance and host control

Use START or serial `E` while the robot is within the arm-angle limit, default 5 degrees. `X` or SELECT stops balancing. Tilt beyond 55 degrees or pitch rate beyond 450 degrees/s also stops balancing.

Fresh host `V` lines override gamepad drive for 600 ms by default. Host enable going from 1 to 0 requests a stop; going to 1 does not arm. A first `V,...,0` is not an unconditional stop, so use `X` for an explicit balancing stop.

Gamepad disconnect zeros its axes without disabling balance. When all drive sources are stale, the controller receives zero drive input.

## Ring states

Priority order in `leds_update()`:

1. Fall, arm, or speed-change overlay.
2. Gesture rainbow.
3. Red hazard when host input is fresh, the robot is balancing, drive input is near zero, and measured speed exceeds 0.10 m/s.
4. Violet climb display.
5. Pink position-error display above 0.045 m while holding.
6. Direction beam while driving; amber for reverse.
7. Amber stiff-hold display.
8. Cyan balancing display with tilt-dependent color.
9. Green pending-arm display.
10. Idle ember display.

The hazard display is conditional, not a general indicator that the host owns drive authority. A green gamepad greeting is blended into selected patterns.

## Payload

Pan and zoom use LEDC channels 4/5 at 50 Hz and 16-bit resolution. Torch uses channel 6 at 20 kHz and 10-bit resolution. Camera input is rate-controlled, so releasing the stick leaves the requested position unchanged.

Pan defaults to 120 through 190 degrees with home 165; zoom defaults to 80 through 140 with home 120. The pulse-writing function clamps physical output to 180 degrees even when the requested pan value is higher.

Torch starts off with a compiled preset of 25%. `_torchApply()` clamps duty to the 90% compile-time cap. Payload updates run in both balance states. No disarm-time torch-off call is present in the current sketch.

## Related documentation

- [Serial protocol](PROTOCOL.md)
- [Module map](system_architecture.md)
- [Control theory](../../docs/BaseLink/PID_Theory_and_Math.md)
- [BLDC configuration and decisions](../../docs/BLDC_V4.md)
- [Troubleshooting](../../docs/BaseLink/Troubleshooting_Guide.md)
