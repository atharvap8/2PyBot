# Blue power-button status LED — GPIO 32

Active-high wiring: GPIO 32 to the LED's positive input, LED negative to GND.
This controls the indicator LED, not the latching switch contacts.

| State | Blue indicator |
| --- | --- |
| Boot, waiting for Radxa, sensor settling/calibration | Continuous blink: 500 ms on / 500 ms off |
| Ready, motors off | Steady on |
| START requested, waiting upright | Blink: 200 ms on / 200 ms off |
| Balancing | Smooth breathing fade every 2.4 seconds |
| Operator stop / e-stop | Two short flashes, repeating every 1.6 seconds |
| Firmware maintenance / auto-tune | Three short flashes, repeating every 1.6 seconds |
| IMU failure, motor link/mode/fault, fall, rate spike or control-loop overrun | Fast blink: 100 ms on / 100 ms off |

Fall/rate/overrun/motor-stop errors stay latched until the next accepted local
START/E request. A still-present motor fault continues to show fast blinking.

Hardware PWM uses LEDC channel 0 / timer 0 at 1 kHz, separate from the servos,
torch and WS2812 ring. An ESP timer updates brightness at 50 Hz so boot waits,
calibration delays and fatal-error loops do not freeze the animation. The LED
runs after firmware starts; it cannot animate while the ESP32 is reset/flashing.

Pin, polarity and PWM allocation are defined in `firmware/BaseLink/config.h`.
Tests were intentionally skipped at the owner's request.
