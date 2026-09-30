# Legacy ESP-NOW Joystick Transmitter

Reference: [Controller.ino](../../firmware/Controller/Controller.ino). This sketch remains in the repository for the older ESP-NOW control path. Current BaseLink uses a direct Bluepad32 gamepad and has no ESP-NOW receiver.

## Hardware and build

| Signal | GPIO |
| :--- | :--- |
| Joystick X | 34, ADC1 |
| Joystick Y | 35, ADC1 |
| Joystick switch | 32, pull-up, active LOW |
| Status LED | 2 |

Power the joystick at 3.3 V with common ground. ADC1 is used because Wi-Fi occupies ADC2 resources. The sketch uses the stock ESP32 Arduino 3.x core and the `wifi_tx_info_t` send-callback signature. Serial diagnostics run at 115200 baud.

Set `receiverMAC[]` to the intended legacy receiver's STA address. The transmitter configures an unencrypted peer on channel 0.

## Sampling and processing

1. On boot, average 64 samples per axis at 5 ms spacing to capture the resting center.
2. Every 20 ms, average four ADC readings per axis.
3. Subtract the calibrated center and divide by `JOY_RANGE=2048`.
4. Apply an EMA with `JOY_SMOOTH=0.7`.
5. Apply a 0.10 deadzone and rescale the remaining range.
6. Clamp to -1 through 1, then scale forward by 5.0 and steering by 1.0.
7. Poll the switch. A press edge toggles the enable flag; no explicit debounce interval is implemented.
8. Send the packed packet when the peer was added successfully.

## Packet

```cpp
typedef struct __attribute__((packed)) {
    float forward;
    float steering;
    uint8_t enable;
} JoystickPacket;
```

The packed payload is 9 bytes: two 32-bit floats and one enable byte. Forward uses the legacy -5 through +5 target-offset units; steering is -1 through +1. The receiver must agree on layout, scaling, and radio channel.

## Diagnostics

- Boot prints the local STA MAC, target MAC, center calibration, and peer status.
- ESP-NOW initialization failure halts the sketch.
- Peer-add failure leaves it running without transmission.
- Failed sends toggle the status LED.
- Every 200 ms, serial prints forward, steering, and enable values.

For the active gamepad mapping and pairing instructions, see [BaseLink README](../../firmware/BaseLink/README.md).
