// model.js — what the robot is made of, where each block sits on the
// 1920 × 1016 stage, and how it is wired. Health comes from the server;
// this file is the static half: layout, wiring, and what to check.
//
// Hardware facts follow firmware/BaseLink/config.h (v4, M0601 RS485 hubs)
// and the console's config/tof settings. Update this file when wiring changes.

export const STAGE = { w: 1920, h: 1016 };

// Bus colours are CSS variables in sysmap.css.
export const BUSES = [
  ["i2c", "I2C"], ["uart", "USB serial"], ["rs485", "RS485"], ["pwm", "PWM / LED data"],
  ["csi", "CSI video"], ["wifi", "Wi-Fi"], ["bt", "Bluetooth"],
];

// kind: leaf | hub | group.  side: where the block hangs from.
export const NODES = {
  camera: {
    kind: "leaf", x: 40, y: 68, w: 290, h: 104, icon: "camera",
    title: "IMX708 camera", sub: "Front lens, CSI ribbon",
    path: "IMX708 → MIPI CSI → Radxa ISP → H.264 encoder → MediaMTX → WebRTC",
    checks: ["Open the cockpit video. Black video with a running encoder points at the CSI ribbon.",
      "Restart video from the console Camera tab.",
      "Console service log shows the encoder's exit reason."],
    log: /stream|camera|encoder|mediamtx|cedar|imx708/i,
  },
  tof: {
    kind: "leaf", x: 40, y: 244, w: 290, h: 104, icon: "grid",
    title: "VL53L5CX ToF", sub: "8×8 depth, front face",
    path: "VL53L5CX → I2C bus 7 (0x29) → ToF worker → forward guard → TG to ESP32",
    checks: ["If the guard is on, the ESP32 holds forward speed at zero while depth is stale.",
      "Check the sensor cable and its 3.3 V supply.",
      "Run i2cdetect -y 7 on the Radxa; the sensor answers at 0x29."],
    log: /tof|vl53|depth/i,
  },
  clients: {
    kind: "leaf", x: 40, y: 418, w: 290, h: 104, icon: "phone",
    title: "Phones and browsers", sub: "Hotspot 10.42.0.1, 2.4 GHz",
    path: "Android app / browser → Wi-Fi hotspot → console :8080, video :8889, map :6969",
    checks: ["Hotspot down: check nmcli connection show --active.",
      "The app cannot drive; the gamepad stays on the ESP32."],
    log: /hotspot|wifi|nmcli/i,
  },
  radxa: {
    kind: "hub", x: 440, y: 196, w: 230, h: 236,
    title: "Radxa Cubie", sub: "Vision, console, hotspot",
    path: "Runs the console, the video stream, the ToF worker and this map",
    checks: ["High temperature throttles video first.", "journalctl -u 2pybot-console -n 100"],
    log: /\[console\]/,
  },
  esp32: {
    kind: "hub", x: 1240, y: 186, w: 240, h: 246,
    title: "ESP32", sub: "Balance controller, WROOM",
    path: "200 Hz balance loop, M0601 current control, Bluepad32 on core 0",
    checks: ["Startup waits for the Radxa ACK before IMU calibration.",
      "Loop over 20 ms disarms the robot.",
      "The blue button LED mirrors this state."],
    log: /\[(BOOT|MAIN|BS|STATE|ARM|FALL)\]|^BS,/,
  },
  imu: {
    kind: "leaf", x: 1590, y: 60, w: 290, h: 96, icon: "axis",
    title: "ISM6HG256X IMU", sub: "Accel + gyro, 960 Hz",
    path: "ISM6HG256X → I2C 400 kHz (GPIO 21/22) → Mahony AHRS → pitch filter → balance loop",
    checks: ["An init error halts startup; check the I2C harness.", "Keep the robot still during gyro calibration."],
    log: /\[IMU\]|FATAL/,
  },
  compass: {
    kind: "leaf", x: 1590, y: 188, w: 290, h: 96, icon: "compass",
    title: "QMC5883L", sub: "Compass, tilt-compensated", lineCount: 1,
    path: "QMC5883L → same I2C bus as the IMU → heading in the O record",
    checks: ["Firmware has no compass health flag. A heading that never moves while driving is the only hint.",
      "Magnets near the board (motors) skew it; recalibrate offsets in the IMU tab."],
    log: /QMC|compass|\[MAG\]/i,
  },
  gamepad: {
    kind: "leaf", x: 1590, y: 316, w: 290, h: 96, icon: "pad",
    title: "EVOFOX gamepad", sub: "Direct Bluetooth, Bluepad32", lineCount: 1,
    path: "EVOFOX One S → Bluetooth → Bluepad32 (core 0) → arming and drive input",
    checks: ["Pairing: Home + B.", "SELECT stops the robot locally; START arms it."],
    log: /\[PAD\]/,
  },
  lights: {
    kind: "leaf", x: 1590, y: 444, w: 290, h: 96, icon: "ring",
    title: "LED ring, button LED", sub: "WS2812B GPIO 15, blue GPIO 32",
    path: "ESP32 → I2S DMA → 16-LED ring;  ESP32 → LEDC PWM → blue button LED",
    checks: ["Compare the blue LED with the pattern shown here; a mismatch means a wiring or power fault."],
    log: /\[LED\]/,
  },
  wheel_l: {
    kind: "group", x: 690, y: 700, w: 390, h: 296,
    title: "Left wheel", sub: "M0601 id 1",
    stages: [["UART1", "TX 17, RX 16"], ["RS485", "DE 4, 115200"], ["M0601", "id 1, hub motor"]],
    path: "ESP32 UART1 → RS485 transceiver (DE GPIO 4) → M0601 id 1. DRIVE frames out, current/rpm/position back.",
    checks: ["Fault byte: 0x01 sensor, 0x02 bus overcurrent, 0x04 phase overcurrent, 0x08 stall, 0x10 overheat.",
      "Faults clear by themselves after about 5 s once the cause is gone.",
      "Rising timeouts with no fault: check A/B wiring and the transceiver's DE line."],
    log: /\[HUB\].*LEFT|LEFT id/,
  },
  wheel_r: {
    kind: "group", x: 1095, y: 700, w: 390, h: 296,
    title: "Right wheel", sub: "M0601 id 2",
    stages: [["UART2", "TX 19, RX 18"], ["RS485", "DE 23, 115200"], ["M0601", "id 2, hub motor"]],
    path: "ESP32 UART2 → RS485 transceiver (DE GPIO 23) → M0601 id 2. DRIVE frames out, current/rpm/position back.",
    checks: ["Fault byte: 0x01 sensor, 0x02 bus overcurrent, 0x04 phase overcurrent, 0x08 stall, 0x10 overheat.",
      "Faults clear by themselves after about 5 s once the cause is gone.",
      "Rising timeouts with no fault: check A/B wiring and the transceiver's DE line."],
    log: /\[HUB\].*RIGHT|RIGHT id/,
  },
  payload: {
    kind: "group", x: 1500, y: 700, w: 380, h: 296,
    title: "Camera payload", sub: "3 outputs",
    path: "ESP32 LEDC PWM → pan servo (GPIO 26), zoom servo (GPIO 0), torch MOSFET (GPIO 2)",
    checks: ["GPIO 0 and GPIO 2 are boot strap pins; a stuck servo can block flashing.",
      "The torch has a hard duty cap in firmware."],
    log: /\[PAY\]/,
  },
};

// Wires. `node` is the block whose health colours the wire.
// flow: packet direction along the path as drawn — "fwd", "rev" or "both".
export const WIRES = [
  { id: "m_radxa", mount: true, d: "M780 296 H670" },
  { id: "m_esp32", mount: true, d: "M1140 296 H1240" },
  { id: "serial", node: "serial", bus: "uart", flow: "both", speed: 1.0,
    d: "M560 196 V96 Q560 70 586 70 H1334 Q1360 70 1360 96 V186",
    tag: [960, 70, "USB serial 460 800: telemetry up, commands down"] },
  { id: "camera", node: "camera", bus: "csi", flow: "fwd", speed: 1.3,
    d: "M330 120 C400 120 380 250 440 250", tag: [385, 184, "MIPI CSI"] },
  { id: "tof", node: "tof", bus: "i2c", flow: "fwd", speed: 0.8,
    d: "M330 296 H440", tag: [385, 296, "I2C-7, 0x29"] },
  { id: "clients", node: "clients", bus: "wifi", flow: "rev", speed: 0.9, dashed: true,
    d: "M330 470 C400 470 380 340 440 340", tag: [385, 406, "Wi-Fi, WebRTC"] },
  { id: "imu", node: "imu", bus: "i2c", flow: "rev", speed: 1.4,
    d: "M1480 250 C1540 250 1530 108 1590 108", tag: [1535, 166, "I2C 400 kHz"] },
  { id: "compass", node: "compass", bus: "i2c", flow: "rev", speed: 0.7,
    d: "M1480 270 C1540 270 1530 236 1590 236" },
  { id: "gamepad", node: "gamepad", bus: "bt", flow: "fwd", speed: 0.9, dashed: true,
    d: "M1590 364 C1530 364 1540 320 1480 320", tag: [1535, 300, "Bluepad32"] },
  { id: "lights", node: "lights", bus: "pwm", flow: "fwd", speed: 0.6,
    d: "M1480 345 C1540 345 1530 492 1590 492", tag: [1535, 420, "GPIO 15, 32"] },
  { id: "wheel_l", node: "wheel_l", bus: "rs485", flow: "both", speed: 1.2,
    d: "M1300 432 V610 Q1300 640 1270 640 H910 Q880 640 880 670 V700",
    tag: [1080, 640, "RS485 id 1"] },
  { id: "wheel_r", node: "wheel_r", bus: "rs485", flow: "both", speed: 1.2,
    d: "M1350 432 V700", tag: [1350, 560, "RS485 id 2"] },
  { id: "payload", node: "payload", bus: "pwm", flow: "fwd", speed: 0.6,
    d: "M1410 432 V610 Q1410 640 1440 640 H1660 Q1690 640 1690 670 V700",
    tag: [1560, 640, "PWM 26, 0, 2"] },
];

// The serial link is a wire with its own health, inspected like a block.
export const SERIAL_INFO = {
  title: "USB serial link", sub: "Radxa ↔ ESP32",
  path: "Radxa /dev/ttyUSB0 ↔ ESP32 USB UART at 460 800 baud. O at 50 Hz, D, HM at 10 Hz up; V, P, TG, BR down.",
  checks: ["Only the console opens this port. A second program will break it.",
    "Port open but silent: the ESP32 may be held in reset or flashing.",
    "Opening the port must not reset the ESP32 (DTR/RTS are held low)."],
  log: /\[console\] (serial|opened)/,
};

export const PINS = {
  esp32: [
    ["17 / 16", "UART1 TX / RX", "Left M0601, RS485"], ["4", "RS485 DE + /RE", "Left transceiver"],
    ["19 / 18", "UART2 TX / RX", "Right M0601, RS485"], ["23", "RS485 DE + /RE", "Right transceiver"],
    ["21 / 22", "I2C SDA / SCL, 400 kHz", "ISM6HG256X IMU, QMC5883L"],
    ["15", "WS2812B data (I2S DMA)", "16-LED ring"], ["32", "LEDC PWM 1 kHz", "Blue button LED"],
    ["26", "LEDC PWM 50 Hz", "Pan servo"], ["0", "LEDC PWM 50 Hz (strap pin)", "Zoom servo"],
    ["2", "LEDC PWM 20 kHz (strap pin)", "Torch MOSFET"], ["USB", "UART0, 460 800 baud", "Radxa console"],
  ],
  radxa: [
    ["CSI", "MIPI camera port", "IMX708 → /dev/2pybot-imx708"], ["I2C-7", "Address 0x29", "VL53L5CX ToF"],
    ["USB", "/dev/ttyUSB0, 460 800", "ESP32"], ["Wi-Fi", "Hotspot 10.42.0.1, channel 6", "Phones and browsers"],
    [":8080", "Console HTTP / WS", "Cockpit, tuning"], [":8889", "MediaMTX WHEP", "Live video"],
    [":6969", "This map", "Also at :8080/sysmap/"],
  ],
};
