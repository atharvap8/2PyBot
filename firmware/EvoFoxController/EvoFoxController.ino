/*
 * ============================================================
 *  EvoFoxController.ino — EvoFox One S Gamepad Bridge
 * ============================================================
 *  Pairs with an EvoFox One S gamepad over Bluetooth (Bluepad32)
 *  and forwards drive commands to the self-balancing robot via
 *  ESP-NOW using the same JoystickPacket as the analog joystick
 *  transmitter — no firmware change needed on the robot.
 *
 *  Hardware:
 *    - ESP32 DevKit (classic ESP32; BT Classic + WiFi coexist)
 *    - EvoFox One S gamepad in Bluetooth (X-Input) mode
 *
 *  Library:
 *    - Bluepad32 for Arduino
 *      Install: Boards Manager URL
 *      https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
 *      then select board "ESP32 Dev Module" under the
 *      "esp32_bluepad32" platform and include <Bluepad32.h>.
 *
 *  Pairing the EvoFox One S:
 *    1. Switch the controller to Bluetooth mode (mode switch /
 *       combo per manual, usually HOME + Y until LED double-blinks).
 *    2. Power this ESP32 — Bluepad32 auto-scans and pairs.
 *    3. Status LED goes solid when a gamepad connects.
 *
 *  Controls:
 *    Left stick Y   -> forward/backward lean
 *    Right stick X  -> steering
 *    A (South)      -> toggle motor enable
 *    B (East)       -> instant disable (kill)
 *    Deadzones and expo applied for smooth control.
 *
 *  Author: Atharva
 *  Date:   July 2026
 * ============================================================
 */

#include <Bluepad32.h>
#include <esp_now.h>
#include <WiFi.h>
#include "esp_mac.h"
#include "config.h"

// ============================================================
//  PACKET STRUCTURE (must match espnow_comm.h on the robot)
// ============================================================
typedef struct __attribute__((packed)) {
    float   forward;   // Target pitch offset (degrees).
    float   steering;  // Steering multiplier (-1.0 to +1.0).
    uint8_t enable;    // Motor enable flag (0 = off, 1 = on).
} JoystickPacket;

// ============================================================
//  GLOBAL STATE
// ============================================================
static ControllerPtr gamepad = nullptr;

static bool  peerAdded   = false;
static bool  motorEnable = false;
static bool  lastBtnA    = false;

static float smoothFwd   = 0.0f;
static float smoothSteer = 0.0f;

static unsigned long lastSendMs = 0;

// ============================================================
//  HELPERS
// ============================================================
static String macToString(const uint8_t *mac) {
    char buffer[18];
    sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buffer);
}

// Deadzone + rescale so remaining range maps smoothly to 0..1.
static float applyDeadzone(float raw) {
    if (fabsf(raw) < STICK_DEADZONE) return 0.0f;
    float sign = (raw > 0.0f) ? 1.0f : -1.0f;
    return sign * (fabsf(raw) - STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
}

// Cubic expo blend: fine control near center, full range at edges.
static float applyExpo(float x) {
    return (1.0f - STICK_EXPO) * x + STICK_EXPO * x * x * x;
}

// ============================================================
//  BLUEPAD32 CALLBACKS
// ============================================================
static void onConnectedController(ControllerPtr ctl) {
    if (gamepad == nullptr) {
        gamepad = ctl;
        digitalWrite(STATUS_LED, HIGH);
        Serial.printf("[GP] Gamepad connected: %s\n",
                      ctl->getModelName().c_str());
    } else {
        Serial.println("[GP] Extra gamepad rejected (one already active)");
        ctl->disconnect();
    }
}

static void onDisconnectedController(ControllerPtr ctl) {
    if (gamepad == ctl) {
        gamepad = nullptr;
        motorEnable = false;   // Safety: kill motors on link loss.
        digitalWrite(STATUS_LED, LOW);
        Serial.println("[GP] Gamepad disconnected — motors disabled");
    }
}

// ============================================================
//  ESP-NOW SEND CALLBACK
// ============================================================
static void onSendComplete(const wifi_tx_info_t *info,
                           esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS && gamepad == nullptr) {
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  EVOFOX ONE S BRIDGE  —  ESP-NOW");
    Serial.println("========================================");

    // ---- ESP-NOW (WiFi STA) ----
    WiFi.mode(WIFI_STA);
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    Serial.printf("[TX] This device MAC: %s\n", macToString(mac).c_str());
    Serial.printf("[TX] Target receiver:  %s\n",
                  macToString(receiverMAC).c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[TX] ESP-NOW init FAILED — halting");
        while (true) { delay(1000); }
    }
    esp_now_register_send_cb(onSendComplete);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, receiverMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        peerAdded = true;
        Serial.println("[TX] Peer registered");
    } else {
        Serial.println("[TX] Peer registration FAILED");
    }

    // ---- Bluepad32 (Bluetooth) ----
    BP32.setup(&onConnectedController, &onDisconnectedController);
    // Do not auto-delete stored keys: keeps EvoFox paired across boots.
    // Uncomment to force re-pairing:
    // BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    Serial.println("[GP] Scanning for EvoFox One S (Bluetooth mode)...");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
    BP32.update();

    if (millis() - lastSendMs < SEND_INTERVAL) return;
    lastSendMs = millis();

    float fwd = 0.0f, steer = 0.0f;

    if (gamepad && gamepad->isConnected() && gamepad->isGamepad()) {
        // Bluepad32 axes: -512..511. Normalize to -1..1.
        float rawFwd   = -gamepad->axisY()  / 512.0f;  // Up = forward.
        float rawSteer =  gamepad->axisRX() / 512.0f;  // Right stick X.

        rawFwd   = constrain(rawFwd,   -1.0f, 1.0f);
        rawSteer = constrain(rawSteer, -1.0f, 1.0f);

        fwd   = applyExpo(applyDeadzone(rawFwd));
        steer = applyExpo(applyDeadzone(rawSteer));

        // A toggles enable (edge-triggered), B is an instant kill.
        bool btnA = gamepad->a();
        if (btnA && !lastBtnA) {
            motorEnable = !motorEnable;
            Serial.printf("[GP] Motors %s\n",
                          motorEnable ? "ENABLED" : "DISABLED");
        }
        lastBtnA = btnA;

        if (gamepad->b()) {
            motorEnable = false;
        }

        // Rumble feedback on enable state change is available:
        // gamepad->playDualRumble(0, 100, 0x40, 0x40);
    } else {
        // No gamepad: send zeroes so the robot's watchdog stays calm
        // but never commands motion.
        motorEnable = false;
        lastBtnA = false;
    }

    // Exponential smoothing to soften stick steps.
    smoothFwd   = STICK_SMOOTH * smoothFwd   + (1.0f - STICK_SMOOTH) * fwd;
    smoothSteer = STICK_SMOOTH * smoothSteer + (1.0f - STICK_SMOOTH) * steer;

    JoystickPacket pkt;
    pkt.forward  = smoothFwd * MAX_TARGET_ANGLE;
    pkt.steering = smoothSteer * MAX_STEERING;
    pkt.enable   = motorEnable ? 1 : 0;

    if (peerAdded) {
        esp_now_send(receiverMAC, (const uint8_t *)&pkt, sizeof(pkt));
    }
}
