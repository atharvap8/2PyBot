/*
 * ============================================================
 *  Controller.ino — ESP-NOW Joystick Transmitter
 * ============================================================
 *  Reads a 2-axis analog joystick and sends drive commands
 *  to the self-balancing robot via ESP-NOW at 50 Hz.
 *
 *  Hardware:
 *    - ESP32 DevKit (any variant)
 *    - 2-axis analog joystick module (VRx, VRy, SW)
 *
 *  Wiring:
 *    VRx -> GPIO 34  (ADC1 — required, ADC2 unavailable with WiFi)
 *    VRy -> GPIO 35
 *    SW  -> GPIO 32  (internal pull-up, active low)
 *    +5V -> 3.3V
 *    GND -> GND
 *
 *  Author: Atharva
 *  Date:   April 2026
 * ============================================================
 */

#include <esp_now.h>
#include <WiFi.h>
#include "esp_mac.h"

// ============================================================
//  RECEIVER MAC ADDRESS
// ============================================================
// Must match the STA MAC printed by the robot on startup.
uint8_t receiverMAC[] = {0xEC, 0x62, 0x60, 0x99, 0x97, 0xE0};

// Returns the formatted MAC address string for a given interface.
String getInterfaceMacAddress(esp_mac_type_t interface) {
    String mac = "";
    unsigned char mac_base[6] = {0};
    if (esp_read_mac(mac_base, interface) == ESP_OK) {
        char buffer[18];
        sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X",
                mac_base[0], mac_base[1], mac_base[2],
                mac_base[3], mac_base[4], mac_base[5]);
        mac = buffer;
    }
    return mac;
}

// ============================================================
//  JOYSTICK CONFIGURATION
// ============================================================
#define JOY_X_PIN       34      // Steering axis (left/right).
#define JOY_Y_PIN       35      // Drive axis (forward/backward).
#define JOY_BTN_PIN     32      // Pushbutton (active low, internal pull-up).
#define STATUS_LED       2

#define MAX_TARGET_ANGLE 5.0f   // Maximum lean command (degrees).
#define MAX_STEERING     1.0f   // Maximum steering multiplier (normalized).

// Resting-position offsets populated during boot calibration.
int joyCenterX = 2048;
int joyCenterY = 2048;

#define JOY_RANGE       2048    // Half-range of a 12-bit ADC (0–4095).
#define DEADZONE        0.10f   // Fractional center deadzone.
#define JOY_SMOOTH      0.7f    // EMA coefficient for ADC smoothing.

// ============================================================
//  TRANSMIT RATE
// ============================================================
#define SEND_RATE_HZ    50
#define SEND_INTERVAL   (1000 / SEND_RATE_HZ)

// ============================================================
//  PACKET STRUCTURE
// ============================================================
// __attribute__((packed)) ensures identical memory layout on
// both transmitter and receiver regardless of compiler settings.
typedef struct __attribute__((packed)) {
    float   forward;   // Target pitch offset (degrees).
    float   steering;  // Steering multiplier (-1.0 to +1.0).
    uint8_t enable;    // Motor enable flag (0 = off, 1 = on).
} JoystickPacket;

// ============================================================
//  GLOBAL STATE
// ============================================================
bool  peerAdded = false;
bool  enabled   = false;
bool  lastBtn   = false;
float smoothX   = 0.0f;
float smoothY   = 0.0f;

// ============================================================
//  ESP-NOW SEND CALLBACK
// ============================================================
// Signature required by ESP32 Arduino Core 3.x / IDF 5.x.
// Blinks the status LED on a failed transmission.
void onSendComplete(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    }
}

// ============================================================
//  applyDeadzone()
// ============================================================
// Ignores inputs below the deadzone threshold and rescales the
// remaining active range to fill 0.0–1.0 smoothly.
float applyDeadzone(float raw) {
    if (fabsf(raw) < DEADZONE) return 0.0f;
    float sign = (raw > 0.0f) ? 1.0f : -1.0f;
    return sign * (fabsf(raw) - DEADZONE) / (1.0f - DEADZONE);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(JOY_BTN_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED, OUTPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // Full 3.3V range on ADC inputs.

    Serial.println();
    Serial.println("========================================");
    Serial.println("  JOYSTICK TRANSMITTER  —  ESP-NOW");
    Serial.println("========================================");

    // Boot calibration: average 64 samples to find the resting center.
    long sumX = 0, sumY = 0;
    const int CAL_N = 64;
    Serial.print("[TX] Calibrating center... ");
    for (int i = 0; i < CAL_N; i++) {
        sumX += analogRead(JOY_X_PIN);
        sumY += analogRead(JOY_Y_PIN);
        delay(5);
    }
    joyCenterX = (int)(sumX / CAL_N);
    joyCenterY = (int)(sumY / CAL_N);
    Serial.printf("X=%d  Y=%d\n", joyCenterX, joyCenterY);

    WiFi.mode(WIFI_STA);
    Serial.printf("[TX] This device MAC: %s\n", getInterfaceMacAddress(ESP_MAC_WIFI_STA).c_str());
    Serial.printf("[TX] Target receiver:  %02X:%02X:%02X:%02X:%02X:%02X\n",
                  receiverMAC[0], receiverMAC[1], receiverMAC[2],
                  receiverMAC[3], receiverMAC[4], receiverMAC[5]);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[TX] ESP-NOW init FAILED — halting");
        while (true) delay(1000);
    }
    esp_now_register_send_cb(onSendComplete);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, receiverMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) == ESP_OK) {
        peerAdded = true;
        Serial.println("[TX] Peer added OK");
    } else {
        Serial.println("[TX] Failed to add peer!");
    }

    Serial.println("[TX] Ready — press joystick button to toggle enable");
    Serial.println();
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
    static unsigned long lastSendMs  = 0;
    static unsigned long lastPrintMs = 0;

    unsigned long now = millis();
    if (now - lastSendMs < SEND_INTERVAL) return;
    lastSendMs = now;

    // 4-sample oversample to reduce ADC noise.
    const int N = 4;
    int sumX = 0, sumY = 0;
    for (int i = 0; i < N; i++) {
        sumX += analogRead(JOY_X_PIN);
        sumY += analogRead(JOY_Y_PIN);
    }

    // Remove calibration offset and normalize to ±1.0.
    float rawX = (float)(sumX / N - joyCenterX) / JOY_RANGE;
    float rawY = (float)(sumY / N - joyCenterY) / JOY_RANGE;

    // EMA smoothing.
    smoothX += JOY_SMOOTH * (rawX - smoothX);
    smoothY += JOY_SMOOTH * (rawY - smoothY);

    float normalizedX = constrain(applyDeadzone(smoothX), -1.0f, 1.0f);
    float normalizedY = constrain(applyDeadzone(smoothY), -1.0f, 1.0f);

    float fx = normalizedX * MAX_STEERING;
    float fy = normalizedY * MAX_TARGET_ANGLE;

    // Rising-edge toggle for motor enable button.
    bool btn = !digitalRead(JOY_BTN_PIN);
    if (btn && !lastBtn) {
        enabled = !enabled;
        digitalWrite(STATUS_LED, enabled ? HIGH : LOW);
        Serial.printf("[TX] Motors %s\n", enabled ? "ENABLED" : "DISABLED");
    }
    lastBtn = btn;

    JoystickPacket pkt;
    pkt.forward  = fy;
    pkt.steering = fx;
    pkt.enable   = enabled ? 1 : 0;

    if (peerAdded) {
        esp_now_send(receiverMAC, (uint8_t *)&pkt, sizeof(pkt));
    }

    if (now - lastPrintMs >= 200) {
        lastPrintMs = now;
        Serial.printf("[TX] F=%+.2f  S=%+.2f  EN=%d\n",
                      pkt.forward, pkt.steering, pkt.enable);
    }
}
