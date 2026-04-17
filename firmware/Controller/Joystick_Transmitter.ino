/*
 * ============================================================
 *  Joystick_Transmitter.ino
 * ============================================================
 *  ESP32 joystick controller for the Self-Balancing Robot.
 *  Reads a 2-axis analog joystick and sends drive commands
 *  via ESP-NOW to the robot receiver at 50 Hz.
 *
 *  Hardware:
 *    - ESP32 DevKit (any variant).
 *    - 2-axis analog joystick module (VRx, VRy, SW).
 *
 *  Wiring:
 *    VRx  -> GPIO 34  (ADC1_CH6).
 *    VRy  -> GPIO 35  (ADC1_CH7).
 *    SW   -> GPIO 32  (internal pullup).
 *    +5V  -> 3.3V.
 *    GND  -> GND.
 *
 *  Note: ADC1 pins (GPIO 32-39) are used because ADC2 is 
 *        unavailable when the WiFi radio is active.
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
// This array defines the hardcoded MAC address of the receiver robot.
// Ensure this matches the output printed by the robot during startup.
uint8_t receiverMAC[] = {0xEC, 0x62, 0x60, 0x99, 0x97, 0xE0};

// ============================================================
//  MAC ADDRESS HELPER
// ============================================================
// Retrieves and formats the specified interface MAC address into a string.
// This is primarily used for logging the transmitter's own MAC address to the terminal.
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
// Pin definitions for the joystick module.
#define JOY_X_PIN       34      // Analog input for steering axis (left/right).
#define JOY_Y_PIN       35      // Analog input for drive axis (forward/backward).
#define JOY_BTN_PIN     32      // Digital input for joystick pushbutton (active low).
#define STATUS_LED       2      // Digital output for the onboard status LED.

// Mathematical bounds for mapping the joystick to actual robot limits.
#define MAX_TARGET_ANGLE 5.0f   // Maximum forward/backward lean angle command in degrees.
#define MAX_STEERING     1.0f   // Maximum steering differential multiplier (normalized -1.0 to 1.0).

// Joystick center offsets determined during the auto-calibration phase.
int joyCenterX = 2048;
int joyCenterY = 2048;

// ADC parsing configurations.
#define JOY_RANGE       2048    // Assumed half-range of a 12-bit ADC (resolves 0 to 4095).
#define DEADZONE        0.10f   // Fractional threshold below which mechanical stick drift is ignored.
#define JOY_SMOOTH      0.7f    // Exponential Moving Average filter coefficient to mitigate ADC noise.

// ============================================================
//  TRANSMIT RATE
// ============================================================
// Packet transmission scheduling parameters.
#define SEND_RATE_HZ    50
#define SEND_INTERVAL   (1000 / SEND_RATE_HZ)

// ============================================================
//  PACKET STRUCTURE
// ============================================================
// Data payload definition. Packed attribute guarantees memory alignment 
// consistency across different compilers and architectures.
typedef struct __attribute__((packed)) {
    float forward;    // Maps directly to the target angle (degrees).
    float steering;   // Steering multiplier (normalized).
    uint8_t enable;   // Boolean flag indicating if motors should be active.
} JoystickPacket;

// ============================================================
//  GLOBAL STATE
// ============================================================
bool peerAdded  = false; // Tracks if the receiver peer was successfully registered.
bool enabled    = false; // Tracks the current toggled state of the motor enable flag.
bool lastBtn    = false; // Stores the previous frame's button state for edge detection.
float smoothX   = 0.0f;  // Persistent history for the X-axis EMA filter.
float smoothY   = 0.0f;  // Persistent history for the Y-axis EMA filter.

// ============================================================
//  ESP-NOW SEND CALLBACK (3.x signature)
// ============================================================
// Triggered asynchronously after a transmit attempt. 
// Uses the wifi_tx_info_t signature native to ESP32 Core 3.x (IDF 5.5).
void onSendComplete(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    // Invert the status LED briefly to signify a failed transmission.
    if (status != ESP_NOW_SEND_SUCCESS) {
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    }
}

// ============================================================
//  HELPER: Apply deadzone and rescale
// ============================================================
// Remaps a raw normalized value (-1.0 to 1.0) to ignore the central deadzone,
// while expanding the remaining active region smoothly out to the limits.
float applyDeadzone(float raw) {
    // Return zero if inside the mechanical center noise threshold.
    if (fabsf(raw) < DEADZONE) {
        return 0.0f;
    }
    // Extract the directional sign of the input.
    float sign = (raw > 0.0f) ? 1.0f : -1.0f;
    
    // Scale the useful input range proportionally.
    return sign * (fabsf(raw) - DEADZONE) / (1.0f - DEADZONE);
}

// ============================================================
//  SETUP ROUTINE
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    // Initialize hardware pins.
    pinMode(JOY_BTN_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED, OUTPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // Extends ADC read range to 3.3V full scale.

    Serial.println();
    Serial.println("========================================");
    Serial.println("  JOYSTICK TRANSMITTER  —  ESP-NOW");
    Serial.println("========================================");

    // Run an initial calibration cycle to record the resting position of the joystick.
    // This removes mechanical bias caused by off-center potentiometers.
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

    // Initialize the WiFi radio in Station Mode, a requirement for ESP-NOW.
    WiFi.mode(WIFI_STA);
    Serial.printf("[TX] This device MAC: %s\n", getInterfaceMacAddress(ESP_MAC_WIFI_STA).c_str());
    Serial.printf("[TX] Target receiver:  %02X:%02X:%02X:%02X:%02X:%02X\n",
                  receiverMAC[0], receiverMAC[1], receiverMAC[2],
                  receiverMAC[3], receiverMAC[4], receiverMAC[5]);

    // Bootstrap the ESP-NOW networking subsystem.
    if (esp_now_init() != ESP_OK) {
        Serial.println("[TX] ESP-NOW init FAILED — halting");
        while (true) delay(1000);
    }
    esp_now_register_send_cb(onSendComplete);

    // Register the robot receiver as an authorized peer node.
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
    
    // Throttle the loop execution to adhere to the designated transmission rate.
    if (now - lastSendMs < SEND_INTERVAL) return;
    lastSendMs = now;

    // Perform multi-sampling to mitigate instantaneous ADC noise spikes.
    const int N = 4;
    int sumX = 0, sumY = 0;
    for (int i = 0; i < N; i++) {
        sumX += analogRead(JOY_X_PIN);
        sumY += analogRead(JOY_Y_PIN);
    }
    
    // Remove the calibration offset to center the data around zero.
    float rawX = (float)(sumX / N - joyCenterX) / JOY_RANGE;
    float rawY = (float)(sumY / N - joyCenterY) / JOY_RANGE;

    // Apply the Exponential Moving Average filter to smooth structural jitters.
    smoothX += JOY_SMOOTH * (rawX - smoothX);
    smoothY += JOY_SMOOTH * (rawY - smoothY);

    // Apply the deadzone algorithm and clamp values to ensure strict normalization.
    float normalizedX = constrain(applyDeadzone(smoothX), -1.0f, 1.0f);
    float normalizedY = constrain(applyDeadzone(smoothY), -1.0f, 1.0f);

    // Scale normalized values up to the absolute transmission limitations.
    float fx = normalizedX * MAX_STEERING;
    float fy = normalizedY * MAX_TARGET_ANGLE;

    // Handle button toggle operations using a simple edge detection check.
    bool btn = !digitalRead(JOY_BTN_PIN);  // Active-low logic inversion.
    if (btn && !lastBtn) {
        enabled = !enabled;
        digitalWrite(STATUS_LED, enabled ? HIGH : LOW);
        Serial.printf("[TX] Motors %s\n", enabled ? "ENABLED" : "DISABLED");
    }
    lastBtn = btn;

    // Compile the packet structure with the filtered, processed values.
    JoystickPacket pkt;
    pkt.forward  = fy;
    pkt.steering = fx;
    pkt.enable   = enabled ? 1 : 0;

    // Dispatch the payload explicitly to the configured target node.
    if (peerAdded) {
        esp_now_send(receiverMAC, (uint8_t *)&pkt, sizeof(pkt));
    }

    // Refresh the local telemetry monitor occasionally for debugging over Serial.
    if (now - lastPrintMs >= 200) {
        lastPrintMs = now;
        Serial.printf("[TX] F=%+.2f  S=%+.2f  EN=%d\n",
                      pkt.forward, pkt.steering, pkt.enable);
    }
}
