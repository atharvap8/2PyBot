/*
 * ============================================================
 *  espnow_comm.h — ESP-NOW Joystick Communication
 * ============================================================
 *  Defines the shared packet structure and receiver logic for
 *  the ESP-NOW joystick link between the controller and the
 *  self-balancing robot.
 *
 *  Include this header in both the transmitter and receiver
 *  sketches to guarantee identical memory layout.
 * ============================================================
 */

#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <esp_now.h>
#include <WiFi.h>
#include "esp_mac.h"

// ============================================================
//  MAC ADDRESS HELPERS
// ============================================================

// Returns the MAC address of the specified interface as a
// formatted hex string (e.g., "AA:BB:CC:DD:EE:FF").
inline String getInterfaceMacAddress(esp_mac_type_t interface) {
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

// Returns the factory-burned eFuse base MAC address.
inline String getDefaultMacAddress() {
    String mac = "";
    unsigned char mac_base[6] = {0};
    if (esp_efuse_mac_get_default(mac_base) == ESP_OK) {
        char buffer[18];
        sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X",
                mac_base[0], mac_base[1], mac_base[2],
                mac_base[3], mac_base[4], mac_base[5]);
        mac = buffer;
    }
    return mac;
}

// ============================================================
//  PACKET STRUCTURE
// ============================================================
// __attribute__((packed)) prevents compiler padding so the struct
// layout is identical on both the transmitter and receiver.
typedef struct __attribute__((packed)) {
    float   forward;   // Target pitch offset (degrees, -5.0 to +5.0).
    float   steering;  // Lateral steering multiplier (-1.0 to +1.0).
    uint8_t enable;    // Motor enable flag (0 = off, 1 = on).
} JoystickPacket;

// ============================================================
//  RECEIVER STATE
// ============================================================
// volatile: these are written by the ESP-NOW receive callback
// (interrupt context) and read by the main loop.
volatile float         joyForward      = 0.0f;
volatile float         joySteering     = 0.0f;
volatile uint8_t       joyEnable       = 0;
volatile unsigned long lastJoyPacketMs = 0;

// If no packet arrives within this window, inputs are zeroed.
#define JOY_TIMEOUT_MS  200

// ============================================================
//  ESP-NOW RECEIVE CALLBACK
// ============================================================
// Signature required by ESP32 Arduino Core 3.x / IDF 5.x.
void onEspNowReceive(const esp_now_recv_info_t *info,
                     const uint8_t *data, int len)
{
    // Reject packets with incorrect size (noise or foreign devices).
    if (len == sizeof(JoystickPacket)) {
        const JoystickPacket *pkt = (const JoystickPacket *)data;
        joyForward      = pkt->forward;
        joySteering     = pkt->steering;
        joyEnable       = pkt->enable;
        lastJoyPacketMs = millis();
    }
}

// ============================================================
//  INITIALISATION
// ============================================================
// Must be called before Bluetooth init, as both share the radio.
inline bool espnow_receiver_begin() {

    WiFi.mode(WIFI_STA);
    Serial.printf("[ESPNOW] Receiver MAC (STA): %s\n",
                  getInterfaceMacAddress(ESP_MAC_WIFI_STA).c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] Init FAILED");
        return false;
    }

    esp_now_register_recv_cb(onEspNowReceive);
    Serial.println("[ESPNOW] Receiver ready — waiting for joystick");
    return true;
}

#endif // ESPNOW_COMM_H
