/*
 * ============================================================
 *  espnow_comm.h — ESP-NOW Joystick Communication
 * ============================================================
 *  Shared packet structure between the joystick transmitter and
 *  the self-balancing robot receiver.
 *
 *  Include this header in both the transmitter and receiver
 *  sketches to ensure the packet data structure aligns correctly
 *  in memory across both devices.
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

// Retrieves the 6-byte MAC address associated with a specific logical
// interface on the ESP32 (e.g., Station, SoftAP), returning it as a
// standard formatted string.
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

// Retrieves the absolute hardware factory-default base MAC address
// permanently burned into the ESP32 construct eFuse.
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
// Data payload explicitly structuring the joystick's translated output.
// The packed attribute strips compiler optimization padding margins, 
// guaranteeing absolute byte structure replication across platforms.
typedef struct __attribute__((packed)) {
    float forward;    // The absolute target pitch angle measured in degrees (-5.0 to 5.0).
    float steering;   // The lateral steering differential multiplier (-1.0 to 1.0).
    uint8_t enable;   // Evaluates logical boolean states determining motor activation.
} JoystickPacket;

// ============================================================
//  RECEIVER STATE
// ============================================================
// Volatile directives inform the compiler that these variables are
// altered asynchronously by hardware hardware interrupts (the callbacks)
// and should not be aggressively cached in registers.

volatile float joyForward  = 0.0f;
volatile float joySteering = 0.0f;
volatile uint8_t joyEnable = 0;
volatile unsigned long lastJoyPacketMs = 0;

// Defines the total milliseconds allowed between valid packets.
// If this duration elapses, the receiver safely zeroes mechanical inputs.
#define JOY_TIMEOUT_MS  200

// ============================================================
//  ESP-NOW RECEIVE CALLBACK
// ============================================================
// Invokes autonomously whenever a validated ESP-NOW payload is received
// utilizing the required ESP32 Core 3.x+ (IDF 5.x) function signature.
void onEspNowReceive(const esp_now_recv_info_t *info,
                     const uint8_t *data, int len)
{
    // A rigid length check ensures that malformed packets, or data generated
    // by other unassociated hardware networks, are safely rejected.
    if (len == sizeof(JoystickPacket)) {
        // Cast the bytes into the shared struct representation.
        const JoystickPacket *pkt = (const JoystickPacket *)data;
        
        // Pass the explicit payload properties into volatile memory space.
        joyForward  = pkt->forward;
        joySteering = pkt->steering;
        joyEnable   = pkt->enable;
        
        // Register the timestamp dictating the active connection status.
        lastJoyPacketMs = millis();
    }
}

// ============================================================
//  INITIALISATION
// ============================================================
// Bootstraps and secures the dedicated ESP-NOW networking protocol layer.
// This requires logical initialisation prior to attempting Bluetooth
// connectivity, as both systems aggressively contest identical radio hardware.
inline bool espnow_receiver_begin() {
    
    // Establishing the WiFi Station mode is an absolute requirement 
    // for standard ESP-NOW functionality.
    WiFi.mode(WIFI_STA);
    
    // Provides immediate visual access to the required target MAC 
    // address via the serial terminal upon successful boot.
    Serial.printf("[ESPNOW] Receiver MAC (STA): %s\n", getInterfaceMacAddress(ESP_MAC_WIFI_STA).c_str());

    // Validates that the underlying protocol framework was initialized.
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] Init FAILED");
        return false;
    }

    // Links the predefined handler function definitively into the ESP-NOW core.
    esp_now_register_recv_cb(onEspNowReceive);
    Serial.println("[ESPNOW] Receiver ready — waiting for joystick");
    return true;
}

#endif // ESPNOW_COMM_H
