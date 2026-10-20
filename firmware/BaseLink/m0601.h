#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

// M0601: 115200 8N1, ten-byte frames. Current raw +/-32767 = +/-8 A.
// DRIVE replies contain pos16; QUERY replies contain temperature + pos8.
namespace m0601 {
constexpr uint32_t BAUD = 115200;
constexpr uint8_t FRAME_LEN = 10, CMD_DRIVE = 0x64, CMD_QUERY = 0x74, CMD_MODE = 0xa0;
enum Mode : uint8_t { MODE_CURRENT = 1, MODE_VELOCITY = 2, MODE_POSITION = 3 };
constexpr float POS_PER_REV = 32767.0f, CURRENT_FULL_A = 8.0f;
constexpr int16_t RPM_CMD_MAX = 330;
constexpr uint32_t REPLY_TIMEOUT_US = 8000, RESYNC_US = 10000;
uint8_t crc8(const uint8_t* data, size_t n);
void frameDrive(uint8_t* f, uint8_t id, int16_t value, uint8_t accel, bool brake);
void frameQuery(uint8_t* f, uint8_t id);
void frameMode(uint8_t* f, uint8_t id, uint8_t mode);
inline int16_t ampsToRaw(float a) {
    float raw = a * (32767.0f / CURRENT_FULL_A);
    return (int16_t)lroundf(fmaxf(-32767, fminf(32767, raw)));
}
inline float rawToAmps(int16_t raw) { return raw * (CURRENT_FULL_A / 32767.0f); }
}

struct M0601Feedback {
    uint8_t mode = 0, fault = 0;
    float currentA = 0;
    int16_t rpm = 0, tempC = -1;
    uint16_t pos16 = 0;
    bool posValid = false;
    uint32_t rxMs = 0, frames = 0, posFrames = 0, crcErrors = 0, dropped = 0, timeouts = 0;
};

class M0601Port {
public:
    M0601Port(HardwareSerial& serial, uint8_t id, int8_t rx, int8_t tx, int8_t de, const char* name);
    void begin();
    void service();
    bool canSend() const;
    bool needsModeConfirmation() const { return reconfirmMode; }
    bool sendDriveRaw(int16_t value, uint8_t accel = 0);
    bool sendQuery();
    bool sendMode(uint8_t mode);
    const M0601Feedback& fb() const { return feedback; }
    const char* name() const { return label; }
    uint8_t id() const { return motorId; }
    bool hwDe() const { return hardwareDe; }

private:
    enum Pending : uint8_t { NONE, DRIVE, QUERY };
    bool write(const uint8_t* frame, Pending kind);
    void decode(const uint8_t* frame);
    HardwareSerial& serial;
    uint8_t motorId;
    int8_t rxPin, txPin, dePin;
    const char* label;
    bool hardwareDe = false, resync = false;
    bool reconfirmMode = true;
    Pending pending = NONE;
    uint32_t sentUs = 0, quietSinceUs = 0;
    uint8_t buffer[40] = {}, length = 0;
    M0601Feedback feedback;
};
