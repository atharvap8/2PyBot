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
// A reply normally lands ~2 ms after its request. Anything older than this
// when the next frame is due is counted as missed; it never blocks sending.
constexpr uint32_t REPLY_TIMEOUT_US = 8000;
// Documented fault bits: 0x01 sensor, 0x02 bus overcurrent, 0x04 phase
// overcurrent, 0x08 stall, 0x10 overheat. Any other bit means the "frame" is a
// misaligned/corrupt byte window that happened to pass the 8-bit CRC (1/256).
constexpr uint8_t FAULT_MASK = 0x1F;
// A fault must repeat in this many consecutive valid replies (~7.5 ms at the
// 400 Hz loop) before it counts. One corrupt frame must not drop the robot.
constexpr uint8_t FAULT_CONFIRM = 3;
// No M0601 reply can exceed this (no-load ~330 rpm); larger is corruption.
constexpr int16_t RPM_PLAUSIBLE = 900;
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
    uint8_t mode = 0, fault = 0;          // fault: confirmed (FAULT_CONFIRM consecutive replies)
    uint8_t rawFault = 0, lastTransient = 0;
    float currentA = 0;
    int16_t rpm = 0, tempC = -1;
    uint16_t pos16 = 0;
    bool posValid = false;
    uint32_t rxMs = 0, frames = 0, posFrames = 0, crcErrors = 0, dropped = 0, timeouts = 0;
    uint32_t implausible = 0;             // CRC passed but content impossible (rejected)
    uint32_t transientFaults = 0;         // fault byte seen but not confirmed (ignored)
    uint8_t lastFrame[10] = {};           // last decoded reply, printed with fault reports
};

class M0601Port {
public:
    M0601Port(HardwareSerial& serial, uint8_t id, int8_t rx, int8_t tx, int8_t de, const char* name);
    void begin();
    void service();
    // v4.0.3: the port never refuses to send. One frame per cycle goes out
    // regardless of whether the previous reply arrived (the supplied
    // BaseLinkHub behaviour that balanced on this hardware). A missed reply
    // only costs feedback freshness; link loss is judged by reply age.
    bool sendDriveRaw(int16_t value, uint8_t accel = 0);
    bool sendQuery();
    bool sendMode(uint8_t mode);
    const M0601Feedback& fb() const { return feedback; }
    const char* name() const { return label; }
    uint8_t id() const { return motorId; }
    bool hwDe() const { return hardwareDe; }

private:
    enum Pending : uint8_t { NONE, DRIVE, QUERY, AMBIGUOUS };
    bool write(const uint8_t* frame, Pending kind);
    void decode(const uint8_t* frame);
    HardwareSerial& serial;
    uint8_t motorId;
    int8_t rxPin, txPin, dePin;
    const char* label;
    bool hardwareDe = false;
    Pending pending = NONE;
    uint32_t sentUs = 0;
    uint8_t buffer[40] = {}, length = 0;
    uint8_t faultCandidate = 0, faultCount = 0, modeCandidate = 0;
    static bool plausible(const uint8_t* frame);
    M0601Feedback feedback;
};
