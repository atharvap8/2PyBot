#include "m0601.h"
#include "hal/uart_types.h"

namespace m0601 {
uint8_t crc8(const uint8_t* data, size_t n) {
    uint8_t crc = 0;
    for (size_t i = 0; i < n; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) crc = (crc & 1) ? (crc >> 1) ^ 0x8c : crc >> 1;
    }
    return crc;
}
void frameDrive(uint8_t* f, uint8_t id, int16_t value, uint8_t accel, bool brake) {
    memset(f, 0, FRAME_LEN);
    f[0] = id; f[1] = CMD_DRIVE;
    const uint16_t raw = (uint16_t)value;
    f[2] = raw >> 8; f[3] = raw & 0xff;
    f[6] = accel; f[7] = brake ? 0xff : 0;
    f[9] = crc8(f, 9);
}
void frameQuery(uint8_t* f, uint8_t id) {
    memset(f, 0, FRAME_LEN); f[0] = id; f[1] = CMD_QUERY; f[9] = crc8(f, 9);
}
void frameMode(uint8_t* f, uint8_t id, uint8_t mode) {
    memset(f, 0, FRAME_LEN); f[0] = id; f[1] = CMD_MODE;
    f[9] = mode; // Vendor format: mode byte, deliberately NOT CRC.
}
}

M0601Port::M0601Port(HardwareSerial& port, uint8_t id, int8_t rx, int8_t tx, int8_t de, const char* name)
    : serial(port), motorId(id), rxPin(rx), txPin(tx), dePin(de), label(name) {}

void M0601Port::begin() {
    pinMode(dePin, OUTPUT); digitalWrite(dePin, LOW);
    serial.setRxBufferSize(256);
    serial.begin(m0601::BAUD, SERIAL_8N1, rxPin, txPin);
    hardwareDe = serial.setPins(-1, -1, -1, dePin) && serial.setMode(UART_MODE_RS485_HALF_DUPLEX);
    if (!hardwareDe) {
        serial.setMode(UART_MODE_UART);
        pinMode(dePin, OUTPUT); digitalWrite(dePin, LOW);
    }
    while (serial.available()) serial.read();
    length = 0; pending = NONE;
}

bool M0601Port::write(const uint8_t* frame, Pending kind) {
    if (pending != NONE) {
        // Previous reply missing or still in flight. Do NOT stall: a stale
        // torque setpoint is far worse for balance than a stale feedback
        // sample. If a late reply still arrives, its common fields (mode,
        // current, rpm, fault) are valid; only the byte 6/7 layout is unknown
        // when the two outstanding requests differ in kind.
        ++feedback.timeouts;
        if (pending != kind) kind = (kind == NONE) ? NONE : AMBIGUOUS;
    }
    pending = kind; sentUs = micros();
    if (!hardwareDe) digitalWrite(dePin, HIGH);
    serial.write(frame, m0601::FRAME_LEN);
    if (!hardwareDe) { serial.flush(); digitalWrite(dePin, LOW); }
    return true;
}
bool M0601Port::sendDriveRaw(int16_t value, uint8_t accel) {
    uint8_t f[m0601::FRAME_LEN]; m0601::frameDrive(f, motorId, value, accel, false); return write(f, DRIVE);
}
bool M0601Port::sendQuery() {
    uint8_t f[m0601::FRAME_LEN]; m0601::frameQuery(f, motorId); return write(f, QUERY);
}
bool M0601Port::sendMode(uint8_t mode) {
    uint8_t f[m0601::FRAME_LEN]; m0601::frameMode(f, motorId, mode); return write(f, NONE);
}

void M0601Port::decode(const uint8_t* f) {
    feedback.mode = f[1];
    feedback.currentA = m0601::rawToAmps((int16_t)((f[2] << 8) | f[3]));
    feedback.rpm = (int16_t)((f[4] << 8) | f[5]);
    if (pending == QUERY) feedback.tempC = f[6];
    else if (pending == DRIVE) {
        feedback.pos16 = (uint16_t)((f[6] << 8) | f[7]);
        feedback.posValid = true; ++feedback.posFrames;
    } // AMBIGUOUS: DRIVE/QUERY layout unknown, keep common fields only
    feedback.fault = f[8]; feedback.rxMs = millis(); ++feedback.frames;
    pending = NONE;
}

void M0601Port::service() {
    const uint32_t now = micros();
    while (serial.available() && length < sizeof(buffer)) buffer[length++] = (uint8_t)serial.read();
    while (length >= m0601::FRAME_LEN) {
        const bool header = buffer[0] == motorId && buffer[1] >= 1 && buffer[1] <= 3;
        if (header && m0601::crc8(buffer, 9) == buffer[9]) {
            // Unsolicited/duplicate/late frames must not be decoded as the
            // next transaction's layout. Only one request can be outstanding.
            if (pending != NONE) decode(buffer);
            else ++feedback.dropped;
            memmove(buffer, buffer + m0601::FRAME_LEN, length - m0601::FRAME_LEN);
            length -= m0601::FRAME_LEN;
        } else {
            if (header) ++feedback.crcErrors;
            memmove(buffer, buffer + 1, --length); ++feedback.dropped;
        }
    }
    // A reply this old is lost. Forget the request so a stray late frame is
    // not decoded with a guessed layout. Link loss is judged by reply age.
    if (pending != NONE && now - sentUs > m0601::REPLY_TIMEOUT_US) {
        ++feedback.timeouts;
        pending = NONE;
    }
}
