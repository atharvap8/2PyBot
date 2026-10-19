#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdarg>
#include <deque>
#include <vector>
#include <string>
#include <initializer_list>
#define PI 3.14159265358979323846
#define SERIAL_8N1 0
#define OUTPUT 1
#define LOW 0
#define HIGH 1
extern uint32_t testMicros;
inline uint32_t micros() { return testMicros; }
inline uint32_t millis() { return testMicros / 1000; }
inline void delay(uint32_t ms) { testMicros += ms * 1000; }
inline void pinMode(int,int) {}
inline void digitalWrite(int,int) {}
struct HardwareSerial {
    std::deque<uint8_t> rx;
    std::vector<uint8_t> tx;
    std::string output;
    int capacity = 4096;
    void setRxBufferSize(int) {}
    void begin(uint32_t,int,int,int) {}
    bool setPins(int,int,int,int) { return true; }
    bool setMode(int) { return true; }
    int available() { return (int)rx.size(); }
    int read() { int c=rx.front(); rx.pop_front(); return c; }
    int availableForWrite() { return capacity; }
    size_t write(const uint8_t* data,size_t n) {
        tx.insert(tx.end(),data,data+n); output.append((const char*)data,n); return n;
    }
    void flush() {}
    template<class... Args> void printf(const char* format,Args... args) {
        char text[2048]; int n=snprintf(text,sizeof(text),format,args...);
        output.append(text,(size_t)n);
    }
    void println(const char* text) { output+=text; output+='\n'; }
};
extern HardwareSerial Serial,Serial1,Serial2;
