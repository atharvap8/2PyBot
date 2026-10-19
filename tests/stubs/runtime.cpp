#include "Arduino.h"
#include "Preferences.h"
uint32_t testMicros=1000;
HardwareSerial Serial,Serial1,Serial2;
std::map<std::string,float> testNvs;
