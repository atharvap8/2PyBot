#pragma once
#include <map>
#include <string>
extern std::map<std::string,float> testNvs;
struct Preferences {
    std::string ns;
    bool begin(const char* name,bool) { ns=name; return true; }
    float getFloat(const char* key,float fallback) {
        auto found=testNvs.find(ns+":"+key); return found==testNvs.end()?fallback:found->second;
    }
    unsigned char getUChar(const char* key,unsigned char fallback) { return (unsigned char)getFloat(key,fallback); }
    void putFloat(const char* key,float value) { testNvs[ns+":"+key]=value; }
    void putUChar(const char* key,unsigned char value) { putFloat(key,value); }
    void end() {}
};
