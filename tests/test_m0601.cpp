#include "m0601.h"
#include <cassert>
#include <algorithm>
using namespace m0601;

static void inject(HardwareSerial& serial,const uint8_t* frame,size_t n=10) {
    for(size_t i=0;i<n;++i) serial.rx.push_back(frame[i]);
}
int main() {
    assert(crc8((const uint8_t*)"123456789",9)==0xa1);
    uint8_t frame[10];
    frameDrive(frame,1,100,0,false);
    const uint8_t golden[]={1,0x64,0,0x64,0,0,0,0,0,0x4f};
    assert(std::equal(frame,frame+10,golden));
    frameDrive(frame,1,-150,0,false); assert(frame[2]==0xff && frame[3]==0x6a && frame[9]==0x5a);
    frameMode(frame,1,MODE_CURRENT); assert(frame[1]==0xa0 && frame[9]==1);
    HardwareSerial serial;
    M0601Port port(serial,1,16,17,4,"LEFT"); port.begin();
    const uint8_t drive[]={1,2,0,0x39,0,0,0x5d,0xf1,0,0x6f};
    const uint8_t query[]={1,2,0,0x37,0,0,0x1e,0xbb,0,0xd5};
    assert(port.sendDriveRaw(0));
    inject(serial,drive,4); port.service(); assert(port.fb().frames==0);
    inject(serial,drive+4,6); port.service();
    assert(port.fb().pos16==0x5df1 && port.fb().posFrames==1);
    assert(port.sendQuery()); inject(serial,query); port.service();
    assert(port.fb().tempC==30 && port.fb().pos16==0x5df1 && port.fb().posFrames==1);
    uint8_t bad[10]; std::copy(drive,drive+10,bad); bad[9]^=1;
    assert(port.sendDriveRaw(0)); inject(serial,bad); port.service();
    assert(port.fb().frames==2 && port.fb().crcErrors==1);
    // v4.0.3: a missing reply never blocks the next frame (no torque freeze).
    assert(port.sendDriveRaw(0));
    assert(port.fb().timeouts==1);                       // previous DRIVE reply counted lost
    inject(serial,drive); port.service();               // reply to the newest DRIVE
    assert(port.fb().posFrames==2 && port.fb().mode==2);
    // Request older than the timeout is forgotten; a stray late frame is dropped.
    assert(port.sendDriveRaw(0)); testMicros+=9000; port.service();
    assert(port.fb().timeouts==2);
    inject(serial,drive); port.service(); assert(port.fb().posFrames==2);
    // DRIVE outstanding, QUERY sent: a late reply has an unknown byte-6/7 layout.
    // Common fields (mode/fault/link) update; position and temperature do not.
    const uint32_t framesBefore=port.fb().frames;
    assert(port.sendDriveRaw(0)); assert(port.sendQuery());
    inject(serial,query); port.service();
    assert(port.fb().frames==framesBefore+1 && port.fb().posFrames==2 && port.fb().pos16==0x5df1);
    // Reply already queued when the CPU was late: parsed before any timeout.
    assert(port.sendDriveRaw(0));inject(serial,drive);testMicros+=9000;port.service();
    assert(port.fb().posFrames==3 && port.fb().mode==2 && port.fb().timeouts==3);
    puts("M0601: golden frames, split RX, CRC, layout ownership, never-blocking send and layout ambiguity passed");
}
