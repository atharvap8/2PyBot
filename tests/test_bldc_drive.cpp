#include "hub_drive.h"
#include <cassert>

bool params_motionActive() { return hub.isEnabled(); }
void params_applyHardware(bool) { hub.applyTuning(); }
static int queries=0;
static void reply(HardwareSerial& serial,uint8_t id) {
    if(serial.tx.empty())return;
    const size_t start=serial.tx.size()-10;
    uint8_t command=serial.tx[start+1];
    if(command==m0601::CMD_MODE) {serial.tx.clear();return;}
    uint8_t f[10]={id,m0601::MODE_CURRENT,0,0,0,0,0,0,0,0};
    if(command==m0601::CMD_QUERY) {++queries;f[6]=25;}
    f[9]=m0601::crc8(f,9);
    for(auto b:f)serial.rx.push_back(b);
}
static int16_t wireCurrent(const HardwareSerial& serial) {
    size_t p=serial.tx.size()-10;
    assert(serial.tx[p+1]==m0601::CMD_DRIVE);
    return (int16_t)((serial.tx[p+2]<<8)|serial.tx[p+3]);
}
static void cycle() {
    Serial1.tx.clear();Serial2.tx.clear();
    hub.transmit(.005f);
    reply(Serial1,1);reply(Serial2,2);
    testMicros+=5000;hub.service(.005f);
}
int main() {
    params_begin();hub.applyTuning();
    for(int i=0;i<8;++i)cycle();
    assert(hub.modeOk() && hub.linkOk(60));
    hub.enable();hub.setSpeeds(.1f*P.countsPerM,.1f*P.countsPerM);cycle();
    assert(hub.currentCmdL()>0 && hub.currentCmdR()>0);
    assert(wireCurrent(Serial1)>0 && wireCurrent(Serial2)<0); // mirrored mount, ISGN=-1
    hub.disable();cycle();hub.enable();
    hub.setSpeeds(-.05f*P.countsPerM,.05f*P.countsPerM);cycle();
    assert(hub.currentCmdL()<0 && hub.currentCmdR()>0);
    assert(wireCurrent(Serial1)<0 && wireCurrent(Serial2)<0);
    assert(std::fabs(hub.currentCmdL()+hub.currentCmdR())<.00001f);
    // Direct balance acceleration survives a fixed target and overspeed,
    // and its wire polarity remains correct during a forward catch.
    hub.disable();cycle();hub.enable();
    hub.setBalanceCommand(0,0,6,true);cycle();
    assert(hub.currentCmdL()>.4f && hub.currentCmdR()>.4f);
    assert(wireCurrent(Serial1)>0 && wireCurrent(Serial2)<0);
    const int before=queries;
    for(int i=0;i<300;++i)cycle();
    assert(queries==before); // never inject temperature/pos8 replies into armed pos16 feedback
    assert(params_set("LDIR",1)==-3);
    hub.transmit(.005f);testMicros+=9000;hub.service(.005f);
    hub.transmit(.005f);
    assert(!hub.isEnabled() && !hub.modeOk() && hub.currentCmdL()==0 && hub.currentCmdR()==0);
    puts("BLDC drive: actual UART polarity, pure-turn symmetry, armed DRIVE-only polling and timeout disable passed");
}
