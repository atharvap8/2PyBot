#include "hub_drive.h"
#include <cassert>

bool params_motionActive() { return hub.isEnabled(); }
void params_applyHardware(bool) { hub.applyTuning(); }
static int queries=0;
static uint8_t replyMode=m0601::MODE_CURRENT;
static void reply(HardwareSerial& serial,uint8_t id) {
    if(serial.tx.empty())return;
    const size_t start=serial.tx.size()-10;
    uint8_t command=serial.tx[start+1];
    if(command==m0601::CMD_MODE) {serial.tx.clear();return;}
    uint8_t f[10]={id,replyMode,0,0,0,0,0,0,0,0};
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
    params_begin();assert(P.drvMode==2);assert(params_set("DRVMODE",1)==0);hub.applyTuning();
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
    // v4.0.3: a lost reply does not stop either wheel's next torque frame.
    Serial1.tx.clear();Serial2.tx.clear();
    hub.transmit(.005f);                                    // no reply injected
    testMicros+=5000;hub.service(.005f);
    Serial1.tx.clear();Serial2.tx.clear();
    hub.transmit(.005f);
    assert(hub.isEnabled() && hub.modeOk());
    assert(Serial1.tx.size()==10 && Serial1.tx[1]==m0601::CMD_DRIVE); // sent despite the gap
    assert(Serial2.tx.size()==10 && Serial2.tx[1]==m0601::CMD_DRIVE);
    reply(Serial1,1);reply(Serial2,2);testMicros+=5000;hub.service(.005f);
    for(int i=0;i<5;++i)cycle();
    assert(hub.isEnabled() && hub.modeOk());
    // Motor rebooted into VELOCITY mode: the very next reply disarms.
    Serial1.tx.clear();Serial2.tx.clear();hub.transmit(.005f);
    {uint8_t f[10]={1,m0601::MODE_VELOCITY,0,0,0,0,0,0,0,0};f[9]=m0601::crc8(f,9);for(auto b:f)Serial1.rx.push_back(b);}
    reply(Serial2,2);testMicros+=5000;hub.service(.005f);
    assert(!hub.modeOk());
    Serial1.tx.clear();Serial2.tx.clear();hub.transmit(.005f);
    assert(!hub.isEnabled() && hub.currentCmdL()==0 && hub.currentCmdR()==0);
    assert(Serial1.tx[1]==m0601::CMD_MODE || Serial1.tx[1]==m0601::CMD_QUERY); // handshake, no drive value
    // Total silence: link age expires and the drive stays disabled.
    hub.enable();
    testMicros+=61000;hub.service(.005f);hub.transmit(.005f);
    assert(!hub.isEnabled() && !hub.modeOk() && hub.currentCmdL()==0 && hub.currentCmdR()==0);
    // Explicit velocity balancing: confirmed mode 2, robot-forward wire signs,
    // no extra current PI or balance ramp, and common speed before steering.
    assert(params_set("DRVMODE",2)==0);
    assert(hub.velocityMode() && hub.balanceModeSelected() && !hub.modeOk());
    replyMode=m0601::MODE_VELOCITY;
    for(int i=0;i<8;++i)cycle();
    assert(hub.modeOk());
    assert(params_set("HUBACC",150)==0);
    hub.enable();hub.setBalanceCommand(.1f*P.countsPerM,.1f*P.countsPerM,6,false);cycle();
    assert(wireCurrent(Serial1)>0 && wireCurrent(Serial2)<0); // fields now contain signed RPM
    assert(std::abs(wireCurrent(Serial1))==18 && std::abs(wireCurrent(Serial2))==18);
    assert(Serial1.tx[6]==0 && Serial2.tx[6]==0);
    assert(hub.currentCmdL()==0 && hub.currentCmdR()==0);
    assert(params_set("DRVMODE",1)==-3 && params_set("VSIGN",1)==-3);
    const float limit=hub.velocityLimit();
    hub.setBalanceCommand((limit-.1f)*P.countsPerM,(limit+.1f)*P.countsPerM,6,true);cycle();
    assert(std::abs(wireCurrent(Serial1))==330 && std::abs(wireCurrent(Serial2))==330 && hub.saturated());
    hub.disable();cycle();cycle();
    assert(wireCurrent(Serial1)==0 && wireCurrent(Serial2)==0);
    assert(params_set("DRVMODE",0)==0 && !hub.balanceModeSelected()); // bench stays bench
    assert(params_set("DRVMODE",1)==0 && !hub.modeOk()); // old mode-2 reply cannot arm current mode
    replyMode=m0601::MODE_CURRENT;
    for(int i=0;i<8;++i)cycle();
    assert(hub.modeOk() && !hub.velocityMode());
    puts("BLDC drive: actual UART polarity, pure-turn symmetry, armed DRIVE-only polling, never-blocking send, mode-loss and timeout disable passed");
}
