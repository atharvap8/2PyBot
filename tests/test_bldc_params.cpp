#include "params.h"
#include <cassert>
#include <map>
#include <string>
static bool active=false;
bool params_motionActive() { return active; }
void params_applyHardware(bool) {}

int main() {
    testNvs["2pybot:_ver"]=1;
    testNvs["2pybot:MAXSPD"]=9000;
    testNvs["2pybot-hub:_ver"]=1;
    testNvs["2pybot-hub:ISGN"]=1;
    params_begin();
    assert(P.curSign==-1 && P.rpmSign==-1 && P.maxSpdSt==0 && P.hubVSrc==1);
    assert(P.stepsPerM==P.countsPerM && P.vMax>.7f && P.vMax<.75f);
    assert(params_set("ISGN",1)==-2 && params_set("RPMSGN",1)==-2 && params_set("LDIR",0)==-2);
    assert(params_set("DRVMODE",.5f)==-2 && params_set("CURRENT",5000)==-2);
    active=true;
    assert(params_set("DRVMODE",0)==-3 && params_set("LDIR",1)==-3);
    assert(params_set("HUBKP",3.5f)==0); // live dynamics tuning remains available
    params_save(); assert(testNvs.count("2pybot-bldc4:_ver")==0);
    Serial.output.clear(); Serial.capacity=0;
    params_dump(); params_service(); assert(Serial.output.empty()); // no blocking output
    Serial.capacity=1024;
    for(int i=0;i<180;++i) { if(i==30)params_dump(); params_service(); }
    assert(Serial.output.find("PR,HUBSCHEMA,4\n")!=std::string::npos);
    assert(Serial.output.find("P.\n")!=std::string::npos); // retry did not starve completion
    active=false; params_save(); assert(testNvs["2pybot-bldc4:ISGN"]==-1);
    testNvs["2pybot-bldc4:ISGN"]=1; testNvs["2pybot-bldc4:RPMSGN"]=1;
    params_load(); assert(P.curSign==-1 && P.rpmSign==-1);
    puts("BLDC parameters: namespace isolation, polarity, bounds, active-write locks and nonblocking dumps passed");
}
