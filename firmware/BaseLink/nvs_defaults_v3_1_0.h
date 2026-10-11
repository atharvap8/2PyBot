#pragma once
#include <stddef.h>
#include <string.h>

// Confirmed PL/NVS readback, revision 3.1.0. New parameters retain their fallback.
struct CapturedDefault { const char* key; float value; };
static const CapturedDefault CAPTURED_DEFAULTS_310[] = {
  {"K1",-7.3234f},{"K2",-7.4495f},{"K3",-42.9506f},{"K4",-6.6862f},{"K5",-2.1082f},
  {"TRIM",1.54042f},{"ZLIM",0.5f},{"EXCLAMP",0.15f},{"VSATBLD",3.0f},{"BRAKELK",0.3f},
  {"S1",-60.0226f},{"S2",-31.0364f},{"S3",-82.0431f},{"S4",-10.2571f},{"S5",-25.8199f},
  {"WHEELR",0.05f},{"MAXSPD",8500.0f},{"MAXACC",20000.0f},{"CURRENT",1500.0f},
  {"DRIVEV",0.908784f},{"DRIVEDB",0.03f},{"LOWDRV",0.5f},{"LOWSTR",0.5f},
  {"JOYFWD",-0.2f},{"JOYSTR",1.0f},{"STICKDZ",0.1f},{"SNAPIN",10.0f},{"SNAPOUT",26.0f},
  {"EXPOSTR",0.65f},{"EXPODRV",0.35f},{"MAXSTR",3003.08f},{"STRSLEW",15000.0f},{"STRDB",0.05f},
  {"YAWKP",1.5f},{"YAWKD",0.02f},{"YAWMAX",1200.0f},{"YAWRELCH",2000.0f},{"YAWTILT",12.0f},
  {"TRACKW",0.155f},{"LOOKMAX",18.0f},{"ARMANG",5.0f},{"MAXTILT",55.0f},{"MAXRATE",450.0f},
  {"RADXATMO",600.0f},{"JOYTMO",200.0f},{"BOOTWAIT",1.0f},{"IMUCUT",50.0f},
  {"MAHKP",2.0f},{"MAHKI",0.005f},{"PITCHSGN",-1.0f},{"ENCSGN",1.0f},{"RATESGN",1.0f},
  {"CLMBZ",2.0f},{"CLMBV",0.25f},{"CLMBSTR",0.4f},{"LEDBRI",200.0f},{"LEDFRONT",7.5f},
  {"SVYAWR",70.0f},{"SVZOOMR",45.0f},{"SVYAWMIN",120.0f},{"SVYAWMAX",190.0f},
  {"SVZMIN",80.0f},{"SVZMAX",140.0f},{"TRCHSTP",10.0f},
  {"C1",-22.2369f},{"C2",-20.0745f},{"C3",-81.3063f},{"C4",-12.7029f},{"C5",-8.165f},
  {"SGTHRSL",76.0f},{"SGTHRSR",77.0f},{"TCOOL",300.0f},{"CSSEMIN",5.0f},{"CSSEMAX",2.0f},
  {"MAGOFFX",0.0f},{"MAGOFFY",0.0f},{"MAGOFFZ",0.0f},
  {"MAGSGNX",1.0f},{"MAGSGNY",1.0f},{"MAGSGNZ",1.0f},{"ACCSGN",-1.0f},{"GYROSGN",-1.0f},
  {"PADDZ",0.08f},{"PADFWD",1.0f},{"PADSTR",-1.0f},{"PADRPT1",400.0f},{"PADRPT",130.0f},
  {"BOOTHIGH",0.0f},{"PADYAW",-1.0f},{"PADZOOM",1.0f},
  {"SVUSMIN",500.0f},{"SVUSMAX",2400.0f},{"SVYAWHOME",165.0f},{"SVZHOME",120.0f},
  {"SVRATE",50.0f},{"SVIDLE",0.0f},{"TRCHBOOT",25.0f},{"TRCHMIN",5.0f},{"LEDFPS",50.0f},
  {"TERREN",1.0f},{"TERRTHR",0.18f},{"TERRSOFT",0.55f},{"TERRTAU",0.35f},
  {"DROPG",0.55f},{"DROPMS",35.0f},{"LANDMS",350.0f},{"AIRFRZ",1.0f}
};
static inline float captured_default_310(const char* key, float fallback) {
  for (size_t i=0;i<sizeof(CAPTURED_DEFAULTS_310)/sizeof(CAPTURED_DEFAULTS_310[0]);++i)
    if (!strcmp(key,CAPTURED_DEFAULTS_310[i].key)) return CAPTURED_DEFAULTS_310[i].value;
  return fallback;
}
