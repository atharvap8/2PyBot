/*
 * params.cpp — single definition of the parameter store.
 * Table, bounds and protocol live here; params.h only declares.
 */
#include "params.h"

Params P = {
#define X(f, k, d, lo, hi, g, desc)  (float)(d),
    PARAM_TABLE(X)
#undef X
    0, 0, 0, 0, 0, 0
};

const ParamMeta PARAM_META[] = {
#define X(f, k, d, lo, hi, g, desc) { k, offsetof(Params, f), (float)(d), (float)(lo), (float)(hi), g, desc },
    PARAM_TABLE(X)
#undef X
};
const size_t PARAM_COUNT = sizeof(PARAM_META) / sizeof(PARAM_META[0]);

static Preferences _prefs;
// Neither stepper microsteps nor experimental hub tuning is a v4 baseline.
// Their namespaces remain intact for rollback; no NVS erase is performed.
static const char* NVS_NS = "2pybot-bldc4";



// ============================================================
//  Derived values — one place, always consistent
// ============================================================
void params_recompute() {
    if (P.wheelR < 0.005f) P.wheelR = 0.005f;
    P.countsPerM = (float)ENCODER_CPR    / (2.0f * PI * P.wheelR);
    P.stepsPerM  = P.countsPerM;
    P.vNoLoad    = P.supplyV * M0601_KV_RPM_PER_V / 60.0f * 2.0f * PI * P.wheelR;
    P.maxSpdEff  = P.vNoLoad * P.countsPerM;
    if (P.maxSpdSt > 0) P.maxSpdEff = fminf(P.maxSpdEff, P.maxSpdSt);
    P.vMax       = P.maxSpdEff / P.countsPerM;
    P.aMax       = P.motAccel  / P.stepsPerM;
}

float* params_ptr(size_t i) {
    return (float*)((uint8_t*)&P + PARAM_META[i].offset);
}

int params_index(const char* key) {
    for (size_t i = 0; i < PARAM_COUNT; i++)
        if (!strcasecmp(key, PARAM_META[i].key)) return (int)i;
    return -1;
}

// Bounds-checked write. Returns 0 OK, -1 unknown key, -2 out of range.
int params_set(const char* key, float v, bool quiet) {
    int i = params_index(key);
    if (i < 0) return -1;
    const ParamMeta& m = PARAM_META[i];
    if (!(v >= m.lo && v <= m.hi)) return -2;           // also rejects NaN
    if ((!strcasecmp(key,"LDIR") || !strcasecmp(key,"RDIR")) && fabsf(v)!=1.0f) return -2;
    if ((!strcasecmp(key,"DRVMODE") || !strcasecmp(key,"HUBVSRC")) && v!=0.0f && v!=1.0f) return -2;
    if (params_motionActive() && (!strcasecmp(key,"DRVMODE") || !strcasecmp(key,"LDIR") ||
        !strcasecmp(key,"RDIR") || !strcasecmp(key,"WHEELR") || !strcasecmp(key,"HUBVSRC"))) return -3;
    bool currentChanged = !strcasecmp(key, "CURRENT") || !strcasecmp(key, "DRVMODE");
    *params_ptr(i) = v;
    params_recompute();
    params_applyHardware(currentChanged);
    if (!quiet) Serial.printf("P!,%s,%.6g\n", m.key, v);
    return 0;
}

void params_defaults() {
    if (params_motionActive()) { Serial.println("PE,stop motors before restoring defaults"); return; }
    for (size_t i = 0; i < PARAM_COUNT; i++) *params_ptr(i) = PARAM_META[i].def;
    params_recompute();
    params_applyHardware(true);
}

// ============================================================
//  NVS persistence
// ============================================================
void params_save() {
    if (params_motionActive()) { Serial.println("PE,stop motors before saving NVS"); return; }
    _prefs.begin(NVS_NS, false);
    for (size_t i = 0; i < PARAM_COUNT; i++)
        _prefs.putFloat(PARAM_META[i].key, *params_ptr(i));
    _prefs.putUChar("_ver", 1);
    _prefs.end();
    Serial.println("PS!,saved");
}

void params_load() {
    if (params_motionActive()) { Serial.println("PE,stop motors before loading NVS"); return; }
    _prefs.begin(NVS_NS, true);
    bool have = _prefs.getUChar("_ver", 0) == 1;
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        float v = have ? _prefs.getFloat(PARAM_META[i].key, PARAM_META[i].def) : PARAM_META[i].def;
        const ParamMeta& m = PARAM_META[i];
        const bool discreteSign = !strcmp(m.key,"LDIR") || !strcmp(m.key,"RDIR");
        const bool discreteMode = !strcmp(m.key,"DRVMODE") || !strcmp(m.key,"HUBVSRC");
        *params_ptr(i) = (v >= m.lo && v <= m.hi && (!discreteSign || fabsf(v)==1) &&
                         (!discreteMode || v==0 || v==1)) ? v : m.def;
    }
    _prefs.end();
    params_recompute();
    params_applyHardware(true);
    Serial.printf("PL!,%s\n", have ? "loaded from NVS" : "no NVS, using defaults");
}

// ============================================================
//  Dump — the GUI builds its whole UI from this
// ============================================================
static int dumpRow = -1;
static bool dumpAgain = false;
void params_dump() {
    if (dumpRow < 0) dumpRow = 0;
    else dumpAgain = true; // retries may arrive before this asynchronous dump finishes
}

void params_service() {
    if (dumpRow < 0) return;
    char line[256];
    const char* derivedKeys[] = {"STEPSM","COUNTSM","VMAX","AMAX","VNOLOAD","HUBMOTOR","HUBSCHEMA",
        "TOFGUARD","BOOTGATE","STARTUPACK","FALLPREVENT","AUTOCLIMB","AUTODESCENT","FLASHLOCK","NAVIGATION"};
    const float derivedValues[] = {P.stepsPerM,P.countsPerM,P.vMax,P.aMax,P.vNoLoad,1,4,1,1,1,1,1,1,1,1};
    const int end = (int)PARAM_COUNT + 1 + sizeof(derivedValues)/sizeof(derivedValues[0]);
    int length;
    if (dumpRow == 0) length = snprintf(line,sizeof(line),"P#,%u\n",(unsigned)PARAM_COUNT);
    else if (dumpRow <= (int)PARAM_COUNT) {
        const size_t index = dumpRow - 1;
        const auto& m = PARAM_META[index];
        length = snprintf(line,sizeof(line),"P,%s,%.6g,%.6g,%.6g,%s,%s\n",m.key,*params_ptr(index),m.lo,m.hi,m.group,m.desc);
    } else if (dumpRow < end) {
        const int index = dumpRow - (int)PARAM_COUNT - 1;
        length = snprintf(line,sizeof(line),"PR,%s,%.6g\n",derivedKeys[index],derivedValues[index]);
    } else length = snprintf(line,sizeof(line),"P.\n");
    // One complete line per cycle, only if it fits without waiting for UART.
    // P? must never pause a balancing loop for a ~10 kB synchronous dump.
    if (length <= 0 || length >= (int)sizeof(line) || Serial.availableForWrite() < length + 96) return;
    Serial.write((const uint8_t*)line,(size_t)length);
    if (++dumpRow > end) { dumpRow = dumpAgain ? 0 : -1; dumpAgain = false; }
}

// ============================================================
//  Protocol entry point. Returns true if the line was a P-command.
//  Call this FIRST in handleLine().
// ============================================================
bool params_handleLine(char* line) {
    if (line[0] != 'P' && line[0] != 'p') return false;

    if (line[1] == '?') { params_dump();  return true; }
    if ((line[1]=='S' || line[1]=='L' || line[1]=='D') && params_motionActive()) {
        Serial.println("PE,stop motors before NVS/default operations"); return true;
    }
    if (line[1] == 'S') { params_save();  return true; }
    if (line[1] == 'L') { params_load();  return true; }
    if (line[1] == 'D') { params_defaults(); Serial.println("PD!,defaults restored (not saved)"); return true; }

    if (line[1] == ',') {
        char* key = line + 2;
        char* comma = strchr(key, ',');
        if (!comma) { Serial.println("PE,malformed"); return true; }
        *comma = '\0';
        char* end = nullptr;
        float v = strtof(comma + 1, &end);
        if (end == comma + 1 || *end != '\0') { Serial.println("PE,invalid number"); return true; }
        int r = params_set(key, v);
        if (r == -1) Serial.printf("PE,unknown key %s\n", key);
        if (r == -2) {
            int i = params_index(key);
            Serial.printf("PE,%s out of range [%.6g,%.6g]\n",
                          key, PARAM_META[i].lo, PARAM_META[i].hi);
        }
        if (r == -3) Serial.println("PE,stop motors before changing drive hardware");
        return true;
    }
    return false;   // not ours (e.g. a future 'P' command) — let others try
}

void params_begin() {
    params_load();          // NVS if present, else compiled defaults
    Serial.printf("[PARAM] %u tunables | wheelR %.4f m -> %.1f steps/m, "
                  "vMax %.2f m/s, aMax %.2f m/s2\n",
                  (unsigned)PARAM_COUNT, P.wheelR, P.stepsPerM, P.vMax, P.aMax);
}

