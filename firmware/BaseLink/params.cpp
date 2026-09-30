/*
 * params.cpp — single definition of the parameter store.
 * Table, bounds and protocol live here; params.h only declares.
 */
#include "params.h"

Params P = {
#define X(f, k, d, lo, hi, g, desc)  (float)(d),
    PARAM_TABLE(X)
#undef X
    0, 0, 0, 0
};

const ParamMeta PARAM_META[] = {
#define X(f, k, d, lo, hi, g, desc) { k, offsetof(Params, f), (float)(d), (float)(lo), (float)(hi), g, desc },
    PARAM_TABLE(X)
#undef X
};
const size_t PARAM_COUNT = sizeof(PARAM_META) / sizeof(PARAM_META[0]);

static Preferences _prefs;



// ============================================================
//  Derived values — one place, always consistent
// ============================================================
void params_recompute() {
    if (P.wheelR < 0.005f) P.wheelR = 0.005f;
    P.stepsPerM  = (float)USTEPS_PER_REV / (2.0f * PI * P.wheelR);
    P.countsPerM = (float)ENCODER_CPR    / (2.0f * PI * P.wheelR);
    P.vMax       = P.maxSpdSt  / P.stepsPerM;
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
    bool currentChanged = !strcasecmp(key, "CURRENT");
    *params_ptr(i) = v;
    params_recompute();
    params_applyHardware(currentChanged);
    if (!quiet) Serial.printf("P!,%s,%.6g\n", m.key, v);
    return 0;
}

void params_defaults() {
    for (size_t i = 0; i < PARAM_COUNT; i++) *params_ptr(i) = PARAM_META[i].def;
    params_recompute();
    params_applyHardware(true);
}

// ============================================================
//  NVS persistence
// ============================================================
void params_save() {
    _prefs.begin("2pybot", false);
    for (size_t i = 0; i < PARAM_COUNT; i++)
        _prefs.putFloat(PARAM_META[i].key, *params_ptr(i));
    _prefs.putUChar("_ver", 1);
    _prefs.end();
    Serial.println("PS!,saved");
}

void params_load() {
    _prefs.begin("2pybot", true);
    bool have = _prefs.getUChar("_ver", 0) == 1;
    if (have) {
        for (size_t i = 0; i < PARAM_COUNT; i++) {
            float v = _prefs.getFloat(PARAM_META[i].key, PARAM_META[i].def);
            const ParamMeta& m = PARAM_META[i];
            *params_ptr(i) = (v >= m.lo && v <= m.hi) ? v : m.def;   // bounds win
        }
    }
    _prefs.end();
    params_recompute();
    params_applyHardware(true);
    Serial.printf("PL!,%s\n", have ? "loaded from NVS" : "no NVS, using defaults");
}

// ============================================================
//  Dump — the GUI builds its whole UI from this
// ============================================================
void params_dump() {
    Serial.printf("P#,%u\n", (unsigned)PARAM_COUNT);
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        const ParamMeta& m = PARAM_META[i];
        Serial.printf("P,%s,%.6g,%.6g,%.6g,%s,%s\n",
                      m.key, *params_ptr(i), m.lo, m.hi, m.group, m.desc);
    }
    // derived, read-only, so the GUI can show the consequences live
    Serial.printf("PR,STEPSM,%.2f\n",  P.stepsPerM);
    Serial.printf("PR,COUNTSM,%.2f\n", P.countsPerM);
    Serial.printf("PR,VMAX,%.4f\n",    P.vMax);
    Serial.printf("PR,AMAX,%.4f\n",    P.aMax);
    Serial.println("P.");
}

// ============================================================
//  Protocol entry point. Returns true if the line was a P-command.
//  Call this FIRST in handleLine().
// ============================================================
bool params_handleLine(char* line) {
    if (line[0] != 'P' && line[0] != 'p') return false;

    if (line[1] == '?') { params_dump();  return true; }
    if (line[1] == 'S') { params_save();  return true; }
    if (line[1] == 'L') { params_load();  return true; }
    if (line[1] == 'D') { params_defaults(); Serial.println("PD!,defaults restored (not saved)"); return true; }

    if (line[1] == ',') {
        char* key = line + 2;
        char* comma = strchr(key, ',');
        if (!comma) { Serial.println("PE,malformed"); return true; }
        *comma = '\0';
        float v = atof(comma + 1);
        int r = params_set(key, v);
        if (r == -1) Serial.printf("PE,unknown key %s\n", key);
        if (r == -2) {
            int i = params_index(key);
            Serial.printf("PE,%s out of range [%.6g,%.6g]\n",
                          key, PARAM_META[i].lo, PARAM_META[i].hi);
        }
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

