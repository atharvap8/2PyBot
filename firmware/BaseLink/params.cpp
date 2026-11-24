/*
 * params.cpp — single definition of the parameter store.
 * Table, bounds and protocol live here; params.h only declares.
 */
#include "params.h"
#include "nvs.h"
#include "nvs_flash.h"

Params P = {
#define X(f, k, d, lo, hi, g, desc)  (float)(d),
    PARAM_TABLE(X)
#undef X
    0, 0, 0, 0, 0, 0, 0
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
    // These are fixed firmware requirements, not overridable NVS defaults.
    P.pitchTrim=PITCH_TRIM_DEG;
    P.maxSteerSt=MAX_STEER_STEPS;
    if (P.wheelR < 0.005f) P.wheelR = 0.005f;
    P.countsPerM = (float)ENCODER_CPR    / (2.0f * PI * P.wheelR);
    P.stepsPerM  = P.countsPerM;
    const float freeRpm = P.freeRpm > 0 ? P.freeRpm : P.supplyV * M0601_KV_RPM_PER_V;
    P.vNoLoad    = freeRpm / 60.0f * 2.0f * PI * P.wheelR; // informational, never a control clamp
    P.maxSpdEff  = P.maxSpdSt; // zero means no firmware speed ceiling
    P.vMax       = P.maxSpdEff / P.countsPerM;
    P.aMax       = P.motAccel  / P.stepsPerM;
    // DRIVEV is the operator's full-stick target. Estimated/measured free
    // speed and legacy SPDHEAD are informational, never hidden drive clamps.
    float drive = P.maxDriveV;
    if (P.vMax > 0) drive = fminf(drive, P.vMax);
    P.driveVEff  = fmaxf(0.05f, drive);
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
// All parameters are stored as ONE blob of (FNV-1a(key), value) pairs. The old
// layout used one NVS entry per key (~150 blobs, ~450 entries), which filled
// the 20 KB NVS partition together with the Bluetooth bond keys: every save
// failed and every boot read "no NVS". A single ~1.2 KB blob needs ~45 entries.
struct __attribute__((packed)) ParamRecord { uint32_t hash; float value; };
static uint32_t keyHash(const char* k) {
    uint32_t h = 2166136261u;
    while (*k) { h ^= (uint8_t)*k++; h *= 16777619u; }
    return h;
}
static const char* BLOB_KEY = "pblob";

// PN: what occupies the NVS partition (namespaces and entry counts).
void params_nvsReport() {
    nvs_stats_t st = {};
    if (nvs_get_stats(NULL, &st) == ESP_OK)
        Serial.printf("PN,stats,used=%u,free=%u,total=%u,namespaces=%u\n", (unsigned)st.used_entries,
                      (unsigned)st.free_entries, (unsigned)st.total_entries, (unsigned)st.namespace_count);
    struct NsCount { char name[16]; unsigned keys; };
    static NsCount ns[24]; size_t nns = 0;
    nvs_iterator_t it = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY);
    while (it) {
        nvs_entry_info_t info; nvs_entry_info(it, &info);
        size_t k = 0;
        while (k < nns && strcmp(ns[k].name, info.namespace_name)) k++;
        if (k == nns && nns < 24) { strncpy(ns[nns].name, info.namespace_name, 15); ns[nns].name[15] = 0; ns[nns].keys = 0; nns++; }
        if (k < nns) ns[k].keys++;
        it = nvs_entry_next(it);
    }
    nvs_release_iterator(it);
    for (size_t k = 0; k < nns; k++) Serial.printf("PN,ns,%s,%u\n", ns[k].name, ns[k].keys);
    Serial.println("PN.");
}

void params_save() {
    if (params_motionActive()) { Serial.println("PE,stop motors before saving NVS"); return; }
    static ParamRecord rec[PARAM_TABLE_MAX];
    if (PARAM_COUNT > PARAM_TABLE_MAX) { Serial.println("PE,parameter table too large for blob"); return; }
    for (size_t i = 0; i < PARAM_COUNT; i++) { rec[i].hash = keyHash(PARAM_META[i].key); rec[i].value = *params_ptr(i); }
    const size_t bytes = PARAM_COUNT * sizeof(ParamRecord);
    if (!_prefs.begin(NVS_NS, false)) { Serial.println("PE,NVS open for save failed"); return; }
    // Drop the old per-key entries (frees the space they wasted), then write the blob.
    const bool cleared = _prefs.clear();
    bool written = _prefs.putBytes(BLOB_KEY, rec, bytes) == bytes;
    if (written) written=_prefs.putUChar("_ver",2)==1;
    if (written) written=_prefs.putUChar("_cur5s",1)==1;
    if (written) written=_prefs.putUChar("_input3",1)==1;
    if (written) written=_prefs.putUChar("_brake2",1)==1;
    const size_t freeEntries = _prefs.freeEntries();
    _prefs.end();
    if (!written) { Serial.printf("PE,NVS write failed (cleared=%d, free entries=%u); save not confirmed\n", (int)cleared, (unsigned)freeEntries); return; }
    if (!_prefs.begin(NVS_NS,true)) { Serial.println("PE,NVS reopen for verification failed"); return; }
    static ParamRecord back[PARAM_TABLE_MAX];
    bool verified = _prefs.getBytesLength(BLOB_KEY) == bytes && _prefs.getBytes(BLOB_KEY, back, bytes) == bytes &&
                    !memcmp(back, rec, bytes);
    verified=verified && _prefs.getUChar("_ver",0)==2 && _prefs.getUChar("_cur5s",0)==1 && _prefs.getUChar("_input3",0)==1 && _prefs.getUChar("_brake2",0)==1;
    _prefs.end();
    if (!verified) { Serial.println("PE,NVS readback failed; save not confirmed"); return; }
    Serial.printf("PS!,saved (%u params, %u NVS entries free)\n", (unsigned)PARAM_COUNT, (unsigned)freeEntries);
}

void params_load() {
    if (params_motionActive()) { Serial.println("PE,stop motors before loading NVS"); return; }
    const bool opened=_prefs.begin(NVS_NS, true);
    const uint8_t ver = opened ? _prefs.getUChar("_ver", 0) : 0;
    // v2: one blob; v1: legacy one-key-per-parameter (read once, rewritten as a blob by PS)
    static ParamRecord rec[PARAM_TABLE_MAX];
    size_t nrec = 0;
    if (ver == 2) {
        const size_t len = _prefs.getBytesLength(BLOB_KEY);
        if (len && len % sizeof(ParamRecord) == 0 && len <= sizeof(rec) && _prefs.getBytes(BLOB_KEY, rec, len) == len)
            nrec = len / sizeof(ParamRecord);
    }
    bool have = (ver == 1) || (ver == 2 && nrec > 0);
    const bool migrate = !opened || _prefs.getUChar("_cur5s", 0) != 1;
    const bool migrateInput = !opened || _prefs.getUChar("_input3", 0) != 1;
    const bool oldInputProfile = opened && _prefs.getUChar("_input2",0)==1;
    const bool migrateBrake = !opened || _prefs.getUChar("_brake2", 0) != 1;
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        float v = PARAM_META[i].def;
        if (ver == 1) v = _prefs.getFloat(PARAM_META[i].key, PARAM_META[i].def);
        else if (nrec) {
            const uint32_t h = keyHash(PARAM_META[i].key);
            for (size_t j = 0; j < nrec; j++) if (rec[j].hash == h) { v = rec[j].value; break; }
        }
        const ParamMeta& m = PARAM_META[i];
        const bool discreteSign = !strcmp(m.key,"LDIR") || !strcmp(m.key,"RDIR");
        const bool discreteMode = !strcmp(m.key,"DRVMODE") || !strcmp(m.key,"HUBVSRC");
        *params_ptr(i) = (v >= m.lo && v <= m.hi && (!discreteSign || fabsf(v)==1) &&
                         (!discreteMode || v==0 || v==1)) ? v : m.def;
    }
    if (opened) _prefs.end();
    if (migrate) {
        // One-time RAM baseline for the owner's current-mode / 18 V / 2.6 kg
        // setup. Preserve trim, wheel calibration, motor polarity and payload
        // calibration. PS persists these values and marks migration complete.
        P.drvMode=1;P.supplyV=SUPPLY_V;P.hubMass=HUB_MASS_KG;
        P.k1=LQR_K1;P.k2=LQR_K2;P.k3=LQR_K3;P.k4=LQR_K4;P.k5=LQR_K5;
        P.motorMa=MOTOR_CURRENT_MA;P.maxSpdSt=MAX_SPEED_STEPS;P.freeRpm=0;
        P.motAccel=6.0f*M0601_CPR/(2.0f*PI*P.wheelR);
        Serial.println("[PARAM] Current-mode 18 V / 2.6 kg baseline loaded in RAM; PS saves it");
    }
    if (migrateInput) {
        if (have && !oldInputProfile) P.camExpo=P.expoSteer;
        P.pitchTrim=PITCH_TRIM_DEG;
        P.maxSteerSt=MAX_STEER_STEPS;
        P.steerSlew=0;P.hubYawSlew=0;P.spdLoStr=1;P.expoSteer=0;
        Serial.println("[PARAM] Fixed TRIM=-1.2603 / MAXSTR=4000 active; PS confirms NVS persistence");
    }
    if (migrateBrake) {
        // 4.7.6: the DRVDEC ramp lengthened every stop; bubble back to the measured model.
        const char* keys[] = {"DRVDEC", "BUBDEC", "BUBLAT", "BUBMRG", "VERRMAX"};
        for (const char* k : keys) { int i = params_index(k); if (i >= 0) *params_ptr(i) = PARAM_META[i].def; }
        Serial.println("[PARAM] Braking profile 2 defaults loaded in RAM (DRVDEC off, VERRMAX 0.7, bubble 0.5/0.3/0.08); PS saves it");
    }
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
        "TOFGUARD","BOOTGATE","STARTUPACK","FALLPREVENT","AUTOCLIMB","AUTODESCENT","FLASHLOCK","NAVIGATION","VDRIVE","HOSTLED","VELBAL","CURONLY","CURPROFILE","VELCTRL","VAMAX","INPUTPROFILE","FIXEDINPUT","PARAMACK","DIRECTYAW","SELFRIGHT","BUBBLE","BRAKEPROFILE","PADREPAIR","SLEEP"};
    // PADREPAIR 2: BT,FORGET restarts and wipes the Bluetooth storage. SLEEP 1: SL,1/SL,0 power-down.
    const float derivedValues[] = {P.stepsPerM,P.countsPerM,P.vMax,P.aMax,P.vNoLoad,1,4,1,1,1,0,1,1,1,1,P.driveVEff,0,0,1,1,0,0,2,1,2,1,1,2,2,2,1};
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
    if (line[1] == 'N' && line[2] == '\0') { params_nvsReport(); return true; }
    if (line[1] == 'X' && line[2] == ',') {
        // Erase one whole namespace (old firmware generations). Never the BT bonds.
        const char* ns = line + 3;
        if (params_motionActive()) { Serial.println("PE,stop motors before NVS operations"); return true; }
        if (!*ns || strlen(ns) > 15 || !strncmp(ns, "bt", 2) || !strcmp(ns, "phy") || !strcmp(ns, "nvs.net80211")) {
            Serial.println("PE,refusing to erase that namespace"); return true;
        }
        Preferences p;
        const bool ok = p.begin(ns, false) && p.clear();
        p.end();
        Serial.printf("PX!,%s,%s\n", ns, ok ? "erased" : "failed");
        params_nvsReport();
        return true;
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
                  "free %.2f m/s, full stick %.2f m/s, aMax %.2f m/s2\n",
                  (unsigned)PARAM_COUNT, P.wheelR, P.stepsPerM, P.vNoLoad, P.driveVEff, P.aMax);
}

