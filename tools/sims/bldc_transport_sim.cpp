// Common-mode balance + RS485 transport simulation used for the 4.0.3 analysis.
// Build: g++ -O2 -std=c++17 -I firmware/BaseLink tools/sims/bldc_transport_sim.cpp -o sim
// Env: DBG=<ms debug compute every 4th cycle> LOSS=<reply loss> SPIKE=<6 ms spike prob> HEAD=<SPDHEAD>
//      MB MW L IB TAU plant overrides. Columns: 4.0.2 as flashed, 4.0.3, 4.0.2 control + good transport.
// Common-mode wheeled-inverted-pendulum simulation of the 2PyBot BLDC controllers.
// Plant: body + two hub wheels, M0601 voltage/current limits (README sec. 6), 1 ms
// command latency, position replies one cycle old, pos16 quantisation, gyro noise.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include "hub_control.h"
#include "fall_prevention.h"

static const float PI_F = 3.14159265f;
static double envd(const char* k,double d){const char* v=getenv(k);return v?atof(v):d;}
struct Plant {
    double mb = envd("MB",1.6), mw = envd("MW",0.4), r = 0.052, l = envd("L",0.12), Ib = envd("IB",0.0063), Iw = 2 * 2.7e-4;
    double Kt = 0.75, ke = 12.0 / (133.0 / 60 * 2 * M_PI), R = 6.7, V = 12.0;
    double x = 0, v = 0, th = 0, om = 0, iAct = 0; // iAct per motor
    double wheelAngle() const { return x / r; }
    void step(double iCmd, double dt) {
        const double w = v / r;
        double hi = (V - ke * w) / R, lo = (-V - ke * w) / R;
        double target = std::fmax(lo, std::fmin(hi, iCmd));
        iAct += (target - iAct) * (1 - std::exp(-dt / envd("TAU",0.001)));
        const double tau = 2 * Kt * iAct - 0.04 * std::tanh(w * 20); // two motors, friction
        // M x'' + mb l cos th th'' = tau/r + mb l sin th om^2
        // mb l cos th x'' + J th'' = mb g l sin th - tau
        const double M = mb + mw + Iw / (r * r), J = Ib + mb * l * l;
        const double a11 = M, a12 = mb * l * std::cos(th), a21 = a12, a22 = J;
        const double b1 = tau / r + mb * l * std::sin(th) * om * om;
        const double b2 = mb * 9.81 * l * std::sin(th) - tau;
        const double det = a11 * a22 - a12 * a21;
        const double xdd = (b1 * a22 - a12 * b2) / det, thdd = (a11 * b2 - a21 * b1) / det;
        v += xdd * dt; x += v * dt; om += thdd * dt; th += om * dt;
    }
};

struct Result { bool fell; double maxTilt, maxSpeed, rmsTilt; double tFall; };

enum Variant { ORIGINAL, V402, FIXED, V402CTRL };
struct Scenario { double th0Deg, om0Dps, pushNs, pushAt, driveV, duration, voltage; };


static double DBG_MS = 0, LOSS = 0, SPIKE = 0;
static Result run(Variant var, const Scenario& sc, unsigned seed, bool trace = false) {
    Plant p; p.V = sc.voltage; p.th = sc.th0Deg * M_PI / 180; p.om = sc.om0Dps * M_PI / 180;
    std::mt19937 rng(seed); std::normal_distribution<double> n(0, 1); std::uniform_real_distribution<double> U(0, 1);
    const float K1 = -7.3234f, K2 = -7.4495f, K3 = -42.9506f, K4 = -6.6862f, K5 = -2.1082f;
    const float aMax = 6, r = .052f;
    const float vFree = 133.0f / 60 * 2 * PI_F * r * (float)(sc.voltage / 12.0);
    hub_control::Settings s4{4, 4, .5f, 1, 2, r, .75f, 1.5f, 2, .35f, 6, 2.7f};
    hub_control::Settings s6{6, 8, 1, 1, 2, r, .75f, 1.5f, 2, .35f, 6, 2.7f};
    const hub_control::Settings& sFix = s4;
    hub_control::CurrentLoop loop; loop.reset();
    FallPrevention fp;
    float vCmd = 0, vPrevTarget = 0, iInt = 0, zInt = 0, xRef = 0, velF = 0, posPrev = 0, wv = 0;
    // transport state
    bool pending = false, resync = false, reconfirm = false, replyInFlight = false, replyIsQuery = false;
    double sentAt = 0, replyAt = 0, replyPos = 0, quietSince = 0, lastRx = 0;
    double posKnown = 0, posKnownPrev = 0, lastPosT = 0; bool havePos = false; int posFrames = 0, usedFrames = 0;
    double bridge = 0; // original rpm bridge
    double iApplyAt = -1, iPending = 0, iApplied = 0;
    Result res{false, 0, 0, 0, -1}; double sumSq = 0; int samples = 0;
    const double h = 1e-4; double t = 0, nextCycle = 0, lastCycle = -0.005; int k = 0; int sends = 0, drives = 0;
    while (t < sc.duration) {
        if (t + 1e-9 >= nextCycle) {
            const double dt = t - lastCycle; lastCycle = t;
            // service(): parse a reply that has arrived
            if (replyInFlight && replyAt <= t) {
                replyInFlight = false; lastRx = replyAt;
                if (var == V402) {
                    if (pending) { pending = false; reconfirm = false; if (!replyIsQuery) { posKnown = replyPos; ++posFrames; } }
                    else quietSince = replyAt; // dropped during resync
                } else { if (!replyIsQuery) { posKnown = replyPos; ++posFrames; bridge = 0; } }
            }
            if (var == V402) {
                if (resync) { if (t - quietSince >= .010) resync = false; }
                else if (pending && t - sentAt > .008) { pending = false; resync = true; reconfirm = true; quietSince = t; }
            }
            const double quant = std::round(posKnown / (2 * M_PI) * 32767) / 32767 * 2 * M_PI * p.r;
            float wheelPos;
            if (var == V402) {
                wheelPos = (float)quant;
                if (posFrames != usedFrames) {
                    const double el = t - lastPosT; lastPosT = t; usedFrames = posFrames;
                    if (havePos && el > 0) wv += (1 - std::exp(-2 * PI_F * 30 * el)) * ((float)((quant - posKnownPrev) / el) - wv);
                    posKnownPrev = quant; havePos = true;
                }
            } else {
                if (posFrames == usedFrames && t - lastRx < .05) bridge += std::round(p.v / p.r * 60 / (2 * M_PI)) * 2 * M_PI / 60 * p.r * dt;
                usedFrames = posFrames;
                wheelPos = (float)(quant + bridge);
                wv += (1 - std::exp(-2 * PI_F * 30 * dt)) * ((float)((wheelPos - posKnownPrev) / dt) - wv);
                posKnownPrev = wheelPos;
            }
            const float pitch = (float)(p.th * 180 / M_PI + 0.05 * n(rng));
            const float rate = (float)(p.om * 180 / M_PI + 0.6 * n(rng));
            const float posM = wheelPos;
            const float vRaw = (posM - posPrev) / (float)dt; posPrev = posM;
            velF += (1 - std::exp(-2 * PI_F * 15 * dt)) * (vRaw - velF);
            if (std::fabs(pitch) > 55) { res.fell = true; res.tFall = t; break; }
            float th = pitch * PI_F / 180, om = rate * PI_F / 180, u;
            const float refLimit = var == FIXED ? (float)envd("HEAD",0.70) * vFree : vFree;
            float requested = (float)sc.driveV;
            if (var != V402 && var != V402CTRL) requested = std::fmax(-refLimit, std::fmin(refLimit, requested));
            fp.update(requested, velF, pitch, rate, vCmd, (var != ORIGINAL) ? 0 : vFree, aMax, 55, (float)dt, false);
            const bool driving = std::fabs(sc.driveV) > .01f;
            if (driving) { xRef = posM + velF * .3f; zInt = 0; u = -(K2 * (velF - fp.effectiveRef) + K3 * th + K4 * om); }
            else {
                float ex = std::fmax(-.15f, std::fmin(.15f, posM - xRef));
                zInt = std::fmax(-.5f, std::fmin(.5f, zInt + ex * (float)dt));
                u = -(K1 * ex + K2 * velF + K3 * th + K4 * om + K5 * zInt);
            }
            u = std::fmax(-aMax, std::fmin(aMax, u));
            // compute time of this cycle, then transmit
            double c = .0009 + ((k % 4 == 0) ? DBG_MS * 1e-3 : 0) + (U(rng) < SPIKE ? .006 : 0);
            const double txAt = t + c;
            float iCmd = 0; bool sendDrive = false, sendQuery = false;
            if (var == V402) {
                auto vel = hub_control::integrateVelocity(vCmd, u, velF, (float)dt, s4.kp, s4.currentLimit, 0);
                vCmd = vel.value;
                const bool rec = fp.recovering || std::fabs(pitch) >= 6 || std::fabs(rate) >= 40;
                if (!pending && !resync) { if (reconfirm) sendQuery = true; else sendDrive = true; }
                if (sendDrive) { iCmd = loop.update(vCmd, vCmd, wv, wv, (float)(txAt - sentAt > .05 ? .005 : std::fmax(.001, txAt - sentAt)), s4, u, rec).left; }
            } else if (var == ORIGINAL) {
                vCmd = std::fmax(-vFree, std::fmin(vFree, vCmd + u * (float)dt));
                float aff = std::fmax(-40.f, std::fmin(40.f, (vCmd - vPrevTarget) / (float)dt)); vPrevTarget = vCmd;
                const float e = vCmd - wv, kff = 1 * 2 * r / (2 * .75f), iMax = 2.7f;
                const float pre = kff * aff + 6 * e + iInt;
                if (!((pre >= iMax && e > 0) || (pre <= -iMax && e < 0))) iInt = std::fmax(-1.f, std::fmin(1.f, iInt + 8 * e * (float)dt));
                iCmd = std::fmax(-iMax, std::fmin(iMax, kff * aff + 6 * e + iInt)); sendDrive = true;
            } else if (var == V402CTRL) {
                auto vel = hub_control::integrateVelocity(vCmd, u, velF, (float)dt, s4.kp, s4.currentLimit, 0);
                vCmd = vel.value;
                const bool rec = fp.recovering || std::fabs(pitch) >= 6 || std::fabs(rate) >= 40;
                iCmd = loop.update(vCmd, vCmd, wv, wv, (float)dt, s4, u, rec).left; sendDrive = true;
            } else {
                auto vel = hub_control::integrateVelocity(vCmd, u, velF, (float)dt, sFix.kp, sFix.currentLimit, 0);
                vCmd = vel.value;
                const bool rec = fp.recovering || std::fabs(pitch) >= 6 || std::fabs(rate) >= 40;
                iCmd = loop.update(vCmd, vCmd, wv, wv, (float)dt, sFix, u, rec).left; sendDrive = true;
            }
            if (sendDrive || sendQuery) {
                ++sends; if (sendDrive) { ++drives; iPending = iCmd; iApplyAt = txAt + .0009; }
                sentAt = txAt; pending = true; replyIsQuery = sendQuery;
                if (U(rng) >= LOSS) { replyInFlight = true; replyAt = txAt + .0009 + .0003 + .0009; replyPos = -1; }
                else replyInFlight = false;
            }
            nextCycle = std::fmax(lastCycle + .005, txAt + .0002); ++k;
            if (trace && k % 10 == 0) printf("%.2f th=%6.2f om=%7.1f v=%5.2f vCmd=%5.2f u=%5.2f i=%5.2f\n", t, p.th * 180 / M_PI, p.om * 180 / M_PI, p.v, vCmd, u, iCmd);
        }
        if (iApplyAt >= 0 && t >= iApplyAt) { iApplied = iPending; iApplyAt = -1; }
        if (replyInFlight && replyPos < 0 && t >= replyAt - .0012) replyPos = p.wheelAngle();
        if (sc.pushNs != 0 && std::fabs(t - sc.pushAt) < .05) p.om += sc.pushNs / (p.Ib + p.mb * p.l * p.l) / .1 * h;
        p.step(iApplied, h); t += h;
        const double tilt = std::fabs(p.th * 180 / M_PI);
        res.maxTilt = std::fmax(res.maxTilt, tilt); res.maxSpeed = std::fmax(res.maxSpeed, std::fabs(p.v));
        sumSq += tilt * tilt; ++samples;
    }
    if (trace) printf("cycles %d sends %d drives %d\n", k, sends, drives);
    res.rmsTilt = std::sqrt(sumSq / std::max(1, samples));
    return res;
}

int main(int argc, char** argv) {
    DBG_MS = envd("DBG", 0); LOSS = envd("LOSS", 0); SPIKE = envd("SPIKE", 0);
    const char* names[] = {"original-6/8/1", "v4.0.2", "fixed", "402ctl+goodTx"};
    if (argc > 1) { // trace one
        int v = atoi(argv[1]); Scenario sc{atof(argv[2]), atof(argv[3]), 0, 0, atof(argv[4]), 6, argc > 5 ? atof(argv[5]) : 12};
        auto r = run((Variant)v, sc, 1, true);
        printf("fell=%d t=%.2f maxTilt=%.1f\n", r.fell, r.tFall, r.maxTilt); return 0;
    }
    struct Named { const char* n; Scenario s; } cases[] = {
        {"arm 2.9deg -25dps", {2.9, -25, 0, 0, 0, 8, 12}},
        {"arm -4deg +30dps", {-4, 30, 0, 0, 0, 8, 12}},
        {"shove 0.4 N*m*s", {0, 0, 0.04, 2, 0, 8, 12}},
        {"shove 0.08", {0, 0, 0.08, 2, 0, 8, 12}},
        {"drive 0.40 m/s", {0, 0, 0, 0, .40, 8, 12}},
        {"drive 0.55 m/s", {0, 0, 0, 0, .55, 8, 12}},
        {"drive 0.60 m/s", {0, 0, 0, 0, .60, 8, 12}},
        {"drive .5 + shove", {0, 0, 0.06, 4, .5, 8, 12}},
        {"drive 1.0 + shove", {0, 0, 0.06, 4, 1.0, 8, 12}},
        {"drive 1.00 m/s @12V", {0, 0, 0, 0, 1.0, 8, 12}},
        {"drive 1.00 m/s @16.8V", {0, 0, 0, 0, 1.0, 8, 16.8}},
        {"drive 1.00 m/s @21V", {0, 0, 0, 0, 1.0, 8, 21}},
    };
    for (auto& c : cases) {
        printf("%-24s", c.n);
        for (int v = 1; v < 4; ++v) {
            int falls = 0; double maxT = 0, rms = 0, sp = 0;
            for (unsigned seed = 1; seed <= 20; ++seed) {
                auto r = run((Variant)v, c.s, seed);
                falls += r.fell; maxT = std::fmax(maxT, r.maxTilt); rms += r.rmsTilt / 20; sp = std::fmax(sp, r.maxSpeed);
            }
            printf(" | %-14s falls %2d/20 maxTilt %5.1f rms %4.2f vmax %.2f", names[v], falls, maxT, rms, sp);
        }
        printf("\n");
    }
}
