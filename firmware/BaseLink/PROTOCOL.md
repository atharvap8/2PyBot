# 2PyBot serial protocol (ESP32 <-> Radxa), v2

115200 8N1 over USB, newline-terminated ASCII. Everything the GUI and the
Android app need flows over this one link.

## ESP32 -> host (unsolicited streams)

| line | rate | meaning |
|---|---|---|
| `O,<ms>,<encL>,<encR>,<pitchDeg>,<yawDeg>` | 50 Hz | odometry, always on |
| `D,...` | on `L` toggle | human/machine debug stream |
| `[...]` | as they happen | free-text log lines (boot, faults) |

## Parameters — the GUI builds itself from these

| command | reply | meaning |
|---|---|---|
| `P?` | `P#,<n>` then n x `P,...` then `PR,...` then `P.` | dump everything |
| `P,<key>,<value>` | `P!,<key>,<value>` or `PE,<reason>` | set one, bounds-checked |
| `PS` | `PS!,saved` | persist all to NVS |
| `PL` | `PL!,...` | reload from NVS |
| `PD` | `PD!,defaults restored (not saved)` | compiled defaults |

Dump row format:

```
P,<key>,<value>,<min>,<max>,<group>,<description>
PR,<key>,<value>            # derived, read-only
P.                          # end of dump
```

`group` is one of BALANCE, STEPPER, DRIVE, YAW, SAFETY, IMU, CLIMB, LED,
PAYLOAD — **the GUI tabs come straight from this field**, so adding a
parameter in firmware makes it appear in the GUI with no GUI change.

Derived read-only values reported after every dump: `STEPSM`, `COUNTSM`,
`VMAX`, `AMAX`. Change `WHEELR` and all four update — the GUI should re-dump
after any set to show the consequences.

63 parameters at present: BALANCE 15, DRIVE 11, YAW 10, PAYLOAD 7, IMU 6,
SAFETY 5, STEPPER 4, CLIMB 3, LED 2.

## Host -> ESP32 (existing commands, unchanged)

`V,<fwd>,<steer>,<en>` drive authority · `E` enable · `X` e-stop ·
`C` cal gyro · `S` settings · `L` debug toggle · `R` reset ·
`G,<yes|no|spin|dance|stop>` gesture · `H,<0|1>` stiff hold ·
`A,<-1..1>` look · `K1=`..`K5=`, `T=`, `M=` legacy tuners (still work).

## Safety invariants the protocol preserves

1. Every `P,` write is range-checked in firmware. A bad value is refused with
   `PE,` and nothing changes. The GUI cannot push the robot outside its bounds.
2. `TORCH_MAX_PCT` is a compile-time hard cap in `payload.h` and is **not**
   exposed as a parameter.
3. A remote link may STOP the robot but never START it (see the arming fix).
