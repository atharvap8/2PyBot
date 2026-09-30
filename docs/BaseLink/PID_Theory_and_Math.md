# LQR/LQI Control Theory and Implementation

The current balance controller is implemented in [BaseLink.ino](../../firmware/BaseLink/BaseLink.ino). It uses state feedback with a position integral. Earlier PID control is covered in [project history](Project_History.md).

## State and units

| Symbol | Firmware value | Units |
| :--- | :--- | :--- |
| x | `posM`, encoder-average position | m |
| x_ref | `xRef`, position reference | m |
| v | `velF`, filtered encoder velocity | m/s |
| theta | `pitchF * DEG_TO_RAD` | rad |
| omega | `rateF * DEG_TO_RAD` | rad/s |
| z | `zInt`, integral of position error | m·s |
| u | `uOut`, requested wheel acceleration | m/s² |

Pitch and pitch rate are measured independently. The rate term comes from the gyro, not a numerical derivative of filtered pitch. Wheel position comes from MT6816 feedback, not generated step counts.

## Normal position hold

Let the position error be limited by `P.exClamp`:

$$e_x = \operatorname{clip}(x-x_{ref},-e_{max},e_{max})$$

The integral is accumulated and limited by `P.zIntLim`:

$$z_{next} = \operatorname{clip}(z+e_x\Delta t,-z_{max},z_{max})$$

The controller requests acceleration:

$$u = -(K_1e_x + K_2v + K_3\theta + K_4\omega + K_5z)$$

The K1 to K5 terms respectively act on position, velocity, pitch, pitch rate, and position integral. The integral compensates sustained position error. It is not an integral of pitch error.

Compiled normal gains:

| Gain | Default |
| :--- | :--- |
| K1 | -7.0547 |
| K2 | -6.5334 |
| K3 | -34.5881 |
| K4 | -4.2854 |
| K5 | -2.1082 |

These negative gains are used with the leading minus sign shown above. Their effect depends on the sensor and motor sign conventions. NVS settings can replace the defaults.

## Velocity tracking

When forward input exceeds `DRIVEDB`, normal driving switches to velocity tracking:

$$v_{in} = cmdFwd\,P.maxDriveV$$
$$u = -(K_2(v-v_{in}) + K_3\theta + K_4\omega)$$

`zInt` is zeroed. The hold reference moves to `posM + velF * P.brakeLook`, providing a braking lookahead when input returns to zero. Joystick input requests velocity rather than a target lean angle.

## Stiff hold and climb

Stiff hold uses S1 to S5 with the same five-state expression while not driving. Active drive input selects the normal velocity branch instead.

Climb mode runs before the driving test. It ramps the position reference with `cmdFwd * P.climbVel`, confines the reference within `P.exClamp` of the robot, and uses C1 to C5 with a wider integral bound `P.climbZLim`. Steering authority is multiplied by `P.climbStr`.

Changing mode or gains does not establish a guaranteed slope limit. Traction, motor torque, geometry, supply voltage, and acceleration headroom determine the hardware envelope.

## Acceleration, velocity, and saturation

$$u_{limited}=\operatorname{clip}(u,-a_{max},a_{max})$$
$$v_{cmd,next}=\operatorname{clip}(v_{cmd}+u_{limited}\Delta t,-v_{max},v_{max})$$

Above 90% of the velocity ceiling, the reference moves toward measured position using `VSATBLD`, and the integral decays. This gives up position error rather than keeping an unreachable hold target.

Conversions are recomputed in `params_recompute()`:

$$stepsPerM=\frac{200\times8}{2\pi R}$$
$$countsPerM=\frac{4096}{2\pi R}$$
$$v_{max}=\frac{MAXSPD}{stepsPerM},\qquad a_{max}=\frac{MAXACC}{stepsPerM}$$

For the current 0.050 m radius, 8500 microsteps/s speed limit, and 20000 microsteps/s² acceleration limit, these give approximately 5093 steps/m, 13038 counts/m, 1.67 m/s, and 3.93 m/s². The default drive request limit is 0.70 m/s.

## Terrain gain adaptation

`terrain.h` estimates roughness from acceleration-magnitude error and pitch rate:

$$roughInput=|\|a\|-1|+0.35\frac{|pitchRate|}{400}$$

An EMA with `TERRTAU` smooths the estimate. Roughness progressively reduces the gain multiplier from 1 to `TERRSOFT`, default 0.55. Confirmed airborne and landing states use the full softening value.

- Normal hold and velocity tracking multiply K3/K4 by this value.
- Climb multiplies C3/C4.
- Stiff-hold S3/S4 remain unchanged.
- `AIRFRZ` suppresses integral accumulation in hold and climb while airborne. Saturation bleed remains active; driving still resets the integral.

## Heading control

Heading is represented by right-minus-left encoder counts. With no steering command:

$$steerSteps=\operatorname{clip}(dErr\,P.yawKp-diffRateF\,P.yawKd,-P.yawMaxSt,P.yawMaxSt)$$

The look command adds a heading offset converted through track width and counts per metre. Large differential error or excessive pitch re-latches heading. Compass yaw is telemetry, not the heading-control measurement.

The wheel rates are:

$$left=v_{cmd}\,stepsPerM-steerSteps$$
$$right=v_{cmd}\,stepsPerM+steerSteps$$

The common velocity is constrained by `P.vMax`; the final per-wheel rates are clamped to the 20 kHz timer limit by `setSpeeds()`.

## Filtering

The Mahony estimate uses accelerometer and gyro feedback. Two cascaded pitch EMAs use:

$$\alpha=1-e^{-2\pi f_c/200}$$

`IMUCUT` sets the pitch cutoff, default 50 Hz. Encoder velocity and differential rate use a 15 Hz EMA. Lower cutoffs reduce noise but add delay. There is no PID derivative-filter parameter in the current sketch.
