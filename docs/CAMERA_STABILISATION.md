# Mechanical image stabilisation for 2PyBot

The best first addition is a **balanced, one-axis brushless pitch gimbal**:
the balancing chassis' pitch is the main low-frequency rotation. Add a roll
axis if uneven terrain produces significant side tilt. A dedicated gimbal
controller and camera-side IMU keep this loop separate from wheel balancing.

Recommended stack:

```text
short stiff chassis bracket → selected silicone isolators → pan carrier
→ two-sided bearing-supported pitch gimbal → rigid camera/ToF pod
```

- Put the pitch pivot through the pod centre of mass, support it on both sides,
  and balance it mechanically before powering the motor. Keep top mass low;
  added mass changes the balancer's dynamics and needs a balance check.
- Use four appropriately selected silicone bobbins/gimbal dampers for faster
  motor/wheel vibration. Very soft foam can introduce its own resonance and
  wobble; choose stiffness for the actual payload and measured vibration.
- Use a short ribbed or metal-backed bracket. Remove loose horns/bearing play,
  secure the M12 optics/PCB, and check tyre/hub concentricity and fasteners.
- Give the CSI ribbon a gentle flexible service loop near the pivot, respecting
  its bend radius. Cable tension must not pull the gimbal off target.
- Moving the camera nearer the wheel axle reduces pitch-induced translation;
  a rigidly attached camera still rotates by the chassis angle. The gimbal
  supplies the angular correction.
- Existing MG90S yaw/“zoom” mechanisms are not documented as an actively
  stabilised pitch/roll gimbal. Check their physical axes before repurposing;
  gear backlash/deadband limit smooth small-angle video stabilisation.

Keep camera and ToF rigidly together on the moving pod if future fusion matters.
Their relative transform stays fixed, but pod-to-base pose needs measured gimbal
angles/camera IMU timing. Moving only the camera makes camera–ToF extrinsics
time-varying; this is not yet implemented in the perception stack. Changing ToF
height changes ground projection. Lens intrinsics are separate: changing focal,
focus or zoom settings may require new intrinsic calibration.

Typical parts: small pitch gimbal motor/controller, camera IMU, balanced cradle,
two bearing supports, four selected silicone dampers, travel retainers, and a
flexible CSI loop. Approximately ±25–30° pitch clearance is a starting design
range, not a physically verified fit to the existing enclosure.
