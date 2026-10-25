# AeroForge Units and Physical Conventions

> **Documentation status (October 2026):** Units and numerical conventions are the interface contract for interpreting model outputs and performance studies.
> **Evidence boundary:** Design statements describe intended behavior; current implementation and measured results are established by source, tests, and archived benchmark reports, not by this specification alone.


## 1. Purpose

This document defines AeroForge's authoritative internal units and physical quantity conventions.

The goal is to prevent unit ambiguity from entering the simulation core.

---

## 2. Internal Unit System

AeroForge uses **SI units internally**.

| Quantity | Internal Unit |
|---|---|
| Length / position | meter (`m`) |
| Time | second (`s`) |
| Velocity | meter per second (`m/s`) |
| Acceleration | meter per second squared (`m/s²`) |
| Mass | kilogram (`kg`) |
| Force | newton (`N`) |
| Moment / torque | newton-meter (`N·m`) |
| Angle | radian (`rad`) |
| Angular velocity | radian per second (`rad/s`) |
| Angular acceleration | radian per second squared (`rad/s²`) |
| Pressure | pascal (`Pa`) |
| Temperature | kelvin (`K`) |
| Density | kilogram per cubic meter (`kg/m³`) |
| Area | square meter (`m²`) |
| Volume | cubic meter (`m³`) |
| Mass flow | kilogram per second (`kg/s`) |
| Speed of sound | meter per second (`m/s`) |
| Dynamic pressure | pascal (`Pa`) |

---

## 3. Angle Convention

All physics, dynamics, control, and coordinate-transform calculations use radians.

Degrees may appear only at boundaries such as:

- human-readable configuration
- reports
- plots
- UI/visualization

Conversions must be explicit.

---

## 4. Time Convention

Simulation time is measured in seconds from the simulation epoch.

The initial epoch is normally:

```text
t = 0.0 s
```

at simulation start.

The physics clock is independent of wall-clock time.

For fixed-step simulation, the preferred conceptual representation is:

```text
stepIndex
timeStepSeconds
simulationTime = startTime + stepIndex * timeStepSeconds
```

This avoids relying exclusively on repeated floating-point addition for timekeeping.

---

## 5. Altitude Convention

The initial navigation frame is NED, so the navigation `Z` coordinate is positive downward.

For a ground-origin local frame:

\[
altitude = -z_N
\]

Code should distinguish clearly between:

- down-position coordinate
- altitude above the reference ground plane

They are not interchangeable names.

---

## 6. Temperature

Internal atmospheric temperature is stored in kelvin.

Celsius or Fahrenheit may be accepted/displayed only at external boundaries.

---

## 7. Pressure

Pressure is represented in pascals internally.

Values such as kPa may be used for display but must be converted before entering core models.

---

## 8. Floating-Point Precision

`double` is AeroForge's default scalar representation.

`float` should be introduced only for a documented interoperability or performance reason.

---

## 9. Unit Naming at API Boundaries

When a function boundary could plausibly be misunderstood, include the unit in the name.

Examples:

```cpp
setTimeStepSeconds(double timeStepSeconds);
setAltitudeMeters(double altitudeMeters);
```

Within strongly contextual internal code, excessively verbose names are not required.

---

## 10. Configuration Units

Initial AeroForge configuration files should use SI units unless a field explicitly declares otherwise.

The configuration schema/documentation must state units for every physical field.

A future UI may allow alternate user-facing units, but the parsed configuration passed into simulation code should already be normalized to SI.

---

## 11. Telemetry Units

Every telemetry channel must have a documented unit.

Telemetry metadata should make units discoverable without requiring the consumer to inspect source code.

---

## 12. Constants

Physical constants must:

- use explicit SI units
- have descriptive names
- be documented when their value depends on a standard or model
- avoid unexplained literals in equations

Example:

```cpp
constexpr double StandardGravityMetersPerSecondSquared = 9.80665;
```

A shorter name may be used where context and documentation already guarantee the unit.

---

## 13. Units Acceptance Criteria

- [x] SI selected as the internal unit system.
- [x] Radians selected for internal angles.
- [x] Kelvin selected for internal temperature.
- [x] Pascals selected for internal pressure.
- [x] `double` selected as the default scalar.
- [x] Simulation time uses seconds.
- [x] NED down-coordinate and altitude are explicitly distinguished.
- [x] Configuration fields must document their units.
- [x] Telemetry channels must document their units.
- [x] External unit conversions occur at boundaries, not inside core physics.
