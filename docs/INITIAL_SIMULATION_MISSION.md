# AeroForge Initial Simulation Mission

## 1. Purpose

This document defines the initial reference mission for AeroForge.

AeroForge is a general-purpose aerospace simulation, guidance, navigation, and control platform written in modern C++. The platform is intended to support deterministic, testable, and extensible simulation of aerospace vehicles.

The first reference vehicle will be a **single-stage guided research rocket**.

The rocket is not the final scope of AeroForge. It is the first vehicle used to exercise the simulation platform's major engineering capabilities, including rigid-body dynamics, propulsion, aerodynamics, sensors, actuators, guidance, navigation, control, uncertainty, failure injection, and Monte Carlo analysis.

---

## 2. Platform Objective

AeroForge is not intended to be only a rocket simulator.

The platform should remain vehicle-agnostic and eventually support additional aerospace vehicles such as:

- aircraft
- UAVs
- experimental flight vehicles
- other rigid-body aerospace systems

The initial rocket exists as the first reference implementation.

The long-term architectural relationship is:

```text
AeroForge Simulation Platform
            |
            +-- Research Rocket
            |
            +-- Aircraft
            |
            +-- UAV
            |
            +-- Future Vehicles
```

Simulation infrastructure, environmental models, numerical integration, telemetry, failure injection, and analysis should not be unnecessarily coupled to rocket-specific behavior.

---

## 3. Initial Reference Vehicle

The first fully modeled vehicle will be a:

**Single-stage guided research rocket**

The rocket should eventually exercise the following capabilities:

- six-degree-of-freedom rigid-body motion
- time-dependent thrust
- changing mass
- changing center of mass
- changing inertia properties
- atmospheric flight
- aerodynamic forces and moments
- wind disturbances
- simulated sensors
- simulated actuators
- navigation/state estimation
- guidance
- closed-loop attitude control
- failure injection
- deterministic replay
- Monte Carlo uncertainty analysis
- telemetry recording

A rocket is used first because it provides meaningful flight-dynamics and control problems without requiring the full system complexity of an aircraft.

---

## 4. Flagship Mission Statement

The eventual flagship mission is:

> Simulate a six-degree-of-freedom research rocket following a commanded ascent trajectory while compensating for atmospheric disturbances, sensor uncertainty, actuator limitations, and injected component failures.

The mission should demonstrate both the simulation platform and the behavior of the closed-loop guidance, navigation, and control system.

---

## 5. Nominal Flight Sequence

The nominal flight will progress through the following major phases:

```text
Pre-Launch
    |
    v
Ignition
    |
    v
Powered Ascent
    |
    v
Guided / Stabilized Ascent
    |
    v
Motor Burnout
    |
    v
Coast
    |
    v
Apogee
    |
    v
Descent
    |
    v
Simulation Termination
```

Recovery systems such as parachutes are not required for the initial flagship mission.

They may be introduced later if they provide useful additional simulation or control challenges.

---

## 6. Degrees of Freedom

The final reference rocket will use **six-degree-of-freedom (6DOF)** rigid-body dynamics.

The six degrees of freedom consist of:

### Translation

- X position
- Y position
- Z position

with corresponding velocity components.

### Rotation

- roll
- pitch
- yaw

The internal orientation representation should eventually use quaternions rather than Euler angles.

Euler angles may still be used for human-readable telemetry and visualization.

---

## 7. Attitude Control Objective

The rocket should eventually actively control:

- pitch
- yaw
- roll

The guidance system will determine the desired flight state or attitude.

The control system will attempt to minimize the difference between desired and estimated motion.

Example:

```text
Desired pitch: 82.0 deg
Actual pitch:  80.7 deg
Error:          1.3 deg
```

Angles may be displayed in degrees, but AeroForge physics and control calculations use radians internally.

---

## 8. Control Architecture

The controller must not directly modify the rocket's orientation or state.

The required architecture is:

```text
Guidance
    |
    v
Desired State / Attitude

Controller
    |
    v
Actuator Command

Actuator Model
    |
    v
Physical Actuator State

Force / Moment Generation
    |
    v
Vehicle Dynamics

Numerical Integrator
    |
    v
Updated Vehicle State
```

This separation is a core AeroForge architectural requirement.

---

## 9. Initial Active Control Mechanism

The first active control mechanism should be **thrust vector control (TVC)**.

The rocket motor's thrust direction will be allowed to change within physical actuator limits.

Conceptually:

```text
Rocket Motor
     |
     v
   Thrust
     \
      \
       Gimbaled Thrust
```

The offset thrust direction generates control moments that rotate the vehicle.

Aerodynamic control surfaces may be added later.

---

## 10. Environment Model

The flagship simulation should eventually include the following environmental effects.

### 10.1 Gravity

Initial implementation:

- constant gravitational acceleration

Later implementation:

- altitude-dependent gravity

### 10.2 Atmosphere

The atmosphere should eventually provide:

- temperature
- pressure
- density
- speed of sound

as functions of altitude.

### 10.3 Wind

The environment should eventually support:

- constant wind
- altitude-dependent wind
- gust disturbances

The flagship mission should include at least one intentionally disruptive lateral gust or comparable disturbance.

The control system should demonstrate recovery toward the commanded state.

---

## 11. Propulsion Model

The rocket motor must eventually use a **time-dependent thrust curve**.

The architecture should support:

\[
T = T(t)
\]

rather than assuming constant thrust.

Example conceptual thrust data:

```text
Time        Thrust
0.00 s         0 N
0.05 s      1800 N
0.20 s      3400 N
1.50 s      3100 N
2.50 s      1200 N
2.70 s         0 N
```

The final thrust curve values will be defined during propulsion-model development.

---

## 12. Changing Vehicle Properties

The rocket's properties must be allowed to change during powered flight.

The total mass follows:

\[
m = m(t)
\]

The vehicle model should eventually distinguish among:

- dry mass
- propellant mass
- total mass

Propellant consumption may also affect:

- center of mass
- inertia tensor

These values should evolve according to the vehicle model rather than being treated as permanently constant.

---

## 13. Sensor Suite

The initial reference vehicle should eventually include at least:

### IMU

- accelerometer
- gyroscope

### GPS

- position
- velocity

### Barometric Altimeter

- altitude-related measurement

The closed-loop GNC system must not receive perfect truth-state data in the final flagship mission.

The required information flow is:

```text
True Simulation State
        |
        v
Sensor Models
        |
        v
Noisy / Delayed Measurements
        |
        v
Navigation / State Estimation
        |
        v
Estimated State
        |
        v
Guidance and Control
```

Truth-state access may still be used for validation, telemetry, debugging, and analysis.

---

## 14. Sensor Imperfections

Sensor models should eventually support configurable imperfections such as:

- Gaussian noise
- constant bias
- bias drift
- latency
- update rate
- saturation
- dropout

Example:

```text
True angular velocity:
0.402 rad/s

Measured angular velocity:
0.409 rad/s
```

The controller should operate on the estimated state derived from simulated measurements rather than perfect truth.

---

## 15. Actuator Imperfections

Actuator models should eventually support:

- maximum deflection
- minimum deflection
- rate limits
- latency
- deadband
- saturation
- degradation
- failure

Example:

```text
Controller command:
TVC pitch = 8 deg

Current actuator state:
TVC pitch = 3 deg

Maximum actuator rate:
25 deg/s
```

The actuator must require physical simulation time to respond to commands.

---

## 16. Navigation

Navigation answers:

> Where is the vehicle, and what is its current motion?

The initial implementation may use simple state-estimation techniques.

The architecture should allow later introduction of more sophisticated estimators such as an **Extended Kalman Filter (EKF)**.

The intended architecture is:

```text
True State
    |
    v
Sensors
    |
    v
Navigation Filter
    |
    v
Estimated State
```

Navigation must remain a separate subsystem from guidance and control.

---

## 17. Guidance

Guidance answers:

> Where should the vehicle go?

The first guidance implementation may use a predetermined schedule.

Example:

```text
Time        Desired Pitch
0 s             90 deg
2 s             88 deg
5 s             82 deg
10 s            72 deg
```

Later guidance implementations may respond dynamically to:

- position
- velocity
- trajectory error
- mission objectives
- environmental conditions

The initial mission does not require optimal guidance.

---

## 18. Control

Control answers:

> What actuator command should be issued to reduce the difference between the estimated state and the desired state?

The first controller should be a **PID attitude controller**.

Conceptually:

\[
e(t) = \theta_{desired} - \theta_{estimated}
\]

and:

\[
u(t) =
K_P e(t)
+
K_I \int e(t)\,dt
+
K_D \frac{de(t)}{dt}
\]

Separate control channels should eventually support:

- pitch
- yaw
- roll

More advanced control techniques may be introduced later.

---

## 19. Disturbances and Uncertainty

A perfect nominal trajectory is not sufficient for the flagship demonstration.

The simulation should eventually support uncertainty in parameters such as:

- vehicle mass
- motor thrust
- aerodynamic coefficients
- wind
- launch orientation
- sensor bias
- sensor noise
- actuator delay

The mission should also support explicit disturbances or failures such as:

- lateral wind gust
- temporary GPS loss
- increased gyro bias
- partial actuator degradation

The key engineering question is:

> Can the vehicle remain within mission constraints despite uncertainty and failures?

---

## 20. Mission Success

Mission success must be defined quantitatively.

The simulator should not rely on subjective statements such as:

> The rocket looked stable.

A mission may eventually evaluate metrics such as:

```text
Maximum attitude error
Maximum trajectory error
Maximum angle of attack
Maximum dynamic pressure
Maximum actuator utilization
Target altitude error
Navigation error
```

Example report:

```text
Mission Success: YES

Maximum attitude error:       3.2 deg
Maximum trajectory error:    21.7 m
Maximum angle of attack:      6.1 deg
Maximum dynamic pressure:    43.2 kPa
Maximum actuator command:    71 percent
Target altitude error:       14.5 m
```

Final thresholds will be established only after the relevant physical models and reference mission are implemented.

---

## 21. Monte Carlo Mission Evaluation

The flagship mission should eventually support repeated randomized simulation runs.

Example:

```text
Simulations:            10,000
Successful missions:     9,842
Failed missions:           158
Mission success rate:     98.42 percent
```

Failures should be classified where possible.

Potential failure categories include:

- control instability
- navigation failure
- actuator saturation
- excessive trajectory error
- excessive angle of attack
- mission-limit violation

Monte Carlo analysis is intended to demonstrate both simulation robustness and engineering analysis capability.

---

## 22. Telemetry Requirements

Each simulation should eventually record sufficient telemetry to reconstruct and analyze the flight.

Planned telemetry includes:

- simulation time
- position
- velocity
- acceleration
- orientation
- angular velocity
- mass
- center of mass
- thrust
- aerodynamic forces
- aerodynamic moments
- air density
- wind
- Mach number
- dynamic pressure
- angle of attack
- sensor measurements
- estimated state
- guidance commands
- controller outputs
- actuator states
- failure events

Telemetry should remain separate from the core physics logic.

---

## 23. Flagship Demonstration

The long-term AeroForge flagship demonstration is:

### AeroForge Autonomous Research Rocket Simulation

```text
6DOF Research Rocket
        |
        v
Variable Mass
        |
        v
Time-Dependent Rocket Motor
        |
        v
Atmospheric Flight
        |
        v
Aerodynamic Forces and Moments
        |
        v
Wind and Gust Disturbances
        |
        v
Noisy Sensors
        |
        v
Navigation Estimator
        |
        v
Trajectory Guidance
        |
        v
PID Attitude Control
        |
        v
Limited TVC Actuator
        |
        v
Failure Injection
        |
        v
Monte Carlo Uncertainty Analysis
```

The final demonstration should report mission-level performance, not merely produce an animation.

---

## 24. Development Progression

The reference vehicle should be developed incrementally.

The intended progression is:

```text
Point Mass
    |
    v
3DOF Projectile
    |
    v
3DOF Rocket
    |
    v
6DOF Rigid Body
    |
    v
6DOF Powered Rocket
    |
    v
6DOF Aerodynamic Rocket
    |
    v
Sensors
    |
    v
Actuators
    |
    v
Navigation
    |
    v
Guidance
    |
    v
Control
    |
    v
Failures
    |
    v
Monte Carlo
```

Each stage should be validated before additional model complexity is introduced.

---

## 25. Explicit Initial Exclusions

The following capabilities are outside the initial project scope:

- orbital mechanics
- multi-stage launch vehicles
- flexible-body dynamics
- structural finite-element analysis
- computational fluid dynamics
- detailed combustion simulation
- Earth rotation effects
- relativistic effects
- full terrain simulation
- photorealistic graphics
- air-traffic simulation

These exclusions are intended to control scope rather than permanently prohibit future expansion.

---

## 26. Engineering Principles

The initial mission should reinforce the following AeroForge principles:

1. Build the simulation core before visualization.
2. Keep truth state separate from estimated state.
3. Keep guidance, navigation, and control separate.
4. Controllers generate commands; they do not directly modify state.
5. Actuators have physical dynamics and limits.
6. Environmental models remain independent of specific vehicles.
7. Physics models should be independently testable.
8. Mission success should be quantitative.
9. Randomness should be deterministic when seeded.
10. New model complexity should be added only after lower-level behavior is validated.

---

## 27. Phase 0.5 Decisions

The following decisions are considered the Phase 0.5 baseline:

| Area | Decision |
|---|---|
| First reference vehicle | Single-stage guided research rocket |
| Platform scope | General-purpose aerospace simulation and GNC |
| Final reference dynamics | 6DOF rigid body |
| Primary initial active control | Thrust vector control |
| Initial sensors | IMU, GPS, barometric altimeter |
| Controller truth-state access | Not allowed in final flagship mission |
| Navigation / Guidance / Control | Separate subsystems |
| Initial controller | PID attitude control |
| Environment | Gravity, atmosphere, wind |
| Propulsion | Time-dependent thrust curve |
| Vehicle mass | Changes during powered flight |
| Sensor imperfections | Modeled |
| Actuator imperfections | Modeled |
| Failure injection | Required long-term capability |
| Mission evaluation | Quantitative |
| Monte Carlo | Required flagship capability |
| Telemetry | Required |
| Visualization | Secondary to simulation correctness |

---

## 28. Phase 0.5 Acceptance Criteria

Phase 0.5 is complete when the project has formally established that:

- [x] The first reference vehicle is a research rocket.
- [x] AeroForge remains vehicle-agnostic.
- [x] The final reference vehicle uses 6DOF rigid-body dynamics.
- [x] Thrust vector control is the initial active-control mechanism.
- [x] The planned sensor suite includes an IMU, GPS, and barometric altimeter.
- [x] GNC does not rely on perfect truth-state information in the flagship mission.
- [x] Guidance, navigation, and control are separate responsibilities.
- [x] PID attitude control is the initial controller.
- [x] Gravity, atmosphere, and wind are part of the planned environment.
- [x] Propulsion supports time-dependent thrust.
- [x] Vehicle mass changes during powered flight.
- [x] Sensor and actuator imperfections are part of the model.
- [x] Failure injection is part of the eventual mission.
- [x] Mission success is quantitatively evaluated.
- [x] Monte Carlo analysis is part of the flagship demonstration.
- [x] Telemetry is recorded.
- [x] Initial out-of-scope physics have been identified.
- [x] The mission specification is under version control.

---

## 29. Revision Policy

The reference mission may evolve as AeroForge becomes more capable.

Changes should preserve the following core intent:

- the platform remains reusable beyond one rocket
- each physical model can be validated
- closed-loop GNC operates on simulated measurements
- actuator and sensor limitations are represented
- mission outcomes are measurable
- uncertainty and failure behavior can be reproduced and analyzed

Significant changes to the reference mission should be documented in this file.
