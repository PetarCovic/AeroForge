# AeroForge Simulation Scope and Assumptions

> **Documentation status (October 2026):** This is the original detailed fidelity/scope specification, retained for model limitations and engineering tradeoffs. Forward-looking language and fidelity levels are design targets—not a claim that every feature is complete.
> **Evidence boundary:** Design statements describe intended behavior; current implementation and measured results are established by source, tests, and archived benchmark reports, not by this specification alone.


## 1. Purpose

This document defines the physical scope, modeling assumptions, fidelity strategy, and explicit limitations for AeroForge.

AeroForge should evolve from mathematically simple, highly verifiable models into progressively more realistic aerospace simulation models. The project does **not** attempt to maximize physical fidelity from the beginning.

> Prefer a simple, explicitly documented, and verified model over a complex model whose correctness has not been demonstrated.

---

## 2. Progressive Fidelity Philosophy

AeroForge uses progressively replaceable physical models.

```text
GravityModel
├── ConstantGravity
└── AltitudeDependentGravity

AtmosphereModel
├── VacuumAtmosphere
├── ConstantAtmosphere
└── StandardAtmosphere

WindModel
├── NoWind
├── ConstantWind
├── LayeredWind
└── GustWind
```

This allows simple analytical validation, incremental complexity, controlled model comparisons, easier testing, and future model replacement without rewriting the simulator.

Complexity should be introduced only when it is needed to answer an engineering question or improve required accuracy.

---

## 3. Rigid-Body Assumption

The initial AeroForge vehicle model assumes the vehicle behaves as a **rigid body**.

The vehicle may translate and rotate, but its structure does not deform.

Initially excluded effects include:

- structural bending
- structural twisting
- fin deformation
- structural vibration
- aeroelastic deformation
- propellant slosh
- flexible-body modes

Baseline:

```text
Vehicle = rigid body
```

Flexible-body effects may be considered as a future extension.

---

## 4. Initial Earth Model

AeroForge initially uses a **flat, local Earth model**.

The simulation operates in a local Cartesian region rather than modeling the full curved Earth. This assumes reference missions occur over sufficiently limited distances and altitudes that Earth curvature can initially be neglected.

The initial model therefore does not require latitude, longitude, spherical Earth geometry, geodetic coordinate transformations, or global navigation frames.

Possible later extensions include:

```text
Local Cartesian
      ↓
ECEF
      ↓
ECI
```

---

## 5. Earth Rotation

Earth rotation is initially ignored.

The simulator therefore does not initially model:

- Coriolis acceleration
- centrifugal acceleration
- Earth angular velocity
- rotational-frame effects caused by Earth rotation

Baseline:

```text
Earth rotation = ignored
```

A rotating-Earth model may be introduced later if the operational domain requires it.

---

## 6. Gravity

### 6.1 Initial Gravity Model

The first gravity model uses constant gravitational acceleration:

\[
g_0 = 9.80665\ \mathrm{m/s^2}
\]

The initial model assumes constant gravity magnitude and direction, with no latitude variation, altitude variation, or Earth-rotation effects.

This model is especially useful for analytical projectile and free-fall validation.

### 6.2 Future Gravity Model

A later gravity model may include altitude dependence:

\[
g(h)
=
g_0
\left(
\frac{R_E}{R_E+h}
\right)^2
\]

The dynamics architecture must therefore avoid treating gravity as a permanently hard-coded scalar.

---

## 7. Atmosphere

AeroForge should support multiple atmospheric fidelity levels.

### Level 0 — Vacuum

```text
density = 0
pressure = 0
```

Used primarily for projectile validation, integrator verification, and force-free motion tests.

### Level 1 — Constant Atmosphere

The atmosphere may initially use constant density, pressure, and temperature. This is useful for validating aerodynamic force models independently of altitude variation.

### Level 2 — Standard Atmosphere

A later atmospheric model should provide altitude-dependent:

- temperature
- pressure
- density
- speed of sound

The implementation should follow a recognized standard-atmosphere formulation rather than an arbitrary custom approximation.

The atmosphere is initially assumed to be horizontally uniform, primarily altitude-dependent, and deterministic unless explicitly configured otherwise.

---

## 8. Weather

AeroForge is not a meteorological simulation platform.

### Planned environmental disturbance support

- deterministic wind
- altitude-dependent wind
- configurable gusts
- stochastic wind disturbances

### Not initially modeled

- thunderstorms
- weather fronts
- cloud physics
- precipitation
- humidity fields
- detailed local temperature fields
- real-time weather forecasting
- full atmospheric fluid simulation

The intended focus is flight dynamics rather than atmospheric science.

---

## 9. Wind

Wind is conceptually separate from the atmosphere model.

The atmosphere model provides thermodynamic properties such as density, pressure, temperature, and speed of sound. The wind model provides air velocity.

Planned progression:

```text
NoWind
   ↓
ConstantWind
   ↓
AltitudeDependentWind
   ↓
GustModel
```

---

## 10. Air-Relative Velocity

Aerodynamic forces depend on vehicle motion relative to the surrounding air.

\[
\mathbf{V}_{air}
=
\mathbf{V}_{vehicle}
-
\mathbf{V}_{wind}
\]

Ground-relative velocity and air-relative velocity are therefore distinct quantities.

This distinction becomes mandatory once wind is introduced.

---

## 11. Vehicle Mass

Early mathematical validation scenarios may assume:

```text
mass = constant
```

The powered rocket model will later support:

\[
m = m(t)
\]

The vehicle model should eventually distinguish dry mass, propellant mass, and total mass.

Mass changes must arise from a defined vehicle or propulsion model rather than arbitrary changes inside the dynamics integrator.

---

## 12. Center of Mass

The initial rigid-body validation model may assume:

```text
center of mass = fixed
```

Once propellant consumption is introduced, AeroForge should support a varying center of mass.

The dynamics architecture must not assume the center of mass is permanently fixed.

---

## 13. Inertia Tensor

The eventual six-degree-of-freedom vehicle model includes an inertia tensor:

\[
\mathbf{I}
\]

Initial 6DOF validation may use a constant inertia tensor. Later powered-flight models may support variable inertia as propellant mass and distribution change.

Mass properties should eventually be represented as a coherent subsystem:

```text
MassProperties
├── mass
├── centerOfMass
└── inertiaTensor
```

---

## 14. Propellant Behavior

The initial propulsion model does not simulate detailed internal propellant fluid behavior.

Not initially modeled:

- propellant slosh
- fluid free-surface dynamics
- combustion chemistry
- chamber-flow CFD
- detailed thermal combustion physics
- internal structural deformation

Instead, the propulsion model supplies higher-level engineering quantities such as thrust, mass flow, and remaining propellant.

---

## 15. Propulsion Fidelity

Propulsion should evolve through distinct levels.

### Level 0 — No Propulsion

Used for free fall, projectile motion, and integrator validation.

### Level 1 — Constant Thrust

Used for analytical acceleration tests and simple powered-flight validation.

### Level 2 — Time-Dependent Thrust

The rocket motor eventually supports:

\[
T = T(t)
\]

through a thrust curve or equivalent representation.

### Possible Future Extensions

- pressure-dependent thrust
- temperature effects
- thrust uncertainty
- more detailed motor models

Detailed combustion simulation is not an initial AeroForge goal.

---

## 16. Aerodynamic Fidelity

Aerodynamics should be introduced progressively.

### Initial Model

Drag only:

\[
F_D
=
\frac{1}{2}
\rho V^2 C_D A
\]

### Later Extensions

- lift
- side force
- rolling moment
- pitching moment
- yawing moment
- angle-of-attack effects
- sideslip effects
- Mach-dependent coefficients

The project does not initially attempt computational fluid dynamics.

---

## 17. Aerodynamic Coefficients

Early validation may use constant coefficients such as `Cd`, `Cl`, and `Cm`.

Later models may allow coefficients to depend on state variables:

\[
C_D = C_D(M,\alpha)
\]

Possible progression:

```text
ConstantCoefficientModel
            ↓
FunctionalCoefficientModel
            ↓
LookupTableCoefficientModel
```

This keeps coefficient fidelity replaceable rather than embedding assumptions directly into rigid-body dynamics.

---

## 18. Compressibility and Mach Number

The rocket may eventually enter flight regimes where compressibility matters.

AeroForge should therefore support Mach number:

\[
M = \frac{V}{a}
\]

where `V` is air-relative speed and `a` is local speed of sound.

Early projectile validation does not require detailed compressible-flow effects.

Planned progression:

```text
Simple Drag
    ↓
Mach Calculation
    ↓
Mach-Dependent Aerodynamic Coefficients
```

AeroForge does not initially simulate shock waves or compressible-flow CFD.

---

## 19. Thermal Effects

The initial simulator ignores:

- aerodynamic heating
- motor heating
- thermal structural response
- material temperature evolution

Environmental temperature may later be recorded or used by specific models.

Full thermal analysis is outside the initial simulation scope.

---

## 20. Structural Mechanics

AeroForge does not initially perform structural analysis from first principles.

Not initially modeled:

- stress
- strain
- buckling
- structural fracture
- finite-element structures
- material deformation

Simplified engineering constraints may later be monitored, including maximum dynamic pressure, acceleration, angular rate, or load factor.

A mission may be marked as failed if a configured engineering limit is exceeded.

---

## 21. Ground Model

The initial ground is represented as a flat plane.

A simple condition such as:

```text
altitude <= ground altitude
```

may indicate ground contact or mission termination.

Not initially modeled:

- terrain meshes
- mountains
- buildings
- runway surface detail
- soil mechanics
- detailed landing interaction

Visualization may later show terrain without making it part of the physical ground model.

---

## 22. Collision Physics

AeroForge is not initially a general-purpose collision simulator.

Not initially modeled:

- vehicle-to-vehicle collisions
- arbitrary mesh collision
- fragmentation
- structural impact response
- debris dynamics

A ground-impact event may simply terminate the simulation.

---

## 23. Orientation Representation

The final 6DOF state uses **quaternions internally** for orientation.

Conceptually:

\[
q = [w,x,y,z]
\]

Euler angles remain useful for telemetry, plotting, debugging, visualization, and human-readable configuration.

Euler angles should not be the authoritative internal 6DOF orientation representation.

---

## 24. Translational State

The eventual translational rigid-body state includes:

\[
\mathbf{r}
\]

for position and:

\[
\mathbf{v}
\]

for velocity.

The exact reference frame and axis conventions are defined separately in the coordinate-system specification.

---

## 25. Rotational State

The eventual rotational state includes:

- orientation quaternion
- angular velocity vector

The exact frame used for angular velocity and the quaternion direction convention must be defined in the coordinate-system specification.

---

## 26. Simulation Time

Simulation time is authoritative for all physics calculations.

Physics must not depend on wall-clock frame timing.

A given simulated duration may execute faster than real time, at real time, or slower than real time without changing the mathematical simulation result.

---

## 27. Initial Timestep Strategy

The initial numerical integrators use a **fixed timestep**.

\[
\Delta t = \text{configurable constant}
\]

AeroForge must not assume one universal timestep value.

Timestep adequacy will later be evaluated through convergence studies.

Possible progression:

```text
Fixed Step
├── Euler
├── RK2
└── RK4

Adaptive Step
└── RK45
```

Adaptive integration is a future capability, not an initial requirement.

---

## 28. Deterministic Execution

Determinism is a core AeroForge requirement.

Given the same:

- initial state
- configuration
- physics models
- numerical integrator
- timestep
- random seed

AeroForge should produce the same simulation result, subject to explicitly documented compiler and platform floating-point limitations.

This requirement affects sensor noise, actuator noise, wind disturbances, failure injection, Monte Carlo simulation, parallel execution, telemetry, and debugging.

---

## 29. Truth State

AeroForge maintains an authoritative **truth state**.

This is the mathematically integrated physical state of the simulated vehicle.

Truth state may be used by:

- physics models
- verification
- telemetry
- debugging
- analysis

Once sensor and GNC simulation is introduced, the controller must not directly consume perfect truth state in the flagship scenario.

```text
Truth State
    ↓
Sensors
    ↓
Navigation / Estimator
    ↓
Estimated State
    ↓
Guidance / Control
```

---

## 30. Sensor Timing

Sensors may operate at rates different from the physics integrator.

Conceptual example:

```text
Physics:      1000 Hz
IMU:           200 Hz
GPS:            10 Hz
Barometer:      50 Hz
```

These values are illustrative only.

The architectural requirement is that subsystems are not required to update once per physics timestep.

---

## 31. Actuator Timing

Actuator dynamics are separate from control commands.

A commanded actuator state is not assumed to equal the physical actuator state instantaneously.

Actuator models may eventually include:

- response delay
- rate limits
- saturation
- deadband
- position limits

```text
Controller Command
        ↓
Actuator Dynamics
        ↓
Physical Actuator State
```

---

## 32. Sensor Physical Detail

Sensors are initially behavioral engineering models rather than low-level hardware simulations.

An IMU model may eventually represent:

```text
true physical quantity
+ bias
+ noise
+ drift
+ saturation
+ latency
```

AeroForge does not initially model MEMS internal mechanics, semiconductor physics, electronic circuit behavior, or transistor-level ADC behavior.

---

## 33. Actuator Physical Detail

Actuator models also operate at an engineering abstraction level.

A TVC actuator may model command, position, rate limit, position limit, delay, response, and failure.

It does not initially model individual motor windings, detailed gear-tooth contact, transistor switching, or full electromechanical field behavior.

---

## 34. Initial GNC Scope

The initial GNC roadmap is deliberately simple.

### Navigation

Initial:

```text
simple state estimation
```

Later:

```text
possible EKF or other estimator
```

### Guidance

Initial:

```text
prescribed trajectory or attitude schedule
```

Later:

```text
state-dependent guidance
```

### Control

Initial:

```text
PID
```

Later:

```text
possible advanced control techniques
```

Advanced GNC should not be introduced before the underlying physics models have been validated.

---

## 35. Initial Operational Domain

The initial AeroForge operational domain is:

**Local atmospheric flight**

The architecture therefore prioritizes:

- atmospheric effects
- aerodynamics
- wind
- local gravity
- rigid-body attitude dynamics
- propulsion
- sensors and control

The initial scope does not prioritize orbital flight, interplanetary flight, orbital perturbations, or celestial-body ephemerides.

Orbital functionality may become a future extension.

---

## 36. Model Naming and Fidelity Claims

AeroForge documentation should describe physical models precisely rather than using vague claims such as:

```text
high fidelity
realistic physics
advanced atmosphere
```

Prefer explicit descriptions such as:

```text
constant-density atmosphere
altitude-dependent gravity
standard-atmosphere model
Mach-dependent drag coefficient
fixed-step RK4 integrator
```

Fidelity claims should be supported by the implemented model and its validation evidence.

---

## 37. Model Validity Documentation

Every significant physics model should eventually document:

- model name
- purpose
- assumptions
- inputs
- outputs
- units
- coordinate frame
- valid operating domain
- limitations
- validation method
- references where applicable

Example:

```text
Model:
ConstantGravity

Assumptions:
- uniform gravitational field
- fixed gravity direction

Input:
vehicle position

Output:
gravity acceleration vector

Units:
m/s^2

Valid Use:
local and low-altitude validation scenarios

Limitations:
- no altitude variation
- no latitude variation
- no Earth rotation
```

---

## 38. Complexity Rule

A more complicated physical model should be introduced only when:

- the current model prevents a required engineering analysis
- the current model produces unacceptable error
- the next project milestone explicitly requires higher fidelity
- validation data exists or can be produced for the more complex model

Example:

```text
Vacuum
    ↓
Validate Translational Dynamics

Constant Atmosphere
    ↓
Validate Drag

Standard Atmosphere
    ↓
Analyze Altitude Effects
```

Complexity is earned through validation.

---

## 39. Verification Before Fidelity

AeroForge prioritizes:

```text
simple + verified
```

over:

```text
complex + unverified
```

Increasing model fidelity must not eliminate lower-fidelity validation scenarios.

---

## 40. Fidelity Ladder

### Fidelity Level 0 — Mathematical Validation

```text
Point mass
Flat Earth
Constant gravity
Vacuum
Constant mass
No rotation
No propulsion
```

Primary uses:

- free fall
- projectile motion
- integrator validation
- numerical convergence studies

### Fidelity Level 1 — Basic Atmospheric Vehicle

```text
Point mass
Constant gravity
Simple atmosphere
Drag
Wind
```

Primary uses:

- 3DOF aerodynamic validation
- air-relative velocity testing

### Fidelity Level 2 — Powered Vehicle

```text
3DOF
Variable mass
Thrust
Atmosphere
Drag
Wind
```

Primary use:

- basic rocket trajectory simulation

### Fidelity Level 3 — 6DOF Rigid Body

```text
Translation
Rotation
Quaternion orientation
Angular velocity
Inertia tensor
Applied forces
Applied moments
```

Primary use:

- rigid-body dynamics verification

### Fidelity Level 4 — Full Rocket Flight Model

```text
6DOF
Variable mass properties
Aerodynamic forces and moments
Propulsion
Atmosphere
Wind
```

Primary use:

- open-loop rocket flight dynamics

### Fidelity Level 5 — Closed-Loop Vehicle

```text
Sensors
Navigation
Guidance
Control
Actuators
```

Primary use:

- closed-loop GNC analysis

### Fidelity Level 6 — Robustness Simulation

```text
Noise
Uncertainty
Failures
Monte Carlo
Deterministic replay
```

Primary use:

- robustness and mission-success analysis

---

## 41. Scope Summary

| Physical Effect | Initial | Later |
|---|---|---|
| Rigid body | Yes | Yes |
| Flexible body | No | Possible |
| Flat Earth | Yes | Replaceable |
| Earth curvature | No | Possible |
| Earth rotation | No | Possible |
| Constant gravity | Yes | Yes |
| Altitude-dependent gravity | No | Planned |
| Vacuum | Yes, for validation | Yes |
| Atmosphere | Simple | Standard atmosphere |
| Wind | Simple | Layered / gusts |
| Weather simulation | No | Not planned |
| Constant mass | Yes, validation | Yes |
| Variable mass | No initially | Planned |
| Variable center of mass | No initially | Planned |
| Variable inertia | No initially | Planned |
| Constant thrust | Validation model | Yes |
| Thrust curve | No initially | Planned |
| Combustion physics | No | Not planned |
| Drag | Planned early | Yes |
| Lift / side force | No initially | Planned |
| Aerodynamic moments | No initially | Planned |
| CFD | No | Not planned |
| Quaternion attitude | At 6DOF stage | Yes |
| Structural deformation | No | Not initially |
| Structural limit monitoring | No initially | Planned |
| Flat ground | Yes | Yes initially |
| Terrain physics | No | Not initially |
| General collision physics | No | Not initially |
| Sensors | No initially | Planned |
| Sensor noise / bias | No initially | Planned |
| Actuator dynamics | No initially | Planned |
| PID control | No initially | Planned |
| EKF | No initially | Possible |
| Determinism | Yes | Yes |
| Monte Carlo | No initially | Planned |
| Orbital mechanics | No | Possible future extension |

---

## 42. Model Scope Baseline Decisions

The following decisions are the Model Scope baseline:

- AeroForge uses progressive model fidelity.
- Initial vehicle dynamics assume a rigid body.
- The initial Earth model is flat and local.
- Earth rotation is initially ignored.
- Constant gravity is the first gravity model.
- Vacuum scenarios are supported for verification.
- Atmospheric modeling begins simple and may become altitude-dependent.
- Wind remains logically separate from atmospheric thermodynamic properties.
- Weather simulation is out of scope.
- Constant mass is acceptable for early validation.
- Rocket mass becomes variable during powered-flight development.
- Center of mass and inertia may later vary with vehicle mass properties.
- Detailed propellant fluid dynamics are out of scope.
- Propulsion progresses from no thrust to constant thrust to thrust curves.
- Aerodynamics progresses from drag to full forces and moments.
- CFD is out of scope.
- Aerodynamic calculations use air-relative velocity.
- Detailed thermal analysis is initially out of scope.
- Structural deformation is initially out of scope.
- Engineering limit monitoring may be introduced later.
- The initial ground model is a flat plane.
- General collision physics is out of scope.
- Quaternions are the internal orientation representation for 6DOF dynamics.
- Simulation time is independent of wall-clock time.
- Fixed timesteps are used initially.
- Adaptive integration may be introduced later.
- Deterministic execution is a core requirement.
- Truth state remains distinct from estimated state.
- Sensors may operate at different rates from physics.
- Actuator state is distinct from actuator command.
- Sensors and actuators are modeled behaviorally rather than at component-physics level.
- Initial GNC remains deliberately simple.
- The initial operational domain is local atmospheric flight.
- Physical-model fidelity claims must be explicit and defensible.
- Every significant model should document assumptions and validity.
- Verification takes priority over unnecessary fidelity.

---

## 43. Model Scope Acceptance Criteria

Model Scope is complete when the project has formally established that:

- [x] AeroForge uses progressive fidelity.
- [x] Initial vehicle physics assumes a rigid body.
- [x] The initial Earth model is flat/local.
- [x] Earth rotation is initially ignored.
- [x] Constant gravity is the first gravity model.
- [x] More advanced gravity remains replaceable.
- [x] Vacuum trajectories are supported for validation.
- [x] Atmosphere begins with simple models.
- [x] Wind is independent from atmospheric thermodynamic modeling.
- [x] Weather simulation is outside initial scope.
- [x] Constant mass is acceptable for early validation.
- [x] Rocket mass becomes variable in later fidelity levels.
- [x] Center of mass and inertia may become variable.
- [x] Propellant fluid dynamics are outside initial scope.
- [x] Propulsion progresses from simple to time-dependent thrust.
- [x] Aerodynamics progresses from drag to full forces and moments.
- [x] CFD is outside initial scope.
- [x] Aerodynamic calculations use air-relative velocity.
- [x] Thermal analysis is outside initial scope.
- [x] Structural deformation is outside initial scope.
- [x] Engineering flight/structural limits may be monitored.
- [x] Ground is initially a flat plane.
- [x] General collision physics is outside initial scope.
- [x] Quaternions are planned for 6DOF orientation.
- [x] Simulation time is independent of wall-clock time.
- [x] Fixed timesteps are used initially.
- [x] Adaptive integration remains a future option.
- [x] Deterministic execution is required.
- [x] Truth state remains separate from estimated state.
- [x] Sensor rates may differ from the physics update rate.
- [x] Actuator command and physical actuator state remain distinct.
- [x] Sensor and actuator models are behavioral.
- [x] Initial GNC remains intentionally simple.
- [x] The initial operational domain is local atmospheric flight.
- [x] Every significant physical model should document its assumptions and validity.
- [x] Verification takes priority over unnecessary fidelity.
- [x] Fidelity increases progressively.

---

## 44. Revision Policy

This scope document is expected to evolve as AeroForge becomes more capable.

Changes should preserve the following principles:

- model assumptions remain explicit
- increased fidelity remains justified
- models remain replaceable where practical
- higher-fidelity models are validated before being trusted
- lower-fidelity validation scenarios remain available
- documented limitations are not hidden behind vague claims of realism

Significant changes to AeroForge's physical scope should be reflected in this document.
