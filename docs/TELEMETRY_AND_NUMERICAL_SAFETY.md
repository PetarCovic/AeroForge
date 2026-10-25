# AeroForge Telemetry and Numerical Safety

> **Documentation status (October 2026):** The repository includes telemetry recording, CSV output, events, flight/GNC analyses, and tests. Where this document specifies additional guarantees or a planned interface, confirm these against implementation before presenting them as verified.
> **Evidence boundary:** Design statements describe intended behavior; current implementation and measured results are established by source, tests, and archived benchmark reports, not by this specification alone.


## 1. Purpose

This document defines the original telemetry, diagnostics, error-handling, and numerical-safety requirements.

---

## 2. Telemetry Principles

Telemetry is observational.

It must not alter physics behavior.

The simulation should be able to run with telemetry enabled or disabled without changing the mathematical result.

---

## 3. Initial Output Formats

Initial recommended outputs:

### Time-Series Telemetry

**CSV**

Useful for:

- Python analysis
- plotting
- debugging
- spreadsheet inspection

### Run Metadata

**JSON**

Useful for:

- configuration identity
- seed
- toolchain/version information
- mission result
- model selection

A compact binary telemetry format may be introduced later for very large Monte Carlo workloads.

---

## 4. Planned Telemetry Channels

Core channels eventually include:

### Time

- simulation time
- step index

### Truth State

- position
- velocity
- orientation quaternion
- derived Euler angles
- angular velocity

### Vehicle

- mass
- center of mass
- inertia values where useful
- propellant mass
- thrust

### Environment

- gravity
- density
- pressure
- temperature
- speed of sound
- wind

### Flight Metrics

- Mach number
- dynamic pressure
- angle of attack
- sideslip angle

### GNC

- sensor measurements
- estimated state
- guidance targets
- controller error
- controller output

### Actuators

- commanded state
- physical state
- saturation/rate-limit state

### Failures

- failure start/end events
- active failures

---

## 5. Telemetry Metadata

Every telemetry channel must have:

- stable name
- description
- unit
- frame where applicable
- sampling rate/source where relevant

A consumer should not need to read C++ source code to determine what a column means.

---

## 6. Sampling

Telemetry sampling may differ from the physics timestep.

The logging system should eventually support:

- every physics step
- fixed telemetry rate
- event-triggered records
- subsystem-native rate

Telemetry timestamps always use simulation time.

---

## 7. Buffering

Early implementation may write synchronously for simplicity.

Later high-throughput simulation should use buffering so disk I/O does not dominate the physics loop.

Any asynchronous logging architecture must preserve deterministic simulation behavior.

---

## 8. Diagnostics vs Telemetry

AeroForge should distinguish:

### Telemetry

Structured simulation data intended for analysis.

### Diagnostic Logs

Human-readable messages such as:

```text
INFO
WARNING
ERROR
```

### Run Result

A structured final status describing:

- completed
- mission failed
- numerical failure
- invalid configuration
- terminated by user/tool

---

## 9. Numerical Safety Checks

AeroForge should detect numerical invalidity as close to its source as practical.

Checks may include:

- `NaN`
- infinity
- invalid timestep
- non-positive mass
- invalid quaternion norm
- invalid atmosphere result
- invalid inertia tensor
- singular or near-singular matrix where inversion is required
- state values outside a model's documented domain

---

## 10. Quaternion Safety

Orientation quaternions must remain finite and near unit length.

After numerical integration, normalization may be applied as an explicit numerical-maintenance step.

A quaternion that is zero, non-finite, or grossly outside tolerance is a numerical failure, not something to silently repair.

---

## 11. Mass Safety

Physical mass must remain positive.

A model that generates non-positive total mass is invalid.

Do not silently clamp negative mass to zero.

---

## 12. Timestep Safety

The timestep must be:

```text
finite
positive
within any model-specific configured limits
```

A non-positive or non-finite timestep rejects the run before simulation begins.

---

## 13. Inertia Safety

When a full inertia tensor is introduced, it should be validated for physical plausibility.

At minimum:

- finite entries
- symmetry within tolerance
- invertibility
- positive-definite behavior for valid rigid bodies

---

## 14. Error Categories

### Configuration Error

Detected before simulation starts.

Action:

```text
Reject run
```

### Programming Invariant Failure

Represents an internal bug.

Action:

```text
Fail fast in development/testing
```

Assertions may be used for internal invariants, but not as the sole validation for user configuration.

### Numerical Failure

Examples:

- NaN state
- divergent quaternion
- invalid matrix inversion

Action:

```text
Terminate run and preserve diagnostic context
```

### Physical/Mission Limit Violation

Examples:

- excessive dynamic pressure
- actuator saturation
- target miss

Action:

```text
Record mission failure/limit event
```

The simulation may continue if the scenario is configured to study post-failure behavior.

### Modeled Saturation

Examples:

- actuator position limit
- sensor range limit

Action:

```text
Apply explicit model-defined saturation
```

This is not an error; it is part of the component model.

---

## 15. Clamping Policy

Values must not be silently clamped merely to avoid errors.

Clamping is permitted only when it represents an explicit physical/model rule such as actuator saturation.

Numerical bugs should remain visible.

---

## 16. Failure Context

On numerical termination, diagnostics should eventually capture:

- simulation time
- step index
- current state
- active component/model
- configuration hash
- seed
- recent relevant events

This enables deterministic replay and debugging.

---

## 17. Telemetry and Safety Acceptance Criteria

- [x] Telemetry remains observational.
- [x] CSV selected for initial time-series output.
- [x] JSON selected for run metadata.
- [x] Planned telemetry channels identified.
- [x] Units and frames are required in telemetry metadata.
- [x] Simulation timestamps are authoritative.
- [x] Telemetry and diagnostics are separate concepts.
- [x] Numerical safety checks identified.
- [x] Quaternion validity policy defined.
- [x] Mass/timestep/inertia validation requirements defined.
- [x] Error categories defined.
- [x] Silent clamping prohibited except for explicit modeled saturation.
- [x] Numerical failure context requirements defined.
