# AeroForge Configuration and Reproducibility

## 1. Purpose

This document defines the Phase 0.22 and 0.23 configuration and deterministic-execution architecture.

---

## 2. Initial Configuration Format

AeroForge will initially use **JSON** configuration files.

Reasons:

- explicit syntax
- predictable typing
- broad tooling support
- straightforward C++ parsing
- easy versioning and validation
- suitable for deterministic machine-readable configuration

YAML support may be considered later if human-authored mission files become cumbersome.

---

## 3. Configuration Categories

The logical configuration model includes:

```text
simulation
vehicle
environment
sensors
actuators
gnc
failures
telemetry
```

A single top-level mission file may reference subsystem files later, but the initial implementation may keep configuration in one JSON document.

---

## 4. Configuration Versioning

Every top-level configuration should include a schema/configuration version.

Example:

```json
{
  "schemaVersion": 1
}
```

Breaking configuration changes require a version change or migration path.

---

## 5. Units

Physical configuration fields use SI units unless explicitly documented otherwise.

Field documentation must state units.

Ambiguous unit inference is not permitted.

---

## 6. Validation

Configuration is validated before simulation begins.

Invalid configuration should prevent run startup.

Examples of invalid inputs:

- non-positive timestep
- negative mass
- invalid quaternion
- impossible actuator limits
- unsupported model name
- missing required field
- duplicate component identifiers

The simulator should report clear validation errors rather than silently substituting values.

---

## 7. Master Seed

Each stochastic run has an explicit master random seed.

Example:

```json
{
  "randomSeed": 837291
}
```

If a seed is automatically generated, the selected value must be recorded in run metadata so the run can be reproduced.

---

## 8. Deterministic Random Streams

AeroForge should avoid one uncontrolled global sequence whose output depends on incidental call order.

The preferred long-term design is:

```text
Master Seed
├── sensor/imu stream
├── sensor/gps stream
├── wind stream
├── failure stream
└── monte-carlo-parameter stream
```

Named or stable subsystem streams reduce unintended coupling between random consumers.

Adding one random draw to a GPS model should not ideally change the wind realization.

---

## 9. Reproducibility Contract

Within the initially supported reproducibility domain, the same:

- executable/build
- configuration
- initial state
- integrator
- timestep
- random seed

should produce the same result.

Exact bitwise equality across different compilers, CPU architectures, math libraries, or optimization settings is **not initially guaranteed**.

The initial practical goal is deterministic replay within the same supported build/platform configuration.

---

## 10. Run Metadata

Every important simulation run should eventually record:

```text
AeroForge version
Git commit
build type
compiler/toolchain identifier
configuration/schema version
configuration hash
random seed
integrator
timestep
start/end conditions
platform information
run timestamp
```

This metadata allows a result to be traced back to the exact software/configuration context.

---

## 11. Configuration Hash

A normalized configuration hash should eventually be recorded with the run.

The hash should be calculated from the effective configuration after defaults and references are resolved.

This creates a compact identifier for the actual simulation inputs.

---

## 12. Defaults

Defaults may exist, but they must be explicit in code/schema documentation.

Critical physical parameters should not rely on undocumented implicit values.

The final effective configuration should be exportable or recordable for reproduction.

---

## 13. Monte Carlo Configuration

Monte Carlo runs should distinguish:

- deterministic base configuration
- uncertain parameter definitions
- master experiment seed
- individual run seed/stream identity

A failed Monte Carlo case must be reproducible independently.

---

## 14. Phase 0.22–0.23 Acceptance Criteria

- [x] JSON selected as the initial configuration format.
- [x] Configuration categories defined.
- [x] Schema versioning required.
- [x] SI units required for physical configuration values.
- [x] Configuration validation occurs before run startup.
- [x] Master random seed is explicit/recorded.
- [x] Deterministic subsystem random streams are planned.
- [x] Reproducibility domain is explicitly defined.
- [x] Run metadata requirements defined.
- [x] Effective configuration hash planned.
- [x] Monte Carlo replay requirements defined.
