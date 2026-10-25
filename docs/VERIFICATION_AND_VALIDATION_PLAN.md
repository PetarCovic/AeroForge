# AeroForge Verification and Validation Plan

> **Documentation status (October 2026):** This is a detailed verification and validation strategy, not a signed validation certificate. Passing unit tests does not establish physical model fidelity or real-time deadline guarantees on all hardware.
> **Evidence boundary:** Design statements describe intended behavior; current implementation and measured results are established by source, tests, and archived benchmark reports, not by this specification alone.


## 1. Purpose

This document defines the original testing, verification, validation, regression, and benchmarking strategy.

AeroForge treats verification and validation as part of model implementation, not a final cleanup task.

---

## 2. Definitions

### Verification

Answers:

> Did we implement the equations/software correctly?

Examples:

- unit tests
- analytical comparisons
- convergence studies
- conservation checks

### Validation

Answers:

> Is the selected model adequate for the intended physical use?

Examples:

- comparison against trusted reference data
- known atmosphere tables
- published motor data
- accepted flight-dynamics reference cases

These concepts overlap in practice but should remain conceptually distinct.

---

## 3. Test Framework

Initial C++ test framework:

```text
GoogleTest
```

CMake should expose tests through:

```text
CTest
```

Tests must be runnable independently of Visual Studio.

---

## 4. Test Categories

### Unit Tests

Verify individual classes/functions.

Examples:

- vector operations
- quaternion multiplication
- atmosphere interpolation
- actuator limits

### Analytical Verification Tests

Compare simulation results with closed-form solutions.

Examples:

- constant-velocity motion
- free fall
- projectile motion
- constant acceleration
- constant thrust with fixed mass

### Integration Tests

Verify multiple components together.

Examples:

- environment + dynamics + integrator
- sensor + estimator
- controller + actuator

### Regression Tests

Detect unintended behavioral changes.

Use tolerances and carefully selected reference outputs.

### Convergence Tests

Verify numerical error decreases as timestep decreases.

Critical for integrators.

### Conservation Tests

Where the model permits:

- linear momentum
- angular momentum
- mechanical energy

### Statistical Tests

For stochastic models:

- mean
- variance
- distribution sanity
- deterministic seeded replay

### Performance Benchmarks

Measure:

- simulation steps per second
- runs per second
- Monte Carlo throughput
- memory use where relevant

Performance results are not correctness tests.

---

## 5. Floating-Point Comparison

Avoid direct equality for computed floating-point results.

Use:

- absolute tolerance
- relative tolerance
- vector norm tolerance
- problem-specific error metrics

Tolerance values must be justified by the model/test rather than selected solely to make a test pass.

---

## 6. Analytical Baseline Cases

AeroForge should preserve a growing suite of simple cases.

### Case A — Zero Force

Expected:

```text
constant velocity
```

### Case B — Constant Gravity

Expected:

\[
z(t) = z_0 + v_0 t + \frac{1}{2} a t^2
\]

subject to the selected axis convention.

### Case C — Constant Acceleration

Expected exact analytical position/velocity.

### Case D — Projectile Motion

Expected parabolic trajectory under constant gravity with no drag.

### Case E — Constant Thrust, Constant Mass

Expected analytical acceleration.

These remain useful even after much higher-fidelity models exist.

---

## 7. Integrator Verification

Each numerical integrator should be tested against the same reference problems.

Metrics include:

- final-state error
- trajectory error
- function evaluations
- runtime
- observed convergence order

For RK4, a timestep-refinement study should demonstrate behavior consistent with fourth-order convergence in an appropriate smooth problem.

---

## 8. Coordinate-System Verification

Tests must verify:

- 90-degree principal-axis rotations
- quaternion normalization
- inverse transforms
- `R_AB * R_BA ≈ I`
- round-trip vector transformation
- body/navigation sign conventions

Frame errors are considered high severity.

---

## 9. Physics Model Validation Template

Every significant physical model should eventually provide:

```text
Model name
Purpose
Assumptions
Reference frame
Units
Valid operating domain
Reference source/data
Verification tests
Validation comparison
Known limitations
```

---

## 10. Regression Strategy

Regression tests should focus on behavior that is expected to remain stable.

Avoid excessively large golden files when a smaller invariant or metric can detect the same defect.

If a reference output changes intentionally:

1. explain why
2. re-run validation
3. update the reference deliberately
4. record the change in version control

---

## 11. Stochastic Verification

A fixed seed must reproduce the same stochastic scenario within the supported reproducibility domain.

Statistical tests should not depend on one random draw.

Monte Carlo tests should evaluate distributions or summary metrics with appropriate tolerance.

---

## 12. Continuous Integration Goal

Later CI should run at minimum:

```text
configure
build
unit tests
analytical verification tests
static analysis subset
```

Long-running Monte Carlo and heavy benchmarks may run separately.

---

## 13. Definition of Done for a Physics Model

A physical model is not complete merely because it compiles.

A model is considered implementation-complete when:

- interface is documented
- units are documented
- frames are documented
- assumptions are documented
- invalid inputs are handled
- unit tests exist
- at least one meaningful verification/validation method exists
- telemetry/diagnostic hooks exist where useful
- warnings remain clean

---

## 14. Verification Acceptance Criteria

- [x] GoogleTest selected.
- [x] CTest integration planned.
- [x] Unit-test strategy defined.
- [x] Analytical verification strategy defined.
- [x] Integration/regression strategies defined.
- [x] Convergence testing required for numerical integrators.
- [x] Conservation tests planned where applicable.
- [x] Statistical testing strategy defined.
- [x] Performance benchmarks separated from correctness.
- [x] Floating-point comparison policy defined.
- [x] Baseline analytical cases defined.
- [x] Physics model definition-of-done established.
