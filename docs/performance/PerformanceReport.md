# AeroForge Performance Report

> **Historical measured profiling and optimization report.** The before/after measurements below come from the supplied AeroForge report. The profiling session date and raw timing/profiler files were not included in the supplied archive. Ratios have been checked against reported means; profiler event/sample counts are not heap-allocation totals. **Later measurements in the real-time report use a separate run/configuration** and must not be presented as additional speedups. See [Measurement Audit](MeasurementAudit.md).


## Overview

The recorded optimization work focused on profiling and optimizing the performance of AeroForge's numerical simulation, telemetry, guidance/navigation/control, and Monte Carlo infrastructure.

The optimization process followed a measurement-first approach:

1. Build dedicated performance benchmark suites.
2. Establish Release-mode performance baselines.
3. Profile CPU usage using the Visual Studio Performance Profiler.
4. Identify measured performance bottlenecks.
5. Optimize only confirmed hotspots.
6. Re-run the same benchmarks after each optimization.
7. Compare before-and-after latency and throughput.
8. Re-profile the optimized implementation.

The two primary optimization targets identified during profiling were:

- CSV telemetry serialization.
- Temporary state allocation during Runge-Kutta 4 integration.

---

## Test Environment

- **Platform:** Windows x64
- **Language:** C++20
- **Compiler:** Microsoft Visual C++
- **Build system:** CMake + Ninja
- **Build type:** Release
- **Benchmark executable:** `AeroForgeBenchmarks`
- **Profiler:** Visual Studio CPU Usage Profiler
- **Timing source:** `std::chrono::steady_clock`
- **Profiling symbols:** Release PDB symbols enabled
- **Optimization:** Release compiler optimizations remained enabled while profiling

Debug symbols were generated for the Release executable so profiler samples could be mapped to AeroForge functions without disabling compiler optimization.

---

## Benchmark Methodology

The benchmark suite was designed to reduce measurement noise and prevent the compiler from eliminating benchmarked operations.

Each benchmark:

- Runs multiple measurement batches.
- Performs many operations per batch.
- Measures complete batches rather than individual nanosecond-scale operations.
- Uses observable results to prevent dead-code elimination.
- Avoids console output inside timed regions.
- Runs using an optimized Release build.
- Uses deterministic benchmark inputs.
- Reports both latency and throughput.

Each `BenchmarkResult` contains:

- Benchmark name.
- Total number of iterations.
- Total wall-clock time.
- Mean latency per operation.
- Minimum batch-average latency.
- Maximum batch-average latency.
- Operations per second.

Performance comparisons were made using the same machine, build configuration, benchmark workload, and iteration counts.

---

# Benchmark Coverage

## Math Benchmarks

The math benchmark suite measures frequently used low-level operations.

Benchmarks:

- `Vector3Magnitude`
- `Vector3CrossProduct`
- `QuaternionMultiplication`
- `QuaternionRotation`

These operations are used throughout AeroForge's dynamics, navigation, guidance, and coordinate transformation systems.

---

## Numerical Integration Benchmarks

The integration benchmark suite measures the primary numerical integrators.

Benchmarks:

- `EulerIntegratorStep`
- `RK4IntegratorStep`

A deterministic harmonic oscillator dynamics model is used so each integration step depends on the previous state and cannot be trivially optimized away.

---

## 6DOF Dynamics Benchmarks

The 6DOF benchmark suite measures rigid-body dynamics performance.

Benchmarks:

- `SixDoFDynamicsEvaluate`
- `SixDoFRK4Step`

`SixDoFDynamicsEvaluate` measures one evaluation of the full 13-element rigid-body state derivative.

`SixDoFRK4Step` measures complete RK4 propagation of the 6DOF state.

---

## Guidance, Navigation, and Control Benchmarks

The GNC benchmark suite measures common guidance, estimation, and control operations.

Benchmarks:

- `AttitudeError`
- `AttitudePdController`
- `InertialNavigator`
- `GncCycle`

The combined GNC benchmark performs representative attitude guidance, control command generation, and inertial navigation propagation.

---

## Telemetry Benchmarks

The telemetry benchmark suite measures data recording, post-flight analysis, and serialization.

Benchmarks:

- `TelemetryRecorder`
- `FlightAnalyzer10kSamples`
- `GncAnalyzer10kSamples`
- `CsvTelemetryWriter10kSamples`

The CSV benchmark serializes 10,000 complete telemetry samples per benchmark iteration.

---

## Monte Carlo Benchmarks

The Monte Carlo benchmark suite measures stochastic parameter generation and trial execution infrastructure.

Benchmarks:

- `MonteCarloParameterSampling`
- `MonteCarloRunner`

The runner benchmark includes deterministic trial generation, parameter sampling, callback execution, and result collection.

---

# Initial Profiling Results

The initial CPU profile identified CSV telemetry serialization as the dominant workload in the complete benchmark execution.

`CsvTelemetryWriter::write()` accounted for approximately:

- **89.8% of total sampled CPU time**

The function itself had very low self CPU usage, indicating that the majority of the cost occurred inside the standard-library functions called by the CSV writer.

Notable functions visible in the initial profile included:

- `std::operator<<`
- `operator new`
- `operator delete`
- `std::vector<double>::_Buy_nonzero`
- `RK4Integrator::step`
- `Quaternion::rotate`
- `Quaternion::operator*`
- `Vector3::magnitude`
- `State::operator+`

The profile indicated two primary opportunities for optimization:

1. Reduce floating-point stream formatting overhead in CSV serialization.
2. Reduce dynamic allocation and temporary `State` construction during RK4 integration.

---

# Optimization 1 — CSV Telemetry Serialization

## Original Implementation

The original `CsvTelemetryWriter` serialized telemetry using repeated stream insertion operations.

Conceptually, each telemetry row was written as:

```cpp
output
    << value1 << ','
    << value2 << ','
    << value3 << ','
    << value4 << ','
    // ...
    << finalValue
    << '\n';
```

Each telemetry sample contains approximately 40 output fields.

For a 10,000-sample telemetry export, this resulted in hundreds of thousands of formatted stream insertion operations.

The profiler showed that repeated stream formatting was the dominant cost of the benchmark suite.

---

## Optimized Implementation

The CSV writer was redesigned around `std::to_chars`.

The optimized implementation:

- Converts floating-point values using `std::to_chars`.
- Uses `std::chars_format::general`.
- Preserves 17 digits of floating-point precision.
- Uses a stack-based temporary character buffer for each conversion.
- Builds each complete CSV row in a reusable `std::string`.
- Reserves row capacity before entering the sample loop.
- Reuses the same string allocation with `clear()`.
- Appends commas and newline characters with `push_back()`.
- Encodes boolean values directly as `'0'` or `'1'`.
- Writes each completed telemetry row with one `ostream::write()` operation.
- Preserves the existing CSV schema and column ordering.
- Preserves stream error validation.

The optimized data flow is conceptually:

```text
TelemetrySample
      |
      v
std::to_chars
      |
      v
Reusable row buffer
      |
      v
Single output.write()
```

The original implementation followed a much more expensive path:

```text
TelemetrySample
      |
      v
operator<<
operator<<
operator<<
operator<<
...
      |
      v
iostream formatting machinery
```

---

# CSV Performance Results

## Before Optimization

`CsvTelemetryWriter10kSamples`

- **Mean latency:** approximately 317.777 ms per 10,000-sample export
- **Throughput:** approximately 3.147 exports/s

---

## After Optimization

`CsvTelemetryWriter10kSamples`

- **Mean latency:** approximately 40.527 ms per 10,000-sample export
- **Throughput:** approximately 24.675 exports/s

---

## CSV Improvement

The optimized implementation achieved approximately:

- **7.84x higher throughput**
- **87.2% lower serialization latency**
- Approximately **684% higher throughput**

The optimized implementation preserved the telemetry schema and double-precision output while substantially reducing serialization overhead.

---

# Optimization 2 — RK4 State Allocation

## Original Implementation

The original RK4 integrator relied heavily on value-returning `State` arithmetic expressions.

Examples included:

```cpp
state + k1 * (dt / 2.0)
```

and:

```cpp
state + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * (dt / 6.0)
```

`State` internally stores its elements in a `std::vector<double>`.

As a result, these expression chains could create several temporary `State` objects during every RK4 step.

Each temporary `State` could require:

- Construction of a new `std::vector<double>`.
- Heap allocation.
- Element copying.
- Destruction.
- Heap deallocation.

The profiler showed allocation-related functions among the notable CPU consumers:

- `operator new`
- `operator delete`
- `std::vector<double>::_Buy_nonzero`

This suggested that temporary state construction was producing measurable overhead in the numerical integration path.

---

# State::addScaled Optimization

A new in-place `State` operation was added:

```cpp
State& State::addScaled(
    const State& other,
    double scalar);
```

Its operation is equivalent to:

```text
this[i] += other[i] * scalar
```

for every state element.

The method validates that both states have matching dimensions and modifies the existing state storage in place.

This avoids constructing an additional scaled `State`.

---

# Optimized RK4 Structure

The RK4 implementation was rewritten to reuse intermediate state storage.

The optimized sequence is conceptually:

```text
k1 = evaluate(time, state)

intermediate = state
intermediate.addScaled(k1, dt / 2)
k2 = evaluate(time + dt / 2, intermediate)

intermediate = state
intermediate.addScaled(k2, dt / 2)
k3 = evaluate(time + dt / 2, intermediate)

intermediate = state
intermediate.addScaled(k3, dt)
k4 = evaluate(time + dt, intermediate)
```

The final result is accumulated in place:

```text
result = state

result.addScaled(k1, dt / 6)
result.addScaled(k2, dt / 3)
result.addScaled(k3, dt / 3)
result.addScaled(k4, dt / 6)
```

This remains mathematically equivalent to the standard fourth-order Runge-Kutta update:

```text
state_next =
    state +
    (dt / 6) *
    (k1 + 2*k2 + 2*k3 + k4)
```

The optimization changes memory behavior rather than the integration mathematics.

All existing validation for:

- finite timestep,
- positive timestep,
- finite simulation time,
- finite state,
- derivative dimensions,
- finite derivatives,

remains intact.

---

# RK4 Performance Results

## Generic RK4 Integrator

### Before Optimization

`RK4IntegratorStep`

- **Mean latency:** approximately 0.916 us/step
- **Throughput:** approximately 1.091 million steps/s

### After Optimization

`RK4IntegratorStep`

- **Mean latency:** approximately 0.363 us/step
- **Throughput:** approximately 2.751 million steps/s

### Improvement

The RK4 state-allocation optimization achieved approximately:

- **2.52x higher throughput**
- **60.3% lower step latency**

---

# Full 6DOF RK4 Performance

## Before Optimization

`SixDoFRK4Step`

- **Mean latency:** approximately 1.740 us/step
- **Throughput:** approximately 574,600 steps/s

## After Optimization

`SixDoFRK4Step`

- **Mean latency:** approximately 1.194 us/step
- **Throughput:** approximately 837,700 steps/s

## Improvement

The full 6DOF integration path achieved approximately:

- **1.46x higher throughput**
- **31.4% lower step latency**

The improvement is smaller than the isolated generic RK4 improvement because a full 6DOF RK4 step also spends time evaluating rigid-body translational and rotational dynamics.

Those dynamics calculations are not directly affected by the temporary-state allocation optimization.

---

# Before / After Performance Summary

| Benchmark | Before | After | Improvement |
| --- | ---: | ---: | ---: |
| CSV export — 10k samples | 317.777 ms | 40.527 ms | 87.2% lower latency |
| CSV throughput | 3.147 exports/s | 24.675 exports/s | 7.84x |
| RK4 integrator step | 0.916 us | 0.363 us | 60.3% lower latency |
| RK4 throughput | 1.091M steps/s | 2.751M steps/s | 2.52x |
| 6DOF RK4 step | 1.740 us | 1.194 us | 31.4% lower latency |
| 6DOF RK4 throughput | 574.6K steps/s | 837.7K steps/s | 1.46x |

---

# Representative Post-Optimization Benchmark Results

The optimized benchmark suite produced approximately the following results.

## Math

| Benchmark | Mean Latency | Throughput |
| --- | ---: | ---: |
| Vector3Magnitude | 0.0140 us | 71.36M ops/s |
| Vector3CrossProduct | 0.00710 us | 140.94M ops/s |
| QuaternionMultiplication | 0.00957 us | 104.51M ops/s |
| QuaternionRotation | 0.01625 us | 61.55M ops/s |

---

## Integrators

| Benchmark | Mean Latency | Throughput |
| --- | ---: | ---: |
| EulerIntegratorStep | 0.1699 us | 5.885M steps/s |
| RK4IntegratorStep | 0.3634 us | 2.751M steps/s |

---

## 6DOF Dynamics

| Benchmark | Mean Latency | Throughput |
| --- | ---: | ---: |
| SixDoFDynamicsEvaluate | 0.2343 us | 4.268M evaluations/s |
| SixDoFRK4Step | 1.1937 us | 837.7K steps/s |

---

## Guidance, Navigation, and Control

| Benchmark | Mean Latency | Throughput |
| --- | ---: | ---: |
| AttitudeError | 0.0880 us | 11.37M ops/s |
| AttitudePdController | 0.1439 us | 6.95M updates/s |
| InertialNavigator | 0.1757 us | 5.69M updates/s |
| GncCycle | 0.2679 us | 3.73M cycles/s |

---

## Telemetry

| Benchmark | Mean Latency | Throughput |
| --- | ---: | ---: |
| TelemetryRecorder | 0.3511 us | 2.85M samples/s |
| FlightAnalyzer10kSamples | 97.78 us | 10.23K analyses/s |
| GncAnalyzer10kSamples | 91.03 us | 10.99K analyses/s |
| CsvTelemetryWriter10kSamples | 40.527 ms | 24.67 exports/s |

---

## Monte Carlo

| Benchmark | Mean Latency | Throughput |
| --- | ---: | ---: |
| MonteCarloParameterSampling | 0.0483 us | 20.71M samples/s |
| MonteCarloRunner | 2.307 us | 433.5K trials/s |

---

# Post-Optimization Profiling

After the CSV and RK4 optimizations, the CPU profile changed substantially.

The dominant functions became standard-library floating-point conversion routines associated with `std::to_chars`.

Notable post-optimization functions included:

```text
std::__mulShift_mod1e9
std::__d2fixed_buffered_n<char>
std::_Floating_to_chars_general_precision<double>
```

Representative profiler results included approximately:

| Function | Total CPU | Self CPU |
| --- | ---: | ---: |
| `std::__mulShift_mod1e9` | 27.74% | 27.67% |
| `std::__d2fixed_buffered_n<char>` | 9.56% | 9.51% |
| `std::_Floating_to_chars_general_precision<double>` | 4.62% | 4.60% |

This indicates that the remaining CSV cost is primarily the actual conversion of binary floating-point values into decimal text rather than general iostream infrastructure.

This is an expected result because serialization fundamentally requires numerical-to-text conversion.

Further major reductions would likely require changing the telemetry representation, reducing output precision, or introducing a binary telemetry format.

Those changes are outside the scope of this optimization because they would alter output behavior or file format.

---

# Allocation Profiling

Before the RK4 optimization, allocation-related functions appeared prominently in the CPU profile.

One representative profile showed:

```text
operator new: 709 sampled calls/events
```

After the RK4 state-reuse optimization, a comparable profile showed:

```text
operator new: 313 sampled calls/events
```

This represents an approximate:

```text
56% reduction
```

in sampled allocation activity.

Profiler sample counts are not exact allocation counts and should not be interpreted as a direct count of heap operations.

However, the reduction provides supporting evidence for the benchmark results and confirms that temporary state allocation was significantly reduced.

---

# Numerical-Core Performance

The optimized full 6DOF RK4 benchmark achieved approximately:

```text
837,700 integration steps per second
```

The benchmark uses a timestep of:

```text
dt = 0.0001 seconds
```

At this timestep:

```text
10,000 integration steps = 1 simulated second
```

Therefore, the isolated 6DOF RK4 integration path demonstrates substantial computational headroom relative to real-time execution.

The theoretical ratio for this isolated benchmark is approximately:

```text
837,700 / 10,000 = 83.77
```

or roughly:

```text
84x real time
```

for the isolated dynamics/integration workload.

This value should **not** be interpreted as the complete AeroForge end-to-end real-time factor.

The benchmark does not include every subsystem that would execute during a complete simulation, including all combinations of:

- atmosphere calculations,
- aerodynamic calculations,
- propulsion,
- sensors,
- navigation,
- guidance,
- control,
- actuators,
- failure injection,
- telemetry,
- event logging,
- visualization,
- hardware-in-the-loop communication.

End-to-end real-time performance is evaluated separately in later AeroForge phases.

---

# Why Optimization Stopped Here

The goal of this measured optimization study was not to optimize every function in the codebase.

The goal was to use profiling data to identify meaningful bottlenecks and eliminate them without sacrificing correctness or maintainability.

After optimization:

- CSV stream overhead was largely eliminated.
- RK4 temporary state allocation was substantially reduced.
- RK4 no longer appeared as a leading CPU hotspot.
- The remaining major CSV cost was actual floating-point conversion.
- Low-level `Vector3` and quaternion operations remained extremely inexpensive.
- Additional optimization would have increasingly smaller returns.

Further optimization without profiling evidence would therefore risk becoming premature optimization.

---

# Correctness Preservation

Performance optimization was performed without intentionally changing simulation behavior.

The optimized implementation retains:

- C++20 compatibility.
- Existing state dimensions.
- Existing RK4 mathematics.
- Existing timestep validation.
- Existing finite-state validation.
- Existing derivative validation.
- Existing telemetry column ordering.
- Existing telemetry schema.
- Existing boolean encoding.
- 17-digit floating-point output precision.
- Existing error handling.

Automated tests should be run after each optimization to ensure numerical and serialization behavior remains valid.

---

# Engineering Conclusions

The optimization effort demonstrated a measurement-driven optimization process rather than speculative optimization.

The initial profiler identified two significant bottlenecks:

1. Stream-based telemetry serialization.
2. Temporary dynamic allocation during RK4 integration.

Targeted changes resulted in substantial measurable performance improvements.

Key results include:

- Approximately **7.84x faster CSV telemetry serialization**.
- Approximately **87.2% lower CSV serialization latency**.
- Approximately **2.52x faster generic RK4 integration**.
- Approximately **60.3% lower generic RK4 step latency**.
- Approximately **1.46x faster full 6DOF RK4 integration**.
- Approximately **31.4% lower full 6DOF RK4 latency**.
- Significant reduction in sampled allocation activity.
- No requirement to reduce numerical precision.
- No requirement to change RK4 integration mathematics.

The optimized AeroForge numerical core demonstrates high computational throughput while maintaining modularity, validation, and numerical clarity.

---

# Resume-Relevant Results

The strongest performance results are:

- Optimized telemetry serialization using `std::to_chars` and reusable buffers, reducing 10,000-sample CSV export latency by approximately **87%** and improving throughput by approximately **7.8x**.
- Reduced RK4 temporary state allocations through in-place scaled accumulation and intermediate-state reuse, improving generic RK4 throughput by approximately **2.5x**.
- Improved full 6DOF RK4 propagation throughput by approximately **46%**, reaching approximately **838,000 integration steps per second** in the isolated benchmark.

Final resume wording should use validated and repeatable benchmark measurements rather than theoretical or unverified estimates.

---

# Remaining performance work

At the time this report was drafted, the remaining work was performance regression validation. Regression tests were subsequently included in the local history; consult the current test suite for their scope.

Performance regression validation should ensure that future AeroForge changes do not accidentally introduce severe performance degradation.

Regression validation should:

- Avoid extremely strict wall-clock thresholds.
- Avoid tests that fail because of normal scheduler or hardware variability.
- Use broad sanity thresholds.
- Detect major regressions rather than tiny timing differences.
- Keep correctness tests separate from benchmark reporting.
- Preserve repeatability across development machines where practical.

Future optimization work should continue to be informed by fresh benchmark and profiler evidence.

---

# Final  Performance Summary

AeroForge optimization established a dedicated C++ performance benchmark suite, used CPU profiling to identify real bottlenecks, and applied targeted optimizations to two major workloads.

The resulting implementation demonstrated:

- high-throughput vector and quaternion mathematics,
- multi-million-step-per-second numerical integration,
- sub-microsecond GNC operations,
- substantially reduced RK4 allocation overhead,
- high-throughput 6DOF propagation,
- multi-million-sample-per-second telemetry recording,
- optimized high-precision CSV telemetry serialization,
- scalable Monte Carlo infrastructure.

These results provide a quantitative baseline for future real-time simulation, visualization, hardware-in-the-loop, and validation work in subsequent AeroForge phases.
