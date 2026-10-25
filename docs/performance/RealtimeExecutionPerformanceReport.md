# Real-Time Execution Performance and Optimization Report

> **Historical measured profiling and optimization report.** The before/after measurements below come from the supplied AeroForge report. The profiling session date and raw timing/profiler files were not included in the supplied archive. Ratios have been checked against reported means; profiler event/sample counts are not heap-allocation totals. **Later measurements in the real-time report use a separate run/configuration** and must not be presented as additional speedups. See [Measurement Audit](MeasurementAudit.md).


## Overview

AeroForge includes real-time, fixed-rate, and unpaced execution modes for aerospace simulation workloads. This document evaluates the execution system with emphasis on:

- real-time pacing accuracy,
- faster-than-real-time simulation performance,
- deadline and overrun detection,
- end-to-end simulation throughput,
- incremental subsystem cost,
- long-duration execution stability, and
- performance regression resistance.

The objective is to determine whether the simulation runtime can reliably support real-time execution while retaining substantial computational headroom for faster-than-real-time analysis, Monte Carlo workloads, future visualization, and hardware-in-the-loop integration.

All reported real-time execution benchmarks were run using the optimized x64 Release build.

At the time of testing, the full automated test suite contained **1,180 tests**, all of which passed.

---

## Execution Modes

AeroForge supports three execution modes.

### Unpaced

The simulation executes as quickly as computational resources allow.

This mode is intended for:

- offline analysis,
- Monte Carlo simulation,
- parameter sweeps,
- deterministic replay,
- performance testing, and
- faster-than-real-time mission simulation.

### Real-Time

Simulation time is synchronized with elapsed wall-clock time.

For example, a simulation operating at a physics timestep of:

```text
dt = 0.001 s
```

executes at a nominal physics frequency of:

```text
1000 Hz
```

and one simulated second is intended to correspond to approximately one wall-clock second.

### Fixed-Rate

Simulation execution is paced according to a requested wall-clock update frequency independently of the simulation timestep.

This allows simulation time resolution and execution frequency to be controlled independently.

---

## Timing Architecture

Real-time pacing uses `std::chrono::steady_clock` to avoid dependence on changes to system wall-clock time.

The simulation pacer uses a hybrid sleep-and-spin strategy.

For deadlines sufficiently far in the future, execution sleeps until shortly before the target deadline. The final portion of the interval uses active spinning to reduce operating-system scheduler overshoot.

This design was selected after observing significant scheduling overshoot from ordinary sleep-based pacing on Windows.

The current pacing strategy therefore favors timing precision over minimum CPU utilization during short real-time intervals.

---

## Simulation-Time Stability

The simulation runner derives simulation time from the integer simulation step count instead of repeatedly accumulating the timestep.

Conceptually:

```text
simulationTime = stepCount × simulationDt
```

rather than:

```text
simulationTime += simulationDt
```

This avoids cumulative floating-point drift over long fixed-step simulations.

A small floating-point tolerance is used when evaluating the final simulation boundary.

This behavior was validated after regression testing detected a case where repeated floating-point accumulation caused a nominal 10,000-step simulation to execute 10,001 steps.

The corrected implementation executes the expected number of steps while continuing to support partial final timesteps for durations that are not exact multiples of the configured timestep.

---

# Real-Time Pacing Validation

## On-Schedule Real-Time Execution

A lightweight simulation workload was executed in real-time mode using:

```text
Simulation timestep: 0.001 s
Nominal update rate:  1000 Hz
```

Aggregated results across ten runs were:

| Metric | Result |
|---|---:|
| Simulated time | 10.0 s |
| Wall time | 10.0 s |
| Real-time factor | 1.000× |
| Simulation steps | 10,000 |
| Deadline misses | 0 |
| Maximum overrun | 0 s |

The simulation maintained the requested real-time schedule without recorded deadline misses.

---

## Deliberate Overload Test

Deadline detection was validated using a workload intentionally designed to exceed its execution budget.

Each simulation step had:

```text
Available budget: approximately 1 ms
Injected workload: approximately 2 ms
```

Aggregated results were:

| Metric | Result |
|---|---:|
| Simulated time | 1.0 s |
| Wall time | 2.00004 s |
| Real-time factor | 0.499989× |
| Simulation steps | 1,000 |
| Deadline misses | 1,000 |
| Maximum overrun | 0.100015 s |

Every overloaded step was correctly identified as a deadline miss.

The resulting real-time factor of approximately **0.5×** is consistent with a workload requiring approximately twice the available execution time.

This demonstrates that deadline instrumentation detects genuine timing failure rather than simply reporting successful pacing under all conditions.

---

## Fixed-Rate Execution

Fixed-rate execution was tested using:

```text
Simulation timestep: 0.0001 s
Target execution rate: 1000 Hz
```

Results aggregated across ten runs were:

| Metric | Result |
|---|---:|
| Simulated time | 1.0 s |
| Wall time | 10.0 s |
| Real-time factor | 0.100× |
| Simulation steps | 10,000 |
| Deadline misses | 0 |
| Maximum overrun | 0 s |

This demonstrates that simulation timestep and wall-clock execution rate are independent.

The runner can therefore maintain a simulation resolution finer than the requested wall-clock update rate.

---

# Faster-Than-Real-Time Performance

## Runner Overhead

The unpaced runner was tested using 100,000 simulation callbacks per complete run.

### Empty/Minimal Step

```text
Mean execution time per 100,000-step run: 421.680 us
```

### Lightweight Deterministic Step

```text
Mean execution time per 100,000-step run: 419.998 us
```

The similar results indicate that basic simulation-runner overhead is very small relative to the cost of full dynamics and subsystem processing.

---

## 6DOF RK4 Propagation

A standalone six-degree-of-freedom rigid-body simulation was executed using RK4 integration.

Configuration:

```text
Simulated duration: 10 s
Physics timestep:   0.0001 s
Integration steps:  100,000
```

Results:

```text
Mean wall time: 59.830 ms
```

This corresponds to roughly:

```text
167× real time
```

for a 10 kHz 6DOF RK4 propagation workload.

---

# End-to-End Nominal Simulation

The integrated nominal-flight benchmark exercises the major runtime components together.

Nominal update rates are:

| Component | Frequency |
|---|---:|
| 6DOF physics | 1000 Hz |
| IMU | 200 Hz |
| Navigation propagation | 200 Hz |
| GPS | 10 Hz |
| Guidance and control | 100 Hz |
| Telemetry | 100 Hz |

The integrated workload includes:

- six-degree-of-freedom rigid-body dynamics,
- RK4 numerical integration,
- IMU sampling,
- GPS sampling,
- inertial navigation,
- GPS correction,
- attitude guidance,
- attitude PD control,
- body-moment actuator dynamics,
- closed-loop moment application, and
- telemetry recording.

The benchmark executes ten independent 10-second simulation runs. Specialized real-time statistics therefore represent an aggregate of **100 simulated seconds**.

Results:

| Metric | Result |
|---|---:|
| Simulated time | 100 s |
| Wall time | 0.0731406 s |
| Real-time factor | **1367.23×** |
| Physics steps | 100,000 |
| Deadline misses | 0 |
| Maximum overrun | 0 s |

The complete nominal execution stack therefore runs at approximately:

# **1,367× real time**

under the benchmark configuration.

A single 10-second nominal simulation requires approximately:

```text
7.34 ms
```

of wall-clock execution time.

This provides substantial computational headroom above the requirements for real-time execution.

---

# Progressive Subsystem Analysis

To estimate the computational cost of individual simulation layers, progressively larger configurations were benchmarked using identical simulation duration and timestep settings.

| Configuration | Real-Time Factor |
|---|---:|
| 6DOF dynamics | **1636.28×** |
| + Sensors | **1532.62×** |
| + Navigation | **1490.46×** |
| + GNC | **1367.35×** |
| + Actuators | **1390.84×** |
| + Telemetry / Full Stack | **1367.23×** |

Approximate changes between successive configurations were:

| Added Component | Approximate RTF Change |
|---|---:|
| Sensors | -6.3% |
| Navigation | -2.8% |
| GNC | -8.3% |
| Actuators | +1.7% |
| Telemetry | -1.7% |

The small apparent performance increase after adding actuators should not be interpreted as the actuator subsystem improving execution speed. At these execution times, small differences caused by CPU frequency behavior, cache state, operating-system scheduling, and ordinary benchmark variation can reorder nearby measurements.

The more meaningful comparison is between the beginning and end of the progressive stack.

Adding sensors, navigation, GNC, actuators, and telemetry reduced measured RTF from approximately:

```text
1636× → 1367×
```

or approximately **16% overall**.

The complete subsystem stack therefore preserves the large majority of the underlying 6DOF simulation's available computational throughput.

---

# Long-Duration Stability

A continuous **300-second nominal simulation** was executed to detect performance degradation that might develop during longer runs.

Results:

| Metric | Result |
|---|---:|
| Simulated duration | 300 s |
| Wall time | 0.234582 s |
| Real-time factor | **1278.87×** |
| Physics steps | 300,000 |
| Deadline misses | 0 |
| Maximum overrun | 0 s |

The first and final 1,000 physics steps were independently timed.

| Window | Mean Step Time |
|---|---:|
| First 1,000 steps | 0.7861 us |
| Final 1,000 steps | 0.7077 us |
| Reported timing change | -9.97% |

No long-duration timing degradation was observed.

The final timing window was approximately 10% faster than the initial window. This should be interpreted as normal benchmark/runtime variation rather than evidence that the simulator inherently becomes faster over time.

The important observation is that execution cost did **not increase progressively** during the 300,000-step run.

---

# Supporting Component Performance

The real-time execution results are supported by microbenchmarks covering the primary computational components.

## Mathematical Operations

| Operation | Mean Time | Operations/sec |
|---|---:|---:|
| Vector magnitude | 0.00954 us | 104.8 M |
| Vector cross product | 0.00703 us | 142.3 M |
| Quaternion multiplication | 0.00985 us | 101.5 M |
| Quaternion rotation | 0.01678 us | 59.6 M |

---

## Numerical Integration

| Operation | Mean Time | Operations/sec |
|---|---:|---:|
| Euler step | 0.09234 us | 10.83 M |
| RK4 step | 0.20600 us | 4.85 M |

---

## Six-Degree-of-Freedom Dynamics

| Operation | Mean Time | Operations/sec |
|---|---:|---:|
| Dynamics evaluation | 0.11656 us | 8.58 M |
| 6DOF RK4 step | 0.57223 us | 1.75 M |

---

## Guidance, Navigation, and Control

| Operation | Mean Time |
|---|---:|
| Attitude error | 0.06876 us |
| Attitude PD controller | 0.07521 us |
| Inertial navigator | 0.08407 us |
| Combined GNC cycle | 0.14216 us |

These measurements show that individual GNC operations remain inexpensive relative to the available execution budget.

---

## Telemetry

| Operation | Mean Time |
|---|---:|
| Telemetry record | 0.26917 us |
| Flight analysis, 10k samples | 73.87 us |
| GNC analysis, 10k samples | 65.30 us |
| CSV serialization, 10k samples | 21.18 ms |

CSV serialization remains substantially more expensive than in-memory telemetry recording because floating-point values must be converted to textual representations.

The optimized CSV path nevertheless sustains approximately:

```text
47.2 exports/second
```

for 10,000-sample telemetry datasets.

CSV generation is performed as an analysis/export operation and does not represent the per-step cost of ordinary in-memory telemetry recording.

---

# Performance Optimization

Earlier profiling identified telemetry CSV serialization as the dominant analyzed export cost.

The serialization implementation was optimized using:

- `std::to_chars` for floating-point conversion,
- reusable row buffers,
- stack-allocated conversion buffers,
- reduced temporary string creation, and
- one output write operation per completed CSV row.

Numerical integration was also optimized by reducing unnecessary temporary state construction and allowing RK4 intermediate states to be reused.

Following optimization, the primary real-time simulation workload is no longer dominated by infrastructure overhead.

Current results show that a complete nominal simulation stack remains more than three orders of magnitude faster than real time under the benchmark configuration.

---

# Regression Testing

Performance-sensitive execution paths are covered by broad regression tests.

These tests are intentionally designed to detect severe regressions rather than enforce hardware-specific benchmark values.

Coverage includes:

- large RK4 workloads,
- telemetry recording,
- CSV serialization,
- unpaced execution,
- real-time pacing,
- fixed-rate pacing,
- deadline miss detection,
- overrun measurement,
- correct simulation step counts, and
- partial final timestep handling.

A floating-point boundary regression was discovered during this testing when a nominal 10,000-step simulation incorrectly executed 10,001 callbacks.

The simulation clock was subsequently changed to derive fixed-step simulation time from the integer step index rather than cumulative floating-point addition.

After correction:

```text
Automated tests: 1180 / 1180 passed
```

---

# Conclusions

The real-time execution system satisfies the current performance objectives.

Key measured results are:

- **1.000× real-time execution with zero deadline misses** under the nominal pacing test.
- Correct detection of **100% of deliberately overloaded deadlines**.
- Independent fixed-rate execution at the requested **1000 Hz**.
- Approximately **1367× real-time performance** for the complete nominal flight-simulation stack.
- Approximately **1279× real-time performance** during a continuous 300-second simulation.
- **300,000 continuous physics steps** with no observed long-duration timing degradation.
- Only approximately **16% throughput reduction** between the standalone 6DOF stack and the complete sensor/navigation/GNC/actuator/telemetry stack.
- **1,180 / 1,180 automated tests passing** after the final timing corrections.

These measurements demonstrate substantial execution headroom for future simulation workloads while preserving accurate real-time pacing when wall-clock synchronization is required.

The current runtime is therefore suitable as the execution foundation for additional high-cost functionality such as visualization, more detailed vehicle and environmental models, increased sensor complexity, hardware-in-the-loop interfaces, and expanded real-time telemetry.
