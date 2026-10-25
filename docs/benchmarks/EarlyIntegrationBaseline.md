# AeroForge Early Integration Performance Baseline

> **Historical measured run — October 14, 2026.** Measurements, platform details, and checksums below are transcribed from the original AeroForge baseline report. Values are not fresh measurements, and no raw console output or per-run samples were provided in the documentation archive. Calculated throughput and per-step latency are consistent with 125,000 steps per configuration to normal reporting precision. Do **not** compare this directly with later isolated optimization benchmarks: workload and benchmark harness differ. See [Measurement Audit](../performance/MeasurementAudit.md).


## Overview

This document records the baseline performance of the AeroForge generic simulation core after completion of the early numerical integration infrastructure.

The benchmark compares the Forward Euler and fourth-order Runge-Kutta (RK4) integrators across increasing numerical state sizes. The purpose of this benchmark is to establish a repeatable performance baseline before later optimization and profiling work.

---

## Benchmark Configuration

**Build configuration:** Release
**Language:** C++20
**Build system:** CMake
**Platform:** Windows
**Compiler:** MSVC 19.51.36257
**CPU:** AMD Ryzen 9 5900X 12-Core Processor
**Operating system:** Windows 11 Home
**Benchmark date:** October 14, 2026

### Benchmark Parameters

Each configuration executed:

- 25,000 simulation steps per run
- 5 runs
- 125,000 total integration steps per configuration
- Fixed timestep: 0.001 seconds
- State sizes: 2, 16, 64, and 256 variables

The benchmark uses the same deterministic dynamics model for both integrators.

A checksum of the final simulation states is calculated to ensure benchmark results are consumed and numerical calculations are not optimized away.

---

## Results

| Integrator | State Size | Steps / Run | Runs | Elapsed Time (s) | Steps / Second | ns / Step | Checksum |
|---|---:|---:|---:|---:|---:|---:|---:|
| Euler | 2 | 25,000 | 5 | 0.014266 | 8,762,398.79 | 114.12 | 7.043100 |
| RK4 | 2 | 25,000 | 5 | 0.077869 | 1,605,268.36 | 622.95 | 7.043119 |
| Euler | 16 | 25,000 | 5 | 0.026411 | 4,732,876.45 | 211.29 | 18.234415 |
| RK4 | 16 | 25,000 | 5 | 0.128907 | 969,688.32 | 1,031.26 | 18.235087 |
| Euler | 64 | 25,000 | 5 | 0.064097 | 1,950,178.40 | 512.77 | 18.630573 |
| RK4 | 64 | 25,000 | 5 | 0.285493 | 437,838.65 | 2,283.95 | 18.631463 |
| Euler | 256 | 25,000 | 5 | 0.213510 | 585,451.30 | 1,708.08 | 18.630577 |
| RK4 | 256 | 25,000 | 5 | 0.883182 | 141,533.62 | 7,065.46 | 18.631466 |

---

## Observations

### Euler Throughput

Forward Euler achieved approximately:

- **8.76 million steps/second** for a 2-element state
- **4.73 million steps/second** for a 16-element state
- **1.95 million steps/second** for a 64-element state
- **585 thousand steps/second** for a 256-element state

Per-step execution cost increased from approximately **114 ns** to **1.71 µs** as the state size increased from 2 to 256 variables.

---

### RK4 Throughput

RK4 achieved approximately:

- **1.61 million steps/second** for a 2-element state
- **970 thousand steps/second** for a 16-element state
- **438 thousand steps/second** for a 64-element state
- **142 thousand steps/second** for a 256-element state

Per-step execution cost increased from approximately **623 ns** to **7.07 µs** across the tested state-size range.

---

## Euler vs. RK4

RK4 is more computationally expensive per simulation step than Forward Euler.

This is expected because RK4 evaluates the dynamics model four times per integration step and constructs multiple intermediate states before calculating the final weighted derivative.

Observed RK4-to-Euler per-step cost ratios were approximately:

| State Size | RK4 / Euler Cost |
|---:|---:|
| 2 | 5.46× |
| 16 | 4.88× |
| 64 | 4.45× |
| 256 | 4.14× |

The cost ratio decreases as state size increases, suggesting that shared state-manipulation costs become a larger portion of total Euler execution time while RK4's four dynamics evaluations remain the dominant additional cost.

---

## Scaling Behavior

Both integrators show increasing execution cost as state dimensionality increases.

From state size 2 to state size 256:

- Euler increased from **114.12 ns/step** to **1,708.08 ns/step**
- RK4 increased from **622.95 ns/step** to **7,065.46 ns/step**

This corresponds to approximately:

- **15.0×** higher Euler per-step cost
- **11.3×** higher RK4 per-step cost

for a **128× increase in state dimensionality**.

Across the measured range, the increase in per-step time is less than the increase in state dimension; this is a property of the reported workload, not a claim of asymptotic scaling for arbitrary state sizes.

---

## Numerical Result Comparison

Euler and RK4 produced similar but non-identical final checksums.

This is expected because the two integrators approximate the trajectory differently.

The checksums are used only as benchmark sanity checks and should not be interpreted as an accuracy metric.

Numerical accuracy is validated separately through analytical convergence tests in the numerical integrator test suite.

---

## Historical Numerical-Core Baseline Summary

The AeroForge simulation core currently demonstrates:

- deterministic fixed-step simulation
- interchangeable numerical integrators
- Forward Euler integration
- fourth-order Runge-Kutta integration
- multi-dimensional state support
- repeatable simulation execution
- numerical convergence against analytical solutions
- approximately first-order Euler convergence
- approximately fourth-order RK4 convergence
- measurable scaling across increasing state dimensions
- Release-mode throughput exceeding one million steps per second for small-state RK4 simulations
- Release-mode throughput exceeding eight million steps per second for small-state Euler simulations

These results serve as the initial AeroForge performance baseline.

---

## Historical Follow-Up Plan

Potential subsequent investigations included:

- CPU profiling and hotspot analysis
- memory allocation frequency
- temporary `State` allocations
- cache behavior
- compiler optimization effects
- Debug vs. Release performance
- vectorization opportunities
- move semantics and allocation reduction
- fixed-size versus dynamically sized state representations
- dynamics-model complexity
- full 6DOF vehicle-model throughput
- sensor and controller overhead
- telemetry overhead
- Monte Carlo throughput
- parallel scenario execution
- real-time factor
- simulations per second
- memory scaling with concurrent simulations

Optimizations should be evaluated against this historical integration baseline to ensure that performance changes are measurable and do not compromise numerical correctness or deterministic behavior.
