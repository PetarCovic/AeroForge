# AeroForge

**C++20 six-degree-of-freedom (6DOF) aerospace simulation and guidance, navigation, and control (GNC) platform.**

AeroForge is a modular engineering project for studying rigid-body flight dynamics, numerical integration, aerospace sensors, navigation, attitude control, stochastic failures, reproducible experiments, and execution performance. Its reference application is a guided research rocket; the core mathematics and simulation components are designed to be reusable.

> **Status:** The subsystems listed below are present in the local history through the benchmarks/examples commit (`b3e593e`). An integrated, polished flagship rocket mission is a **design goal**, not a claim that every mission requirement has been realized. The visualization backend and unfinished frontend are **not part of this documented baseline** and will be committed together when complete.

## Implemented areas

| Subsystem | Included functionality |
| --- | --- |
| Mathematics | `Vector3`, `Matrix`, `Quaternion`, geometry/frame transforms |
| Simulation | State and derivative abstractions, dynamics/integrator interfaces, Euler, RK4, clock and runner |
| Dynamics | Translational, rotational, rigid-body 6DOF models, force/moment accumulation |
| Environment and aerodynamics | Standard atmosphere, wind, relative air velocity, air data and aerodynamic modeling |
| Rocket modeling | Thrust curves, propulsion, propellant depletion, variable mass/inertia and rocket dynamics |
| Sensors and navigation | Accelerometer, gyroscope, IMU, GPS, sensor timing, inertial navigation and GPS correction |
| Guidance and actuation | Attitude setpoint/hold guidance, PD attitude controller, throttle, gimbal, body-moment actuators |
| Robustness | Fault injection, Monte Carlo trials/statistics, seed management and deterministic replay tests |
| Telemetry and analysis | Telemetry recording, sampling, CSV output, events, flight/GNC and Monte Carlo reports |
| Execution and profiling | Real-time pacing, execution results, timing summaries, regression tests and benchmark suites |

**Implementation vs. mission integration:** A subsystem's presence and passing component tests do not establish full end-to-end flight-system validation. Refer to the test suite and the [reference mission](docs/REFERENCE_MISSION.md) for that distinction.

## Coordinate and numerical conventions

- Right-handed **NED navigation frame**: `+X` North, `+Y` East, `+Z` Down.
- Right-handed **FRD body frame**: `+X` Forward, `+Y` Right, `+Z` Down.
- **`q_NB` maps body-frame vectors to navigation-frame vectors**, with scalar-first `wxyz` storage.
- SI units; all internal angles in radians; `double` precision for physical quantities.
- Simulation time is distinct from wall-clock pacing.

See [coordinate systems](docs/COORDINATE_SYSTEMS.md), [units](docs/UNITS_AND_CONVENTIONS.md), and [simulation assumptions](docs/SIMULATION_SCOPE_AND_ASSUMPTIONS.md).

## Build and test

**Prerequisites:** Git, CMake, and a C++20-capable compiler. The test configuration obtains GoogleTest through CMake dependency fetching, so an initial internet connection may be necessary. The demonstrated environment is Windows with MSVC/Visual Studio; other compilers have not been established by the supplied validation evidence.

From the repository root using Windows Command Prompt:

```cmd
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --config Debug --parallel 8
ctest --test-dir build -C Debug --output-on-failure
```

For a Release build of the benchmarks and examples:

```cmd
cmake --build build --config Release --parallel 8
```

Build targets include `AeroForgeCore`, `AeroForge`, `AeroForgeTests`, `AeroForgeBenchmarks`, and `AeroForgeBallisticExample`. Benchmarks measure performance; they are not substitutes for correctness or physical validation tests. Avoid comparing Debug throughput to Release measurements.

## Repository layout

```text
AeroForge/
├── include/        Public headers organized by subsystem
├── src/            Matching implementations
├── tests/          Subsystem, analytical and integration tests
├── benchmarks/     Numerical and subsystem performance suites
├── examples/       Ballistic projectile example
├── docs/           Technical specifications and measured reports
├── CMakeLists.txt
└── README.md
```

Subsystem folders include `math`, `simulation`, `dynamics`, `environment`, `aerodynamics`, `rocket`, `sensors`, `navigation`, `actuators`, `gnc`, `failures`, `montecarlo`, `reproducibility`, `telemetry`, and `performance` (as applicable under `include/`, `src/`, and `tests/`).

## Technical documentation

| Document | Purpose |
| --- | --- |
| [System architecture](docs/SYSTEM_ARCHITECTURE.md) | Subsystems, data flow, timing and design boundaries |
| [Reference mission](docs/REFERENCE_MISSION.md) | Intended guided-rocket demonstration and acceptance criteria |
| [Simulation scope and assumptions](docs/SIMULATION_SCOPE_AND_ASSUMPTIONS.md) | Model fidelity, approximations and exclusions |
| [Coordinate systems](docs/COORDINATE_SYSTEMS.md) | Frame, quaternion and transform contracts |
| [Units and conventions](docs/UNITS_AND_CONVENTIONS.md) | SI quantities, angle/time/altitude conventions |
| [Configuration and reproducibility](docs/CONFIGURATION_AND_REPRODUCIBILITY.md) | Seeds, replay and proposed mission-file contract |
| [Telemetry and numerical safety](docs/TELEMETRY_AND_NUMERICAL_SAFETY.md) | Recording, reporting, invariants and failure handling |
| [Verification and validation](docs/VERIFICATION_AND_VALIDATION_PLAN.md) | Analytical, integration, regression and statistical checks |
| [Coding standards](docs/CODING_STANDARDS.md) | C++20 conventions and engineering review criteria |
| [Early integration baseline](docs/benchmarks/EarlyIntegrationBaseline.md) | Historical Euler/RK4 throughput measurements |
| [Performance report](docs/performance/PerformanceReport.md) | Profiling, numerical-core and CSV optimizations |
| [Real-time performance](docs/performance/RealtimeExecutionPerformanceReport.md) | Pacing and workload-specific timing measurements |
| [Benchmarking guide](docs/BENCHMARKING_AND_PERFORMANCE.md) | Methodology, reproducibility, benchmark suites and interpretation |
| [Measurement audit](docs/performance/MeasurementAudit.md) | Arithmetic audit, report provenance, evidence limits and comparison warnings |

## Recorded performance — representative workloads

These are **historical measurements**, not fresh benchmarks or universal performance guarantees. Measurements belong to different sessions and configurations, so do not use figures across reports as before/after comparisons.

| Workload / configuration | Reported result | Provenance |
| --- | ---: | --- |
| Euler, 2-variable state, historical 25k steps × 5 runs | 8.76 million steps/s | [Integration baseline](docs/benchmarks/EarlyIntegrationBaseline.md) |
| RK4, 2-variable state, same historical baseline | 1.61 million steps/s | [Integration baseline](docs/benchmarks/EarlyIntegrationBaseline.md) |
| CSV telemetry export, 10,000 samples, optimized profiling comparison | 40.527 ms/export, versus 317.777 ms before | [Optimization report](docs/performance/PerformanceReport.md) |
| Generic RK4 step, optimized profiling comparison | 0.363 µs/step, versus 0.916 µs before | [Optimization report](docs/performance/PerformanceReport.md) |
| Full 6DOF RK4 step, optimized profiling comparison | 1.194 µs/step, versus 1.740 µs before | [Optimization report](docs/performance/PerformanceReport.md) |
| End-to-end nominal stack, ten 10-second runs, 1 kHz physics | 100 simulated s in 0.0731406 wall s; **1,367.23× real-time** | [Real-time performance](docs/performance/RealtimeExecutionPerformanceReport.md) |
| Continuous nominal-stack stability, 300 simulated s | 0.234582 wall s; **1,278.87× real-time** | [Real-time performance](docs/performance/RealtimeExecutionPerformanceReport.md) |

**Accuracy and reproducibility:** The [benchmarking guide](docs/BENCHMARKING_AND_PERFORMANCE.md) explains benchmark types, how to rerun them, comparable configurations, and the limits of the existing evidence. The [measurement audit](docs/performance/MeasurementAudit.md) recalculates key derived figures from the original reported timings. We have not received raw timing logs or profiler exports for independent primary-evidence verification.

## Limits and next steps

AeroForge is not a flight-certified simulator. It does not claim computational fluid dynamics, flexible-body/structural analysis, high-fidelity combustion, orbital mechanics, or a validated operational flight controller. Some design documents describe **proposed** formats, interfaces and future reference-mission capabilities; treat source files and executable tests as the evidence of current implementation. A complete visualization experience remains in development outside this commit.

**License:** No license file was included in the supplied repository inventory. Choose a license explicitly if you intend to permit outside redistribution or reuse.
