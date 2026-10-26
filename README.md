
# AeroForge

**C++20 six-degree-of-freedom (6DOF) aerospace simulation and guidance, navigation, and control (GNC) platform.**

AeroForge is a modular aerospace engineering platform for studying rigid-body flight dynamics, numerical integration, atmospheric and aerodynamic modeling, rocket propulsion, aerospace sensors, navigation, attitude control, stochastic failures, reproducible experiments, simulation performance, and interactive flight visualization.

Its reference application is a guided research rocket, but the underlying mathematics, simulation infrastructure, and visualization components are designed to be reusable across different aerospace systems.

> **Development status:** AeroForge includes implemented mathematical, dynamics, simulation, rocket, sensor, navigation, control, telemetry, and performance subsystems. Phase 17 adds an interactive visualization application supporting recorded telemetry playback and physics-driven live 6DOF simulation. These features have been built and exercised locally, with the automated regression suite reported passing.
>
> The complete, integrated guided-rocket reference mission remains a development goal. Implemented components and passing tests should not be interpreted as validation of a complete operational flight-control system.
>
> Phase 17 visualization changes are being finalized locally and may not yet be present in the remote repository.

---

## Features

### Implemented Subsystems

| Subsystem | Included functionality |
|---|---|
| Mathematics | `Vector3`, `Matrix`, `Quaternion`, geometry, and coordinate-frame transformations |
| Simulation | State and derivative abstractions, dynamics/integrator interfaces, Euler and RK4 integration, simulation clock, and simulation runner |
| Dynamics | Translational, rotational, and rigid-body 6DOF models; force and moment accumulation |
| Environment and aerodynamics | Standard atmosphere, wind, relative air velocity, air data, and aerodynamic modeling |
| Rocket modeling | Thrust curves, propulsion, propellant depletion, variable mass/inertia, and rocket dynamics |
| Sensors | Accelerometer, gyroscope, IMU, GPS, sensor timing, and measurement models |
| Navigation | Inertial navigation, GPS correction, and navigation-state propagation |
| Guidance and control | Attitude setpoints, hold-attitude guidance, PD attitude control, and control commands |
| Actuation | Throttle, gimbal, and body-moment actuator models |
| Robustness | Fault injection, Monte Carlo trials/statistics, seed management, and deterministic replay tests |
| Telemetry and analysis | Telemetry recording, sampling, CSV export, event annotations, flight/GNC analysis, and Monte Carlo reports |
| Execution and profiling | Real-time pacing, execution results, timing summaries, performance benchmarks, and regression testing |
| Visualization | CSV playback, interactive telemetry plots, 2D trajectories, 3D vehicle attitude, live telemetry, events, and GUI performance diagnostics |
| Live 6DOF integration | RK4-integrated rigid-body simulation running on a worker thread with thread-safe snapshot publishing |

**Implementation versus integration:** Individual subsystem implementations and their tests do not establish full end-to-end flight-system validation. The [reference mission](docs/REFERENCE_MISSION.md) defines the broader intended application.

---

## Architecture

AeroForge separates physical models, numerical integration, simulation execution, telemetry, and visualization.

```text
AeroForge
|
+-- Mathematics
|   +-- Vector and matrix operations
|   +-- Quaternion attitude representation
|   +-- Coordinate transformations
|
+-- Simulation
|   +-- State and StateDerivative
|   +-- DynamicsModel interface
|   +-- Euler and RK4 integrators
|   +-- SimulationRunner
|   +-- Real-time execution
|
+-- Aerospace Models
|   +-- Translational and rotational dynamics
|   +-- Rigid-body 6DOF dynamics
|   +-- Atmospheric and aerodynamic models
|   +-- Rocket engines and propellant
|   +-- Variable mass and inertia
|
+-- Guidance, Navigation, and Control
|   +-- Sensor models
|   +-- Navigation algorithms
|   +-- Attitude guidance
|   +-- PD control
|   +-- Actuator models
|
+-- Experiment Infrastructure
|   +-- Fault injection
|   +-- Monte Carlo simulation
|   +-- Reproducibility
|   +-- Telemetry recording
|   +-- Performance benchmarking
|
+-- Visualization
    +-- CSV telemetry loading
    +-- Recorded playback
    +-- Live telemetry publishing
    +-- 2D trajectory plots
    +-- 3D vehicle wireframe
    +-- GUI telemetry and event panels
    +-- Physics-driven live 6DOF demonstration
```

### Design Principles

AeroForge follows several core architectural principles:

- **Separation of concerns:** Physical dynamics are independent of numerical integrators and rendering.
- **Deterministic simulation:** Simulation time is authoritative; wall-clock pacing is handled separately.
- **Reusable components:** Mathematical and simulation abstractions are not tied to one rocket configuration.
- **Explicit coordinate conventions:** Position, velocity, orientation, and angular velocity have documented frames.
- **Testability:** Subsystems can be validated independently using analytical, integration, and regression tests.
- **Performance awareness:** Numerical and execution workloads are benchmarked using measured results.
- **Visualization isolation:** Rendering does not directly control the physics integrator.

See [System Architecture](docs/SYSTEM_ARCHITECTURE.md) for additional details.

---

## Coordinate and Numerical Conventions

AeroForge uses the following conventions:

| Property | Convention |
|---|---|
| Navigation frame | Right-handed North-East-Down (NED) |
| Navigation +X | North |
| Navigation +Y | East |
| Navigation +Z | Down |
| Body frame | Right-handed Forward-Right-Down (FRD) |
| Body +X | Forward, toward vehicle nose |
| Body +Y | Right |
| Body +Z | Down |
| Attitude | Body-to-navigation quaternion `q_NB` |
| Quaternion storage | Scalar-first `[w, x, y, z]` |
| Units | SI |
| Internal angles | Radians |
| Physical numerical precision | `double` |
| Authoritative simulation time | Simulation clock |

The baseline 6DOF rigid-body state contains 13 components:

| State indices | Quantity | Reference frame |
|---|---|---|
| 0–2 | Position | Navigation |
| 3–5 | Velocity | Navigation |
| 6–9 | Attitude quaternion | Body to navigation |
| 10–12 | Angular velocity | Body |

The attitude quaternion transforms vectors from body coordinates into navigation coordinates.

For altitude relative to the local NED origin:

\[
h = -z_N
\]

The rocket's body-forward axis is not assumed to align with the navigation frame's North axis.

See:

- [Coordinate Systems](docs/COORDINATE_SYSTEMS.md)
- [Units and Conventions](docs/UNITS_AND_CONVENTIONS.md)
- [Simulation Scope and Assumptions](docs/SIMULATION_SCOPE_AND_ASSUMPTIONS.md)

---

## Building AeroForge

### Prerequisites

The demonstrated development environment uses:

- Windows
- Visual Studio / Microsoft C++ compiler
- C++20
- CMake
- Ninja
- Git

Building the graphical visualizer additionally requires its configured GLFW, OpenGL, Dear ImGui, and ImPlot dependencies.

The testing configuration uses GoogleTest. An initial internet connection may be required when CMake fetches dependencies.

The commands below use the tested Windows x64 CMake/Ninja workflow. Other operating systems and compiler configurations have not been established by the supplied validation evidence.

### Configure a Release Build

From the repository root, using PowerShell:

```powershell
cmake -S . -B out/build/x64-release-clean -G Ninja -DCMAKE_BUILD_TYPE=Release -DAEROFORGE_BUILD_VISUALIZATION=ON -DBUILD_TESTING=ON
```

Use an x64 MSVC development environment. The compiler and linker must target the same architecture.

### Build All Targets

```powershell
cmake --build out/build/x64-release-clean --parallel 8
```

### Run Automated Tests

```powershell
ctest --test-dir out/build/x64-release-clean --output-on-failure
```

The automated regression suite has been reported passing following the Phase 17 live visualization changes.

### Build Individual Targets

Core library:

```powershell
cmake --build out/build/x64-release-clean --target AeroForgeCore --parallel 8
```

Tests:

```powershell
cmake --build out/build/x64-release-clean --target AeroForgeTests --parallel 8
```

Visualization application:

```powershell
cmake --build out/build/x64-release-clean --target AeroForgeVisualizer --parallel 8
```

Visualization benchmarks:

```powershell
cmake --build out/build/x64-release-clean --target AeroForgeVisualizationBenchmarks --parallel 8
```

Recorded 6DOF example:

```powershell
cmake --build out/build/x64-release-clean --target AeroForge6DoFRecordingExample --parallel 8
```

Benchmarks measure execution performance; they do not replace correctness or physical validation testing.

Do not compare Debug and Release benchmark results as if they were equivalent.

### Optional Headless Build

The visualization feature can be disabled when configuring a separate build:

```powershell
cmake -S . -B out/build/x64-headless -G Ninja -DCMAKE_BUILD_TYPE=Release -DAEROFORGE_BUILD_VISUALIZATION=OFF -DBUILD_TESTING=ON
```

Build:

```powershell
cmake --build out/build/x64-headless --parallel 8
```

Test:

```powershell
ctest --test-dir out/build/x64-headless --output-on-failure
```

This is an additional validation configuration. It should not be recorded as passing until independently executed successfully.

---

## Interactive Flight Visualization

Phase 17 introduces **AeroForge Visualizer**, a desktop visualization application built with GLFW, OpenGL, Dear ImGui, and ImPlot.

The visualizer supports recorded telemetry, live simulation data, interactive engineering plots, and vehicle attitude visualization.

### Visualization Panels

| Panel | Functionality |
|---|---|
| Dashboard | Current flight state, simulation time, and selected telemetry source |
| Playback | Playback controls, seeking, restarting, and playback speed |
| Telemetry | Interactive time-series plots of simulation data |
| Trajectory | North-East ground track and vertical flight profile |
| Vehicle 3D | Wireframe vehicle with quaternion-driven 3D orientation |
| Events | Time-stamped simulation and flight event annotations |
| Performance | GUI frame-interval statistics, FPS, and retained telemetry information |

Panels can be displayed, hidden, rearranged, and docked within the application.

### Launch the Visualizer

```powershell
.\out\build\x64-release-clean\AeroForgeVisualizer.exe
```

The visualizer supports three telemetry modes:

1. Recorded telemetry playback
2. Synthetic live demonstration
3. Physics-driven live 6DOF simulation

### Recorded Flight Playback

Select:

**File > Open telemetry CSV...**

Load an AeroForge-compatible telemetry recording.

Supported playback interactions include:

- Play and pause
- Restart
- Seek to a selected simulation time
- Adjust playback speed
- Inspect synchronized trajectory and telemetry
- View recorded vehicle attitude

The Vehicle 3D panel uses recorded body-to-navigation quaternions to orient the vehicle wireframe.

The trajectory panel displays horizontal motion when horizontal position data is available. For altitude-only datasets, it can display the vertical profile without inventing horizontal motion.

---

## Recorded 6DOF Simulation Example

AeroForge includes a 6DOF example that generates calculated rigid-body flight telemetry.

Source:

`examples/rocket_6dof_flight.cpp`

Executable target:

`AeroForge6DoFRecordingExample`

### Build the Example

```powershell
cmake --build out/build/x64-release-clean --target AeroForge6DoFRecordingExample --parallel 8
```

### Generate Telemetry

Create the output directory if necessary:

```powershell
New-Item -ItemType Directory -Force -Path examples\output
```

Run:

```powershell
.\out\build\x64-release-clean\examples\AeroForge6DoFRecordingExample.exe "examples/output/rocket_6dof_flight.csv"
```

Output:

`examples/output/rocket_6dof_flight.csv`

The current example generates 1,001 snapshots across a 10-second simulation at a 0.01-second timestep.

The recording includes:

- Navigation-frame position
- Navigation-frame velocity
- Body-to-navigation quaternion
- Body-frame angular velocity
- Associated visualization telemetry

The example uses AeroForge's existing 6DOF dynamics and RK4 numerical integration.

Its applied forces, moments, mass, and inertia are simplified. It is a rigid-body demonstration, not a complete powered-rocket flight model.

### Replay the Flight

1. Launch AeroForge Visualizer.
2. Select **File > Open telemetry CSV...**.
3. Load `examples/output/rocket_6dof_flight.csv`.
4. Open the Trajectory, Telemetry, and Vehicle 3D panels.
5. Play the recording or seek between different flight times.

The vehicle wireframe should change orientation as the recorded attitude quaternion evolves.

Recorded 6DOF quaternion-driven vehicle rotation has been validated in the local application.

---

## Physics-Driven Live 6DOF Simulation

AeroForge supports real-time visualization of its own numerical physics simulation.

Unlike recorded playback, this mode runs the simulation while the GUI is active.

### Start a Live Simulation

Launch AeroForge Visualizer, then select:

**File > Start 6DOF live simulation**

The application runs `SixDoFDynamicsModel` through `SimulationRunner` and `RK4Integrator`.

The simulation is executed on a separate C++ worker thread.

### Live Simulation Data Flow

```text
SixDoFDynamicsModel
        |
        v
RK4Integrator
        |
        v
SimulationRunner
        |
        v
13-State Rigid-Body State
        |
        v
VisualizationSnapshot
        |
        v
LiveTelemetryPublisher
        |
        v
VisualizationSession
        |
        +--> Dashboard
        |
        +--> Telemetry
        |
        +--> Trajectory
        |
        +--> Vehicle 3D
        |
        +--> Events / Performance
```

### Current Live Simulation Configuration

| Parameter | Configuration |
|---|---|
| Dynamics | `SixDoFDynamicsModel` |
| Integrator | RK4 |
| Simulation timestep | 0.01 seconds |
| Total integration steps | 1,000 |
| Simulated duration | 10 seconds |
| Execution | Separate C++ worker thread |
| Timing | Real-time pacing using a steady clock |
| State publishing | `LiveTelemetryPublisher` |
| Cancellation | `std::stop_token` |
| GUI history capacity | 2,048 samples |

The worker computes position, velocity, attitude, and angular velocity and publishes corresponding snapshots.

The worker does not invoke GLFW, OpenGL, Dear ImGui, or ImPlot directly.

### Starting and Stopping

The File menu provides:

- **Start synthetic live demo**
- **Start 6DOF live simulation**
- **Stop live source**

Starting one live source stops the previous source.

Loading a recorded CSV also stops the active simulation worker.

The application requests cancellation and joins the simulation thread during shutdown.

### Latest-Snapshot Semantics

`LiveTelemetryPublisher` retains the latest published snapshot.

This allows the GUI to retrieve recent simulation state without maintaining an unbounded publisher-side queue.

However, intermediate snapshots may be overwritten before the GUI displays them.

Live visualization should therefore not be interpreted as lossless telemetry recording.

Use recorded telemetry output when a complete flight history is required.

### Physical Modeling Limitations

The current physics-driven live demonstration uses simplified constant forces, moments, mass, and inertia.

It does not yet integrate all of AeroForge's available rocket, aerodynamic, sensor, navigation, and GNC components into a single guided flight.

Estimated state fields in this demonstration copy the simulated truth state. They do not represent independent navigation-estimator results.

The live demonstration validates the physics-to-visualization integration rather than the complete guided-rocket reference mission.

---

## Verification and Testing

AeroForge includes automated testing across its mathematical, numerical, dynamics, sensor, navigation, control, telemetry, and execution components.

Its verification approach includes:

- Analytical checks
- Unit tests
- Integration tests
- Numerical consistency checks
- Deterministic simulation tests
- Regression tests
- Performance benchmarks

### Phase 17 Visualization Validation

The visualization implementation has been exercised with:

- Recorded telemetry loading
- Playback and seeking
- External altitude-only telemetry
- 6DOF recorded simulation telemetry
- Quaternion-driven vehicle rotation
- Synthetic live telemetry
- Real 6DOF live simulation
- GUI performance diagnostics
- Automated regression testing

The full automated suite was reported passing after live 6DOF integration.

Specific source-switching, cancellation, and long-duration stability claims should be based on their corresponding tests rather than inferred from basic GUI operation.

Detailed plans and results are available in:

- [Verification and Validation Plan](docs/VERIFICATION_AND_VALIDATION_PLAN.md)
- [Installation and Validation](docs/INSTALL_AND_VALIDATION.md)
- [Visualization Performance Report](docs/performance/VisualizationPerformanceReport.md)

---

## Performance and Benchmarking

AeroForge includes numerical simulation and visualization benchmarks.

Results are workload-specific and reflect the tested configuration.

Historical core-simulation benchmarks and newer Phase 17 visualization results were collected in different sessions. They must not be treated as directly comparable before/after measurements.

### Historical Numerical Performance

| Workload / configuration | Reported result | Provenance |
|---|---:|---|
| Euler integration, two-variable state, 25,000 steps × 5 runs | 8.76 million steps/s | [Integration baseline](docs/benchmarks/EarlyIntegrationBaseline.md) |
| RK4 integration, two-variable state, same historical baseline | 1.61 million steps/s | [Integration baseline](docs/benchmarks/EarlyIntegrationBaseline.md) |
| CSV telemetry export, 10,000 samples | 40.527 ms/export, versus 317.777 ms before optimization | [Performance report](docs/performance/PerformanceReport.md) |
| Generic RK4 integration step | 0.363 µs/step, versus 0.916 µs before optimization | [Performance report](docs/performance/PerformanceReport.md) |
| Full 6DOF RK4 integration step | 1.194 µs/step, versus 1.740 µs before optimization | [Performance report](docs/performance/PerformanceReport.md) |
| Nominal execution stack, ten 10-second runs at 1 kHz | 100 simulated seconds in 0.0731406 wall seconds; 1,367.23× real-time | [Real-time performance](docs/performance/RealtimeExecutionPerformanceReport.md) |
| Continuous nominal-stack execution, 300 simulated seconds | 0.234582 wall seconds; 1,278.87× real-time | [Real-time performance](docs/performance/RealtimeExecutionPerformanceReport.md) |

These are historical reported measurements, not independent physical accuracy validations.

### Phase 17 Visualization Benchmark Environment

| Hardware | Specification |
|---|---|
| CPU | AMD Ryzen 9 5900X |
| CPU cores / threads | 12 / 24 |
| GPU | NVIDIA GeForce RTX 3080 |
| GPU driver | 32.0.16.1088 |
| RAM | 63.91 GiB reported |
| Build | Windows x64, CMake/Ninja, Release |

### CPU Visualization Benchmarks

| Benchmark workload | Mean batch time |
|---|---:|
| CSV parsing, 50,000 rows | 490.232 ms |
| Plot decimation, 50,000 points | 0.171 ms |
| Camera projection, 1,000 points | 0.126 ms |
| Snapshot publishing, 50,000 snapshots | 1.462 ms |

Each result is the execution time for the complete named batch, not a single row, point, or snapshot.

### GUI Frame-Interval Measurements

| Metric | Measured result |
|---|---:|
| Sample window | 600 frames |
| Mean frame interval | 16.6 ms |
| Estimated frame rate | 60.24 FPS |
| p50 frame interval | 16.6 ms |
| p95 frame interval | 16.853 ms |
| p99 frame interval | 17.044 ms |
| Frames exceeding 16.667 ms | 59.2% |

### Memory Measurements

| Metric | Measured result |
|---|---:|
| Samples | 60 |
| Average working set | 70.3308 MiB |
| Minimum working set | 70.33 MiB |
| Maximum working set | 70.34 MiB |

The frame and memory captures were not labeled with a specific active GUI workload.

Frame-interval measurements may include VSync, CPU work, driver behavior, and operating-system scheduling. They are not GPU-only rendering times.

The 60-sample working-set results indicate stable memory use during that sampled interval, but do not establish the absence of memory leaks across longer workloads.

For benchmark methodology, interpretation, and limitations, see:

- [Benchmarking and Performance](docs/BENCHMARKING_AND_PERFORMANCE.md)
- [Measurement Audit](docs/performance/MeasurementAudit.md)
- [Visualization Performance Report](docs/performance/VisualizationPerformanceReport.md)

---

## Repository Structure

```text
AeroForge/
|
+-- include/
|   +-- math/
|   +-- simulation/
|   +-- dynamics/
|   +-- environment/
|   +-- aerodynamics/
|   +-- rocket/
|   +-- sensors/
|   +-- navigation/
|   +-- gnc/
|   +-- actuators/
|   +-- failures/
|   +-- montecarlo/
|   +-- reproducibility/
|   +-- telemetry/
|   +-- performance/
|   +-- visualization/
|
+-- src/
|   +-- Core subsystem implementations
|   +-- visualization/
|       +-- Live telemetry
|       +-- Recorded telemetry
|       +-- Live 6DOF simulation
|       +-- GUI application and panels
|
+-- tests/
|   +-- Core subsystem tests
|   +-- Integration tests
|   +-- visualization/
|
+-- benchmarks/
|   +-- Core benchmarks
|   +-- visualization/
|
+-- examples/
|   +-- ballistic_projectile.cpp
|   +-- rocket_6dof_flight.cpp
|   +-- data/
|   +-- output/              Generated recordings; not required in Git
|
+-- docs/
|   +-- Technical specifications
|   +-- Verification and validation
|   +-- Visualization documentation
|   +-- performance/
|   +-- benchmarks/
|
+-- CMakeLists.txt
+-- README.md
```

The repository contains both reusable simulation infrastructure and standalone examples.

Generated build files, benchmark outputs, temporary datasets, and local GUI configuration files are not required for the documented source baseline.

---

## Technical Documentation

| Document | Purpose |
|---|---|
| [System Architecture](docs/SYSTEM_ARCHITECTURE.md) | Subsystems, data flow, timing, and architectural boundaries |
| [Reference Mission](docs/REFERENCE_MISSION.md) | Intended guided-rocket demonstration and acceptance criteria |
| [Simulation Scope and Assumptions](docs/SIMULATION_SCOPE_AND_ASSUMPTIONS.md) | Physics fidelity, approximations, and exclusions |
| [Coordinate Systems](docs/COORDINATE_SYSTEMS.md) | Coordinate frames, quaternion conventions, and transformations |
| [Units and Conventions](docs/UNITS_AND_CONVENTIONS.md) | SI quantities, angle, time, and altitude conventions |
| [Configuration and Reproducibility](docs/CONFIGURATION_AND_REPRODUCIBILITY.md) | Seeds, replay, and proposed mission configuration contract |
| [Telemetry and Numerical Safety](docs/TELEMETRY_AND_NUMERICAL_SAFETY.md) | Recording, reporting, invariants, and error handling |
| [Verification and Validation Plan](docs/VERIFICATION_AND_VALIDATION_PLAN.md) | Analytical, integration, regression, and statistical checks |
| [Coding Standards](docs/CODING_STANDARDS.md) | C++20 conventions and engineering review criteria |
| [Benchmarking and Performance](docs/BENCHMARKING_AND_PERFORMANCE.md) | Benchmarking methodology, reproducibility, and comparisons |
| [Early Integration Baseline](docs/benchmarks/EarlyIntegrationBaseline.md) | Historical Euler and RK4 performance |
| [Performance Report](docs/performance/PerformanceReport.md) | Numerical-core and telemetry optimization measurements |
| [Real-Time Performance Report](docs/performance/RealtimeExecutionPerformanceReport.md) | Real-time execution and workload-specific measurements |
| [Measurement Audit](docs/performance/MeasurementAudit.md) | Derived measurement verification and evidence limitations |
| [Visualization Architecture](docs/VISUALIZATION_ARCHITECTURE.md) | Recorded/live visualization architecture and thread ownership |
| [Visualization User Guide](docs/VISUALIZATION_USER_GUIDE.md) | Playback, live modes, panels, and interaction |
| [Installation and Validation](docs/INSTALL_AND_VALIDATION.md) | Build procedures and Phase 17 validation |
| [Visualization Performance Report](docs/performance/VisualizationPerformanceReport.md) | Phase 17 CPU, GUI, and memory measurements |

Some design documents also describe proposed capabilities. The presence of a design specification does not imply every described integration has been completed.

---

## Current Limitations

AeroForge is an engineering simulation and research project, not a flight-certified simulator or operational flight controller.

It does not claim:

- Computational fluid dynamics
- Flexible-body or structural dynamics
- High-fidelity combustion modeling
- Orbital mechanics
- Flight certification
- Validated real-world guidance or control authority
- Complete end-to-end validation of the guided research-rocket reference mission

Although AeroForge contains propulsion, navigation, guidance, control, sensor, and failure-modeling components, their presence does not mean every component has been integrated into a single validated mission.

The Phase 17 live visualization example uses simplified rigid-body dynamics and does not replace a full powered-rocket or closed-loop GNC demonstration.

---

## Future Development

Potential future development includes:

1. Completing the integrated guided-rocket reference mission.
2. Connecting realistic thrust, propellant depletion, variable mass, aerodynamics, navigation, and GNC into a single flight scenario.
3. Validating the complete simulated flight against analytical results and suitable reference data.
4. Expanding failure-injection and Monte Carlo experiments for integrated missions.
5. Adding more complete live telemetry transport and recording capabilities.
6. Extending visualization performance instrumentation to GPU timings and simulation-to-display latency.
7. Improving vehicle visualization and engineering analysis workflows.

These are future development directions rather than claims of functionality already validated in Phase 17.

---

## License

No license was included in the previously documented repository inventory.

A license should be selected and added explicitly before permitting external redistribution or reuse under defined terms.
