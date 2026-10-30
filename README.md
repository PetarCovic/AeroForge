# AeroForge

**A C++20 six-degree-of-freedom (6DOF) aerospace simulation, guidance, navigation, and control (GNC), visualization, and hardware-in-the-loop platform.**

AeroForge is a modular engineering project for simulating rigid-body motion, numerical integration, atmosphere and aerodynamics, rocket propulsion, sensors, navigation, guidance, control, actuators, faults, and repeatable experiments. It includes a desktop visualization application and both software and physical-controller hardware-in-the-loop (HIL) execution.

The **guided research rocket** is the platform's reference mission, not the limit of its intended architecture. AeroForge's components are designed for reuse across different aerospace vehicle models.

> **Project status:** The core simulation, 6DOF dynamics, supporting aerospace models, live visualization, software HIL, and physical ESP32 HIL have been implemented and exercised locally. The automated regression suite was reported passing after hardware HIL integration. Software HIL benchmark evidence and physical HIL benchmark evidence have passed internal-consistency audits. The full guided-rocket reference mission and manual hardware disconnect/recovery sign-off are **not** claimed as complete.
>
> **Timing distinction:** The physical HIL simulation maintained a tested 100 Hz physics schedule, but the ESP32 did **not** provide a new controller response on every physics step. See [measured HIL results](docs/performance/HilPerformanceReport.md).

## Highlights

- **6DOF simulation:** Translational and rotational rigid-body dynamics with quaternion attitude, numerical integration, forces, moments, mass, and inertia.
- **Aerospace models:** Standard atmosphere, wind, aerodynamics, rocket thrust, propellant depletion, and time-varying mass properties.
- **GNC and experiment infrastructure:** IMU/GPS models, inertial navigation, GPS correction, attitude control, actuator models, injected faults, Monte Carlo analysis, and seeded replay.
- **Interactive visualizer:** Recorded CSV playback, live physics, telemetry plots, trajectories, 3D vehicle attitude, events, performance diagnostics, and HIL monitoring.
- **Software and physical HIL:** Versioned binary sensor/control exchange with COBS framing, CRC-32, sequence validation, command freshness tracking, and watchdog fallback; a physical ESP32 communicates over USB-to-UART.
- **Engineering evidence:** GoogleTest/CTest regression coverage, analytical/integration tests, timed benchmarks, and a C++ tool for auditing saved benchmark evidence.

## Measured results

Recent measurements from the Windows x64 Release configuration are summarized below. These are observations from the recorded workloads, **not guarantees** for other hardware or configurations.

| Workload | Steps | Accepted controller responses | Relevant result |
|---|---:|---:|---|
| Software HIL, unpaced | 100,000 | 100,000 | Mean step computation **0.00497655 ms**; zero execution deadline misses |
| Software HIL, paced at 100 Hz | 10,000 | 10,000 | **0** execution/completion deadline misses; **0.1618 ms** p99 start lateness |
| Physical ESP32 HIL, 460800 baud, 100 Hz physics | 1,000 | 641 | **997/1,000** steps had a still-valid command; mean controller response **15.5717 ms**; zero observed simulation deadline misses |

The physical run reported **`HARDWARE ACCEPTANCE THRESHOLD NOT MET`** against its 90% fresh-response criterion. The physics runner can retain an earlier command while it remains valid; its high valid-command coverage does not mean a new command arrived every 10 ms. The physical benchmark's CSV/JSON **evidence audit passed with a warning**, which does not override the benchmark's performance result.

See [current verification results](docs/verification/CurrentVerificationResults.md), [HIL performance report](docs/performance/HilPerformanceReport.md), and [benchmark methodology](docs/BENCHMARKING_AND_PERFORMANCE.md).

## Architecture

```text
                          AeroForge
                              |
             +----------------+----------------+
             |                |                |
       Simulation Core     Aerospace        Experiment
             |             Models          Infrastructure
       +-----+-----+      +---+----+       +-----+------+
       |           |      |        |       |            |
    Math /       6DOF   Atmosphere Rocket  Faults /    Telemetry /
    Frames     Dynamics  / Aero     / Mass   Monte Carlo Replay
       |           |      |        |       |            |
       +-----------+------+--------+-------+------------+
                              |
                         GNC / Actuators
                              |
                     Simulation State / Clock
                              |
                  +-----------+-----------+
                  |                       |
             Visualization               HIL
                  |                       |
        Recorded and Live GUI     Software loopback
                                  or ESP32 over UART
```

Core models and numerical integration do not depend on the GUI. Simulation time is authoritative for physics; pacing and serial response timing are measured separately using host wall-clock time. The rendering thread consumes published state rather than advancing the simulation itself.

### Subsystems

| Area | Included functionality |
|---|---|
| Mathematics and geometry | `Vector3`, `Matrix`, `Quaternion`, transforms and coordinate-frame conversion |
| Numerical simulation | State/derivative models, Euler and RK4 integrators, simulation clock, fixed-step and paced runners |
| Dynamics | Translational, rotational, and 6DOF rigid-body models; accumulated forces and moments |
| Environment and aerodynamics | Atmosphere, wind, relative air velocity, air data, and aerodynamic force/moment models |
| Rocket propulsion | Thrust curves, engine, propellant depletion, mass/inertia variation, and rocket dynamics |
| Sensors and navigation | Accelerometer, gyroscope, IMU, GPS, sensor schedules, inertial navigation, GPS correction |
| Guidance and actuation | Attitude guidance, PD control, throttle, gimbal, body-moment actuator models |
| Experimentation | Fault schedules/injectors, Monte Carlo trials/statistics, seeded reproducibility and replay |
| Data and analysis | Telemetry recording, CSV export, event logs, flight/GNC analysis, benchmarking |
| Visualization | GLFW/OpenGL/Dear ImGui/ImPlot GUI; recorded and live data; trajectories; 3D attitude |
| Hardware-in-the-loop | Packet codec, stream parser, serial worker, HIL bridge, watchdog, timing monitor, and ESP32 firmware |

For an implementation-focused map, see [Current Implementation](docs/architecture/CurrentImplementation.md). The longer [System Architecture](docs/SYSTEM_ARCHITECTURE.md) describes the engineering design contract, including some forward-looking requirements.

## Physical conventions

AeroForge's baseline rigid-body state is a **13-component** representation:

| Indices | Quantity | Frame / representation |
|---|---|---|
| 0–2 | Position | Local navigation, North-East-Down (NED) |
| 3–5 | Velocity | NED |
| 6–9 | Attitude | Scalar-first body-to-navigation quaternion `q_NB` (`w, x, y, z`) |
| 10–12 | Angular velocity | Forward-Right-Down (FRD) body frame |

Physical calculations use SI units, radians for internal angles, and `double` precision in the core physics models. With the navigation origin at the reference ground plane, altitude is `-z_N`. A vertical rocket's body-forward axis should not be confused with the navigation-frame North axis.

The conventions are specified in [Coordinate Systems](docs/COORDINATE_SYSTEMS.md), [Units and Conventions](docs/UNITS_AND_CONVENTIONS.md), and [Simulation Scope and Assumptions](docs/SIMULATION_SCOPE_AND_ASSUMPTIONS.md).

## Quick start: build and test

### Requirements

The demonstrated build environment is **Windows x64 with MSVC**, CMake, Ninja, and a C++20 toolchain. The visualization target requires an OpenGL-capable environment; CMake fetches or configures GLFW, Dear ImGui, and ImPlot. GoogleTest is used for automated testing. Initial dependency retrieval may require network access.

Other compiler/OS configurations are not claimed as verified by the measurements in this README.

Open a PowerShell terminal with the x64 Visual Studio build environment configured and run these commands from the repository root.

### Configure

```powershell
cmake -S . -B out/build/x64-release-clean -G Ninja -DCMAKE_BUILD_TYPE=Release -DAEROFORGE_BUILD_VISUALIZATION=ON -DBUILD_TESTING=ON
```

### Build

```powershell
cmake --build out/build/x64-release-clean --parallel 8
```

To build the principal test, GUI, and HIL targets explicitly:

```powershell
cmake --build out/build/x64-release-clean --target AeroForgeTests AeroForgeVisualizer AeroForgeHilBenchmarks AeroForgeHilHardwareBenchmarks AeroForgeVerificationAudit --parallel 8
```

### Run the automated tests

```powershell
ctest --test-dir out/build/x64-release-clean --output-on-failure
```

These commands use the target names registered by the recent local HIL/verification integration. Run `cmake --build out/build/x64-release-clean --target help` if your checked-out revision differs from that configuration.

For troubleshooting, a headless build option, and a longer verification checklist, see [Build, Installation, and Validation](docs/INSTALL_AND_VALIDATION.md).

## Interactive visualization

Launch the desktop application:

```powershell
.\out\build\x64-release-clean\AeroForgeVisualizer.exe
```

The application supports these distinct data sources:

- **Recorded CSV playback:** Play, pause, seek, and inspect recorded telemetry and quaternion-based vehicle orientation.
- **Synthetic live demonstration:** Procedurally generated telemetry used to exercise the GUI; it is not physics output.
- **Physics-driven live 6DOF simulation:** A background C++ worker integrates the rigid-body model and publishes snapshots for the GUI.
- **Software HIL:** A controller exchanges simulated sensor/control packets within the host process.
- **Physical HIL:** The PC exchanges binary sensor/control packets with an external ESP32 over serial.

Available displays include the dashboard, telemetry plots, trajectory, 3D vehicle, events, performance diagnostics, and HIL monitor. Live GUI history is bounded; it is **not** a lossless recorder of every physics step.

For menu interactions, source switching, and appropriate physical-hardware configuration, read the [Visualizer User Guide](docs/visualization/VISUALIZATION_USER_GUIDE.md) and [Visualization Architecture](docs/visualization/VISUALIZATION_ARCHITECTURE.md).

### Recorded 6DOF example

Build the existing recording example:

```powershell
cmake --build out/build/x64-release-clean --target AeroForge6DoFRecordingExample --parallel 8
```

Create a destination and run it:

```powershell
New-Item -ItemType Directory -Force -Path examples\output
.\out\build\x64-release-clean\examples\AeroForge6DoFRecordingExample.exe "examples/output/rocket_6dof_flight.csv"
```

Open the generated CSV in the visualizer to inspect simulated position, velocity, and attitude over time. This is a simplified 6DOF rigid-body demonstration, **not** the fully integrated powered, guided research-rocket mission.

## Hardware-in-the-loop (HIL)

AeroForge supports software loopback and physical control exchange using the same logical sensor/control packet protocol. Its hardware runner uses asynchronous responses so that physics steps need not block on every UART round trip. Command sequence validation, freshness limits, and watchdog handling determine whether a returned command can be used; a zero-moment fallback is used when no acceptable command is available.

```text
AeroForge (host PC)                         External ESP32
-------------------                        --------------
6DOF physics / virtual IMU
         |
         +--- sensor packet ----> UART ----> packet validation
                                              rate controller
         <--- command packet --- UART <---- commanded moments
         |
command freshness / watchdog
         |
next physics integration step
```

### ESP32 hardware used for the recorded run

- **Chip:** Original ESP32-D0WD-V3 (revision 3.1), **not** ESP32-S3.
- **USB serial adapter:** CH340, observed as `COM3` on the development PC.
- **Firmware target:** ESP-IDF `esp32`.
- **Physical HIL application UART:** 460800 baud in the latest recorded benchmark.
- **Transport:** UART0 via the board's USB-to-serial connection; no external IMU or motor is needed.

The firmware source directory may still be named `firmware/esp32s3/` for historical reasons. **The ESP-IDF target must match the actual chip**, regardless of the directory name.

To build or flash the firmware from an ESP-IDF-enabled PowerShell terminal:

```powershell
cd firmware/esp32s3
idf.py set-target esp32
idf.py build
idf.py -p COM3 -b 115200 flash
```

Here, `115200` is the **flashing speed**, not necessarily the firmware's application UART speed. The AeroForge host baud must match the baud actually configured in the flashed firmware. Close `idf.py monitor` and other serial programs before opening COM3 from AeroForge.

See [ESP32 Setup](docs/hil/ESP32Setup.md), [HIL Architecture](docs/hil/HIL_ARCHITECTURE.md), and [Wire Protocol](docs/hil/HilWireProtocol.md) before changing firmware or UART settings.

### Software HIL benchmark

```powershell
.\out\build\x64-release-clean\AeroForgeHilBenchmarks.exe 100000 out\verification\software-unpaced
.\out\build\x64-release-clean\AeroForgeHilBenchmarks.exe 10000 out\verification\software-paced paced
```

The recorded software benchmark emits `hil_samples.csv` and `hil_summary.json` in each selected output folder. Audit the saved evidence with the C++ verification tool:

```powershell
.\out\build\x64-release-clean\AeroForgeVerificationAudit.exe software out\verification\software-unpaced\hil_samples.csv out\verification\software-unpaced\hil_summary.json out\verification\SoftwareAudit.md
.\out\build\x64-release-clean\AeroForgeVerificationAudit.exe software out\verification\software-paced\hil_samples.csv out\verification\software-paced\hil_summary.json out\verification\SoftwarePacedAudit.md
```

The two recorded software audits **passed**. They also noted that per-row CSV `deadline_misses` was unavailable, so execution miss totals were read from the JSON summaries.

### Physical HIL benchmark

After confirming the board's port and firmware application baud:

```powershell
.\out\build\x64-release-clean\AeroForgeHilHardwareBenchmarks.exe COM3 1000 out\hil-benchmarks\hardware-460800 460800
```

Audit the saved result:

```powershell
.\out\build\x64-release-clean\AeroForgeVerificationAudit.exe hardware out\hil-benchmarks\hardware-460800\hil_hardware_samples.csv out\hil-benchmarks\hardware-460800\hil_hardware_summary.json out\verification\HardwareAudit.md
```

The physical audit **passed with a response-rate warning**, while the underlying hardware benchmark **did not meet** its 90% fresh-command acceptance target. The serial performance improvement work has been deferred rather than misrepresented as complete.

**Safety and verification:** HIL tests described here use a simulated vehicle, not physical actuators. GUI USB disconnect, port release, zero-command fallback, and reconnection are separate manual checks; they have not been recorded as completed in the available verification evidence. Do not attach motors or other physical actuators during these checks.

## Verification and performance evidence

AeroForge uses GoogleTest/CTest component and integration tests, analytical comparisons, deterministic replay tests, numerical and performance benchmarks, and recorded verification evidence. Existing coverage includes numerical integration, 6DOF rigid-body dynamics, atmosphere/aerodynamics, propulsion, sensors, GNC, navigation, fault injection, Monte Carlo, telemetry, and visualization.

Three recent HIL evidence datasets passed the C++ audit: **100,000-step software unpaced**, **10,000-step software paced**, and **1,000-step physical HIL**. An evidence audit checks recorded-data consistency; it does not replace external physical validation or make a failed performance threshold pass.

Selected historical *different-workload* benchmarks, retained for engineering context:

| Historical workload | Reported result | Source |
|---|---:|---|
| Euler, two-state numerical baseline | 8.76 million integration steps/s | [Early Integration Baseline](docs/benchmarks/EarlyIntegrationBaseline.md) |
| RK4, two-state numerical baseline | 1.61 million integration steps/s | [Early Integration Baseline](docs/benchmarks/EarlyIntegrationBaseline.md) |
| CSV export, 10,000 samples, optimized | 40.527 ms (317.777 ms before optimization) | [Performance Report](docs/performance/PerformanceReport.md) |
| Generic RK4 integration step, optimized | 0.363 μs (0.916 μs before optimization) | [Performance Report](docs/performance/PerformanceReport.md) |
| Visualization GUI frame interval, 600-frame capture | 16.6 ms average, approximately 60.24 FPS | [Visualization Performance Report](docs/performance/VisualizationPerformanceReport.md) |

These results were collected with different harnesses and conditions. They must not be treated as a single matched end-to-end performance comparison. See [Measurement Audit](docs/performance/MeasurementAudit.md) and [Real-Time Execution Performance Report](docs/performance/RealtimeExecutionPerformanceReport.md) for context and limitations.

## Project structure

```text
AeroForge/
├── include/                     Public C++ subsystem interfaces
├── src/
│   ├── math/                    Math and geometry
│   ├── simulation/              Integrators, clocks, execution
│   ├── dynamics/                Translational, rotational, 6DOF
│   ├── environment/             Atmosphere and wind
│   ├── aerodynamics/            Aerodynamic model components
│   ├── rocket/                  Propulsion and variable mass
│   ├── sensors/                 Sensor simulation
│   ├── navigation/              State estimation/correction
│   ├── gnc/                     Guidance and controllers
│   ├── actuators/               Actuator models
│   ├── failures/                Fault injection
│   ├── montecarlo/              Statistical experiments
│   ├── reproducibility/         Replay and deterministic seeds
│   ├── telemetry/               Recording and analysis
│   ├── hil/                     Software/physical HIL runtime
│   └── visualization/           Core visualization and GUI
├── tests/                       Existing GoogleTest suites
├── benchmarks/                  Core, visualization, HIL workloads
├── examples/                    Standalone demonstrations
├── firmware/esp32s3/            ESP-IDF project (target: esp32)
├── tools/verification/          C++ benchmark evidence auditor
├── docs/                        Specifications, guides, reports
├── cmake/                       Supporting CMake integration
├── CMakeLists.txt
└── README.md
```

Generated build output, benchmark samples, and telemetry recordings should remain separate from source-controlled code unless intentionally curated as example data or release evidence.

## Documentation

**[Browse the complete documentation index](docs/README.md)** for all current guides, specifications, and preserved historical reports.

| Topic | Document |
|---|---|
| What is implemented | [Current Implementation](docs/architecture/CurrentImplementation.md) |
| Architecture and modeling scope | [System Architecture](docs/SYSTEM_ARCHITECTURE.md), [Scope and Assumptions](docs/SIMULATION_SCOPE_AND_ASSUMPTIONS.md) |
| Reference guided-rocket mission | [Reference Mission](docs/REFERENCE_MISSION.md) |
| Build and testing | [Installation and Validation](docs/INSTALL_AND_VALIDATION.md) |
| Coordinate and unit conventions | [Coordinate Systems](docs/COORDINATE_SYSTEMS.md), [Units](docs/UNITS_AND_CONVENTIONS.md) |
| Recorded/live GUI | [Visualizer User Guide](docs/visualization/VISUALIZATION_USER_GUIDE.md) |
| HIL architecture and protocol | [HIL Architecture](docs/hil/HIL_ARCHITECTURE.md), [Wire Protocol](docs/hil/HilWireProtocol.md) |
| ESP32 hardware setup | [ESP32 Setup](docs/hil/ESP32Setup.md) |
| Current measured verification | [Verification Results](docs/verification/CurrentVerificationResults.md), [HIL Performance](docs/performance/HilPerformanceReport.md) |
| Numerical and visualization performance | [Benchmarking](docs/BENCHMARKING_AND_PERFORMANCE.md), [Visualization Performance](docs/performance/VisualizationPerformanceReport.md) |
| Coding and reproducibility conventions | [Coding Standards](docs/CODING_STANDARDS.md), [Configuration and Reproducibility](docs/CONFIGURATION_AND_REPRODUCIBILITY.md) |

## Limitations and future work

AeroForge is an engineering simulation and learning/research project. It is **not** a flight-certified simulator, safety-qualified physical controller, or evidence of a complete flight-ready guidance system. Its detailed guided-rocket reference mission remains a broader integration and validation objective; the present live 6DOF demonstrations use simplified physical inputs.

Current limitations include:

- No claim of complete reference-mission physical validation against real flight data.
- No hard real-time guarantees from Windows host scheduling or CH340 USB serial communication.
- Physical HIL produced fewer fresh controller replies than 100 Hz physics steps and missed its defined fresh-response acceptance threshold.
- Manual physical-GUI disconnect, watchdog fallback, and reconnection validation has not yet been signed off in the reported evidence.
- Several advanced fidelity models described in design documents remain intended extensions rather than verified features.

Potential improvements include additional guided-flight integration, richer model validation, longer hardware endurance, serial throughput tuning, and broader compiler/platform coverage. Deferred enhancements will be organized in `NextSteps.md` after the release-preparation work is finished; that document is not presented as already existing.

## Licensing

No license is asserted by this README. Refer to the repository's actual `LICENSE` file, if present, for applicable permissions and conditions. Until a license is selected and published, do not assume the source is granted an open-source license.
