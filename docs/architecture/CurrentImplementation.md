# AeroForge Implemented System Overview

## Purpose

This document connects the platform's architectural **design specifications** to the subsystems and benchmark behavior that have actually been implemented or reported. The longer [System Architecture](../SYSTEM_ARCHITECTURE.md), [Reference Mission](../REFERENCE_MISSION.md), and [Simulation Scope and Assumptions](../SIMULATION_SCOPE_AND_ASSUMPTIONS.md) documents include forward-looking requirements and should not be read as a claim that the complete guided-rocket flagship mission has been demonstrated.

## Simulation core

The C++20 core contains mathematical vector/matrix/quaternion types; frame transforms; simulation state and timekeeping; Euler and RK4 integration; a general simulation runner; and paced execution infrastructure. The modeled 6DOF state comprises navigation-frame position/velocity, a scalar-first body-to-navigation quaternion, and body-frame angular velocity.

AeroForge uses SI units internally, a right-handed North-East-Down local navigation frame, and Forward-Right-Down body axes as documented in the normative contracts:

- [Units and Physical Conventions](../UNITS_AND_CONVENTIONS.md)
- [Coordinate Systems](../COORDINATE_SYSTEMS.md)

## Dynamics and aerospace modules

The current source tree includes translational/rotational dynamics and mass/inertia handling, standard-atmosphere and wind support, aerodynamic calculations, and rocket propulsion/mass-depletion components. Their availability as individual components must not be confused with an end-to-end validated guided ascent trajectory.

## Guidance, navigation, and faults

The source tree includes IMU/GPS sensor models, inertial navigation and GPS correction, attitude guidance and PD control, actuator models, fault schedules/injectors, reproducibility infrastructure, and Monte Carlo analysis. Existing GoogleTests cover representative component, integration, analytical, and deterministic replay behavior.

## Telemetry and visualization

The telemetry subsystem records simulation observations and supports analysis/export. The GUI uses GLFW, OpenGL, Dear ImGui, and ImPlot for recorded playback, live 6DOF visualization, plots, and HIL monitoring. Rendering is separate from the authoritative physics clock and should not mutate the flight state.

## HIL execution

Software HIL executes a software controller in the host process; physical HIL sends simulated sensor packets to an ESP32-D0WD-V3 over CH340 USB-UART. Commands return asynchronously and can be held for several physics steps while still valid. Expired/invalid command handling uses a zero-body-moment fallback. The recorded hardware benchmark did not achieve a new physical command for each 10 ms physics step; this distinction is part of the published performance limitations.

See [HIL architecture](../hil/HIL_ARCHITECTURE.md), [wire protocol](../hil/HilWireProtocol.md), and [performance](../performance/HilPerformanceReport.md).

## Implementation boundaries and evidence

| Claim | Basis | Qualification |
|---|---|---|
| 6DOF/RK4 engine present | Current source structure and existing tests | Reference mission's complete rocket ascent remains a separate validation goal |
| Software loopback runs | 100,000-step and 10,000-step current benchmarks | Test workload and machine specific |
| 100 Hz software physics pacing | 10,000-step paced benchmark | Zero *observed* deadline misses, not hard real-time certification |
| ESP32 external control functions | 10/10 serial smoke test; 1,000-step physical benchmark | Only 641 fresh responses in the 1,000-step run |
| Benchmark evidence internally consistent | C++ auditor reports | Physical response-rate target still not met |
| GUI physical fault recovery | Planned manual verification | Not established as passed by available results |

## Current repository entry points

| Subsystem | Representative source locations |
|---|---|
| Core C++ interfaces | `include/` |
| Simulation / dynamics | `src/simulation/`, `src/dynamics/` |
| Environment / propulsion | `src/environment/`, `src/aerodynamics/`, `src/rocket/` |
| GNC and fault logic | `src/sensors/`, `src/navigation/`, `src/gnc/`, `src/actuators/`, `src/failures/` |
| Reproducibility and telemetry | `src/reproducibility/`, `src/telemetry/` |
| HIL | `src/hil/`, `benchmarks/hil/` |
| Visualization | `src/visualization/`, `src/visualization/gui/` |
| Tests | `tests/` |
| Firmware | `firmware/esp32s3/` (legacy folder name; configured target is `esp32`) |

## Future work separation

New capabilities, serial tuning, full rocket mission integration, portability, and optional HIL improvements should be documented as future work until implemented and measured. The release roadmap will collect the deferred improvements in a dedicated `NextSteps.md` after the main release preparation is complete.
