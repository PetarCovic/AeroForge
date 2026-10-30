# AeroForge Engineering Documentation

AeroForge is a C++20 aerospace simulation platform with a six-degree-of-freedom dynamics engine, numerical integration, environment and propulsion models, guidance/navigation/control components, telemetry, visualization, and software/physical hardware-in-the-loop (HIL) execution.

**Evidence standard:** A detailed design specification describes *intended* behavior; source and tests show *implemented* behavior; benchmark output establishes *measured* behavior under specific conditions. These are not interchangeable. The flagship guided-research-rocket reference mission remains a design/validation objective rather than a demonstrated complete flight campaign.

## Start here

| Goal | Document |
|---|---|
| See what exists and what remains | [Implemented System Overview](architecture/CurrentImplementation.md) |
| Build, run tests, and launch the GUI | [Build, Installation, and Validation](INSTALL_AND_VALIDATION.md) |
| Use recorded/live visualization | [Visualizer User Guide](visualization/VISUALIZATION_USER_GUIDE.md) |
| Configure the ESP32 | [ESP32 HIL Setup](hil/ESP32Setup.md) |
| Interpret current verified measurements | [Current Verification Results](verification/CurrentVerificationResults.md) |
| Understand HIL timing and limitations | [HIL Performance Report](performance/HilPerformanceReport.md) |

## System specifications

- [System Architecture](SYSTEM_ARCHITECTURE.md) — architectural contract and module relationships; includes forward-looking design.
- [Simulation Scope and Assumptions](SIMULATION_SCOPE_AND_ASSUMPTIONS.md) — model fidelity and exclusions.
- [Reference Mission](REFERENCE_MISSION.md) — detailed aspirational guided-research-rocket requirements and success criteria.
- [Coordinate Systems](COORDINATE_SYSTEMS.md) — NED/FRD frames and quaternion conventions.
- [Units and Physical Conventions](UNITS_AND_CONVENTIONS.md) — authoritative SI-unit contracts.
- [Telemetry and Numerical Safety](TELEMETRY_AND_NUMERICAL_SAFETY.md) — diagnostics and mathematical-safety requirements.
- [Configuration and Reproducibility](CONFIGURATION_AND_REPRODUCIBILITY.md) — seed/replay contracts and configuration design; not all proposed JSON-schema features are verified.
- [Coding Standards](CODING_STANDARDS.md) — engineering conventions, not a certification of conformance.
- [Verification and Validation Plan](VERIFICATION_AND_VALIDATION_PLAN.md) — general testing strategy and analytical validation methodology.

## Visualization and HIL

- [Visualization Architecture](visualization/VISUALIZATION_ARCHITECTURE.md)
- [Visualizer User Guide](visualization/VISUALIZATION_USER_GUIDE.md)
- [HIL Architecture](hil/HIL_ARCHITECTURE.md)
- [HIL Wire Protocol](hil/HilWireProtocol.md)
- [ESP32 Setup](hil/ESP32Setup.md)

## Performance and measured evidence

- [Benchmarking and Performance Engineering](BENCHMARKING_AND_PERFORMANCE.md) — interpretation guidance and historical numerical benchmarks.
- [Early Integrator Baseline](benchmarks/EarlyIntegrationBaseline.md) — historical Euler/RK4 scaling data.
- [Numerical Performance Report](performance/PerformanceReport.md) — historical profiling and optimizations.
- [Real-Time Execution Performance Report](performance/RealtimeExecutionPerformanceReport.md) — historical pacing and throughput results.
- [Measurement Audit](performance/MeasurementAudit.md) — arithmetic cross-checks for historical reports; not new execution evidence.
- [Visualization Performance Report](performance/VisualizationPerformanceReport.md) — historical GUI, parsing, and memory measurements.
- [HIL Performance Report](performance/HilPerformanceReport.md) — current software and hardware HIL measurements, including known limitations.
- [Current Verification Results](verification/CurrentVerificationResults.md) — automated results, evidence audits, and outstanding manual sign-offs.

## Working with the evidence

The latest software HIL verification recorded 100,000 accepted commands in an unpaced run and 10,000 accepted commands in a 100 Hz paced run. The physical ESP32 benchmark recorded 1,000 physics steps and 641 accepted new responses at 460800 baud. Its benchmark response-acceptance threshold was **not met**, even though its saved evidence passed the consistency audit.

Do not claim guaranteed hard real-time behavior, complete guided rocket mission validation, or manual hardware disconnect/reconnect test success from these numbers alone. See the corresponding reports for exact conditions and limitations.

## Repository and documentation hygiene

Preserve historical measured data. Prefer updating obsolete status descriptions over deleting valuable records. If a document moves, update relative links that reference it. Development milestones belong in planning discussions, not source or documentation names. Deferred enhancements will be collected in `NextSteps.md` after the release roadmap is complete.
