# AeroForge — Next Steps

Post-v0.1.0 development backlog. This list contains **unfinished or deferred work only**. Priorities reflect engineering value, correctness, and safety—not a commitment to implement everything.

## High Priority

- [ ] **Complete the integrated guided-rocket mission** — Combine propulsion, variable mass, aerodynamics, sensors, GNC, and 6DOF integration into one reproducible scenario.
- [ ] **Validate the integrated flight model** — Compare trajectories and event behavior against analytical/reference cases; document numerical tolerances and limitations.
- [ ] **Improve physical HIL connection and recovery** — Diagnose ESP32 USB reset/boot behavior and ensure clean reconnects start a fresh control session.
- [ ] **Verify physical HIL fail-safe behavior** — Test disconnects, stale commands, port contention, and applied zero-moment fallback with **no physical actuators connected**; record evidence.
- [ ] **Investigate physical HIL response throughput** — Measure where command updates are delayed and why fresh responses do not arrive on every 100 Hz physics step.
- [ ] **Define and test HIL command timing** — Specify command freshness, delayed responses, zero-order hold, queues, and timing effects on closed-loop stability.
- [ ] **Validate guidance and control performance** — Measure attitude tracking, disturbance recovery, actuator saturation, and differences between software and hardware timing.
- [ ] **Validate navigation and estimation** — Compare inertial/GPS-derived estimates against truth, including noisy and delayed measurements.
- [ ] **Expand robustness testing** — Run static analysis/sanitizers where supported; cover invalid telemetry, corrupt packets, concurrency, and resource cleanup.
- [ ] **Improve reproducibility metadata** — Record software revision, compiler, configuration, seeds, and environment for repeatable simulation and benchmark results.

## Medium Priority

- [ ] **Run extended physical HIL endurance tests** — Track memory, serial reliability, watchdog transitions, and recovery across long sessions.
- [ ] **Expand HIL protocol fault tests** — Exercise malformed COBS/CRC packets, invalid values, replayed sequences, MCU resets, and host/firmware compatibility.
- [ ] **Improve ESP32 firmware configuration** — Make baud, pins, controller gains, and limits configurable; rename the legacy `esp32s3` source directory safely.
- [ ] **Improve HIL visualization diagnostics** — Clarify command age, accepted/rejected responses, serial queues, applied controls, and measured timing; validate long-running plot behavior.
- [ ] **Expand numerical studies** — Check representative RK4 convergence, quaternion behavior, and appropriate conservation invariants.
- [ ] **Increase environment and vehicle-model fidelity** — Add richer wind, aerodynamic coefficient tables, actuator dynamics, and sensor errors where needed.
- [ ] **Extend Monte Carlo analysis** — Quantify uncertainty and failure rates across wind, thrust, mass, navigation, and controller parameters.
- [ ] **Add versioned scenario configuration** — Validate units, model selections, and reproducible seeds when loading simulations from configuration files.
- [ ] **Improve telemetry and analysis exports** — Export plots and full sampled results, including truth-versus-estimate and command-versus-applied comparisons.
- [ ] **Test compiler and platform portability** — Verify the portable core with GCC/Clang and isolate Windows-specific serial/timing code.
- [ ] **Harden CI and documentation** — Pin workflow dependencies, review permissions, check documentation links, and preserve measured benchmark evidence.
- [ ] **Test the release on another Windows environment** — Confirm standalone execution and runtime dependency requirements outside the development machine.

## Low Priority

- [ ] **Enhance 3D visualization** — Add optional rocket meshes, camera modes, force/moment vectors, and simulation-driven actuator animation.
- [ ] **Add advanced flight comparisons** — Show nominal versus fault-injected trajectories with synchronized plots and event timelines.
- [ ] **Support another vehicle type** — Reuse the simulation core with an aircraft or UAV model to demonstrate extensibility.
- [ ] **Explore advanced navigation and control** — Evaluate EKF/complementary filters or LQR/PID variants when validated scenarios justify them.
- [ ] **Explore higher-fidelity physics** — Consider ECEF/ECI frames, turbulence, flexible-body effects, or propellant slosh only for concrete requirements.
- [ ] **Evaluate adaptive integration** — Compare error-controlled integrators with existing fixed-step RK4 when model accuracy demands it.
- [ ] **Optimize heavy workloads** — Investigate parallel Monte Carlo, structured tracing, or binary telemetry only after profiling identifies a bottleneck.
