# AeroForge — Next Steps and Engineering Backlog

> **Status:** Proposed future work, not a claim that any item has been implemented. Prioritize measured gaps over feature count. This backlog is independent of development-stage numbering.
>
> **Working baseline:** C++20 6DOF simulation with Euler/RK4, reusable aerospace/GNC models, recorded and live visualization, software HIL, and physical ESP32-D0WD-V3 HIL over CH340 USB-UART. Recent software HIL runs completed 100,000 unpaced and 10,000 paced steps with all corresponding commands accepted; the paced run reported zero observed execution/completion deadline misses. The recorded physical run at 460800 baud completed 1,000 simulation steps, received 641 accepted controller responses, used valid control on 997 steps, and had zero observed execution/completion deadline misses. These are workload-specific results, not hard-real-time or flight-certification claims.

## Prioritization rules

- **Release-critical:** correctness, reproducibility, clarity, legal/distribution readiness, and demonstrated failure behavior.
- **High impact:** improves defensibility in technical reviews, closes a measured bottleneck, or makes the system easier to extend.
- **Optional research:** expands fidelity or platform scope only after a repeatable baseline is available.
- Every optimization starts with a hypothesis, an instrumented baseline, acceptance criteria, and repeatable test conditions.
- Extend existing tests rather than creating duplicate suites. Use separate hardware/manual evidence where physical behavior cannot be verified in a host-only unit test.

## A. Release readiness and unresolved validation

### A1. Complete hardware GUI failure-mode sign-off — release-critical

- [ ] Confirm the visualizer's physical HIL mode connects to the actual ESP32 using a baud rate matching flashed firmware (most recent tested application rate: 460800 baud).
- [ ] Confirm the HIL Monitor displays real serial state, accepted/rejected response counts, watchdog state, and zero-command fallback indicators.
- [ ] Stop the session and verify COM3 is released; reopen it successfully from a fresh session.
- [ ] Open COM3 in another process and confirm the visualizer handles port contention without crashing or hanging.
- [ ] Unplug the ESP32 during a run **with no actuators connected**; verify that the **actual applied** body-moment command transitions to zero when command validity expires.
- [ ] Reconnect and start a **new** session; verify that old commands cannot be reused across resets or new sequence sessions.
- [ ] Record timestamps, screenshots/logs, expected behavior, observed behavior, and any failure in a verification report.

**Done when:** all checks have evidence and no unsafe/stale applied command behavior remains. Do not equate an unchanging GUI icon with physical failsafe proof.

### A2. Activate and verify repository CI — release-critical

- [ ] Commit the actual current local C++ source, firmware, documentation, CMake, and workflow YAMLs to the intended GitHub branch.
- [ ] Confirm Windows headless and visualization builds complete in GitHub Actions; run existing CTest tests.
- [ ] Confirm original ESP32 firmware builds in CI with an explicit `esp32` target despite the legacy `firmware/esp32s3` directory name.
- [ ] Manually run the software HIL benchmark/evidence-audit workflow; inspect artifacts, including runner environment and timing variability.
- [ ] Investigate any failing action versions, download restrictions, missing assets, toolchain mismatches, or runtime DLL packaging.
- [ ] Pin action versions to immutable commit SHAs when stabilizing production CI; review third-party action permissions.
- [ ] Introduce a basic branch protection rule requiring the validated build/test checks before merging, if the repository workflow warrants it.

**Done when:** the workflows visibly pass on the published source revision. YAML presence alone is not proof of successful CI.

### A3. Make Windows releases genuinely runnable — release-critical

- [ ] Test a clean clone and fresh Release configuration on another Windows machine or isolated environment.
- [ ] Confirm all required runtime DLLs, Visual C++ redistributable requirements, assets, relative paths, and GUI resources are accounted for.
- [ ] Audit CMake target output locations under both Ninja and Visual Studio multi-config generators. Verify draft-release scripts locate the right executables.
- [ ] Check dependency licenses for GLFW, Dear ImGui, ImPlot, GoogleTest, and ESP-IDF components; add the project `LICENSE` only after choosing terms.
- [ ] Include operating-system support, toolchain versions, hardware requirements, known limitations, and verification evidence in release notes.
- [ ] Generate and verify SHA-256 checksums for release artifacts.
- [ ] Create a draft release and install/run it independently before publishing.

**Done when:** a reviewer can obtain the published package, run a documented demo, and verify its provenance without relying on the developer's workspace.

### A4. Static analysis and robustness — high impact

- [ ] Enable MSVC `/analyze` or clang-tidy on the supported code path; address confirmed issues instead of suppressing warnings wholesale.
- [ ] Run compiler warning builds at `/W4 /permissive-` and Clang/GCC `-Wall -Wextra -Wpedantic` where supported.
- [ ] Run AddressSanitizer and relevant undefined-behavior checks for supported target/compiler combinations; exclude only genuinely incompatible platform-specific modules.
- [ ] Review use of references to mutable force/moment accumulators; document object lifetime and thread ownership.
- [ ] Review COM-handle RAII cleanup, `std::jthread` cancellation, queue bounds, exception propagation, and output-path validation.
- [ ] Make incomplete/corrupt CSV, non-finite configuration, invalid matrix/quaternion, oversized frame, and unexpected endpoint closure failure modes explicit.
- [ ] Where applicable, adopt fuzz testing for the COBS stream parser and packet decoder with bounded memory and recovery assertions.

### A5. Documentation correctness and media — high impact

- [ ] Add actual visualizer screenshots, a short physics demo, and a physical HIL demonstration video with measured overlays.
- [ ] Keep the flagship guided-rocket mission labeled as an *integrated future goal* until independently validated.
- [ ] Use measured benchmark numbers with build, machine, workload, sample count, clock, and timestamp; preserve historical data separately.
- [ ] Maintain a module/method-level engineering handbook after major API changes.
- [ ] Check documentation links and remove stale development-stage identifiers from repository-facing content.
- [ ] Add meaningful CI badges only after the workflows exist and pass.

## B. Physical HIL performance and reliability

### B1. Characterize the actual command-update bottleneck — high impact

**Existing evidence:** at 115200 baud, 1,000 steps returned 639 accepted responses, ~22.07 ms mean latency; at 460800 baud, 641 accepted responses, ~15.57 ms mean latency. The simulation itself maintained a nominal 10 ms schedule with zero observed deadline misses. The approximately 64% response ratio is **not proof of 359 lost frames**: late, queued, unprocessed, or filtered responses can affect the count.

- [ ] Distinguish **sent sensor samples**, **queued samples**, **completed UART transmissions**, **received frames**, **decoded commands**, **rejected commands**, and **applied fresh commands** with separate counters.
- [ ] Separate host serialization, serial write, CH340/USB, MCU receive, controller computation, MCU transmit, and host decode latency where hardware timestamps permit.
- [ ] Review `HilSerialWorker` buffering, nonblocking `WindowsSerialTransport` reads, polling cadence, and queue backpressure.
- [ ] Investigate the firmware's blocking `uart_wait_tx_done()` and UART read timeout; compare nonblocking TX or event-driven receive after confirming protocol invariants.
- [ ] Run controlled throughput/latency trials at 115200, 230400, and 460800 baud; repeat across machines and loads.
- [ ] Record mean, median, p95, p99, maximum, missing/duplicate/stale command counts, and watchdog events.
- [ ] Use a predeclared practical target, e.g. ≥90% newly accepted commands at 100 Hz **if and only if** the target is necessary for the intended controller requirement; do not weaken checks simply to make a benchmark green.

### B2. Asynchronous command pipeline and timing contract — high impact

- [ ] Formally document whether the controller acts on sample `n` during physics step `n`, `n+1`, or whenever it arrives; forbid future-state access.
- [ ] Make `latest fresh command`, `zero-order hold`, time-to-live, queue replacement, and deadline behavior configurable and observable.
- [ ] Model transport delay explicitly and compare closed-loop stability at 20, 50, 100 Hz physical command rates with 100 Hz physics integration.
- [ ] Test response reordering, sequence wraparound, MCU reset, stale replay, and command held during intermittent communication.
- [ ] Consider bounded queue/drop-oldest semantics when the producer outruns the controller; distinguish loss by policy from corruption.

### B3. Endurance, fault injection, and hardware variation — high impact

- [ ] Run long-duration physical sessions with RAM/handle tracking and explicit reconnect cycles.
- [ ] Inject corrupt CRCs, truncated COBS frames, invalid lengths, wrong protocol versions, NaN/Inf floats, duplicate and future sequences.
- [ ] Verify watchdog fallback by examining *applied moments*, not merely packet counters.
- [ ] Compare an original ESP32 using CH340 with an ESP32-S3 native USB Serial/JTAG board **only after** the current setup is well characterized; document USB/UART transport differences.
- [ ] Add host↔firmware byte-level protocol compatibility vectors, preferably common goldens, before changing the wire format.

### B4. Firmware maintainability — medium impact

- [ ] Rename the legacy firmware directory from `esp32s3` to an accurate, hardware-neutral location and update CMake, docs, and CI references atomically.
- [ ] Make UART baud, pins, watchdog threshold, controller gains, and command limits configurable through a checked ESP-IDF Kconfig or compile-time build contract.
- [ ] Add firmware version/protocol compatibility identification if needed; distinguish planned HELLO/STATUS messages from implemented ones.
- [ ] Consider an explicit controller fault status and a safe command on recovery, followed by a new session handshake rather than implicit state continuation.
- [ ] Preserve an independent safe shutdown policy if physical actuators are ever introduced.

## C. Simulation mathematics and physical validity

### C1. Complete and independently validate the reference guided-rocket mission — highest model-value

- [ ] Combine rocket thrust curves, propellant depletion, time-varying mass/inertia, atmosphere, aerodynamics, sensor models, navigation, guidance, actuators, and 6DOF integration in one reproducible scenario.
- [ ] Document ignition, ascent, burnout, coast, apogee, and termination logic.
- [ ] Verify force and moment accounting, thrust application point, center-of-mass movement, quaternion invariants, and event ordering.
- [ ] Compare accessible analytical/reference cases, including ballistic segments and burnout/coast trajectories, within justified tolerances.
- [ ] Compare noisy navigation against truth; measure estimator error and attitude tracking errors explicitly.
- [ ] Run Monte Carlo trials across launch, wind, thrust, mass-property, and sensor uncertainties; report failure probabilities and credible uncertainty ranges.
- [ ] Create a single recorded and live demo with plots, 3D attitude, event timeline, and downloadable results.

**Done when:** the full integrated mission, not merely its individual components, passes documented numerical, integration, and scenario-specific physical validations.

### C2. Numerical convergence and conservation — high impact

- [ ] Repeat RK4 convergence against known solutions and smaller timesteps for representative 6DOF/rocket cases; report observed order where assumptions permit.
- [ ] Track energy, momentum, and angular-momentum invariants only in appropriate force-free/conservative models.
- [ ] Stress quaternion renormalization and verify no attitude-frame sign flips or discontinuous GUI interpolation.
- [ ] Audit variable mass/inertia treatment and coupling between propellant state and integration stages; clarify whether mass is updated continuously or discretely.
- [ ] Evaluate adaptive RK methods only if error control is needed; keep fixed-step integration for reproducible HIL when appropriate.

### C3. Fidelity extensions — optional research

- [ ] Add layered wind, gust spectra, turbulence, altitude-dependent gravity, and density variation once reference comparisons exist.
- [ ] Add aerodynamic coefficient tables, Mach/angle-of-attack interpolation, aerodynamic moments, and uncertainty envelopes.
- [ ] Consider sensor correlation, latency, dropouts, temperature drift, GPS outage, and inertial bias evolution.
- [ ] Consider actuator deadband, backlash, dynamics, power limits, and coupled failure behavior.
- [ ] If requirements justify it, progress from flat local Earth/NED to ECEF/ECI and rotating-Earth models; preserve explicit transforms and domain boundaries.
- [ ] Flexible-body/aeroelastic, propellant slosh, CFD, and high-fidelity combustion are separate research modules, not trivial upgrades.

## D. GNC, estimation, and mission analysis

### D1. Controller comparisons — high impact

- [ ] Compare current PD attitude controller and embedded rate damper with PID/LQR where the linearized operating point is justified.
- [ ] Quantify overshoot, settling time, steady-state error, command saturation, and recovery after disturbances.
- [ ] Model sample-and-hold and delayed control explicitly; compare SIL/HIL behavior to ideal in-process control under the same initial conditions.
- [ ] Evaluate actuator limits and anti-windup if adding integral control.
- [ ] Add a clear controller tuning procedure with repeatable trial data.

### D2. Navigation and state estimation — high impact

- [ ] Compare inertial propagation, GPS correction, and independent estimation filters against truth over reproducible routes.
- [ ] Consider complementary/EKF filtering after defining state, noise covariances, observability, and update schedules.
- [ ] Add sensor time alignment and delayed-measurement treatment before claiming estimator performance.
- [ ] Report estimation errors and failure responses separately from true vehicle motion.

### D3. Vehicle extensibility — optional

- [ ] Demonstrate the same 6DOF/integrator/environment stack with a different model (aircraft or UAV) while keeping vehicle-specific forces/moments in dedicated components.
- [ ] Introduce a mission/scenario loader only after defining a tested, versioned schema and strict validation of units, paths, and model selections.
- [ ] Extend events, Monte Carlo sweeps, and reproducible configuration snapshots for scenario comparisons.

## E. Developer experience, portability, and observability

### E1. Configuration and reproducibility — high impact

- [ ] Implement or verify versioned configuration loading, schema validation, SI unit handling, and deterministic seed streams.
- [ ] Record git revision, build type, compiler, CPU/OS, input config hash, RNG seeds, and dependency versions in run metadata where available.
- [ ] Provide a one-command reproducibility recipe that recreates a selected benchmark/flight recording from a clean clone.
- [ ] Use recorded fixture/golden baselines sparingly and pair them with analytically justified checks.

### E2. Portable core and packaging — medium impact

- [ ] Build/test the portable simulation core with Clang and GCC; separate Win32 COM transport and high-resolution waitable-timer interfaces behind platform boundaries.
- [ ] Retain Windows as the reference GUI/HIL environment until comparable platform validation exists.
- [ ] Consider CMake presets, CPack or installer generation after verifying actual dependency redistribution.
- [ ] Confirm tests are discoverable and runnable without proprietary IDE project files.

### E3. Visualization and analysis — medium impact

- [ ] Show truth versus estimated states clearly; never label copied truth as a measured navigation estimate.
- [ ] Add controller-latency distribution, command age, watchdog transitions, and serial queue depth to the HIL Monitor.
- [ ] Provide standardized figure exports (trajectory, altitude, angular rates, commanded/applied moments, error bands).
- [ ] Add controlled trajectory sampling and export to avoid mistaking bounded latest-snapshot GUI history for complete telemetry recording.
- [ ] Measure simulation-to-display latency separately from GUI frame time and numerical execution time.

### E4. More advanced engineering — optional

- [ ] Add structured events/tracing for simulation, controller, and transport scheduling without placing I/O inside timed inner loops.
- [ ] Compare single-thread, worker-thread, and event-driven designs on measurable workloads before changing threading architecture.
- [ ] Explore parallel independent Monte Carlo trials only after confirming deterministic per-trial seeds and reproducible aggregation.
- [ ] Consider binary telemetry or columnar export only if CSV is demonstrably a bottleneck.

## Work selection template

For each chosen item, record:

1. **Problem / evidence:** what currently fails or is limited, with reproducible data.
2. **Hypothesis:** specific suspected cause or model discrepancy.
3. **Options:** the current approach and credible alternatives, with expected costs.
4. **Acceptance criteria:** measurable correctness, timing, or usability outcome.
5. **Implementation:** minimal relevant changes, naming and ownership rationale.
6. **Validation:** existing tests to rerun, new targeted coverage only for genuine gaps, and hardware/manual checks when appropriate.
7. **Result:** measured before/after, regressions, limitations, and next follow-up.

## Recommended order after the initial release

1. Prove outstanding physical disconnect/recovery behavior and stabilize CI/release packaging, if not completed.
2. Finish the guided-rocket end-to-end integrated reference mission and corresponding validation.
3. Add more precise physical HIL instrumentation and diagnose the current throughput bottleneck.
4. Demonstrate delayed-control behavior and improve the HIL pipeline only where measured benefit warrants it.
5. Expand estimation/GNC comparisons, realistic uncertainty, and Monte Carlo experiments.
6. Improve portability, configuration, visualization evidence, and optional high-fidelity models.

**Non-goals:** hardware actuation or real-flight deployment without a dedicated safety design and appropriate engineering review; invented performance guarantees; adding multiple alternative frameworks that solve no measured problem; rewriting validated components purely for novelty.