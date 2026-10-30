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

- [ ] Capture portfolio screenshots only after the GUI layout, HIL plotting, and data-source labeling checks in **Section F**; make 3D flight, telemetry, and HIL state easy to see in a single view.
- [ ] Add an actual 6DOF physics demo, a separate software HIL demo, and a verified physical ESP32 HIL video; display measured metrics without suggesting the physical controller supplies a new command on every 100 Hz physics step.
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

- [ ] Complete the **release-facing GUI polish and correctness work in Section F** before capturing definitive screenshots or demos.
- [ ] Show truth versus estimated states clearly; never label copied truth as a measured navigation estimate.
- [ ] Add controller-latency distribution, command age, watchdog transitions, and serial queue depth to the HIL Monitor without confusing response latency with simulation computation time.
- [ ] Provide standardized figure exports (trajectory, altitude, angular rates, commanded/applied moments, error bands).
- [ ] Add controlled trajectory sampling and export to avoid mistaking bounded latest-snapshot GUI history for complete telemetry recording.
- [ ] Measure simulation-to-display latency separately from GUI frame time and numerical execution time.

### E4. More advanced engineering — optional

- [ ] Add structured events/tracing for simulation, controller, and transport scheduling without placing I/O inside timed inner loops.
- [ ] Compare single-thread, worker-thread, and event-driven designs on measurable workloads before changing threading architecture.
- [ ] Explore parallel independent Monte Carlo trials only after confirming deterministic per-trial seeds and reproducible aggregation.
- [ ] Consider binary telemetry or columnar export only if CSV is demonstrably a bottleneck.

## F. GUI clarity, plotting correctness, and portfolio demonstrations

> **Motivation:** The current Dear ImGui / ImPlot interface exposes working flight plots, playback, performance metrics, and HIL status. Screenshots nevertheless show narrow side panels, clipped or crowded labels, considerable unused space, a hidden-by-default Vehicle 3D view, and a large floating HIL Monitor covering other panels. These are usability and presentation opportunities, **not evidence that the physics engine is incorrect**. Keep the tested simulation/HIL subsystems unchanged unless an independently reproduced defect requires a fix.
>
> **Evidence caution:** Screenshot data from `visualization_demo_SYNTHETIC.csv` is synthetic and must be labeled as such. The physical HIL screenshot showed `RUNNING` serial worker but `WAITING` watchdog and **zero accepted commands**; do not portray that particular capture as a healthy closed-loop hardware demonstration. Seek a new, genuinely healthy capture after investigating its cause. The plots' peculiar ranges are observed symptoms that require inspection, not confirmed timestamping bugs.

### F1. Investigate HIL plot ranges and live-time behavior — release-critical

- [ ] Reproduce the HIL Monitor case where the X axes span roughly **0.01–0.03 seconds** while the monitor reports **hundreds of steps**. Inspect the actual stored telemetry timestamps, ring-buffer retention, units, ImPlot axis limits, and auto-fit behavior before changing code.
- [ ] Check that **simulation time in seconds** is consistently used on X axes; verify whether any values are mistakenly interpreted as milliseconds, indices, recent-sample offsets, or fixed initial limits.
- [ ] Verify that plots in live 6DOF mode advance as samples arrive, with an appropriate moving/auto-fit window and a way to inspect complete recorded history when available.
- [ ] Fix or make configurable the case where live telemetry occupies only the beginning of a fixed **30-second** horizontal range during a short live run; do not force automatic zoom in recorded playback when a user has selected a deliberate range.
- [ ] Validate empty buffers, the first sample, a long-running stream, source changes, simulation restarts, and out-of-order/duplicate timestamps without frozen axes or crashes.
- [ ] Make minuscule execution and communication latencies legible with meaningful units and auto-scaled Y axes. Do not use a common 10–25 ms Y scale when actual observations are microseconds, unless that comparison is intentional and clearly labeled.
- [ ] Distinguish per-step **computation duration**, start-time **scheduler lateness**, controller **round-trip response latency**, and **USB/serial** transport effects; show only instrumented metrics and accurately label them.

**Done when:** the displayed axes match actual sampled timestamps and value magnitudes in software and physical HIL, the latest point is visibly advancing during live runs, and recorded playback preserves usable navigation/zoom behavior. Capture before/after screenshots and existing telemetry evidence.

### F2. Introduce task-specific docked layouts — high impact before demos

- [ ] Keep **Dear ImGui docking** and existing rendering architecture; avoid rebuilding the GUI or adding a second GUI framework solely for appearance.
- [ ] Create or refine default workspaces named **Flight Overview**, **GNC Analysis**, **Software HIL**, and **Hardware HIL** using existing panel infrastructure.
- [ ] Make **Vehicle 3D visible alongside Trajectory** in the Flight Overview rather than hidden behind an inactive tab. Retain a resizeable arrangement suitable for typical laptop and desktop displays.
- [ ] Give software and hardware HIL a dedicated **dockable monitor workspace**, not a near-full-screen floating overlay. Preserve the ability to detach or maximize it when desired.
- [ ] Avoid allocating large default areas to an empty Events panel or disabled Playback controls. Allow users to show or hide these panels without losing access to functionality.
- [ ] Provide **Reset Layout** and remember intentional user adjustments without making new users inherit a confusing local layout.
- [ ] Verify presentation at normal Windows scaling and at 125%/150% DPI, including window resizing and a smaller display. Confirm no clipping, overdraw, or inaccessible controls.

**Done when:** opening the application or selecting a workspace reveals the most relevant 3D/plot/health information immediately, with no obscured principal panel and no need for extensive manual rearrangement.

### F3. Replace cramped text readouts with a readable flight summary — high impact

- [ ] Present clearly labeled, aligned values for **simulation time, source/mode, altitude, speed, orientation, angular rates, and active/paused state**, as supported by the actual telemetry.
- [ ] Use compact numerical readouts for key metrics with explicit units; retain full-precision details in an expandable diagnostics view or tooltip.
- [ ] Fix current Dashboard and Performance panel truncation, long line wrapping, horizontal crowding, and overlapping header/status elements.
- [ ] Use consistent section hierarchy, padding, font sizes, numeric precision, and contrast; preserve technical precision without forcing users to read dense, tiny monospace paragraphs.
- [ ] Treat **recorded**, **synthetic live**, **physics-driven live**, **software HIL**, and **physical HIL** as visibly distinct data sources. Never imply that a synthetic test file or copied truth state is a measured estimator result.
- [ ] Clearly indicate absent instrumentation (for example, `not measured`) rather than reporting a misleading zero.

**Done when:** all essential values can be read at a glance in a normal window size and a screenshot makes the active simulation mode and measured data provenance unambiguous.

### F4. Reorganize HIL status and health reporting — high impact before demos

- [ ] Add a compact top-line status grouping for **transport type**, **COM port/baud** when physical, **serial worker**, **watchdog**, and **effective control authority**.
- [ ] Surface **simulation-step count/rate**, queued/dropped sensor samples, accepted/rejected responses, accepted-response ratio, valid-held-control steps, zero-command fallback steps, and deadline misses when available.
- [ ] Show the **age of the last accepted command** and differentiate `WAITING`, `HEALTHY`, `TIMED_OUT`, disconnected, and fault conditions. Define exact displayed states from source rather than inventing state names.
- [ ] Group plots by **simulation execution**, **communications processing**, **response latency**, and eventually **commanded versus applied body moments**; include units and measured sample counts.
- [ ] Investigate the actual hardware screenshot where the serial worker was `RUNNING` but the watchdog remained `WAITING`, accepted commands stayed at **0**, and dropped/queued counts rose. Verify firmware baud compatibility, firmware start/reset behavior, transport RX/TX counts, framing, and sequence validation before using it as a portfolio demo.
- [ ] Ensure a healthy screenshot/video shows an actual **accepted-command count increasing**, valid applied moments, a healthy watchdog, and zero fallback only when valid. Do not claim that the command rate equals 100 Hz unless measured.
- [ ] Make controller disconnection, fault, and stop transitions visually clear while testing the **applied command** and not only the status label.

**Done when:** a reviewer can tell in seconds whether the physics loop is running, the external controller is responding, commands are valid, and the simulation is applying them safely. Screenshots must be supported by benchmark logs or a reproducible demo run.

### F5. Prepare reproducible, honest demo scenarios — high impact before release

- [ ] Provide a **real RK4-driven 6DOF recorded/live demonstration** showing visible vehicle rotation, trajectory evolution, and synchronized telemetry. Prefer it over the synthetic CSV for the main GitHub showcase.
- [ ] Use a clear, modestly interesting physically justified motion example; do not fabricate a guided powered-rocket ascent or autonomous controller capabilities that are not integrated and verified.
- [ ] Prepare a software HIL closed-loop example with a visible angular-rate decay or other measurable control effect, corresponding commanded/applied moments, a readable response count, and no misleading synthetic labels.
- [ ] Prepare a physical ESP32 HIL example with the board and GUI visible, **matching 460800 baud firmware and host settings** where applicable, accepted commands increasing, and telemetry/command age accurately displayed.
- [ ] Record a fault/disconnection sequence **only with no actuators attached and after the manual safety procedure has passed**, including a visible transition to fallback zero and a new-session recovery.
- [ ] Capture a small set of stills: **Flight Overview**, **Vehicle 3D / trajectory**, **Software HIL health**, **Physical HIL healthy**, and **verification/performance evidence**. Annotate what is simulated, measured, synthetic, or hardware-derived.
- [ ] Keep source/configuration, build revision, hardware, and reproducible steps near each demo asset; use short captions and avoid screenshots containing personal paths or sensitive local configuration.

**Done when:** a viewer can reproduce each demo and understand what is real physics, what is simulated controller behavior, what is physical hardware integration, and what performance claims are actually supported.

### F6. Visual consistency and usability details — medium impact

- [ ] Refine the existing dark theme with consistent spacing, panel borders, axis-label legibility, legends, plot colors, and modest visual hierarchy; prioritize accessibility and clarity over decorative effects.
- [ ] Apply consistent plot colors to **truth**, **estimate**, **command**, and **applied output**, using readable legends and color-independent labels when feasible.
- [ ] Provide intentional defaults for visible channels, legend placement, scroll/zoom behavior, and an optional **follow-live** mode.
- [ ] Add explanatory tooltips for technical metrics (for example, `deadline miss`, `response latency`, `estimated state`, `watchdog`) so a new evaluator can interpret the data without reading source code.
- [ ] Keep keyboard/mouse controls predictable; audit layout persistence and cleanup on close/restart.

### F7. Advanced visualization — optional research after release

- [ ] Replace or complement the wireframe with an optional higher-quality 3D rocket mesh while retaining a low-cost engineering wireframe mode.
- [ ] Add orbit, follow, chase, and body-fixed camera views only when the required transform and camera behavior are tested.
- [ ] Display optional force, moment, velocity, angular-rate, and actuator vectors with properly labeled reference frames and scales.
- [ ] Add vehicle thrust and actuator motion animation driven by real simulation values, not arbitrary decorative movement.
- [ ] Provide a flight-event timeline for ignition, burnout, coast, apogee, faults, and recovery **only for events the integrated scenario actually produces**.
- [ ] Compare nominal versus fault-injected runs with side-by-side plots, matched time axes, and clearly distinguished random seeds/configurations.

**Boundary:** these additions are future presentation/research features, not prerequisites for publishing a correct simulation library. Do not sacrifice determinism, measurement integrity, or HIL safety to make the interface more cinematic.

## Work selection template

For each chosen item, record:

1. **Problem / evidence:** what currently fails or is limited, with reproducible data.
2. **Hypothesis:** specific suspected cause or model discrepancy.
3. **Options:** the current approach and credible alternatives, with expected costs.
4. **Acceptance criteria:** measurable correctness, timing, or usability outcome.
5. **Implementation:** minimal relevant changes, naming and ownership rationale.
6. **Validation:** existing tests to rerun, new targeted coverage only for genuine gaps, and hardware/manual checks when appropriate.
7. **Result:** measured before/after, regressions, limitations, and next follow-up.

## Recommended execution order

### Before final screenshots and the initial release

1. Confirm any outstanding physical disconnect/recovery safety behavior (**A1**); do not release a physical HIL demonstration that has not passed its safety review.
2. Check and correct observable GUI plot/time-axis issues (**F1**) and inspect the hardware HIL monitor's zero-accepted-command capture (**F4**).
3. Improve docking, default workspaces, the Vehicle 3D/Trajectory arrangement, readable metrics, and data-source labeling (**F2–F4**).
4. Prepare and record an honestly labeled, reproducible 6DOF/Software HIL/physical HIL showcase (**F5**); add screenshots/links to documentation (**A5**).
5. Finish packaging/license/dependency validation and review the draft GitHub release (**A3**). Keep proven CI and code-quality results documented (**A2**, **A4**).

### After the initial release

1. Finish the guided-rocket end-to-end integrated reference mission and corresponding validation (**C1**).
2. Add more precise physical HIL instrumentation and diagnose the current throughput bottleneck (**B1**).
3. Demonstrate delayed-control behavior and improve the HIL pipeline only where measured benefit warrants it (**B2–B4**).
4. Expand estimation/GNC comparisons, realistic uncertainty, and Monte Carlo experiments (**C2–C3**, **D**).
5. Improve portability, versioned configuration, observability, and detailed visual presentation (**E**, **F6–F7**).

**Non-goals:** hardware actuation or real-flight deployment without a dedicated safety design and appropriate engineering review; invented performance guarantees; adding multiple alternative frameworks that solve no measured problem; rewriting validated components purely for novelty.
