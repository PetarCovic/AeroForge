
# AeroForge — Phase 17 Visualization Performance Report

## 1. Overview

This document records performance measurements for AeroForge's
Phase 17 visualization platform.

The visualization platform includes:

- Recorded telemetry CSV ingestion and playback
- Interactive telemetry graphs using ImPlot
- North-East-Down trajectory visualization
- Quaternion-driven 3D vehicle wireframe
- Synthetic live telemetry demonstration
- Physics-driven live 6DOF simulation
- Event annotations and GUI performance diagnostics

Measurements were collected from a Windows Release build.

The results distinguish between:

1. CPU component microbenchmarks
2. GUI frame-interval measurements
3. Process memory measurements

These results characterize the tested implementation and
environment. They are not hardware-independent performance
guarantees.

---

## 2. Test Environment

| Component | Specification |
|---|---|
| CPU | AMD Ryzen 9 5900X |
| Physical CPU cores | 12 |
| Logical processors | 24 |
| GPU | NVIDIA GeForce RTX 3080 |
| GPU driver | 32.0.16.1088 |
| Installed RAM | 63.91 GiB, as reported by PowerShell |
| Operating system | Windows |
| Compiler | Microsoft Visual C++ / MSVC |
| Build system | CMake + Ninja |
| Build configuration | Release |
| C++ standard | C++20 |
| GUI | GLFW, OpenGL, Dear ImGui, ImPlot |
| Build directory | out/build/x64-release-clean |
| VSync | Requested via glfwSwapInterval(1) |
| Display resolution | Not recorded |
| Display refresh rate | Not recorded |

The source configuration was built using the x64 MSVC toolchain.

---

## 3. CPU Visualization Benchmarks

### 3.1 Benchmark Configuration

Executable:

`AeroForgeVisualizationBenchmarks.exe`

Results were collected from the clean Release build.

The benchmarks measure batches of operations. An iteration
is a complete workload of the size specified in the
benchmark name.

For example, `CSV_parse_50000` measures a batch of
50,000 rows, not a single CSV row.

### 3.2 Measured Results

| Benchmark | Iterations | Total wall time (s) | Mean time per iteration (ms) |
|---|---:|---:|---:|
| CSV_parse_50000 | 10 | 4.9023190 | 490.231900 |
| Plot_decimate_50000 | 300 | 0.0514357 | 0.171452 |
| Camera_projection_1000 | 1000 | 0.1263054 | 0.126305 |
| Snapshot_publish_50000 | 20 | 0.0292484 | 1.462420 |

Original benchmark output:

```text
checksum=1800000
benchmark,iterations,wall_seconds,mean_us_per_iteration
CSV_parse_50000,10,4.902319,490231.9
Plot_decimate_50000,300,0.0514357,171.452333333
Camera_projection_1000,1000,0.1263054,126.3054
Snapshot_publish_50000,20,0.0292484,1462.42
```

### 3.3 Interpretation

**CSV parsing**

A 50,000-row telemetry recording required approximately
490.23 ms per parsing operation.

This is a batch-level processing measurement, not the
latency of parsing a single row.

**Plot decimation**

Decimating a 50,000-point dataset required approximately
0.171 ms per operation.

This indicates that the decimation routine is inexpensive
relative to the frame intervals measured in the GUI.

The benchmark does not include complete GPU rendering.

**Camera projection**

Projecting 1,000 points through the CPU camera-projection
path required approximately 0.126 ms per batch.

This measures the projection calculation, not an entire
rendered OpenGL frame.

**Snapshot publishing**

Publishing 50,000 snapshots through the live publisher
required approximately 1.462 ms per benchmark iteration.

This measures the benchmark's publishing workload and does
not establish actual simulation-to-screen latency.

---

## 4. GUI Frame-Interval Measurements

### 4.1 Methodology

The AeroForge Performance panel uses FrameTimingStats
to measure intervals between consecutive GUI-loop
iterations.

The reported statistics were calculated from a
rolling window of 600 frame samples.

These intervals may include:

- CPU rendering and application work
- Waiting for VSync
- Operating-system scheduling
- Driver and presentation overhead

They do not isolate GPU execution time.

The active GUI workload was not recorded alongside this
measurement, so the results should not be attributed to
a specific playback or live simulation scenario.

### 4.2 Measured Results

| Metric | Result |
|---|---:|
| Window samples | 600 |
| Mean frame interval | 16.6 ms |
| Estimated FPS from mean interval | 60.24 |
| p50 frame interval | 16.6 ms |
| p95 frame interval | 16.853 ms |
| p99 frame interval | 17.044 ms |
| Frames exceeding 16.667 ms | 59.2% |
| Visual telemetry samples retained | 601 |

### 4.3 Interpretation

The observed mean frame interval corresponds to
approximately 60.2 frames per second.

The p95 and p99 frame intervals were approximately
16.85 ms and 17.04 ms, respectively.

A total of 59.2% of sampled frame intervals exceeded
the 16.667 ms reference budget.

This is important when interpreting the results:
the observed mean is close to the reference budget,
but a majority of sampled intervals exceeded it.

Because VSync was requested, the observed frame rate may
be influenced by monitor refresh timing and presentation
behavior.

The statistics demonstrate the frame intervals observed
during the sampled session. They do not establish
performance across every GUI workload.

---

## 5. Process Memory Measurements

### 5.1 Methodology

PowerShell sampled the AeroForgeVisualizer process using:

- WorkingSet64
- PrivateMemorySize64

One sample was collected approximately every second
for 60 samples.

The samples were exported to:

`out/build/x64-release-clean/visualization_memory.csv`

The active workload was not recorded with the memory
samples.

### 5.2 Measured Results

| Metric | Result |
|---|---:|
| Sample count | 60 |
| Average working set | 70.3308 MB |
| Minimum working set | 70.33 MB |
| Maximum working set | 70.34 MB |
| Observed working-set range | 0.01 MB |

These values use the PowerShell calculation
`WorkingSet64 / 1MB`, where `1MB` is 1,048,576 bytes.

### 5.3 Interpretation

Working-set memory remained nearly constant during the
observed sampling period.

The difference between the minimum and maximum displayed
working-set measurements was approximately 0.01 MB.

This is a favorable observation for this particular run.

However, one 60-sample working-set capture does not prove
the absence of memory leaks.

Additional testing across repeated recording loads,
source switching, and long-running sessions is useful.

The PrivateMB values were sampled by the original
PowerShell command but were not included in the reported
summary above.

---

## 6. Real 6DOF Simulation Integration

Phase 17 includes a physics-driven live visualization
source implemented by `SixDoFLiveSimulation`.

It uses:

- SixDoFDynamicsModel
- RK4Integrator
- SimulationRunner
- LiveTelemetryPublisher
- VisualizationSession

The live demonstration uses a 0.01-second simulation
timestep over 1,000 integration steps.

The worker publishes an initial snapshot and a snapshot
after each integration step when not cancelled.

The simulation uses simplified constant external forces,
moments, mass, and inertia.

It does not yet implement full powered-rocket flight,
propellant depletion, or active GNC control.

The simulation worker uses a separate C++ thread and
does not call OpenGL or Dear ImGui directly.

The latest-snapshot publisher does not guarantee that
every integrated simulation state is rendered.

---

## 7. Functional Validation

The following capabilities have been observed working:

- Recorded CSV loading and playback
- Correct display of altitude-only external flight data
- Quaternion-based 3D vehicle rotation from a generated
  6DOF recording
- Real 6DOF live simulation in AeroForge Visualizer
- Successful execution of the full automated test suite

The full regression suite was reported passing after
the live integration changes.

Exact final test counts and the final headless build
result should be recorded during release sign-off.

---

## 8. Measurement Limitations

The current measurements do not include:

- GPU-only render time
- End-to-end simulation-to-display latency
- Thread scheduling latency distributions
- Number of overwritten live publisher snapshots
- Long-term memory growth across hours
- Frame timing across repeated controlled workloads
- Multi-run confidence intervals

Frame intervals and component execution times are
different quantities and must not be compared as if
they measure the same operation.

No numerical GUI acceptance threshold was established
before these measurements. Therefore, this report records
observed performance rather than retroactively claiming
that a specific performance target was met.

---

## 9. Conclusions

AeroForge Phase 17 provides a working interactive
visualization system for recorded and live 6DOF
rigid-body simulation data.

In the measured Release build:

- Plot decimation of 50,000 points took about 0.171 ms.
- Projection of 1,000 camera points took about 0.126 ms.
- Publishing 50,000 snapshots took about 1.462 ms.
- The sampled GUI mean frame interval was 16.6 ms.
- The sampled GUI p95 interval was 16.853 ms.
- The observed average process working set was 70.33 MB.

These results establish a reproducible performance
baseline for future AeroForge optimization work.

More controlled per-workload GUI measurements and
long-duration reliability testing remain potential
follow-up improvements.

---

## 10. Reproduction Commands

Configure the Release build:

`cmake -S . -B out/build/x64-release-clean -G Ninja -DCMAKE_BUILD_TYPE=Release -DAEROFORGE_BUILD_VISUALIZATION=ON -DBUILD_TESTING=ON`

Build visualization benchmarks:

`cmake --build out/build/x64-release-clean --target AeroForgeVisualizationBenchmarks --parallel 8`

Build the visualizer:

`cmake --build out/build/x64-release-clean --target AeroForgeVisualizer --parallel 8`

Run all registered tests:

`ctest --test-dir out/build/x64-release-clean --output-on-failure`

Run the GUI:

`.\out\build\x64-release-clean\AeroForgeVisualizer.exe`

Run the benchmark executable from the Release build tree.

Capture GUI performance from the Performance panel and
collect process memory measurements independently.

---

## 11. Future Performance Work

Potential future improvements include:

1. GPU frame-time instrumentation.
2. Direct simulation-to-display latency measurements.
3. Publisher sequence numbers and overwrite accounting.
4. Additional controlled workloads with repeated trials.
5. Extended memory stability and stress testing.
6. Profiling camera rendering and plotting hot paths.

These are not prerequisites for the functional completion
of Phase 17.
