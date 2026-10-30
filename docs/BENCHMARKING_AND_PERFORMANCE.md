# AeroForge Benchmarking and Performance Engineering

> **Scope and evidence standard (October 2026):** Sections covering numerical, optimization, and real-time measurements preserve historical report data and their documented limitations. Later software and physical HIL runs have separate saved CSV/JSON evidence and auditor results; they are documented in [`performance/HilPerformanceReport.md`](performance/HilPerformanceReport.md) and [`verification/CurrentVerificationResults.md`](verification/CurrentVerificationResults.md). Benchmark results from different workloads are not directly comparable. Verify command-line options against the executable in the local build.

## 1. Questions the measurements answer

AeroForge performance studies answer three distinct questions:

1. **Numerical throughput** — how much CPU time individual math, integrator, dynamics, control, telemetry, and Monte Carlo operations consume in a particular benchmark workload.
2. **Optimization efficacy** — how much the latency of exactly the *same* workload changed before and after a measured code change.
3. **Execution-system behavior** — whether real-time pacing meets a requested wall-clock schedule and, in unpaced mode, how much simulated time is propagated per second of wall time.

These are not interchangeable. A 6DOF RK4 microbenchmark does not include every operation in a complete mission, and faster-than-real-time unpaced execution does not demonstrate a hard real-time guarantee.

## 2. Evidence hierarchy and source documents

The following documents are original report data preserved with clarification:

- [`benchmarks/EarlyIntegrationBaseline.md`](benchmarks/EarlyIntegrationBaseline.md): **October 14, 2026** historical Euler/RK4 scaling measurements; 25,000 steps × 5 runs per state size; includes elapsed times, throughput, per-step cost and checksums.
- [`performance/PerformanceReport.md`](performance/PerformanceReport.md): optimized Release build, CPU profiler study, benchmark suites, before-and-after telemetry and RK4 timing, sample counts and representative post-optimization microbenchmarks.
- [`performance/RealtimeExecutionPerformanceReport.md`](performance/RealtimeExecutionPerformanceReport.md): separately recorded unpaced, real-time and fixed-rate behavior, integrated nominal-flight throughput, incremental subsystem loading and a 300-second simulated-duration run.
- [`performance/MeasurementAudit.md`](performance/MeasurementAudit.md): arithmetic recomputation and exact interpretation limits for the numbers above.

**Evidence tiers:** historical report = recorded measurement; verified calculation = arithmetic reproduced from reported numbers; evidence audit = automated checks on an available CSV/JSON pair; independent reproduction = a new run with recorded source revision, raw logs, and environment metadata. The recent HIL datasets have passed an internal-consistency audit, but that is not independent external validation.

## 3. Experimental environments

### 3.1 Early integrator baseline (reported October 14, 2026)

| Parameter | Reported setting |
| --- | --- |
| Operating system | Windows 11 Home |
| CPU | AMD Ryzen 9 5900X 12-Core Processor |
| Compiler | MSVC 19.51.36257 |
| Language / build | C++20 / CMake Release |
| Model | Same deterministic dynamics workload across integrators |
| Timestep | 0.001 s |
| Steps / run | 25,000 |
| Runs / configuration | 5 |
| State dimensions | 2, 16, 64, 256 |
| Consumption check | Final-state checksum |

The baseline's throughput is based on **125,000 aggregate steps per state/integrator configuration**. A number like 8.76 million steps/s refers to this small-state numerical workload, not a complete guided-flight simulation.

### 3.2 Optimized numerical/profiling results

The report specifies Windows x64, Microsoft Visual C++, CMake + Ninja, C++20, Release optimizations, Visual Studio CPU Usage Profiler, `std::chrono::steady_clock`, and Release PDB symbols. It does **not** provide an exact CPU model, compiler build, dated raw log, core affinity, power governor, or number of repetitions for each specific table in the supplied excerpt. Do not silently substitute the hardware metadata from the October 14 baseline as verified provenance for this later session.

### 3.3 Real-time execution results

The real-time report specifies an optimized x64 Release build and the workload rates reproduced in that report. It does **not** supply raw per-run timing distributions, a precision-clock calibration trace, or an independent measurement log. The nominal 1,367.23× RTF is a historical point estimate from the documented configuration; it is not a prediction of performance on other systems.

## 4. Build and run on Windows

From the root of a *committed, independently buildable* source snapshot:

```cmd
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --config Release --parallel 8
ctest --test-dir build -C Release --output-on-failure
```

If the generator is multi-configuration (for example, Visual Studio), `--config Release` selects the desired binaries. For single-configuration generators (for example, Ninja), configure a separate build directory with `-DCMAKE_BUILD_TYPE=Release` and use that build directory consistently.

The benchmark executable target in the committed build is `AeroForgeBenchmarks`. Its precise location depends on the generator: inspect `build\Release\`, `build\benchmarks\Release\`, or your generator's target output directory. **Inspect `benchmarks/main.cpp` for supported suite selection and output options before invoking it.** This guide deliberately does not invent executable flags.

Running `ctest` validates the registered correctness/regression tests but does **not** imply benchmark suites were run. Separately launch the Release benchmark executable and save its unmodified output.

## 5. Experimental controls and comparability

For every new measurement, record at minimum:

- Git commit SHA and whether the working tree is clean.
- Build generator, target architecture, build type, exact compiler version and compiler flags.
- OS build, CPU model, active core count, power plan and (if practical) clock behavior.
- Exact executable command and selected suite, input state, timestep, iteration/batch counts and repetitions.
- Warm-up strategy, measured region, clock source, units and sample aggregation formula.
- Whether filesystem I/O, memory allocation, formatting, stdout, sensor noise and workload initialization occur inside the timed region.
- Raw individual batch measurements, elapsed time, iteration counts, checksum or other observable output.
- Relevant CPU profiler capture and whether each profiled result corresponds to a separate, unprofiled timing run.

Run benchmark candidates on the **same host and configuration** when claiming a code optimization. Avoid direct comparisons across workload definitions or changes in machine, compiler, runtime or sample aggregation. Do not publish a cross-report speedup unless an explicitly matched workload establishes comparability.

## 6. Reported measurements, by experiment

### 6.1 Original Euler / RK4 scaling study

| State variables | Euler ns/step | RK4 ns/step | Euler steps/s | RK4 steps/s |
| ---: | ---: | ---: | ---: | ---: |
| 2 | 114.12 | 622.95 | 8,762,398.79 | 1,605,268.36 |
| 16 | 211.29 | 1,031.26 | 4,732,876.45 | 969,688.32 |
| 64 | 512.77 | 2,283.95 | 1,950,178.40 | 437,838.65 |
| 256 | 1,708.08 | 7,065.46 | 585,451.30 | 141,533.62 |

The values are transcribed from the historical report. Division of the reported total steps by the reported rounded elapsed time reproduces them to reporting precision; see the audit for rounding residuals. Workload dimensionality and integrator evaluation count both influence scaling.

### 6.2 Before / after optimizer study — same reported benchmark workloads

| Metric | Before | After | Relative change calculated from displayed means |
| --- | ---: | ---: | ---: |
| 10k-sample CSV export | 317.777 ms | 40.527 ms | 7.841× throughput; 87.247% lower latency |
| Generic RK4 step | 0.916 µs | 0.363 µs | 2.523× throughput; 60.371% lower latency |
| 6DOF RK4 step | 1.740 µs | 1.194 µs | 1.457× throughput; 31.379% lower latency |

The CSV optimization reduced stream-formatting overhead via `std::to_chars` and reusable buffers while preserving the recorded 17-digit double-precision formatting. The RK4 optimization reused intermediate states and reduced temporary allocations; the four-stage RK4 scheme was not changed.

The reported CPU profile contained `operator new` sample/event counts of **709 before** and **313 after** (about **55.85% fewer sampled events**). These are not an instrumented count of all `new` calls; do not describe them as precisely measured heap allocations. The report's post-optimization hotspot was floating-point-to-decimal conversion.

### 6.3 Real-time runner and integrated nominal flight

| Scenario | Documented settings | Reported timing and outcome |
| --- | --- | --- |
| On-schedule pacing | 1 ms timestep; 10 runs aggregated | 10 simulated s / 10 wall s = 1.000×; 10,000 steps; 0 deadline misses |
| Deliberate overload | ~1 ms budget; ~2 ms workload | 1 simulated s / 2.00004 wall s = 0.499989×; 1,000 / 1,000 misses; 0.100015 s reported maximum overrun |
| Fixed-rate pacing | dt = 0.0001 s; 1,000 Hz target; 10 runs aggregated | 1 simulated s / 10 wall s = 0.100×; 10,000 steps; 0 misses |
| Standalone 6DOF RK4 | 10 simulated s; dt = 0.0001 s; 100,000 steps | mean wall 59.830 ms; ~167.14× |
| Full nominal stack | ten 10-s runs; physics 1,000 Hz; IMU/nav 200 Hz; GPS 10 Hz; GNC/telemetry 100 Hz | 100 simulated s / 0.0731406 wall s = **1,367.23×**; 100,000 steps |
| Continuous stability | 300 simulated s; 300,000 physics steps | 300 s / 0.234582 s = **1,278.87×** |

**Caution:** A maximum overrun of `0.100015 s` in the overloaded scenario is preserved exactly as reported; without the underlying sample definitions it must not be reinterpreted as a one-step execution time. An RTF greater than 1 is not evidence of hard real-time scheduling or physical model validity. Deadline-miss counts pertain to the specified tests, not universal guarantees.

### 6.4 Layer-by-layer nominal stack

| Workload | Historical measured RTF |
| --- | ---: |
| 6DOF dynamics | 1,636.28× |
| With sensors | 1,532.62× |
| With navigation | 1,490.46× |
| With GNC | 1,367.35× |
| With actuators | 1,390.84× |
| With telemetry / complete stack | 1,367.23× |

The end-to-end difference from 1,636.28× to 1,367.23× corresponds to **16.443% less measured RTF** in this sequence. The actuator-stage RTF increases slightly relative to the immediately preceding stage; that is measurement variability, *not evidence that adding an actuator accelerates the simulation*.

### 6.5 Long-duration timed windows

For the 300,000-step unpaced run, the report gives **0.7861 µs/step** in the first 1,000-step window and **0.7077 µs/step** in the final window, a **9.97% decrease** in reported window mean. This supports the limited conclusion that the recorded run did not exhibit an increasing step-time trend between those checkpoints; it does not measure memory leaks, longer-term performance, thermal behavior, or scheduling guarantees.

### 6.6 Isolated post-optimization components

The profiling report and the real-time report give **different** isolated microbenchmark results. Examples include:

| Workload | Profiling report | Later real-time report |
| --- | ---: | ---: |
| Generic RK4 step | 0.3634 µs | 0.20600 µs |
| Full 6DOF RK4 step | 1.1937 µs | 0.57223 µs |
| CSV serialization, 10k samples | 40.527 ms | 21.18 ms |
| Combined GNC cycle | 0.2679 µs | 0.14216 µs |

**Do not calculate improvement factors between these columns.** There is insufficient evidence here to establish identical hardware, benchmark harness, compiler flags, input workloads or matched before/after implementation. The later values can be separately described as reported measurements.

## 7. Statistical reporting for future benchmarks

For new runs, report at least **count, raw run-level elapsed times, mean, median, minimum, maximum, and standard deviation** when repeated samples are available; consider p95/p99 only with a sufficiently large, well-defined sample population. No p95 or confidence interval is reconstructable from the archived aggregate reports. Do not fabricate them.

For the CPU benchmark, batch small operations and divide the timed aggregate by the known operation count, retaining unrounded elapsed times. For real-time pacing, record per-step scheduled and actual timestamps, missed deadlines, maximum/percentile lateness, total wall time and simulation time. Run both nominal pacing and an intentionally overloaded negative-control workload.

## 8. Correctness before speed

Retest deterministic replay, time-step boundaries, finite-value validation, quaternion normalization, sensor/nav integration, CSV schema/precision, and actuation whenever optimizing. Numerical convergence order and physically correct simulation outputs are **not** proven by high performance. Compare results with known solutions and invariants, not with timing alone.

For CSV regression checks, cover column order, output precision, escaping/format, NaN/Inf behavior according to actual interface contracts, and roundtrip reading where supported.

## 9. Suggested evidence folder for next benchmark run

Store original evidence outside the repository or in a reviewed checked-in `benchmarks/results/` directory:

```text
benchmarks/results/YYYY-MM-DD_<commit>/
  environment.md            # CPU, OS, compiler, generator, flags
  commands.txt              # exact configure/build/run commands
  raw_output.txt            # stdout/stderr, unedited
  timings.csv               # every repetition/batch if supported
  results.md                # derived summary and formulas
  profiler/                 # optional profiler capture (large files may stay external)
```

A run should be treated as independently reproducible only when its input revision, workload, build setup and raw observations are preserved together.

## 10. Interpretation and publication guidance

Use language such as **“Measured 1,367× faster than real time for a nominal 10-second, 1-kHz-physics benchmark on an optimized x64 Release build”** rather than “AeroForge is always 1,367× real time.”

Likewise, say **“Reduced 10,000-sample CSV export latency from 317.777 ms to 40.527 ms in a matched optimization benchmark”** rather than claiming universal 8× CSV acceleration. No results in these reports establish operational flight certification, cross-platform performance, deterministic real-time scheduling, or a general-purpose physics engine performance rank.


## 11. Current HIL benchmarks and evidence auditing

The latest verified software-loopback workload completed **100,000/100,000 accepted commands unpaced** and **10,000/10,000 commands paced at a 100 Hz simulation schedule**, with zero observed execution deadline misses and zero paced completion deadline misses. The paced run reported **0.1618 ms p99 start lateness**. The physical ESP32 benchmark at 460800 baud completed **1,000 physics steps**, with **641 accepted new controller responses** and **997 steps having valid control**. Its mean response latency was **15.5717 ms**, and the benchmark did **not** meet its 90% new-command acceptance criterion.

The C++ evidence auditor passed the software-unpaced, software-paced, and hardware CSV/JSON pairs. Its pass status means the data files are internally consistent under implemented checks; it does **not** make a failed physical throughput target pass. Details and source file names are provided in the [HIL performance report](performance/HilPerformanceReport.md).

The existing software benchmark emits `hil_samples.csv` and `hil_summary.json`. Avoid inferring filenames from development milestones or renaming only the invocation path without changing the producer.

Do not label a paced 100 Hz simulation as a verified 100 Hz external-controller loop unless fresh hardware command timing independently establishes that rate.
