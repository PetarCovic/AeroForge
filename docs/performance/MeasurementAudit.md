# AeroForge Performance Measurement Audit

> **Audit type:** Source-document arithmetic verification, performed October 25, 2026. All input figures are transcribed from the three historical AeroForge Markdown reports in this package. **No raw benchmark stdout, iteration-level CSV, profiler export, or compiled source revision was provided with the uploaded documentation archive.** The audit verifies that derived ratios agree with *reported* source figures; it does not independently attest to measurement collection, hardware identity, program execution, physical model correctness or repeatability.

## Primary numerical evidence and boundaries

| ID | Original source | Reported configuration | Evidence available |
| --- | --- | --- | --- |
| B1 | [`EarlyIntegrationBaseline.md`](../benchmarks/EarlyIntegrationBaseline.md) | Windows 11 Home, AMD Ryzen 9 5900X, MSVC 19.51.36257, Release, 25,000 steps × 5 runs, dt=0.001 s, state sizes 2/16/64/256, October 14, 2026 | Tabulated total times, rates, ns/step and checksums; raw run outputs absent |
| P1 | [`PerformanceReport.md`](PerformanceReport.md) | Windows x64 C++20 Release, MSVC, CMake+Ninja, Visual Studio CPU Usage profiler and benchmark executable | Before/after means, profiler sample counts, post-optimization microbenchmarks; exact source revision, raw logs and session date absent |
| R1 | [`RealtimeExecutionPerformanceReport.md`](RealtimeExecutionPerformanceReport.md) | Optimized x64 Release; stated nominal rates and pacing/overload configurations | Aggregated timing, step counts, deadline misses, RTF, stage series; individual run distributions absent |

## Checked equations

Given a timed workload of *N* completed operations in duration *t* seconds:

- Throughput (ops/s) = `N / t`.
- Mean seconds/operation = `t / N`.
- Real-time factor = `simulated_duration_seconds / wall_duration_seconds`.
- Before/after speed factor = `before_mean_latency / after_mean_latency`, **only for a matched workload**.
- Percent latency decrease = `100 × (1 − after / before)`.
- Percent RTF decrease = `100 × (1 − final_RTF / initial_RTF)`.

### A. Original integration baseline (B1)

Each row represents `25,000 × 5 = 125,000` completed steps. The reported table uses rounded elapsed seconds; recalculation from those rounded times may differ slightly in trailing decimal places from original tabulated rates.

| Integrator | State size | Reported total seconds | Reported ns/step | Recomputed ns/step from rounded total | Reported steps/s | Recomputed steps/s from rounded total |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Euler | 2 | 0.014266 | 114.12 | 114.13 | 8,762,398.79 | 8,762,091.69 |
| RK4 | 2 | 0.077869 | 622.95 | 622.95 | 1,605,268.36 | 1,605,260.12 |
| Euler | 16 | 0.026411 | 211.29 | 211.29 | 4,732,876.45 | 4,732,876.45 |
| RK4 | 16 | 0.128907 | 1,031.26 | 1,031.26 | 969,688.32 | 969,691.33 |
| Euler | 64 | 0.064097 | 512.77 | 512.78 | 1,950,178.40 | 1,950,169.27 |
| RK4 | 64 | 0.285493 | 2,283.95 | 2,283.94 | 437,838.65 | 437,839.11 |
| Euler | 256 | 0.213510 | 1,708.08 | 1,708.08 | 585,451.30 | 585,452.67 |
| RK4 | 256 | 0.883182 | 7,065.46 | 7,065.46 | 141,533.62 | 141,533.68 |

All baseline rows are consistent within precision induced by rounding the recorded elapsed time. The 2-variable Euler row shows the largest difference (about 0.0035% in throughput), which cannot be resolved without the unrounded primary timer output; retain the original published table and avoid claiming exact equality at the last digit.

### B. Recorded matched optimization results (P1)

| Workload | Before | After | Calculated speed factor | Calculated latency decrease | Rounded wording permitted |
| --- | ---: | ---: | ---: | ---: | --- |
| CSV, 10,000 samples/export | 317.777 ms | 40.527 ms | 7.841× | 87.247% | 7.84× / 87.2% |
| Generic RK4 step | 0.916 µs | 0.363 µs | 2.523× | 60.371% | 2.52× / ~60.4% |
| Complete 6DOF RK4 step | 1.740 µs | 1.194 µs | 1.457× | 31.379% | 1.46× / 31.4% |

Note that the optimized results table elsewhere in P1 has additional display precision (`0.3634 µs`, `1.1937 µs`); **the before/after comparisons use the report's explicitly matched rounded before/after means**, rather than mixing one table's precision with another.

The reported profiler `operator new` samples/events were 709 and 313: `(709 − 313) / 709 × 100 = 55.85%`. This supports approximately 56% fewer **sampled events**, not a measured 56% reduction in all heap allocations.

### C. Real-time factor calculations (R1)

| Workload | Simulated seconds | Wall seconds | Recomputed RTF | Reported RTF |
| --- | ---: | ---: | ---: | ---: |
| Nominal complete stack, ten 10-second runs | 100 | 0.0731406 | 1,367.22969× | 1,367.23× |
| Continuous nominal 300-second simulation | 300 | 0.234582 | 1,278.87050× | 1,278.87× |
| Standalone 6DOF RK4, 100,000 steps | 10 | 0.059830 | 167.14023× | ~167× |
| Deliberate overload | 1 | 2.00004 | 0.499990× | 0.499989× |
| On-schedule aggregate | 10 | 10.0 | 1.000× | 1.000× |
| Fixed-rate aggregate | 1 | 10.0 | 0.100× | 0.100× |

**Tiny overload RTF mismatch:** `1 / 2.00004 = 0.4999900002`, whereas the report lists `0.499989`. The difference is around **0.000001×**, plausibly caused by rounded displayed wall time vs higher-precision values. Keep the original result explicitly labeled *as reported*; do not assert a uniquely reconstructed exact RTF from the rounded wall-time entry.

Additional arithmetic:

- Full-stack relative RTF reduction: `100 × (1 − 1367.23 / 1636.28) = 16.4428%` → **~16.4%**.
- First vs last 1,000-step window mean: `100 × (0.7077 / 0.7861 − 1) = −9.9733%` → **−9.97%**.
- Deliberate overload: `1,000 / 1,000` reported missed deadlines → **100% of deliberately overloaded steps** in that trial configuration.

### D. Noncomparable historical microbenchmarks

| Metric | P1 profiling table | R1 later runtime study | Appropriate interpretation |
| --- | ---: | ---: | --- |
| Generic RK4 mean | 0.3634 µs | 0.20600 µs | Two reported measurement sets; no proven matched A/B study |
| Full 6DOF RK4 mean | 1.1937 µs | 0.57223 µs | Two reported measurement sets; no proven matched A/B study |
| CSV, 10,000 samples | 40.527 ms | 21.18 ms | Two reported measurement sets; no proven matched A/B study |
| GNC cycle mean | 0.2679 µs | 0.14216 µs | Two reported measurement sets; no proven matched A/B study |

**Do not present the later figures as additional measured speedups over P1** without the exact same workload, machine, binary and measurement harness. Even if the code is faster in reality, these documents do not establish the causal effect.

## Unverifiable from the uploaded evidence

- Exact original per-run timing, warm-up count, variability, p95/p99, confidence intervals or independent repeats.
- Whether all historical sessions ran on the same CPU; CPU is explicitly given for B1 but not P1/R1 in the supplied text.
- Current benchmark-executable CLI or source implementation details, beyond filenames and claims recorded in the supplied documentation and Git history.
- Current number of tests; the **1,180** count in R1 was recorded at benchmark time.
- The meaning of `maximum overrun` beyond the report's own metric and recorded value.
- Whether reported checksums correspond to expected analytical solutions; the original baseline correctly notes they are anti-optimization/sanity checks, not accuracy measurements.

## Publication rules

1. Cite the specific source and workload beside every performance number.
2. Preserve units, batch size, dates where documented and Release/Debug distinction.
3. Label reported measurement separately from a computed ratio.
4. Do not conflate simulation correctness, physical validation, real-time pacing and throughput.
5. When raw runs are collected in a future session, store the logs and regenerate the report from those raw records.
