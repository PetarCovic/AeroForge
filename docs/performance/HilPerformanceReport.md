# AeroForge Hardware-in-the-Loop Performance Report

## Scope and evidence

This report separates **software-loopback benchmark results** from **physical ESP32 UART measurements**. The data below were reported from AeroForge runs during October 2026. A standalone C++ verification auditor subsequently checked the three current CSV/JSON evidence pairs for internal consistency. The report does not claim that all manual GUI/disconnect checks have passed.

The 100 Hz figure refers to the **simulation physics schedule**, not a guaranteed fresh 100 Hz command stream from the ESP32. A valid prior command may be reused within its freshness window.

## Test configurations

| Environment | Controller | Transport | Cadence |
|---|---|---|---|
| Software HIL, unpaced | Host software rate controller | In-process loopback | No deliberate wall-clock pacing |
| Software HIL, paced | Host software rate controller | In-process loopback | 100 Hz, `dt = 0.01 s` |
| Physical HIL | ESP32-D0WD-V3 | CH340 USB UART on Windows COM3 | 100 Hz simulation; async controller responses |

Host benchmarks were built in a Windows x64 C++20 Release configuration. Physical serial was measured at both 115200 and 460800 application UART baud. These are workload-specific measurements, not hardware-independent guarantees.

## Current software-loopback results

| Metric | Unpaced | Paced (100 Hz) |
|---|---:|---:|
| Simulation steps | 100,000 | 10,000 |
| Accepted commands | 100,000 | 10,000 |
| Mean step computation | 0.00497655 ms | 0.0134706 ms |
| Maximum step computation | 0.0691 ms | 0.1596 ms |
| Mean command response | 0.0018699 ms | 0.00347054 ms |
| Execution deadline misses | 0 | 0 |
| Completion deadline misses | Not applicable | 0 |
| Mean start lateness | Not measured | 0.0200162 ms |
| p95 start lateness | Not measured | 0.1061 ms |
| p99 start lateness | Not measured | 0.1618 ms |
| Maximum start lateness | Not measured | 0.4821 ms |

The unpaced run measures work cost without intentional scheduling delays. The paced run completed 10,000 steps representing 100 seconds of simulated time, with no *observed* execution or completion deadline misses.

Evidence locations from the recorded commands:

```text
out/verification/software-unpaced/hil_samples.csv
out/verification/software-unpaced/hil_summary.json
out/verification/software-paced/hil_samples.csv
out/verification/software-paced/hil_summary.json
out/verification/SoftwareAudit.md
out/verification/SoftwarePacedAudit.md
```

The auditor passed both software datasets. It issued an informational warning that the CSV did not contain `deadline_misses`; execution misses were read from the JSON summaries instead. Consequently, per-row deadline miss reconciliation was unavailable.

## Physical ESP32 measurements

### Standalone serial smoke test

Ten consecutive sensor-to-command exchanges completed with correct returned moments and matching sensor references. The test reported **10/10 passed**, **15.637 ms mean**, and **19.134 ms maximum** round-trip time at 115200 baud. This was a sequential request/response communication test, not a sustained 100 Hz benchmark.

### Full physical closed-loop benchmark

| Metric | 100 steps, 115200 | 1,000 steps, 115200 | 1,000 steps, 460800 |
|---|---:|---:|---:|
| Simulation steps completed | 100 | 1,000 | 1,000 |
| Accepted controller responses | 63 | 639 | 641 |
| Steps with valid control | 96 | 998 | 997 |
| Zero-command fallback steps | 4 | 2 | 3 |
| Mean response latency | 22.5397 ms | 22.0685 ms | 15.5717 ms |
| Maximum response latency | 40.0017 ms | 40.1087 ms | 20.1281 ms |
| Mean computation time | 0.013307 ms | 0.011031 ms | 0.0108036 ms |
| Execution deadline misses | 0 | 0 | 0 |
| Completion deadline misses | 0 | 0 | 0 |
| RX dropped bytes | 0 | 0 | 0 |
| Last serial error | 0 | 0 | 0 |

For the 460800-baud run, mean/p95/p99/maximum start lateness was **0.0244842 / 0.1083 / 0.195 / 0.4697 ms**.

Raising application UART speed from 115200 to 460800 lowered the reported mean response latency from 22.0685 to 15.5717 ms (approximately 29.4%) for the 1,000-step runs. Accepted responses changed only from 639 to 641. The result suggests that UART bit rate alone is not sufficient to produce a fresh controller response on every 10 ms physics step. It does not isolate Windows scheduling, driver buffering, USB buffering, firmware waits, or host command rejection.

### Command availability versus response rate

The 460800-baud run reported **641 accepted responses for 1,000 physics steps** (64.1 responses per 100 simulated steps), but **997 steps with valid control** (99.7%) because fresh-enough prior commands could be retained. The three remaining steps used zero-command fallback.

These are **not** interchangeable percentages. The hardware benchmark printed `HARDWARE ACCEPTANCE THRESHOLD NOT MET` under its 90% newly accepted response criterion. That result must remain visible in any public performance claim.

### Physical evidence audit

The existing C++ auditor reported:

```text
HIL evidence audit PASSED (1000 steps, 641 commands, 1 warning(s))
WARNING: Controller response rate below 90% of simulation step rate; this is a performance observation, not a corrupt evidence file
```

A passing evidence audit means that the saved CSV and JSON were internally consistent under the auditor's checks; it **does not** reverse the benchmark's performance threshold failure.

```text
out/hil-benchmarks/hardware-460800/hil_hardware_samples.csv
out/hil-benchmarks/hardware-460800/hil_hardware_summary.json
out/verification/HardwareAudit.md
```

## Historical software-loopback baseline

An earlier report recorded three software HIL runs: 10,000 unpaced steps, 1,000 paced steps, and 10,000 extended paced steps. All **21,000 commands** were accepted, and all three runs reported **zero execution deadline misses**.

| Historical metric | Unpaced | Short paced | Extended paced |
|---|---:|---:|---:|
| Steps | 10,000 | 1,000 | 10,000 |
| Mean step time (ms) | 0.00439981 | 0.0150915 | 0.0140376 |
| Maximum step time (ms) | 0.0639 | 0.0509 | 0.2452 |
| Mean response time (ms) | 0.00184549 | 0.0042372 | 0.00409161 |
| Accepted commands | 10,000 | 1,000 | 10,000 |

Those measurements are retained as historical evidence; they were not collected under the exact conditions of the newer software results and should not be used as a matched optimization comparison. The earlier report did not include explicit start-lateness measurements. The later precise-pacing benchmark did.

## Reproduction

From the repository root:

```powershell
.\out\build\x64-release-clean\AeroForgeHilBenchmarks.exe 100000 out\verification\software-unpaced
.\out\build\x64-release-clean\AeroForgeHilBenchmarks.exe 10000 out\verification\software-paced paced
.\out\build\x64-release-clean\AeroForgeHilHardwareBenchmarks.exe COM3 1000 out\hil-benchmarks\hardware-460800 460800
```

Use the benchmark's printed filenames when running `AeroForgeVerificationAudit`; the current software writer uses `hil_samples.csv` and `hil_summary.json`. Physical runs require the actual ESP32 and host UART rate to match. Capture machine metadata, source revision, and exact commands for future comparisons.

## Limitations and next investigation

- Windows USB serial does not offer a hard real-time bound.
- The physical benchmark's 100 Hz *physics* pace is not proof of 100 Hz fresh controller responses.
- New command acceptance and steps with a valid held command are distinct.
- The hardware audit warning and original benchmark non-acceptance remain part of the evidence.
- Test output did not itself establish manual GUI USB disconnect/reconnect safety.
- Longer endurance, read-buffer tuning, protocol diagnostics, and more UART/control rates are optional follow-up investigations, to be triaged after the current project release rather than silently represented as completed.

For current system-wide evidence, see [CurrentVerificationResults.md](../verification/CurrentVerificationResults.md). For historical CPU and numerical performance studies, see [BENCHMARKING_AND_PERFORMANCE.md](../BENCHMARKING_AND_PERFORMANCE.md).
