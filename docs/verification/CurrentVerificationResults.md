# AeroForge Current Verification Results

## Scope and reporting rules

This is a record of the test and benchmark results reported during the October 2026 AeroForge development session. It distinguishes automated correctness tests, workload measurements, evidence auditing, and manual validation. An audit passing means the saved evidence is internally consistent under the auditor's implemented checks; it is not equivalent to all performance acceptance criteria passing.

Do not interpret the numbers below as proof of flight qualification, comprehensive model validation against physical flight data, or hard real-time guarantees.

## Automated regression suite

The full AeroForge automated test suite was reported **passing** after HIL hardware integration. The source tree includes established tests for math, numerical integration, 6DOF rigid-body physics, atmosphere/aerodynamics, propulsion, sensors, navigation, GNC, fault injection, reproducibility, telemetry, and visualization.

An exact current test count and full CTest XML/log were not supplied with these benchmark outputs. The earlier historical report's 1,180-test count is not treated as the current count. Repeat-until-fail runs and an optional headless build remain separately unconfirmed.

To reproduce:

```powershell
ctest --test-dir out/build/x64-release-clean --output-on-failure
ctest --test-dir out/build/x64-release-clean --repeat until-fail:3 --output-on-failure
```

## Software HIL verification

| Metric | Unpaced | Paced, 100 Hz |
|---|---:|---:|
| Steps | 100,000 | 10,000 |
| Accepted commands | 100,000 | 10,000 |
| Mean step execution | 0.00497655 ms | 0.0134706 ms |
| Maximum step execution | 0.0691 ms | 0.1596 ms |
| Mean controller response | 0.0018699 ms | 0.00347054 ms |
| Execution deadline misses | 0 | 0 |
| Completion deadline misses | Not measured | 0 |
| p99 start lateness | Not measured | 0.1618 ms |

The software evidence auditor reported **PASSED** for both CSV/JSON pairs. Both audits issued one informational warning: the software CSV lacks `deadline_misses`, so the execution-miss count was read from JSON rather than checked against per-step CSV values.

Saved evidence paths:

```text
out/verification/software-unpaced/hil_samples.csv
out/verification/software-unpaced/hil_summary.json
out/verification/software-paced/hil_samples.csv
out/verification/software-paced/hil_summary.json
out/verification/SoftwareAudit.md
out/verification/SoftwarePacedAudit.md
```

## Physical ESP32 HIL verification

The final reported physical run used an ESP32-D0WD-V3 on a CH340 serial adapter, COM3, at **460800 application UART baud**, with 1,000 simulated steps paced at 100 Hz.

| Metric | Observed value |
|---|---:|
| Simulated physics steps | 1,000/1,000 |
| Newly accepted controller responses | 641 |
| Steps with valid control | 997 |
| Zero-moment fallback steps | 3 |
| Mean response latency | 15.5717 ms |
| Maximum response latency | 20.1281 ms |
| Mean computation duration | 0.0108036 ms |
| p99 start lateness | 0.195 ms |
| Execution / completion deadline misses | 0 / 0 |
| RX dropped bytes / last serial error | 0 / 0 |

The benchmark reported **HARDWARE ACCEPTANCE THRESHOLD NOT MET**. The new-response count is 64.1% of the physics-step count, below the tested 90% threshold. The physics loop retained a valid command on 99.7% of its steps by reusing the latest still-fresh response. Do not describe this as a freshly updated 100 Hz hardware controller.

The C++ evidence auditor subsequently reported **PASSED** for the physical CSV/JSON pair with one warning that the controller response rate fell below 90%. This establishes internal evidence consistency, not successful completion of the separate hardware throughput target.

Saved evidence paths:

```text
out/hil-benchmarks/hardware-460800/hil_hardware_samples.csv
out/hil-benchmarks/hardware-460800/hil_hardware_summary.json
out/verification/HardwareAudit.md
```

Earlier physical baselines at 115200 baud are preserved in the [HIL performance report](../performance/HilPerformanceReport.md).

## Verification status summary

| Area | Status | Evidence |
|---|---|---|
| Full current GoogleTest run | Reported passed | Developer's local regression run |
| Software HIL unpaced | Passed | 100,000-step benchmark and evidence audit |
| Software HIL paced | Passed under measured conditions | 10,000-step benchmark, zero deadline misses, evidence audit |
| Physical serial packet exchange | Passed | 10/10 initial sensor/control exchanges |
| Physical 100 Hz physics pacing | Passed under measured conditions | 1,000 steps, zero deadline misses |
| Physical 90% fresh-command acceptance | **Not met** | 641/1,000 responses |
| Hardware data consistency audit | Passed with warning | HardwareAudit.md |
| GUI start/stop and COM-port release | Manual sign-off pending | No recorded results yet |
| USB disconnect and zero-moment fallback | Manual sign-off pending | No recorded results yet |
| Reconnect and recovery | Manual sign-off pending | No recorded results yet |
| Extended hardware endurance | Not established | Not included in current evidence |

## Manual verification record

To close the remaining manual verification work, record actual observations below; do not mark items complete by assumption.

| Check | Result | Evidence / notes |
|---|---|---|
| Recorded playback and physics-driven live view | Pending | |
| Software HIL monitor and source switch | Pending | |
| Hardware GUI opens correct COM port and baud | Pending | |
| Stopping session releases serial port | Pending | |
| Busy COM port fails without crash | Pending | |
| USB unplug causes timeout and zero-moment fallback | Pending | |
| Reconnect establishes fresh valid responses | Pending | |

Use no attached physical actuators during these checks. The GUI's source label alone is insufficient proof that the applied body moment fell to zero; inspect the command/fallback values or corresponding telemetry.

## Reproduction and publication

See [INSTALL_AND_VALIDATION.md](../INSTALL_AND_VALIDATION.md) for commands. Archive the exact source revision, compiler/build configuration, OS and host information, raw console output, and benchmark-generated data when preparing public claims. Historical numerical and visualization studies have different workloads and are not directly comparable to the HIL results.
