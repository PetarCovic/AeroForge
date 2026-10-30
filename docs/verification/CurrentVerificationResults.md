# AeroForge Current Verification Results

## Scope and reporting rules

This is a record of the test and benchmark results reported during the October 2026 AeroForge development session. It distinguishes automated correctness tests, workload measurements, evidence auditing, and manual validation. An audit passing means the saved evidence is internally consistent under the auditor's implemented checks; it is not equivalent to all performance acceptance criteria passing.

Do not interpret the numbers below as proof of flight qualification, comprehensive model validation against physical flight data, or hard real-time guarantees.

## Automated regression suite

**October 30, 2026 — v0.1 Release Verification:** A clean Windows x64 Release configuration at `out/build/x64-release-v01` completed successfully. CTest reported **1,557 registered tests**, and the developer confirmed that **all 1,557 passed**. The AeroForge visualizer launched and worked during a local manual check.

The suite covers math, numerical integration, 6DOF rigid-body physics, atmosphere/aerodynamics, propulsion, sensors, navigation, GNC, fault injection, reproducibility, telemetry, and visualization. The count supersedes the earlier historical report's 1,180-test count for this local release configuration.

These results are developer-reported. The full CTest output/log, repeat-until-fail results, and an independent headless build have not been provided in this verification record; those checks remain separately unconfirmed. A passing host regression suite does not establish flight-model fidelity or physical-HIL safety.

To reproduce the current release configuration:

```powershell
cmake -S . -B out/build/x64-release-v01 -G Ninja -DCMAKE_BUILD_TYPE=Release -DAEROFORGE_BUILD_VISUALIZATION=ON -DBUILD_TESTING=ON
cmake --build out/build/x64-release-v01 --parallel 8
ctest --test-dir out/build/x64-release-v01 --output-on-failure
```

Optional additional stability check (not counted as passed without a recorded result):

```powershell
ctest --test-dir out/build/x64-release-v01 --repeat until-fail:3 --output-on-failure
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

The final reported 1,000-step physical benchmark used an ESP32-D0WD-V3 on a CH340 serial adapter, COM3, at **460800 application UART baud**, with 1,000 simulated steps paced at 100 Hz.

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

**Additional known limitation:** Subsequent USB reconnection experiments did not demonstrate reliable automatic return to normal physical HIL operation without manually resetting the ESP32. Those diagnostic runs are distinct from the 1,000-step benchmark above. Automated recovery and acceptance remain deferred and are not release claims.

## Verification status summary

| Area | Status | Evidence |
|---|---|---|
| Current CTest regression suite | **Passed: 1,557/1,557** | Clean Windows x64 Release build and developer-reported CTest result, October 30, 2026 |
| Visualizer basic launch/operation | Passed local manual check | Developer-reported GUI check, October 30, 2026 |
| Repeated CTest / separate headless build | Not confirmed | No completion record supplied |
| Software HIL unpaced | Passed | 100,000-step benchmark and evidence audit |
| Software HIL paced | Passed under measured conditions | 10,000-step benchmark, zero deadline misses, evidence audit |
| Physical serial packet exchange | Passed | 10/10 initial sensor/control exchanges |
| Physical 100 Hz physics pacing | Passed under measured conditions | 1,000 steps, zero deadline misses |
| Physical 90% fresh-command acceptance | **Not met** | 641/1,000 responses |
| Hardware data consistency audit | Passed with warning | HardwareAudit.md |
| Physical GUI start/stop and COM-port release | Manual sign-off pending | No recorded results yet |
| USB disconnect and applied zero-moment fallback | Manual sign-off pending | Not established solely by communication loss or GUI status |
| Automatic reconnect/recovery | **Not demonstrated reliably** | Subsequent USB reconnection experiments; manual reset needed |
| Extended hardware endurance | Not established | Not included in current evidence |

## Manual verification record

The latest manual GUI check confirmed launch and basic operation, but it is not a substitute for physical serial fault-injection sign-off. Record additional observations below; do not mark items complete by assumption.

| Check | Result | Evidence / notes |
|---|---|---|
| Visualizer launches and operates | Passed (local check) | Developer-reported, October 30, 2026 |
| Recorded playback and physics-driven live view | Not separately signed off | Demo recordings exist; individual manual validation results not logged here |
| Software HIL monitor and source switch | Not separately signed off | Demo recording exists; detailed manual test log not supplied |
| Hardware GUI opens correct COM port and baud | Pending | |
| Stopping session releases serial port | Pending | |
| Busy COM port fails without crash | Pending | |
| USB unplug causes timeout and applied zero-moment fallback | Pending | |
| Reconnect establishes fresh valid responses without manual reset | Not confirmed | Observed USB reconnection difficulties |

Use no attached physical actuators during these checks. The GUI's source label alone is insufficient proof that the applied body moment fell to zero; inspect the command/fallback values or corresponding telemetry.

## Reproduction and publication

See [INSTALL_AND_VALIDATION.md](../INSTALL_AND_VALIDATION.md) for commands. Archive the exact source revision, compiler/build configuration, OS and host information, raw console output, and benchmark-generated data when preparing public claims. Historical numerical and visualization studies have different workloads and are not directly comparable to the HIL results.
