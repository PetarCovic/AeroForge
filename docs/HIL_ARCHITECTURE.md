# AeroForge Hardware-in-the-Loop Architecture

## Status and evidence

Software loopback and physical ESP32 HIL have been implemented and exercised. The current physical configuration is an ESP32-D0WD-V3 connected to a Windows host through a CH340 USB-to-UART adapter on a locally assigned COM port. Automated benchmark evidence exists; GUI disconnect and reconnect behavior must be signed off separately and is not asserted as verified by this document.

This is a bench-top simulation interface. It is not flight-qualified control software and does not directly actuate a physical flight vehicle.

## Responsibilities

**AeroForge host:** maintains authoritative simulation time, advances the 13-component rigid-body state, synthesizes sensor measurements, communicates with the controller, checks sequence and freshness, bounds applied body moments, records telemetry, and optionally visualizes the run.

**External controller:** validates sensor packets, calculates a three-axis moment request, and returns a command referencing the sensor sequence. The ESP32 firmware currently implements angular-rate damping with bounded body moments; it does not control motors or propulsion hardware.

```text
             AeroForge host (Windows)
  +------------------------------------------+
  | 6DOF state -> sensor sample -> serializer |
  |       |                         |         |
  |  dynamics <- command latch <- validator  |
  |       |                         ^         |
  |  precise pacer                 serial I/O  |
  +------------------------------------------+
                                | USB / CH340
                                v
                 ESP32 UART0 / COBS / CRC32
                         rate controller
                                |
                           command packet
                                |
                    back to AeroForge host
```

## Execution models

- **Software loopback:** host-resident controller and transport; suitable for deterministic and fast benchmark runs.
- **Physical HIL:** serial worker and Windows serial transport exchange binary packets with the ESP32 firmware.
- **External-process software controller:** remains an architectural extension; a working independent-process deployment has not been established by the provided verification evidence.

The physical implementation is **asynchronous**. It does not wait for the ESP32 to answer each sensor sample before advancing a physics step. Instead, it validates incoming commands and may reuse the latest still-fresh command. A timed-out, rejected, or unavailable command must cause zero commanded body moment as the safe fallback. Distinguish *steps with valid control* from *new controller responses* in all reports.

## Simulation time and deadlines

The tested fixed step is `dt = 0.01 s` (100 Hz simulation updates). Simulation time comes from the simulation step count; host scheduling and USB communication are observed using a monotonic host clock. ESP32-local clock values are a third time domain and are not assumed synchronized to either host clock.

The host uses the existing precise pacer to target step deadlines. Zero measured deadline misses on a particular run do **not** prove hard real-time guarantees on Windows.

## Host implementation map

| Responsibility | Existing implementation |
|---|---|
| Framing and integrity | `HilCobs`, `HilCrc32`, `HilStreamParser`, `HilPacketCodec` |
| Packet exchange | `HilWireEndpoint` |
| Command validation | `HilSequenceTracker`, `HilSimulationBridge`, `HilCommandWatchdog` |
| Software controller | `HilSoftwareController`, `LoopbackTransport` |
| Physical serial I/O | `WindowsSerialTransport`, `HilSerialWorker` |
| Closed-loop execution | `HilClosedLoopStep`, `HilMonitoredRun`, `HilHardwareRun` |
| Scheduling and metrics | `HilPrecisePacer`, `HilTimingMonitor`, `HilTelemetryStore` |
| GUI inspection | `HilDashboardPanel`, `VisualizationApp` |

These names describe source boundaries, not a guarantee that every optional GUI action was manually validated.

## Wire protocol

The implemented binary frames use little-endian payload fields, COBS framing with a `0x00` delimiter, and CRC-32/ISO-HDLC for integrity. The demonstrated message types are **sensor sample** (`2`) and **control command** (`3`). See [HilWireProtocol.md](HilWireProtocol.md) for the byte layout and parser requirements.

Earlier design proposals listed `HELLO`, `STATUS`, and `FAULT` messages. These must **not** be advertised as implemented message types without corresponding host and firmware support.

## Firmware and serial connection

The currently connected device is an original ESP32 (`ESP32-D0WD-V3`), not an ESP32-S3. Its CH340 adapter exposes a Windows COM port; the observed system used `COM3`. The firmware uses UART0 with GPIO1 TX and GPIO3 RX, and 8-N-1 framing. Baud rate must match between firmware and host. The demonstrated higher-rate application configuration is `460800` baud.

The existing source directory was named `firmware/esp32s3` before the hardware identity was confirmed. The **ESP-IDF target must be `esp32`** regardless of that folder name. See [ESP32Setup.md](ESP32Setup.md).

## Validation and known performance

The current 460800-baud hardware benchmark completed 1,000 simulation steps with 641 accepted controller responses, 997 steps with valid control, 3 zero-command fallback steps, and zero measured execution/completion deadline misses. Mean host-observed response latency was 15.5717 ms; the physical command stream was **not** validated at a fresh 100 Hz response rate.

The corresponding CSV/JSON evidence passed the verification auditor with a warning that the new-command acceptance ratio was below the optional 90% threshold. Software HIL achieved 100,000/100,000 accepted commands unpaced and 10,000/10,000 accepted commands paced. See [HilPerformanceReport.md](../performance/HilPerformanceReport.md) and [CurrentVerificationResults.md](../verification/CurrentVerificationResults.md).

## Safety and limitations

- The authoritative 6DOF state remains on the host.
- Sensor and command packets must be checked for integrity, validity, sequence, and bounded values.
- The host must not apply stale, non-finite, or out-of-range controller commands.
- A missing or expired command must not leave a nonzero command latched indefinitely.
- USB disconnect, COM-port contention, restart, and GUI shutdown require explicit manual verification before safety claims are published.
- Hardware acceptance below one new command per simulated step is a documented limitation, not evidence that the physics loop missed its timing deadlines.
- Windows plus CH340 USB serial is not a hard real-time or safety-certified control channel.
