# AeroForge HIL Wire Protocol

## Scope

This document describes the binary sensor/command protocol exercised by the current PC and ESP32 firmware. It is intended for bench-top simulations, not safety-certified control of physical flight hardware. The host's actual `HilPacketCodec` and the firmware's `HilWireProtocol` implementations remain the authoritative byte-level sources if this document and code diverge.

## Transport framing

- Byte stream: Windows serial port to CH340 UART0 on ESP32.
- Host/firmware application baud rate: must match; 460800 baud was used in the latest demonstrated hardware benchmark.
- UART setting: 8 data bits, no parity, one stop bit, no application hardware flow control.
- Frame encoding: **COBS**; each encoded frame ends with byte `0x00`.
- Multi-byte integer byte order: **little-endian**.
- Floating point: IEEE 754 32-bit little-endian `float` fields.
- Integrity: **CRC-32/ISO-HDLC** over the decoded header and payload, stored as a little-endian 32-bit integer immediately after the payload.
- CRC parameters: initial register `0xFFFFFFFF`, reflected polynomial `0xEDB88320`, final XOR `0xFFFFFFFF`.

The receiver accumulates a frame until the zero delimiter, then decodes COBS, checks the minimum length, version, type, declared payload length, and CRC. Corrupt or oversized frames are discarded; the parser must recover at the next delimiter.

## Decoded packet header

| Offset (bytes) | Size | Field | Meaning |
|---:|---:|---|---|
| 0 | 1 | `version` | Protocol version, currently `1` |
| 1 | 1 | `messageType` | `2` = sensor; `3` = control |
| 2 | 2 | `payloadLength` | Little-endian byte count |
| 4 | 4 | `sequence` | Sequence number for this packet |

The header is eight bytes. The payload begins at byte offset eight. Four CRC bytes follow the payload. COBS encoding and the final zero delimiter are applied **after** building the decoded packet.

## Sensor sample (`messageType = 2`)

Declared payload length: **32 bytes**.

| Payload offset | Size | Field | Unit |
|---:|---:|---|---|
| 0 | 8 | `timestamp` | Simulation sample time in microseconds under the demonstrated firmware/test convention |
| 8 | 4 | `specificForceB[0]` | m/s², body X |
| 12 | 4 | `specificForceB[1]` | m/s², body Y |
| 16 | 4 | `specificForceB[2]` | m/s², body Z |
| 20 | 4 | `angularVelocityB[0]` | rad/s, body X |
| 24 | 4 | `angularVelocityB[1]` | rad/s, body Y |
| 28 | 4 | `angularVelocityB[2]` | rad/s, body Z |

The header `sequence` identifies the sensor sample. The controller must not assume it has been sent the full authoritative vehicle state; the packet includes only the specified simulated sensor channels.

## Control command (`messageType = 3`)

Declared payload length: **17 bytes**.

| Payload offset | Size | Field | Unit |
|---:|---:|---|---|
| 0 | 4 | `referencedSensorSequence` | Identifier of the sensor sample used |
| 4 | 4 | `commandedMomentB[0]` | N·m, body X |
| 8 | 4 | `commandedMomentB[1]` | N·m, body Y |
| 12 | 4 | `commandedMomentB[2]` | N·m, body Z |
| 16 | 1 | `controllerStatus` | Firmware/host-defined status code |

The control packet has an independent header sequence. `referencedSensorSequence` links the command to the originating sensor sample. The current rate-damping firmware uses a gain of `-5` on each measured body angular velocity and clamps commanded moments to ±20 N·m.

## Host-side validity and freshness

Receiving a CRC-valid command is necessary but not sufficient for applying it. The host must enforce the applicable packet sequence policy, referenced sensor sequence, finite-value checks, command bounds, and freshness/watchdog policy. Physical HIL may reuse the newest still-valid command for multiple simulation steps. Missing, expired, or rejected control must safely fall back to zero commanded moment.

## Error handling and startup bytes

The original ESP32 may emit ROM boot messages on UART0 after reset. These bytes are not part of the AeroForge protocol. A host parser must ignore invalid frames and resynchronize on the `0x00` delimiter. Avoid application console output on the binary HIL port; close serial-monitor programs before AeroForge opens it.

## Compatibility discipline

If a field, byte order, message type, CRC rule, unit, or encoding changes, update the host encoder/decoder, firmware decoder/encoder, compatibility tests, and this document together. Do not transmit raw C++ structures directly; memory padding and layout are not a wire format.

`HELLO`, `STATUS`, and `FAULT` were included in earlier architectural proposals but are **not confirmed implemented wire messages** by the hardware communication tests described here. Do not claim a handshake occurred during the tested exchanges.

## Related documents

- [HIL architecture](HIL_ARCHITECTURE.md)
- [ESP32 setup](ESP32Setup.md)
- [HIL performance](../performance/HilPerformanceReport.md)
