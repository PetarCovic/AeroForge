# AeroForge Visualizer User Guide

## Build and start

From the repository root in an x64 MSVC environment:

```powershell
cmake --build out/build/x64-release-clean --target AeroForgeVisualizer --parallel 8
.\out\build\x64-release-clean\AeroForgeVisualizer.exe
```

For clean configuration and test instructions, see [INSTALL_AND_VALIDATION.md](../INSTALL_AND_VALIDATION.md).

## Recorded telemetry

1. Select **File > Open telemetry CSV...**.
2. Open a compatible CSV recording (for example `examples/output/rocket_6dof_flight.csv` after generating the example).
3. Use **Playback** to play, pause, seek, restart, and adjust playback speed.
4. Inspect **Telemetry**, **Trajectory**, and **Vehicle 3D**.

Recorded telemetry may be physically simulated data or an external dataset. Missing position or attitude must never be silently described as measured values.

## Live 6DOF simulation

1. Select **File > Start 6DOF live simulation**.
2. Observe the **Dashboard**, **Telemetry**, **Trajectory**, **Vehicle 3D**, and **Performance** panels.
3. Confirm simulation time advances and the displayed quaternion orientation changes as expected.
4. Select **File > Stop live source** to stop the simulation worker.

This live demonstration uses simplified forces and moments. It is not the complete reference guided-rocket mission.

## Software HIL

1. Select **File > Start HIL software loopback**.
2. Open **View > HIL Monitor**.
3. Confirm the controller source is software loopback and command/step counters update.
4. Stop the live source before opening another producer.

Software loopback keeps the controller inside the host process and does not measure USB or ESP32 latency.

## Physical ESP32 HIL

1. Connect the ESP32-D0WD-V3 through the onboard CH340 USB adapter and identify the Windows COM port.
2. Ensure firmware and AeroForge agree on baud rate. The latest demonstrated physical benchmark used **460800 baud** and **COM3**; ports vary across machines.
3. Close `idf.py monitor` or any serial terminal that occupies the port.
4. In a build whose hardware GUI action is configured for the correct port and baud, select **File > Start HIL hardware (COM3)**.
5. Open **View > HIL Monitor** and inspect response counters, command availability, and fallback indicators.
6. Use **File > Stop live source** when finished.

**Configuration warning:** Some GUI builds may still use a 115200-baud default even though the ESP32 was reflashed for 460800 baud. Verify the GUI's actual port/baud configuration before considering the physical GUI test valid. The standalone hardware benchmark's success does not prove that the GUI's selected baud rate is correct.

## Safe shutdown and recovery

With no external actuators attached, the manual verification procedure should cover stopping a running session, attempting to open an occupied COM port, disconnecting the ESP32, verifying zero-moment fallback after command expiry, reconnecting, and starting a fresh session. Do not claim that these tests passed until observed and recorded.

The GUI should never leave a stale nonzero controller command active indefinitely. If unexpected behavior occurs, stop the session and inspect the HIL monitor and benchmark diagnostics before resuming.

## Data interpretation

- **Synthetic live:** procedural demonstration data; not a physics solution.
- **Physics live:** calculated 6DOF dynamics; not automatically an estimated navigation solution.
- **Software HIL:** simulated dynamics controlled by host-side controller logic.
- **Hardware HIL:** simulated dynamics driven by actual controller responses, potentially reusing the latest fresh response across steps.
- **Recorded CSV:** previously captured data, subject to its documented schema and missing channels.

The GUI's bounded live history is not a complete flight recording. Use CSV output for exhaustive offline analysis.

## Related documentation

- [Visualization architecture](VISUALIZATION_ARCHITECTURE.md)
- [HIL architecture](../hil/HIL_ARCHITECTURE.md)
- [ESP32 setup](../hil/ESP32Setup.md)
- [Visualization performance](../performance/VisualizationPerformanceReport.md)
