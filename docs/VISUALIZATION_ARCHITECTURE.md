# AeroForge Visualization Architecture

## Purpose and status

AeroForge supports recorded telemetry playback, synthetic live demonstration data, physics-driven six-degree-of-freedom (6DOF) simulation, and HIL monitoring. The application uses GLFW/OpenGL, Dear ImGui, and ImPlot. Its live display is observational: rendering must not control simulation time or modify the authoritative vehicle state.

## Data flow

```text
 Recorded CSV -----------+
                         |
 Synthetic producer -----+--> VisualizationSession --> panels / plots / 3D
                         |
 6DOF simulation --------+
       |                 |
       +--> LiveTelemetryPublisher
                         |
 Hardware / software HIL +--> HIL telemetry / monitoring
```

The producer and renderer run on separate execution paths. The GUI thread owns Dear ImGui, GLFW, and OpenGL operations. Worker threads publish snapshots and must not call rendering APIs.

## Authoritative state and frames

The live physics source uses a 13-component rigid-body representation:

| Indices | Quantity | Frame / storage |
|---|---|---|
| 0–2 | Position | Local navigation, NED |
| 3–5 | Velocity | Local navigation, NED |
| 6–9 | Attitude | Scalar-first `[w, x, y, z]`, body to navigation |
| 10–12 | Angular velocity | Body frame |

The authoritative physics state is distinct from estimated navigation state, synthetic demonstration data, and recorded datasets with missing fields. See [COORDINATE_SYSTEMS.md](../COORDINATE_SYSTEMS.md).

## Live simulation implementation

`SixDoFLiveSimulation` uses the existing 6DOF dynamics and RK4 integration to publish snapshots. The documented live demonstration uses a 0.01-second timestep and simplified forces/moments. It is not evidence that a complete powered guided-rocket mission has been visualized end-to-end.

`VisualizationApp` manages worker lifecycle and source switching. Worker cancellation is requested before joining/disposing the producer. Opening a new source should stop the previous live producer; resource cleanup and source switching are part of the manual acceptance checks.

## Snapshot retention

`LiveTelemetryPublisher` retains the latest snapshot; intermediate snapshots may be overwritten before the GUI reads them. `VisualizationSession` retains a bounded plotting history (the earlier implementation documented a capacity of 2,048 samples). Neither mechanism is a lossless flight recorder. Use recorded telemetry CSV for complete post-run history.

## Panels

- **Dashboard:** active source and current state.
- **Playback:** recorded-data timeline and playback controls.
- **Telemetry:** selected time histories via ImPlot.
- **Trajectory:** NED-based track and altitude history.
- **Vehicle 3D:** quaternion-driven orientation display.
- **Events:** available event annotations.
- **Performance:** GUI frame-interval observations.
- **HIL Monitor:** software/hardware controller status and telemetry.

Availability of a panel is distinct from verification of every mode and failure scenario.

## Synchronization and timing

GUI frame rate, physics update rate, sensor sampling cadence, and external-controller response rate are separate metrics. VSync may influence GUI frame intervals, but must not become the physics time source. The 100 Hz HIL physics benchmark results do not imply a 100 FPS renderer or a fresh 100 Hz ESP32 command stream.

## Verification status

Automated regression tests and existing visualization microbenchmarks cover components of the system. The recorded visualization performance report describes a measured historical GUI session; it does not establish GPU-only rendering time or all-workload performance. GUI hardware disconnect/reconnect behavior remains subject to a separate manual sign-off. See [VISUALIZATION_USER_GUIDE.md](VISUALIZATION_USER_GUIDE.md) and [VisualizationPerformanceReport.md](../performance/VisualizationPerformanceReport.md).
