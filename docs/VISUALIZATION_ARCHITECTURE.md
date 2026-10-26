
## Phase 17.10 — Physics-Driven Live Visualization

### Overview

AeroForge supports both recorded telemetry playback and
physics-driven live visualization.

Recorded visualization loads a previously generated CSV.

Live visualization allows AeroForge to calculate 6DOF
rigid-body motion and publish state snapshots while the
GUI remains responsive.

### Architecture

```text
             AeroForge Physics Engine
                      |
              SixDoFDynamicsModel
                      |
                RK4Integrator
                      |
                SimulationRunner
                      |
                13-State Rigid Body
                      |
              Snapshot Conversion
                      |
             LiveTelemetryPublisher
                      |
              VisualizationSession
                      |
        +-------------+-------------+
        |             |             |
     Telemetry     Trajectory     Vehicle 3D
        |             |             |
        +-------------+-------------+
                      |
                Dear ImGui GUI
```

### Authoritative State

The 6DOF state is represented as:

| Index | Variable | Frame |
|---|---|---|
| 0–2 | Position | Navigation / NED |
| 3–5 | Velocity | Navigation / NED |
| 6–9 | Quaternion [w, x, y, z] | Body to navigation |
| 10–12 | Angular velocity | Body |

The quaternion is scalar-first and represents the
orientation of the body frame relative to navigation.

### Live Simulation

Implementation:

`include/visualization/SixDoFLiveSimulation.h`

`src/visualization/SixDoFLiveSimulation.cpp`

The live simulation:

- Uses the existing 6DOF dynamics engine.
- Integrates using RK4.
- Uses simulation time as the authoritative time.
- Can pace execution using a steady wall-clock timer.
- Publishes snapshots to LiveTelemetryPublisher.
- Supports cancellation using std::stop_token.

The current demonstration runs 1,000 integration steps
at dt = 0.01 seconds, representing 10 simulated seconds.

It uses simplified constant forces and moments.

### Threading and Ownership

VisualizationApp starts the real simulation on a
std::jthread.

The simulation thread calculates state and publishes
snapshots. It never calls GLFW, OpenGL, Dear ImGui, or
ImPlot rendering functions.

The GUI thread reads the publisher through
VisualizationSession.

The publisher uses a mutex to protect access to its
latest snapshot.

VisualizationApp requests cancellation and joins its
worker thread before releasing the associated publisher.

Switching live sources, loading a recording, or closing
the application stops the active simulation worker.

### Snapshot Retention

LiveTelemetryPublisher stores only the latest snapshot.

This design avoids an unbounded publisher-side queue,
but intermediate simulation states can be overwritten
before the GUI observes them.

VisualizationSession retains a bounded history for
live plotting, with a capacity of 2,048 samples.

The retained GUI history must not be treated as a
lossless flight recording.

Use recorded CSV telemetry when a complete history is
required.

### Current Simulation Limitations

The live 6DOF demonstration is a functional integration
test of the simulator and visualizer.

It does not currently include:

- Time-varying rocket engine thrust
- Propellant depletion
- Vehicle mass depletion
- Complete aerodynamic force modeling
- Active guidance and attitude control
- Navigation estimator outputs

Estimated position, velocity, attitude, and angular
velocity currently mirror the authoritative simulated
truth state in this demonstration.

They are not independent navigation estimates.

### Verification

Phase 17 verification includes:

- Recorded telemetry loading
- 6DOF quaternion animation
- Physics-driven live visualization
- Worker cancellation and source switching
- Automated regression testing
- CPU visualization benchmarks
- GUI frame-interval monitoring
- Process memory measurement

See:

`docs/performance/VisualizationPerformanceReport.md`
