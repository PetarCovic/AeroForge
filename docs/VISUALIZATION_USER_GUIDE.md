# AeroForge Visualizer — user guide (Phases 17.3–17.9 candidate)

## Build

From the project root in a configured MSVC terminal:

```powershell
cmake -S . -B build-phase17-docking -DAEROFORGE_BUILD_VISUALIZATION=ON -DBUILD_TESTING=ON
cmake --build build-phase17-docking --config Debug --parallel 8
ctest --test-dir build-phase17-docking -C Debug --output-on-failure
.\build-phase17-docking\Debug\AeroForgeVisualizer.exe
```

With Visual Studio's single-configuration Ninja preset, use that preset's build directory and executable path.

## Recorded playback

1. Use **File > Open telemetry CSV...**, enter an absolute path and press **Load recording**. Alternatively, drag a CSV onto the application window.
2. The **Playback** panel allows play, pause, restart, seek, and changing playback speed.
3. All panels query the same simulation timestamp from `VisualizationSession`.
4. Use **File > Open events CSV...** to load separate event annotations. This does not alter simulation state.

The canonical visualization CSV is produced by `TelemetryCsvLoader::write`. It contains one nonnegative, strictly increasing timestamp per row and 35 numeric state components. Its 36 column headers are exactly those emitted by that method. Required fields: timestamp, truth NED position, truth NED velocity and truth body-to-navigation quaternion. Optional groups must appear in full. Missing estimate groups default to corresponding truth data; missing angular rates/moments/errors default to zero. This is for presentation purposes, not an assertion that the navigation estimate was correct.

**Compatibility limitation:** The local `CsvTelemetryWriter.cpp` and `TelemetrySample.h` were not supplied in the uploaded ZIP. The current simulation telemetry CSV may have a different schema. If so, map its columns in the loader or export a canonical visualization CSV using the writer; never relabel unmatched columns silently.

**Frame conventions:** NED truth position (`+X North, +Y East, +Z Down`), FRD body (`+X forward, +Y right, +Z down`), `q_NB` rotates body vectors into NED, quaternion order wxyz. Altitude above the NED origin is `-positionN.z` (not necessarily above mean sea level).

## Live mode

**File > Start synthetic live demo** starts a test producer that publishes to `LiveTelemetryPublisher`. The synthetic values DO NOT represent a physical rocket simulation. The external integration API is `VisualizationSession::attachLive(std::shared_ptr<const LiveTelemetryPublisher>)`. The actual AeroForge simulation producer must publish `VisualizationSnapshot` at its own telemetry cadence; that producer-to-GUI wiring is not provided here because the simulation's callback interface was not part of the supplied files. `LiveVisualizationModel` retains bounded trajectory history; the UI maintains a 2048-snapshot live plot window.

Use **File > Stop live demo** before opening a new recording. The publisher uses `std::try_to_lock` for producers; a full proof of bounded worst-case execution time and deterministic equivalence requires instrumentation of the actual simulation callback.

## Visualization

- **Dashboard:** time, truth/estimate position, speed, attitude and source.
- **Telemetry:** channel-selectable downsampled time series; time series may be simplified visually. Full snapshots remain loaded.
- **Trajectory:** east/north ground track and altitude over simulation time.
- **Vehicle 3D:** lightweight 3D wireframe projected into an ImGui draw-list with mouse orbit and zoom. No true GPU mesh/shader pipeline in this candidate.
- **Events:** optional event annotations loaded separately.
- **Performance:** rolling wall-frame duration with interpolation-based p50/p95/p99, FPS, and frame budget miss rate. These are GUI wall intervals, including VSync and scheduling, not simulation RTF or GPU timings.

## Event annotations format

Event CSV uses exactly `simulationTimeS,type,message` followed by rows, for example:

```csv
simulationTimeS,type,message
0.5,INFO,Controller enabled
3.2,WARNING,GPS fault injected
```

This simple event importer treats everything after the second comma as message text; it does not support quoted multiline fields.

## Validation requirements

- Debug and Release configurations build in headless and graphical modes.
- Run the complete CTest suite and verify GUI interaction manually.
- Validate CSV round-trip, corrupt/missing data, duplicate times, quaternion inputs, and interpolation boundaries.
- Record frame p50/p95/p99 under the same hardware, workload, VSync, size, and build settings.
- Separately verify simulation output equality with publishing enabled/disabled against the **actual** producer interface.
- Do not publish timing claims until measured. The supplied benchmark runner prints CSV for new runs but does not invent historical results.
