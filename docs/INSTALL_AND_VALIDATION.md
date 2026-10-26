
## Phase 17 — Final Build and Validation

### Prerequisites

- CMake
- Ninja
- C++20-compatible compiler
- Windows x64 MSVC development environment
- OpenGL-capable graphics hardware
- Visualization dependencies configured by the project

### Release Configuration

Run from the AeroForge repository root:

```powershell
cmake -S . -B out/build/x64-release-clean -G Ninja -DCMAKE_BUILD_TYPE=Release -DAEROFORGE_BUILD_VISUALIZATION=ON -DBUILD_TESTING=ON
```

### Build All Targets

```powershell
cmake --build out/build/x64-release-clean --parallel 8
```

### Run the Regression Suite

```powershell
ctest --test-dir out/build/x64-release-clean --output-on-failure
```

The full automated suite was reported passing after
the Phase 17 live 6DOF integration was implemented.

### Run the Visualizer

```powershell
.\out\build\x64-release-clean\AeroForgeVisualizer.exe
```

### Recorded 6DOF Validation

Build the example:

```powershell
cmake --build out/build/x64-release-clean --target AeroForge6DoFRecordingExample --parallel 8
```

Generate telemetry:

```powershell
.\out\build\x64-release-clean\examples\AeroForge6DoFRecordingExample.exe "examples/output/rocket_6dof_flight.csv"
```

Load the generated CSV in AeroForge Visualizer.

Verify:

- Simulation time increases.
- Position and velocity change.
- Quaternion components change.
- Vehicle 3D orientation changes during playback.
- Seeking updates the vehicle attitude.

### Live 6DOF Validation

1. Launch AeroForge Visualizer.
2. Select File > Start 6DOF live simulation.
3. Observe the trajectory and Vehicle 3D panels.
4. Verify simulation time advances.
5. Verify the wireframe rotates.
6. Select File > Stop live source.
7. Restart the simulation.
8. Switch to synthetic live mode.
9. Load recorded telemetry afterward.
10. Confirm the application closes cleanly.

### Performance Validation

Run the visualization benchmark target:

```powershell
cmake --build out/build/x64-release-clean --target AeroForgeVisualizationBenchmarks --parallel 8
```

Use the Performance panel to collect frame intervals.

Use an independent PowerShell terminal to collect
working-set and private process memory measurements.

Record the test hardware, workload, sampling duration,
and results.

The Phase 17 benchmark report is available at:

`docs/performance/VisualizationPerformanceReport.md`

### Optional Headless Validation

Configure a separate build without visualization:

```powershell
cmake -S . -B out/build/x64-headless -G Ninja -DCMAKE_BUILD_TYPE=Release -DAEROFORGE_BUILD_VISUALIZATION=OFF -DBUILD_TESTING=ON
```

Build:

```powershell
cmake --build out/build/x64-headless --parallel 8
```

Test:

```powershell
ctest --test-dir out/build/x64-headless --output-on-failure
```

Headless validation is a recommended final sign-off
step. Do not mark it completed until it passes.

### Troubleshooting: MSVC Architecture Mismatch

An earlier CMake configuration failed with:

```text
LNK1112: module machine type 'x64'
conflicts with target machine type 'x86'
```

The compiler selected an x64 binary while the linker
selected an x86 binary.

The resolution was to use a consistent x64 MSVC
development environment and configure a fresh Ninja
build directory.

Do not mix Hostx64/x64 compilation with
Hostx86/x86 linking.

For Ninja builds, architecture selection is normally
controlled by the compiler environment rather than
the Visual Studio generator's -A option.

### Final Release Criteria

Phase 17 is ready for sign-off when:

- All intended visualization targets build.
- The current full regression suite passes.
- Recorded 6DOF visualization works.
- Live 6DOF visualization works.
- Stopping and switching sources work safely.
- Performance measurements are documented.
- Manual GUI lifecycle checks pass.
- Documentation matches the implementation.
- The final staged Git diff contains only intended files.
