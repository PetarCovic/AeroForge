# AeroForge Build, Installation, and Validation

## Scope

This guide covers the currently demonstrated Windows x64 C++20 build, automated tests, simulation tools, visualization, and optional physical ESP32 HIL connection. It is not a claim of complete reference-mission qualification or a cross-platform support guarantee.

## Prerequisites

- Windows x64 and an x64 MSVC development environment
- CMake and Ninja available on `PATH`
- A C++20-capable compiler
- OpenGL-capable graphics hardware for the GUI
- Network access on first configure if CMake FetchContent needs to retrieve GLFW, Dear ImGui, ImPlot, or GoogleTest
- Optional: ESP-IDF and a CH340-connected ESP32 for physical HIL

Avoid mixing x86 and x64 compiler/linker environments; `LNK1112` usually indicates an architecture mismatch.

## Configure and build

From the repository root:

```powershell
cmake -S . -B out/build/x64-release-clean -G Ninja -DCMAKE_BUILD_TYPE=Release -DAEROFORGE_BUILD_VISUALIZATION=ON -DBUILD_TESTING=ON
cmake --build out/build/x64-release-clean --parallel 8
```

Build individual targets when appropriate:

```powershell
cmake --build out/build/x64-release-clean --target AeroForgeTests AeroForgeVisualizer AeroForgeHilBenchmarks AeroForgeHilHardwareBenchmarks AeroForgeVerificationAudit --parallel 8
```

These commands assume the HIL hardware and verification-audit targets have been registered in the local root CMake configuration.

## Automated tests

```powershell
ctest --test-dir out/build/x64-release-clean --output-on-failure
```

Repeat registered tests to look for intermittent failures:

```powershell
ctest --test-dir out/build/x64-release-clean --repeat until-fail:3 --output-on-failure
```

A passing suite verifies its registered assertions; it does not establish external physical fidelity, hardware fault recovery, or hard real-time guarantees.

## Visualization

```powershell
.\out\build\x64-release-clean\AeroForgeVisualizer.exe
```

See the [Visualizer user guide](visualization/VISUALIZATION_USER_GUIDE.md) for recorded/live/HIL modes and manual checks.

## Software HIL benchmark and evidence audit

```powershell
.\out\build\x64-release-clean\AeroForgeHilBenchmarks.exe 100000 out\verification\software-unpaced
.\out\build\x64-release-clean\AeroForgeHilBenchmarks.exe 10000 out\verification\software-paced paced
```

The demonstrated benchmark writes `hil_samples.csv` and `hil_summary.json` in each selected output directory. Audit these outputs without changing or rerunning working benchmark code:

```powershell
.\out\build\x64-release-clean\AeroForgeVerificationAudit.exe software out\verification\software-unpaced\hil_samples.csv out\verification\software-unpaced\hil_summary.json out\verification\SoftwareAudit.md
.\out\build\x64-release-clean\AeroForgeVerificationAudit.exe software out\verification\software-paced\hil_samples.csv out\verification\software-paced\hil_summary.json out\verification\SoftwarePacedAudit.md
```

The auditor may warn that the software CSV lacks a `deadline_misses` column and that the summary value was read from JSON. This warning is a documented schema limitation, not a failed benchmark.

## Physical ESP32 HIL benchmark

After verifying the flashed firmware's application UART rate and the Windows port:

```powershell
.\out\build\x64-release-clean\AeroForgeHilHardwareBenchmarks.exe COM3 1000 out\hil-benchmarks\hardware-460800 460800
```

`COM3` and `460800` reproduce the demonstrated local setup; do not assume they apply to another board or to firmware left at 115200 baud. Serial terminals must be closed first.

Audit an existing captured run:

```powershell
.\out\build\x64-release-clean\AeroForgeVerificationAudit.exe hardware out\hil-benchmarks\hardware-460800\hil_hardware_samples.csv out\hil-benchmarks\hardware-460800\hil_hardware_summary.json out\verification\HardwareAudit.md
```

The demonstrated run completed its simulation steps without deadline misses, but its new-command acceptance rate did not meet the benchmark's 90% optional threshold. Do not describe it as a fresh 100 Hz physical controller loop. See [current verification results](verification/CurrentVerificationResults.md).

## Optional headless build

```powershell
cmake -S . -B out/build/x64-headless -G Ninja -DCMAKE_BUILD_TYPE=Release -DAEROFORGE_BUILD_VISUALIZATION=OFF -DBUILD_TESTING=ON
cmake --build out/build/x64-headless --parallel 8
ctest --test-dir out/build/x64-headless --output-on-failure
```

Headless verification must be run separately; these instructions alone do not establish that it has passed.

## Hardware firmware

The connected board is an original ESP32-D0WD-V3, not an ESP32-S3. The existing firmware source folder may still be named `firmware/esp32s3`; the ESP-IDF target must be `esp32`:

```powershell
cd firmware/esp32s3
idf.py set-target esp32
idf.py build
idf.py -p COM3 -b 115200 flash
```

The flashing baud and the firmware's **application UART baud** are separate. Follow [ESP32Setup.md](hil/ESP32Setup.md) before changing either.

## Manual safety validation

Perform these without attached physical actuators: software GUI loopback, physical GUI communications at matching baud, clean stop and COM-port release, occupied-port error, disconnect with zero-command fallback, reconnect and fresh-session startup. Record observed results. Do not infer success from a hardware benchmark alone.

## Troubleshooting

- **`LNK1112`:** open a consistent x64 MSVC environment and reconfigure a clean build directory.
- **COM port unavailable:** close `idf.py monitor` and other terminal applications.
- **No hardware responses:** confirm ESP32 family, COM port, and matching firmware/host application baud rate.
- **Benchmark audit fails to open CSV:** use the exact filenames printed by that benchmark, not assumed schema names.
- **Hardware acceptance warning:** distinguish command updates from steps reusing a still-valid previous command.
