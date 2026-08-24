# AeroForge Coding Standards

## 1. Purpose

This document defines the coding conventions and engineering standards for AeroForge.

AeroForge is a modern C++ aerospace simulation, guidance, navigation, and control platform. The codebase is intended to emphasize correctness, numerical reliability, portability, readability, deterministic behavior, and maintainability.

These standards apply to production source code, tests, benchmarks, simulation utilities, and supporting C++ tools unless a specific subsystem documents an exception.

---

## 2. Language Standard

AeroForge uses **C++20** as its minimum language standard.

Modern C++ features should be used when they improve correctness, safety, or clarity. Examples include:

- `constexpr`
- `[[nodiscard]]`
- `enum class`
- `std::array`
- `std::span`
- `std::optional`
- `std::unique_ptr`
- `std::numbers`

Advanced language features should not be introduced solely for cleverness. Numerical and simulation code should remain easy to inspect and reason about.

---

## 3. Build System

AeroForge uses **CMake** as the authoritative build system.

The project must not depend on Visual Studio solution or project files to build.

The same source tree should remain compatible with multiple development environments and compilers where practical, including:

- MSVC
- GCC
- Clang

Target-specific CMake configuration is preferred over unnecessary global compiler state.

---

## 4. Namespace Convention

All AeroForge code must live under the root namespace:

```cpp
namespace aeroforge
{
}
```

Major subsystems should use nested namespaces where appropriate:

```cpp
namespace aeroforge::math
{
}

namespace aeroforge::dynamics
{
}

namespace aeroforge::environment
{
}

namespace aeroforge::vehicle
{
}

namespace aeroforge::gnc
{
}
```

Do not use:

```cpp
using namespace std;
```

This rule is especially strict for header files.

Standard-library names should remain explicitly qualified:

```cpp
std::vector<double>
std::string
std::cout
```

---

## 5. File Naming

Files representing C++ classes or major types use **PascalCase**.

Examples:

```text
Vector3.h
Vector3.cpp
Quaternion.h
Quaternion.cpp
RigidBodyState.h
RigidBodyState.cpp
AtmosphereModel.h
AtmosphereModel.cpp
SimulationRunner.h
SimulationRunner.cpp
```

Header files use the `.h` extension.

Source files use the `.cpp` extension.

Where a class has both a header and implementation file, the filenames should match the type name.

---

## 6. Type Naming

Classes, structs, enums, and other named types use **PascalCase**.

Examples:

```cpp
class SimulationRunner;
class AtmosphereModel;
class RocketMotor;

struct VehicleState;
struct MassProperties;
struct SensorMeasurement;
```

Use a `struct` primarily for data-oriented aggregates.

Use a `class` when the type owns significant behavior, invariants, or resources.

---

## 7. Function Naming

Functions and methods use **camelCase**.

Examples:

```cpp
calculateDragForce();
updateState();
getAltitude();
transformToBodyFrame();
computeDerivative();
```

Do not mix naming conventions inside the project.

Avoid styles such as:

```cpp
CalculateDragForce();
calculate_drag_force();
Calculate_dragForce();
```

---

## 8. Variable Naming

Local variables and parameters use **camelCase**.

Examples:

```cpp
double simulationTime;
double airDensity;
double angularVelocity;
double dragCoefficient;
```

Avoid Hungarian notation and type prefixes such as:

```cpp
dVelocity
fMass
pVehicle
```

The type system should communicate type information.

---

## 9. Private Member Variables

Private member variables use **camelCase with a trailing underscore**.

Example:

```cpp
class SimulationRunner
{
private:
    double simulationTime_;
    double timeStep_;
};
```

This clearly distinguishes member state from local variables and parameters.

---

## 10. Constants

Named constants use **PascalCase**.

Examples:

```cpp
constexpr double StandardGravity = 9.80665;
constexpr double SeaLevelPressure = 101325.0;
```

Class constants may use the same style:

```cpp
class AtmosphereModel
{
private:
    static constexpr double StandardGravity = 9.80665;
};
```

Avoid uppercase constant names that resemble preprocessor macros.

---

## 11. Enumerations

Prefer scoped enumerations:

```cpp
enum class FlightPhase
{
    PreLaunch,
    PoweredAscent,
    Coast,
    Descent,
    Landed
};
```

Avoid unscoped `enum` unless there is a specific reason to use one.

Scoped enums reduce accidental conversions and naming collisions.

---

## 12. Boolean Naming

Boolean values should read naturally and clearly describe their meaning.

Preferred examples:

```cpp
bool isActive;
bool hasFailed;
bool isValid;
bool shouldTerminate;
bool enableLogging;
```

Boolean-returning functions should follow the same principle:

```cpp
bool isValid() const;
bool hasFuel() const;
```

Avoid vague names such as:

```cpp
bool check;
bool failure;
bool flag;
```

unless the meaning is obvious from an extremely narrow local context.

---

## 13. Units

AeroForge uses **SI units internally** for physics and simulation calculations.

Canonical units are:

| Quantity | Unit |
|---|---|
| Distance | meters |
| Time | seconds |
| Velocity | meters per second |
| Acceleration | meters per second squared |
| Mass | kilograms |
| Force | newtons |
| Torque / Moment | newton-meters |
| Angle | radians |
| Angular velocity | radians per second |
| Pressure | pascals |
| Temperature | kelvin |
| Density | kilograms per cubic meter |

Unit conversions should occur at input, output, configuration, or visualization boundaries.

Core physics logic should not mix SI and non-SI units.

Where an API boundary could be ambiguous, units should be made explicit in the name:

```cpp
setAltitudeMeters(double altitude);
```

Strongly typed physical units may be considered later, but AeroForge will not initially implement a custom units framework.

---

## 14. Angles

All angles are represented in **radians internally**.

Degrees may be accepted or displayed by configuration, user interfaces, or visualization systems, but they must be converted before entering core physics calculations.

Do not allow different subsystems to silently use different angular units.

---

## 15. Floating-Point Type

Use `double` as the default floating-point type throughout AeroForge.

Examples:

```cpp
double mass;
double time;
double velocity;
double pressure;
```

Use `float` only when profiling or a specific external interface demonstrates a concrete need.

Numerical accuracy takes priority over small per-scalar memory savings in the default simulation core.

---

## 16. Integer Types

Use integer types according to their purpose.

For container indexing, prefer:

```cpp
std::size_t index;
```

When an exact binary width is part of an interface or serialization format, use fixed-width types such as:

```cpp
std::uint32_t
std::uint64_t
```

Do not replace all ordinary integers with fixed-width types unless the width is actually part of the requirement.

---

## 17. Const Correctness

Use `const` when a value or object should not be modified.

For larger objects that only need to be read:

```cpp
void update(const VehicleState& state);
```

Member functions that do not modify object state should be marked `const`:

```cpp
double getMass() const;
```

Small primitive parameters should normally be passed by value:

```cpp
double calculateDrag(
    double airDensity,
    double velocity,
    double dragCoefficient,
    double area);
```

Avoid unnecessary copies of larger objects.

---

## 18. Parameter Passing

General guidelines:

- Small primitive values: pass by value.
- Large read-only objects: pass by `const` reference.
- Objects intentionally modified by the function: pass by non-const reference.
- Use pointers when nullability or pointer semantics are meaningful.
- Do not use pointers merely because an object is large.

Examples:

```cpp
void setTimeStep(double timeStep);

void update(const VehicleState& state);

void integrate(VehicleState& state);
```

---

## 19. Ownership and Memory Management

Prefer automatic storage duration and clear ownership.

Default preference order:

1. Stack objects
2. `std::unique_ptr`
3. `std::shared_ptr` only when ownership is genuinely shared

Avoid manual owning `new` and `delete`.

Avoid:

```cpp
Vehicle* vehicle = new Vehicle();
delete vehicle;
```

Prefer:

```cpp
Vehicle vehicle;
```

or, when dynamic ownership is required:

```cpp
std::unique_ptr<Vehicle> vehicle;
```

Raw pointers may be used as non-owning observers when their lifetime relationship is clear.

Ownership must never be ambiguous.

---

## 20. Global State

Global mutable state is prohibited unless a narrowly justified exception is documented.

Avoid constructs such as:

```cpp
double globalSimulationTime;
Vehicle* activeVehicle;
Atmosphere atmosphere;
```

Simulation state should live inside explicitly owned objects.

This requirement supports:

- deterministic execution
- unit testing
- parallel Monte Carlo simulation
- multiple simultaneous simulation instances
- reproducibility

---

## 21. Random Number Generation

Subsystems must not independently construct uncontrolled random-number generators whenever they need noise.

Avoid:

```cpp
std::random_device rd;
```

inside arbitrary simulation components.

Random behavior must eventually derive from explicitly managed simulator RNG state.

The long-term deterministic requirement is:

```text
same configuration
+
same initial conditions
+
same random seed
=
same simulation result
```

This is required for failure reproduction and Monte Carlo debugging.

---

## 22. Error Handling

Invalid configuration or physically invalid state should be detected rather than silently corrected.

Example:

```cpp
if (timeStep <= 0.0)
{
    throw std::invalid_argument("Time step must be positive.");
}
```

Do not silently hide invalid physics:

```cpp
if (mass < 0.0)
{
    mass = 0.0;
}
```

unless clamping is an explicitly documented part of the physical model.

The simulator should eventually distinguish among:

- configuration errors
- programming errors
- numerical failures
- recoverable simulation events
- physical limits and saturation

---

## 23. `[[nodiscard]]`

Use `[[nodiscard]]` when ignoring a return value would likely indicate an error.

Examples:

```cpp
[[nodiscard]]
bool validateConfiguration() const;
```

```cpp
[[nodiscard]]
Vector3 calculateForce() const;
```

Do not apply `[[nodiscard]]` indiscriminately to every function.

---

## 24. `constexpr`

Use `constexpr` where compile-time evaluation is natural and improves correctness or clarity.

Example:

```cpp
constexpr double StandardGravity = 9.80665;
```

Mathematical utilities may also use `constexpr` when appropriate.

Do not introduce unnecessary template complexity solely to maximize compile-time evaluation.

---

## 25. Header Protection

Use:

```cpp
#pragma once
```

for AeroForge headers.

Example:

```cpp
#pragma once

namespace aeroforge
{

class SimulationRunner
{
};

}
```

---

## 26. Include Order

Use the following include order:

1. The source file's own header
2. Other AeroForge headers
3. Third-party headers
4. Standard-library headers

Example:

```cpp
#include "SimulationRunner.h"

#include "math/Vector3.h"

#include <Eigen/Dense>

#include <iostream>
#include <vector>
```

Within each group, keep includes alphabetized where practical.

A `.cpp` file should include its own header first when it has one. This helps detect headers that accidentally depend on another include being present first.

---

## 27. Header Dependencies

Keep header dependencies reasonably small.

Prefer forward declarations when they improve dependency boundaries without harming readability.

Implementation details should remain in `.cpp` files where practical.

Do not over-optimize compile times at the expense of clarity.

---

## 28. Macros

Avoid preprocessor macros unless the preprocessor is genuinely required.

Avoid:

```cpp
#define PI 3.14159
#define GRAVITY 9.81
#define MAX_SENSOR_COUNT 10
```

Prefer typed constants:

```cpp
constexpr double StandardGravity = 9.80665;
```

---

## 29. Comments

Comments should explain **why**, assumptions, mathematical meaning, or non-obvious behavior.

Avoid comments that merely restate obvious code.

Poor:

```cpp
// Add velocity to position.
position += velocity;
```

Useful:

```cpp
// Aerodynamic force is calculated using air-relative velocity,
// not ground-relative velocity.
```

For equations and physical models, document the governing relationship:

```cpp
// Euler rigid-body rotational equation:
// I * omegaDot + omega x (I * omega) = appliedMoment
```

Important physics models should eventually document:

- assumptions
- coordinate frame
- units
- source/reference
- valid operating range
- known limitations

---

## 30. Function Design

Functions should represent one coherent operation.

Prefer:

```cpp
calculateAerodynamicForces();
calculateGravity();
integrateState();
updateSensors();
```

over large functions that combine unrelated responsibilities.

Do not enforce an arbitrary maximum line count.

Numerical methods may legitimately be longer when splitting them would make the mathematics harder to verify.

---

## 31. Magic Numbers

Avoid unexplained numerical literals in simulation logic.

Poor:

```cpp
velocity += 9.80665 * 0.001;
```

Better:

```cpp
velocity += StandardGravity * timeStep;
```

Named constants and configuration values should communicate physical meaning.

---

## 32. Formatting

AeroForge uses `clang-format` for automated formatting.

The project standard is:

- 4-space indentation
- spaces only
- 100-column target
- Allman braces
- left-aligned pointers and references
- sorted includes where practical

A project-level `.clang-format` file should enforce these rules.

Recommended configuration:

```yaml
BasedOnStyle: Microsoft

IndentWidth: 4
TabWidth: 4
UseTab: Never

ColumnLimit: 100

BreakBeforeBraces: Allman

AllowShortFunctionsOnASingleLine: Empty
AllowShortIfStatementsOnASingleLine: Never
AllowShortLoopsOnASingleLine: false

PointerAlignment: Left
ReferenceAlignment: Left

SortIncludes: CaseSensitive
```

Example formatting:

```cpp
namespace aeroforge::dynamics
{

class RigidBody
{
public:
    explicit RigidBody(double mass);

    [[nodiscard]] double getMass() const;

private:
    double mass_;
};

}
```

---

## 33. Static Analysis

AeroForge will use `clang-tidy` for static analysis.

The initial focus should include checks for:

- bug-prone constructs
- unsafe conversions
- suspicious ownership
- modern C++ improvements
- performance issues
- readability problems that affect correctness

The exact rule set may evolve as the project grows.

Formatting is controlled by `clang-format`.

Static analysis is controlled by `clang-tidy`.

---

## 34. Compiler Warnings

AeroForge should normally compile with **zero meaningful warnings**.

Recommended warning levels:

### MSVC

```text
/W4
/permissive-
/Zc:__cplusplus
```

### GCC / Clang

```text
-Wall
-Wextra
-Wpedantic
```

Warnings should not be treated as background noise.

If a warning must be suppressed, the reason should be understood and documented.

Warnings-as-errors may later be enabled in CI.

---

## 35. Determinism

Deterministic simulation is a core engineering requirement.

A simulation should eventually be reproducible from:

- configuration
- initial conditions
- timestep/integration settings
- random seed
- software version

Random behavior, sensor noise, failure injection, and Monte Carlo uncertainty must be designed around this requirement.

---

## 36. Engineering Priorities

When standards conflict, AeroForge should prioritize:

1. Correctness
2. Numerical reliability
3. Reproducibility
4. Clarity
5. Maintainability
6. Portability
7. Performance
8. Concision

Performance optimization should be guided by profiling and validation rather than assumptions.

---

## 37. Summary

The project-wide defaults are:

| Area | AeroForge Standard |
|---|---|
| Language | C++20 |
| Build system | CMake |
| Root namespace | `aeroforge` |
| Types/classes | PascalCase |
| Functions | camelCase |
| Local variables | camelCase |
| Private members | camelCase_ |
| Constants | PascalCase |
| Enumerations | `enum class` |
| Header extension | `.h` |
| Source extension | `.cpp` |
| Header protection | `#pragma once` |
| Indentation | 4 spaces |
| Braces | Allman |
| Line-length target | 100 columns |
| Physics units | SI |
| Internal angles | radians |
| Default floating point | `double` |
| Ownership | stack → `unique_ptr` → `shared_ptr` |
| Manual owning `new` / `delete` | avoid |
| Global mutable state | prohibited |
| Macros | avoid |
| Formatting | `clang-format` |
| Static analysis | `clang-tidy` |
| Compiler warnings | `/W4` or `-Wall -Wextra -Wpedantic` |
| Normal warning count | 0 |
| Randomness | centrally seeded and deterministic |

---

## 38. Revision Policy

These standards are intended to provide a stable baseline, not prevent justified engineering changes.

A convention may be revised when there is a concrete technical reason, such as:

- interoperability with a third-party library
- measurable performance concerns
- improved safety or correctness
- compiler/toolchain constraints
- clearer subsystem architecture

Changes to project-wide standards should be documented and applied consistently.
