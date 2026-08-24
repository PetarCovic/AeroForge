# AeroForge Repository Structure

## 1. Purpose

This document defines the intended long-term repository organization for AeroForge.

The repository should remain understandable as the project grows from a small simulator into a multi-module engineering platform.

---

## 2. Target Layout

```text
AeroForge/
├── CMakeLists.txt
├── README.md
├── .clang-format
├── .gitattributes
├── .gitignore
│
├── include/
│   └── aeroforge/
│       ├── core/
│       ├── math/
│       ├── environment/
│       ├── dynamics/
│       ├── vehicle/
│       ├── sensors/
│       ├── actuators/
│       ├── gnc/
│       ├── failures/
│       ├── logging/
│       └── analysis/
│
├── src/
│   ├── core/
│   ├── math/
│   ├── environment/
│   ├── dynamics/
│   ├── vehicle/
│   ├── sensors/
│   ├── actuators/
│   ├── gnc/
│   ├── failures/
│   ├── logging/
│   ├── analysis/
│   └── main.cpp
│
├── tests/
│   ├── unit/
│   ├── verification/
│   ├── integration/
│   └── regression/
│
├── benchmarks/
│
├── config/
│   ├── examples/
│   └── schemas/
│
├── docs/
│
├── examples/
│
├── analysis/
│   ├── scripts/
│   └── notebooks/
│
├── tools/
│
└── cmake/
```

Not every directory needs to exist immediately.

Directories should be created when they gain real content rather than being populated with meaningless placeholder files.

---

## 3. `include/aeroforge`

Contains public/internal reusable headers exposed across AeroForge targets.

The `aeroforge/` prefix prevents include-path collisions.

Example:

```cpp
#include <aeroforge/math/Vector3.h>
```

---

## 4. `src`

Contains C++ implementation files and private implementation headers where appropriate.

Subsystem directories should mirror meaningful module boundaries rather than arbitrary file counts.

---

## 5. `tests`

### `unit`

Small isolated tests.

### `verification`

Analytical, convergence, conservation, and model-verification cases.

### `integration`

Tests involving multiple subsystems.

### `regression`

Stable reference behavior and bug-regression cases.

---

## 6. `benchmarks`

Contains performance benchmarks only.

Benchmarks must not become correctness tests.

---

## 7. `config`

Contains example mission/vehicle configuration and later schemas.

Production simulation results should not be committed here.

---

## 8. `docs`

Contains engineering specifications.

Phase 0 documentation includes:

```text
CODING_STANDARDS.md
INITIAL_SIMULATION_MISSION.md
SIMULATION_SCOPE_AND_ASSUMPTIONS.md
COORDINATE_SYSTEMS.md
UNITS_AND_CONVENTIONS.md
SYSTEM_ARCHITECTURE.md
CONFIGURATION_AND_REPRODUCIBILITY.md
TELEMETRY_AND_NUMERICAL_SAFETY.md
VERIFICATION_AND_VALIDATION_PLAN.md
REPOSITORY_STRUCTURE.md
SYSTEM_DESIGN.md
PHASE_1_ACCEPTANCE_CRITERIA.md
PHASE_0_COMPLETION_CHECKLIST.md
```

---

## 9. `examples`

Contains small runnable examples demonstrating public simulator capabilities.

Examples should be simpler than the flagship application and useful for learning/testing APIs.

---

## 10. `analysis`

Contains Python or other analysis tooling used to inspect simulation output.

This is intentionally separate from the C++ simulation core.

---

## 11. `tools`

Contains developer utilities that are not part of the physics engine.

Examples may later include:

- configuration converters
- telemetry inspection utilities
- data preprocessing

---

## 12. `cmake`

Contains reusable CMake helper modules only when the root build actually needs them.

Do not create CMake abstraction layers before they provide real value.

---

## 13. CMake Target Direction

The expected eventual target structure is approximately:

```text
AeroForgeCore
      ↑
      ├── AeroForgeCLI
      ├── AeroForgeTests
      ├── AeroForgeBenchmarks
      └── future visualization/tools
```

The simulation engine should become a reusable library target.

The command-line executable should be a thin consumer of that library.

---

## 14. No Generated Artifacts in Git

Do not commit:

```text
.vs/
out/
build/
CMakeFiles/
CMakeCache.txt
*.obj
*.o
*.exe
*.pdb
```

Simulation output datasets should also remain outside source control unless they are intentionally small validation fixtures.

---

## 15. Phase 0.28 Acceptance Criteria

- [x] Long-term repository layout defined.
- [x] Public header convention defined.
- [x] Source/test/benchmark responsibilities separated.
- [x] Configuration and analysis locations defined.
- [x] Documentation location defined.
- [x] Future CMake target direction defined.
- [x] Generated artifacts excluded from version control.
- [x] Empty placeholder directories are not required.
