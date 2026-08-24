# AeroForge Phase 0 Completion Checklist

## 1. Purpose

This document combines the Phase 0.32 architecture review and Phase 0.33 completion/tagging gate.

Do not tag Phase 0 complete until this review passes.

---

## 2. Build and Repository

- [ ] Root CMake configuration is authoritative.
- [ ] Project builds successfully through Visual Studio.
- [ ] Project builds successfully through command-line CMake.
- [ ] C++20 requirement is explicit.
- [ ] Compiler warnings are enabled.
- [ ] Build has zero meaningful warnings.
- [ ] Generated build artifacts are ignored by Git.
- [ ] Repository structure matches the documented direction.

---

## 3. Documentation

- [ ] `CODING_STANDARDS.md` exists.
- [ ] `INITIAL_SIMULATION_MISSION.md` exists.
- [ ] `SIMULATION_SCOPE_AND_ASSUMPTIONS.md` exists.
- [ ] `COORDINATE_SYSTEMS.md` exists.
- [ ] `UNITS_AND_CONVENTIONS.md` exists.
- [ ] `SYSTEM_ARCHITECTURE.md` exists.
- [ ] `CONFIGURATION_AND_REPRODUCIBILITY.md` exists.
- [ ] `TELEMETRY_AND_NUMERICAL_SAFETY.md` exists.
- [ ] `VERIFICATION_AND_VALIDATION_PLAN.md` exists.
- [ ] `REPOSITORY_STRUCTURE.md` exists.
- [ ] `SYSTEM_DESIGN.md` exists.
- [ ] `PHASE_1_ACCEPTANCE_CRITERIA.md` exists.

---

## 4. Architecture Review

### Coupling

- [ ] Math does not depend on high-level systems.
- [ ] Environment does not depend on a specific vehicle.
- [ ] Dynamics does not depend on a specific controller.
- [ ] Visualization is not part of the physics core.
- [ ] Telemetry does not control physics.
- [ ] No circular module dependency has been designed.

### Ownership

- [ ] Simulation-state ownership is clear.
- [ ] Vehicle composition is clear.
- [ ] Mass-property ownership is clear.
- [ ] Sensor/actuator state ownership is clear.
- [ ] No global mutable state is required by the architecture.

### Frames

- [ ] Navigation axes are unambiguous.
- [ ] Body axes are unambiguous.
- [ ] Rotation signs are unambiguous.
- [ ] Quaternion direction is unambiguous.
- [ ] Rotation-matrix naming is unambiguous.
- [ ] Force/moment frame contracts are documented.

### Units

- [ ] SI internal units are documented.
- [ ] Radians are documented.
- [ ] NED altitude/down distinction is documented.
- [ ] Configuration and telemetry unit rules are documented.

### Time and Scheduling

- [ ] Simulation time is separate from wall-clock time.
- [ ] Fixed-step strategy is documented.
- [ ] Multi-rate sensor/controller behavior is allowed.
- [ ] Integrator and dynamics responsibilities are separate.

### Determinism

- [ ] Master seed requirement is documented.
- [ ] Random-stream strategy is documented.
- [ ] Run metadata requirements are documented.
- [ ] Reproducibility limitations are documented.

### Safety

- [ ] NaN/inf handling is documented.
- [ ] Invalid mass/timestep handling is documented.
- [ ] Quaternion safety is documented.
- [ ] Silent clamping policy is documented.
- [ ] Configuration validation occurs before simulation.

### Verification

- [ ] Analytical baseline cases are defined.
- [ ] Floating-point comparison policy is defined.
- [ ] Integrator convergence testing is required.
- [ ] Physics-model definition-of-done is defined.
- [ ] Phase 1 acceptance criteria are explicit.

---

## 5. Contradiction Review

Read the Phase 0 documents together and verify that no two files disagree about:

- axes
- handedness
- units
- quaternion direction
- gravity direction
- time representation
- initial operating domain
- truth-state access
- configuration format
- determinism
- vehicle scope

If a contradiction exists, resolve it before tagging the phase.

---

## 6. Scope Review

Confirm that Phase 0 has not accidentally committed AeroForge to:

- a rocket-only engine
- a visualization-first architecture
- one numerical integrator
- one atmosphere model
- one gravity model
- direct controller access to truth state
- wall-clock-driven physics
- hidden unit conversion
- global random state
- untestable monolithic classes

---

## 7. Final Git Review

Run:

```powershell
git status
```

Confirm only intentional source/documentation changes remain.

Then:

```powershell
git add .
git commit -m "Complete AeroForge Phase 0 system design"
```

Review:

```powershell
git log -1
git status
```

The working tree should be clean.

---

## 8. Phase 0 Tag

Once every required item passes:

```powershell
git tag -a v0.1.0-phase0 -m "AeroForge Phase 0 complete"
```

Then, if the repository has a configured remote and you want the tag published:

```powershell
git push
git push origin v0.1.0-phase0
```

---

## 9. Phase 0 Exit Criteria

- [ ] Architecture review passed.
- [ ] Documentation review passed.
- [ ] Contradiction review passed.
- [ ] Build review passed.
- [ ] Git working tree clean.
- [ ] Phase 0 completion commit created.
- [ ] `v0.1.0-phase0` tag created.
- [ ] Phase 1 acceptance criteria ready for implementation.

When these are complete, AeroForge may begin Phase 1.
