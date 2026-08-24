# AeroForge Phase 1 Acceptance Criteria

## 1. Purpose

Phase 1 builds the mathematical foundation required by later flight dynamics.

Phase 1 is not complete merely when math classes exist. The operations must be validated against known results and coordinate-system conventions.

---

## 2. Required Components

Phase 1 should provide:

```text
Vector3
Matrix3
Quaternion
CoordinateTransforms
```

Additional small math utilities may be introduced only when required by these components.

---

## 3. `Vector3`

Required behavior:

- construction
- component access
- addition/subtraction
- scalar multiply/divide
- dot product
- cross product
- magnitude/norm
- squared magnitude
- normalization
- finite-value validation
- comparison helpers for tests

Required verification examples:

- orthogonal dot product = 0
- basis-vector cross products match right-handed convention
- normalized non-zero vector has norm ≈ 1
- normalization of zero vector is rejected explicitly

---

## 4. `Matrix3`

Required behavior:

- identity
- construction
- matrix-vector multiplication
- matrix-matrix multiplication
- transpose
- determinant
- inverse when valid
- finite-value validation

Required verification:

- identity multiplication
- transpose properties
- known determinant
- inverse round trip
- singular inverse rejection
- rotation matrix orthonormality tests

---

## 5. `Quaternion`

Required behavior:

- scalar-first storage `[w,x,y,z]`
- identity quaternion
- norm
- normalization
- conjugate
- inverse for valid quaternion
- multiplication
- vector rotation
- conversion to/from rotation matrix
- controlled Euler conversion for telemetry/testing

Required verification:

- identity rotation
- 90-degree principal-axis rotations
- `q * q^-1 ≈ identity`
- quaternion norm after normalization
- quaternion/matrix rotation equivalence
- round-trip body/navigation vector rotations

---

## 6. Coordinate Transform Utilities

Must follow `COORDINATE_SYSTEMS.md`.

Required capabilities:

- body -> navigation vector transform
- navigation -> body vector transform
- rotation-matrix equivalents
- clearly named frame direction

No transform API may use ambiguous names such as:

```text
rotateVector()
```

when the source and destination frame are important.

Prefer names/contracts that make the direction explicit.

---

## 7. Testing

Phase 1 must include GoogleTest coverage for every public math operation.

Tests must include:

- exact/simple arithmetic cases
- tolerance-based floating-point cases
- invalid-input behavior
- frame/sign convention cases

---

## 8. Numerical Requirements

- `double` used by default
- no NaN/inf produced for valid inputs
- invalid normalizations/inversions fail explicitly
- no compiler warnings
- no hidden degree/radian ambiguity
- quaternion convention matches Phase 0 documentation

---

## 9. Build Requirements

Phase 1 must:

- build through CMake
- build from Visual Studio
- build from command line using CMake
- expose tests through CTest
- remain independent of visualization libraries

---

## 10. Phase 1 Exit Criteria

- [ ] `Vector3` implemented and tested.
- [ ] `Matrix3` implemented and tested.
- [ ] `Quaternion` implemented and tested.
- [ ] Coordinate transforms implemented and tested.
- [ ] Right-handed cross-product tests pass.
- [ ] NED/body frame sign tests pass.
- [ ] Quaternion body-to-navigation convention is verified.
- [ ] Singular/invalid math operations fail explicitly.
- [ ] GoogleTest suite passes.
- [ ] CTest runs the suite successfully.
- [ ] Build has zero meaningful warnings.
- [ ] CMake command-line build succeeds.
- [ ] Documentation/comments state units/frames where relevant.
