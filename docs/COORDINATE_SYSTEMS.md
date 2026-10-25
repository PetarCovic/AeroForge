# AeroForge Coordinate Systems

> **Documentation status (October 2026):** Coordinate and quaternion contracts are normative for the design: right-handed NED navigation, FRD body, body-to-navigation `q_NB`, scalar-first quaternion storage, and SI quantities.
> **Evidence boundary:** Design statements describe intended behavior; current implementation and measured results are established by source, tests, and archived benchmark reports, not by this specification alone.


## 1. Purpose

This document defines AeroForge's coordinate frames, axis directions, handedness, rotation signs, quaternion convention, and transformation notation.

These conventions are authoritative. Physics code should never infer or redefine them locally.

---

## 2. Global Convention

AeroForge uses **right-handed coordinate systems**.

The initial operational domain is local atmospheric flight over a flat Earth. The initial navigation/world frame therefore uses a local **North-East-Down (NED)** convention.

This choice matches common aerospace practice and gives AeroForge a clean path toward later navigation-frame work.

---

## 3. Initial Navigation Frame — `N`

The local navigation frame is denoted `N`.

Axes:

- `+X_N` = North
- `+Y_N` = East
- `+Z_N` = Down

Therefore:

```text
          North (+X_N)
               ^
               |
               |
               o ----> East (+Y_N)

Down (+Z_N) points into the page/ground direction.
```

Altitude is therefore the negative of the local down-position component when the origin is at ground level:

\[
h = -z_N
\]

The initial flat-Earth world frame and local navigation frame are treated as coincident.

Later versions may distinguish local navigation, ECEF, and ECI frames.

---

## 4. Body Frame — `B`

The body frame is fixed to the vehicle.

Axes:

- `+X_B` = forward through the vehicle nose
- `+Y_B` = vehicle right
- `+Z_B` = vehicle down

This is also right-handed.

For a vertically launched rocket, the vehicle's initial body orientation will be configured so `+X_B` points along the rocket longitudinal axis even though the local navigation frame uses `+X_N` for North.

The initial attitude is therefore not assumed to be identity merely because the vehicle is upright.

---

## 5. Positive Rotations

Positive rotations follow the **right-hand rule**.

With the body-axis convention above:

- positive roll = rotation about `+X_B`
- positive pitch = rotation about `+Y_B`
- positive yaw = rotation about `+Z_B`

Interpretation:

- positive roll tends toward right-wing/right-side-down rotation
- positive pitch raises the nose
- positive yaw turns the nose to the right

No subsystem may use an alternate sign convention without an explicit conversion at its boundary.

---

## 6. State-Vector Frame Conventions

The baseline rigid-body state uses:

- position expressed in navigation frame
- translational velocity expressed in navigation frame
- orientation represented by a body-to-navigation quaternion
- angular velocity expressed in body frame

Conceptually:

```text
position_N
velocity_N
q_NB
omega_BN_B
```

where:

- `q_NB` rotates a vector from body coordinates into navigation coordinates
- `omega_BN_B` is the angular velocity of the body relative to the navigation frame, expressed in body coordinates

---

## 7. Quaternion Convention

AeroForge uses unit quaternions in scalar-first form:

\[
q = [w, x, y, z]
\]

The authoritative orientation quaternion is:

```text
q_NB
```

meaning **body-to-navigation rotation**.

For a body-frame vector:

\[
\mathbf{v}_N = R_{NB}\mathbf{v}_B
\]

The inverse transformation is:

\[
\mathbf{v}_B = R_{BN}\mathbf{v}_N
\]

with:

\[
R_{BN} = R_{NB}^{T}
\]

and, for a unit quaternion:

\[
q_{BN} = q_{NB}^{*}
\]

where `*` denotes quaternion conjugation.

Quaternions must remain normalized within a documented numerical tolerance.

---

## 8. Rotation-Matrix Naming

A rotation matrix `R_AB` transforms coordinates from frame `B` into frame `A`.

Therefore:

\[
\mathbf{v}_A = R_{AB}\mathbf{v}_B
\]

Examples:

```text
R_NB : body -> navigation
R_BN : navigation -> body
```

This notation should be used consistently in code comments, documentation, tests, and derivations.

---

## 9. Force and Moment Frames

AeroForge uses the following baseline contracts:

### Gravity

Gravity acceleration is naturally produced in the navigation frame.

### Aerodynamics

Aerodynamic force and moment models should return results in the body frame unless a model explicitly documents another contract.

### Propulsion

Propulsion forces and moments should return results in the body frame.

### Actuator-Generated Loads

Actuator-generated forces and moments should return results in the body frame.

### Dynamics

The dynamics layer is responsible for transforming quantities into the frame required by each equation of motion.

Body-frame forces can be transformed into navigation coordinates for translational dynamics, while rotational moments are accumulated in the body frame.

---

## 10. Angle Representation

Angles are stored and computed in **radians** internally.

Human-readable output may use degrees, but degree/radian conversion must occur at a clearly defined boundary.

Euler angles are not the authoritative orientation state.

They may be derived from `q_NB` for:

- telemetry
- plots
- debugging
- visualization
- configuration convenience

---

## 11. Initial Vertical Rocket Orientation

Because the body `+X_B` axis points through the nose and the navigation `+Z_N` axis points downward, an upright rocket does not generally correspond to the identity quaternion.

Initial orientation must be explicitly constructed from the mission's launch direction.

This prevents hidden assumptions such as:

```text
identity quaternion = upright rocket
```

which would be incorrect under the selected NED/body-axis conventions.

---

## 12. Future Frames

Possible future frames include:

- ECEF — Earth-Centered, Earth-Fixed
- ECI — Earth-Centered Inertial
- local NED frames tied to geodetic position
- sensor-specific frames
- actuator-specific frames
- aerodynamic/wind frames

Any future frame must document:

- origin
- axis directions
- handedness
- motion relative to other frames
- transformation notation

---

## 13. Coordinate-System Rules

1. All frames are right-handed unless explicitly documented otherwise.
2. Frame information must be explicit at API boundaries.
3. A vector's frame must never be inferred solely from its variable type.
4. Rotation matrices use the `R_AB` convention: frame B to frame A.
5. The authoritative attitude quaternion is `q_NB`.
6. Position and translational velocity are expressed in navigation coordinates.
7. Angular velocity is expressed in body coordinates.
8. Aerodynamic and propulsion moments are expressed about the current center of mass unless otherwise documented.
9. Internal angles use radians.
10. Coordinate conversions should be centralized and tested.

---

## 14. Validation Requirements

Coordinate-transform tests must eventually verify:

- identity rotation
- 90-degree rotations about each principal axis
- quaternion normalization
- quaternion inverse/conjugate behavior
- `R_AB * R_BA = I`
- vector round-trip transformations
- Euler -> quaternion -> Euler consistency away from singular display cases
- body-to-navigation and navigation-to-body inverse consistency

---

## 15. Frame Conventions Acceptance Criteria

- [x] Right-handed coordinate systems selected.
- [x] Local NED selected as the initial navigation/world convention.
- [x] Body axes defined as X-forward, Y-right, Z-down.
- [x] Positive roll, pitch, and yaw follow the right-hand rule.
- [x] Position and velocity are navigation-frame quantities.
- [x] Angular velocity is a body-frame quantity.
- [x] Scalar-first quaternion layout selected.
- [x] `q_NB` selected as the authoritative body-to-navigation orientation.
- [x] Rotation-matrix naming convention defined.
- [x] Force/moment frame contracts established.
- [x] Internal angles remain radians.
- [x] Future frame expansion remains possible.
