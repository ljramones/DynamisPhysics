# DynamisPhysics → Vectrix Migration (Phase 1), and Verified Defects

Status: **planned**. Vectrix 2.4.0 adds the kernels this note maps to.

The ownership rule is `/Users/larrymitchell/Dynamis/docs/architecture/math-gpu-ownership.md`, section 2: vectrix
supplies pure functions and never steps. Bodies, solvers, vehicle models and the backend integrations stay here.

The defects in section 2 were verified on 2026-11-22 against `main` at `6a439d1`, which uses vectrix 2.0.0,
DynamisCollision 1.1.1, ode4j 0.5.3 and jolt-jni 3.5.2. The checks used reproductions run on both backends; the
Jolt native library loads on macOS arm64.

## 1. Replace with vectrix

| Physics code | Vectrix 2.4.0 |
|---|---|
| `ode4j/vehicle/Ode4jPacejkaTire` and `jolt/vehicle/PacejkaTireModel` (identical; the Jolt force function is dead code) | `physics.Pacejkaf.magicFormula`; `slipRatio` (signed, correct in reverse; defect 3); `slipAngle` (`atan2`) |
| `Ode4jVehicleSystem.java:163`: `SpringDamperf.suspensionForce` | `SpringDamperf.suspensionForceFromCompression(compression, compressionRate, k, c)` (defect 1) |
| `Ode4jSpringController` (F = −kx − cv), default damping `2√k` (also `JoltConstraintFactory.java:75`) | `SpringDamperf.force`; `criticalDamping(k, m)` = `2√(km)` (defect 6); `dampingForRatio`, `stiffnessForFrequency`; `dampedStep` for an exact, unconditionally stable step |
| `ode4j/body/Ode4jInertiaComputer`: hulls and meshes treated as spheres | `physics.MassPropertiesf.polyhedron`: exact volume, center of mass and inertia (defect 5) |
| `Ode4jCompoundMassProperties` and `JoltCompoundMassProperties` (duplicates) | `MassPropertiesf.combine`, `parallelAxis`, `rotate`, `*Volume` |
| Ragdoll PD control | already uses `PdControllersf` |

Conversions between vectrix and ODE/Jolt types stay here, since the backend types are third-party. There are about
139 conversion lines in 24 files. Reusing scratch instances, instead of allocating on each call, makes them
allocation-free.

**Careful with `vectrix.core.Math.clamp`.** Its argument order is `clamp(min, max, value)`, inherited from JOML. The
JDK's `java.lang.Math.clamp` uses `clamp(value, min, max)`. Defect A below comes from this.

## 2. Verified defects

**Notation.** "Run" means a reproduction executed against the real classes. The expected values are derived from
the physics, or from the other backend where that one is correct. Line numbers refer to `6a439d1`.

### Vehicles

| # | Defect | Evidence | Status |
|---|---|---|---|
| 1 | **ODE suspension force is inverted.** `Ode4jVehicleSystem.java:163-169` passes the actual compression and compression rate to `SpringDamperf.suspensionForce`, which computes `k (rest − length) − c · lengthRate`. Line 177 sets `normalLoad = suspForce`, so the tyre load goes negative, and a negative load flips `brakeForce` (:199), so braking pushes along the velocity. | Repository defaults (travel 0.2, k 25000, c 2000). Compression 0 → 5000 N (should be 0). 0.2 → 0 N (should be 5000). 0.2 at rate 1 → −2000 N (should be 7000). 0.2 at rate 3 → −6000 N (should be 11000). **Full simulation** (Ode4jVehicleTest car, 1200 kg, dropped from y = 1): the chassis ends on its belly at y ≈ 0.39 and is still jittering after 10 s. The correct rest height is y ≈ 0.832 (2943 N per wheel → compression 0.118 m). Defects A and B make it worse. | **Confirmed** |
| 3 | **Slip.** (a) Jolt computes slip from the unsigned `horizontalSpeed` (`JoltVehicleSystem.java:159, 283-285`) while the wheel spin ω is signed. (b) ODE estimates ω as `\|v\|/r + 5·throttle` (`Ode4jVehicleSystem.java:273-284`), so the service brake never reduces ω and braking slip is never modelled. | (a) Run: rolling backward at 4.90 m/s gives slip −1.999 on all wheels, where the true slip is ≈ 0. This also fires spurious `WheelSlipEvent`s. (b) Run: slip 3.5 below 0.5 m/s at full throttle, and 0 when coasting. Slip is negative only with the handbrake (−1 above 0.5 m/s). | **(a) Confirmed. (b) Confirmed, narrower than first suspected:** the handbrake does produce negative slip. |
| — | *Surface friction applied twice* (`:183-187` divides slip by μ and scales the peak by μ) | That is the Pacejka similarity method, F_μ(s) = μ·F₀(s/μ). Run: the initial slope is 18.998 at μ = 1 and 18.990 at μ = 0.5, so the stiffness is unchanged while the peak drops, as intended. Caveat: the reported `slipRatio` and the slip-event threshold use the scaled value (:212, :219). | **Not a defect** |
| D | **Jolt reads the suspension spring as a frequency.** `JoltVehicleAdapter.java:44-45` calls `setStiffness` and `setDamping` without switching the spring mode, which defaults to frequency and damping. ODE treats the same fields as N/m and N·s/m. | Read-back: `mode=FrequencyAndDamping frequency=25000.0 damping=2000.0`, i.e. 25 kHz with a damping ratio of 2000. Run: the default car never settles; it hops between y = 1.0 and 1.48 for 4 s with its wheels mostly off the ground. | **Confirmed** |
| F | **ODE traction and braking are counted twice.** The Pacejka force `fx` (`Ode4jVehicleSystem.java:188-193`) is applied on top of the separate drive force `wheelTorque / r` (:241-243). With the handbrake, a negative `fx` is added on top of `brakeForce` (:197-198). | Code | **Confirmed by reading** |
| G | **Jolt steering torque depends on the timestep.** `JoltVehicleSystem.java:274` multiplies the yaw torque by `dt`; ODE (:247) does not. | Code | **Confirmed by reading** |

### Bodies, forces and mass

| # | Defect | Evidence | Status |
|---|---|---|---|
| 2 | **ODE applies impulses as forces.** `Ode4jForceAccumulator.java:40`: `addForceAtPos(e.impulse(), …)`, flushed once per substep (`Ode4jStepLoop.java:115`). No caller compensates (`Ode4jPhysicsWorld.java:180-182` forwards the value unchanged), and the tests only assert `vx > 0`. | Run: 1 kg sphere, J = 10, dt = 1/60 → vx = 0.16667 (= J·dt/m); it should be 10. With 4 substeps, vx = 0.041667. Jolt applies a true impulse (`JoltBodyRegistry.java:126`), but see defect C. | **Confirmed** |
| C | **Jolt ignores the configured mass.** `JoltBodyRegistry.java:55-68` never sets the mass or overrides the mass properties, so Jolt uses its default density of 1000. This breaks parity between the backends in every mass-dependent behaviour. | A "1 kg" sphere of radius 0.5 behaves as 524 kg. The "1200 kg" `simpleCar` chassis is 7040 kg. Run: J = 10 gives vx = 0.01908 = 10/524. | **Confirmed** |
| 5 | **ODE hull and mesh inertia is about 1.8× too large.** `Ode4jInertiaComputer.java:49-51` treats them as a sphere of radius half the AABB diagonal (:65-72). The compound path does the same for non-primitive children (`Ode4jCompoundMassProperties.java:126-129`). | Run, unit cube with m = 6: `CONVEX_HULL` and `TRIANGLE_MESH` both give diag(1.8, 1.8, 1.8). `BOX` gives 1.0, and the exact value is m/6 = 1.0. | **Confirmed** |
| 6 | **The default damping `2√k` is critical only for unit mass.** `Ode4jSpringController.java:56` and `JoltConstraintFactory.java:75`, with k defaulting to 40. | The damping ratio is ζ = c / (2√(km)) = 1/√m: 1 at 1 kg, 0.316 at 10 kg, 0.1 at 100 kg. On Jolt the tests' "1 kg" `box(0.25³)` is really 125 kg (defect C), which gives ζ ≈ 0.09. | **Confirmed** |
| 7 | **Capsule and cylinder axes disagree.** ODE geoms are Z-aligned (`Ode4jShapeAdapter.java:33-40`, with no offset rotation). ODE single-body inertia is Y-aligned (`Ode4jInertiaComputer.java:36-43`, via vectrix `InertiaTensorf`, documented as +Y). ODE compound inertia is Z-aligned (`Ode4jCompoundMassProperties.java:120, 124`, axis 3). Collision, Jolt and the wireframes (`ShapeWireframes.java:70-88`) are Y-aligned, and nothing reconciles them. | Runs: an ODE capsule is hit at y = 0.2 by a downward ray and at z = 0.7 along −Z; on Jolt it is the opposite (0.7 and 0.2). ODE single-body inertia diag(0.1455, 0.0192, 0.1455) is Y-aligned, while ODE compound inertia diag(0.1455, 0.1455, 0.0192) is Z-aligned. Net: a single ODE capsule body has its geometry along Z and its inertia along Y. | **Confirmed** |
| 8 | *Scaled bone matrices feed non-unit quaternions to the ragdolls* (`Ode4jRagdollSystem.java:179`, `JoltRagdollSystem.java:179`, `getUnnormalizedRotation`) | vectrix normalises each column before converting. Run: rotateY(0.7) with uniform scale 0.01, and with non-uniform scale (2, 0.5, 3), both give \|q\| = 1.000000. Only a **mirrored** matrix fails: scale (−1, 1, 1) gives \|q\| = 0.7071, a non-rotation. | **Not reproduced**, except for mirrored bone matrices |
| H | **Ragdoll bones missing from the pose snap to the origin.** At alpha ≤ 0.001 a missing bone gets the identity matrix (`AnimisPose.java:15`), so its body is moved to the origin every step (both ragdoll systems at :181-182). | Code | **Confirmed by reading** |

### Shapes, queries and shared helpers

| # | Defect | Evidence | Status |
|---|---|---|---|
| 4 | **Jolt planes are mirrored.** `JoltShapeAdapter.java:56` passes Collision's `d` (plane `n·x = d`, also ODE's convention) into Jolt's `Plane`, which is `n·x + c = 0`. The fix is `-p.distance()`. | Run with plane (0, 1, 0), d = 1: on ODE a sphere of radius 0.5 rests at y = 1.4999 (correct 1.5). On Jolt it rests at y = −0.5, and a ray hits the plane at y = −1.0 (correct 1.0). With d = 0 the backends agree, which is why `planeY()` hides it. | **Confirmed** |
| 9 | **Jolt raycasts return a hard-coded normal.** `JoltRaycastExecutor.java:48`: `new Vector3f(0f, 1f, 0f)`. The character slope test (`JoltCharacterController.java:244`) therefore always passes, and the fallback (:182-191) marks the character grounded on any slope. | Run: a ray onto a box tilted 60° about Z returns (0, 1, 0); the correct normal is (−0.866, 0.5, 0). | **Confirmed** |
| A | **`vectrix.core.Math.clamp` is called with the wrong argument order.** Its signature is `clamp(min, max, value)` = `max(min, min(max, value))`. Physics calls `clamp(value, min, max)`, which drops the upper bound. Affected (16 sites, the only ones in the engine): `Ode4jVehicleSystem.java:76, 77, 159, 184, 199`; `JoltVehicleSystem.java:102, 106, 110, 191, 192, 254`; `Ode4jEngineModel.java:14, 21`; both ragdoll systems at :143; `Ode4jTransmissionModel.java:42` (harmless today). | Run: `clamp(5f, 0f, 1f)` = 5. `clamp(0.55f, 0f, 0.2f)` = 0.55. `clamp(4f, -3f, 3f)` = 4. `clamp(9000f, 900f, 6500f)` = 9000. As a result, throttle, brake, steering and blend alpha accept values above 1, suspension compression is not capped at the travel, and engine RPM is unbounded above. Present since commit c60c53d. | **Confirmed** |
| B | **ODE wheel rays hit their own chassis.** `simpleCar`'s wheel attachment points (y = −0.4) sit on the bottom face of `box(…, 0.4, …)`. `Ode4jRaycastExecutor` neither excludes the chassis nor honours `layerMask`. | Run: a ray from (−0.75, 0.6, 1.4) with the chassis at y = 1 returns fraction 0.0 on the chassis. Starting 5 cm lower, it hits the ground at fraction 0.846. With defects A and 1, the compression becomes 0.55 and F = −8750 N, which pulls the car into the ground. | **Confirmed** |
| E | **Raycasts ignore `layerMask` on both backends.** Jolt `castRay` is called without filters (`JoltRaycastExecutor.java:40`), and Jolt's `raycastAll` returns at most one hit (:60-61). The character's `collidesWith` mask (`JoltCharacterController.java:237`) has no effect. | Code | **Confirmed by reading** |

Defects 1-9 come from the 2026-11-22 survey; A-H were found during verification.

## 3. Fixing order

1. **Defect A**, the clamp order, at all 16 sites. It is a mechanical change that bounds throttle, brake, steering,
   blend and compression again.
2. **Defects 1 and B**, the ODE vehicle. Switch to `suspensionForceFromCompression`, and exclude the chassis from
   its own wheel rays. Then add a regression test: the default car must settle at y ≈ 0.832.
3. **Defects 2 and C**, backend parity. Make ODE impulses real impulses, and make Jolt honour the configured mass.
   Then add a test that asserts the same Δv = J/m on both backends.
4. **Defects 4, 9 and D**, the Jolt plane sign, ray normal and spring mode. Each is a one-line fix.
5. **Defects 5, 7 and 6.** Use `MassPropertiesf.polyhedron` and `combine`, pick one capsule/cylinder axis, and use
   `criticalDamping(k, m)`.
6. **The rest:** defects 3, F, G, H and E.

Every fix should come with the reproduction from section 2 as a regression test.

## 4. Migration order

1. Fix the defects in section 3, steps 1-4 (none depend on vectrix).
2. Bump vectrix to 2.4.0 once it is released.
3. Switch to `suspensionForceFromCompression`, `Pacejkaf` and `criticalDamping(k, m)`. Delete the dead Jolt Pacejka
   copy.
4. Use `MassPropertiesf.polyhedron` for hulls and meshes, and `combine` for compounds. Delete both compound
   duplicates.
5. Decide one capsule/cylinder axis convention (defect 7).
