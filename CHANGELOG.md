# Changelog

## 0.3.0 (2026-02-25)

### Added
- Physics tuning control plane with profile-driven resolution:
  - `DETERMINISTIC`, `DEFAULT`, `PERF`
  - explicit precedence: system properties > config fields > profile defaults > backend fallbacks.
- Jolt allocator/thread tuning integration:
  - allocator mode selection (`SAFE`/`MALLOC`/`IMPL`)
  - allocator size clamping and safeguards
  - PERF auto-thread policy with safety cap.
- Extended benchmark realism suite:
  - constraint-heavy, compound-heavy, and mixed-scene benchmarks
  - profile compare workflow for `DEFAULT` vs `PERF`.
- Targeted, gated integration parity suite:
  - `MixedSceneParityTest`
  - `TerrainMeshParityTest`
  - `SnapshotMidMotionParityTest`
  - gate script: `scripts/gate-parity-integration.sh`.
- Benchmark baseline update files:
  - `bench-baselines/0.3.0.json`
  - `bench-baselines/raw/0.3.0-jmh.json`

### Changed
- README updated with tuning profiles, integration parity gate usage, and benchmark profile compare guidance.
- Jolt profile application tests and resolver coverage expanded for clamp/precedence behavior.

### Notes
- Integration parity tests are gated by `-Dphysics.parity.integration=true` to keep default parity gates fast.
- Deterministic guarantees remain backend-local in deterministic mode; cross-backend checks are behavioural.

## 0.2.0 (2026-02-25)

### Added
- Compound runtime parity across ODE4J and Jolt, including COM-aligned offsets and compound dynamics parity tests.
- 6DOF spring fidelity improvements with deterministic pre-solve spring control and cross-backend parity coverage.
- Mechanical constraints realism:
  - gear ratio coupling
  - rack-pinion linear/angular coupling
  - pulley rope-length coupling
  with deterministic parity tests.
- MeshForge workflow integration:
  - PackedMesh extraction helpers (POSITION + UINT16/UINT32 index support)
  - PackedMesh -> `CollisionShape` builders for triangle-mesh, convex hull, and fitted capsule
  - Jolt runtime support for convex-hull and triangle-mesh shape creation.
- Terrain material tag mapping hook via `TerrainMaterialTags`.
- Heavy mixed-scene snapshot parity test coverage.
- Benchmark baseline update files:
  - `bench-baselines/0.2.0.json`
  - `bench-baselines/raw/0.2.0-jmh.json`

### Changed
- Extended Jolt parity gate with mesh workflow coverage.
- Maintained deterministic and heavy-scene snapshot gate compatibility after mesh and constraint fidelity changes.

### Notes
- Cross-backend snapshot checks remain behavioural parity (not byte-identical between engines).
- ODE4J and Jolt determinism guarantees remain backend-local in deterministic mode.

## 0.1.0 (2026-02-25)

### Added
- Typed collision shapes in DynamisCollision 1.1.1 integration path.
- ODE4J backend feature completion:
  - shape adapter for all `ShapeType` variants
  - constraint system + builders
  - vehicle system + events
  - character controller + landing hints/events
  - ragdoll blend system + get-up hints
  - snapshot/restore + determinism gate
  - debug renderer via DynamisGPU
- Jolt backend feature completion:
  - core world/body/raycast/events parity
  - character parity (`CharacterVirtual` path)
  - vehicle parity (`VehicleConstraint` path)
  - ragdoll parity + blend/hints
  - snapshot/restore parity
  - deterministic mode (1 thread) and multithread mode (configurable)
  - optional stress gate `JoltParallelStabilityTest`
- Step 13 benchmark module (`dynamisphysics-bench`) with JMH suites:
  - `RigidBodyStepBenchmark`
  - `RaycastBenchmark`
  - `VehicleBenchmark`

### Changed
- Documented gate commands for ODE4J, Jolt parity, and Jolt stress runs.
- Added helper scripts:
  - `scripts/gate-ode4j.sh`
  - `scripts/gate-jolt.sh`
  - `scripts/gate-jolt-stress.sh`
  - `scripts/bench.sh`

### Notes
- Jolt JNI execution remains JDK 25-only in this repository.
- Snapshot parity is behavioural across backends; byte identity is backend-local.
- Deterministic mode defaults to single-thread Jolt stepping; override with `-Djolt.threads=<N>`.
